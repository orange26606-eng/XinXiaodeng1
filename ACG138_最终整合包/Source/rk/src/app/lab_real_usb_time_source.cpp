/* USB 雷达采集与时间戳输出。 */
#include "sl_lidar.h"
#include "scan_core.h"
#include "lab_real_time.h"
#include <cmath>
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
static volatile sig_atomic_t running=1;
static FILE *timing_csv=nullptr;static int csv_failed=0;
static void stop(int) {running=0;}
static uint64_t now_us() {return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void pause_ms(unsigned ms) {for(unsigned n=0;n<ms && running;n+=50)std::this_thread::sleep_for(std::chrono::milliseconds(50));}
struct Cleanup {
    sl::ILidarDriver *driver;
    bool motor=false;
    ~Cleanup() {if(motor){driver->stop(1000);std::this_thread::sleep_for(std::chrono::milliseconds(200));driver->setMotorSpeed(0);}driver->disconnect();}
};
static bool report(const char *stage,sl_result result) {
    if(SL_IS_OK(result))return true;
    std::fprintf(stderr,"USB_FAIL stage=%s code=0x%08x\n",stage,unsigned(result));return false;
}
static int session(const char *device,int socket,const sockaddr_in &address,const sockaddr_in &timing_address,uint32_t token) {
    auto channel_result=sl::createSerialPortChannel(device,460800);
    if(!channel_result)return 1;
    std::unique_ptr<sl::IChannel> channel(*channel_result);
    auto driver_result=sl::createLidarDriver();if(!driver_result)return 1;
    std::unique_ptr<sl::ILidarDriver> driver(*driver_result);Cleanup cleanup{driver.get()};
    if(!report("open",driver->connect(channel.get())))return 1;
    sl_lidar_response_device_info_t info{};
    if(!report("device_info",driver->getDeviceInfo(info)))return 1;
    std::printf("USB_DEVICE model=%u firmware=%u.%u serial=",info.model,info.firmware_version>>8,info.firmware_version&255);
    for(auto c:info.serialnum)std::printf("%02X",unsigned(c));std::puts("");
    sl_lidar_response_device_health_t health{};
    if(!report("health",driver->getHealth(health)))return 1;
    std::printf("USB_HEALTH status=%u code=%u\n",health.status,health.error_code);
    if(health.status==SL_LIDAR_STATUS_ERROR || !running)return 1;
    cleanup.motor=true;if(!report("start_motor",driver->setMotorSpeed()))return 1;
    sl::LidarScanMode mode{};if(!report("start_scan",driver->startScan(false,true,0,&mode)))return 1;
    std::printf("USB_SCANNING mode=%.64s session=%u destination=%s:%u\n",mode.scan_mode,token,inet_ntoa(address.sin_addr),ntohs(address.sin_port));
    sl_lidar_response_measurement_node_hq_t raw[SCAN_MAX_POINTS];
    auto frame=std::unique_ptr<scan_frame>(new scan_frame{});
    frame->source=1;frame->flags=1;frame->session=token;
    unsigned failures=0;uint64_t previous=0,last_log=0;unsigned long long send_failures=0;
    while(running) {
        size_t count=SCAN_MAX_POINTS;sl_u64 first_us=0;sl_result result=driver->grabScanDataHqWithTimeStamp(raw,count,first_us,1000);
        if(SL_IS_FAIL(result)) {++failures;report("grab",result);if(failures>=5)return 1;continue;}
        failures=0;if(!count || count>=SCAN_MAX_POINTS)continue;
        frame->count=(uint16_t)count;frame->timestamp_us=now_us();++frame->sequence;
        frame->period_us=previous && frame->timestamp_us-previous<=1000000?(uint32_t)(frame->timestamp_us-previous):0;
        previous=frame->timestamp_us;
        /* 保留LS01原主机获取结束时刻语义，新伴随记录按会话、序号、点数和准确原时间戳关联。
 */
        if(std::isfinite(mode.us_per_sample)&&mode.us_per_sample>0&&mode.us_per_sample<=100000) {
            uint32_t sample_ns=(uint32_t)std::llround(mode.us_per_sample*1000.0);
            uint64_t duration=(uint64_t)(count-1)*sample_ns/1000;
            if(first_us<=UINT64_MAX-duration) {
                lab_real_scan_time timing{};timing.session=token;timing.sequence=frame->sequence;
                timing.count=(uint32_t)count;timing.sample_period_ns=sample_ns;timing.model=info.model;timing.firmware=info.firmware_version;
                timing.legacy_end_us=timing.fetched_us=frame->timestamp_us;timing.first_us=first_us;timing.last_us=first_us+duration;
                uint8_t bytes[LAB_SCAN_TIME_BYTES];lab_status status=lab_real_scan_time_encode(bytes,&timing);
                if(status==LAB_OK) {
                    if(timing_csv&&std::fprintf(timing_csv,"HARDWARE,%u,%u,%u,%u,%llu,%llu,%llu,%zu,%u\n",token,frame->sequence,unsigned(info.model),unsigned(info.firmware_version),(unsigned long long)timing.first_us,(unsigned long long)timing.last_us,(unsigned long long)timing.fetched_us,count,sample_ns)<0){csv_failed=1;running=0;}
                    if(sendto(socket,bytes,sizeof(bytes),MSG_DONTWAIT,(const sockaddr*)&timing_address,sizeof(timing_address))!=(ssize_t)sizeof(bytes))++send_failures;
                }else std::fprintf(stderr,"SDK_TIME_REJECTED seq=%u first=%llu last=%llu fetched=%llu; no false timestamp emitted\n",frame->sequence,(unsigned long long)first_us,(unsigned long long)timing.last_us,(unsigned long long)timing.fetched_us);
            }
        }
        for(size_t i=0;i<count;++i)frame->points[i]={raw[i].angle_z_q14,raw[i].dist_mm_q2,
            (uint8_t)(raw[i].quality>>SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT),raw[i].flag};
        for(unsigned offset=0;offset<count;offset+=SCAN_CHUNK_POINTS) {
            uint8_t packet[SCAN_PACKET_MAX];uint16_t n=(uint16_t)std::min<size_t>(SCAN_CHUNK_POINTS,count-offset);
            size_t length=scan_encode(packet,sizeof(packet),frame.get(),(uint16_t)offset,n);
            if(sendto(socket,packet,length,MSG_DONTWAIT,(const sockaddr*)&address,sizeof(address))!=(ssize_t)length)++send_failures;
        }
        if(frame->timestamp_us-last_log>=5000000) {
            std::printf("USB_SCAN sequence=%u points=%u send_failures=%llu\n",frame->sequence,frame->count,send_failures);last_log=frame->timestamp_us;
        }
    }
    return 0;
}
int main(int argc,char **argv) {
    const char *device="/dev/ttyUSB0",*host="127.0.0.1",*csv_path=nullptr;unsigned port=6210,time_port=6211;bool execute=false;
    for(int i=1;i<argc;++i) {
        if(!std::strcmp(argv[i],"--run")){execute=true;continue;}
        if(!std::strcmp(argv[i],"--info")){std::puts("RK C1 SDK first-point timestamp + ST01 companion; CLOCK_MONOTONIC; no vehicle commands; board untested");return 0;}
        if(!std::strcmp(argv[i],"--help")) {std::puts("lidar-usb-time-source [--run] [--device /dev/ttyUSB0] [--host 127.0.0.1] [--port 6210] [--time-port 6211] [--csv NEW.csv]\nContinuous C1 acquisition, 460800 8N1. No vehicle commands.");return 0;}
        if(i+1>=argc)return 1;
        if(!std::strcmp(argv[i],"--device"))device=argv[++i];
        else if(!std::strcmp(argv[i],"--csv"))csv_path=argv[++i];
        else if(!std::strcmp(argv[i],"--host"))host=argv[++i];
        else if(!std::strcmp(argv[i],"--time-port")){char *end=nullptr;unsigned long n=std::strtoul(argv[++i],&end,10);if(!*argv[i]||*end||n<1024||n>65535)return 1;time_port=(unsigned)n;}
        else if(!std::strcmp(argv[i],"--port")){char *end=nullptr;unsigned long n=std::strtoul(argv[++i],&end,10);if(!*argv[i]||*end||n<1024||n>65535)return 1;port=(unsigned)n;}
        else return 1;
    }
    if(port==time_port)return 1;
    if(!execute){std::printf("PREFLIGHT ONLY device=%s LS01=%u ST01=%u; no device/socket/lock opened\n",device,port,time_port);return 0;}
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::setvbuf(stdout,nullptr,_IOLBF,0);
    int lock=open("/tmp/acg138-c1-usb.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)<0){std::perror("Another C1 source or lock error");return 2;}
    int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);if(fd<0){std::perror("socket");return 2;}
    sockaddr_in destination{};destination.sin_family=AF_INET;destination.sin_port=htons((uint16_t)port);
    if(inet_pton(AF_INET,host,&destination.sin_addr)!=1){std::fprintf(stderr,"Invalid IPv4 host\n");return 1;}
    sockaddr_in timing_destination=destination;timing_destination.sin_port=htons((uint16_t)time_port);
    if(csv_path){timing_csv=std::fopen(csv_path,"wbx");if(!timing_csv){close(fd);close(lock);return 2;}std::fputs("source,session,sequence,model,firmware,first_us,last_us,fetched_us,count,sample_period_ns\n",timing_csv);}
    unsigned reconnect=0;
    while(running) {
        uint32_t token=0;if(getrandom(&token,sizeof(token),0)!=(ssize_t)sizeof(token))token=(uint32_t)(now_us()^(uint64_t)getpid()^++reconnect);
        if(!token)token=1;
        session(device,fd,destination,timing_destination,token);
        if(running){std::fprintf(stderr,"USB_RECONNECT waiting=2000ms device=%s\n",device);pause_ms(2000);}
    }
    if(timing_csv&&std::fclose(timing_csv))csv_failed=1;close(fd);close(lock);return csv_failed?2:0;
}
