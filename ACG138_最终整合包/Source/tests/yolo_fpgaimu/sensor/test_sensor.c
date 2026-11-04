#include "lab_real_mpusvc.h"
#include "lab_codec.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rtl_fixtures.h"
static unsigned assertions;
#define CHECK(x) do{++assertions;if(!(x)){fprintf(stderr,"FAIL line=%d %s\n",__LINE__,#x);abort();}}while(0)
static void u32(uint8_t*p,uint32_t v){for(unsigned k=0;k<4;k++)p[k]=(uint8_t)(v>>(8*k));}
static void i16(uint8_t*p,int v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void q24(uint8_t*p,double v){int32_t n=(int32_t)llround(v*16777216.);uint32_t raw;memcpy(&raw,&n,4);u32(p,raw);}
static void raw_packet(uint8_t*p,uint32_t seq,uint32_t stamp,uint32_t epoch,int count,unsigned mask){
 memset(p,0,100);memcpy(p,"I9E1",4);p[4]=2;p[5]=I9_AG_VALID|I9_MPU_READY|I9_ENCODERS_READY;p[6]=0x71;p[7]=0x48;
 u32(p+8,seq);u32(p+12,stamp);u32(p+16,stamp-1000);u32(p+20,stamp-1000);
 for(unsigned k=0;k<4;k++)u32(p+24+4*k,(uint32_t)(k&1?-count:count));i16(p+44,8192);
 p[60]=p[61]=p[62]=128;p[81]=(uint8_t)mask;u32(p+84,seq);u32(p+88,seq);u32(p+92,epoch);u32(p+96,lab_crc32(p,96));
}
static void f6_packet(uint8_t*p,uint32_t seq,uint32_t source,uint32_t epoch,double roll,double pitch,double yaw){
 memset(p,0,100);memcpy(p,"F6A1",4);p[4]=1;p[5]=3;p[6]=0x71;p[7]=2;u32(p+8,seq);u32(p+12,source);u32(p+16,source+300);u32(p+20,seq);
 double cr=cos(roll/2),sr=sin(roll/2),cp=cos(pitch/2),sp=sin(pitch/2),cy=cos(yaw/2),sy=sin(yaw/2);double q[4]={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
 for(unsigned k=0;k<4;k++)q24(p+24+4*k,q[k]);q24(p+40,roll);q24(p+44,pitch);q24(p+48,yaw);u32(p+76,300);p[80]=144;p[81]=1;u32(p+88,source);u32(p+92,epoch);u32(p+96,lab_crc32(p,96));
}
static mpusvc_core*open_core(unsigned mode){struct odom_config o;odom_default_config(&o);o.calibrated=1;struct i9_config i;i9_default_config(&i);mpusvc_core*s=NULL;CHECK(mpusvc_open(&s,LAB_REAL_HARDWARE,123,1,&o,&i)==LAB_OK);CHECK(mpusvc_attitude_mode(s,mode)==LAB_OK);return s;}
static void codecs(void){
 uint8_t p[100];f6_attitude a;f6_packet(p,4,0xfffffff0u,3,.2,-.3,.4);CHECK(!f6_decode(&a,p,100));CHECK(a.age_us==300&&a.generated_us==284&&a.epoch==3);CHECK(fabs(a.rpy[0]-.2)<1e-7);CHECK(f6_decode(&a,p,99)<0);p[42]^=1;CHECK(f6_decode(&a,p,100)==-2);p[42]^=1;
 p[83]=1;u32(p+96,lab_crc32(p,96));CHECK(f6_decode(&a,p,100)<0);p[83]=0;p[7]=5;u32(p+96,lab_crc32(p,96));CHECK(f6_decode(&a,p,100)<0);
 p[5]=0;CHECK(f6_decode(&a,p,100)==-2);u32(p+96,lab_crc32(p,96));CHECK(!f6_decode(&a,p,100));
 p[5]=3;p[7]=2;memset(p+24,0,16);u32(p+96,lab_crc32(p,96));CHECK(f6_decode(&a,p,100)<0);
}
static void cache_and_mapping(void){
 int axes[3]={1,2,3};f6_selector s;f6_selector_init(&s,F6_MODE_AUTO,axes);uint8_t p[100],f[100];raw_packet(p,17,101000,2,0,15);struct i9_packet raw;CHECK(!i9_decode(&raw,p,100));f6_packet(f,17,100000,2,.1,.2,.3);CHECK(f6_selector_push(&s,f,100,200000)==1);CHECK(f6_selector_find(&s,&raw,200001)!=NULL);
 raw.ag_us++;CHECK(!f6_selector_find(&s,&raw,200001));raw.ag_us--;raw.ag_sequence++;CHECK(!f6_selector_find(&s,&raw,200001));raw.ag_sequence--;raw.reset_epoch++;CHECK(!f6_selector_find(&s,&raw,200001));raw.reset_epoch--;
 CHECK(f6_selector_push(&s,f,100,299000)==0);CHECK(s.duplicates==1);CHECK(!f6_selector_find(&s,&raw,250001));CHECK(f6_selector_find(&s,&raw,249999)!=NULL);raw.flags=0;CHECK(!f6_selector_find(&s,&raw,200001));raw.flags=5;
 struct i9_state rk={0};rk.q[0]=1;rk.bias_ready=1;rk.cal_count=400;struct mpu_attitude_selection selected={0};s.raw_received_us=200000;s.raw_wait_us=1;CHECK(f6_selector_hook(&s,&raw,&rk,&selected));CHECK(selected.source==F6_SOURCE_FPGA);CHECK(fabs(selected.rpy[0]-.1)<1e-6&&fabs(selected.rpy[1]-.2)<1e-6&&fabs(selected.rpy[2]-.3)<1e-6);
 f6_packet(f,18,105000,2,0,0,0);u32(f+52,655);u32(f+56,(uint32_t)-131);u32(f+60,65);u32(f+96,lab_crc32(f,96));raw.ag_sequence=18;raw.ag_us=105000;raw.snapshot_us=106000;CHECK(f6_selector_push(&s,f,100,301000)==1);s.raw_received_us=301001;s.raw_wait_us=0;CHECK(f6_selector_hook(&s,&raw,&rk,&selected));CHECK(fabs(selected.bias[0]-.17453292519943295)<1e-12);CHECK(fabs(selected.bias[1]+.03490658503988659)<1e-12);
 f6_packet(f,17,100000,2,.1,.2,.3);raw.ag_sequence=17;raw.ag_us=100000;raw.snapshot_us=101000;
 /* 实际带符号轴映射，仅切换数据源时对齐偏航相对零点。 */
 int rotated[3]={-2,1,3};f6_selector_init(&s,F6_MODE_FPGA,rotated);CHECK(f6_selector_push(&s,f,100,200000)==1);s.raw_received_us=200001;CHECK(f6_selector_hook(&s,&raw,&rk,&selected));CHECK(selected.source==F6_SOURCE_FPGA);CHECK(fabs(selected.rpy[2]-(.3-1.5707963267948966))<.08);
 /* 覆盖64项缓存、乱序独立样本及坏帧年龄处理。 */
 f6_selector_init(&s,F6_MODE_AUTO,axes);for(unsigned k=0;k<1000;k++){f6_packet(f,k,10000+5000*k,2,0,0,0);CHECK(f6_selector_push(&s,f,100,300000+k)==1);}CHECK(s.accepted==1000&&s.cursor==1000);raw.ag_sequence=17;raw.ag_us=100000;CHECK(!f6_selector_find(&s,&raw,301000));
 f6_packet(f,997,10000+5000*997,2,0,0,0);CHECK(f6_selector_push(&s,f,100,301001)==0);f6_packet(f,1,10000,7,0,0,0);CHECK(f6_selector_push(&s,f,100,301002)==1);CHECK(s.last.epoch==7);f[70]^=1;CHECK(f6_selector_push(&s,f,100,400000)<0);CHECK(s.last_received_us==301002);
}
static void pairing_pose(void){
 mpusvc_core*s=open_core(F6_MODE_AUTO);uint8_t p[100],f[100];mpusvc_view v;uint64_t rx=1000000;
 raw_packet(p,1,50000,0,0,15);CHECK(mpusvc_submit_raw(s,p,100,rx,rx)==LAB_AGAIN);mpusvc_snapshot(s,rx,&v);CHECK(!v.accepted);
 f6_packet(f,1,49000,0,.1,.2,.25);CHECK(mpusvc_push_attitude(s,f,100,rx+3000)==LAB_OK);CHECK(mpusvc_flush_pending(s,rx+3000)==LAB_OK);mpusvc_snapshot(s,rx+4000,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA&&v.bias_ready);CHECK(v.raw_received_us==rx&&v.ag_received_us==rx&&v.raw_wait_us==3000);CHECK(v.ag_sequence==1&&v.attitude_ag_sequence==1);CHECK(fabs(v.rpy[2]-.25)<1e-6);CHECK(v.counts_mask==15);
 uint8_t snapshot[768];CHECK(mpusvc_encode(snapshot,&v)==LAB_OK);FILE*saved=fopen("matched-v1.mpusnap","wb");CHECK(saved&&fwrite(snapshot,1,768,saved)==768);fclose(saved);
 struct odom_shared shared={.magic=ODOM_MAGIC,.version=1,.bytes=sizeof(shared),.sequence=2,.state=v.odometry};saved=fopen("matched-legacy.shm","wb");CHECK(saved&&fwrite(&shared,1,sizeof shared,saved)==sizeof shared);fclose(saved);
 mpusvc_view copy;CHECK(mpusvc_decode(&copy,snapshot,768)==LAB_OK);CHECK(!memcmp(copy.raw,v.raw,100));CHECK(fabs(copy.rpy[2]-.25)<1e-6&&copy.raw_received_us==rx);CHECK(mpusvc_encode_timed(snapshot,&v)==LAB_OK);saved=fopen("matched-v2.mpusnap","wb");CHECK(saved&&fwrite(snapshot,1,768,saved)==768);fclose(saved);CHECK(mpusvc_decode(&copy,snapshot,768)==LAB_OK);CHECK(copy.pose_imu_age_us==v.pose_imu_age_us);
 double previous=v.rpy[2];
 /* 姿态真实配对先于RAW到达，编码器平移沿用原里程计。 */
 for(unsigned k=2;k<=160;k++){uint32_t stamp=50000+k*5000;rx+=5000;raw_packet(p,k,stamp,0,(int)(k/4),15);f6_packet(f,k,stamp-1000,0,0,0,.25+(k-1)*.001);CHECK(mpusvc_push_attitude(s,f,100,rx)==LAB_OK);CHECK(mpusvc_submit_raw(s,p,100,rx+1000,rx+1000)==LAB_OK);mpusvc_snapshot(s,rx+1000,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA);CHECK(fabs(v.rpy[2]-previous)<.002);previous=v.rpy[2];}
 CHECK(v.have_pose&&v.odometry.total_distance>.002&&v.odometry.accepted>20);CHECK(v.odometry.session==1);
 /* 姿态缺失仅保持12ms，再显式使用新鲜RK解算回退，航向不跳变。 */
 rx+=20000;raw_packet(p,161,860000,0,41,15);CHECK(mpusvc_submit_raw(s,p,100,rx,rx)==LAB_AGAIN);CHECK(mpusvc_flush_pending(s,rx+11999)==LAB_AGAIN);CHECK(mpusvc_flush_pending(s,rx+12000)==LAB_OK);mpusvc_snapshot(s,rx+12000,&v);CHECK(v.attitude_source==F6_SOURCE_RK);CHECK(fabs(v.rpy[2]-previous)<1e-5);CHECK(v.raw_received_us==rx);CHECK(strstr(v.attitude_reason,"FALLBACK")!=NULL);
 rx+=20000;raw_packet(p,162,865000,0,41,15);f6_packet(f,162,864000,0,0,0,-2);CHECK(mpusvc_push_attitude(s,f,100,rx)==LAB_OK);CHECK(mpusvc_submit_raw(s,p,100,rx+1,rx+1)==LAB_OK);mpusvc_snapshot(s,rx+1,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA);CHECK(fabs(v.rpy[2]-previous)<1e-5);CHECK(v.attitude_switches>=2);
 /* 传感器重启后不能继续附带旧epoch。 */
 rx+=10000;raw_packet(p,163,870000,1,0,0);CHECK(mpusvc_submit_raw(s,p,100,rx,rx)==LAB_AGAIN);CHECK(mpusvc_flush_pending(s,rx+12000)==LAB_OK);mpusvc_snapshot(s,rx+12000,&v);CHECK(v.epoch==1&&v.session==2&&v.counts_mask==0);CHECK(v.attitude_source==F6_SOURCE_RK&&!v.bias_ready);CHECK(v.odometry.total_distance==0);mpusvc_close(s);
}
static void modes_and_wrap(void){
 uint8_t p[100],f[100];mpusvc_view v;mpusvc_core*s=open_core(F6_MODE_FPGA);raw_packet(p,1,10000,0,0,0);CHECK(mpusvc_submit_raw(s,p,100,1000000,1000000)==LAB_AGAIN);CHECK(mpusvc_flush_pending(s,1012000)==LAB_OK);mpusvc_snapshot(s,1012000,&v);CHECK(v.attitude_source==F6_SOURCE_UNAVAILABLE&&!v.bias_ready&&v.counts_mask==0);CHECK(!v.odometry.imu_used);mpusvc_close(s);
 s=open_core(F6_MODE_AUTO);raw_packet(p,1,50000,0,0,0);f6_packet(f,1,49000,0,0,0,0);CHECK(mpusvc_push_attitude(s,f,100,1000000)==LAB_OK);CHECK(mpusvc_submit_raw(s,p,100,1000001,1000001)==LAB_OK);mpusvc_snapshot(s,1000002,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA&&v.bias_ready&&v.counts_mask==0&&v.fusion_quality==I9_WAIT_ENCODER);CHECK(!v.odometry.imu_used&&v.odometry.total_distance==0);mpusvc_close(s);
 s=open_core(F6_MODE_AUTO);uint64_t rx=1000000;uint32_t stamp=0xfffff000u;
 for(unsigned k=1;k<40;k++){raw_packet(p,k,stamp,4,(int)k,15);f6_packet(f,k,stamp-1000,4,0,0,.01*k);CHECK(mpusvc_push_attitude(s,f,100,rx)==LAB_OK);CHECK(mpusvc_submit_raw(s,p,100,rx+1,rx+1)==LAB_OK);mpusvc_snapshot(s,rx+1,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA);CHECK(v.raw_fpga_us>=0xfffff000u);rx+=5000;stamp+=5000;}CHECK(v.raw_fpga_us>UINT32_MAX);mpusvc_close(s);
 /* 待处理队列满时不增长，每行原始编码器按顺序处理。 */
 s=open_core(F6_MODE_AUTO);for(unsigned k=1;k<=200;k++){raw_packet(p,k,100000+k*5000,0,(int)k,15);lab_status code=mpusvc_submit_raw(s,p,100,1000000+k,1000000+k);CHECK(code==LAB_OK||code==LAB_AGAIN);}CHECK(mpusvc_flush_pending(s,2000000)==LAB_OK);mpusvc_snapshot(s,2000000,&v);CHECK(v.accepted==200&&v.raw_pending_overflow>0);mpusvc_close(s);
}
static void rk_regression(void){
 struct odom_config o;odom_default_config(&o);struct i9_config c;i9_default_config(&c);struct mpu_adapter reference;CHECK(!mpu_adapter_init(&reference,&o,&c));mpusvc_core*s=NULL;CHECK(mpusvc_open(&s,LAB_REAL_TEST_FIXTURE,1,1,&o,&c)==LAB_OK);
 uint8_t p[100];mpusvc_view v;for(unsigned k=1;k<=5000;k++){uint64_t rx=1000000+(uint64_t)k*5000;int count=k<=500?0:(int)(k-500)/4;raw_packet(p,k,100000+k*5000,0,count,15);CHECK(mpu_adapter_push(&reference,p,100,rx/1000)>=0);CHECK(mpusvc_push(s,p,100,rx)==LAB_OK);mpusvc_snapshot(s,rx,&v);CHECK(!memcmp(v.q,reference.fusion.q,sizeof v.q));CHECK(v.rpy[0]==reference.fusion.roll&&v.rpy[1]==reference.fusion.pitch&&v.rpy[2]==reference.fusion.yaw);CHECK(v.odometry.accepted==reference.odom.state.accepted);CHECK(v.odometry.total_distance==reference.odom.state.total_distance&&v.odometry.heading==reference.odom.state.heading&&v.odometry.x==reference.odom.state.x&&v.odometry.y==reference.odom.state.y);CHECK(v.attitude_source==F6_SOURCE_RK);}
 CHECK(v.bias_ready&&v.odometry.total_distance>.1);mpusvc_close(s);
}

static void axes_and_rtl(void){
 int perms[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};unsigned maps=0;
 for(unsigned p=0;p<6;p++)for(unsigned bits=0;bits<8;bits++){
  struct i9_config config;i9_default_config(&config);for(unsigned k=0;k<3;k++)config.axes[k]=(bits&(1u<<k))?-perms[p][k]:perms[p][k];if(!i9_config_valid(&config))continue;++maps;
  for(unsigned row=0;row<20;row++){double roll=-.7+.06*row,pitch=-.5+.04*row,yaw=-2+.1*row;f6_selector sel;f6_selector_init(&sel,F6_MODE_AUTO,config.axes);uint8_t r[100],a[100];raw_packet(r,row+1,100000+5000*row,8,0,15);f6_packet(a,row+1,99000+5000*row,8,roll,pitch,yaw);CHECK(f6_selector_push(&sel,a,100,1000000+5000*row)==1);struct i9_packet raw;CHECK(!i9_decode(&raw,r,100));sel.raw_received_us=1000001+5000*row;struct i9_state rk={0};rk.q[0]=1;struct mpu_attitude_selection out;CHECK(f6_selector_hook(&sel,&raw,&rk,&out));CHECK(out.source==F6_SOURCE_FPGA);
   double native[3]={-sin(pitch),sin(roll)*cos(pitch),cos(roll)*cos(pitch)},body[3]={-sin(out.rpy[1]),sin(out.rpy[0])*cos(out.rpy[1]),cos(out.rpy[0])*cos(out.rpy[1])};
   for(unsigned k=0;k<3;k++){int axis=config.axes[k];double expected=(axis<0?-1:1)*native[(axis<0?-axis:axis)-1];CHECK(fabs(expected-body[k])<2e-6);}
   double norm=0;for(unsigned k=0;k<4;k++)norm+=out.q[k]*out.q[k];CHECK(fabs(norm-1)<1e-12);
  }
 }CHECK(maps==24);
 unsigned paired=0,masked=0;
 for(unsigned k=0;k<sizeof(rtl_attitude)/100;k++){f6_attitude a;CHECK(!f6_decode(&a,rtl_attitude[k],100));if((a.flags&3)!=3)continue;
  for(unsigned j=0;j<sizeof(rtl_raw)/100;j++){struct i9_packet raw;CHECK(!i9_decode(&raw,rtl_raw[j],100));if(raw.reset_epoch!=a.epoch||raw.ag_sequence!=a.sequence||raw.ag_us!=a.source_us)continue;
   mpusvc_core*c=open_core(F6_MODE_AUTO);CHECK(mpusvc_push_attitude(c,rtl_attitude[k],100,1000000)==LAB_OK);CHECK(mpusvc_submit_raw(c,rtl_raw[j],100,1000001,1000001)==LAB_OK);mpusvc_view v;mpusvc_snapshot(c,1000002,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA);CHECK(v.epoch==a.epoch&&v.ag_sequence==a.sequence);CHECK(fabs(v.rpy[0]-a.rpy[0])<1e-6&&fabs(v.rpy[1]-a.rpy[1])<1e-6&&fabs(v.rpy[2]-a.rpy[2])<1e-6);CHECK(v.counts_mask==raw.initialized);if(!raw.initialized){CHECK(!v.odometry.imu_used);++masked;}mpusvc_close(c);++paired;break;
  }
 }CHECK(paired>0);printf("PASS full-vehicle RTL payload C interop paired=%u encoder_unready_preserved=%u signed_axes=%u\n",paired,masked,maps);
 /* 计数器和源时刻同时回绕不视为重启。 */
 mpusvc_core*c=open_core(F6_MODE_AUTO);uint32_t seq=0xfffffff0u,stamp=0xfffff000u;uint64_t rx=1000000;uint8_t raw[100],a[100];
 for(unsigned k=0;k<40;k++,seq++,stamp+=5000,rx+=5000){raw_packet(raw,seq,stamp,0,(int)k,15);f6_packet(a,seq,stamp-1000,0,0,0,.01*k);CHECK(mpusvc_push_attitude(c,a,100,rx)==LAB_OK);CHECK(mpusvc_submit_raw(c,raw,100,rx+1,rx+1)==LAB_OK);mpusvc_view v;mpusvc_snapshot(c,rx+1,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA&&v.session==1&&v.restarts==0);}mpusvc_close(c);
}


static void export_one(const char*label,const char*provenance,mpusvc_core*c,const uint8_t*raw,const uint8_t*f6,uint64_t now){
 mpusvc_view v;mpusvc_snapshot(c,now,&v);CHECK(v.attitude_source==F6_SOURCE_FPGA&&v.bias_ready);
 /* 实际发布循环而非核心snapshot()持有export_sequence，测试中确定性递增产生sample01和sample02，源及接收时间不变。
 */
 v.export_sequence=!strcmp(label,"sample02")?2:1;
 char name[256];uint8_t b[768];FILE*out;
 for(unsigned timed=0;timed<2;timed++){CHECK((timed?mpusvc_encode_timed(b,&v):mpusvc_encode(b,&v))==LAB_OK);snprintf(name,sizeof name,"fixtures/%s-%s.mpusnap",label,timed?"timed":"v1");out=fopen(name,"wb");CHECK(out&&fwrite(b,1,768,out)==768);fclose(out);}
 struct odom_shared m={.magic=ODOM_MAGIC,.version=1,.bytes=sizeof(m),.sequence=2,.state=v.odometry};snprintf(name,sizeof name,"fixtures/%s-legacy.shm",label);out=fopen(name,"wb");CHECK(out&&fwrite(&m,1,sizeof m,out)==sizeof m);fclose(out);
 snprintf(name,sizeof name,"fixtures/%s.raw.bin",label);out=fopen(name,"wb");CHECK(out&&fwrite(raw,1,100,out)==100);fclose(out);snprintf(name,sizeof name,"fixtures/%s.f6.bin",label);out=fopen(name,"wb");CHECK(out&&fwrite(f6,1,100,out)==100);fclose(out);
 snprintf(name,sizeof name,"fixtures/%s.json",label);out=fopen(name,"wb");CHECK(out);fprintf(out,"{\"provenance\":\"%s\",\"instance\":\"%llu\",\"export_sequence\":\"%llu\",\"snapshot_flags\":%u,\"publisher\":\"test emulates actual POSIX ++export_sequence bookkeeping\",\"raw_epoch\":%u,\"session\":%u,\"raw_sequence\":%u,\"ag_sequence\":%u,\"raw_fpga_us\":\"%llu\",\"raw_received_us\":\"%llu\",\"ag_fpga_us\":\"%llu\",\"ag_received_us\":\"%llu\",\"pose_sequence\":%u,\"pose_fpga_us\":\"%llu\",\"pose_received_us\":\"%llu\",\"pose_imu_age_us\":\"%llu\",\"published_us\":\"%llu\",\"rpy_rad\":[%.12g,%.12g,%.12g],\"q\":[%.12g,%.12g,%.12g,%.12g],\"pose_xy_heading_distance\":[%.12g,%.12g,%.12g,%.12g],\"counts_mask\":%u,\"bias_ready\":%u,\"cal_samples\":%u,\"attitude_source\":\"FPGA6\",\"config_crc\":%u,\"parameter_version\":\"%llu\",\"snapshot_bytes\":768,\"legacy_bytes\":4224}\n",provenance,(unsigned long long)v.instance,(unsigned long long)v.export_sequence,v.flags,v.epoch,v.session,(unsigned)(raw[8]|(uint32_t)raw[9]<<8|(uint32_t)raw[10]<<16|(uint32_t)raw[11]<<24),v.ag_sequence,(unsigned long long)v.raw_fpga_us,(unsigned long long)v.raw_received_us,(unsigned long long)v.ag_fpga_us,(unsigned long long)v.ag_received_us,v.pose_sequence,(unsigned long long)v.pose_fpga_us,(unsigned long long)v.pose_received_us,(unsigned long long)v.pose_imu_age_us,(unsigned long long)v.published_us,v.rpy[0],v.rpy[1],v.rpy[2],v.q[0],v.q[1],v.q[2],v.q[3],v.odometry.x,v.odometry.y,v.odometry.heading,v.odometry.total_distance,v.counts_mask,v.bias_ready,v.cal_samples,v.config_crc,(unsigned long long)v.parameter_version);fclose(out);
}
static void export_fixtures(void){
 mpusvc_core*c=open_core(F6_MODE_AUTO);uint8_t raw[100],a[100];
 for(unsigned k=1;k<=9;k++){uint32_t stamp=1000000+5000*k;uint64_t rx=2000000+5000*k;raw_packet(raw,k,stamp,0,(int)k,15);f6_packet(a,k,stamp-1000,0,.1,.2,.25+.001*(k-1));CHECK(mpusvc_push_attitude(c,a,100,rx-200)==LAB_OK);CHECK(mpusvc_submit_raw(c,raw,100,rx,rx)==LAB_OK);if(k==5)export_one("sample01","synthetic production5ms cadence passed through actual sensor core",c,raw,a,rx+100);if(k==9)export_one("sample02","synthetic production5ms cadence passed through actual sensor core",c,raw,a,rx+100);}
 mpusvc_close(c);
 for(unsigned k=0;k<sizeof(rtl_attitude)/100;k++){f6_attitude f;CHECK(!f6_decode(&f,rtl_attitude[k],100));if((f.flags&3)!=3)continue;for(unsigned j=0;j<sizeof(rtl_raw)/100;j++){struct i9_packet p;CHECK(!i9_decode(&p,rtl_raw[j],100));if(p.reset_epoch!=f.epoch||p.ag_sequence!=f.sequence||p.ag_us!=f.source_us)continue;c=open_core(F6_MODE_AUTO);CHECK(mpusvc_push_attitude(c,rtl_attitude[k],100,3000000)==LAB_OK);CHECK(mpusvc_submit_raw(c,rtl_raw[j],100,3000001,3000001)==LAB_OK);export_one("rtl-actual","unchanged full-vehicle RTL GMII payloads; simulated RK receipt time",c,rtl_raw[j],rtl_attitude[k],3000002);mpusvc_close(c);return;}}
 CHECK(0);
}

int main(void){export_fixtures();axes_and_rtl();codecs();cache_and_mapping();pairing_pose();modes_and_wrap();rk_regression();printf("PASS sensor FPGA exact-pair/source-switch/axis/CRC/reorder/wrap/encoder/ABI/RK regression assertions=%u no_device_io\n",assertions);return 0;}
