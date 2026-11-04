#include "mpu_adapter.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846
static void put32(uint8_t*p,uint32_t x){for(int i=0;i<4;i++)p[i]=(uint8_t)(x>>(8*i));}
int mpu_config_load(struct i9_config*c,const char*path){
 FILE*f=fopen(path,"r");if(!f)return -1;
 struct i9_config next=*c;char line[256],key[64],tail;double value;int bad=0;
 while(fgets(line,sizeof(line),f)){
  if(line[0]=='#'||line[0]=='\n'||line[0]=='\r')continue;
  if(sscanf(line," %63[^=]= %lf %c",key,&value,&tail)!=2||!isfinite(value)){bad=1;break;}
  if(!strcmp(key,"cal_seconds"))next.cal_seconds=value;
  else if(!strncmp(key,"axis_",5)&&strlen(key)==6&&key[5]>='x'&&key[5]<='z'){
   if(value!=trunc(value)||fabs(value)<1||fabs(value)>3){bad=1;break;}next.axes[key[5]-'x']=(int)value;
  }else{bad=1;break;}
 }
 if(ferror(f))bad=1;fclose(f);if(!i9_config_valid(&next))bad=1;
 if(bad)return -1;*c=next;return 0;
}
int mpu_adapter_init(struct mpu_adapter*a,const struct odom_config*o,const struct i9_config*c){
 if(!a||!o||!c||!i9_config_valid(c))return -1;
 memset(a,0,sizeof(*a));a->odom_config=*o;a->fusion_config=*c;
 a->fusion_config.left_mask=o->left_mask;for(int i=0;i<4;i++)a->fusion_config.signs[i]=o->signs[i];
 if(i9_init(&a->fusion,&a->fusion_config))return -1;
 odom_init(&a->odom,o);a->session=1;a->imu.state=IMU_WAITING;return 0;
}
int mpu_adapter_push_select(struct mpu_adapter*a,const uint8_t*data,size_t bytes,uint64_t now,mpu_attitude_selector selector,void*user,struct mpu_attitude_selection*selected){
 struct i9_packet p;
 if(bytes!=100||data[4]!=2||i9_decode(&p,data,bytes)){++a->packet_errors;a->imu.invalid_count=a->packet_errors;return -1;}
 int reboot=a->fusion.have_packet&&(int32_t)(p.sequence-a->fusion.previous_sequence)<0&&
  (int32_t)(p.snapshot_us-a->fusion.previous_us)<0&&p.snapshot_us<1000000&&now>a->received_ms+300;
 if(a->fusion.have_packet&&!reboot&&(int32_t)(p.sequence-a->fusion.previous_sequence)<=0){++a->packet_errors;++a->imu.order_count;a->imu.invalid_count=a->packet_errors;return -1;}
 if(a->have_epoch&&(p.reset_epoch!=a->epoch||reboot)){
  ++a->session;++a->restarts;
  i9_init(&a->fusion,&a->fusion_config);odom_init(&a->odom,&a->odom_config);
  memset(&a->imu,0,sizeof(a->imu));a->imu.state=IMU_WAITING;
 }
 a->epoch=p.reset_epoch;a->have_epoch=1;
 int result=i9_push(&a->fusion,&p);if(result<0){++a->packet_errors;return -1;}
 a->received_ms=now;struct i9_state*f=&a->fusion;
 struct mpu_attitude_selection local={0};local.valid=1;local.bias_ready=f->bias_ready;local.cal_samples=f->cal_count;local.source=1;
 memcpy(local.q,f->q,sizeof local.q);memcpy(local.bias,f->bias,sizeof local.bias);local.rpy[0]=f->roll;local.rpy[1]=f->pitch;local.rpy[2]=f->yaw;
 if(selector)selector(user,&p,f,&local);if(selected)*selected=local;
 uint32_t age=p.snapshot_us-p.ag_us;
 int good=local.bias_ready&&p.whoami==0x71&&age<=50000&&
  (p.flags&(I9_AG_VALID|I9_MPU_READY))==(I9_AG_VALID|I9_MPU_READY);
 struct imu_model*m=&a->imu;m->seen_packet=1;m->last_packet_ms=now;m->have_sequence=1;m->sequence=p.sequence;
 m->invalid_count=a->packet_errors;m->missing_count=f->missing;m->error_count=(uint64_t)p.errors+p.overflows;
 if(good){
  if(!m->have_angles||m->good_sequence!=p.ag_sequence){++m->good_count;m->good_sequence=p.ag_sequence;}
  m->have_angles=1;m->last_good_ms=now>=age/1000?now-age/1000:0;
  m->status=0;m->state=IMU_LIVE;
  m->degrees[0]=local.rpy[0]*180/PI;m->degrees[1]=local.rpy[1]*180/PI;m->degrees[2]=local.rpy[2]*180/PI;
 }else{
  m->state=local.bias_ready?IMU_DEVICE_ERROR:IMU_WAITING;
  m->status=local.bias_ready?(p.i2c_error>=1&&p.i2c_error<=4?p.i2c_error:5):0;
 }
 // Feed the existing per-wheel odometry and trace/UI implementation. OD01 is
 // synthesized in process only; no loopback packets or spoofed FPGA addresses.
 uint32_t ms=(uint32_t)(f->time_us/1000);
 if(a->odom.have_packet&&ms-a->odom.fpga_ms<20)return result;
 uint8_t wire[64]={0};memcpy(wire,"OD01",4);wire[4]=1;wire[6]=p.initialized;wire[7]=good?0:3;
 put32(wire+8,p.sequence);put32(wire+12,ms);put32(wire+16,a->session);
 for(int i=0;i<4;i++){uint32_t raw;memcpy(&raw,&p.counts[i],4);put32(wire+20+4*i,raw);}
 float yaw=(float)local.rpy[2];uint32_t bits;memcpy(&bits,&yaw,4);put32(wire+36,bits);
 put32(wire+40,good?age/1000:UINT32_MAX);put32(wire+44,p.ag_sequence);put32(wire+48,a->session);
 put32(wire+60,i9_crc32(wire,60));odom_push(&a->odom,wire,64,now);
 // Preserve a distinct origin after S7/reload, even though model was re-created.
 a->odom.state.session=a->session;
 return result;
}
int mpu_adapter_push(struct mpu_adapter*a,const uint8_t*p,size_t n,uint64_t now){return mpu_adapter_push_select(a,p,n,now,NULL,NULL,NULL);}
size_t mpu_adapter_json(const struct mpu_adapter*a,char*out,size_t bytes,uint64_t now){
 const struct i9_state*f=&a->fusion;const struct i9_packet*p=&f->packet;
 uint64_t age=now>=a->received_ms?now-a->received_ms:0;
 int n=snprintf(out,bytes,"{\"source\":\"MPU9250_RAW_RK_FUSION\",\"motor_output\":false,\"seen\":%s,\"age_ms\":%llu,\"bias_ready\":%s,\"cal_samples\":%u,\"whoami\":%u,\"mag_id\":%u,\"flags\":%u,\"last_error\":%u,\"i2c_errors\":%u,\"mag_overflows\":%u,\"tx_drops\":%u,\"encoder_errors\":%u,\"ab\":%u,\"epoch\":%u,\"restarts\":%llu,\"packet_errors\":%llu,\"sequence\":%u,\"quality\":\"%s\",\"rpy_deg\":[%.4f,%.4f,%.4f],\"rpm\":[%.3f,%.3f,%.3f,%.3f],\"mag_uT\":[%.3f,%.3f,%.3f],\"temperature_c\":%.2f}\n",
  f->have_packet?"true":"false",(unsigned long long)age,f->bias_ready?"true":"false",f->cal_count,
  p->whoami,p->mag_id,p->flags,p->i2c_error,p->errors,p->overflows,p->tx_drops,p->encoder_errors,p->ab,a->epoch,
  (unsigned long long)a->restarts,(unsigned long long)a->packet_errors,p->sequence,i9_quality_name(f->quality),
  f->roll*180/PI,f->pitch*180/PI,f->yaw*180/PI,f->rpm[0],f->rpm[1],f->rpm[2],f->rpm[3],f->mag[0],f->mag[1],f->mag[2],f->temperature_c);
 return n>0&&(size_t)n<bytes?(size_t)n:0;
}
