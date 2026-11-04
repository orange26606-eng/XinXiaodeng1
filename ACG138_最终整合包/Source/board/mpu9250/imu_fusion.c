#include "imu_fusion.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define PI 3.14159265358979323846
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static int16_t i16(const uint8_t *p){uint16_t u=(uint16_t)p[0]|(uint16_t)p[1]<<8;int16_t v;memcpy(&v,&u,2);return v;}
static double wrap(double x){return atan2(sin(x),cos(x));}
static double norm3(const double *a){return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);}
uint32_t i9_crc32(const uint8_t *p,size_t n){uint32_t c=~0u;
 for(size_t i=0;i<n;++i){c^=p[i];for(int j=0;j<8;++j)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
int i9_decode(struct i9_packet *out,const uint8_t *p,size_t n){
 if(!out||!p||!((n==96&&p[4]==1)||(n==100&&p[4]==2))||memcmp(p,"I9E1",4)||(p[5]&~31u)||p[83]||
    (p[81]&~15u)||u32(p+n-4)!=i9_crc32(p,n-4))return I9_BAD_PACKET;
 struct i9_packet v={0};v.flags=p[5];v.whoami=p[6];v.mag_id=p[7];v.sequence=u32(p+8);
 v.snapshot_us=u32(p+12);v.ag_us=u32(p+16);v.mag_us=u32(p+20);
 for(int i=0;i<4;++i){uint32_t u=u32(p+24+i*4);memcpy(&v.counts[i],&u,4);}
 for(int i=0;i<3;++i){v.accel[i]=i16(p+40+i*2);v.gyro[i]=i16(p+48+i*2);v.mag[i]=i16(p+54+i*2);v.asa[i]=p[60+i];}
 v.temperature=i16(p+46);v.i2c_error=p[63];v.errors=u32(p+64);v.overflows=u32(p+68);v.tx_drops=u32(p+72);
 v.encoder_errors=u32(p+76);v.ab=p[80];v.initialized=p[81];v.phase=p[82];v.ag_sequence=u32(p+84);v.mag_sequence=u32(p+88);
 v.reset_epoch=n==100?u32(p+92):0;*out=v;return I9_OK;
}
void i9_default_config(struct i9_config *c){if(!c)return;memset(c,0,sizeof(*c));
 c->wheel_mm=67.5;c->track_mm=290;c->counts_per_rev=1320;c->gyro_weight=.9;c->cal_seconds=2;c->left_mask=5;
 for(int i=0;i<4;++i)c->signs[i]=(i&1)?-1:1;for(int i=0;i<3;++i)c->axes[i]=i+1;
}
int i9_config_valid(const struct i9_config *c){
 if(!c||!isfinite(c->wheel_mm)||c->wheel_mm<20||c->wheel_mm>500||!isfinite(c->track_mm)||c->track_mm<50||c->track_mm>1000||
    !isfinite(c->counts_per_rev)||c->counts_per_rev<1||c->counts_per_rev>1000000||
    !isfinite(c->gyro_weight)||c->gyro_weight<0||c->gyro_weight>1||
    !isfinite(c->cal_seconds)||c->cal_seconds<1||c->cal_seconds>30||c->left_mask<1||c->left_mask>14)return 0;
 unsigned left=0,used=0;int permutation[3],sign=1;
 for(int i=0;i<4;++i){if(c->signs[i]!=1&&c->signs[i]!=-1)return 0;left+=(c->left_mask>>i)&1;}
 if(left!=2)return 0;
 for(int i=0;i<3;++i){int a=abs(c->axes[i]);if(a<1||a>3||(used&(1u<<a)))return 0;used|=1u<<a;permutation[i]=a;if(c->axes[i]<0)sign=-sign;}
 for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)if(permutation[i]>permutation[j])sign=-sign;
 return sign==1;
}
int i9_init(struct i9_state *s,const struct i9_config *c){if(!s||!i9_config_valid(c))return -1;
 memset(s,0,sizeof(*s));s->config=*c;s->q[0]=1;s->quality=I9_WAIT_IMU;return 0;}
static void map_axes(const struct i9_config *c,const double in[3],double out[3]){
 for(int i=0;i<3;++i)out[i]=(c->axes[i]<0?-1:1)*in[abs(c->axes[i])-1];
}
static void euler(struct i9_state *s){const double *q=s->q;
 s->roll=atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
 double p=2*(q[0]*q[2]-q[3]*q[1]);s->pitch=asin(fmax(-1,fmin(1,p)));
 s->yaw=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}
static void tilt_init(struct i9_state *s){
 double r=atan2(s->accel[1],s->accel[2]),p=atan2(-s->accel[0],hypot(s->accel[1],s->accel[2]));
 double cr=cos(r*.5),sr=sin(r*.5),cp=cos(p*.5),sp=sin(p*.5);
 s->q[0]=cr*cp;s->q[1]=sr*cp;s->q[2]=cr*sp;s->q[3]=-sr*sp;euler(s);
}
static void attitude(struct i9_state *s,double dt){
 double w[3];for(int i=0;i<3;++i)w[i]=s->gyro[i]-s->bias[i];
 double *q=s->q,n=norm3(s->accel);
 if(fabs(n-1)<.15){
  double a[3]={s->accel[0]/n,s->accel[1]/n,s->accel[2]/n};
  double v[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
  w[0]+=2*(a[1]*v[2]-a[2]*v[1]);w[1]+=2*(a[2]*v[0]-a[0]*v[2]);w[2]+=2*(a[0]*v[1]-a[1]*v[0]);
 }else ++s->accel_rejected;
 double speed=norm3(w),half=speed*dt*.5,factor=speed>1e-12?sin(half)/speed:dt*.5;
 double d[4]={cos(half),w[0]*factor,w[1]*factor,w[2]*factor};
 double next[4]={q[0]*d[0]-q[1]*d[1]-q[2]*d[2]-q[3]*d[3],q[0]*d[1]+q[1]*d[0]+q[2]*d[3]-q[3]*d[2],
  q[0]*d[2]-q[1]*d[3]+q[2]*d[0]+q[3]*d[1],q[0]*d[3]+q[1]*d[2]-q[2]*d[1]+q[3]*d[0]};
 double length=sqrt(next[0]*next[0]+next[1]*next[1]+next[2]*next[2]+next[3]*next[3]);
 for(int i=0;i<4;++i)q[i]=next[i]/length;euler(s);
}
static void reset_cal(struct i9_state *s){s->cal_count=0;memset(s->cal_sum,0,sizeof(s->cal_sum));memset(s->cal_squared,0,sizeof(s->cal_squared));++s->cal_resets;}
int i9_push(struct i9_state *s,const struct i9_packet *p){
 if(!s||!p)return I9_BAD_PACKET;
 uint32_t us=0,seq_delta=0;double wheel[4]={0};int moving=0,gap=0;
 if(s->have_packet){
  seq_delta=p->sequence-s->previous_sequence;us=p->snapshot_us-s->previous_us;
  if(!seq_delta||seq_delta>=0x80000000u||!us||us>=0x80000000u){++s->rejected;return I9_OLD_PACKET;}
  s->missing+=seq_delta-1;s->time_us+=us;gap=us>250000;
  for(int i=0;i<4;++i){uint32_t a,b;memcpy(&a,&p->counts[i],4);memcpy(&b,&s->previous_counts[i],4);uint32_t d=a-b;int32_t delta;memcpy(&delta,&d,4);
   wheel[i]=delta*s->config.signs[i]*PI*s->config.wheel_mm/(s->config.counts_per_rev*1000);
   if(delta)moving=1;if(fabs(wheel[i])>us*1e-6*1.5+.005)gap=1;
  }
 }else s->time_us=p->snapshot_us;
 s->have_packet=1;s->previous_sequence=p->sequence;s->previous_us=p->snapshot_us;s->packet=*p;
 memcpy(s->previous_counts,p->counts,sizeof(s->previous_counts));++s->accepted;
 double a[3],w[3],m[3];
 for(int i=0;i<3;++i){a[i]=p->accel[i]/8192.0;w[i]=p->gyro[i]/65.5*(PI/180);m[i]=p->mag[i]*.15*((p->asa[i]-128)/256.0+1);}
 map_axes(&s->config,a,s->accel);map_axes(&s->config,w,s->gyro);
 double mag_in_accel_axes[3]={m[1],m[0],-m[2]};map_axes(&s->config,mag_in_accel_axes,s->mag);
 s->temperature_c=p->temperature/333.87+21;
 if(us&&!gap&&p->initialized==15){
  for(int i=0;i<4;++i)s->rpm_counts[i]+=wheel[i]/(PI*s->config.wheel_mm*.001);
  s->rpm_seconds+=us*1e-6;
  if(s->rpm_seconds>=.1){for(int i=0;i<4;++i){s->rpm[i]=60*s->rpm_counts[i]/s->rpm_seconds;s->rpm_counts[i]=0;}s->rpm_seconds=0;}
 }
 uint32_t age=p->snapshot_us-p->ag_us;
 int ag_good=(p->flags&(I9_AG_VALID|I9_MPU_READY))==(I9_AG_VALID|I9_MPU_READY)&&p->whoami==0x71&&age<=50000&&age<=s->time_us;
 if(ag_good&&(!s->have_ag||p->ag_sequence!=s->previous_ag_sequence)){
  uint64_t time=s->time_us-age;double dt=s->have_ag?(time-s->last_ag_time_us)*1e-6:0;
  if(s->have_ag&&(time<=s->last_ag_time_us||dt>.1)){s->have_yaw_reference=0;gap=1;dt=0;}
  s->last_ag_time_us=time;s->previous_ag_sequence=p->ag_sequence;s->have_ag=1;
  if(!s->bias_ready){
   s->quality=I9_CALIBRATING;
   if(moving||fabs(norm3(s->accel)-1)>.2||norm3(s->gyro)>15*PI/180)reset_cal(s);
   else{
    for(int i=0;i<3;++i){s->cal_sum[i]+=s->gyro[i];s->cal_squared[i]+=s->gyro[i]*s->gyro[i];}++s->cal_count;
    if(s->cal_count>=(unsigned)(s->config.cal_seconds*200)){
     int stable=1;for(int i=0;i<3;++i){double b=s->cal_sum[i]/s->cal_count;
      if(s->cal_squared[i]/s->cal_count-b*b>pow(.5*PI/180,2))stable=0;s->bias[i]=b;}
     if(stable){s->bias_ready=1;tilt_init(s);s->heading=0;s->have_yaw_reference=0;}else reset_cal(s);
    }
   }
  }else if(dt>0)attitude(s,dt);
 }
 if(!s->bias_ready){if(!ag_good)s->quality=I9_WAIT_IMU;return gap?I9_GAP:I9_OK;}
 if(p->initialized!=15){s->quality=I9_WAIT_ENCODER;s->have_yaw_reference=0;return I9_OK;}
 if(gap){++s->gaps;s->quality=I9_DATA_GAP;s->have_yaw_reference=0;return I9_GAP;}
 double dl=0,dr=0;for(int i=0;i<4;++i){if((s->config.left_mask>>i)&1)dl+=wheel[i]*.5;else dr+=wheel[i]*.5;}
 double dt=us*1e-6;
 double ds=(dl+dr)*.5,dtheta=(dr-dl)/(s->config.track_mm*.001);
 if(ag_good){
  /* 用平面外推将最新姿态对齐到编码器快照。该近似使用陀螺仪 Z 轴，要求传感器接近水平安装，不属于完整惯性导航。 */
  double reference=wrap(s->yaw+(s->gyro[2]-s->bias[2])*age*1e-6);
  if(s->have_yaw_reference){double dg=wrap(reference-s->previous_yaw_reference);
   if(fabs(dg)<=3*dt+.02)dtheta=s->config.gyro_weight*dg+(1-s->config.gyro_weight)*dtheta;}
  s->previous_yaw_reference=reference;s->have_yaw_reference=1;s->quality=I9_FUSED;
 }else{s->quality=I9_WHEELS_ONLY;s->have_yaw_reference=0;}
 double half=dtheta*.5,factor=fabs(half)>1e-10?sin(half)/half:1;
 s->x+=ds*factor*cos(s->heading+half);s->y+=ds*factor*sin(s->heading+half);
 s->heading=wrap(s->heading+dtheta);s->distance+=fabs(ds);return I9_OK;
}
const char *i9_quality_name(int q){static const char *const names[]={"WAIT_IMU","KEEP_STILL_CALIBRATING","FUSED_UNCALIBRATED","WHEELS_ONLY","WAIT_ENCODER","DATA_GAP"};
 return q>=0&&q<6?names[q]:"UNKNOWN";}
