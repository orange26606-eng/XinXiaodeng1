#include "object_lidar_geometry.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846
int ol_config_load(const char*path,ol_config*c){
 if(!c)return 0;memset(c,0,sizeof(*c));c->image_width=960;c->image_height=1080;c->fx=c->fy=c->cx=c->cy=c->k1=c->k2=c->p1=c->p2=c->k3=NAN;
 c->frame_max_age_ms=500;c->scan_max_age_ms=800;c->pair_max_delta_ms=150;c->confirm_frames=3;c->cluster_gap_m=.18;c->min_cluster_points=3;c->min_pixel_span=8;c->min_bbox_fraction=.08;c->max_pose_uncertainty_m=.15;c->max_time_uncertainty_ms=50;c->temporal_gate_m=.2;c->bbox_margin_px=24;
 if(!path)return 0;FILE*f=fopen(path,"rb");if(!f)return 0;
 struct field{const char*n;size_t off;}fields[]={
#define F(n) {#n,offsetof(ol_config,n)}
 F(enabled),F(calibration_enabled),F(reference_eye),F(image_width),F(image_height),F(fx),F(fy),F(cx),F(cy),F(k1),F(k2),F(p1),F(p2),F(k3),F(camera_to_body_verified),F(camera_x_m),F(camera_y_m),F(camera_z_m),F(camera_roll_deg),F(camera_pitch_deg),F(camera_yaw_deg),F(camera_delay_ms),F(lidar_z_m),F(lidar_height_verified),F(frame_max_age_ms),F(scan_max_age_ms),F(pair_max_delta_ms),F(confirm_frames),F(cluster_gap_m),F(min_cluster_points),F(min_pixel_span),F(min_bbox_fraction),F(max_pose_uncertainty_m),F(max_time_uncertainty_ms),F(temporal_gate_m),F(bbox_margin_px)
#undef F
 };unsigned char seen[sizeof(fields)/sizeof(fields[0])];memset(seen,0,sizeof(seen));char line[256];int ok=1;unsigned lines=0;
 while(fgets(line,sizeof(line),f)){if(++lines>128||(!strchr(line,'\n')&&!feof(f))){ok=0;break;}char*p=line;while(*p==' '||*p=='\t')p++;if(!*p||*p=='#'||*p=='\r'||*p=='\n')continue;char key[96],extra;double d;if(sscanf(p,"%95[^=]=%lf %c",key,&d,&extra)!=2||!isfinite(d)){ok=0;break;}size_t z=strlen(key);while(z&&(key[z-1]==' '||key[z-1]=='\t'))key[--z]=0;unsigned i;for(i=0;i<sizeof(fields)/sizeof(fields[0]);i++)if(!strcmp(key,fields[i].n))break;if(i==sizeof(fields)/sizeof(fields[0])||seen[i]){ok=0;break;}seen[i]=1;*(double*)((char*)c+fields[i].off)=d;
 }if(ferror(f))ok=0;fclose(f);if(!ok)return 0;
 if(c->enabled!=0&&c->enabled!=1)return 0;if(!c->enabled)return 1;
 if(c->calibration_enabled!=1||c->camera_to_body_verified!=1||c->lidar_height_verified!=1||c->reference_eye<0||c->reference_eye>1||floor(c->reference_eye)!=c->reference_eye||c->image_width!=960||c->image_height!=1080)return 0;
 if(!isfinite(c->fx)||!isfinite(c->fy)||!isfinite(c->cx)||!isfinite(c->cy)||!isfinite(c->k1)||!isfinite(c->k2)||!isfinite(c->p1)||!isfinite(c->p2)||!isfinite(c->k3)||c->fx<10||c->fx>10000||c->fy<10||c->fy>10000||c->cx<0||c->cx>960||c->cy<0||c->cy>1080||fabs(c->k1)>5||fabs(c->k2)>5||fabs(c->k3)>5||fabs(c->p1)>1||fabs(c->p2)>1)return 0;
 if(fabs(c->camera_x_m)>2||fabs(c->camera_y_m)>2||fabs(c->camera_z_m)>2||fabs(c->lidar_z_m)>2||fabs(c->camera_roll_deg)>180||fabs(c->camera_pitch_deg)>180||fabs(c->camera_yaw_deg)>180||c->camera_delay_ms<0||c->camera_delay_ms>500)return 0;
 return c->frame_max_age_ms>=50&&c->frame_max_age_ms<=1000&&c->scan_max_age_ms>=50&&c->scan_max_age_ms<=1000&&c->pair_max_delta_ms>=1&&c->pair_max_delta_ms<=200&&c->confirm_frames>=3&&c->confirm_frames<=20&&floor(c->confirm_frames)==c->confirm_frames&&c->cluster_gap_m>=.03&&c->cluster_gap_m<=.4&&c->min_cluster_points>=3&&c->min_cluster_points<=32&&floor(c->min_cluster_points)==c->min_cluster_points&&c->min_pixel_span>=2&&c->min_pixel_span<=100&&c->min_bbox_fraction>=.03&&c->min_bbox_fraction<=.5&&c->max_pose_uncertainty_m>0&&c->max_pose_uncertainty_m<=.3&&c->max_time_uncertainty_ms>=1&&c->max_time_uncertainty_ms<=100&&c->temporal_gate_m>=.03&&c->temporal_gate_m<=.5&&c->bbox_margin_px>=4&&c->bbox_margin_px<=100;
}
static void rotation(const ol_config*c,double r[3][3]){double x=c->camera_roll_deg*PI/180,y=c->camera_pitch_deg*PI/180,z=c->camera_yaw_deg*PI/180;double a=cos(x),b=sin(x),d=cos(y),e=sin(y),f=cos(z),g=sin(z);r[0][0]=f*d;r[0][1]=f*e*b-g*a;r[0][2]=f*e*a+g*b;r[1][0]=g*d;r[1][1]=g*e*b+f*a;r[1][2]=g*e*a-f*b;r[2][0]=-e;r[2][1]=d*b;r[2][2]=d*a;}
int ol_project(const ol_config*c,const passage_config*p,const lab_pose*s,const lab_pose*i,double range,double angle,double*u,double*v,double*mx,double*my){
 double a=angle+p->mount_yaw_rad,bx=p->mount_x_m+range*cos(a),by=p->mount_y_m+range*sin(a),cs=cos(s->yaw_rad),ss=sin(s->yaw_rad);
 *mx=s->x_m+cs*bx-ss*by;*my=s->y_m+ss*bx+cs*by;double dx=*mx-i->x_m,dy=*my-i->y_m,ci=cos(i->yaw_rad),si=sin(i->yaw_rad);
 double b[3]={ci*dx+si*dy-c->camera_x_m,-si*dx+ci*dy-c->camera_y_m,c->lidar_z_m-c->camera_z_m},r[3][3],q[3]={0};rotation(c,r);for(int k=0;k<3;k++)for(int n=0;n<3;n++)q[k]+=r[n][k]*b[n];
 /* 先逆安装RzRyRx，再逆光学到车体的(z,-x,-y)轴映射。 */
 double z=q[0];if(!isfinite(z)||z<=.03)return 0;double x=-q[1]/z,y=-q[2]/z,r2=x*x+y*y;if(r2>16)return 0;double rad=1+c->k1*r2+c->k2*r2*r2+c->k3*r2*r2*r2;
 *u=c->fx*(x*rad+2*c->p1*x*y+c->p2*(r2+2*x*x))+c->cx;*v=c->fy*(y*rad+c->p1*(r2+2*y*y)+2*c->p2*x*y)+c->cy;return isfinite(*u)&&isfinite(*v)&&*u>=0&&*u<960&&*v>=0&&*v<1080;
}
