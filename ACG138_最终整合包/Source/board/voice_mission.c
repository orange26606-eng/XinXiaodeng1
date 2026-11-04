#include "voice_mission.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fresh(uint64_t t,uint64_t n,uint64_t max){return t&&t<=n&&n-t<=max;}
static void state(voice_mission*t,const char*s,const char*m){snprintf(t->stage,sizeof t->stage,"%s",s);snprintf(t->message,sizeof t->message,"%s",m);}
void voice_mission_cancel(voice_mission*t,uint64_t floor,const char*why){if(t->id>floor)floor=t->id;if(floor>t->floor)t->floor=floor;t->active=0;t->revision++;state(t,"CANCELLED",why?why:"已明确停车；旧语音任务不会恢复");}
int voice_mission_accept(voice_mission*t,const phone_goal_message*m,const char*session,uint64_t seq,uint64_t now){
 if(!t||!m||!session||strlen(session)!=32||!seq||(!strcmp(t->session,session)&&seq<=t->last_sequence)||!m->map_id)return 0;
 if(m->operation==PHONE_SEMANTIC_CANCEL){t->last_sequence=seq;if(m->map_id<t->id){if(m->map_id>t->floor)t->floor=m->map_id;return 2;}voice_mission_cancel(t,m->map_id,"用户明确取消语音任务");return 3;}
 if(m->operation==PHONE_SEMANTIC_START){if(!t->active||t->id!=m->map_id||strcmp(t->session,session))return 0;if(!strcmp(t->spec.mode,"GO")||!strcmp(t->spec.mode,"FOLLOW")){t->last_sequence=seq;return 5;}if(strcmp(t->spec.mode,"PREVIEW"))return 0;t->last_sequence=seq;strcpy(t->spec.mode,"GO");if(t->prepared)t->phase=3;state(t,"START_PENDING","明确开始已收到；等待V8任务开始回执");return 4;}
 semantic_spec spec;if(!semantic_unpack(m,&spec)||m->map_id<=t->floor)return 0;
 if(m->map_id==t->id){if(!t->active||strcmp(t->session,session)||memcmp(&t->spec,&spec,sizeof spec))return 0;t->last_sequence=seq;t->duplicates++;return 2;}
 if(m->map_id<t->id)return 0;
 /* APK63每条新语句产生新的公开任务编号。相同的运行中跟随任务继续执行，不先STOP或重新准备；旧取消编号不能撤销新任务。
 */
 if(t->active&&t->phase>=4&&!strcmp(t->session,session)&&!strcmp(spec.mode,"FOLLOW")&&!memcmp(&t->spec,&spec,sizeof spec)&&strcmp(t->stage,"START_REJECTED")&&strcmp(t->stage,"WAIT_IDENTITY")){
  if(t->id>t->floor)t->floor=t->id;t->id=m->map_id;t->last_sequence=seq;t->revision++;t->duplicates++;return 6;
 }
 uint64_t floor=t->floor,revision=t->revision+1;if(t->id>floor)floor=t->id;memset(t,0,sizeof *t);t->floor=floor;t->revision=revision;t->id=m->map_id;t->last_sequence=seq;t->spec=spec;t->active=1;t->linked=1;memcpy(t->session,session,33);t->last_action_us=now;state(t,"SEARCH","语音任务已接收；等待真实目标，不虚构位置");return 1;
}
static int cat(const char*a,const char*b){return !*a||!strcmp(a,b)||(!strcmp(a,"cup")&&!strcmp(b,"glass"))||(!strcmp(a,"cell phone")&&!strcmp(b,"cell_phone"));}
static int attributes_match(const semantic_spec*s,const obj_detection*d){return obj_attribute_color_match(&d->attributes,s->color);}
static int observed(const obj_frame*f,int id){if(!f)return 0;for(int k=0;k<f->count;k++)if(f->objects[k].track_id==id&&f->objects[k].observed&&f->objects[k].bbox_valid)return 1;return 0;}
static int choose(const semantic_spec*s,const obj_frame*f){
 int found[OBJ_MAX],n=0;
 for(int k=0;k<f->count;k++)if(f->objects[k].observed&&f->objects[k].bbox_valid&&cat(s->category,f->objects[k].label)&&attributes_match(s,&f->objects[k]))found[n++]=k;
 if(!n)return -1;
 if(s->size[0]){
  int metric[OBJ_MAX],count=0;
  for(int i=0;i<n;i++){double e=obj_attribute_metric_extent(&f->objects[found[i]].attributes,s->category);if(isfinite(e)&&e>0)metric[count++]=found[i];}
  if(!count)return -1;
  for(int a=0;a<count;a++)for(int b=a+1;b<count;b++)if(obj_attribute_metric_extent(&f->objects[metric[a]].attributes,s->category)>obj_attribute_metric_extent(&f->objects[metric[b]].attributes,s->category)){int q=metric[a];metric[a]=metric[b];metric[b]=q;}
  if(s->position[0]||s->ordinal){
   /* “左边的小箱子”先筛出实际满足小尺寸的箱子，再按左右选择，不是先找全局最小框。
 */
   n=0;for(int k=0;k<count;k++)if(!strcmp(f->objects[metric[k]].attributes.size,s->size))found[n++]=metric[k];
   if(!n){
    if(count==1)return -1;
    int begin=!strcmp(s->size,"large")?count/2:!strcmp(s->size,"medium")?count/2:0;
    int end=!strcmp(s->size,"small")?(count+1)/2:!strcmp(s->size,"medium")?count/2+1:count;
    for(int k=begin;k<end;k++)found[n++]=metric[k];
   }
  }else{
   if(count==1&&strcmp(f->objects[metric[0]].attributes.size,s->size))return -1;
   int rank=!strcmp(s->size,"small")?0:!strcmp(s->size,"large")?count-1:count/2;found[0]=metric[rank];n=1;
  }
 }
 if(s->reference&&!s->position[0]&&!s->ordinal){for(int k=0;k<n;k++)if(f->objects[found[k]].track_id==f->selected)return f->selected;if(!s->category[0])return -1;}
 if(n>1&&!s->position[0]&&!s->ordinal){
  int best=found[0];double rank=1e30;
  for(int i=0;i<n;i++){const obj_detection*d=&f->objects[found[i]];if(d->track_id==f->selected)return d->track_id;double q=d->bearing_valid&&isfinite(d->bearing)?fabs(d->bearing):fabs((d->bbox[0]+d->bbox[2])*.5-480.)/480.;if(q<rank||(q==rank&&d->track_id<f->objects[best].track_id)){rank=q;best=found[i];}}
  return f->objects[best].track_id;
 }
 int distance=!strcmp(s->position,"near")||!strcmp(s->position,"far");
 for(int k=0;distance&&k<n;k++)if(!f->objects[found[k]].depth_valid)return -1;
 for(int a=0;a<n;a++)for(int b=a+1;b<n;b++){
  const obj_detection*x=&f->objects[found[a]],*y=&f->objects[found[b]];
  double av=distance?x->z:(x->bbox[0]+x->bbox[2])*.5,bv=distance?y->z:(y->bbox[0]+y->bbox[2])*.5;
  if(av>bv){int q=found[a];found[a]=found[b];found[b]=q;}
 }
 int reverse=!strcmp(s->position,"right")||!strcmp(s->position,"far");
 int k=s->ordinal?(s->ordinal<=(unsigned)n?(reverse?n-(int)s->ordinal:(int)s->ordinal-1):-1):reverse?n-1:!strcmp(s->position,"center")?(n%2?n/2:-1):0;
 return k>=0?f->objects[found[k]].track_id:-1;
}
static const char*category_name(const char*s){return !strcmp(s,"box")?"箱子":!strcmp(s,"bottle")?"水瓶":!strcmp(s,"person")?"人":!strcmp(s,"cup")?"杯子":!strcmp(s,"cell phone")||!strcmp(s,"cell_phone")?"手机":!strcmp(s,"remote")?"遥控器":s;}
static void selection_wait(voice_mission*t,const obj_frame*f,const char*stage){
 unsigned categories=0,colors=0,color_known=0,metric=0,size_label=0,positionable=0;const semantic_spec*s=&t->spec;
 for(int i=0;f&&i<f->count;i++){const obj_detection*d=f->objects+i;if(!d->observed||!cat(s->category,d->label))continue;categories++;if(d->attributes.color_valid)color_known++;if(!attributes_match(s,d))continue;colors++;double extent=obj_attribute_metric_extent(&d->attributes,s->category);if(isfinite(extent)&&extent>0){metric++;if(!strcmp(d->attributes.size,s->size))size_label++;}if(d->bbox_valid)positionable++;}
 t->selection_category_count=categories;t->selection_color_count=colors;t->selection_metric_count=metric;t->selection_frame_id=f?f->frame_id:0;t->selection_source_us=f?f->frame_us:0;
 const char*reason="POSITION_OR_ORDINAL_UNAVAILABLE";char msg[256];
 if(!categories){reason="CATEGORY_NOT_OBSERVED";snprintf(msg,sizeof msg,"当前真实检测没有%s；任务保留并继续寻找",s->category[0]?category_name(s->category):"所指目标");}
 else if(s->color[0]&&!colors){reason=color_known?"COLOR_NOT_MATCHED":"COLOR_UNAVAILABLE";snprintf(msg,sizeof msg,color_known?"已检测到%s，但没有匹配指定%s的目标；未知颜色未冒充匹配，继续观察":"已检测到%s，但颜色尚不可辨，未确认指定%s；任务保留继续观察",category_name(s->category),obj_color_zh(obj_color_id(s->color)));}
 else if(s->size[0]&&!metric){reason="SIZE_METRIC_UNAVAILABLE";snprintf(msg,sizeof msg,"类别和颜色已匹配，但尚无真实尺寸支持大小判断；不能按画面框大小选错目标，保留任务继续观察");}
 else if(s->size[0]&&metric==1&&!size_label){reason="SIZE_NOT_MATCHED";snprintf(msg,sizeof msg,"已有一笔真实尺寸，但当前%s不符合指定大小条件；保留任务，不用像素框冒充实物大小",category_name(s->category));}
 else if(!positionable){reason="CURRENT_BOX_POSITION_UNAVAILABLE";snprintf(msg,sizeof msg,"类别和属性已匹配，但当前没有可用的真实框位置；继续等待新检测，不等待PNG");}
 else if(!strcmp(s->position,"near")||!strcmp(s->position,"far")){reason="DISTANCE_ORDER_UNAVAILABLE";snprintf(msg,sizeof msg,"目标类别和属性已匹配，但缺少候选距离，尚不能按最近或最远排序；任务保留");}
 else if(s->ordinal){reason="ORDINAL_NOT_AVAILABLE";snprintf(msg,sizeof msg,"当前匹配候选不足以确定第%u个目标；类别匹配%u、颜色匹配%u、真实尺寸%u，任务保留",s->ordinal,categories,colors,metric);}
 else if(s->reference){reason="REFERENCED_TARGET_NOT_OBSERVED";snprintf(msg,sizeof msg,"当前所指目标没有满足条件的真实观测；保留原指代，不自动换成其他物体");}
 else snprintf(msg,sizeof msg,"当前目标条件已部分匹配，但指定方位尚无合适候选；类别匹配%u、颜色匹配%u、真实尺寸%u，继续观察",categories,colors,metric);
 snprintf(t->selection_reason,sizeof t->selection_reason,"%s",reason);state(t,stage,msg);
}
/* “跟随我”指向当前可见的人，优先采用当前选中且符合的目标，否则选择观测到的居中人物，不增加抓拍、分数或连续确认条件。

 */
static int follow_choose(const semantic_spec*s,const obj_frame*f){
 if(!f||!f->epoch)return -1;
 if(s->position[0]||s->ordinal||s->size[0]){semantic_spec ranked=*s;ranked.reference=0;return choose(&ranked,f);}
 if(!s->category[0])return choose(s,f);
 int best=-1;double rank=1e30;
 for(int k=0;k<f->count;k++){
  const obj_detection*d=&f->objects[k];
  if(d->track_id<1||!d->observed||!cat(s->category,d->label)||!attributes_match(s,d))continue;
  if(d->track_id==f->selected)return d->track_id;
  
  double score=d->bearing_valid&&isfinite(d->bearing)?fabs(d->bearing):1e20;
  if(score==1e20&&d->bbox_valid&&isfinite(d->bbox[0])&&isfinite(d->bbox[2]))score=fabs((d->bbox[0]+d->bbox[2])*.5-480.0)/480.0;
  if(best<1||score<rank||(score==rank&&d->track_id<best)){best=d->track_id;rank=score;}
 }
 return best;
}
static int track_observed(const obj_frame*f,int id){if(!f)return 0;for(int i=0;i<f->count;i++)if(f->objects[i].track_id==id&&f->objects[i].observed)return 1;return 0;}
int voice_mission_step(voice_mission*t,const obj_frame*f,const obj_frame*shot,const voice_receipt*r,uint64_t now){
 if(!t->active)return 0;if(!t->linked){state(t,"WAIT_LINK","真实控制心跳中断；保留任务等待原会话，原V8失联保护仍停车");return 0;}
 int live=r&&fresh(r->published_us,now,1500000),same=live&&r->epoch==t->epoch&&r->track==t->track&&!strcmp(r->mode,!strcmp(t->spec.mode,"FOLLOW")?"follow":"static");
 /* 显式前往请求在没有相机目标时也携带类别，表示未绑定目标的搜索意图，不构造检测编号。
 */
 if(t->phase<2&&!strcmp(t->spec.mode,"GO")&&t->spec.category[0]&&!t->spec.position[0]&&!t->spec.ordinal&&!t->spec.reference){t->search_requested=1;t->epoch=0;t->track=0;state(t,"SEARCH_TURN","静止任务向左搜索：0.30rad/s，无总时限；发现类别、颜色和大小条件匹配的真实目标即锁定");return VOICE_SEARCH_PREPARE;}
 if(t->phase==2){if(live&&r->receipt_id==t->prepare_id&&!strcmp(r->receipt_state,"REJECTED")){state(t,"PREPARE_REJECTED",r->receipt_reason);if(!strcmp(r->receipt_reason,"SELECTED_OBJECT_NOT_YET_EXPORTED_WAIT_NEW_FRAME")){t->phase=1;t->prepare_id=0;}return 0;}
 if(same&&r->request_id==t->prepare_id&&r->prepared){t->prepared=1;t->phase=3;state(t,"PREPARED","V8已确认目标准备；尚未把排队回执当作运动许可");}else{state(t,"WAIT_PREPARE_ACK","等待本次V8准备回执；不重复创建任务、不提前开始");return 0;}}
 if(t->phase==3){if(!strcmp(t->spec.mode,"PREVIEW")){state(t,"PREVIEW","仅预览；说开始导航后才提交一次START");return 0;}return VOICE_START;}
 if(t->phase>=4){if(!live){state(t,"WAIT_CONTROLLER","V8任务回报超过1500毫秒；保留任务，不盲目重发开始");return 0;}if(r->receipt_id==t->start_id&&!strcmp(r->receipt_state,"REJECTED")){state(t,"START_REJECTED",r->receipt_reason);return 0;}if((t->search_requested||r->epoch==t->epoch)&&r->request_id==t->start_id&&!strcmp(r->mode,!strcmp(t->spec.mode,"FOLLOW")?"follow":"static")){t->epoch=r->epoch;t->track=r->track;if(!strcmp(r->state,"ARRIVED")){t->active=0;state(t,"ARRIVED","V8已确认到达并保持停车");return 0;}if(!strcmp(r->state,"CANCELLED")){voice_mission_cancel(t,t->id,"V8任务已取消，语音任务不会自动重新开始");return 0;}t->phase=5;state(t,r->state,r->message);
 /* 控制器锁定目标后才执行可选感知选择，该操作不阻止首次转向或前进，也不能恢复已取消任务。
 */
 if((t->search_requested||t->direct_static)&&t->track>0&&f&&shot&&f->epoch==t->epoch&&shot->epoch==t->epoch&&f->selected!=t->track&&observed(shot,t->track)&&(!t->selection_id||now-t->last_action_us>=1000000))return VOICE_SELECT;
 return 0;}state(t,"WAIT_START_ACK","START已提交一次；只查原请求，不因漏回执再次启动");return 0;}
 if(!f||!fresh(f->frame_us,now,2000000)){state(t,"WAIT_IMAGE","等待2秒以内的真实目标图像；任务没有总寻找超时");return 0;}
 /* 准备阶段重新评估目标；进入执行或取消状态后固定该次任务选择。 */
 if(t->phase<2&&t->track&&(f->epoch!=t->epoch||!track_observed(f,t->track))&&t->spec.category[0]&&!t->spec.reference){t->track=0;t->epoch=0;t->selection_id=0;}
 if(!strcmp(t->spec.mode,"FOLLOW")){
  int id=follow_choose(&t->spec,f);
  if(id<1){selection_wait(t,f,"WAIT_PERSON");return 0;}
  snprintf(t->selection_reason,sizeof t->selection_reason,"MATCHED_REAL_DETECTION");t->direct_follow=1;t->track=id;t->epoch=f->epoch;
  state(t,"PREPARING_DIRECT_FOLLOW","当前画面已识别到跟随对象，直接准备并自动开始；不等待抓拍或选中确认");return VOICE_DIRECT_PREPARE;
 }
 if(!strcmp(t->spec.mode,"GO")){
  int id=choose(&t->spec,f);if(id<1){selection_wait(t,f,"SEARCH");return 0;}
  snprintf(t->selection_reason,sizeof t->selection_reason,"MATCHED_REAL_DETECTION");t->direct_static=1;t->track=id;t->epoch=f->epoch;
  state(t,"PREPARING_DIRECT_STATIC","已用当前真实检测绑定指定静止目标，直接准备并自动开始；不等待PNG或感知选中回执");return VOICE_DIRECT_STATIC_PREPARE;
 }
 if(t->track&&f->epoch!=t->epoch){state(t,"WAIT_IDENTITY","感知采集会话改变，保留任务；需要明确重新选择目标");return 0;}
 if(!t->track){int id=choose(&t->spec,f);if(id<1){selection_wait(t,f,"SEARCH");return 0;}t->track=id;t->epoch=f->epoch;}
 if(!strcmp(t->spec.mode,"FIND")||!strcmp(t->spec.mode,"SELECT")){state(t,"FOUND","已发现满足类别、颜色和大小的真实目标；此命令不请求电机运动");if(f->selected!=t->track&&shot&&shot->epoch==t->epoch&&fresh(shot->frame_us,now,2000000)&&observed(shot,t->track)&&(!t->selection_id||now-t->last_action_us>=1000000))return VOICE_SELECT;return 0;}
 if(f->selected!=t->track){if(!shot||shot->epoch!=t->epoch||!fresh(shot->frame_us,now,2000000)||!observed(shot,t->track)){state(t,"WAIT_SNAPSHOT","等待包含目标的真实抓拍，未伪造图像帧身份");return 0;}state(t,"SELECTING","等待感知确认同一目标；旧选中回调不能启动新任务");if(!t->selection_id||now-t->last_action_us>=1000000)return VOICE_SELECT;return 0;}
 if(!strcmp(t->spec.mode,"FIND")||!strcmp(t->spec.mode,"SELECT")){state(t,"FOUND","已找到并选中真实物体；此命令不请求电机运动");return 0;}
 state(t,"PREPARING","真实选择已确认，准备走V8原有控制事务");return VOICE_PREPARE;
}
int voice_mission_control_message(const voice_mission*t,phone_goal_message*m){
 if(!t||!m||!t->active)return 0;memset(m,0,sizeof *m);m->object_epoch=t->epoch;m->object_track_id=t->track;m->x_m=t->spec.distance;
 m->reserved=t->direct_follow?5:t->direct_static?7:t->search_requested?4:!strcmp(t->spec.mode,"FOLLOW")?2:1;
 return !t->search_requested||semantic_search_pack(m,&t->spec);
}
void voice_mission_issued(voice_mission*t,unsigned action,uint64_t id,uint64_t now){t->last_action_us=now;t->attempts++;if(action==VOICE_SELECT)t->selection_id=id;else if(action==VOICE_PREPARE||action==VOICE_SEARCH_PREPARE||action==VOICE_DIRECT_PREPARE||action==VOICE_DIRECT_STATIC_PREPARE){t->prepare_id=id;t->phase=2;}else if(action==VOICE_START){t->start_id=id;t->phase=4;}}
static void safe(char*out,size_t cap,const char*s){size_t n=0;for(;*s&&n+1<cap;s++){unsigned char c=(unsigned char)*s;out[n++]=c<32||c=='"'||c=='\\'?'_':(char)c;}out[n]=0;}
int voice_mission_json(const voice_mission*t,const voice_receipt*r,uint64_t now,char*b,size_t cap){char msg[300];safe(msg,sizeof msg,t->message);int count=snprintf(b,cap,"{\"version\":1,\"published_us\":\"%llu\",\"task_id\":\"%llu\",\"active\":%s,\"linked\":%s,\"mode\":\"%s\",\"category\":\"%s\",\"position\":\"%s\",\"ordinal\":%u,\"reference\":%s,\"stage\":\"%s\",\"message\":\"%s\",\"epoch\":\"%llu\",\"track_id\":%d,\"prepared\":%s,\"selection_attempts\":%u,\"duplicates\":%u,\"search_cycles\":0,\"drive_enabled\":%s,\"enabled\":%s,\"pending_enable\":%s,\"requested_v\":%.4f,\"requested_w\":%.4f,\"protected_v\":%.4f,\"protected_w\":%.4f,\"prepare_request_id\":\"%llu\",\"start_request_id\":\"%llu\",\"controller_reason\":\"%s\"}",(unsigned long long)now,(unsigned long long)t->id,t->active?"true":"false",t->linked?"true":"false",t->spec.mode,t->spec.category,t->spec.position,t->spec.ordinal,t->spec.reference?"true":"false",t->stage,msg,(unsigned long long)t->epoch,t->track,t->prepared?"true":"false",t->attempts,t->duplicates,r&&r->drive?"true":"false",r&&r->enabled?"true":"false",r&&r->pending?"true":"false",r?r->v:0,r?r->w:0,r?r->pv:0,r?r->pw:0,(unsigned long long)t->prepare_id,(unsigned long long)t->start_id,r?r->reason:"WAIT_CONTROLLER");
 if(count<0||(size_t)count>=cap)return count;
 const char*layer=!t->active?"TASK_INACTIVE":!t->linked?"PHONE_SESSION":t->phase==2?"PREPARE_RECEIPT":t->phase==3?"USER_START_OR_START_QUEUE":t->phase==4?"START_RECEIPT":t->phase>=5?"CONTROLLER_EXECUTION":"TARGET_SELECTION";
 int more=snprintf(b+count-1,cap-(size_t)count+1,",\"execution_phase\":%d,\"search_left\":%s,\"search_angular_limit\":%.2f,\"blocked_layer\":\"%s\",\"controller_age_ms\":%lld,\"controller_request_id\":\"%llu\",\"controller_state\":\"%s\",\"controller_receipt_state\":\"%s\",\"controller_receipt_reason\":\"%s\"}",t->phase,t->search_requested?"true":"false",t->search_requested?.30:0,layer,(long long)(r&&r->published_us&&r->published_us<=now?(now-r->published_us)/1000:UINT64_MAX),(unsigned long long)(r?r->request_id:0),r?r->state:"",r?r->receipt_state:"",r?r->receipt_reason:"");
 if(more<0||(size_t)more>=cap-(size_t)count+1)return -1;
 count=count-1+more;
 int tail=snprintf(b+count-1,cap-(size_t)count+1,",\"direct_follow\":%s,\"snapshot_required_to_start\":%s}",t->direct_follow?"true":"false",(!strcmp(t->spec.mode,"FOLLOW")||t->search_requested||t->direct_static)?"false":"true");
 if(tail<0||(size_t)tail>=cap-(size_t)count+1)return -1;
 count=count-1+tail;
 int input=snprintf(b+count-1,cap-(size_t)count+1,",\"input_source\":\"%s\",\"input_frame_id\":\"%llu\",\"input_source_us\":\"%llu\",\"input_objects\":%d,\"input_observed_persons\":%d}",t->input_kind,(unsigned long long)t->input_frame_id,(unsigned long long)t->input_frame_us,t->input_count,t->input_observed_persons);
 if(input<0||(size_t)input>=cap-(size_t)count+1)return -1;count=count-1+input;
 int attrs=snprintf(b+count-1,cap-(size_t)count+1,",\"requested_color\":\"%s\",\"requested_size\":\"%s\",\"direct_static\":%s}",t->spec.color,t->spec.size,t->direct_static?"true":"false");
 if(attrs<0||(size_t)attrs>=cap-(size_t)count+1)return -1;count=count-1+attrs;
 int select=snprintf(b+count-1,cap-(size_t)count+1,",\"selection_reason\":\"%s\",\"selection_category_count\":%u,\"selection_color_count\":%u,\"selection_metric_count\":%u,\"selection_frame_id\":\"%llu\",\"selection_source_us\":\"%llu\"}",t->selection_reason,t->selection_category_count,t->selection_color_count,t->selection_metric_count,(unsigned long long)t->selection_frame_id,(unsigned long long)t->selection_source_us);
 if(select<0||(size_t)select>=cap-(size_t)count+1)return -1;return count-1+select;
}
