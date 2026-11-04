/* 场景分类器，只处理输入样本。传感器外参和净空融合由调用方提供。 */
#ifndef ACG_R4_SCENE_CLASSIFIER_INC
#define ACG_R4_SCENE_CLASSIFIER_INC
#include "r4_scene_types.h"
#include <math.h>
#include <string.h>
#define R4_PI 3.14159265358979323846
static double r4_wrap(double a){return atan2(sin(a),cos(a));}
static double r4_min(double a,double b){return a<b?a:b;}
static void r4_scene_config_default(r4_scene_config*c){
 memset(c,0,sizeof(*c));c->direction_max_age_us=CONT_SCAN_US;c->scene_confirm_us=500000;c->scene_exit_us=800000;
 c->band_up_hold_us=300000;c->band_up_hyst=1.1;c->w_straight=.1;
 c->body_length_m=.333;c->body_width_m=.320;c->body_pad_m=.010;
 c->front_brake_m=R6_FRONT_EMERGENCY_MM/1000.;c->side_straight_m=R6_SIDE_STRAIGHT_EMERGENCY_MM/1000.;c->side_turn_m=R6_SIDE_TURN_MM/1000.;c->side_release_m=.060;
 c->reverse_speed_m_s=R6_REVERSE_SPEED_M_S;c->reverse_target_m=R6_REVERSE_TARGET_M;c->reverse_hard_m=R6_REVERSE_HARD_M;
 c->reverse_extended_m=c->reverse_target_m;c->reverse_extended_hard_m=c->reverse_hard_m;c->reverse_max_us=(uint64_t)R6_REVERSE_MAX_MS*1000;c->reverse_extended_max_us=c->reverse_max_us;
 c->reverse_attempts=20;c->reverse_extended_attempts=1;c->explore_rounds=6;
 c->pivot_w=2*78*R4_PI*.0675/(60*.29);c->explore_step_rad=30*R4_PI/180;
 c->explore_side_budget_rad=90*R4_PI/180;c->explore_fast_m_s=.20;c->explore_crawl_m_s=.18;
 c->explore_window_us=10000000;c->explore_progress_m=.050;c->switch_hold_us=500000;c->switch_score_ratio=1.3;
 c->doorway_length_m=.5;c->doorway_max_us=1500000;c->continuous_exploration=1;
}
static void r4_scene_init(r4_scene_state*s){memset(s,0,sizeof(*s));s->scene=R4_GENERAL;s->candidate_scene=R4_GENERAL;s->previous_front=-1;}
static int r4_direction_usable(const r4_direction*d,uint64_t now,uint64_t max_age){
 return d&&isfinite(d->clearance_m)&&d->clearance_m>=0&&(d->quality&R4_FRESH)&&
 !(d->quality&(R4_INVALID|R4_CONTRADICTORY|R4_RAW_ZERO))&&d->source_us&&d->source_us<=now&&now-d->source_us<=(d->max_age_us&&d->max_age_us<max_age?d->max_age_us:max_age);
}
static int r4_new_sample(const r4_direction*d,uint64_t*stamp,uint32_t*seq){
 if(d->source_us<=*stamp||(*stamp&&d->sequence==*seq))return 0;
 *stamp=d->source_us;*seq=d->sequence;return 1;
}
static const char*r4_scene_name(uint32_t scene){switch(scene){
 case R4_OPEN:return "开阔";case R4_FRONT_NEAR:return "前方近障";case R4_WALL_LEFT:return "左侧沿墙";
 case R4_WALL_RIGHT:return "右侧沿墙";case R4_CORRIDOR:return "走廊";case R4_CORNER:return "拐角";
 case R4_DEAD_END:return "死胡同";case R4_DOORWAY:return "门口";case R4_LOW_OBSTACLE:return "低矮障碍";
 case R4_GLASS:return "疑似玻璃";case R4_OVERHANG:return "悬空障碍";case R4_RETREAT:return "后退";
 case R4_GENERAL:return "一般环境";default:return "未分类环境";}}
static uint32_t r4_scene_rank(uint32_t scene){switch(scene){
 case R4_FRONT_NEAR:case R4_DEAD_END:case R4_LOW_OBSTACLE:case R4_GLASS:case R4_OVERHANG:return 6;
 case R4_RETREAT:return 7;case R4_DOORWAY:return 5;case R4_CORNER:return 4;
 case R4_CORRIDOR:return 3;case R4_WALL_LEFT:case R4_WALL_RIGHT:return 2;case R4_OPEN:return 1;default:return 0;}}
static uint32_t r4_scene_classify(const r4_scene_input*i,const r4_scene_config*c,r4_scene_state*s){
 int f=r4_direction_usable(&i->direction[0],i->now_us,c->direction_max_age_us);
 int l=r4_direction_usable(&i->direction[1],i->now_us,c->direction_max_age_us);
 int r=r4_direction_usable(&i->direction[2],i->now_us,c->direction_max_age_us);
 /* 部分方向缺失只影响需要该方向证据的动作，不交给旧S5。
  * GENERAL不代表未知已清空；每个实际候选仍检查自己的来源和完整扫掠。 */
 if(!f||!l||!r)return R4_GENERAL;
 double F=i->direction[0].clearance_m,L=i->direction[1].clearance_m,R=i->direction[2].clearance_m;
 if(r4_new_sample(&i->direction[0],&s->front_trend_source,&s->front_trend_seq)){
  if(s->previous_front>=0&&F<s->previous_front-.001){if(s->front_approach_count<3)s->front_approach_count++;}
  else s->front_approach_count=0;s->previous_front=F;
 }
 /* CORNER只细分场景，不改变前向硬阈值。连续下降必须来自至少三笔不同新观测。 */
 int single_wall=(L>=.015&&L<=.120&&R>.300)||(R>=.015&&R<=.120&&L>.300);
 s->corner_observed=(F>c->front_brake_m&&F<R6_FRONT_OBSERVE_MM/1000.&&s->front_approach_count>=2)||(F>c->front_brake_m&&F<R6_FRONT_OBSERVE_MM/1000.&&single_wall);
 if(i->glass)return R4_GLASS;if(i->low_obstacle)return R4_LOW_OBSTACLE;if(i->overhang)return R4_OVERHANG;
 if(s->stage==R4_CHECK_REAR||s->stage==R4_REVERSE||s->stage==R4_EXIT_TURN)return R4_RETREAT;
 if(F<R6_FRONT_FORWARD_STOP_MM/1000.&&L<.050&&R<.050&&r4_direction_usable(&i->direction[3],i->now_us,c->direction_max_age_us)&&i->direction[3].clearance_m>R6_REVERSE_MIN_FIT_M)return R4_DEAD_END;
 if(s->corner_observed)return R4_CORNER;
 /* 近障是标签，不得抢先覆盖一条已经可执行的路线。 */
 if(F<R6_FRONT_NEAR_MM/1000.)return R4_FRONT_NEAR;
 if(L<.060&&R<.060){
  if(!s->doorway_seen){s->doorway_seen=1;s->doorway_since=i->now_us;s->doorway_x=i->x_m;s->doorway_y=i->y_m;}
  /* 长度、时间都是有限预算：任一耗尽就不再假定是短门口。 */
  if(i->now_us>=s->doorway_since&&i->now_us-s->doorway_since<c->doorway_max_us&&
    hypot(i->x_m-s->doorway_x,i->y_m-s->doorway_y)<c->doorway_length_m)return R4_DOORWAY;
 }else s->doorway_seen=0;
 if(L>=.015&&R>=.015&&L<=.300&&R<=.300)return R4_CORRIDOR;
 if(L>=.015&&L<=.120&&R>.300)return R4_WALL_LEFT;
 if(R>=.015&&R<=.120&&L>.300)return R4_WALL_RIGHT;
 if(F>.600&&L>.300&&R>.300)return R4_OPEN;
 return R4_GENERAL;
}
static void r4_scene_confirm(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,uint32_t candidate){
 /* 500/800ms只稳定显示标签，不转移控制所有权，也不延迟新障碍终检。 */
 if(candidate==s->scene){s->candidate_scene=candidate;s->candidate_since=i->now_us;return;}
 if(candidate!=s->candidate_scene){s->candidate_scene=candidate;s->candidate_since=i->now_us;return;}
 uint64_t hold=r4_scene_rank(candidate)<r4_scene_rank(s->scene)?c->scene_exit_us:c->scene_confirm_us;
 if(i->now_us>=s->candidate_since&&i->now_us-s->candidate_since>=hold){s->scene=candidate;s->scene_since=i->now_us;}
}
static double r4_front_band(double F){static const double bands[R6_SPEED_BAND_COUNT][2]=R6_SPEED_BANDS;for(unsigned k=0;k<R6_SPEED_BAND_COUNT;k++)if(F*1000.+1e-9>=bands[k][0])return bands[k][1];return 0;}
static double r4_band_threshold(double cap){static const double bands[R6_SPEED_BAND_COUNT][2]=R6_SPEED_BANDS;for(unsigned k=0;k<R6_SPEED_BAND_COUNT;k++)if(cap>=bands[k][1]-1e-9)return bands[k][0]/1000.;return R6_FRONT_FORWARD_STOP_MM/1000.;}
static double r4_speed_band(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i){
 const r4_direction*d=&i->direction[0];double target=r4_front_band(d->clearance_m);
 if(!s->initialized){s->speed_cap=r4_min(target,.12);s->initialized=1;}
 if(target<s->speed_cap){s->speed_cap=target;s->band_since=0;s->band_target=target;return target;}
 if(target<=s->speed_cap+1e-9){s->band_since=0;return s->speed_cap;}
 /* 升档要求1.1倍边界且由持续的新观测支持；重复输入不能推进升档计时。 */
 if(d->clearance_m<r4_band_threshold(target)*c->band_up_hyst){s->band_since=0;return s->speed_cap;}
 if(s->band_target!=target||!s->band_since){s->band_target=target;s->band_since=i->now_us;s->band_last_sample=d->source_us;return s->speed_cap;}
 if(d->source_us>s->band_last_sample){s->band_last_sample=d->source_us;
  if(i->now_us>=s->band_since&&i->now_us-s->band_since>=c->band_up_hold_us)s->speed_cap=target;}
 return s->speed_cap;
}
#endif
