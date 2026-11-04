#ifndef ACG_R4_STATUS_TEXT_H
#define ACG_R4_STATUS_TEXT_H
#include <stdint.h>
#include <string.h>
#include "r6_motion_policy.h"
/* 只用于显示，不参与场景仲裁、命令许可或手机会话。 */
static inline const char*r4_scene_zh(uint32_t scene){
 switch(scene){case 0:return "开阔道路";case 1:return "前方近障";case 2:return "左侧沿墙";
 case 3:return "窄道居中";case 4:return "拐角调整";case 5:return "死胡同找出口";
 case 6:return "门口直行";case 7:return "低矮障碍";case 8:return "疑似玻璃墙";
 case 9:return "悬空结构区域";case 10:return "后退腾挪";case 12:return "右侧沿墙";
 case 13:return "等待有效观测";case 14:return "通用已观测环境";default:return "场景待识别";}
}
static inline const char*r4_owner_zh(uint32_t owner){
 switch(owner){case 0:return "路线跟踪";case 1:return "用户停止或故障";case 2:return "紧急近障保护";
 case 3:return "近场场景机";case 4:return "完成已选动作";case 5:return "历史兼容规则";
 case 6:return "等待必要观测，目标保留";default:return "控制归属待确认";}
}
static inline const char*r4_stage_zh(uint32_t stage){
 switch(stage){case 0:return "按场景行驶";case 1:return "检查后方空间";case 2:return "直线后退腾挪";
 case 3:return "沿出口方向调整";case 4:return "停稳，等待两笔至少40毫米的新观测";
 case 11:return "比较两侧出口";case 12:return "转向观察新空间";case 13:return "向已观测空间前探";
 case 14:return "发现出口，恢复目标导航";case 15:return "本轮重新选择方向";
 case 16:return "等待新的可行动作";default:return "动作阶段待确认";}
}
static inline const char*r6_check_zh(int result){
 switch(result){case R6_CHECK_CLEAR:return "当前实测防护通过，非未来碰撞保证";case R6_CHECK_OCCUPIED:return "当前实测近障或车身重叠";
 case R6_CHECK_UNOBSERVED:return "动作涉及区域尚未观测";case R6_CHECK_STALE:return "观测源超过有效年龄";
 case R6_CHECK_POSE_INVALID:return "当前运动位姿不可用";case R6_CHECK_BUDGET:return "本轮计算预算不足，保留路线等待重试";
 case R6_CHECK_WIRE_REJECT:return "底盘方向门拒绝当前候选";default:return "动作参数或检查状态无效";}
}
static inline const char*r6_route_side_zh(int side,double v,double w){
 if(v==0&&w==0)return "停稳";return side>0?"左转":side<0?"右转":v<0?"直退":"直行";
}
typedef struct{const char*code;const char*zh;}r4_reason_label;
/* 精确表同时生成手机显示fallback；新增真实原因必须补齐此表。 */
static const r4_reason_label r4_reason_labels[]={
 {"R8_OBSERVE_BRAKE_BEFORE_CHANGE","缺少该动作的空间证据，先停稳再换观察动作"},
{"R8_OBSERVE_ROUTE_SIDE_TURN","沿路线期望侧小角度转向，获取新的空间观测"},
{"R8_OBSERVE_OTHER_SIDE_TURN","原侧观察动作未通过，向另一已验证方向小角度观察"},
{"R8_OBSERVE_ROUTE_SIDE_ARC","沿路线期望侧执行已检查的短弧线，换视角补充观测"},
{"R8_OBSERVE_OTHER_SIDE_ARC","沿另一已检查短弧线换视角，导航目标保持"},
{"R8_OBSERVE_SHORT_REVERSE","其他观察动作暂不可行，沿已检查后方短退后重看路线"},
{"R8_OBSERVE_SEGMENT_DONE_RESCAN","本段观察动作到达边界，等待新扫描重新判断路线"},
{"R8_OBSERVE_RECHECK_NEW_SCAN","已收到新扫描，重新检查原路线与可行观察动作"},
{"R8_OBSERVE_NO_VERIFIED_ACTION","当前各观察动作都缺少通过证据，保留目标等待新观测"},
{"R8_OBSERVE_WAIT_FRESH_SOURCES","观察动作所需雷达、位姿或轮速过期，等待对应新数据"},
{"R8_OBSERVE_COMPUTE_RETRY","观察动作检查预算不足，保留目标与阶段后重试"},
{"INITIAL_BUFFER_NOT_CLEARING","已进入车身缓冲区，该动作未持续拉开距离，改查其他脱离动作"},
 {"FRONT_20MM_BRAKE","前方新鲜非零净空不超过20毫米，立即刹车后重新检查脱离动作"},
 {"R6_ACTION_CHECK_INVALID","候选动作参数或检查上下文无效，保留目标等待下一轮"},
 {"R6_ACTION_HARD_GATE_REJECTED","候选未获底盘方向门许可，继续核对具体门限和替代动作"},
 {"R6_ACTION_OCCUPIED","完整车身扫掠确认该候选涉及实际障碍，检查其他动作"},
 {"R6_ACTION_POSE_INVALID","当前位姿不可用，无法完成动作扫掠与位移核对"},
 {"R6_ACTION_SOURCE_STALE","该动作依赖的观测源过期，保留路线等待新观测"},
 {"R6_ACTION_UNOBSERVED","该动作涉及区域尚未观测，不能当成空地直接驶入"},
 {"R6_BRAKE_BEFORE_DIRECTION_CHANGE","先降低当前运动，停稳后再切换实际不可行的方向"},
 {"R6_BRAKE_BEFORE_SHORT_RETREAT","前进或转向暂不可行，先停稳再执行已检查的短段后退"},
 {"R6_BRAKING_SPEED_LIMIT_WAIT","当前制动余量不足，沿原路线减速并等待复核"},
 {"R6_COMPUTE_BUDGET_WAIT","本轮计算预算不足，保持路线与恢复阶段，下一轮重试"},
 {"R6_FORWARD_TURNS_UNAVAILABLE_CHECK_SHORT_REAR","前进和两侧转向均未通过检查，改查短段后退路径"},
 {"R6_NEAR_ROUTE_FIRST","前方进入50毫米近障区，仍先检查原路线的低速可行动作"},
 {"R6_NO_ACTIVE_TASK_NO_RECOVERY","当前没有活动导航或巡航任务，不自行开始脱离动作"},
 {"R6_NO_VERIFIED_ACTION_TARGET_PRESERVED","当前没有经过验证的动作，保留目标等待新观测后重试"},
 {"R6_OBSERVE_ROUTE_FIRST","前方进入100毫米观察区，优先沿路线按制动余量减速"},
 {"R6_REAR_PATH_NOT_AVAILABLE","后方不足以完成已检查的短退路径，继续观察和核对路线"},
 {"R6_RETREAT_SEGMENT_DONE_REASSESS_ROUTE","本次短退已到距离或时间边界，停稳并重新核对原路线"},
 {"R6_ROUTE_CLEAR_BRAKE_TO_REJOIN","原路线两笔新观测已证明可行，先刹稳再沿路线恢复"},
 {"R6_ROUTE_CLEAR_WAIT_TWO_NEW","原路线候选已可行，等待第二笔新观测确认恢复"},
 {"R6_ROUTE_RECOVERED_NO_FIXED_TURN","原路线恢复可行，直接恢复跟踪，不执行固定角度退出转向"},
 {"R6_ROUTE_SIDE_TURN","按路线期望侧执行已检查的转向，持续重新核对路线"},
 {"R6_SHORT_RETREAT_012M","执行已检查的短段后退，目标120毫米，硬限150毫米或1.2秒"},
 {"R6_TURN_TIME_REASSESS_WAIT_NEW","本段转向到达角度或时间边界，停稳后用新观测重新核对路线"},
 {"COMPUTE_BUDGET_RETRY_SAME_ROUTE","计算预算不足，保留原路线下一轮重试，不据此换侧或后退"},
 {"MEASURED_MOTION_BRAKE_ROUTE_RETAINED","当前实际运动的制动空间不足，先刹车并保留原路线"},
 {"CURRENT_BRAKING_CHECK_WAIT","当前制动路径尚不能完成检查，等待必要数据或计算预算"},
 {"ROUTE_CURVATURE_PRESERVED_SLOWER","原路线较低速度已通过检查，保持转向方向减速行驶"},
 {"ROUTE_PREFERRED_CLEAR","原路线动作通过检查，按预览方向执行"},
 {"ROUTE_SIDE_REJECTED_OTHER_SIDE_CLEAR","原路线侧动作被确认不可行，另一侧完整检查通过后改向"},
 {"ROUTE_SIDE_ALTERNATIVE_CLEAR","优先侧的替代动作已通过检查，保持路线侧行驶"},
 {"ROUTE_BLOCKED_SCENE_ESCAPE_CHECKED","原路线动作受阻，执行场景机已验证的脱离动作"},
 {"NEARFIELD_SCAN_UNAVAILABLE","当前缺少可用近场扫描，不能核对该候选的完整车身扫掠"},
 {"SONAR_BODY_SWEEP","超声观测点与该候选的车身扫掠相交，继续检查其他动作"},
 {"ACTION_INPUT_INVALID","当前候选参数或检查输入无效，保留任务等待下一轮"},
 {"FRONT_UNDER_30_REVERSE_OR_TURN_ONLY","前方不足30毫米，不继续前移，按实际净空检查转向或短退"},
 {"FPGA_FRONT_UNDER_30MM","前方不足30毫米，底盘不批准前移；继续检查可行脱离动作"},
 {"FPGA_FRONT_NEAR_LATCH_RELEASE_40MM","前向近障锁存尚未释放，等待两笔至少40毫米的新观测"},
 {"PARTIAL_DIRECTION_WAIT_NEW_OBSERVATION","必要方向观测不足，保留目标等待新观测后重新选择动作"},
 {"R5_UNIFIED_ACTION_RESELECT","统一控制器重新选择动作，不继承旧动作的有限重试预算"},
 {"FPGA_STATUS_MISSING","尚未收到FPGA状态，无法核对当前动作的底层许可"},
 {"FPGA_STATUS_AGE_OVER_250MS","FPGA状态超过250毫秒，等待新状态后再检查动作"},
 {"FPGA_INTENT_WHEEL_SHAPE_MISMATCH","候选动作类别与实际左右轮速组合不一致，拒绝本候选"},
 {"FPGA_REVERSE_LEFT_SONAR_UNAVAILABLE","倒车底层门要求左侧超声有效新鲜，当前尚不满足"},
 {"FPGA_REVERSE_RIGHT_SONAR_UNAVAILABLE","倒车底层门要求右侧超声有效新鲜，当前尚不满足"},
 {"FPGA_PIVOT_FRONT_SONAR_UNAVAILABLE","原地转底层门缺少前方有效新鲜超声，改查其他动作"},
 {"FPGA_PIVOT_LEFT_SONAR_UNAVAILABLE","原地转底层门缺少左侧有效新鲜超声，改查其他动作"},
 {"FPGA_PIVOT_RIGHT_SONAR_UNAVAILABLE","原地转底层门缺少右侧有效新鲜超声，改查其他动作"},
 {"FPGA_PIVOT_NEAR_CONFIRMATION_PENDING","原地转涉及的超声近障仍在确认，先检查可行替代动作"},
 {"FPGA_PIVOT_FRONT_UNDER_30MM","原地转底层门要求前方至少30毫米，当前距离不足"},
 {"FPGA_PIVOT_LEFT_UNDER_30MM","原地转底层门要求左侧至少30毫米，当前距离不足"},
 {"FPGA_PIVOT_RIGHT_UNDER_30MM","原地转底层门要求右侧至少30毫米，当前距离不足"},
 {"FPGA_FORWARD_FRONT_SONAR_UNAVAILABLE","前进底层门缺少前方有效新鲜超声，改查后退或其他动作"},
 {"FPGA_FRONT_NEAR_CONFIRMATION_PENDING","前方近障正在确认，当前前移动作未获底层许可"},
 {"FPGA_FRONT_UNDER_100MM","历史版本原因：前方不足100毫米，底层禁止继续前移，检查脱离动作"},
 {"FPGA_FRONT_NEAR_LATCH_RELEASE_150MM","历史版本原因：前方近障锁存尚未释放，底层仍需两笔至少150毫米的新样本"},
 {"FPGA_TURN_LEFT_SONAR_UNAVAILABLE","左转涉及的左侧超声不可用，改查其他候选"},
 {"FPGA_TURN_RIGHT_SONAR_UNAVAILABLE","右转涉及的右侧超声不可用，改查其他候选"},
 {"FPGA_TURN_LEFT_NEAR_PENDING","左侧近障确认未完成，当前左转候选未获底层许可"},
 {"FPGA_TURN_RIGHT_NEAR_PENDING","右侧近障确认未完成，当前右转候选未获底层许可"},
 {"FPGA_TURN_LEFT_UNDER_30MM","左转涉及的左侧不足30毫米，改查可行离障动作"},
 {"FPGA_TURN_RIGHT_UNDER_30MM","右转涉及的右侧不足30毫米，改查可行离障动作"},
 {"FPGA_MIXED_TURN_NEEDS_BOTH_SIDES_30MM","左右轮反向的弧线仍需双侧有效且至少30毫米，改查其他动作"},
 {"FPGA_DIRECTION_GATE_REJECTED","候选未通过FPGA方向门，保留目标并继续选择动作"},
 {"FRONT_DIRECTION_UNAVAILABLE","前方缺少新鲜可信观测，不批准前移，继续检查其他动作"},
 {"FRONT_DIRECTION_UNAVAILABLE_OR_UNDER_100MM","历史版本原因：前方观测不可用或净空不足100毫米，改查可行后退和转向"},
 {"PARTIAL_DIRECTION_NO_ACTIVE_TASK","部分方向观测待更新，当前没有活动导航或巡航任务"},
 {"UNEXPECTED_LEGACY_OWNER_WAIT","控制归属异常，保留目标等待新状态，不切回旧规则"},
 {"SCENE_POSE_INVALID_WAIT_NEW_DATA","运动位姿失效，保留目标等待新位姿，不切回旧规则"},
 {"SCENE_CONFIRMING_UNIFIED_RULES","场景标签确认中，继续按当前净空逐个检查动作"},
 {"GENERAL_OBSERVED_ENVIRONMENT","通用已观测环境，按候选动作检查净空和完整车身扫掠"},
 {"PARTIAL_DIRECTION_FORWARD_CLEAR","部分方向待更新，当前直行的必要观测和车身扫掠已通过"},
 {"PARTIAL_DIRECTION_AWAY_ARC_CLEAR","部分方向待更新，执行已通过扫掠和底盘方向门的离障弧线"},
 {"PARTIAL_DIRECTION_FRONT_UNAVAILABLE_CHECK_REAR","前方观测不可用，不盲目前移，改查已观测后退路径"},
 {"DIRECTION_FRONT_UNAVAILABLE_WAIT_NEW_DATA","前方观测不可用，暂未找到可行替代动作，保留目标等新数据"},
 {"DIRECTION_LEFT_UNAVAILABLE_NO_VERIFIED_ACTION","左侧观测不可用，其他候选尚未通过必要观测或车身扫掠"},
 {"DIRECTION_RIGHT_UNAVAILABLE_NO_VERIFIED_ACTION","右侧观测不可用，其他候选尚未通过必要观测或车身扫掠"},
 {"DIRECTION_SIDES_UNAVAILABLE_NO_VERIFIED_ACTION","左右观测待更新，当前未找到通过终检的动作，目标保留"},
 {"DIRECTION_ALL_UNAVAILABLE_WAIT_NEW_DATA","各方向均缺少可用观测，保留目标等待新数据，不把未知当空地"},
 {"PARTIAL_DIRECTION_REAR_UNAVAILABLE_NO_VERIFIED_ACTION","后方观测不可用，不盲退；其他方向也暂未找到可行动作"},
 {"WHEEL_TARGET_LIMIT","候选左右轮速超过当前轮速或动作上限，改选保持曲率的可执行速度"},
 {"FRONT_100_150_EXPLORE_CORRIDOR_ONLY","历史版本原因：前方100至150毫米，只接受已观测走廊内的直线前探"},
 {"FRONT_BODY_SWEEP","该动作车身扫掠将接近前方障碍，继续选择其他动作"},
 {"FRONT_UNDER_100_REVERSE_OR_TURN_ONLY","历史版本原因：前方不足100毫米，不继续前移，改查后退或转向"},
 {"ROUTE_ALTERNATIVE_ARC","原路线动作未通过检查，执行已验证的备用弧线"},
 {"SCENE_NO_REASON","本轮场景未提供原因码，等待下一笔诊断"},
 {"SCENE_SPEED_BAND","该前进候选超过当前前方净空速度档，改选较低速度"},
 {"SIDE_ACTION_SPEED_ENVELOPE","该候选会接近侧方障碍，速度超过本动作的侧方净空档"},
 {"SIDE_BODY_SWEEP","该动作的车角或车身侧面将接近障碍，继续选择其他动作"},
 {"AVOID_TURN_DONE_WAIT_TWO_NEW_100MM","历史版本原因：避障转向完成，停稳等待两笔前方至少100毫米的新观测"},
 {"EXISTING_COMMITTED_ACTION","正在完成已经选定的动作，暂不切换方向"},
 {"EXIT_TURN_80DEG","历史版本原因：后退后向出口转向约80度"},
 {"EXIT_TURN_BOTH_DIRECTIONS_BLOCKED_WAIT_NEW_DATA","两侧出口转向均未通过扫掠，等待新观测后重选"},
 {"EXIT_TURN_DONE_WAIT_TWO_NEW_100MM","历史版本原因：出口转向完成，停稳等待两笔前方至少100毫米的新观测"},
 {"EXIT_TURN_NO_SWEPT_CLEAR_SIDE","出口转向尚无扫掠通过的方向，继续观察"},
 {"EXIT_TURN_NO_SWEPT_CLEAR_SIDE_WAIT_NEW_DATA","出口转向暂无扫掠通过的方向，等待新观测再检查"},
 {"EXIT_TURN_RESELECT_OPPOSITE","出口转向受阻，完整扫掠已通过，改向另一侧转"},
 {"EXPLICIT_STOP_OR_FAULT","用户停止或实际故障已接管运动输出"},
 {"EXPLORE_CORRIDOR_ONLY_018","历史版本原因：前方100至150毫米且两侧没有开口，按走廊规则前探"},
 {"EXPLORE_DISABLED_NO_ACTIVE_TASK","没有活动导航或巡航任务，结束本轮自主探路"},
 {"EXPLORE_DISABLED_NO_GOAL","没有导航目标，当前按巡航避障规则处理"},
 {"EXPLORE_EXIT_FOUND_RECLASSIFY_TARGET_PRESERVED","发现出口，保留目标并恢复导航"},
 {"EXPLORE_FORWARD_020","前方观测支持以0.20米每秒前探"},
 {"EXPLORE_FORWARD_REJECTED_TRY_REAR","前探未通过检查，转为检查后退空间"},
 {"EXPLORE_FORWARD_REJECTED_TRY_TURN","前探未通过检查，改向已通过完整扫掠的一侧探转"},
 {"EXPLORE_POSE_INVALID","当前位姿不可用，无法核对转角或后退位移"},
 {"EXPLORE_ROUND_10S_50MM_LIMIT","本轮探索已达10秒且净位移不足50毫米，改查后退或新方向"},
 {"EXPLORE_ROUND_BUDGET_EXHAUSTED","本轮探索边界已到，重新检查可行动作"},
 {"EXPLORE_SIDE_BUDGET_OR_NO_OPENING","本轮已检查两侧或暂无出口，改查后退空间"},
 {"EXPLORE_SIDE_SWITCH_HYSTERESIS","暂时保持探路侧，避免左右频繁换向"},
 {"EXPLORE_TURN","向已选侧探转，观察新的出口"},
 {"EXPLORE_TURN_DONE_MEASURE_FRONT","探转完成，重新测量前方空间"},
 {"FRONT_100_150_CORRIDOR_ONLY_018","历史版本原因：前方100至150毫米且两侧没有开口，按走廊规则前探"},
 {"FRONT_100_150_NO_SWEPT_OPEN_SIDE","历史版本原因：前方100至150毫米，尚无通过转向扫掠的侧向出口"},
 {"FRONT_50MM_BRAKE","历史版本原因：前方新鲜非零净空不超过50毫米，立即刹车复核"},
 {"FRONT_50_100_CHECK_REAR_FIRST","历史版本原因：前方50至100毫米，停止前移并检查后退空间"},
 {"FRONT_NEAR_RULE_MOTION","前方进入近障区，按净空分档并寻找可行方向"},
 {"GLASS_WALL_REQUIRES_OBSERVATION_CONFIRMATION","疑似玻璃：超声仍见近障，不能把雷达空洞当通道"},
 {"HOLD_OBSERVE_NEW_DATA_CONTINUOUS_TASK","当前没有通过检查的动作，任务保留并等待新观测"},
 {"HOLD_RETREAT_FULL_PATH_OR_BRAKE_MARGIN","后退路径或制动余量不足，继续选择可行动作"},
 {"HOLD_RETREAT_REAR_UNOBSERVED_OR_NEAR","后方缺少新鲜观测或净空不足，继续观察新空间"},
 {"LEFT_EMERGENCY","左侧进入当前动作的紧急净空范围"},
 {"MANUAL_UNCHANGED_BYPASS","手动操作，场景机不接管方向命令"},
 {"NARROW_EXACT_STRAIGHT_FPGA_30MM_GATE","侧方不足30毫米，保持精确直行以满足底盘方向门"},
 {"NARROW_RECTANGLE_MODEL","直行采用车身矩形净空，转弯仍需独立扫掠检查"},
 {"NEW_OBSERVATION_RETREAT_RETRY","已获得新方向观测，重新检查后退与出口动作"},
 {"NO_FEASIBLE_ACTION_TARGET_PRESERVED","本轮没有通过检查的动作，目标保留并继续找空间"},
 {"PLANNER_ACCEPTED","局部轨迹已通过终检，正在执行"},
 {"PROBE_TURN_BOTH_DIRECTIONS_BLOCKED_WAIT_NEW_DATA","两侧探转均未通过扫掠，等待新观测后重选"},
 {"PROBE_TURN_RESELECT_OPPOSITE","探转方向受阻，完整扫掠已通过，改向另一侧观察"},
 {"RANKED_RETRY_ACCEPTED","局部轨迹已通过终检，正在执行"},
 {"RETREAT_ATTEMPT_BUDGET_EXHAUSTED","本轮后退边界已到，重新检查可行方向"},
 {"RETREAT_BRAKE_BEFORE_EXIT_TURN","本次后退到位，减速后向出口转向"},
 {"RETREAT_DISTANCE_038M","历史版本原因：执行已验证的本次后退目标，常规目标上限0.38米，持续检查后方"},
 {"RETREAT_EXTENDED_DISTANCE_055M","历史版本原因：执行已验证的扩展后退目标，目标上限0.55米，持续检查后方"},
 {"RETREAT_POSE_INVALID","当前位姿不可用，无法核对转角或后退位移"},
 {"RETREAT_REAR_CHANGED_RESELECT_EXIT","后方空间变化，结束本次后退并重选动作"},
 {"RIGHT_EMERGENCY","右侧进入当前动作的紧急净空范围"},
 {"SCENE_CONFIRMATION_PENDING_USE_S5","新场景尚未持续确认，暂按原规则处理"},
 {"SCENE_DATA_UNCERTAIN_USE_S5","本轮方向观测矛盾或失效，暂按原规则处理"},
 {"SCENE_FORWARD_CAP_ZERO","当前场景不允许前移，改查转向或后退空间"},
 {"SCENE_INPUT_INVALID","场景输入缺失或含无效数值"},
 {"SCENE_POSE_INVALID_USE_S5","当前位姿不可用，无法核对转角或后退位移"},
 {"SCENE_ROUTE_ALLOWED","当前场景允许沿路线行驶，近场保护持续检查"},
 {"SCENE_SWEEP_REJECTED","候选动作未通过完整车身扫掠，继续选择其他方向"},
 {"SCENE_TIME_REVERSED","场景输入时间倒退，拒绝使用这笔旧数据"},
 {"SIDE_AWAY_CHECKED_ESCAPE","侧方近障已停稳，完整扫掠通过，执行离障动作"},
 {"SINGLE_SIDE_SAMPLE_CRAWL_CONFIRM","仅一笔侧方近障，按当前动作减速确认，不锁定停车"},
 {"SPECIAL_OBSTACLE_FRONT_RULE","按近障规则处理低矮或悬空结构，不直接钻行"},
 {"TRANSITION_BRAKE_AFTER_PROBE_TURN","探转结束，先降低角速度再切回前探"},
 {"TRANSITION_BRAKE_BEFORE_EXIT_TURN","后退结束，先停稳再向出口转向"},
 {"TRANSITION_BRAKE_BEFORE_REVERSE","先停稳再切换到后退方向"},
 {"TRANSITION_BRAKE_BEFORE_TURN","先停稳再切换到转向动作"},
 {"TURN_HOLD_FRONT_NEAR_BRAKE_BEFORE_REAR_CHECK","历史版本原因：转向后前方不足100毫米，先停稳再检查后方"},
 {"TURN_HOLD_FRONT_UNDER_100_RECHECK_REAR","历史版本原因：转向后前方不足100毫米，改查后退空间，不原地死等"},
 {"TURN_HOLD_RELEASED_RECLASSIFY","两笔前方新样本已通过，重新选择行驶场景"},
 {"TURN_HOLD_WAIT_TWO_NEW_100MM","历史版本原因：转向后停稳，等待两笔不同的新样本且前方均至少100毫米"},
 {"TURN_REJECTED_BRAKE_BEFORE_RESELECT","当前转向被新障碍否决，先停稳再检查相反方向"},
};
static inline const char*r4_reason_zh(const char*r){
 if(!r||!*r)return "等待本轮场景诊断";
 for(unsigned i=0;i<sizeof(r4_reason_labels)/sizeof(r4_reason_labels[0]);i++)
  if(!strcmp(r,r4_reason_labels[i].code))return r4_reason_labels[i].zh;
 if(strstr(r,"SIDE_SINGLE")||strstr(r,"SIDE_FIRST"))return "单笔侧方近障，减速等待另一笔新观测";
 if(strstr(r,"TWO_NEW_100MM"))return "转向后等待两笔不同的新样本，前方均至少100毫米";
 if(strstr(r,"TRANSITION_BRAKE"))return "先降低当前速度，停稳后切换动作方向";
 if(strstr(r,"100_150_CORRIDOR"))return "前方100至150毫米且两侧没有开口，按走廊规则前探";
 if(strstr(r,"EXPLORE_ROUND")||strstr(r,"EXPLORE_SIDE_BUDGET"))return "本轮探转已完成，继续检查另一方向或后退空间";
 if(strstr(r,"DYNAMIC_WINDOW"))return "本轮固定轨迹采样没有合适过渡，交给场景规则衔接";
 if(strstr(r,"COSTMAP")&&strstr(r,"STALE"))return "局部成本图源扫描超过300毫秒，暂用场景规则";
 return "具体限制见原因代码";
}
#endif
