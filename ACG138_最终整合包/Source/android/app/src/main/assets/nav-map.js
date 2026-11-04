'use strict';
// R6_POLICY_BEGIN:由policy/r6-motion.json生成，仅用于显示。
window.ACGR6MotionPolicy=Object.freeze({"config_id":823198465,"config_hex":"0x31110301","software_policy":9,"status_revision":5,"command_version":3,"front":{"front_observe_mm":100,"front_near_mm":50,"front_forward_stop_mm":30,"front_emergency_mm":20,"front_release_mm":40,"front_release_samples":2}});
// R6_POLICY_END
// R4_REASON_LABELS_BEGIN:由r4_status_text.h生成，仅用于显示。
window.ACGR4ReasonLabels=Object.freeze({"R8_OBSERVE_BRAKE_BEFORE_CHANGE":"缺少该动作的空间证据，先停稳再换观察动作","R8_OBSERVE_ROUTE_SIDE_TURN":"沿路线期望侧小角度转向，获取新的空间观测","R8_OBSERVE_OTHER_SIDE_TURN":"原侧观察动作未通过，向另一已验证方向小角度观察","R8_OBSERVE_ROUTE_SIDE_ARC":"沿路线期望侧执行已检查的短弧线，换视角补充观测","R8_OBSERVE_OTHER_SIDE_ARC":"沿另一已检查短弧线换视角，导航目标保持","R8_OBSERVE_SHORT_REVERSE":"其他观察动作暂不可行，沿已检查后方短退后重看路线","R8_OBSERVE_SEGMENT_DONE_RESCAN":"本段观察动作到达边界，等待新扫描重新判断路线","R8_OBSERVE_RECHECK_NEW_SCAN":"已收到新扫描，重新检查原路线与可行观察动作","R8_OBSERVE_NO_VERIFIED_ACTION":"当前各观察动作都缺少通过证据，保留目标等待新观测","R8_OBSERVE_WAIT_FRESH_SOURCES":"观察动作所需雷达、位姿或轮速过期，等待对应新数据","R8_OBSERVE_COMPUTE_RETRY":"观察动作检查预算不足，保留目标与阶段后重试","INITIAL_BUFFER_NOT_CLEARING":"已进入车身缓冲区，该动作未持续拉开距离，改查其他脱离动作","FRONT_20MM_BRAKE":"前方新鲜非零净空不超过20毫米，立即刹车后重新检查脱离动作","R6_ACTION_CHECK_INVALID":"候选动作参数或检查上下文无效，保留目标等待下一轮","R6_ACTION_HARD_GATE_REJECTED":"候选未获底盘方向门许可，继续核对具体门限和替代动作","R6_ACTION_OCCUPIED":"完整车身扫掠确认该候选涉及实际障碍，检查其他动作","R6_ACTION_POSE_INVALID":"当前位姿不可用，无法完成动作扫掠与位移核对","R6_ACTION_SOURCE_STALE":"该动作依赖的观测源过期，保留路线等待新观测","R6_ACTION_UNOBSERVED":"该动作涉及区域尚未观测，不能当成空地直接驶入","R6_BRAKE_BEFORE_DIRECTION_CHANGE":"先降低当前运动，停稳后再切换实际不可行的方向","R6_BRAKE_BEFORE_SHORT_RETREAT":"前进或转向暂不可行，先停稳再执行已检查的短段后退","R6_BRAKING_SPEED_LIMIT_WAIT":"当前制动余量不足，沿原路线减速并等待复核","R6_COMPUTE_BUDGET_WAIT":"本轮计算预算不足，保持路线与恢复阶段，下一轮重试","R6_FORWARD_TURNS_UNAVAILABLE_CHECK_SHORT_REAR":"前进和两侧转向均未通过检查，改查短段后退路径","R6_NEAR_ROUTE_FIRST":"前方进入50毫米近障区，仍先检查原路线的低速可行动作","R6_NO_ACTIVE_TASK_NO_RECOVERY":"当前没有活动导航或巡航任务，不自行开始脱离动作","R6_NO_VERIFIED_ACTION_TARGET_PRESERVED":"当前没有经过验证的动作，保留目标等待新观测后重试","R6_OBSERVE_ROUTE_FIRST":"前方进入100毫米观察区，优先沿路线按制动余量减速","R6_REAR_PATH_NOT_AVAILABLE":"后方不足以完成已检查的短退路径，继续观察和核对路线","R6_RETREAT_SEGMENT_DONE_REASSESS_ROUTE":"本次短退已到距离或时间边界，停稳并重新核对原路线","R6_ROUTE_CLEAR_BRAKE_TO_REJOIN":"原路线两笔新观测已证明可行，先刹稳再沿路线恢复","R6_ROUTE_CLEAR_WAIT_TWO_NEW":"原路线候选已可行，等待第二笔新观测确认恢复","R6_ROUTE_RECOVERED_NO_FIXED_TURN":"原路线恢复可行，直接恢复跟踪，不执行固定角度退出转向","R6_ROUTE_SIDE_TURN":"按路线期望侧执行已检查的转向，持续重新核对路线","R6_SHORT_RETREAT_012M":"执行已检查的短段后退，目标120毫米，硬限150毫米或1.2秒","R6_TURN_TIME_REASSESS_WAIT_NEW":"本段转向到达角度或时间边界，停稳后用新观测重新核对路线","COMPUTE_BUDGET_RETRY_SAME_ROUTE":"计算预算不足，保留原路线下一轮重试，不据此换侧或后退","MEASURED_MOTION_BRAKE_ROUTE_RETAINED":"当前实际运动的制动空间不足，先刹车并保留原路线","CURRENT_BRAKING_CHECK_WAIT":"当前制动路径尚不能完成检查，等待必要数据或计算预算","ROUTE_CURVATURE_PRESERVED_SLOWER":"原路线较低速度已通过检查，保持转向方向减速行驶","ROUTE_PREFERRED_CLEAR":"原路线动作通过检查，按预览方向执行","ROUTE_SIDE_REJECTED_OTHER_SIDE_CLEAR":"原路线侧动作被确认不可行，另一侧完整检查通过后改向","ROUTE_SIDE_ALTERNATIVE_CLEAR":"优先侧的替代动作已通过检查，保持路线侧行驶","ROUTE_BLOCKED_SCENE_ESCAPE_CHECKED":"原路线动作受阻，执行场景机已验证的脱离动作","NEARFIELD_SCAN_UNAVAILABLE":"当前缺少可用近场扫描，不能核对该候选的完整车身扫掠","SONAR_BODY_SWEEP":"超声观测点与该候选的车身扫掠相交，继续检查其他动作","ACTION_INPUT_INVALID":"当前候选参数或检查输入无效，保留任务等待下一轮","FRONT_UNDER_30_REVERSE_OR_TURN_ONLY":"前方不足30毫米，不继续前移，按实际净空检查转向或短退","FPGA_FRONT_UNDER_30MM":"前方不足30毫米，底盘不批准前移；继续检查可行脱离动作","FPGA_FRONT_NEAR_LATCH_RELEASE_40MM":"前向近障锁存尚未释放，等待两笔至少40毫米的新观测","PARTIAL_DIRECTION_WAIT_NEW_OBSERVATION":"必要方向观测不足，保留目标等待新观测后重新选择动作","R5_UNIFIED_ACTION_RESELECT":"统一控制器重新选择动作，不继承旧动作的有限重试预算","FPGA_STATUS_MISSING":"尚未收到FPGA状态，无法核对当前动作的底层许可","FPGA_STATUS_AGE_OVER_250MS":"FPGA状态超过250毫秒，等待新状态后再检查动作","FPGA_INTENT_WHEEL_SHAPE_MISMATCH":"候选动作类别与实际左右轮速组合不一致，拒绝本候选","FPGA_REVERSE_LEFT_SONAR_UNAVAILABLE":"倒车底层门要求左侧超声有效新鲜，当前尚不满足","FPGA_REVERSE_RIGHT_SONAR_UNAVAILABLE":"倒车底层门要求右侧超声有效新鲜，当前尚不满足","FPGA_PIVOT_FRONT_SONAR_UNAVAILABLE":"原地转底层门缺少前方有效新鲜超声，改查其他动作","FPGA_PIVOT_LEFT_SONAR_UNAVAILABLE":"原地转底层门缺少左侧有效新鲜超声，改查其他动作","FPGA_PIVOT_RIGHT_SONAR_UNAVAILABLE":"原地转底层门缺少右侧有效新鲜超声，改查其他动作","FPGA_PIVOT_NEAR_CONFIRMATION_PENDING":"原地转涉及的超声近障仍在确认，先检查可行替代动作","FPGA_PIVOT_FRONT_UNDER_30MM":"原地转底层门要求前方至少30毫米，当前距离不足","FPGA_PIVOT_LEFT_UNDER_30MM":"原地转底层门要求左侧至少30毫米，当前距离不足","FPGA_PIVOT_RIGHT_UNDER_30MM":"原地转底层门要求右侧至少30毫米，当前距离不足","FPGA_FORWARD_FRONT_SONAR_UNAVAILABLE":"前进底层门缺少前方有效新鲜超声，改查后退或其他动作","FPGA_FRONT_NEAR_CONFIRMATION_PENDING":"前方近障正在确认，当前前移动作未获底层许可","FPGA_FRONT_UNDER_100MM":"历史版本原因：前方不足100毫米，底层禁止继续前移，检查脱离动作","FPGA_FRONT_NEAR_LATCH_RELEASE_150MM":"历史版本原因：前方近障锁存尚未释放，底层仍需两笔至少150毫米的新样本","FPGA_TURN_LEFT_SONAR_UNAVAILABLE":"左转涉及的左侧超声不可用，改查其他候选","FPGA_TURN_RIGHT_SONAR_UNAVAILABLE":"右转涉及的右侧超声不可用，改查其他候选","FPGA_TURN_LEFT_NEAR_PENDING":"左侧近障确认未完成，当前左转候选未获底层许可","FPGA_TURN_RIGHT_NEAR_PENDING":"右侧近障确认未完成，当前右转候选未获底层许可","FPGA_TURN_LEFT_UNDER_30MM":"左转涉及的左侧不足30毫米，改查可行离障动作","FPGA_TURN_RIGHT_UNDER_30MM":"右转涉及的右侧不足30毫米，改查可行离障动作","FPGA_MIXED_TURN_NEEDS_BOTH_SIDES_30MM":"左右轮反向的弧线仍需双侧有效且至少30毫米，改查其他动作","FPGA_DIRECTION_GATE_REJECTED":"候选未通过FPGA方向门，保留目标并继续选择动作","FRONT_DIRECTION_UNAVAILABLE":"前方缺少新鲜可信观测，不批准前移，继续检查其他动作","FRONT_DIRECTION_UNAVAILABLE_OR_UNDER_100MM":"历史版本原因：前方观测不可用或净空不足100毫米，改查可行后退和转向","PARTIAL_DIRECTION_NO_ACTIVE_TASK":"部分方向观测待更新，当前没有活动导航或巡航任务","UNEXPECTED_LEGACY_OWNER_WAIT":"控制归属异常，保留目标等待新状态，不切回旧规则","SCENE_POSE_INVALID_WAIT_NEW_DATA":"运动位姿失效，保留目标等待新位姿，不切回旧规则","SCENE_CONFIRMING_UNIFIED_RULES":"场景标签确认中，继续按当前净空逐个检查动作","GENERAL_OBSERVED_ENVIRONMENT":"通用已观测环境，按候选动作检查净空和完整车身扫掠","PARTIAL_DIRECTION_FORWARD_CLEAR":"部分方向待更新，当前直行的必要观测和车身扫掠已通过","PARTIAL_DIRECTION_AWAY_ARC_CLEAR":"部分方向待更新，执行已通过扫掠和底盘方向门的离障弧线","PARTIAL_DIRECTION_FRONT_UNAVAILABLE_CHECK_REAR":"前方观测不可用，不盲目前移，改查已观测后退路径","DIRECTION_FRONT_UNAVAILABLE_WAIT_NEW_DATA":"前方观测不可用，暂未找到可行替代动作，保留目标等新数据","DIRECTION_LEFT_UNAVAILABLE_NO_VERIFIED_ACTION":"左侧观测不可用，其他候选尚未通过必要观测或车身扫掠","DIRECTION_RIGHT_UNAVAILABLE_NO_VERIFIED_ACTION":"右侧观测不可用，其他候选尚未通过必要观测或车身扫掠","DIRECTION_SIDES_UNAVAILABLE_NO_VERIFIED_ACTION":"左右观测待更新，当前未找到通过终检的动作，目标保留","DIRECTION_ALL_UNAVAILABLE_WAIT_NEW_DATA":"各方向均缺少可用观测，保留目标等待新数据，不把未知当空地","PARTIAL_DIRECTION_REAR_UNAVAILABLE_NO_VERIFIED_ACTION":"后方观测不可用，不盲退；其他方向也暂未找到可行动作","WHEEL_TARGET_LIMIT":"候选左右轮速超过当前轮速或动作上限，改选保持曲率的可执行速度","FRONT_100_150_EXPLORE_CORRIDOR_ONLY":"历史版本原因：前方100至150毫米，只接受已观测走廊内的直线前探","FRONT_BODY_SWEEP":"该动作车身扫掠将接近前方障碍，继续选择其他动作","FRONT_UNDER_100_REVERSE_OR_TURN_ONLY":"历史版本原因：前方不足100毫米，不继续前移，改查后退或转向","ROUTE_ALTERNATIVE_ARC":"原路线动作未通过检查，执行已验证的备用弧线","SCENE_NO_REASON":"本轮场景未提供原因码，等待下一笔诊断","SCENE_SPEED_BAND":"该前进候选超过当前前方净空速度档，改选较低速度","SIDE_ACTION_SPEED_ENVELOPE":"该候选会接近侧方障碍，速度超过本动作的侧方净空档","SIDE_BODY_SWEEP":"该动作的车角或车身侧面将接近障碍，继续选择其他动作","AVOID_TURN_DONE_WAIT_TWO_NEW_100MM":"历史版本原因：避障转向完成，停稳等待两笔前方至少100毫米的新观测","EXISTING_COMMITTED_ACTION":"正在完成已经选定的动作，暂不切换方向","EXIT_TURN_80DEG":"历史版本原因：后退后向出口转向约80度","EXIT_TURN_BOTH_DIRECTIONS_BLOCKED_WAIT_NEW_DATA":"两侧出口转向均未通过扫掠，等待新观测后重选","EXIT_TURN_DONE_WAIT_TWO_NEW_100MM":"历史版本原因：出口转向完成，停稳等待两笔前方至少100毫米的新观测","EXIT_TURN_NO_SWEPT_CLEAR_SIDE":"出口转向尚无扫掠通过的方向，继续观察","EXIT_TURN_NO_SWEPT_CLEAR_SIDE_WAIT_NEW_DATA":"出口转向暂无扫掠通过的方向，等待新观测再检查","EXIT_TURN_RESELECT_OPPOSITE":"出口转向受阻，完整扫掠已通过，改向另一侧转","EXPLICIT_STOP_OR_FAULT":"用户停止或实际故障已接管运动输出","EXPLORE_CORRIDOR_ONLY_018":"历史版本原因：前方100至150毫米且两侧没有开口，按走廊规则前探","EXPLORE_DISABLED_NO_ACTIVE_TASK":"没有活动导航或巡航任务，结束本轮自主探路","EXPLORE_DISABLED_NO_GOAL":"没有导航目标，当前按巡航避障规则处理","EXPLORE_EXIT_FOUND_RECLASSIFY_TARGET_PRESERVED":"发现出口，保留目标并恢复导航","EXPLORE_FORWARD_020":"前方观测支持以0.20米每秒前探","EXPLORE_FORWARD_REJECTED_TRY_REAR":"前探未通过检查，转为检查后退空间","EXPLORE_FORWARD_REJECTED_TRY_TURN":"前探未通过检查，改向已通过完整扫掠的一侧探转","EXPLORE_POSE_INVALID":"当前位姿不可用，无法核对转角或后退位移","EXPLORE_ROUND_10S_50MM_LIMIT":"本轮探索已达10秒且净位移不足50毫米，改查后退或新方向","EXPLORE_ROUND_BUDGET_EXHAUSTED":"本轮探索边界已到，重新检查可行动作","EXPLORE_SIDE_BUDGET_OR_NO_OPENING":"本轮已检查两侧或暂无出口，改查后退空间","EXPLORE_SIDE_SWITCH_HYSTERESIS":"暂时保持探路侧，避免左右频繁换向","EXPLORE_TURN":"向已选侧探转，观察新的出口","EXPLORE_TURN_DONE_MEASURE_FRONT":"探转完成，重新测量前方空间","FRONT_100_150_CORRIDOR_ONLY_018":"历史版本原因：前方100至150毫米且两侧没有开口，按走廊规则前探","FRONT_100_150_NO_SWEPT_OPEN_SIDE":"历史版本原因：前方100至150毫米，尚无通过转向扫掠的侧向出口","FRONT_50MM_BRAKE":"历史版本原因：前方新鲜非零净空不超过50毫米，立即刹车复核","FRONT_50_100_CHECK_REAR_FIRST":"历史版本原因：前方50至100毫米，停止前移并检查后退空间","FRONT_NEAR_RULE_MOTION":"前方进入近障区，按净空分档并寻找可行方向","GLASS_WALL_REQUIRES_OBSERVATION_CONFIRMATION":"疑似玻璃：超声仍见近障，不能把雷达空洞当通道","HOLD_OBSERVE_NEW_DATA_CONTINUOUS_TASK":"当前没有通过检查的动作，任务保留并等待新观测","HOLD_RETREAT_FULL_PATH_OR_BRAKE_MARGIN":"后退路径或制动余量不足，继续选择可行动作","HOLD_RETREAT_REAR_UNOBSERVED_OR_NEAR":"后方缺少新鲜观测或净空不足，继续观察新空间","LEFT_EMERGENCY":"左侧进入当前动作的紧急净空范围","MANUAL_UNCHANGED_BYPASS":"手动操作，场景机不接管方向命令","NARROW_EXACT_STRAIGHT_FPGA_30MM_GATE":"侧方不足30毫米，保持精确直行以满足底盘方向门","NARROW_RECTANGLE_MODEL":"直行采用车身矩形净空，转弯仍需独立扫掠检查","NEW_OBSERVATION_RETREAT_RETRY":"已获得新方向观测，重新检查后退与出口动作","NO_FEASIBLE_ACTION_TARGET_PRESERVED":"本轮没有通过检查的动作，目标保留并继续找空间","PLANNER_ACCEPTED":"局部轨迹已通过终检，正在执行","PROBE_TURN_BOTH_DIRECTIONS_BLOCKED_WAIT_NEW_DATA":"两侧探转均未通过扫掠，等待新观测后重选","PROBE_TURN_RESELECT_OPPOSITE":"探转方向受阻，完整扫掠已通过，改向另一侧观察","RANKED_RETRY_ACCEPTED":"局部轨迹已通过终检，正在执行","RETREAT_ATTEMPT_BUDGET_EXHAUSTED":"本轮后退边界已到，重新检查可行方向","RETREAT_BRAKE_BEFORE_EXIT_TURN":"本次后退到位，减速后向出口转向","RETREAT_DISTANCE_038M":"历史版本原因：执行已验证的本次后退目标，常规目标上限0.38米，持续检查后方","RETREAT_EXTENDED_DISTANCE_055M":"历史版本原因：执行已验证的扩展后退目标，目标上限0.55米，持续检查后方","RETREAT_POSE_INVALID":"当前位姿不可用，无法核对转角或后退位移","RETREAT_REAR_CHANGED_RESELECT_EXIT":"后方空间变化，结束本次后退并重选动作","RIGHT_EMERGENCY":"右侧进入当前动作的紧急净空范围","SCENE_CONFIRMATION_PENDING_USE_S5":"新场景尚未持续确认，暂按原规则处理","SCENE_DATA_UNCERTAIN_USE_S5":"本轮方向观测矛盾或失效，暂按原规则处理","SCENE_FORWARD_CAP_ZERO":"当前场景不允许前移，改查转向或后退空间","SCENE_INPUT_INVALID":"场景输入缺失或含无效数值","SCENE_POSE_INVALID_USE_S5":"当前位姿不可用，无法核对转角或后退位移","SCENE_ROUTE_ALLOWED":"当前场景允许沿路线行驶，近场保护持续检查","SCENE_SWEEP_REJECTED":"候选动作未通过完整车身扫掠，继续选择其他方向","SCENE_TIME_REVERSED":"场景输入时间倒退，拒绝使用这笔旧数据","SIDE_AWAY_CHECKED_ESCAPE":"侧方近障已停稳，完整扫掠通过，执行离障动作","SINGLE_SIDE_SAMPLE_CRAWL_CONFIRM":"仅一笔侧方近障，按当前动作减速确认，不锁定停车","SPECIAL_OBSTACLE_FRONT_RULE":"按近障规则处理低矮或悬空结构，不直接钻行","TRANSITION_BRAKE_AFTER_PROBE_TURN":"探转结束，先降低角速度再切回前探","TRANSITION_BRAKE_BEFORE_EXIT_TURN":"后退结束，先停稳再向出口转向","TRANSITION_BRAKE_BEFORE_REVERSE":"先停稳再切换到后退方向","TRANSITION_BRAKE_BEFORE_TURN":"先停稳再切换到转向动作","TURN_HOLD_FRONT_NEAR_BRAKE_BEFORE_REAR_CHECK":"历史版本原因：转向后前方不足100毫米，先停稳再检查后方","TURN_HOLD_FRONT_UNDER_100_RECHECK_REAR":"历史版本原因：转向后前方不足100毫米，改查后退空间，不原地死等","TURN_HOLD_RELEASED_RECLASSIFY":"两笔前方新样本已通过，重新选择行驶场景","TURN_HOLD_WAIT_TWO_NEW_100MM":"历史版本原因：转向后停稳，等待两笔不同的新样本且前方均至少100毫米","TURN_REJECTED_BRAKE_BEFORE_RESELECT":"当前转向被新障碍否决，先停稳再检查相反方向"});
window.ACGR4ReasonText=code=>window.ACGR4ReasonLabels[code]||'';
// R4_REASON_LABELS_END
// R4只读显示：不发送命令，不改变会话、心跳或遥控行为。
window.ACGR4SceneText=m=>{
 if(!m||Number(m.policy_revision)<7)return '';
 const scenes={0:'开阔道路',1:'前方近障',2:'左侧沿墙',3:'窄道居中',4:'拐角调整',5:'死胡同找出口',6:'门口直行',7:'低矮障碍',8:'疑似玻璃墙',9:'悬空结构区域',10:'后退腾挪',12:'右侧沿墙',13:'等待有效观测',14:'通用已观测环境'};
 const owners={0:'路线跟踪',1:'用户停止或故障',2:'紧急近障保护',3:'近场场景机',4:'完成已选动作',5:'历史兼容规则',6:'等待必要观测，目标保留'};
 const stages={0:'按场景行驶',1:'检查后方空间',2:'直线后退腾挪',3:'向出口转向',4:'停稳，等待两笔新样本至少40毫米',11:'比较两侧出口',12:'转向观察新空间',13:'向已观测空间前探',14:'发现出口，恢复目标导航',15:'本轮重新选择方向',16:'等待新的可行动作'};
 const known=typeof m.scene==='string'&&m.scene.length>0;
 const scene=known?(m.scene_zh||scenes[m.scene_id]||'场景待识别'):'等待本轮场景';
 const owner=m.scene_owner_zh||owners[m.scene_owner]||'控制归属待确认';
 const stage=m.scene_stage_zh||stages[m.scene_stage]||'动作阶段待确认';
 const localReason=window.ACGR4ReasonText(m.scene_reason);
 const reason=localReason||m.scene_reason_zh||'具体限制见原因代码';
 const cap=Number.isFinite(m.scene_speed_cap)?' · 场景前进上限'+m.scene_speed_cap.toFixed(2)+' m/s':'';
 const counter=Number.isInteger(m.explore_round)&&m.explore_round>0?' · 已探索'+m.explore_round+'轮':'';
 const code=m.scene_reason?' ['+String(m.scene_reason).slice(0,96)+']':'';
 const pending=m.scene_pending===true?' · 场景确认中，动作继续独立检查':'';
 const directions=window.ACGR5DirectionText(m);
 return '场景：'+scene+' · '+owner+' · '+stage+cap+counter+'\n'+reason+code+pending+(directions?'\n'+directions:'')+(m.scene_calibrated===false?' · 窄缝参数未经实车复核':'');
};
// 仅用于显示，某一方向缺失不作为全局运动许可条件。
window.ACGR5DirectionText=m=>{
 if(!m||!Number.isInteger(m.usable_direction_mask))return '';
 const names=['前','左','右','后'],mask=m.usable_direction_mask&15,ages=m.direction_age_ms;
 return names.map((name,i)=>name+((mask&(1<<i))?'可用':'待更新')+(Array.isArray(ages)&&Number.isFinite(ages[i])?' '+Math.max(0,ages[i]).toFixed(0)+'ms':'')).join('／');
};
window.ACGR6RouteText=m=>{
 if(!m)return '';
 const r=m.route_policy,c=m.action_check,p=m.front_policy,lines=[];
 const side=(s,v,w)=>v===0&&w===0?'停稳':s>0?'左转':s<0?'右转':v<0?'直退':'直行';
 if(r&&[-1,0,1].includes(r.expected_side)&&[-1,0,1].includes(r.actual_side)&&[r.expected_v,r.expected_w,r.actual_v,r.actual_w].every(Number.isFinite)){
  lines.push('路线期望：'+side(r.expected_side,r.expected_v,r.expected_w)+' · 本轮选定：'+side(r.actual_side,r.actual_v,r.actual_w));
  if(r.side_changed===true)lines.push('改向原因：'+(window.ACGR4ReasonText(r.change_reason)||r.change_reason_zh||r.change_reason||'尚未提供具体换侧原因'));
 }
 const f=m.live_feedback,hasLive=!!(f&&f.version===1),fresh=hasLive?f.fresh===true:m.feedback_fresh===true,wheels=hasLive?f.wheel_rpm:m.wheel_rpm;
 if(!fresh||!Array.isArray(wheels)||wheels.length!==4||!wheels.every(Number.isFinite))lines.push('轮速反馈：尚无新鲜完整数据，不能确认正在转动');
 else if(wheels.every(x=>Math.abs(x)<=1))lines.push('轮速反馈：四轮近零，尚未实际执行所选动作');
 else {const left=(wheels[0]+wheels[2])/2,right=(wheels[1]+wheels[3])/2,delta=right-left,band=Math.max(3,Math.abs((left+right)/2)*.1);lines.push('轮速反馈：'+(delta>band?'左转轮速组合':delta< -band?'右转轮速组合':left+right< -2?'倒车轮速组合':'前进轮速组合')+'（编码器反馈，非车身位移确认）');}
 if(c&&c.available===true&&Number.isInteger(c.result)){
  const checks={1:'动作检查通过',0:'实际障碍占用','-1':'动作涉及区域尚未观测','-2':'观测源超过有效年龄','-3':'当前运动位姿不可用','-4':'本轮计算预算不足，保留路线等待重试','-5':'底盘方向门拒绝当前候选','-6':'动作参数或检查状态无效'};
  lines.push((c.current===false?'历史动作检查（当前未执行）：':'最近检查：')+(checks[c.result]||'未知检查结果')+(Number.isFinite(c.age_ms)&&c.age_ms>=0?' · '+c.age_ms.toFixed(0)+'ms前':''));
 }
 if(p&&[p.observe_mm,p.near_mm,p.forward_stop_mm,p.emergency_mm,p.release_mm].every(Number.isFinite))lines.push('前方门限：观察'+p.observe_mm+'／近障'+p.near_mm+'／禁前进'+p.forward_stop_mm+'／急刹'+p.emergency_mm+'／恢复'+p.release_mm+'mm');
 return lines.join('\n');
};
// 绘制、点击、平移、缩放和目标微调共用变换，+X向右、+Y向上。
window.ACGNavMapGeometry={
 pixelToWorld:(v,x,y,w,h)=>[v.x+(x/w-.5)*v.span,v.y+(.5-y/h)*v.span],
 worldToPixel:(v,x,y,w,h)=>[(.5+(x-v.x)/v.span)*w,(.5-(y-v.y)/v.span)*h],
 zoom:(v,f,x,y,w,h,min,max)=>{const a=window.ACGNavMapGeometry.pixelToWorld(v,x,y,w,h),s=Math.max(min,Math.min(max,v.span*f));return{x:a[0]-(x/w-.5)*s,y:a[1]-(.5-y/h)*s,span:s};}
};
(() => {
 const $=id=>document.getElementById(id),dlg=$('navigationMapDialog'),canvas=$('navigationMapCanvas'),ctx=canvas.getContext('2d',{alpha:false}),status=$('navigationMapStatus'),goalText=$('navigationGoalStatus'),preview=$('previewNavigationGoal'),start=$('startNavigationDrive'),hint=$('navigationDriveHint');
 const g=window.ACGNavMapGeometry,pointers=new Map();let timer=null,motionTimer=null,motion=null,motionAt=0,motionIssue="",map=null,selection=null,view={x:0,y:0,span:4},initialized=false,follow=true,gesture=null,multiTouch=false,ignoreClickUntil=0,imageCanvas=null,imageCells=null,lastPublished=null,lastChanged=0,mapReadError=null,requestError=null,mapFailures=0,sessionRecovering=false,sessionMessage="";
 const labels={RUNNING:'正常行驶',ODOMETRY_RUNNING:'正常行驶 · 里程计续行',STARTING_WAIT_WHEELS:'正在起步 · 等待轮速',WAIT_FIRST_FEEDBACK:'等待首笔底盘反馈',WHEELS_ZERO:'已发运动请求 · 实测轮速仍为零',FEEDBACK_STALE:'底盘反馈过期',MOTOR_FEEDBACK_FAULT:'电机反馈故障',MOTOR_RECOVERY:'电机反馈恢复中',ARMING:'正在使能导航',PLANNING:'后台规划路线',NAVIGATION_WAIT:'导航等待 · 目标保留',OBSTACLE_WAIT:'通道受阻 · 目标保留',ULTRASONIC_WAIT:'超声波保护 · 目标保留',ROUTE_RETRY:'路线暂不可用 · 自动重试',SENSOR_OR_LOCALIZATION_WAIT:'传感器或定位等待',GOAL_REACHED:'已到达目标',WAIT_START:'路线已就绪 · 等待开始',STOPPED:'已停止',FAULT_STOPPED:'故障停车',PREVIEW_ONLY:'路线预览 · 不驱动车轮',DETOUR_LEFT:'正在向左绕行',DETOUR_RIGHT:'正在向右绕行',ALIGNING:'正在调整车头',BACK_CHECK:'正在检查后退空间',BACKING:'正在后退脱困',REJOINING:'正在接回原路线'};
 Object.assign(labels,{FRONT_APPROACH:'提前绕行 · 比较左右通道',NF_EMERGENCY:'紧急近障 · 刹车复核',NEAR_AVOID:'前方过近 · 检查后退路径',TURN_HOLD:'转向完成 · 停稳等待新观测',RECOVERY_WAIT:'暂无线行动作 · 目标保留'});
 const stops={GOAL_REACHED:'到达目标',NEW_GOAL:'更换目标',USER_OR_PHONE_CANCEL:'用户停止或接管',STALE_OR_INVALID_LOCALIZATION:'传感器或定位过期',NAVIGATION_WAIT_OR_FAILURE:'规划或避障等待',PLANNING_OVERRUN_STOP_REQUIRES_START:'控制处理超时',INVALID_CHASSIS_FEEDBACK:'底盘反馈失效或故障',PROTECTION_STOP:'底盘保护',CHASSIS_APPLY_FAILED:'运动指令发送失败',SENSOR_POLL_FAILED:'传感器接收故障'};
 const details={OBSTACLE:'近场雷达预测碰撞',SWEEP_BLOCKED:'车身扫过区域不通行',REPLANNING:'重新规划路线',SCAN_STALE:'雷达数据过期',LIDAR_DATA_STALE:'雷达数据过期',POSE_STALE:'位置数据过期',ODOMETRY_DATA_STALE:'IMU或编码器位置过期',NO_PATH:'当前没有可行路线',SEARCH_LIMIT:'搜索达到计算上限，后台重试',ODOMETRY_CONTINUE_2S:'里程计续行',LOCALIZATION_JUMP_RESET_REQUIRED:'定位发生较大跳变',UNKNOWN_NEW_AREA:'车身范围包含未观测区域',OCCUPIED_FOOTPRINT:'车身范围包含占用区域',BACK_REAR_UNKNOWN:'后方未观测，未执行后退',BACK_REAR_BLOCKED:'后方受阻，未执行后退',ULTRASONIC_PROTECTION_BLOCKED:'FPGA超声波保护',FPGA_STATUS_STALE:'FPGA状态反馈过期',NCV1_ACK_ONLY_REASON_6:'本地停止或接管'};
 Object.assign(labels,{PROBING:'正在局部试探 · 目标保留',CRUISE:'自动模式',RECOVERY_RETRY_WAIT:'脱困等待 · 目标保留',RECOVERY_EXHAUSTED:'正在重新选择方向 · 目标保留',CONTROL_RETRY:'控制计算超时 · 等待恢复'});
 Object.assign(stops,{CONTROL_COMPUTE_OVERRUN_RETRY:'控制计算超时，保留目标等待恢复'});
 Object.assign(details,{CONTROL_COMPUTE_OVERRUN_RETRY:'控制计算超时，等待恢复'});
 Object.assign(labels,{SONAR_AVOIDING:'近障绕行 · 选择离障方向',ULTRASONIC_WAIT:'超声波方向限制 · 尝试其他动作',OBSERVATION_WAIT:'观测证据不足 · 保留目标继续判断',REAR_OBSERVATION_WAIT:'后方观测不足 · 尝试前进或转向',REAR_BLOCKED:'后方有障碍 · 尝试其他动作',ALIGNMENT_HOLD:'等待扫描对齐 · 沿用尚未过期的已验证动作',ALIGNMENT_WAIT:'等待扫描对齐 · 目标与控制权保留'});
 Object.assign(labels,{MANUAL_RUNNING:'手动或其他来源正在行驶 · 无自主导航任务',IDLE_NO_GOAL:'没有活动导航任务 · 车轮已停',IDLE_FEEDBACK_UNKNOWN:'没有活动导航任务 · 当前轮速待更新'});
 Object.assign(details,{UNKNOWN_NEW_AREA:'局部观测证据不足，不等同于碰撞',OBSERVED_SHORT_PROBE:'当前观测支持短距离试探',TENTATIVE_UNKNOWN_ROUTE:'路线含待观测区域，边走边验证',REAR_SCAN_STALE:'后方扫描过期，未执行后退',REAR_COVERAGE:'后方有效观测不足，尝试其他方向',REAR_SONAR_STALE:'后退相关测距数据过期',REAR_BODY_OCCUPIED:'后方车身扫过区域有已确认障碍',REAR_GEOMETRY_UNKNOWN:'后方车身范围观测不足',REAR_CLEARANCE:'后方可退距离不足',ALIGNMENT_WAIT_HOLD:'等待扫描时间对齐，保持原有效期内的已验证动作',ALIGNMENT_WAIT_ZERO:'等待扫描时间对齐，原动作已过期，目标保留',SONAR_DIRECTION_REJECTED:'当前方向过近，改选离障动作'});
 Object.assign(details,{FPGA_POLICY_MISMATCH:'FPGA策略不匹配：需要CFG31110301／状态修订5',FRONT_APPROACH:'前向走廊发现障碍，提前选择出口',NF_EMERGENCY:'≤20mm新鲜近障，先刹车再核对',SONAR_GLITCH_RELEASE:'短时近障毛刺已由三笔清空观测解除',NEAR_AVOID:'前方不足30mm时禁止前移，检查其他动作',SIDE_ACTION_REJECT:'当前动作会压缩侧方净空，正在改选离障动作',TURN_HOLD:'转向完成，零速等待两笔至少40mm新观测',PATH_ACTION_MISMATCH:'显示路径与实际动作方向不一致，已按动作更新',MAP_EXPORT_STALLED:'地图导出停滞，保留最后有效地图',REAR_BLOCKED:'后方有障碍，不强行后退',REAR_UNOBSERVED:'后方缺少完整观测，不强行后退',RECOVERY_WAIT:'当前无可行动作，保留目标等待环境变化'});
 Object.assign(details,{LOW_RECOVERY:'恢复前进中，等待两笔250mm新观测再解除低速',FRONT_BODY_SWEEP:'当前动作车身扫掠会接近前障',SIDE_ACTION_REJECT_LEFT_SWEEP:'左侧车头或后角扫掠受阻',SIDE_ACTION_REJECT_RIGHT_SWEEP:'右侧车头或后角扫掠受阻',SIDE_ACTION_REJECT_LEFT_CLOSER:'左侧已近，该动作会继续压缩左侧净空',SIDE_ACTION_REJECT_RIGHT_CLOSER:'右侧已近，该动作会继续压缩右侧净空',SIDE_ACTION_REJECT_LEFT_30MM:'动作将使左侧净空小于30mm',SIDE_ACTION_REJECT_RIGHT_30MM:'动作将使右侧净空小于30mm',SIDE_EMERGENCY_LEFT_NOT_AWAY:'左侧紧急近障，候选没有远离左障',SIDE_EMERGENCY_RIGHT_NOT_AWAY:'右侧紧急近障，候选没有远离右障',FRONT_SPEED_ENVELOPE:'候选速度超过当前前方净空包络',FRONT_CORRIDOR_SWEEP:'前方1.2米走廊有障碍侵入动作路径',NEAR_AVOID_FORWARD_FORBIDDEN:'前方不足100mm，先停稳检查后退',FRONT_100_150_PIVOT_OR_BACKUP_ONLY:'旧规则前方100～150mm限制，本轮正在重选动作',ESCAPE_REQUIRES_150MM:'明显前进避障弧线需要至少150mm前沿净空',FRONT_APPROACH_DIRECTION_HOLD:'保持已选绕行方向，等待另一侧持续明显更优',NEAR_PIVOT_FULL_SWEEP_REQUIRED:'近障原地转需完整车身扫掠检查通过'});
 Object.assign(details,{LOCAL_PROBE:'完整路线未就绪，沿已观测空地局部试探',CRUISE:'雷达选通道巡航',RECOVERY_RETRY_WAIT:'等待再次脱困',RECOVERY_EXHAUSTED:'本轮继续观察新空间，目标保留'});
 const errorNames={AUTH_REQUIRED:'连接密码不正确',SESSION_EXPIRED:'手机控制会话已过期，请重新连接',REQUEST_DUPLICATE:'请求已处理，请等待板端状态',CONTROL_CLAIM_PENDING:'控制权正在同步，请稍后重试',LEGACY_AUTO_REMOVED:'请使用新版自动模式请求',MANUAL_CONTROL_ACTIVE:'请先松开手动方向键',NAV_IPC_UNAVAILABLE:'板端导航服务未就绪',MAP_NOT_READY:'地图尚未生成',MAP_STALE:'板端地图已过期',MAP_FILE_INVALID:'地图文件格式异常',MAP_MEMORY_LIMIT:'地图服务内存不足',MAP_READ_FAILED:'地图读取失败',MAP_SCHEMA_INVALID:'地图版本不兼容',MAP_FORMAT_INVALID:'地图显示数据无效',REQUEST_TIMEOUT:'请求超时',NETWORK_ERROR:'网络请求失败',RESPONSE_INVALID:'返回数据格式不兼容',REQUEST_CONFLICT:'请求冲突',REQUEST_INVALID:'请求格式错误',HTTP_ERROR:'服务请求失败'};
 Object.assign(errorNames,{CLIENT_NOT_READY:'手机连接或前台状态未就绪，请重新连接',CLIENT_REQUEST_BUSY:'上一笔请求仍在发送，本次未发送，请稍后重试',REQUEST_CANCELLED_BY_CONTROL:'请求排队期间发生停止或接管，本次未发送',NAV_ACK_TIMEOUT:'5秒内未收到板端最终回执，可再次预览；不会自动启动车辆',TARGET_ACK_MISMATCH:'回执与当前候选目标不一致，请重新预览',DRIVE_DISABLED:'当前仅预览，请在RK执行 sh begin drive',DRIVE_CAPABILITY_REQUIRED:'当前仅预览，请在RK执行 sh begin drive',MOUNT_NOT_VERIFIED:'雷达安装位置尚未确认',CALIBRATION_REQUIRED:'当前里程计配置未确认',NO_GOAL:'尚未选择目标，请先选点并预览',ALREADY_ACTIVE:'任务已启动，请看当前执行状态',START_PENDING:'任务启动请求已接受，等待实际执行许可',WAIT_START:'已接收目标，等待明确开始',READY:'任务启动条件已满足',NONE:'无额外阻挡'});
 Object.assign(errorNames,{PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE:'当前仅预览，请在RK执行 sh begin drive',LIDAR_OFFSET_NOT_VERIFIED:'雷达安装位置尚未确认',UNCALIBRATED_ARM_DENIED:'里程计配置尚未确认',SELECT_AND_PREVIEW_GOAL_FIRST:'尚未选择目标，请先选点并预览',NAVIGATION_ALREADY_REQUESTED:'导航任务已经接收，不需要反复点击开始',PHONE_SESSION_NOT_OWNED_OR_EXPIRED:'手机未持有当前控制会话或会话已过期，请重新连接',WAIT_FRESH_LOCALIZATION:'等待新的可信定位；收到雷达和IMU不等于已经恢复地图车位',WAIT_FRESH_WHEEL_FEEDBACK:'等待新的轮速反馈',FINAL_MOTOR_FAULT_WAIT_FRESH_HEALTHY_FEEDBACK:'电机反馈故障，等待健康反馈后再明确开始',WAIT_CURRENT_FEASIBLE_ACTION_CONTINUE_EXPLORING:'当前尚未找到可行动作，目标保留并继续观察、检查其他方向',GOAL_ACCEPTED_WAIT_CURRENT_ROUTE_OR_LOCAL_ACTION:'目标已接受，正在规划或检查局部可行动作',GOAL_ACCEPTED:'目标已接受',MAP_RESET_REQUEST_RECEIVED:'重建地图请求已接收',USER_CANCELLED_TARGET_CLEARED:'已停车并取消目标，请重新选点预览',PHONE_LINK_EXPIRED_NAVIGATION_STOPPED:'手机控制心跳过期，已撤销导航执行',MANUAL_OVERRIDE_NAVIGATION_STOPPED:'手动操作接管，导航已经取消',GOAL_PREDATES_MANUAL_REVOKE:'该目标请求早于最近一次停止或手动接管，请重新预览',PHONE_SESSION_BUSY_EXPIRED_OR_REQUEST_REPLAYED:'控制会话不符、已过期或请求重复，请重新连接后重试',PHONE_SESSION_BUSY_OR_EXPIRED:'控制会话正被占用或已过期，请重新连接后重试',CANCEL_OWNER_OR_SEQUENCE_MISMATCH:'取消请求的会话或序号不匹配，停车指令仍独立发送',START_DENIED_CURRENT_MAP_AND_ACCEPTED_GOAL_REQUIRED:'开始请求不是当前地图中的已接受目标，请重新预览',START_PREDATES_MANUAL_REVOKE:'开始请求早于最近一次停止或接管，已拒绝，请重新开始',START_DENIED_PHONE_SESSION:'开始请求的手机会话不匹配',START_DENIED_PHONE_SESSION_OR_SEQUENCE:'开始请求的会话、序号或有效期不符，请重试',START_OWNER_LOST_OR_CANCELLED:'开始请求已因控制权丢失或取消失效',EXPLICIT_NAVIGATION_ENABLE_REQUESTED:'导航任务已经接收，等待底盘执行许可',NAVIGATION_ACTIVE:'导航任务执行中',WAIT_RUNTIME_START_READINESS:'目标保留，等待底盘和定位满足当前动作',NO_ACTIVE_GOAL:'当前已经没有活动目标',START_TARGET_CHANGED_REPREVIEW:'已接受目标发生变化，请重新预览后开始'});
 Object.assign(details,window.ACGR4ReasonLabels);Object.assign(stops,window.ACGR4ReasonLabels);
 errorNames.START_TARGET_CHANGED_REPREVIEW_REQUIRED='已接受的目标坐标发生变化，旧开始请求已拒绝，请重新预览后开始';
 errorNames.AUTO_INTENT_ALREADY_ACCEPTED_WAITING='自动任务已经接收，正在等待可行动作；无需重复启动';
 const operationNames={map:'地图更新',goal:'目标预览',tag:'标记预览',start:'开始请求',stop:'停止请求',reset:'重建请求'};
 let pendingAction=null,lastRequestReceipt=null;
 const validRequestId=id=>typeof id==='string'&&/^\d{1,20}$/.test(id);
 const operationEquivalent=(a,b)=>a===b||(a==='stop'&&b==='cancel');
 const requestReason=reason=>errorNames[reason]||details[reason]||stops[reason]||reason||'等待具体原因';
 function goalMatches(candidate,m){return !!(candidate&&m.has_goal&&Array.isArray(m.goal)&&((Math.abs(m.goal[0]-candidate.x)<1e-6&&Math.abs(m.goal[1]-candidate.y)<1e-6)||(m.goal_adjusted===true&&Array.isArray(m.requested_goal)&&Math.abs(m.requested_goal[0]-candidate.x)<1e-6&&Math.abs(m.requested_goal[1]-candidate.y)<1e-6)));}
 function applyRequestReceipt(m){
  const ack=m.nav_request;
  if(pendingAction&&ack&&validRequestId(ack.id)&&ack.id===pendingAction.id&&operationEquivalent(pendingAction.action,ack.operation)){
   const accepted=['ACCEPTED','ARMING','ACTIVE'].includes(ack.state),rejected=ack.state==='REJECTED';
   if(rejected){lastRequestReceipt={...ack,action:pendingAction.action};requestError={operation:pendingAction.action,code:ack.reason||'REQUEST_REJECTED',message:requestReason(ack.reason)};if(selection&&selection.requestId===ack.id)selection.submitted=false;pendingAction=null;}
   else if(accepted){
    // 编号和当前坐标须同时匹配，已接受的旧目标
    // 不能让另一黄色候选点获得启动许可。
    if(pendingAction.action!=='goal'||!selection||selection.requestId!==ack.id||goalMatches(selection,m)){
     if(pendingAction.action==='goal'&&selection&&selection.requestId===ack.id)selection=null;
     lastRequestReceipt={...ack,action:pendingAction.action};requestError=null;pendingAction=null;
    }
   }
  }
  if(pendingAction&&Date.now()-pendingAction.startedAt>5000){const action=pendingAction.action;if(selection&&selection.requestId===pendingAction.id)selection.submitted=false;requestError={operation:action,code:'NAV_ACK_TIMEOUT',message:requestReason('NAV_ACK_TIMEOUT')};pendingAction=null;}
  if(selection&&selection.submitted&&!pendingAction&&selection.submittedAt&&Date.now()-selection.submittedAt>5000){selection.submitted=false;requestError={operation:'goal',code:'NAV_ACK_TIMEOUT',message:requestReason('NAV_ACK_TIMEOUT')};}
  if(lastRequestReceipt&&ack&&ack.id===lastRequestReceipt.id&&operationEquivalent(lastRequestReceipt.action,ack.operation))lastRequestReceipt={...ack,action:lastRequestReceipt.action};
 }
 function errorText(e){return(operationNames[e.operation]||'导航请求')+'：'+(errorNames[e.code]||e.message||e.code||'请求未完成')+((e.code==='MAP_FORMAT_INVALID'&&e.message)?'：'+String(e.message).slice(0,180):'')+(e.http_status?'（HTTP '+e.http_status+'）':'')+' ['+(e.code||'UNKNOWN')+']';}
 const fmt=n=>typeof n==='number'&&Number.isFinite(n)?n.toFixed(1):'—',rpms=a=>Array.isArray(a)?a.map(fmt).join(' / '):'—';
 let motorState=null,evidence=null,evidenceAt=0;
 function motorText(m){
  const live=motorState&&motorState.fresh&&Date.now()-motorState.at<850;
  const known=live?motorState.robot.feedback_supported===true:!stale()&&m&&m.motor_recovery_known===true;
  const phase=live?motorState.robot.feedback_phase:m&&m.motor_recovery_phase,retries=live?motorState.robot.feedback_retries:m&&m.motor_recovery_retries;
  if(!known||!Number.isInteger(phase)||phase<0||phase>4){$('navigationMotorStatus').textContent='电机恢复阶段：未收到新遥测';return;}
  const names=['电机反馈正常','电机反馈暂缺 · 受限恢复','电机停机等待重试','电机试运行恢复','电机最终反馈故障'];
  $('navigationMotorStatus').textContent=names[phase]+(phase?' · 第'+(Number(retries)||0)+'次重试':'');
  if(m&&m.enabled&&phase)$('navigationVehicleStatus').textContent=names[phase];
 }
 window.navReceiveMotorState=(robot,fresh)=>{motorState={robot:robot||{},fresh:!!fresh,at:Date.now()};if(dlg.open)motorText(map);};
 let pendingObject=null,objectMission=null;
 let tagsSnapshot=null,tagsReceivedAt=0,pendingTag=null;
 Object.assign(details,{ALL_ACTIONS_UNAVAILABLE:'暂未找到可执行动作，目标保留并继续选择',NEARFIELD_OBSERVATION_WAIT:'当前动作区域等待观测，继续选择其他方向',NF_CLEAR:'当前动作可执行',NF_COLLISION:'当前动作扫掠区域有障碍',NF_NEED_OBSERVATION:'当前动作区域等待观测，继续选择其他方向',NF_DATA_STALE:'近场扫描过期',NF_POSE_INVALID:'近场检查缺少有效位姿',NF_INVALID_ACTION:'动作参数无效',NEARFIELD_RECENT:'当前近场观测',NEARFIELD_PENDING:'历史障碍待复核',NEARFIELD_CAPACITY:'旧版缓存容量告警',NEARFIELD_NEED_OBSERVATION:'动作区域需要新观测',NEARFIELD_DATA_STALE:'近场扫描过期',NEARFIELD_POSE_INVALID:'近场位姿无效',NEARFIELD_INVALID_ACTION:'动作参数无效',REAR_NEED_OBSERVATION:'后退区域需要观测，继续检查其他动作',REAR_POSE_INVALID:'后退检查缺少有效位姿',NEARFIELD_COLLISION:'当前动作会碰到近场障碍'});
 const visionReasons={"NONE":"无异常","OK":"正常","CALIBRATION_REQUIRED":"镜头尚未标定，仅显示ID","FRAME_HEADER_INVALID":"原始图像头不完整","FRAME_LAYOUT_INVALID":"原始图像尺寸或版本不匹配","FRAME_TRUNCATED":"原始图像不完整","FRAME_OVERSIZE":"原始图像超出规定长度","FRAME_STALE_AFTER_DETECTION":"识别完成时图像已过期","TAG_CAMERA_EYE_MISMATCH":"标定眼与图像眼不一致","TAG_CAMERA_MOUNT_NOT_VERIFIED":"请先确认摄像头安装位置和朝向","TAG_CAMERA_NOT_CALIBRATED":"请先完成单眼镜头标定","TAG_CONFIG_UNAVAILABLE":"标记配置不存在或无效","TAG_CAPTURE_POSE_NOT_IN_HISTORY":"图像时刻没有对应的定位记录","TAG_IMAGE_STALE_OR_TIME_INVALID":"目标图像过期或时间不匹配","TAG_MAP_CONTEXT_CHANGED":"地图已更换，请重新选择目标","TAG_STALE_MAP":"地图已过期，请等待新地图","TAG_SNAPSHOT_UNAVAILABLE_OR_INVALID":"标记服务尚未输出有效数据","TAG_NOT_VISIBLE":"当前未看见所选标记","TAG_NEEDS_THREE_NEW_FRAMES":"等待三笔新图像确认标记","TAG_POSE_INVALID":"标记姿态无效，请调整距离或角度","TAG_POSE_QUALITY_REJECTED":"测距质量不足，请调整光线或标记位置","TAG_PLANE_NORMAL_AMBIGUOUS":"标记朝向不明确，请调整拍摄角度","TAG_PLANE_NOT_VERTICAL_ENOUGH":"标记倾斜过大，请竖直安装","TAG_INVALID_STANDOFF_OR_BODY":"停车距离或车身参数无效","TAG_WORLD_POSITION_CHANGED":"目标位置发生变化，请重新确认","TAG_APPROACH_OCCUPIED_OR_UNKNOWN":"接近位置被占用或尚未观测","TAG_NAVIGATOR_REJECTED_GOAL":"导航器未接受接近位置，请重新预览","TAG_CONFIRMED_WAIT_ROUTE":"标记已确认，等待路线计算","TAG_WAIT_MAP_LOCALIZATION":"等待地图与定位就绪","TAG_ROUTE_AVAILABLE_EXPLICIT_START_REQUIRED":"路线就绪，请明确点击开始导航","TAG_START_DENIED_WAIT_TARGET_ROUTE_PREVIEW":"请先等待标记路线预览完成","TAG_FIXED_GOAL_NAVIGATION":"正在前往已确认的固定目标","TAG_WAIT_NEW_IMAGE_AFTER_STOP":"已到接近点，等待一笔停车后的新图像","TAG_REOBSERVED_AT_NAV_GOAL":"已到达并重新确认目标",UNCALIBRATED:'镜头尚未标定，仅显示ID',WAIT_FRAME:'等待原始摄像头图像',FRAME_STALE:'图像过期',NO_TAG:'未发现标记',CAMERA_EXTRINSIC_UNVERIFIED:'摄像头安装位置未确认',TAG_NOT_CONFIRMED:'标记位置尚待连续确认',TAG_STALE:'标记观测过期',WAIT_REOBSERVE:'到达接近点，等待重新看见目标',TAG_REOBSERVE_REQUIRED:'等待目标复核',POSE_HISTORY_UNAVAILABLE:'图像时刻没有对应定位',VISION_CONFIG_INVALID:'视觉配置无效'};
 const visionStates={PREVIEW_PENDING:'目标已提交，正在检查路线',PREVIEW_READY:'路线已就绪，等待明确开始',NAVIGATING:'正在前往标记',WAIT_REOBSERVE:'等待近距离目标复核',ARRIVED:'已到达并确认目标',REJECTED:'目标未接受',CANCELLED:'任务已取消'};
 function visionText(s){if(visionReasons[s])return visionReasons[s];if(typeof s==='string'&&s.startsWith('TAG_ROUTE_')){const r=s.slice(10);return '标记路线：'+(details[r]||labels[r]||({NONE:'正在计算',WAIT_TIMEOUT:'正在等待规划',PLANNING:'正在计算路线'}[r])||r);}return s||'等待数据';}
 const tagPanel=document.querySelector('.navigation-tag-controls');
 const tagsVisible=()=>!!tagPanel&&!tagPanel.hidden;
 function updateTags(){
  if(!tagsVisible())return;
  const select=$('navigationTagId'),button=$('previewNavigationTag'),data=tagsSnapshot;
  if(!select||!button)return;
  const old=select.value,valid=data&&Date.now()-tagsReceivedAt<2500&&data.stale!==true&&data.fresh!==false;
  const tags=valid&&Array.isArray(data.tags)?data.tags.filter(t=>Number.isInteger(t.id)&&t.id>=0&&t.id<=586):[];
  select.textContent='';
  if(!tags.length){const o=document.createElement('option');o.value='';o.textContent='尚未识别目标';select.appendChild(o);}
  for(const t of tags){const o=document.createElement('option');o.value=String(t.id);o.textContent='ID '+t.id+(data.calibrated&&Number.isFinite(t.range_m)?' · '+t.range_m.toFixed(2)+'m':' · 待标定')+(t.confirmed?' · 已确认':' · 确认中');select.appendChild(o);}
  if(tags.some(t=>String(t.id)===old))select.value=old;
  const chosen=tags.find(t=>String(t.id)===select.value);
  button.disabled=!(valid&&data.calibrated&&chosen&&chosen.confirmed&&map&&poseFresh(map)&&!stale()&&!mapReadError&&(!map.enabled||map.cruise_mode));
  $('navigationTagStatus').textContent=!valid?'标记数据未就绪或过期':('图像眼 '+data.eye+' · '+(data.calibrated?'镜头已标定':'镜头未标定')+' · '+visionText(data.reason));
 }
 function request(){if(dlg.open&&window.Rover){window.Rover.navReadMap();if(tagsVisible()&&window.Rover.navReadTags)window.Rover.navReadTags();if(window.Rover.objectReadMap)window.Rover.objectReadMap();}updateTags();}
 function stale(){return !!(map&&lastChanged&&Date.now()-lastChanged>15000);}
 function mapIntegrityBlocked(){return !!(mapReadError&&['MAP_FORMAT_INVALID','MAP_SCHEMA_INVALID','MAP_FILE_INVALID','RESPONSE_INVALID'].includes(mapReadError.code));}
 function poseFresh(m){return !!m;} // 运动与位姿时效由RK统一判断，手机不重复设置500ms门限。
 function acceptEvidence(source){
  if(!source||!source.nearfield||!source.avoidance)return;
  try{
   const n=source.nearfield,a=source.avoidance;
   if(n.version!==1||a.version!==1||typeof n.valid!=='boolean'||typeof n.stale!=='boolean'||!Number.isFinite(n.age_ms)||n.age_ms< -1||!Array.isArray(n.hazards)||n.hazards.length>32||!n.hazards.every(p=>Array.isArray(p)&&p.length===3&&p.slice(0,2).every(Number.isFinite)&&typeof p[2]==='string'&&/^\d{1,20}$/.test(p[2]))||![n.front_m,n.left_m,n.right_m,n.rear_m].every(x=>x===null||Number.isFinite(x)&&x>=0)||!Number.isInteger(a.stage)||a.stage<0||a.stage>6||![-1,0,1].includes(a.direction)||![a.turn_target_rad,a.turn_progress_rad,a.backup_target_m,a.backup_progress_m].every(Number.isFinite))throw Error('近场诊断格式不兼容');
   const publication=Number(source.published_us);
   if(!Number.isFinite(publication)||!map||source.map_id!==map.map_id)return;
   if(evidence&&evidence.map_id===source.map_id&&publication<=evidence.publication)return;
   const pose=Array.isArray(source.body_pose)&&source.body_pose.length===3&&source.body_pose.every(Number.isFinite)?source.body_pose:null;
   const merged={...a,policy_revision:source.policy_revision??a.policy_revision,front_approach:source.front_approach??a.front_approach,nf_emergency:source.nf_emergency??a.nf_emergency,turn_hold:source.turn_hold??a.turn_hold,front_clearance_m:source.front_clearance_m??a.front_clearance_m,side_state:source.side_state??a.side_state};
   for(const key of ['scene_id','scene','scene_zh','scene_reason','scene_reason_zh','scene_owner','scene_owner_zh','scene_stage','scene_stage_zh','scene_speed_cap','scene_calibrated','explore_round','usable_direction_mask','direction_age_ms','scene_pending','route_policy','action_check','front_policy'])if(source[key]!==undefined)merged[key]=source[key];
   const feedback=source.live_feedback;
   if(feedback!==undefined&&(!feedback||feedback.version!==1||typeof feedback.fresh!=='boolean'||!Number.isFinite(feedback.age_ms)||feedback.age_ms< -1||!Array.isArray(feedback.wheel_rpm)||feedback.wheel_rpm.length!==4||!feedback.wheel_rpm.every(x=>x===null||Number.isFinite(x))||!Array.isArray(feedback.target_wheel_rpm)||feedback.target_wheel_rpm.length!==4||!feedback.target_wheel_rpm.every(Number.isFinite)))throw Error('实时轮速诊断格式不兼容');
   evidence={map_id:source.map_id,publication,n,a:merged,pose,poseAge:Number.isFinite(source.body_pose_age_ms)?source.body_pose_age_ms:null,feedback:feedback||null,transport:Math.max(0,Number(source.transport_age_ms)||0)};evidenceAt=Date.now();
  }catch(e){const el=$('navigationNearfieldStatus');if(el)el.textContent=e.message;}
 }
 function evidenceAge(){return evidence?evidence.transport+Math.max(0,Date.now()-evidenceAt):Infinity;}
 function nfQueryText(n){
  const q=n&&n.query;
  if(!q||!/^\d{1,20}$/.test(String(q.checked_us))||Number(q.checked_us)<=0||!Number.isInteger(q.code)||q.code<0||q.code>5||!Number.isFinite(q.v)||!Number.isFinite(q.w))return '';
  const source={0:'无障碍来源',1:'最新扫描',2:'近期障碍',3:'历史位置待复核'}[q.source]||'来源待确认';
  const action=q.v<-.001?'后退':q.v>.001?(Math.abs(q.w)>.05?(q.w>0?'前进左弯':'前进右弯'):'前进'):q.w>.05?'原地左转':q.w<-.05?'原地右转':'零速';
  const reason=['可执行','扫掠区域确认有障碍','扫掠区域需要观测','扫描数据过期','动作检查缺少有效位姿','动作参数无效'][q.code];
  return ' · 最近检查：'+action+'／'+reason+(q.code?'／'+source:'');
 }
 function nfStorageText(n){
  const h=n&&n.storage;
  if(!h||![h.cells,h.capacity,h.pending,h.bytes].every(Number.isFinite)||h.capacity<=0||h.cells<0||h.cells>h.capacity)return '';
  return ' · 历史格'+h.cells+'/'+h.capacity+'，待复核'+h.pending+'，'+(h.bytes/1024).toFixed(0)+'KiB';
 }
 function avoidanceText(){
  const el=$('navigationNearfieldStatus'),progress=$('navigationAvoidanceProgress');if(!el||!progress)return;
  if(!evidence||!map||evidence.map_id!==map.map_id){el.textContent='雷达车身净空：等待近场观测';progress.textContent='避让动作：尚未触发';return;}
  const n=evidence.n,a=evidence.a,age=n.age_ms<0?Infinity:n.age_ms+evidenceAge(),fresh=n.valid&&!n.stale&&age<=300;
  const distance=x=>x===null?'未知':(x*100).toFixed(1)+'cm';
  el.textContent='雷达车身净空：前'+distance(n.front_m)+' 左'+distance(n.left_m)+' 右'+distance(n.right_m)+' 后'+distance(n.rear_m)+(fresh?' · 新扫描实时':' · 观测过期／待确认')+(n.held_points?' · 含'+n.held_points+'个保留风险点':'')+' · '+(details[n.reason]||n.reason||'')+nfStorageText(n)+nfQueryText(n);
  if(Number.isInteger(n.forward_returns)&&Number.isInteger(n.forward_bins)){const why={FORWARD_RETURNS_PRESENT:'正前方有有效回波',FORWARD_CORRIDOR_CLEAR_TO_RETURN:'车头窄道有有效回波',FORWARD_NO_VALID_RETURNS:'正前方没有有效回波，不能据此认定空地',FORWARD_SELF_RETURNS_ONLY:'正前方回波落在车体范围，请检查遮挡或安装'};el.textContent+='\n正前方回波 '+n.forward_returns+'点／覆盖'+n.forward_bins+'格（每格5°） · 车前窄道'+(Number.isFinite(n.forward_corridor_m)?'净空'+(100*n.forward_corridor_m).toFixed(1)+'cm':'本扫描无有效落点')+' · '+(why[n.forward_reason]||n.forward_reason||'无近点不等于没有回波');if(n.forward_self_returns>0)el.textContent+=' · 车体范围回波'+n.forward_self_returns+'点';}
  const stages=['无避让任务','连续弧线避让','转向腾挪','后退腾挪','接回原路线','等待新的可行动作','转向后停稳复核'];
  progress.textContent='避让：'+stages[a.stage]+(a.direction?' · 向'+(a.direction>0?'左':'右'):'')+(a.active?' · 任务#'+a.event_id:'')+' · 转角'+(Math.abs(a.turn_progress_rad)*180/Math.PI).toFixed(1)+'°/'+(Math.abs(a.turn_target_rad)*180/Math.PI).toFixed(1)+'° · 后退'+(a.backup_progress_m*100).toFixed(1)+'/'+(a.backup_target_m*100).toFixed(1)+'cm'+(evidenceAge()>600?' · 历史状态':'')+(a.reason?' · '+a.reason:'');
  if(Number(a.policy_revision)>=5){const side=['正常','接近','过近','紧急'];progress.textContent+=' · 策略S'+a.policy_revision+(Number.isFinite(a.front_clearance_m)&&a.front_clearance_m>=0?' · 前沿净空'+(a.front_clearance_m*100).toFixed(1)+'cm':' · 前沿净空未知')+(Array.isArray(a.side_state)?' · 左'+(side[a.side_state[0]]||'未知')+'／右'+(side[a.side_state[1]]||'未知'):'')+(a.nf_emergency?' · 紧急近障刹车复核':a.turn_hold?' · 停稳等待两笔新观测':'');}
  if(a.active&&fresh&&evidenceAge()<=600&&map.enabled&&map.pose_valid!==false&&map.vehicle_status!=='MOTOR_FEEDBACK_FAULT'&&map.vehicle_status!=='NF_EMERGENCY'&&!a.nf_emergency&&!(map.motor_recovery_known&&map.motor_recovery_phase))$('navigationVehicleStatus').textContent='雷达避让 · '+stages[a.stage];
 }
 function drawNearfield(){
  if(!evidence||!map||map.pose_valid===false||stale()||evidence.map_id!==map.map_id||!evidence.pose||evidenceAge()>600)return;
  const n=evidence.n,p=evidence.pose,ca=Math.cos(p[2]),sa=Math.sin(p[2]),fresh=n.valid&&!n.stale&&n.age_ms>=0&&n.age_ms+evidenceAge()<=300;
  for(const h of n.hazards){const age=(evidence.publication-Number(h[2]))/1000+evidenceAge();if(age<0||!Number.isFinite(age))continue;ctx.fillStyle=fresh&&age<=300?'#ff6558':'#929ca4';const q=g.worldToPixel(view,p[0]+ca*h[0]-sa*h[1],p[1]+sa*h[0]+ca*h[1],canvas.width,canvas.height);if(ctx.fillRect)ctx.fillRect(q[0]-3,q[1]-3,6,6);}
 }
 function motionAge(){return motion?Math.max(0,Number(motion.transport_age_ms)||0)+Math.max(0,Date.now()-motionAt):Infinity;}
 function motionFresh(){return !!(motion&&motion.valid===true&&map&&motion.map_id===map.map_id&&map.pose_valid!==false&&!stale()&&motionAge()<motion.remaining_ms);}
 function motionText(){
  const el=$('navigationPredictionStatus');if(!el)return;
  if(!motion||!map||motion.map_id!==map.map_id){el.textContent='预测：'+(motionIssue||'等待当前动作');return;}
  if(!motion.valid){el.textContent='预测：'+({CURRENT_ZERO_ACTION:'当前零速／等待，未生成运动弧线',NO_ACTIVE_ACTION:'尚未开始或当前没有可用动作',WAIT_CURRENT_POSE:'等待当前定位',WAIT_CURRENT_PROTECTED_COMMAND:'等待本轮底盘指令'}[motion.reason]||motion.reason||'等待当前动作');return;}
  el.textContent=(motionFresh()?(motion.source==='PREVIEW'?'青色虚线：动作预览，尚未执行':'青色：当前保护后轮速预测'):'灰色：上次预测已过期，不代表正在执行')+' · 条件预测 '+Number(motion.horizon_s).toFixed(1)+'s'+(Math.abs(motion.v)<.01&&Math.abs(motion.w)>.05?' · 原地'+(motion.w>0?'左':'右')+'转':'')+(motionIssue?' · '+motionIssue:'');
 }
 window.navReceiveMotion=m=>{
  if(!dlg.open)return;try{
   if(!m||m.expected_config_id!==window.ACGR6MotionPolicy.config_hex)throw Error('动作诊断指纹不匹配：本App要求CFG31110301，请成套更新FS和RK');
   if(!m||m.schema!==1||typeof m.map_id!=='string'||!/^\d{1,20}$/.test(m.map_id)||typeof m.published_us!=='string'||!/^\d{1,20}$/.test(m.published_us)||typeof m.valid!=='boolean'||!['PROTECTED_COMMAND','PREVIEW','NONE'].includes(m.source)||!Array.isArray(m.points)||m.points.length>33||!m.points.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite))||!Number.isFinite(m.remaining_ms)||m.remaining_ms<0||m.remaining_ms>200||!Number.isFinite(m.horizon_s)||m.horizon_s<0||m.horizon_s>.80001||!Number.isFinite(m.v)||!Number.isFinite(m.w)||m.valid&&m.points.length<2)throw Error('预测轨迹格式无效');
   if(motion&&motion.map_id===m.map_id&&Number(m.published_us)<Number(motion.published_us))return;
   if(!motion||motion.map_id!==m.map_id||motion.published_us!==m.published_us){motion=m;motionAt=Date.now();}else{motion.transport_age_ms=Math.max(motionAge(),Math.max(0,Number(m.transport_age_ms)||0));motionAt=Date.now();}motionIssue='';acceptEvidence(m);if(map)update();else{motionText();avoidanceText();}
  }catch(e){window.navMotionError(e.message||'预测轨迹不可用');}
 };
 window.navMotionError=s=>{if(!dlg.open)return;motionIssue=String(s||'预测轨迹等待更新').slice(0,160);motionText();};
 function drawPrediction(){
  if(!motion||!map||motion.map_id!==map.map_id||!motion.valid||motionAge()>1000||motion.points.length<2)return;
  const fresh=motionFresh(),color=fresh?'#20d6de':'#929ca4';ctx.strokeStyle=color;ctx.lineWidth=3;if(ctx.setLineDash)ctx.setLineDash(!fresh||motion.source==='PREVIEW'?[6,5]:[]);
  const c=p=>g.worldToPixel(view,p[0],p[1],canvas.width,canvas.height);
  ctx.beginPath();motion.points.forEach((p,i)=>{const q=c(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});ctx.stroke();
  if(Math.abs(motion.v)<.01&&Math.abs(motion.w)>.05){const q=c(motion.points[0]),a=motion.points[0][2],last=motion.points[motion.points.length-1][2];ctx.beginPath();ctx.arc(q[0],q[1],22,-a,-last,motion.w>0);ctx.stroke();const end=c([motion.points[0][0]+.22*Math.cos(last),motion.points[0][1]+.22*Math.sin(last)]);ctx.beginPath();ctx.moveTo(q[0],q[1]);ctx.lineTo(end[0],end[1]);ctx.stroke();}
  if(fresh&&map.geometry&&map.geometry.length_m>0&&map.geometry.width_m>0){ctx.lineWidth=1;for(let i=0;i<motion.points.length;i+=8){const p=motion.points[i],ca=Math.cos(p[2]),sa=Math.sin(p[2]);ctx.beginPath();[[1,1],[1,-1],[-1,-1],[-1,1],[1,1]].forEach((s,k)=>{const x=s[0]*map.geometry.length_m/2,y=s[1]*map.geometry.width_m/2,q=c([p[0]+ca*x-sa*y,p[1]+sa*x+ca*y]);if(k)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});ctx.stroke();}}
  if(ctx.setLineDash)ctx.setLineDash([]);
 }
 function extent(){return map?Math.max(map.width,map.height)*map.resolution:20;}
 function draw(){
  if(!map)return;const m=map;
  if(!imageCanvas||imageCells!==m.cells||imageCanvas.width!==m.width||imageCanvas.height!==m.height){const raw=atob(m.cells);if(raw.length!==m.width*m.height)throw Error('地图数据长度错误');const image=ctx.createImageData(m.width,m.height);for(let y=0;y<m.height;y++)for(let x=0;x<m.width;x++){const q=raw.charCodeAt(y*m.width+x),i=((m.height-1-y)*m.width+x)*4,c=q===255?69:q>=65?18:232;image.data[i]=c;image.data[i+1]=c;image.data[i+2]=c;image.data[i+3]=255;}imageCanvas=document.createElement('canvas');imageCanvas.width=m.width;imageCanvas.height=m.height;imageCanvas.getContext('2d').putImageData(image,0,0);imageCells=m.cells;}
  ctx.fillStyle='#263139';if(ctx.fillRect)ctx.fillRect(0,0,canvas.width,canvas.height);else ctx.clearRect(0,0,canvas.width,canvas.height);ctx.imageSmoothingEnabled=false;
  const sx=(view.x-view.span/2-m.origin_x)/m.resolution,sy=(m.origin_y+m.height*m.resolution-view.y-view.span/2)/m.resolution,ss=view.span/m.resolution;ctx.drawImage(imageCanvas,sx,sy,ss,ss,0,0,canvas.width,canvas.height);
  const coord=p=>g.worldToPixel(view,p[0],p[1],canvas.width,canvas.height);
  function line(points,color){if(!Array.isArray(points))return;ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;points.forEach((p,i)=>{const q=coord(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});ctx.stroke();}
  function cross(p,color,r){const q=coord(p);ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(q[0]-r,q[1]);ctx.lineTo(q[0]+r,q[1]);ctx.moveTo(q[0],q[1]-r);ctx.lineTo(q[0],q[1]+r);ctx.stroke();}
  line(m.trace,'#29aefa');if(m.route_partial&&ctx.setLineDash)ctx.setLineDash([9,5]);line(m.path,'#ffac25');if(ctx.setLineDash)ctx.setLineDash([]);drawPrediction();drawNearfield();if(window.drawObjectMapMarkers)window.drawObjectMapMarkers(ctx,view,canvas.width,canvas.height,m.map_id);if(window.drawObjectScene)try{window.drawObjectScene(ctx,view,canvas.width,canvas.height,m.map_id,m.pose);}catch(error){console.error('object map overlay',error);}
  if(m.vision_target&&m.vision_target.active&&Number.isFinite(m.vision_target.target_x)&&Number.isFinite(m.vision_target.target_y))cross([m.vision_target.target_x,m.vision_target.target_y],'#c782ff',10);
  const poseColor=m.pose_valid===false?'#a2a9af':'#00bb70';
  if(Array.isArray(m.pose)){const q=coord(m.pose);ctx.fillStyle=poseColor;ctx.beginPath();ctx.arc(q[0],q[1],5,0,Math.PI*2);ctx.fill();line([m.pose,[m.pose[0]+.2*Math.cos(m.pose[2]),m.pose[1]+.2*Math.sin(m.pose[2])]],poseColor);}
  if(m.geometry&&m.geometry.length_m>0&&m.geometry.width_m>0&&Array.isArray(m.pose)){const ps=[],a=m.pose[2],ca=Math.cos(a),sa=Math.sin(a);for(const s of [[1,1],[1,-1],[-1,-1],[-1,1],[1,1]]){const x=s[0]*m.geometry.length_m/2,y=s[1]*m.geometry.width_m/2;ps.push([m.pose[0]+ca*x-sa*y,m.pose[1]+sa*x+ca*y]);}line(ps,poseColor);}
  if(m.goal_adjusted&&Array.isArray(m.requested_goal))cross(m.requested_goal,'#8ba0b5',5);if(m.has_goal)cross(m.goal,'#e53458',8);
  if(selection){const q=coord([selection.x,selection.y]);ctx.strokeStyle='#ffc34d';ctx.lineWidth=3;ctx.beginPath();ctx.arc(q[0],q[1],11,0,Math.PI*2);ctx.stroke();cross([selection.x,selection.y],'#ffc34d',16);}
  $('navigationViewStatus').textContent=`视野 ${view.span.toFixed(1)} m · ${follow?'跟随小车':'自由查看'} · 双指缩放／拖动`;$('navigationFollow').textContent=follow?'跟随：开':'跟随：关';
 }
 window.navSessionState=(state,message)=>{sessionRecovering=state!=='connected';sessionMessage=message||'会话状态更新';if(state==='connected'){requestError=null;mapReadError=null;mapFailures=0;pendingTag=null;pendingObject=null;objectMission=null;if(dlg.open){request();if(window.Rover?.navReadMotion)window.Rover.navReadMotion();}}if(map)update();else status.textContent=sessionMessage;};
 window.navRedrawObjectMarkers=()=>{if(dlg.open)draw();};
 function update(){
  if(!map)return;const m=map,isStale=stale(),mountOK=!!(m.geometry&&m.geometry.lidar_offset_verified);applyRequestReceipt(m);
  const sceneEl=$('navigationSceneStatus');if(sceneEl){const display=evidence&&evidence.map_id===m.map_id&&evidence.publication>Number(m.published_us)?{...m,...evidence.a}:m;sceneEl.textContent=m.task_state&&m.task_state.active===false?'当前没有活动导航或巡航任务；历史动作诊断不代表正在避障。':window.ACGR4SceneText(display);const routeText=window.ACGR6RouteText(display);if(routeText)sceneEl.textContent+='\n'+routeText;sceneEl.hidden=!sceneEl.textContent;}
  $('navigationVehicleStatus').textContent=isStale?'地图状态过期 · 等待更新':m.vehicle_status==='MOTOR_FEEDBACK_FAULT'?labels[m.vehicle_status]:m.pose_valid===false?'定位恢复中 · 地图与实时诊断继续更新':m.vehicle_status==='STOPPED'?(m.has_goal?'目标保留 · 当前未开始执行':m.last_stop_reason==='GOAL_REACHED'?'已到达 · 请选新目标':'没有活动导航任务 · 请选点预览'):labels[m.vehicle_status]||m.vehicle_status||'等待车辆状态';$('navigationVehicleStatus').className=!isStale&&m.pose_valid!==false&&['RUNNING','ODOMETRY_RUNNING','DETOUR_LEFT','DETOUR_RIGHT','BACKING','ALIGNING','REJOINING','SONAR_AVOIDING','PROBING','CRUISE','ALIGNMENT_HOLD'].includes(m.vehicle_status)?'vehicle-running':'vehicle-waiting';
  $('navigationCurrentReason').textContent='当前：'+(typeof m.state_zh==='string'&&m.state_zh?m.state_zh.slice(0,240):(details[m.nav_reason]||details[m.reason]||labels[m.vehicle_status]||m.nav_reason||m.reason||'等待数据'))+((m.nav_reason||m.reason)?' ['+(m.nav_reason||m.reason)+']':'');
  if(m.task_state&&m.task_state.active===false&&!m.enabled&&!m.pending_enable&&!m.has_goal)$('navigationCurrentReason').textContent='当前：尚未选择导航目标，请点地图白色空闲区域并预览；上次取消信息见历史停车记录';
  if(m.vehicle_status==='WHEELS_ZERO')$('navigationCurrentReason').textContent='所选动作尚未执行；底盘原因：'+(window.ACGR4ReasonText(m.feedback_reason)||details[m.feedback_reason]||m.feedback_reason||'轮速为零，等待底盘执行反馈')+'；任务阶段：'+$('navigationCurrentReason').textContent;
  if(m.rejected_action&&m.rejected_action.reason&&m.rejected_action.reason!=='NONE')$('navigationCurrentReason').textContent+=' · 最近候选未采用：'+(window.ACGR4ReasonText(m.rejected_action.reason)||details[m.rejected_action.reason]||m.rejected_action.reason)+' ['+m.rejected_action.reason+']';
  const age=Number.isFinite(m.last_stop_age_ms)&&m.last_stop_age_ms>=0?` · ${(m.last_stop_age_ms/1000).toFixed(1)}秒前`:'';
  $('navigationStopReason').textContent='上次停车：'+(m.last_stop_reason?(stops[m.last_stop_reason]||m.last_stop_reason):'无')+(m.last_stop_detail&&m.last_stop_detail!=='NONE'?' · '+(details[m.last_stop_detail]||m.last_stop_detail):'')+age+(m.last_stop_recovered?' · 已恢复':'');
  // 优先使用同一地图较新的5Hz反馈；新反馈无效时不退回旧轮速数据。
  const live=evidence&&evidence.map_id===m.map_id&&evidence.publication>Number(m.published_us)?evidence.feedback:null;
  const feedbackNewer=!!(evidence&&evidence.map_id===m.map_id&&evidence.publication>Number(m.published_us)&&evidence.feedback);
  const liveAge=live?live.age_ms+evidenceAge():Infinity;
  const fresh=!isStale&&(feedbackNewer?live.fresh===true&&live.age_ms>=0&&liveAge<=500:m.feedback_fresh===true&&Number.isFinite(m.feedback_age_ms)&&m.feedback_age_ms+Math.max(0,Date.now()-lastChanged)<=500);
  $('navigationWheelStatus').textContent='轮速目标 '+rpms(feedbackNewer?live.target_wheel_rpm:m.target_wheel_rpm)+' RPM\n'+(fresh?'实际 '+rpms(feedbackNewer?live.wheel_rpm:m.wheel_rpm)+' RPM':'实际 —（反馈过期或未收到）')+(feedbackNewer&&live.age_ms>=0?` · ${Math.max(0,liveAge).toFixed(0)} ms`:Number.isFinite(m.feedback_age_ms)&&m.feedback_age_ms>=0?` · ${m.feedback_age_ms} ms`:'');
  $('navigationMotionStatus').textContent='请求 '+(Array.isArray(m.requested_speed)?`${Number(m.requested_speed[0]).toFixed(3)} m/s · ${Number(m.requested_speed[1]).toFixed(3)} rad/s`:'—')+` · PWM上限 ${m.pwm_limit_percent||70}% · 轮速上限 ${Number.isFinite(m.max_rpm)?m.max_rpm:120} RPM`;
  const raw=m.raw_diagnostics,rawAge=raw?(Number(m.published_us)-Number(raw.received_us))/1000+Math.max(0,Date.now()-lastChanged):Infinity;
  if(raw&&raw.available===true&&Number.isFinite(rawAge)&&rawAge>=0&&rawAge<250&&Array.isArray(raw.effective_pwm)&&raw.effective_pwm.length===4&&raw.effective_pwm.every(x=>Number.isFinite(x)&&x>=0&&x<=2500))$('navigationMotionStatus').textContent+=' · FPGA门控后PWM '+raw.effective_pwm.map(x=>(x/25).toFixed(1)+'%').join('／')+'（比较门限，非模拟电压）';else $('navigationMotionStatus').textContent+=' · 实际PWM未回传或已过期';
  motorText(m);
  const routeCount=Array.isArray(m.path)?m.path.length:0;$('navigationRouteStatus').textContent=routeCount>1?(m.route_complete?'橙色：完整参考路线':m.route_partial?'橙色虚线：已观测的局部通道，尚未到达最终目标':'橙色：参考路线，近段持续复核')+' · '+routeCount+'个路径点':m.has_goal?'完整路线尚未生成 · '+(m.local_probe?'局部试探／绕行中':'后台规划中'):'尚未选择目标';motionText();avoidanceText();
  const recovery=m.phase_text==='BACKING'||m.phase_text==='BACK_CHECK'||m.recovery_target_m>0;
  $('navigationRecoveryStatus').textContent=recovery?`脱困 ${(Number(m.recovery_distance_m)||0).toFixed(2)} / ${(Number(m.recovery_target_m)||.25).toFixed(2)} m · 已执行${Number(m.recovery_attempts)||0}次 · 扩展后退${Number(m.extended_reverse_attempts)||0}次${m.extended_reverse_active?'（执行中）':''} · 目标${m.has_goal?'保留':'未设置'}`:`目标${m.has_goal?'保留':'未设置'} · ${m.phase_text||'等待'}`;
  $('navigationSonarStatus').textContent='近障 '+['前','左','右'].map((name,i)=>{const n=Array.isArray(m.sonar_mm)?m.sonar_mm[i]:null,sample=m.sonar_samples?.[i],age=sample?.age_known?(sample.nonzero_age_max_ms===null?'≥'+sample.nonzero_age_min_ms:'['+sample.nonzero_age_min_ms+','+sample.nonzero_age_max_ms+')')+'ms':'年龄未知';return name+' '+(fresh&&Number.isFinite(n)&&n>0?n+'mm'+((m.sonar_fresh&(1<<i))?'':'（保持）')+' '+age:'—');}).join(' / ')+(m.sonar_front_stable===false?' · 前方恢复需两笔至少40mm新样本':'');
  const src=['复位','本地按键','原自动模式','手机停止','手机手动方向','手机自动','手机控制超时','手机新会话'].filter((_,i)=>(Number(m.local_stop_sources)||0)&(1<<i));$('navigationTakeoverStatus').textContent=src.length?'最近接管：'+src.join('、'):'';
  if(pendingTag&&pendingTag.requestId&&m.vision_target&&String(m.vision_target.request_us)===pendingTag.requestId&&m.vision_target.active&&m.vision_target.tag_id===pendingTag.id&&m.vision_target.state==='PREVIEW_READY'&&Number.isFinite(m.vision_target.standoff_m)&&Math.abs(m.vision_target.standoff_m-pendingTag.standoff)<1e-6)pendingTag=null;
  const on=m.object_navigation;
  if(pendingObject&&pendingObject.request&&on&&on.request_us===pendingObject.request&&on.epoch===pendingObject.epoch&&on.track_id===pendingObject.track_id&&on.association_id===pendingObject.association_id){objectMission=on; if(on.state==='PREVIEW_READY'||on.state==='REJECTED'||on.state==='CANCELLED')pendingObject=null;}
  if(on&&on.active){$('navigationObjectMission').textContent='实物任务 #'+on.track_id+' · '+on.state+' · '+on.reason;objectMission=on;}
  else if(on&&on.active===false&&!pendingObject)objectMission=null;
  const association=window.currentObjectAssociation&&window.currentObjectAssociation(),objectOK=!objectMission||objectMission.state==='PREVIEW_READY'&&association&&association.epoch===objectMission.epoch&&association.track_id===objectMission.track_id&&association.association_id===objectMission.association_id&&association.map_id===m.map_id;

  const usable=poseFresh(m)&&m.nav_usable!==false&&(m.state===1||((m.continuous_test===true||m.odometry_nav_test===true)&&m.nav_usable===true&&m.odometry_fallback===true&&m.state===2)),routeReady=Array.isArray(m.path)&&m.path.length>1;
  start.textContent=m.cruise_mode?(m.auto_intent_accepted===true?'自动任务已接收':'开始自动模式'):'开始导航';
  const sessionBlocked=sessionRecovering||(requestError&&['AUTH_REQUIRED','SESSION_EXPIRED'].includes(requestError.code));
  const authoritative=typeof m.start_allowed==='boolean',locallyReady=mountOK&&m.drive_capable&&m.calibrated&&usable&&(m.cruise_mode?m.can_start_local===true:m.has_goal&&(routeReady||m.can_start_local===true));
  // 原因文字与原因码成对显示，后端原因码不应
  // 拼接到无关的手机端时效或选点提示后面。
  let startBlocker='',startBlockerCode='';
  const block=(code,message)=>{startBlockerCode=code||'START_NOT_READY';startBlocker=message;};
  if(mapIntegrityBlocked())block(mapReadError&&mapReadError.code||'MAP_FILE_INVALID',errorText(mapReadError));
  else if(sessionBlocked)block('PHONE_SESSION_RECONNECTING','手机控制会话正在重新连接');
  else if(pendingAction)block('REQUEST_ACK_PENDING','请求#'+pendingAction.id+'等待板端回执');
  else if(pendingTag)block('TAG_PREVIEW_PENDING','标记预览尚未确认');
  else if(pendingObject)block('OBJECT_PREVIEW_PENDING','实物预览尚未确认');
  else if(!objectOK)block('OBJECT_TARGET_CHANGED','实物目标状态已改变，请重新预览');
  else if(selection)block(selection.submitted?'GOAL_SUBMITTED_WAIT_ACK':'GOAL_PREVIEW_REQUIRED',selection.submitted?'候选目标正在提交':'黄色候选尚未预览，请点“预览路线”');
  else if(m.enabled)block('NAVIGATION_ACTIVE','导航任务已启动，请看当前状态');
  else if(m.pending_enable)block('START_PENDING','导航已接收，等待实际执行条件');
  else if(!poseFresh(m))block('WAIT_FRESH_LOCALIZATION','当前运动位姿尚未就绪或超过500毫秒，等待实时定位'+(m.localizer_reason==='RECOVERY_AMBIGUOUS'?'；雷达匹配找到多个相似位置，尚不能确定车位':''));
  else if(authoritative&&!m.start_allowed)block(m.start_blocker,requestReason(m.start_blocker));
  else if(!authoritative&&!locallyReady){
   if(!mountOK)block('LIDAR_OFFSET_NOT_VERIFIED','雷达安装位置尚未确认');
   else if(!m.drive_capable)block('DRIVE_CAPABILITY_REQUIRED','当前只读预览；请在RK执行 sh begin drive');
   else if(!m.calibrated)block('UNCALIBRATED_ARM_DENIED','里程计配置尚未确认');
   else if(!usable)block('WAIT_FRESH_LOCALIZATION','当前定位不能用于驾驶');
   else if(!m.has_goal&&!m.cruise_mode)block('SELECT_AND_PREVIEW_GOAL_FIRST','尚未选择目标，请先选点并预览');
   else block('ROUTE_NOT_READY','完整路线和局部动作尚未就绪');
  }
  start.disabled=!!startBlocker;
  preview.disabled=!!(mapIntegrityBlocked()||sessionBlocked||pendingAction||!selection||selection.submitted);
  hint.textContent=m.enabled?'导航已使能 · 随时可停车':!m.drive_capable?'路线预览；真实导航需 sh begin drive':!m.calibrated?'里程计未标定，不能开始驾驶':'驾驶模式已加载；预览路线后明确开始';if(m.odometry_fallback===true)hint.textContent+=' · 里程计续行最长2秒';if(!mountOK)hint.textContent+=' · 雷达安装位置未确认';if(m.geometry&&m.geometry.narrow)hint.textContent+=' · 窄道按当前场景限速';
  if(m.has_goal){const why=m.nav_reason&&m.nav_reason!=='NONE'&&m.nav_reason!=='WAIT_TIMEOUT'?m.nav_reason:m.nav_blocking;goalText.textContent=(m.goal_adjusted?'目标已调整至邻近可通行空地；':'目标已被板端接收；')+'路径点数 '+(Array.isArray(m.path)?m.path.length:0)+(why&&why!=='NONE'?' · '+(details[why]||why):'');}else if(m.goal_status&&/STALE_MAP|UNKNOWN|OCCUPIED|OUTSIDE|NOT_TRACKING|NO_NEARBY/.test(m.goal_status))goalText.textContent='目标未接受：'+m.goal_status+'，请重新选点';
  if(selection){const at=`X=${selection.x.toFixed(2)}m Y=${selection.y.toFixed(2)}m`;goalText.textContent=selection.submitted?`黄色候选目标 ${at} · 已提交，等待板端接受`:`黄色候选目标 ${at} · 可微调，点“预览路线”提交`;if(selection.submitted&&m.goal_status&&/STALE_MAP|UNKNOWN|OCCUPIED|OUTSIDE|NOT_TRACKING/.test(m.goal_status))goalText.textContent+=' · '+m.goal_status;}
  for(const id of ['navigationNudgeXMinus','navigationNudgeXPlus','navigationNudgeYMinus','navigationNudgeYPlus'])$(id).disabled=!selection||selection.submitted;
  const loc={MAP_SEED_CONFIRMING:'等待第二笔扫描确认初始地图',RECOVERY_WAIT_STILL:'等待车身静止以恢复定位',RECOVERY_RETRY_WAIT:'定位恢复重试',RECOVERY_WAIT_SCAN:'匹配地图',RECOVERY_CONFIRMING:'确认定位',LOST_REINITIALIZE:'定位失锁',MATCH_AGE_PLUS_TIME_UNCERTAINTY:'定位匹配超时'};status.textContent='地图 '+m.map_version+' · '+(m.odometry_fallback===true?'里程计续行（最长2秒）':m.state===1?'定位跟踪':'等待有效传感器')+' · '+(loc[m.localizer_reason]||m.localizer_reason||m.reason||'等待数据');
  const policy=Number(m.policy_revision||(evidence&&evidence.a.policy_revision)||0);status.textContent+=' · 策略S'+(policy||'未知');
  status.textContent+=' · 地图年龄 '+Math.max(0,(Date.now()-lastChanged)/1000).toFixed(1)+'秒'+(isStale?'（保留历史图；运动由RK实时判断）':'');
  if(sessionMessage)status.textContent+=' · '+sessionMessage;
  if(m.front_approach===true||(evidence&&evidence.a.front_approach===true))status.textContent+=' · 提前绕行：正在比较左右出口';
  if(m.cruise_mode)hint.textContent=m.enabled?'自动模式执行中 · 停车按钮随时可用':'自动模式 · 明确点击开始，不自动启动；选点预览可切换目标导航';
  else if(m.has_goal&&!routeReady&&m.can_start_local===true)hint.textContent='完整路线尚未就绪；可明确开始局部试探，目标仍保留';
  if(m.match_diagnostics){const d=m.match_diagnostics;status.textContent+=` · 匹配 ${Number(d.reference_points)||0}/${Number(d.input_points)||0}点 · 越界${Number(d.outside_points)||0} · 扩图${Number(d.map_expansions)||0}次`;}
  if(mapReadError)status.textContent+=' · '+(mapFailures>=3?'地图连续读取异常：':'本次地图读取未更新：')+errorText(mapReadError)+` · 保留上次地图（${Math.max(0,(Date.now()-lastChanged)/1000).toFixed(1)}秒前）`;
  if(m.path_pending_recheck===true)status.textContent+=' · 显示保留路线，近段持续复核';
  if(m.map_reference_pending===true)status.textContent+=' · 匹配参考图后台更新中';
  if(m.pose_valid===false){status.textContent+=' · '+(m.pose_retained===true&&Array.isArray(m.pose)?`灰色车位为上次定位（${Number.isFinite(m.pose_age_ms)&&m.pose_age_ms>=0?((m.pose_age_ms+Math.max(0,Date.now()-lastChanged))/1000).toFixed(1)+'秒前':'时间未知'}）`:'当前没有可信车位')+' · '+(m.recovery_state||m.localizer_reason||'等待恢复');hint.textContent='地图保留、实时状态持续更新；等待可信定位恢复，当前不能开始驾驶';}
  if(m.provisional_route===true)goalText.textContent+=' · 暂定路线包含待观测区域，不能据此保证可达';
  if(m.has_goal&&m.target_observation_pending===true)goalText.textContent+=' · 目标区域尚待观测，不代表已确认可达';
  if(m.recovery_diagnostics&&Number(m.recovery_diagnostics.state)>0){const d=m.recovery_diagnostics,names=['正常','等待稳定观测','分批搜索恢复位置','候选位置待新扫描确认'];status.textContent+=` · ${names[Number(d.state)]||'定位恢复'} ${Number(d.trials)||0}/${Number(d.total_trials)||0}候选 · 歧义${Number(d.ambiguous)||0}次`;}
  if(m.rear_reason&&m.rear_reason!=='NONE')$('navigationRecoveryStatus').textContent+=' · '+(details[m.rear_reason]||m.rear_reason);
  if(requestError)goalText.textContent+=' · '+errorText(requestError)+(requestError.code==='REQUEST_TIMEOUT'?'；请求可能已送达，以板端状态为准':'；未据此判定轮速故障');
  const vt=tagsVisible()?m.vision_target:null;
  if(vt&&vt.active)$('navigationTagMission').textContent='标记任务 ID '+vt.tag_id+' · '+(visionStates[vt.state]||vt.state||'等待')+(vt.reason?' · '+visionText(vt.reason):'');
  else if(vt&&vt.state)$('navigationTagMission').textContent='标记任务：'+(visionStates[vt.state]||vt.state)+(vt.reason?' · '+visionText(vt.reason):'');
  if(on&&on.state)$('navigationObjectMission').textContent='实物任务 #'+on.track_id+' · '+on.state+' · '+on.reason;
  if(pendingObject)goalText.textContent='实物预览排队 / 待板端接受；尚未开始';
  if(sessionRecovering){start.disabled=true;preview.disabled=true;hint.textContent='正在重连查看会话，旧图保留；重连不会自动启动运动';}
  if(startBlocker)hint.textContent='开始条件：'+startBlocker+' ['+startBlockerCode+']';
  else hint.textContent='可点击开始：提交导航任务；实际动作持续核对当前净空和车身扫掠';
  if(m.motion_ready===false&&(m.enabled||m.pending_enable||m.has_goal))hint.textContent+='\n当前执行等待：'+requestReason(m.motion_blocker)+' ['+(m.motion_blocker||'UNKNOWN')+']；目标保留';
  if(!isStale&&m.pose_valid!==false&&m.vehicle_status!=='MOTOR_FEEDBACK_FAULT'&&m.auto_intent_accepted===true&&m.auto_waiting===true)$('navigationVehicleStatus').textContent='自动任务已接收 · 等待可行动作';
  const receipt=$('navigationRequestStatus');if(receipt){const p=pendingAction,a=lastRequestReceipt;receipt.textContent=p?'请求#'+p.id+' · '+(operationNames[p.action]||p.action)+' · '+(p.queued?'已排队，等待板端接受':'正在发送')+' · '+Math.max(0,(Date.now()-p.startedAt)/1000).toFixed(1)+'秒':a?'请求#'+a.id+' · '+({ACCEPTED:'板端已接受',REJECTED:'板端已拒绝',ARMING:'任务已接收，等待执行许可',ACTIVE:'任务执行中'}[a.state]||a.state)+' · '+requestReason(a.reason):'本次未提交导航请求';}
  updateTags();draw();
 }
 function selectAt(x,y){if(!map||mapIntegrityBlocked())return;pendingObject=null;objectMission=null;requestError=null;pendingTag=null;const p=g.pixelToWorld(view,x,y,canvas.width,canvas.height);selection={x:p[0],y:p[1],map_id:map.map_id,map_version:map.map_version,submitted:false};preview.disabled=false;start.disabled=true;update();}
 function zoom(f,x=canvas.width/2,y=canvas.height/2){view=g.zoom(view,f,x,y,canvas.width,canvas.height,.5,Math.max(4,extent()*2));follow=false;draw();}
 function local(e){const r=canvas.getBoundingClientRect();return{x:(e.clientX-r.left)/r.width*canvas.width,y:(e.clientY-r.top)/r.height*canvas.height};}
 function resetGesture(){const a=Array.from(pointers.values());if(a.length>=2){const p=a[0],q=a[1];gesture={pinch:true,view:{...view},x:(p.x+q.x)/2,y:(p.y+q.y)/2,d:Math.max(1,Math.hypot(p.x-q.x,p.y-q.y))};}else if(a.length)gesture={pinch:false,view:{...view},x:a[0].x,y:a[0].y,moved:false};else gesture=null;}
 canvas.addEventListener('pointerdown',e=>{if(!map)return;if(e.preventDefault)e.preventDefault();pointers.set(e.pointerId,local(e));if(pointers.size>=2)multiTouch=true;if(canvas.setPointerCapture)canvas.setPointerCapture(e.pointerId);resetGesture();});
 canvas.addEventListener('pointermove',e=>{if(!pointers.has(e.pointerId)||!gesture)return;pointers.set(e.pointerId,local(e));const a=Array.from(pointers.values());if(a.length>=2&&gesture.pinch){const p=a[0],q=a[1],x=(p.x+q.x)/2,y=(p.y+q.y)/2,d=Math.max(1,Math.hypot(p.x-q.x,p.y-q.y)),anchor=g.pixelToWorld(gesture.view,gesture.x,gesture.y,canvas.width,canvas.height),s=Math.max(.5,Math.min(Math.max(4,extent()*2),gesture.view.span*gesture.d/d));view={x:anchor[0]-(x/canvas.width-.5)*s,y:anchor[1]-(.5-y/canvas.height)*s,span:s};follow=false;draw();}else if(!gesture.pinch){const p=a[0],dx=p.x-gesture.x,dy=p.y-gesture.y;if(Math.hypot(dx,dy)>6)gesture.moved=true;if(gesture.moved){view={x:gesture.view.x-dx/canvas.width*gesture.view.span,y:gesture.view.y+dy/canvas.height*gesture.view.span,span:gesture.view.span};follow=false;draw();}}});
 canvas.addEventListener('pointerup',e=>{if(!pointers.has(e.pointerId))return;const tap=gesture&&!gesture.pinch&&!gesture.moved&&!multiTouch&&pointers.size===1,p=local(e);pointers.delete(e.pointerId);ignoreClickUntil=Date.now()+500;if(tap)selectAt(p.x,p.y);resetGesture();if(!pointers.size)multiTouch=false;});
 canvas.addEventListener('pointercancel',e=>{pointers.delete(e.pointerId);ignoreClickUntil=Date.now()+500;resetGesture();if(!pointers.size)multiTouch=false;});
 canvas.addEventListener('click',e=>{if(Date.now()<ignoreClickUntil)return;const p=local(e);selectAt(p.x,p.y);});
 canvas.addEventListener('wheel',e=>{if(e.preventDefault)e.preventDefault();const p=local(e);zoom(e.deltaY<0?.8:1.25,p.x,p.y);},{passive:false});
 $('navigationZoomIn').addEventListener('click',()=>zoom(.8));$('navigationZoomOut').addEventListener('click',()=>zoom(1.25));
 $('navigationFollow').addEventListener('click',()=>{follow=!follow;if(follow&&map&&Array.isArray(map.pose)){view.x=map.pose[0];view.y=map.pose[1];}draw();});
 $('navigationHome').addEventListener('click',()=>{follow=true;if(map&&Array.isArray(map.pose)){view.x=map.pose[0];view.y=map.pose[1];view.span=Math.min(4,extent());}draw();});
 $('navigationFit').addEventListener('click',()=>{if(!map)return;follow=false;view={x:map.origin_x+map.width*map.resolution/2,y:map.origin_y+map.height*map.resolution/2,span:extent()};draw();});
 for(const [id,dx,dy] of [['navigationNudgeXMinus',-.025,0],['navigationNudgeXPlus',.025,0],['navigationNudgeYMinus',0,-.025],['navigationNudgeYPlus',0,.025]])$(id).addEventListener('click',()=>{if(!selection||selection.submitted)return;selection.x+=dx;selection.y+=dy;update();});
 $('openNavigationMap').addEventListener('click',()=>{if(!dlg.open)dlg.showModal();request();if(window.Rover?.navReadMotion)window.Rover.navReadMotion();if(!timer)timer=setInterval(()=>{request();if(map)update();},1000);if(!motionTimer)motionTimer=setInterval(()=>{if(window.Rover?.navReadMotion)window.Rover.navReadMotion();if(map)update();},200);});$('closeNavigationMap').addEventListener('click',()=>dlg.close());
 function cancel(){if(window.Rover){window.Rover.command('stop','');if(map)window.Rover.navStop(map.map_id,map.map_version,0,0);}}
 dlg.addEventListener('close',()=>{/* 关闭视图不等于执行停车操作。 */if(timer)clearInterval(timer);if(motionTimer)clearInterval(motionTimer);timer=motionTimer=null;motion=null;motionAt=0;motionIssue="";map=null;selection=null;pendingTag=null;pendingObject=null;objectMission=null;pendingAction=null;lastRequestReceipt=null;initialized=false;pointers.clear();gesture=null;lastPublished=null;lastChanged=0;mapReadError=null;requestError=null;preview.disabled=true;start.disabled=true;});
 $('stopNavigationDrive').addEventListener('click',()=>{selection=null;pendingObject=null;pendingTag=null;objectMission=null;cancel();start.disabled=true;goalText.textContent='已请求停车并取消目标；下一次运行需重新选点预览';});
 $('resetNavigationMap').addEventListener('click',()=>{if(!map||!window.Rover)return;window.Rover.navResetMap(map.map_id,map.map_version);start.disabled=true;preview.disabled=true;selection=null;goalText.textContent='已请求停车并重建；等待新地图后重新选点';});
 start.addEventListener('click',()=>{if(!map||start.disabled||!window.Rover)return;requestError=null;start.disabled=true;const goal=Array.isArray(map.goal)?map.goal:[0,0];if(map.cruise_mode)window.Rover.command("auto","");else window.Rover.navStart(map.map_id,map.map_version,goal[0],goal[1]);goalText.textContent='已明确请求开始；等待板端保护检查';});
 preview.addEventListener('click',()=>{if(!selection||preview.disabled||!window.Rover||!map||selection.map_id!==map.map_id)return;selection.submitted=true;selection.submittedAt=Date.now();window.Rover.navPreviewGoal(selection.map_id,map.map_version,selection.x,selection.y);preview.disabled=true;start.disabled=true;goalText.textContent='目标已提交，等待板端检查和规划';});
 $('navigationTagId').addEventListener('change',updateTags);
 $('previewNavigationTag').addEventListener('click',()=>{if(!map||!window.Rover||$('previewNavigationTag').disabled)return;pendingObject=null;objectMission=null;const id=Number($('navigationTagId').value),distance=Number($('navigationTagStandoff').value);if(!Number.isInteger(id)||!Number.isFinite(distance)||distance<.2||distance>3){$('navigationTagStatus').textContent='停车距离应为0.20～3.00米';return;}selection=null;pendingTag={id,standoff:distance,frameTime:Number(tagsSnapshot&&tagsSnapshot.frame_time_us)||0};preview.disabled=true;start.disabled=true;$('previewNavigationTag').disabled=true;window.Rover.navPreviewTag(map.map_id,map.map_version,id,distance);goalText.textContent='标记目标已提交，等待可达性检查；尚未启动运动';});
 window.ACGNavActions={
 previewObject(a,d){if(!window.Rover||!window.Rover.navPreviewObject)return false;$('openNavigationMap').click();selection=null;pendingTag=null;objectMission=null;pendingObject={epoch:a.epoch,track_id:a.track_id,association_id:a.association_id,request:null};start.disabled=true;preview.disabled=true;window.Rover.navPreviewObject(a.map_id,a.map_version,a.epoch,a.track_id,a.association_id,d);goalText.textContent='实物预览已提交，等待板端接受；尚未启动';return true;},
 start(){if(map)update();if(start.disabled)return false;start.click();return true;},cancel(){cancel();start.disabled=true;pendingObject=null;objectMission=null;},status(){return map?map.object_navigation||{state:map.state}:null;}
 };
 window.objectNavigationQueued=id=>{if(pendingObject)pendingObject.request=id;goalText.textContent='实物请求已排队；等待板端接受，排队不代表路线就绪';request();};
 window.objectNavigationError=message=>{pendingObject=null;objectMission={state:'REJECTED'};start.disabled=true;goalText.textContent=message||'实物路线拒绝';};
 window.navReceiveTags=data=>{if(!dlg.open||!tagsVisible())return;if(!data||data.version!==1||!Array.isArray(data.tags)||data.tags.length>16){window.navTagsError('标记数据格式不兼容');return;}tagsSnapshot=data;tagsReceivedAt=Date.now();updateTags();};
 window.navTagsError=message=>{if(!tagsVisible())return;tagsSnapshot=null;tagsReceivedAt=0;$('previewNavigationTag').disabled=true;$('navigationTagStatus').textContent=message||'标记服务未就绪';};
 window.navGoalSending=(action,id)=>{if(!dlg.open)return;if(!validRequestId(id)){window.navRequestError({operation:action,code:'RESPONSE_INVALID'});return;}pendingAction={action,id,startedAt:Date.now(),queued:false};requestError=null;if(action==='goal'&&selection&&selection.submitted)selection.requestId=id;if(map)update();};
 window.navGoalQueued=(action,requestId,visionRequest)=>{if(!dlg.open)return;if(!validRequestId(requestId)){window.navRequestError({operation:action,code:'RESPONSE_INVALID'});return;}if(pendingAction&&(pendingAction.id!==requestId||pendingAction.action!==action))return;if(!pendingAction&&lastRequestReceipt&&lastRequestReceipt.id===requestId)return;requestError=null;if(!pendingAction)pendingAction={action,id:requestId,startedAt:Date.now(),queued:true};pendingAction.queued=true;if(action==='goal'&&selection&&selection.submitted)selection.requestId=requestId;if(action==='tag'&&pendingTag){if(!validRequestId(visionRequest)){window.navTagsError('标记请求确认格式不兼容，请更新配套RK程序');return;}pendingTag.requestId=visionRequest;}goalText.textContent=action==='start'?'开始请求已排队，等待板端最终回执':action==='stop'?'取消请求已排队':'目标请求已排队；排队不代表已经接受';if(map)update();request();};
 window.navRequestError=e=>{if(!dlg.open)return;e=e&&typeof e==='object'?e:{operation:'map',code:'NETWORK_ERROR',message:String(e||'请求失败')};try{if(e.operation==='map'){mapFailures++;mapReadError=e;if(map)update();else{status.textContent=errorText(e)+' · 尚无可保留地图';start.disabled=true;preview.disabled=true;}}else{requestError=e;if(e.operation==='tag')pendingTag=null;if(e.operation==='goal'&&selection)selection.submitted=false;if(pendingAction&&pendingAction.action===e.operation)pendingAction=null;if(map)update();else goalText.textContent=errorText(e);}}catch(ignored){status.textContent=errorText(e)+' · 保留上次画面';start.disabled=true;preview.disabled=true;}};
 window.navMapError=s=>window.navRequestError({operation:'map',code:'MAP_FORMAT_INVALID',message:s||'地图显示失败'});
 window.navReceiveMap=m=>{if(!dlg.open)return;const previous=map;try{
  if(!m||m.expected_config_id!==window.ACGR6MotionPolicy.config_hex)throw Error('地图指纹不匹配：本App要求CFG31110301，请成套更新FS和RK');
  if(!m||typeof m.map_id!=='string'||!/^\d{1,20}$/.test(m.map_id)||typeof m.map_version!=='string'||!/^\d{1,20}$/.test(m.map_version))throw Error('地图标识无效');
  if(m.schema!==1||!Number.isInteger(m.width)||!Number.isInteger(m.height)||m.width<1||m.height<1||m.width>1000||m.height>1000||!Number.isFinite(m.resolution)||m.resolution<=0||!Number.isFinite(m.origin_x)||!Number.isFinite(m.origin_y))throw Error('地图格式错误');
  if(typeof m.cells!=='string'||m.cells.length>1333336||atob(m.cells).length!==m.width*m.height)throw Error('地图栅格数据无效');
  if(m.pose!==null&&(!Array.isArray(m.pose)||m.pose.length<3||!m.pose.slice(0,3).every(Number.isFinite)))throw Error('地图位置数据无效');
  if(m.pose===null&&m.pose_valid!==false)throw Error('缺失车位必须明确标为无效');
  const pointValid=p=>Array.isArray(p)&&p.length>=2&&p.slice(0,2).every(Number.isFinite);
  for(const list of [m.trace,m.path])if(list!==undefined&&(!Array.isArray(list)||list.length>4096||!list.every(pointValid)))throw Error('地图轨迹或路线无效');
  if((m.has_goal&&!pointValid(m.goal))||(m.goal_adjusted&&!pointValid(m.requested_goal)))throw Error('目标坐标无效');
  if(m.geometry&&['length_m','width_m','margin_m'].some(k=>m.geometry[k]!==undefined&&!Number.isFinite(m.geometry[k])))throw Error('车身几何无效');
  if((map&&map.map_id!==m.map_id)||(selection&&selection.map_id!==m.map_id)){selection=null;initialized=false;motion=null;motionAt=0;preview.disabled=true;goalText.textContent='新地图已建立，请重新选点';}map=m;
  if(!initialized){view={x:Array.isArray(m.pose)?m.pose[0]:m.origin_x+m.width*m.resolution/2,y:Array.isArray(m.pose)?m.pose[1]:m.origin_y+m.height*m.resolution/2,span:Math.min(4,extent())};initialized=true;follow=true;}if(follow&&Array.isArray(m.pose)){view.x=m.pose[0];view.y=m.pose[1];}
  if(!m.published_us||m.published_us!==lastPublished){lastPublished=m.published_us;lastChanged=Date.now()-Math.max(0,Number.isFinite(m.transport_age_ms)?m.transport_age_ms:0);}
  // 新服务提供关联到请求的最终回执，旧快照仅兼容显示，
  // 不能凭位置确认另一笔请求已执行。
  applyRequestReceipt(m);
  acceptEvidence(m);if(m.motion_preview)window.navReceiveMotion({...m.motion_preview,transport_age_ms:Math.max(0,Number(m.transport_age_ms)||0)});mapReadError=null;mapFailures=0;if(requestError&&['AUTH_REQUIRED','SESSION_EXPIRED'].includes(requestError.code))requestError=null;update();
 }catch(e){map=previous;window.navMapError(e.message||'地图显示失败');}};
})();
