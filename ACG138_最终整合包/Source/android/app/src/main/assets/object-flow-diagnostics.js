"use strict";
/* 只读显示，不调用控制接口，不修改观测或源时间。 */
(() => {
 const finite=x=>typeof x==='number'&&Number.isFinite(x),own=(o,k)=>o&&Object.prototype.hasOwnProperty.call(o,k),obj=x=>x&&typeof x==='object'&&!Array.isArray(x)?x:null;
 const fmt=(x,d=2)=>finite(x)?x.toFixed(d):'未知',positive=x=>finite(x)&&x>=0;
 const sources={RADAR:'激光雷达',LIDAR:'激光雷达',RADAR_STEREO:'雷达＋双目',FUSED:'多源融合',STEREO:'双目',STEREO_RAW:'双目原始表面',STEREO_COARSE:'双目粗距',RAW_SURFACE:'目标表面实测',VISUAL:'视觉',MEMORY:'历史记忆',PREDICTED:'预测',RADAR_VISUAL:'雷达＋视觉',NONE:'未提供'};
 const kinds={MEASURED_FILTERED:'实测更新后的滤波估计',OBSERVED:'测量更新后的估计',MEASURED:'测量更新后的估计',FILTERED:'滤波估计',PREDICTED:'短时预测',PREDICTION:'短时预测',MEMORY:'历史记忆',RETAINED:'历史记忆',NONE:'尚无估计',UNKNOWN:'质量未知'};
 const text=(v,labels)=>typeof v==='string'&&v?labels[v]||v:'未上报';
 const stamp=x=>typeof x==='string'&&/^[0-9]{1,20}$/.test(x)&&Number.isSafeInteger(Number(x))?Number(x):finite(x)&&Number.isSafeInteger(x)?x:null;
 function extraAge(t,elapsed){return [t.age_ms,t.client_delay_ms,elapsed].every(x=>x===undefined||positive(x))?(positive(t.age_ms)?t.age_ms:0)+(positive(t.client_delay_ms)?t.client_delay_ms:0)+(positive(elapsed)?elapsed:0):null;}
 function observationAge(t,o,elapsed){const p=stamp(t.published_us),s=stamp(o.source_us),extra=extraAge(t,elapsed);return p!==null&&s!==null&&s>0&&p>=s&&extra!==null?(p-s)/1000+extra:null;}
 function fallback(e,key,t,old){return own(e,key)?e[key]:t[old];}
 function block(t,e,elapsed){
  const r=typeof e.reason==='string'&&e.reason?e.reason:t.runtime_reason||t.navigation_reason||t.reason||'未上报原因';let layer='',recover='';
  if(!t.active&&!t.prepared&&!t.restart_required&&!t.resume_requires_start){layer=t.state==='CANCELLED'?'任务已结束':'待双击目标';recover='双击具体实物发起新任务；当前未请求行驶';}
  else if(t.drive_capable===false){layer='运行入口：仅预览';recover='使用 sh begin drive 启动允许执行的会话后，再明确双击目标';}
  else if(t.state==='PAUSED'){layer='任务：跟随已暂停';recover='点击继续跟随；关闭窗口不会取消任务';}
  else if(t.arrived===true||t.state==='ARRIVED'){layer='任务：静止目标到达保持';recover='保持停车；新的双击任务才重新执行';}
  else if(t.feedback_faults>0||t.resume_requires_start===true){layer='底盘反馈保护';recover=t.can_restart===true?'健康条件已满足，可双击原目标明确重新接管':'等待健康反馈及板端允许恢复，不自动重新起步';}
  else if(/LEASE|OWNER|SESSION|REVOKED|FENCE/.test(r)){layer='控制权 / 会话';recover='确认手机仍连接且持有当前会话，再双击目标；STOP/手动接管始终优先';}
  else if(/CLOCK|TIME_REVERSED|CONTEXT/.test(r)){layer='时间 / 坐标上下文';recover='等待板端时间与坐标上下文恢复；以具体原因码核查';}
  else if(t.pending_enable===true||/WAIT.*ACK|ACK_PENDING|ENABLE_PENDING/.test(r)){layer='底盘：正常等待回执';recover='使能/请求正在确认；继续查看板端回执、最终下发与实测轮速，不重复发送START';}
  else if(/RAW_SCAN|ODOMETRY|RAW_SOURCE|SENSOR|FEEDBACK|TRANSPORT|ACK/.test(r)){layer='原始传感器 / 底盘链路';recover='等待新的雷达、里程计或底盘回执；任务保留不等于已经行驶';}
  else if(/OBSTACLE|COLLISION|SWEEP|SONAR|NEAR|FEASIBLE|BLOCKED/.test(r)){layer='局部避障 / 动作终检';recover='继续检查新净空或其他可行动作；没有放宽原障碍保护';}
  else if(/VERIFY|HOLD_DISTANCE|TOO_CLOSE|WAIT_DISTANCE|ZERO_DESIRED/.test(r)||t.state==='WAIT_DISTANCE'){layer='任务：距离保持 / 停靠复核';recover=t.mode==='follow'?'目标远离后按同一目标的新观测接续；近处不自动倒车':'等待新表面观测复核停靠，不凭记忆距离宣布到达';}
  else if(/TARGET|CANDIDATE|DISTANCE|MEASUREMENT|REACQUIRE/.test(r)||['WAIT_TARGET','CHECK_CANDIDATE','SEARCHING'].includes(t.state)){layer='目标观测 / 身份 / 估计';recover='按板端目标记忆或找回策略执行；新候选不等于已认回原目标';}
  else if(t.enabled===false){layer=t.pending_enable===true?'底盘：等待使能回执':'底盘：尚未使能';recover=t.can_restart===true?'可再次双击原目标明确恢复执行':'等待板端允许执行；查看使能与具体链路原因';}
  else{layer='执行状态';recover='以最终下发指令与实测轮速共同判断，任务active不是车轮运动证明';}
  if(typeof e.blocked_layer==='string'&&e.blocked_layer)layer=text(e.blocked_layer,{TARGET:'目标观测',SENSORS:'原始传感器',CONTROL:'控制权 / 会话',EXECUTION:'执行终检',FEEDBACK:'底盘反馈',CHASSIS:'底盘执行',NONE:'无阻断',TASK:'任务保持'});
  if(typeof e.recovery_condition_zh==='string'&&e.recovery_condition_zh)recover=e.recovery_condition_zh;else if(typeof e.recovery_condition==='string'&&e.recovery_condition)recover=e.recovery_condition;
  const stale=extraAge(t,elapsed);if(stale!==null&&stale>1800){layer='状态历史：当前执行未确认';recover='等待新的任务发布；下列为最后收到的诊断，不能作为当前可执行证明';}
  const cd=obj(t.control_diagnostics)||{};return '阻断 / 保持层：'+layer+'\n具体原因：'+r+'\n恢复条件：'+recover+(typeof cd.cancel_origin==='string'?'\n最近取消来源：'+cd.cancel_origin+' · 原始时间 '+(typeof cd.cancel_origin_us==='string'?cd.cancel_origin_us:'未知')+'us':'');
 }

 /* 只读证据显示，评分与提示不转换成观测或控制命令。 */
 const followReasons={FOLLOW_SPARSE_TRACK:'远距离单点弱支持维持原目标轨迹，不等于确认到达',FOLLOW_MOVING_TRAJECTORY_PRIORITY:'持续移动目标优先：仍须符合原目标方向和轨迹',FOLLOW_SPARSE_WEAK_SUPPORT:'远距离稀疏回波维持原目标轨迹，只是弱测量，不确认到达',FOLLOW_RADAR_GOAL_UPDATED:'同目标新雷达观测已更新方位，不是沿旧位置旋转',FOLLOW_MOVING_CANDIDATE:'运动目标候选：真实扫描支持其独立移动',FOLLOW_STATIONARY_CANDIDATE:'人物静止保持：仍允许静止目标表面参与关联',FOLLOW_WALL_EXCLUDED:'墙面疑似已排除，避免将背景墙当成跟随目标',FOLLOW_RADAR_TRACK:'正在使用同一目标的新雷达观测',FOLLOW_REASSOCIATING_NO_SURFACE:'正在修正关联：当前没有可信目标表面，保留真实旧估计并寻找',FOLLOW_SCAN_DUPLICATE_OR_OLD:'扫描重复或倒序，没有增加观测次数或刷新源时间',
  FOLLOW_WALL_SUSPECT:'墙面疑似：正在区分背景墙与原目标表面',FOLLOW_WALL_REJECTED:'已排除墙面候选，继续寻找目标表面',FOLLOW_ASSOCIATION_CORRECTING:'正在修正雷达与原目标的关联',FOLLOW_ASSOCIATION_CORRECTED:'已修正关联，旧距离历史不再作为新测量',FOLLOW_STATIONARY_PERSON_HOLD:'人物静止保持：运动证据为0也仍可继续关联',FOLLOW_STATIONARY_TARGET_HOLD:'目标静止保持，不要求目标必须运动',FOLLOW_EXIT_LEFT_DIRECTED:'原目标从画面左侧离开，按补偿后的原始关键帧方向找回',FOLLOW_EXIT_RIGHT_DIRECTED:'原目标从画面右侧离开，按补偿后的原始关键帧方向找回',CURRENT_REAL_TARGET_REAPPEARED:'已重新获得真实图像观测，旧离画面提示结束',REAL_SEQUENCE_LEFT_EDGE_EXIT_RAW_UNCOMPENSATED:'连续真实图像支持从左边缘离开；原始方向尚未做车身转动补偿',REAL_SEQUENCE_RIGHT_EDGE_EXIT_RAW_UNCOMPENSATED:'连续真实图像支持从右边缘离开；原始方向尚未做车身转动补偿',CENTER_MISS_NOT_DIRECTIONAL_EXIT:'画面中间漏检，不能判断向左或向右离开',NEAR_CROP_NOT_LATERAL_EXIT:'近处被画面裁切，不能视为侧向离开',EXIT_SEQUENCE_TOO_SHORT:'真实图像序列不足，暂不能判断离画面方向',CENTER_MISS_OR_EDGE_WITHOUT_OUTWARD_SEQUENCE:'没有连续向外运动的真实序列，仅保留目标记忆',EXIT_DIRECTION_SEQUENCE_INCONSISTENT:'连续观测方向不一致，不凭单帧推断离开方向',EXIT_CROSS_EYE_CANONICAL_RAY_UNAVAILABLE:'切眼后缺少规范方向射线，暂不使用跨镜头方向',EXIT_HINT_SOURCE_EXPIRED:'离画面线索已过期，不能当作当前观测',STATIC_MODE_EXIT_CONTROL_DISABLED:'静止模式不采用移动跟随的离画面控制',NONE:'尚无跟随关联异常'};
 const exitStatuses={READY_DIRECTED:'真实方向线索已就绪，尚待动作锁定',ACTIVE_DIRECTED:'控制器采用定向找回线索（运动仍以实际轮速为准）',RECOVERED_VISUAL:'新视觉已恢复，旧找回线索结束',RADAR_PRIORITY:'同目标新雷达优先于离画面线索',EXPIRED:'原始线索已过期，不能新启动找回；已开始的动作另看定向进度',NOT_LATERAL:'中心漏检或近处裁切，不能决定左右方向',UNCOMPENSATED:'缺少真实车体姿态补偿，只显示原始图像线索',NONE:'尚无可用的离画面线索'};
 const integer=x=>Number.isInteger(x)&&x>=0;
 const count=x=>integer(x)?String(x):'未知';
 const percent=x=>finite(x)&&x>=0&&x<=1?(x*100).toFixed(0)+'%':'未知';
 window.objectRadarAssociationDiagnostics=(input,elapsed=0)=>{
  const d=obj(input)||{},r=obj(d.radar_association);if(!r)return '';
  const reasons={ELIGIBLE:'目标候选可参与关联',WALL_EXCLUDED:'墙面候选已排除',NEED_DISTINCT_SCAN_SUPPORT:'等待独立新扫描支持',CONTINUOUS_SELECTED:'与原目标连续，已选中',CURRENT_UNSELECTED:'当前候选未选中'};
  let out='雷达接收层：原始 '+count(r.raw_count)+' / 有效 '+count(r.valid_count)+' · 超距剔除 '+count(r.range_rejected)+'\n目标方向层：区内 '+count(r.sector_count)+' / 区外 '+count(r.sector_rejected)+' · 聚类 '+count(r.clusters);
  out+='\n目标关联剔除：墙面 '+count(r.wall_rejected)+' · 独立扫描支持 '+count(r.temporal_rejected)+' · 轨迹门限 '+count(r.gate_rejected);
  const ageExtra=extraAge(d,elapsed),ageText=x=>positive(x)&&ageExtra!==null?fmt(x+ageExtra,0)+'ms':'未知';
  if(own(r,'provenance')||own(r,'visual_age_ms'))out+='\n来源关系：'+(r.provenance_zh||r.provenance||'未上报')+' · 图像原始年龄 '+ageText(r.visual_age_ms)+' · 图像源 '+(typeof r.visual_source_us==='string'?r.visual_source_us:'未知')+'us；新雷达回波与旧图像时间分开显示。';
  out+='\n本轮首个判断：'+(typeof r.first_reason_zh==='string'?r.first_reason_zh:reasons[r.first_reason]||r.first_reason||'未上报')+' · '+(r.first_reason||'未上报');
  out+='\n选中目标距离：'+(positive(r.selected_range_m)?fmt(r.selected_range_m)+'m':'尚无本轮实测')+' · 源 '+(typeof r.selected_source_us==='string'?r.selected_source_us:'未知')+'us · 原始年龄 '+ageText(r.selected_age_ms);
  const candidates=Array.isArray(r.candidates)?r.candidates.filter(obj).slice(0,16):[];
  for(const c of candidates)out+='\n候选距离 '+fmt(c.range_m)+'m · 方位 '+(finite(c.angle_rad)?(c.angle_rad*180/Math.PI).toFixed(1)+'°':'未知')+' · '+count(c.points)+'点 · '+(reasons[c.reason]||c.reason||'未上报')+' · 年龄 '+ageText(c.age_ms);
  return out+'\n候选回波不等于已确认原目标；原始有效障碍仍独立参与避障。';
 };
 window.objectFollowReasonText=code=>typeof code==='string'?followReasons[code]||code:'未上报';
 window.objectFollowSceneDiagnostics=input=>{
  const d=obj(input)||{},has=['follow_wall_score','follow_motion_score','follow_isolation_score','follow_candidates','association_revision','follow_reason','follow_sparse','follow_support_points','follow_search_half_rad'].some(k=>own(d,k));if(!has)return '';
  let out='跟随关联：'+window.objectFollowReasonText(d.follow_reason)+'\n墙面证据 '+percent(d.follow_wall_score)+' · 目标运动证据 '+percent(d.follow_motion_score)+' · 前景独立性 '+percent(d.follow_isolation_score);
  out+='\n候选 '+count(d.follow_candidates)+' · 排除墙面 '+count(d.follow_wall_rejected)+' · 运动支持 '+count(d.follow_motion_support)+' · 关联修订 '+count(d.association_revision);
  out+='\n持续移动优先限于原目标的合理方向和轨迹；静止人物仍可保留，不因没有运动就当成墙。';
  if(own(d,'follow_sparse')||own(d,'follow_support_points'))out+='\n远距稀疏支持：'+(d.follow_sparse===true?'弱测量，不能确认到达':d.follow_sparse===false?'非稀疏标记':'未上报')+' · 当前实际支持 '+count(d.follow_support_points)+'点';
  if(finite(d.follow_search_half_rad))out+='\n雷达目标关联搜索半角 '+(d.follow_search_half_rad*180/Math.PI).toFixed(1)+'°（不是车身已转角度）';
  return out;
 };

 window.objectObstacleSensitivityDiagnostics=(input,elapsed=0)=>{
  const d=obj(input)||{},o=obj(d.obstacle_sensitivity);if(!o)return '';
  const stampNow=stamp(d.published_us),source=stamp(o.source_us),extra=extraAge(d,elapsed),age=stampNow!==null&&source!==null&&source>0&&stampNow>=source&&extra!==null?(stampNow-source)/1000+extra:null;
  let out='全向障碍感知（独立于目标关联）：前 '+fmt(o.front_range_m,1)+'m / 左右 '+fmt(o.side_range_m,1)+'m / 后 '+fmt(o.rear_range_m,1)+'m';
  out+='\n实收点数 前 '+count(o.front_points)+' / 左 '+count(o.left_points)+' / 右 '+count(o.right_points)+' / 后 '+count(o.rear_points)+' · 原扫描 '+count(o.raw_scan_points)+'点';
  out+='\n扫描原始源 '+(typeof o.source_us==='string'?o.source_us:'未知')+'us · 年龄 '+(age!==null?age.toFixed(0)+'ms':'未知')+'；墙面排除只用于目标关联，障碍回波不会因此删除。';
  return out;
 };
 window.objectDirectedTurnDiagnostics=(input,elapsed=0)=>{
  const d=obj(input)||{},turn=obj(d.directed_turn),hasSparse=own(d,'target_measurement_sparse')||own(d,'target_measurement_support_points');let out='';
  if(hasSparse)out='目标测量支持：'+count(d.target_measurement_support_points)+'点 · '+(d.target_measurement_sparse===true?'稀疏弱测量，不能据此确认到达':d.target_measurement_sparse===false?'非稀疏标记':'质量标记未知');
  if(typeof d.direction_source==='string')out+=(out?'\n':'')+'当前方位来源：'+text(d.direction_source,{EXIT_LATCH:'已锁定的原始离画面线索',RADAR_TRAIL:'雷达实测轨迹接续方位（按原始年龄区分预测）',MEMORY_TRAIL:'历史轨迹记忆方位',STATIC_MEMORY:'静止目标记忆方位',OLD_DIRECTION:'保留的旧方向',SEARCH:'当前找回策略',CANDIDATE:'疑似重现候选方向，尚未确认',RADAR:'新的同目标雷达方位',LIDAR:'新的同目标雷达方位',RADAR_VISUAL:'雷达＋视觉实测方位',VISUAL:'真实图像方位',VISUAL_CURRENT:'真实图像方位',MEASURED_TRAIL:'实测目标轨迹',MEMORY:'历史目标位置',NONE:'未提供'})+'（以原始源时间区分新旧）';
  if(own(d,'direction_source_us')||own(d,'direction_raw_rad')||own(d,'direction_filtered_rad')){const dp=stamp(d.published_us),ds=stamp(d.direction_source_us),extra=extraAge(d,elapsed),age=dp!==null&&ds!==null&&ds>0&&dp>=ds&&extra!==null?(dp-ds)/1000+extra:null;out+='\n方位原始源 '+(typeof d.direction_source_us==='string'?d.direction_source_us:'未知')+'us · 年龄 '+(age!==null?age.toFixed(0)+'ms':'未知')+' · 原始 '+(finite(d.direction_raw_rad)?(d.direction_raw_rad*180/Math.PI).toFixed(1)+'°':'未知')+' / 平滑后 '+(finite(d.direction_filtered_rad)?(d.direction_filtered_rad*180/Math.PI).toFixed(1)+'°':'未知');}
  if(!turn)return out;
  const statuses={WAIT_NEW_EVIDENCE:'已结束本次定向转动，等待真正的新目标证据',PAUSED:'定向动作暂停',REPLACED_BY_NEW_HINT:'原动作已被真正的新来源线索替换',READY:'定向动作等待执行',ACTIVE:'定向动作已锁定，按真实里程计进度执行',REACQUIRED_VISUAL:'新视觉已重现，结束旧定向动作',RADAR_PRIORITY:'同目标新雷达更新方位，优先接续目标',COMPLETED:'已达到实测目标转角',TIMEOUT:'定向动作执行超时，结束旧转向',CANCELLED:'定向动作已被取消',NONE:'当前没有已锁定定向动作'};
  const angle=x=>finite(x)?(x*180/Math.PI).toFixed(1)+'°':'未知';
  const pct=finite(turn.target_angle_rad)&&turn.target_angle_rad>0&&finite(turn.progress_rad)&&turn.progress_rad>=0?Math.min(100,turn.progress_rad/turn.target_angle_rad*100).toFixed(0)+'%':'未知';
  out+=(out?'\n':'')+'出框定向：'+(typeof turn.status==='string'?statuses[turn.status]||turn.status:'未上报阶段')+' · '+(turn.active===true?'动作保留中':turn.active===false?'非活动':'活动状态未知');
  out+='\n目标转角 '+angle(turn.target_angle_rad)+' · 实测进度 '+angle(turn.progress_rad)+' / '+pct+' · 剩余 '+angle(turn.remaining_rad);
  out+='\n开始航向 '+angle(turn.start_yaw_rad)+' · 目标航向 '+angle(turn.goal_yaw_rad)+' · 转向 '+(turn.sign===1?'左':turn.sign===-1?'右':'未知');
  out+='\n执行用时 '+(positive(turn.elapsed_ms)?turn.elapsed_ms.toFixed(0)+'ms':'未知')+' / 上限 '+(positive(turn.timeout_ms)?turn.timeout_ms.toFixed(0)+'ms':'未知')+' · 动作起始 '+(typeof turn.started_us==='string'?turn.started_us:'未知')+'us';
  out+='\n原线索帧 '+(typeof turn.frame_id==='string'?turn.frame_id:'未知')+' · 原源 '+(typeof turn.source_us==='string'?turn.source_us:'未知')+'us；2秒图像提示过期不会刷新观测，已启动动作由实测转角/执行超时/新目标证据决定结束。';
  return out;
 };
 window.objectFollowExitDiagnostics=(input,elapsed=0)=>{
  const d=obj(input)||{},selected=Array.isArray(d.objects)?d.objects.find(x=>x&&x.track_id===d.selected_track_id):null,ex=obj(d.follow_exit)||obj(selected&&selected.follow_exit);if(!ex)return '';
  const taskShape=own(d,'follow_exit'),source=stamp(ex.source_us),base=stamp(taskShape?d.published_us:d.frame_time_us),extra=extraAge(d,elapsed);
  const sourceAge=source!==null&&source>0&&taskShape&&positive(ex.age_ms)&&extra!==null?ex.age_ms+extra:source!==null&&source>0&&base!==null&&base>=source&&extra!==null?(base-source)/1000+extra:null;
  const fresh=ex.valid===true&&sourceAge!==null&&sourceAge<=2000;
  let text='离画面线索（历史，不是当前观测）：'+(ex.valid===true?(fresh?'原始来源在2秒内':sourceAge===null?'来源年龄未知':'来源已过期'):'当前无有效线索');
  text+='\n'+(typeof ex.status==='string'?exitStatuses[ex.status]||ex.status:ex.compensated===true?'方向已按真实姿态补偿':'原始图像方向，尚未证明已做车体姿态补偿');
  const dir=ex.direction===1?'画面左侧':ex.direction===-1?'画面右侧':'未知方向',kind=ex.kind===1?'左边缘离开':ex.kind===2?'右边缘离开':ex.kind===3?'中间漏检，不能定向':ex.kind===4?'近处裁切，不是侧向离开':'尚无离画面类型';
  text+='\n来源类型 '+kind+' · 原始方向 '+dir+' · 补偿 '+(ex.compensated===true?'已完成':ex.compensated===false?'未完成':'未知');
  text+='\n原始帧 '+(typeof ex.frame_id==='string'?ex.frame_id:'未知')+' · 镜头 '+(ex.eye===0?'左眼eye0':ex.eye===1?'右眼eye1':'未上报')+' · 原始时间 '+(typeof ex.source_us==='string'?ex.source_us:'未知')+'us · 年龄 '+(sourceAge!==null?sourceAge.toFixed(0)+'ms':'未知');
  text+='\n原因：'+window.objectFollowReasonText(ex.reason)+' · '+(typeof ex.reason==='string'?ex.reason:'未上报');
  const frames=Array.isArray(ex.keyframes)?ex.keyframes.filter(k=>obj(k)&&typeof k.frame_id==='string'&&/^[1-9][0-9]{0,19}$/.test(k.frame_id)&&typeof k.source_us==='string'&&/^[1-9][0-9]{0,19}$/.test(k.source_us)&&[0,1].includes(k.eye)&&Array.isArray(k.bbox)&&k.bbox.length===4&&k.bbox.every(finite)).slice(-8):[];text+='\n真实关键帧 '+frames.length+'/8（只读元数据，不能当作当前连续观测）';
  for(const k of frames){if(!obj(k))continue;text+='\n  帧 '+(typeof k.frame_id==='string'?k.frame_id:'未知')+' · '+(k.eye===0?'左眼':k.eye===1?'右眼':'镜头未知')+' · 源 '+(typeof k.source_us==='string'?k.source_us:'未知')+'us'+(finite(k.bearing_rad)?' · 原始方位 '+(k.bearing_rad*180/Math.PI).toFixed(1)+'°':' · 无可靠方位');}
  return text;
 };
 window.objectFlowDiagnostics=(input,elapsed=0)=>{
  const t=obj(input)||{},o=obj(t.target_observation),estimate=obj(t.target_estimate),e=obj(t.execution)||{};let distance='';
  if(o){const age=observationAge(t,o,elapsed),recent=o.valid===true&&positive(age)&&age<=1200;
   distance=(recent?'最近实测表面净距：':o.valid===true?'历史实测表面净距（不作实时）：':'本轮无有效目标表面实测：')+(o.valid===true?fmt(o.surface_gap_m)+'m':'—')+'\n来源：'+text(o.source,sources)+' · 原始年龄 '+(positive(age)?age.toFixed(0)+'ms':'未知')+' · 误差 '+fmt(o.error_m)+'m';
   distance+='\n净距口径：车头到目标表面；不等于车体中心直线距离\n安装参数：'+(o.mount_verified===true?'已核对':o.mount_verified===false?'近似，尚未验证':'核对状态未知')+' · 观测原因：'+(o.reason||'未上报');
  }else distance='旧接口参考距离：'+fmt(t.range_m)+'m（未区分实测与估计/记忆）\n来源：'+text(t.source,sources)+'；v2原始观测及安装诊断尚未上报';
  if(estimate){const add=extraAge(t,elapsed),age=positive(estimate.age_ms)&&add!==null?estimate.age_ms+add:null;distance+='\n目标估计 / 记忆净距：'+fmt(estimate.gap_m)+'m · '+text(estimate.kind,kinds)+' · 年龄 '+(positive(age)?age.toFixed(0)+'ms':'未知')+'\n估计位置 X '+fmt(estimate.x_m)+' / Y '+fmt(estimate.y_m)+'m（不能当作新实测）';}
  else distance+='\n目标估计 / 记忆：此版本未上报';
  if(positive(t.standoff_m))distance+='\n设定车头到表面间距：'+fmt(t.standoff_m)+'m';
  const dyn=obj(t.dynamic_navigation);if(dyn){distance+='\n动态导航目标：'+(dyn.direction_only===true?'仅方向':dyn.target_valid===true?'有效导航目标':'尚无有效目标')+(dyn.provisional===true?' · 临时导航探针，不是实测距离':'')+' · 更新代次 '+count(dyn.target_revision);distance+='\n导航目标坐标 X '+fmt(dyn.target_x_m)+' / Y '+fmt(dyn.target_y_m)+'m · 源 '+(typeof dyn.target_source_us==='string'?dyn.target_source_us:'未知')+'us · '+(dyn.reason||'未上报');}
  const requestV=fallback(e,'requested_v',t,'requested_v'),requestW=fallback(e,'requested_w',t,'requested_w'),actualV=fallback(e,'actual_v',t,'protected_v'),actualW=fallback(e,'actual_w',t,'protected_w');
  const wheels=Array.isArray(t.actual_wheel_rpm)&&t.actual_wheel_rpm.length===4?t.actual_wheel_rpm.map(x=>fmt(x,1)).join(',')+' RPM':'未知';
  let motion='任务请求 v '+fmt(requestV)+'m/s / w '+fmt(requestW)+'rad/s\n最终下发指令 v '+fmt(actualV)+'m/s / w '+fmt(actualW)+'rad/s（不是实测车速）\n实测轮速 '+wheels+' · 实际角速度 '+fmt(t.measured_w_rad_s)+'rad/s';
  const cd=obj(t.control_diagnostics)||{};motion+='\n导航输出 v '+fmt(cd.navigation_v)+'m/s / w '+fmt(cd.navigation_w)+'rad/s\n首个否决：'+(typeof cd.first_veto==='string'?cd.first_veto:'尚未上报')+' · 零输出来源：'+(typeof cd.zero_source==='string'?cd.zero_source:'尚未上报');
  motion+='\n保护后减去请求：Δv '+(finite(actualV)&&finite(requestV)?(actualV-requestV).toFixed(3):'未知')+'m/s / Δw '+(finite(actualW)&&finite(requestW)?(actualW-requestW).toFixed(3):'未知')+'rad/s（仅比较指令，不是测速）';
  motion+='\n执行路线：'+text(e.route_kind,{PREVIEW:'只读路径预览',ACTUAL:'实际执行路径',EXECUTION:'实际执行路径',LOCAL:'实际局部路径',NONE:'暂无路径'})+' · 坐标 '+(e.route_frame||'未上报');
  if(t.active===true&&Array.isArray(t.actual_wheel_rpm)&&t.actual_wheel_rpm.length===4&&t.actual_wheel_rpm.every(x=>finite(x)&&Math.abs(x)<.05))motion+='\n任务已激活，但四轮实测为0；不能显示为正在行驶';
  return {distance,motion,blocker:block(t,e,elapsed),observation_age_ms:o?observationAge(t,o,elapsed):null};
 };
})();
