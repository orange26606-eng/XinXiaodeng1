"use strict";
/* 任务状态由RK管理，HTTP200只表示入队，不表示电机运行或已经到达。 */
(() => {
 const $=id=>document.getElementById(id);let mode='static',selection=null,task=null,received=0,pending=null,nextMode=null,lastError='',latchedRejection=false,launch=null,lastSelectionAck=null,launchSerial=0;
 const states={IDLE:'待双击目标',PREPARED:'目标已确认，双击目标即可前往/跟随',READY:'目标已确认，双击目标即可前往/跟随',APPROACHING:'前往静止物体任务',FOLLOWING:'原目标跟随任务',WAIT_DISTANCE:'保持距离，目标远离后自动跟随',WAIT_EXIT_EVIDENCE:'本次定向已结束，等待新目标证据',FOLLOW_EXIT_HINT:'按原始关键帧线索定向找回',OBSERVING_CANDIDATE:'正在对准候选，观察大小',VERIFY_ARRIVAL:'目标已近，保持停车复核',ARRIVED:'静止目标已到达，保持停车',SEARCHING:'正在寻找原目标',SEEKING:'正在寻找原目标',PREDICTING:'短暂遮挡，按原目标轨迹接续',WAIT_TARGET:'等待原目标的新观测',WAIT_SENSORS:'等待传感器恢复',CANCELLED:'任务已结束，双击目标可重新前往/跟随',REJECTED:'请求未接受',PREVIEW:'仅预览，没有导航输出',PAUSED:'跟随已暂停，保留原目标，等待明确继续'};
 const text=s=>s==='ARRIVED'&&task&&task.near_metric===false?'视觉近距停靠完成，保持停车（距离粗估）':s==='ARRIVED'&&task&&task.near_metric===true?'目标表面距离确认到达，保持停车':s==='SEARCHING'||s==='SEEKING'?(mode==='static'?'静止目标正在分阶段找回':'正在找回原跟随目标'):states[s]||(task&&task.message_zh)||'等待板端任务说明';
 const numeric=v=>typeof v==='number'&&Number.isFinite(v);
 const fmt=(v,d=2)=>numeric(v)?v.toFixed(d):'未知';
 const identityReasons={PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE:'当前仅预览，没有电机授权；请用 sh begin drive 重新启动',OBJECT_START_OWNER_OR_SEQUENCE_INVALID:'手机控制权或请求顺序不匹配，请重新连接并明确开始',OBJECT_PREPARE_OWNER_OR_SEQUENCE_INVALID:'当前手机不是有效任务所有者，或准备请求已重复',SELECTED_OBJECT_NOT_YET_EXPORTED_WAIT_NEW_FRAME:'新选目标尚未被控制器导出，请等待新一帧后再准备/开始',OBJECT_START_NOT_CURRENT_PREPARED_TASK:'当前目标或模式与已准备任务不一致，请重新准备',OBJECT_REQUEST_REVOKED_OR_FENCE_UNAVAILABLE:'先前停车/手动已撤销该请求，或控制栅栏暂不可用',FOLLOW_RESUME_REQUIRES_PAUSED_TASK:'只有已暂停的跟随任务才能继续',REACQUIRE_NO_SPATIALLY_PLAUSIBLE_DETECTION_KEEP_MEMORY:'没有与原目标空间位置相符的检测，保留原目标记忆',REACQUIRE_MULTIPLE_PLAUSIBLE_OBJECTS_KEEP_MEMORY:'多个相似候选无法区分，暂不关联新编号',REACQUIRE_FIRST_DETECTION_DIRECTION_ONLY:'首次疑似重现，只调整对准方向，尚未确认认回',REACQUIRE_CONFIRMED_NEW_SOURCE_CONTINUITY:'连续新图像支持同一目标重现',COMPETING_CURRENT_OBJECTS_KEEP_TASK:'多个当前目标竞争关联，保持原任务身份',IDENTITY_AMBIGUOUS:'相似物体存在歧义，暂不替换原目标',NONE:'无候选关联异常',UNAVAILABLE:'尚无关联诊断'};
 function identityReason(s){return identityReasons[s]||(window.objectReasonText&&window.objectReasonText(s))||s||'尚未上报关联判断';}
 function evidence(){
  if(!task)return '目标身份与候选关联：等待板端诊断';
  const reason=task.reason||'',candidate=task.candidate_reason||'';
  let phase=reason.includes('MEMORY')?'记忆位置寻找：沿上次实测位置接近，不表示重新看见目标':reason.includes('REACQUIRE_CANDIDATE')||candidate==='REACQUIRE_FIRST_DETECTION_DIRECTION_ONLY'?'疑似重现对准：候选尚未正式关联原目标':task.state==='CANCELLED'?'任务已取消：不会自动重启；重新双击目标后才能继续':task.state==='SEARCHING'?'原目标暂失，按板端阶段找回':task.active?'追踪原目标：保持原身份，不自动切换同类物体':'等待明确任务请求';
  phase+='\n任务原目标 #'+(Number.isInteger(task.track_id)&&task.track_id>0?task.track_id:'未知')+' · 候选编号 '+(Number.isInteger(task.candidate_track_id)&&task.candidate_track_id>0?'#'+task.candidate_track_id:'尚未关联');
  phase+='\n关联判断：'+identityReason(candidate)+' · '+(candidate||'字段尚未上报');
  if(task.enabled===false)phase+='\n底盘未使能'+(task.pending_enable===true?'，正在等待使能确认':'，当前任务请求不等于已向车轮执行');
  else if(task.enabled===true)phase+='\n底盘已使能（运动仍以实测反馈为准）';else phase+='\n底盘使能状态：旧服务未上报';
  phase+='\n双目原始年龄：'+(numeric(task.depth_age_ms)?task.depth_age_ms.toFixed(0)+'ms':'未知');
  if(numeric(task.depth_age_ms)&&task.depth_age_ms>1200)phase+=' · 双目已过期，当前参考来源：'+(task.source||'等待雷达/记忆状态');
  if(Number.isInteger(task.association_revision)&&task.association_revision>=0)phase+='\n雷达关联修订 '+task.association_revision+(Number.isInteger(task.association_corrections)?' · 累计修正 '+task.association_corrections:'')+(typeof task.association_reason==='string'?' · '+((window.objectFollowReasonText&&window.objectFollowReasonText(task.association_reason))||task.association_reason):'');
  if(window.objectFollowExitDiagnostics){const exit=window.objectFollowExitDiagnostics(task,Math.max(0,Date.now()-received));if(exit)phase+='\n'+exit;}
  if(window.objectDirectedTurnDiagnostics){const turn=window.objectDirectedTurnDiagnostics(task,Math.max(0,Date.now()-received));if(turn)phase+='\n'+turn;}
  return phase;
 }
 const nearReasons={TARGET_SURFACE_NEAR_HOLD:'目标表面距离已进入停靠范围',VISUAL_METRIC_SURFACE_NEAR:'视觉测距支持目标表面已近',VISUAL_COARSE_NEAR_HOLD:'视觉粗距提示目标已近',STATIC_NEAR_VISUAL_REOBSERVED_COARSE:'近处原目标重新出现，继续粗略停靠复核',TARGET_EXPANSION_VISUAL_NEAR_HOLD:'原目标持续增大并接近画面边缘',STATIC_VISUAL_SURFACE_ARRIVAL:'新表面距离确认停靠',STATIC_VISUAL_COARSE_DOCKED:'视觉近距复核完成，未验证精确间距'};
 function nearDetail(t){
  if(!t||!(t.near_hold===true||t.state==='VERIFY_ARRIVAL'||t.state==='ARRIVED')||typeof t.near_metric!=='boolean')return '';
  let out='近距保持：'+(t.near_hold===true?'正在保持停车':'已完成停靠，保持停车');
  out+='\n到达依据：'+(t.near_metric===true?'目标表面距离支持停靠；实际误差以测距及安装精度为准':'视觉近距线索，仅作粗略停靠；没有验证精确20厘米');
  const reason=t.near_reason||t.reason;out+='\n近距原因：'+(nearReasons[reason]||t.message_zh||'等待板端近距诊断')+(reason?' · '+reason:'');
  return out;
 }
 function distanceValid(v){return typeof v==='number'&&Number.isFinite(v)&&v>=.1&&v<=3;}
 function distanceRead(){const t=$('objectStandoff').value.trim();return /^(?:0|[1-9]\d*)(?:\.\d+)?$/.test(t)?Number(t):NaN;}
 function distanceKey(){return mode==='follow'?'acg138.object.distance.follow.v2':'acg138.object.distance.static';}
 function saveDistance(){const n=distanceRead();if(distanceValid(n))try{window.localStorage.setItem(distanceKey(),String(n));}catch(e){}}
 function switchMode(want,remember=true){if(remember)saveDistance();mode=want;let n=.2;try{let raw=window.localStorage.getItem(distanceKey());if(raw===null&&mode==='follow'){const legacy=window.localStorage.getItem('acg138.object.distance.follow');if(legacy!==null&&Number(legacy)!==1)raw=legacy;}if(raw!==null&&/^(?:0|[1-9]\d*)(?:\.\d+)?$/.test(raw)&&distanceValid(Number(raw)))n=Number(raw);if(mode==='follow')window.localStorage.setItem(distanceKey(),String(n));}catch(e){}$('objectStandoff').value=n.toFixed(2);}

 function live(){return task&&Date.now()-received+Math.max(0,Number(task.age_ms)||0)+Math.max(0,Number(task.client_delay_ms)||0)<8000;}
 function canRestart(){return live()&&task.can_restart===true&&task.prepared===true&&task.drive_capable===true&&task.enabled===false&&task.pending_enable===false&&task.state!=='PAUSED';}
 function chosen(){return selection&&selection.track_id>0?selection:window.currentSelectedObject&&window.currentSelectedObject();}
 function taskIdentity(){return task&&task.epoch&&task.track_id>0?{epoch:task.epoch,track_id:task.track_id}:chosen();}
 function update(){
  if(pending&&Date.now()-pending.started>8000){const was=pending;lastError='本次'+(was.action==='start'||was.action==='resume'?'执行结果仍未确认':'任务回执')+'等待超过8秒，已撤销待发意图；不会重复发送START，请查看实际轮速与板端原因';latchedRejection=true;cancelLaunch('task_timeout');if(window.Rover&&window.Rover.objectAbortTaskIntent)window.Rover.objectAbortTaskIntent();pending=null;nextMode=null;}
  const active=live()&&task.active===true;
  $('objectModeStatic').setAttribute('aria-pressed',String(mode==='static'));$('objectModeFollow').setAttribute('aria-pressed',String(mode==='follow'));
  const title=mode==='static'?'静止物体：稳定位置、前往、停靠后保持停止':'跟随物体：持续更新位置，目标停走时自动接续';
  $('objectTaskStatus').title=title;const briefPhase=launch?({WAIT_SELECTION:'等待所选目标确认',PREPARING:'正在登记当前目标任务',STARTING:'正在提交执行请求',QUERYING_START:'查询执行结果，不重复起步'}[launch.phase]||'正在处理双击请求'):task?text(task.state)+(live()?'':'（历史）'):'等待板端任务状态';$('objectTaskStatus').textContent=(mode==='static'?'静止前往':'移动跟随')+' · '+briefPhase+(task&&task.active===true&&task.enabled===false?'\n底盘未使能，正在等待执行条件':task&&task.drive_capable===false?'\n当前仅预览，没有电机授权':'');
  const locked=chosen();if(locked&&(!task||task.active!==true)&&!(task&&task.prepared===true&&String(task.epoch)===String(locked.epoch)&&task.track_id===locked.track_id&&task.mode===mode))$('objectTaskStatus').textContent+='\n已锁定 #'+locked.track_id+'；双击执行';
  if(task)$('objectTaskReason').textContent=(launch?({WAIT_SELECTION:'双击指令已记录，正在等待这次选择的真实确认',PREPARING:'正在确认目标与任务控制权，不需要再次点击',STARTING:'正在提交执行请求，不需要再次点击',QUERYING_START:'执行请求可能已到板端，正在按原编号查询；不会重复发送'}[launch.phase]||'正在处理双击请求'):['READY','PREPARED'].includes(task.state)?'目标已确认；双击同一目标即可直接前往/跟随':task.message_zh||task.reason||'等待具体原因')+'\n任务：'+(task.reason||'未上报')+' · 执行：'+(task.runtime_reason||'等待执行状态')+' · 底盘：'+(task.navigation_reason||'等待底盘状态')+(task.drive_capable===false?'\n当前仅预览，没有电机授权；使用 sh begin drive 启动真实接管':'')+(pending?'\n阶段 '+pending.action+' · 已等待 '+Math.max(0,Date.now()-pending.started)+' / 8000ms · 请求 '+(pending.request||'尚未取得本次编号')+(pending.uncertain?' · 响应不确定，仅查询，不重发':pending.queued?' · HTTP已排队，等待板端真实回执':' · 尚未确认排队'):'');
  if(task&&(task.restart_required===true||task.resume_requires_start===true||(task.prepared===true||task.active===true)&&task.enabled===false))$('objectTaskReason').textContent+='\n恢复执行：'+(task.restart_reason||'等待板端发布明确恢复条件')+(canRestart()?'；可以明确点击重新接管，不会自动起步':'；当前不满足重新接管条件');
  if(lastError)$('objectTaskReason').textContent+='\n请求/读取失败：'+lastError;
  const range=task&&task.range_m;$('objectTaskDistance').textContent=Number.isFinite(range)?'任务参考距离 '+range.toFixed(2)+'m · 来源 '+(task.source||'未上报'):'距离未知：可见目标方向继续维护，距离不是凭框大小伪造';if(task&&distanceValid(task.standoff_m))$('objectTaskDistance').textContent+='\n板端当前设定间距 '+task.standoff_m.toFixed(2)+'m';
  $('objectTaskEvidence').textContent=evidence();
  if(task)$('objectTaskMotion').textContent='请求线速度 '+fmt(task.requested_v)+'m/s · 保护后 '+fmt(task.protected_v)+'m/s\n请求角速度 '+fmt(task.requested_w)+'rad/s · 保护后 '+fmt(task.protected_w)+'rad/s · 实际角速度 '+fmt(task.measured_w_rad_s)+'rad/s\n实测轮速 '+(Array.isArray(task.actual_wheel_rpm)?task.actual_wheel_rpm.map(x=>fmt(x,1)).join(',')+'RPM':'未知')+' · '+(task.feedback_fresh===true?'反馈新鲜':task.feedback_fresh===false?'反馈待恢复':'反馈状态未上报');
  if(task&&window.objectFlowDiagnostics){const flow=window.objectFlowDiagnostics(task,Math.max(0,Date.now()-received));$('objectTaskDistance').textContent=flow.distance;$('objectTaskMotion').textContent=flow.motion;$('objectTaskBlocker').textContent=flow.blocker;}
  const near=nearDetail(task);if(near){$('objectTaskReason').textContent+='\n'+near;$('objectTaskEvidence').textContent+='\n'+near;
   if(task.near_metric===false)$('objectTaskDistance').textContent='视觉近距'+(task.state==='ARRIVED'?'停靠完成':'停止复核')+'；当前间距未可靠测得。'+(distanceValid(task.standoff_m)?'\n设定目标间距 '+task.standoff_m.toFixed(2)+'m（目标值，不是实测到达精度）':'');
  }
  const retry=canRestart(),c=retry?taskIdentity():chosen(),same=task&&c&&String(task.epoch)===String(c.epoch)&&task.track_id===c.track_id&&task.mode===mode;
  $('prepareObjectTask').disabled=!!pending||!c;
  $('startObjectTask').disabled=!!pending||!live()||!same||task.prepared!==true||task.active===true&&!retry||task.arrived===true;
  $('prepareObjectTask').textContent=mode==='static'?'准备前往 / 预览':'准备跟随';$('startObjectTask').textContent=retry?'重新接管 / 恢复执行':mode==='static'?'开始前往':'开始跟随';
  $('startObjectTask').disabled=$('startObjectTask').disabled||task&&task.state==='PAUSED';
  $('pauseObjectTask').hidden=mode!=='follow';$('resumeObjectTask').hidden=mode!=='follow';
  $('pauseObjectTask').disabled=!!pending||!live()||task.mode!=='follow'||task.active!==true||task.state==='PAUSED';
  $('resumeObjectTask').disabled=!!pending||!live()||task.mode!=='follow'||task.state!=='PAUSED'||task.prepared!==true;
  $('cancelObjectTask').textContent=mode==='follow'?'结束跟随':'停车 / 取消任务';
  $('objectDistanceLabel').textContent=mode==='follow'?'跟随间距（可调）':'车头到表面间距';
 }
 function submit(action,autoToken){
  if(pending||!window.Rover||!window.Rover.objectTask)return false;if(action==='start'&&!autoToken&&task&&task.active===true&&!canRestart()){lastError=task.restart_reason||'任务仍在执行或尚未满足明确重新接管条件';update();return false;}if(action==='start'&&!autoToken&&task&&task.state==='PAUSED'){lastError='任务已暂停，请点击继续跟随；不会重新起步';update();return false;}if(action==='pause'&&(!live()||task.mode!=='follow'||task.active!==true)||action==='resume'&&(!live()||task.mode!=='follow'||task.state!=='PAUSED'||task.prepared!==true)){lastError='当前板端状态不允许该暂停/继续操作，请等待新状态';update();return false;}const c=autoToken&&launch&&launch.token===autoToken?{epoch:launch.epoch,track_id:launch.track_id}:['cancel','pause','resume'].includes(action)||action==='start'&&canRestart()?taskIdentity():chosen();if(!c){$('objectTaskReason').textContent='请先选择当前图像中的真实物体';return false;}
  const standoff=autoToken&&launch&&launch.token===autoToken?launch.standoff:['cancel','pause'].includes(action)?task&&distanceValid(task.standoff_m)?task.standoff_m:.2:distanceRead();if(!distanceValid(standoff)){$('objectTaskReason').textContent='间距需为0.10～3.00米';return false;}
  saveDistance();const requestMode=['cancel','pause','resume'].includes(action)&&task?task.mode:mode;if(['pause','resume'].includes(action)&&requestMode!=='follow'){lastError='暂停/继续只用于跟随模式';update();return false;}pending={action,epoch:String(c.epoch),track:c.track_id,mode:requestMode,request:null,started:Date.now(),autoToken:autoToken||null};
  lastError='';latchedRejection=false;$('objectTaskReason').textContent=action==='pause'?'正在请求暂停跟随，等待板端PAUSED回执':action==='resume'?'正在请求继续跟随，等待板端实际接受':action==='start'?'正在请求执行双击目标，等待板端接受':action==='cancel'?'正在停车并取消任务，等待板端回执':'正在确认双击目标与控制权，请等待实际回执';
  window.Rover.objectTask(action,String(c.epoch),c.track_id,requestMode,standoff);update();return true;
 }
 function setMode(want){if(want===mode)return;cancelLaunch('mode_change');if(pending)return;if(live()&&(task.active||task.prepared)&&!['CANCELLED','IDLE','REJECTED'].includes(task.state)){nextMode=want;if(!submit('cancel'))nextMode=null;return;}switchMode(want);update();}
 function cancelLaunch(why){if(why==='close')return;const old=launch,obsolete=pending&&pending.action!=='cancel'?pending:null;launch=null;lastSelectionAck=null;if(obsolete||old&&pending&&pending.autoToken===old.token)pending=null;if((old||obsolete)&&window.Rover&&window.Rover.objectAbortTaskIntent)window.Rover.objectAbortTaskIntent();if(why==='cancel'&&(old&&['STARTING','QUERYING_START'].includes(old.phase)||obsolete&&['start','resume'].includes(obsolete.action))&&window.Rover&&window.Rover.command)window.Rover.command('stop','');}
 window.objectLaunchCancel=cancelLaunch;window.objectLaunchPending=()=>!!launch;
 window.objectSelectionIntent=(epoch,id,gesture)=>{if(window.semanticOwns&&window.semanticOwns()&&window.Rover&&window.Rover.semanticCancel)window.Rover.semanticCancel();cancelLaunch('new_selection');lastSelectionAck=null;};
 window.objectSelectionConfirmed=ack=>{lastSelectionAck=ack;advanceLaunch();};
 function sameTask(c){return live()&&task.epoch===c.epoch&&task.track_id===c.track_id&&task.mode===mode;}
 window.objectTargetExecuting=(epoch,id)=>sameTask({epoch,track_id:id})&&task.active===true&&!canRestart();
 window.objectDoubleActivate=c=>{
  if(window.objectTargetExecuting(c.epoch,c.track_id)){$('objectSelectionStatus').textContent='该目标任务正在执行，不重复接管或打断运动';return;}
  const confirmed=lastSelectionAck;cancelLaunch('replacement_double');if(confirmed&&confirmed.gesture===c.gesture&&confirmed.epoch===c.epoch&&confirmed.track_id===c.track_id)lastSelectionAck=confirmed;const standoff=distanceRead();if(!distanceValid(standoff)){lastError='间距需为0.10～3.00米';update();return;}
  launch={...c,token:++launchSerial,phase:'WAIT_SELECTION',standoff};$('objectSelectionStatus').textContent='已双击，直接请求前往/跟随；不等待选中回执或精确测距';
  advanceLaunch();update();
 };
 function failLaunch(message){launch=null;lastError=message;latchedRejection=true;}
 function advanceLaunch(){if(window.semanticOwns&&window.semanticOwns()){launch=null;pending=null;return;}
  if(!launch||launch.phase!=='WAIT_SELECTION')return;
  const token=launch.token;launch.phase='STARTING';
  if(!submit('start',token))failLaunch('双击目标请求未能发送，请查看原因后重新双击');
 }
 function launchAccepted(request,data){if(!launch||request.autoToken!==launch.token)return;if(request.action==='prepare'){launch.phase='STARTING';if(!submit('start',launch.token))failLaunch('目标已确认，但执行请求未发送；请重新双击重试');}else if(request.action==='start'){launch=null;$('objectSelectionStatus').textContent='双击目标任务已被板端接受；实际运动以执行状态和轮速反馈为准';}}
 window.objectTaskSelection=c=>{selection=c;update();};
 window.objectTargetChanged=(epoch,id,detail)=>{selection={epoch,track_id:id,observed:!!(detail&&detail.observed===true),age_ms:detail&&typeof detail.age_ms==='number'&&Number.isFinite(detail.age_ms)?detail.age_ms:Infinity};update();};
 window.objectTaskSending=(action,id)=>{if(!pending||pending.action!==action||typeof id!=='string'||!/^[1-9]\d{0,19}$/.test(id)||pending.request&&pending.request!==id)return;pending.request=id;pending.sent=true;if(task)consumeTaskReceipt(task);update();};
 window.objectTaskQueued=(action,id)=>{if(!pending||pending.action!==action||typeof id!=='string'||!/^[1-9]\d{0,19}$/.test(id)||pending.request&&pending.request!==id)return;pending.request=id;pending.queued=true;pending.uncertain=false;if(task)consumeTaskReceipt(task);update();};
 window.objectTaskUncertain=(action,id,message)=>{if(!pending||pending.action!==action||pending.request!==id||!['start','resume'].includes(action))return;pending.uncertain=true;lastError='请求 '+id+' 的网络响应未完成：'+(message||'连接超时')+'；仅查询同一请求结果，不自动重发';if(launch&&pending.autoToken===launch.token)launch.phase='QUERYING_START';if(window.Rover&&window.Rover.objectReadTask)window.Rover.objectReadTask();if(task)consumeTaskReceipt(task);update();};
 window.objectTaskError=(action,message)=>{if(action!=='read')launch=null;if(action==='read'&&latchedRejection)return;if(!pending||pending.action===action)pending=null;nextMode=null;lastError=message||'任务服务暂不可用；未自动开始';latchedRejection=action!=='read';update();};
 window.objectReceiveTask=data=>{
  if(!data||data.version!==1||!['static','follow'].includes(data.mode)||typeof data.epoch!=='string'||!Number.isInteger(data.track_id)||typeof data.state!=='string')return;
  task=data;received=Date.now();if(!latchedRejection)lastError='';if(!pending&&(task.active===true||task.state==='PAUSED')&&mode!==task.mode){switchMode(task.mode);if(distanceValid(task.standoff_m)){$('objectStandoff').value=task.standoff_m.toFixed(2);saveDistance();}}
  consumeTaskReceipt(data);update();
 };
 function consumeTaskReceipt(data){
  const receipt=data.task_request;if(pending&&pending.request&&receipt&&String(receipt.id)===pending.request&&receipt.state==='REJECTED'){lastError='板端已拒绝本次'+({prepare:'准备',start:'开始/重新接管',pause:'暂停',resume:'继续',cancel:'结束'}[pending.action]||'请求')+'：'+(identityReasons[receipt.reason]||(window.objectReasonText&&window.objectReasonText(receipt.reason))||receipt.reason||'未上报具体拒绝原因')+' · '+(receipt.reason||'未上报原因码');latchedRejection=true;launch=null;pending=null;nextMode=null;}
  if(pending&&pending.request&&String(data.request_id)===pending.request){
   if(pending.action==='cancel'){if(data.active===false&&['CANCELLED','IDLE'].includes(data.state)){pending=null;if(nextMode){const want=nextMode;nextMode=null;switchMode(want);}}else if(data.state==='REJECTED'){pending=null;nextMode=null;}}
   else if(String(data.epoch)===pending.epoch&&data.track_id===pending.track&&data.mode===pending.mode){const accepted=data.state==='REJECTED'||(pending.action==='pause'?data.state==='PAUSED'&&data.active===false:pending.action==='resume'||pending.action==='start'?data.active===true:pending.action==='prepare'?data.prepared===true&&data.active===false:false);if(accepted){const done=pending;pending=null;lastError='';latchedRejection=false;if(data.state==='REJECTED')launch=null;else launchAccepted(done,data);}}
  }
 }
 window.objectVoicePreferences=want=>{if(want===mode){const d=distanceRead();if(distanceValid(d))return d;}try{const d=Number(window.localStorage.getItem(want==='follow'?'acg138.object.distance.follow.v2':'acg138.object.distance.static'));if(distanceValid(d))return d;}catch(e){}return .2;};
 window.ACGObjectTaskActions={prepare:()=>submit('prepare'),start:()=>submit('start'),stop:()=>{cancelLaunch('cancel');return submit('cancel');},pause:()=>submit('pause'),resume:()=>submit('resume'),mode:setMode,status:()=>task};
 $('objectModeStatic').addEventListener('click',()=>setMode('static'));$('objectModeFollow').addEventListener('click',()=>setMode('follow'));
 $('prepareObjectTask').addEventListener('click',()=>submit('prepare'));$('startObjectTask').addEventListener('click',()=>submit('start'));$('cancelObjectTask').addEventListener('click',()=>{cancelLaunch('cancel');submit('cancel');});
 $('pauseObjectTask').addEventListener('click',()=>submit('pause'));$('resumeObjectTask').addEventListener('click',()=>submit('resume'));$('objectStandoff').addEventListener('change',saveDistance);
 $('objectMapRoute').addEventListener('click',()=>$('openNavigationMap').click());
 setInterval(()=>{if($('objectDialog').open||launch||task&&(task.active||task.state==='PAUSED')||pending){if(window.Rover&&window.Rover.objectReadTask)window.Rover.objectReadTask();update();}},350);
 switchMode('static',false);update();
})();
