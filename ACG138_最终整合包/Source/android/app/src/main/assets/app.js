'use strict';
(() => {
 const $ = id => document.getElementById(id);
 const native = typeof window.Rover !== 'undefined';
 let state=null, receivedAt=0, demo=false, connected=false, pendingMode=null, pendingAt=0, pendingAutoRequest=null;
 let activePointer=null, activeButton=null, lastVideoAt=0, view='stereo', frameImage=null, connecting=false;
 const canvas=$('videoCanvas'),ctx=canvas.getContext('2d',{alpha:false});
 const driveButtons=[...document.querySelectorAll('[data-motion]')];
 function call(name,...args){if(native)try{if(name==='command'&&typeof window.Rover.commandWithOrigin==='function')return window.Rover.commandWithOrigin(args[0],args[1],args[2]||'UI_'+String(args[0]).toUpperCase());return window.Rover[name](...args);}catch(e){message('操作失败，请重新连接');}return null;}
 function displayReason(text){const autoReasons={WAIT_REAL_TICK_FIRST_ACTION:'等待控制器选择可执行动作',WAIT_FRESH_SENSORS_OR_FIRST_ACTION:'等待当前定位、扫描或首个可行动作',WAIT_POST_ARM_FRESH_COMMAND:'任务已接收，等待底盘执行新动作',RK_SHARED_AUTO_CONTROLLER:'自动巡航运行中',AUTO_WAIT_CURRENT_FEASIBLE_ACTION:'当前方向暂不可行，继续检查其他动作',WAIT_CURRENT_FEASIBLE_ACTION_CONTINUE_EXPLORING:'当前尚无可行动作，持续观察并选择其他方向',AUTO_REQUEST_EXPIRED:'自动模式授权在板端接收前已过期，请重新操作',AUTO_SWITCH_GENERATION_REVOKED:'停止或手动操作已撤销本次自动任务',AUTO_PHONE_LINK_EXPIRED:'手机心跳已过期，自动任务已撤销',WAIT_FRESH_LOCALIZATION:'等待新的可信定位',WAIT_FRESH_WHEEL_FEEDBACK:'等待新的轮速反馈'};return autoReasons[text]||(typeof window.ACGR4ReasonText==='function'?(window.ACGR4ReasonText(text)||text):text);}
 function autoIntent(){const a=state?.autonomy;return !!(a&&a.mode_known===true&&a.auto_intent_accepted===true);}
 function autoWaiting(){return autoIntent()&&(state.autonomy.auto_waiting===true||state.autonomy.auto_active!==true);}
 function message(text){$('message').textContent=displayReason(text);}
 // 清空子节点，效果等同空列表的replaceChildren，兼容旧WebView。
 function clearChildren(element){while(element.firstChild)element.removeChild(element.firstChild);}
 function stop(origin='UI_STOP'){if(window.objectLaunchCancel)window.objectLaunchCancel('stop');activePointer=null;if(activeButton)activeButton.classList.remove('active');activeButton=null;call('command','stop','',origin);if(demo&&state){state.control.moving=false;state.control.mode='manual';state.telemetry.robot.mode='manual';render();}}
 function dataFresh(){return connected&&state&&performance.now()-receivedAt<2500&&state.control.telemetry_fresh;}
 function canDrive(motion){const c=state?.control||{};return connected&&c.manual_ready===true&&c.final_motor_fault!==true&&!pendingMode;}
 function number(id,value,digits=1,signed=false){$(id).textContent=Number.isFinite(value)?`${signed&&value>0?'+':''}${value.toFixed(digits)}°`:'—';}
 function render(){
  const fresh=dataFresh(),ctl=state?.control||{},t=state?.telemetry||{},robot=t.robot||{},imu=t.imu||{};
  const online=connected; // 连接状态由原生控制ACK租约决定，遥测年龄只影响显示质量。
  const simulated=demo||state?.simulated;
  $('connectionLabel').textContent=simulated?'模拟连接':online?'已连接':connected?'通信中断':'连接设备';
  $('connection').classList.toggle('online',online);$('demoBadge').hidden=!simulated;
  $('motionStatus').textContent=!online?'未连接':!fresh?'数据超时':ctl.moving?(robot.action||'运动中'):autoWaiting()?'自动任务等待动作':ctl.mode==='auto'?'自动等待':'已停车';
  $('motionStatus').classList.toggle('warning',online&&!fresh);
  const imuLive=online&&imu.status==='live'&&(imu.age_ms||0)+(performance.now()-receivedAt)<1000;
  number('roll',imuLive?imu.roll:NaN,1,true);number('pitch',imuLive?imu.pitch:NaN,1,true);number('yaw',imuLive?imu.yaw:NaN);
  $('imuStatus').textContent=imuLive?'实时':imu.status==='invalid'?'数据异常':online?'等待有效数据':'等待数据';$('imuStatus').classList.toggle('live',imuLive);
  const ranges=robot.ranges||[];let rangeOK=fresh;
  for(let i=0;i<3;i++){
   const el=$(`range${i}`),r=ranges[i]||{},held=fresh&&r.state==='hold';
   const ok=fresh&&(r.state==='live'||held)&&Number.isFinite(r.mm)&&r.mm>0;clearChildren(el);el.classList.remove('warning','danger');
   if(ok){el.append(document.createTextNode((r.mm/10).toFixed(r.mm%10?1:0)));const unit=document.createElement('small');unit.textContent=held?'cm · 保持':r.age_known===false?'cm · 缓存':'cm';el.append(unit);if(r.near||r.mm<(r.stop_mm??(i===0?30:80)))el.classList.add('danger');else if(held)el.classList.add('warning');}
   else{el.textContent=!fresh?'—':r.state==='max'?'＞450 cm':r.state==='blind'?'盲区':r.state==='lost'?'失联':r.state==='waiting'?'等待':r.state==='invalid'?'异常':'—';el.classList.toggle('warning',online);}
   rangeOK=rangeOK&&(r.state==='live'||r.state==='max');
  }
  const zeroHeld=fresh&&ranges.some(r=>r.raw_zero_ignored===true),holding=fresh&&ranges.some(r=>r.state==='hold'),ageUnknown=fresh&&ranges.some(r=>r.age_known===false);
  $('rangeStatus').textContent=!online?'等待数据':!fresh?'数据超时':holding?'缓存保持／观察':ageUnknown?'报文正常 · 测距年龄未知':rangeOK?'实时':'检查测距';$('rangeStatus').classList.toggle('live',rangeOK);
  if(pendingMode==='auto'&&pendingAutoRequest&&state.autonomy?.request_us===pendingAutoRequest&&['DENIED','REJECTED','EXPIRED'].includes(state.autonomy?.status)){message('自动模式未启动 · '+(displayReason(state.autonomy.reason)||'条件未满足'));pendingMode=null;pendingAutoRequest=null;}
  if(pendingMode==='auto'&&pendingAutoRequest&&state.autonomy?.request_us===pendingAutoRequest&&autoIntent()){pendingMode=null;message(autoWaiting()?'自动任务已接收 · '+(displayReason(state.autonomy.reason)||'等待可执行动作'):'自动任务执行中');}
  const modeAck=fresh&&ctl.mode===pendingMode&&(pendingMode!=='auto'||state.autonomy?.auto_intent_accepted===undefined||pendingAutoRequest&&state.autonomy?.request_us===pendingAutoRequest);
  if(pendingMode&&(modeAck||performance.now()-pendingAt>(pendingMode==='auto'?3500:1800))){if(pendingMode==='auto'&&!modeAck&&!autoIntent())message('自动请求仍待板端确认；请查看任务回报，未据此停车或重复启动');pendingMode=null;}
  $('manualMode').classList.toggle('selected',fresh&&ctl.mode==='manual'&&!autoIntent());$('autoMode').classList.toggle('selected',fresh&&(ctl.mode==='auto'||autoIntent()));
  $('manualMode').classList.toggle('pending',pendingMode==='manual');$('autoMode').classList.toggle('pending',pendingMode==='auto');
  $('manualMode').disabled=!connected||!ctl.supported||!!pendingMode;
  $('autoMode').disabled=!online||!fresh||!(ctl.auto_ready??ctl.ready)||!!pendingMode||ctl.mode==='auto'||autoIntent();
  driveButtons.forEach(b=>{b.disabled=!canDrive(b.dataset.motion);});
  if(activeButton&&ctl.final_motor_fault===true)stop(); // 仅处理最终电机故障，绘制、测距和定位质量不触发该故障。
  $('modeHint').textContent=pendingMode?'正在切换，等待板端确认':!online?'连接后可操作':!fresh?'状态显示暂时过期，控制独立运行':!ctl.supported?'板端尚未启用遥控':autoWaiting()?'自动任务已接收 · '+(displayReason(state.autonomy.reason)||'等待可执行动作')+'；车轮当前'+(ctl.moving?'正在运动':'未运动')+'，持续检查新动作':ctl.mode==='auto'?(displayReason(ctl.reason)||'自动模式 · 可随时停车'):!ctl.ready?(displayReason(ctl.reason)||'等待板端就绪'):'手动遥控 · 松手停车';
  const videoOld=lastVideoAt&&performance.now()-lastVideoAt>1200;$('videoNotice').hidden=!videoOld&&!demo;$('videoNotice').textContent=demo?'模拟画面 · 非实时摄像头':'画面已超时';
  $('freshness').textContent=online?`${demo?'模拟数据 · ':''}数据 ${Math.round(performance.now()-receivedAt)} ms`:'ACG138 · ROBOT CONTROL';
  const fp=state?.fpga_policy,hex=x=>Number.isInteger(x)&&x>=0?'CFG'+x.toString(16).toUpperCase().padStart(8,'0'):'未收到';
  const identity='本App要求CFG31110301／状态5 · FPGA回报'+hex(fp?.received_config_id)+'／状态'+(Number.isInteger(fp?.received_status_revision)?fp.received_status_revision:'未知');
  if(fp?.mismatch===true)$('modeHint').textContent=identity+'，请成套更新FS和RK';
  if($('diagMotionIdentity'))$('diagMotionIdentity').textContent=identity+(fp?.mismatch===true?' · 策略不匹配':'');
  renderDiagnostics(fresh,online,ctl,robot,imu);
  renderOdometry(state?.odometry,online);
  if(typeof window.navReceiveMotorState==='function')window.navReceiveMotorState(robot,fresh&&online);
 }
 function nativeVideoLayout(){if(native&&typeof window.Rover.videoLayout==='function'){const r=canvas.getBoundingClientRect();window.Rover.videoLayout(r.x,r.y,r.width,r.height,innerWidth,innerHeight,'stereo');}document.querySelector('.video-viewport').dataset.nativeView='stereo';}
 window.onNativeVideo=(width,height,fps)=>{if(demo||!connected)return;document.documentElement.classList.add('native-video');lastVideoAt=performance.now();$('videoEmpty').hidden=true;$('eyeLabels').hidden=false;$('resolution').textContent=width+' × '+height+(fps>0?' · '+fps.toFixed(1)+' FPS':'');};
 window.nativeVideoError=(text)=>{if(demo)return;document.documentElement.classList.remove('native-video');frameImage=null;drawVideo();$('videoEmpty').hidden=false;$('eyeLabels').hidden=true;$('videoEmptyTitle').textContent='视频重新连接中';$('videoEmptyHint').textContent=text||'等待实时视频';};
 function resizeCanvas(){nativeVideoLayout();const r=canvas.getBoundingClientRect(),d=Math.min(window.devicePixelRatio||1,2);canvas.width=Math.max(1,Math.round(r.width*d));canvas.height=Math.max(1,Math.round(r.height*d));drawVideo();}
 function drawVideo(){const w=canvas.width,h=canvas.height;ctx.fillStyle='#0a1012';ctx.fillRect(0,0,w,h);if(!frameImage)return;const iw=frameImage.naturalWidth||frameImage.width,ih=frameImage.naturalHeight||frameImage.height;let sx=0,sw=iw;if(view!=='stereo'){sw=Math.floor(iw/2);if(view==='right')sx=sw;}const scale=Math.min(w/sw,h/ih),dw=sw*scale,dh=ih*scale;ctx.drawImage(frameImage,sx,0,sw,ih,(w-dw)/2,(h-dh)/2,dw,dh);if(view==='stereo'){ctx.fillStyle='#061114';ctx.fillRect(w/2-1,0,2,h);}}
 window.onVideo=(base64,width,height)=>{if(demo||!connected)return;const img=new Image();img.onload=()=>{frameImage=img;lastVideoAt=performance.now();$('videoEmpty').hidden=true;$('eyeLabels').hidden=false;$('resolution').textContent=`${width} × ${height}`;drawVideo();};img.src='data:image/jpeg;base64,'+base64;};
 window.receiveState=(incoming)=>{
  if(demo)return;
  let parsed;
  try{parsed=typeof incoming==='string'?JSON.parse(incoming):incoming;}
  catch(e){console.error('ACG138 state JSON parse failure',e);message('状态数据显示异常，保留上次数据；控制独立运行');return;}
  try{
   state=parsed;receivedAt=performance.now();connected=true;
   if(state.control?.error)message(state.control.error);
   else if(!activeButton&&!pendingMode)message(autoWaiting()?('自动任务已接收 · '+(displayReason(state.autonomy.reason)||'等待可执行动作')):state.control?.mode==='auto'?('自动模式 · '+(displayReason(state.control?.reason)||'可随时停车')):state.control?.reason||'连接正常 · 按住方向键行驶');
   render();
  }catch(e){const detail=String(e&&e.stack||e).slice(0,1200);console.error('ACG138 state UI rendering failure',detail);if(detail!==window.lastUiError){window.lastUiError=detail;call('reportUiError',detail);}message('界面更新暂时失败，控制独立运行');}
 };
 function connectionBusy(value){connecting=value;$('connectSubmit').disabled=value;$('connectSubmit').textContent=value?'正在连接…':$('manualConnection').open?'连接此地址':'搜索并连接';}
 window.connectionError=(text)=>{if(demo)return;document.documentElement.classList.remove('native-video');connectionBusy(false);connected=false;state=null;stop();$('connectProgress').textContent='';$('connectError').textContent=text;if(text.includes('未找到'))$('manualConnection').open=true;message(text);render();};
 window.connectionProgress=(text)=>{$('connectProgress').textContent=text;message(text);};
 window.connectionEstablished=(host)=>{connectionBusy(false);$('connectProgress').textContent='';$('connectDialog').close();message('已连接小车 · 正在读取实时数据');};
 function connectHost(host){document.documentElement.classList.remove('native-video');stop();demo=false;state=null;connected=false;frameImage=null;lastVideoAt=0;pendingMode=null;$('videoEmpty').hidden=false;$('eyeLabels').hidden=true;$('discoveryDevices').hidden=true;$('connectError').textContent='';connectionBusy(true);drawVideo();call('connect',host,$('tokenInput').value.trim());render();}
 window.discoveryFound=(hosts)=>{connectionBusy(false);const list=$('discoveryDevices');clearChildren(list);$('connectProgress').textContent='发现多台小车，请选择要连接的一台';for(const host of hosts){const button=document.createElement('button');button.type='button';button.textContent='ACG138 小车 · '+host;button.addEventListener('click',()=>connectHost(host));list.append(button);}list.hidden=false;};
 window.stateReadError=(text)=>{if(!demo)message(text);};
 window.controlLinkStatus=(text)=>{if(!demo)message(text);};
 window.manualControlUnavailable=(text)=>{stop();message(text);};
 window.appPaused=()=>{if(!(window.semanticOwns&&window.semanticOwns())){if(window.objectLaunchCancel)window.objectLaunchCancel('background');stop('LIFECYCLE_PAUSE');}pendingMode=null;if(connecting){cancelSearch();$('connectProgress').textContent='搜索已暂停，请重新连接';}};
 driveButtons.forEach(button=>{
  button.addEventListener('pointerdown',e=>{e.preventDefault();if(!canDrive(button.dataset.motion))return;if(activePointer!==null){stop();return;}if(window.objectLaunchCancel)window.objectLaunchCancel('manual');activePointer=e.pointerId;activeButton=button;button.setPointerCapture(e.pointerId);button.classList.add('active');call('command','hold',button.dataset.motion);if(demo){state.control.moving=true;render();}message(button.textContent.trim()+' · 松手停车');});
  const release=e=>{if(activePointer===e.pointerId){e.preventDefault();stop('MANUAL_RELEASE');message('已松开 · 停车指令已发送');}};
  button.addEventListener('pointerup',release);button.addEventListener('pointercancel',release);button.addEventListener('lostpointercapture',release);
 });
 $('stop').addEventListener('pointerdown',e=>{e.preventDefault();pendingMode=null;stop();message(connected?'停车指令已发送，等待板端回报':'未连接 · 已清除运动操作');});
 window.autoModeQueued=id=>{pendingAutoRequest=id;message('自动请求已排队，等待板端接收任务；尚不代表车轮已经运动');render();};
 window.autoModeError=text=>{pendingMode=null;pendingAutoRequest=null;message('自动请求反馈异常 · '+text+'；实际任务状态以板端回报为准');render();};
 function setMode(mode){stop();pendingAutoRequest=null;pendingMode=mode;pendingAt=performance.now();call('command',mode,'');if(demo){state.control.mode=mode;state.telemetry.robot.mode=mode;state.control.moving=mode==='auto';pendingMode=null;}render();}
 $('manualMode').addEventListener('click',()=>setMode('manual'));$('autoMode').addEventListener('click',()=>setMode('auto'));
 function openDialog(){if(!(window.semanticOwns&&window.semanticOwns()))stop('CONNECTION_DIALOG_OPEN');$('connectError').textContent='';$('connectProgress').textContent='';$('discoveryDevices').hidden=true;connectionBusy(false);$('connectDialog').showModal();}
 function cancelSearch(){if(connecting)call('cancelConnect');connectionBusy(false);}
 $('connection').addEventListener('click',openDialog);$('closeDialog').addEventListener('click',()=>{cancelSearch();$('connectDialog').close();});
 $('connectDialog').addEventListener('cancel',()=>{if(!(window.semanticOwns&&window.semanticOwns()))stop();cancelSearch();});
 $('manualConnection').addEventListener('toggle',()=>{if(!connecting)connectionBusy(false);});
 $('connectForm').addEventListener('submit',e=>{e.preventDefault();if(connecting)return;if(!(window.semanticOwns&&window.semanticOwns()))stop();const host=$('manualConnection').open?$('hostInput').value.trim():'',token=$('tokenInput').value.trim();if(host&&(!/^(?:\d{1,3}\.){3}\d{1,3}(?::\d{1,5})?$/.test(host)||host.split(':')[0].split('.').some(n=>+n>255)||host.includes(':')&&(+host.split(':')[1]<1||+host.split(':')[1]>65535))){return $('connectError').textContent='请输入有效的 IPv4 地址，例如 10.169.79.118:8080';}if(!/^[A-Za-z0-9_-]{6,64}$/.test(token)){return $('connectError').textContent='请输入连接密码，默认是 123456';}if(!native){return $('connectError').textContent='电脑预览页请使用模拟演示；真实连接请安装 APK';}connectHost(host);message(host?'正在连接小车…':'正在自动查找小车…');});
 $('disconnectButton').addEventListener('click',()=>{document.documentElement.classList.remove('native-video');stop();cancelSearch();call('disconnect');demo=false;connected=false;state=null;frameImage=null;lastVideoAt=0;pendingMode=null;$('videoEmpty').hidden=false;$('eyeLabels').hidden=true;$('resolution').textContent='等待画面';$('connectDialog').close();message('已断开连接');drawVideo();render();});
 function demoImage(){const d=document.createElement('canvas');d.width=1280;d.height=720;const c=d.getContext('2d');for(let eye=0;eye<2;eye++){c.save();c.translate(eye*640,0);c.fillStyle='#24383e';c.fillRect(0,0,640,720);c.fillStyle='#31474b';c.beginPath();c.moveTo(0,0);c.lineTo(640,0);c.lineTo(350,260);c.lineTo(290,260);c.fill();c.fillStyle='#33474a';c.beginPath();c.moveTo(0,720);c.lineTo(640,720);c.lineTo(350,260);c.lineTo(290,260);c.fill();c.strokeStyle='#75969755';c.lineWidth=2;for(let n=0;n<8;n++){let y=260+Math.pow(n/7,2)*460;c.beginPath();c.moveTo(0,y);c.lineTo(640,y);c.stroke();}for(let n=-3;n<=3;n++){c.beginPath();c.moveTo(320+n*20,260);c.lineTo(320+n*200,720);c.stroke();}c.fillStyle='#758b83';for(let n=0;n<4;n++)c.fillRect(170+n*30,75+n*38,210-n*38,12-n*2);const shift=eye*12;c.strokeStyle='#83e5bf';c.lineWidth=3;c.strokeRect(260+shift,250,92,210);c.fillStyle='#f8f8f8';c.fillRect(260+shift,213,180,37);c.fillStyle='#000';c.font='bold 26px sans-serif';c.fillText('person 94%',268+shift,240);c.fillStyle='#83b0a466';c.beginPath();c.arc(306+shift,285,22,0,Math.PI*2);c.fill();c.fillRect(281+shift,312,50,80);c.fillRect(282+shift,392,18,64);c.fillRect(313+shift,392,18,64);c.fillStyle='#b4c4bf';c.font='15px sans-serif';c.fillText('DEMO / 模拟画面',24,687);c.restore();}frameImage=d;$('resolution').textContent='1280 × 720 · 模拟';$('videoEmpty').hidden=true;$('eyeLabels').hidden=false;lastVideoAt=performance.now();drawVideo();}
 function startDemo(){document.documentElement.classList.remove('native-video');stop();call('disconnect');demo=true;connected=true;pendingMode=null;state={control:{ready:true,manual_ready:true,final_motor_fault:false,telemetry_fresh:true,supported:true,mode:'manual',moving:false,reason:'模拟演示 · 不连接真实小车'},telemetry:{imu:{status:'live',age_ms:0,roll:2.4,pitch:-1.8,yaw:128.6},robot:{mode:'manual',ranges:[{mm:1380,state:'live'},{mm:820,state:'live'},{mm:2140,state:'live'}]}}};receivedAt=performance.now();$('connectDialog').close();demoImage();message('模拟演示 · 按键不会控制真实小车');render();}
 $('demoButton').addEventListener('click',()=>{cancelSearch();startDemo();});
 document.addEventListener('visibilitychange',()=>{if(document.hidden&&!(window.semanticOwns&&window.semanticOwns()))stop('DOCUMENT_HIDDEN');});window.addEventListener('pagehide',()=>{if(!(window.semanticOwns&&window.semanticOwns())){stop('PAGE_HIDE');call('disconnect');}});window.addEventListener('resize',resizeCanvas);document.addEventListener('contextmenu',e=>e.preventDefault());
 setInterval(()=>{if(demo){receivedAt=performance.now();lastVideoAt=performance.now();}if(!document.hidden){call('pulse');if(activeButton&&connected&&state?.control?.final_motor_fault!==true)call('renewHold',activeButton.dataset.motion);}render();},80);
 // BEGIN FAULT DISPLAY
 const diagnosticReasons=['正常（未报告额外保护）','前方过近','侧方过近','测距失效或等待','电机反馈保护','数据保持 / 观察 · 限速或等待','低速恢复或绕行','自动复测净空','原地转向超时（旧版状态）· 自动复测','当前方向安全条件不满足','等待编码器初始化','手动停车 / 没有运动指令'];
 const diagnosticRangeStates={live:'报文有效（非零样本年龄见通道详情）',max:'超量程（非故障）',hold:'缓存保持 / 观察',blind:'盲区或非零过近',lost:'测距失效',waiting:'等待可信测距',invalid:'测距异常',stale:'数据过期'};
 const diagnosticRangeErrors=['正常（未报告接收错误）','传感器帧校验错误','UART 接收错误（串口帧）','传感器帧内字节超时','盲区 / 非零过近','尚无可信测距','测距失效（断流或观察超时等）','数据保持 / 近障确认或恢复'];
 // 反馈阶段来自A3显式遥测，不仅凭FB推断。
 function recoveryInfo(value){
  const supported=value?.feedbackSupported??value?.feedback_supported,phase=value?.feedbackPhase??value?.feedback_phase,retries=value?.feedbackRetries??value?.feedback_retries;
  if(supported!==true||!Number.isInteger(phase)||phase<0||phase>4||!Number.isInteger(retries)||retries<0||retries>2)return null;
  return {phase,retries,text:['反馈正常','受限恢复','等待重试','低速验证','重试用尽 · 已停机'][phase]};
 }
 let lastDiagnosticSnapshot=null,lastDiagnosticSource=null;
 function diagnosticInteger(value,max=65535){return typeof value==='number'&&Number.isInteger(value)&&value>=0&&value<=max?value:null;}
 function diagnosticSnapshot(ctl,robot,imu){return {at:receivedAt,reason:diagnosticInteger(robot.stop_reason),feedback:diagnosticInteger(robot.feedback,15),feedbackSupported:robot.feedback_supported===true,feedbackPhase:diagnosticInteger(robot.feedback_phase,4),feedbackRetries:diagnosticInteger(robot.feedback_retries,2),actual:diagnosticInteger(robot.actual),remoteFlags:diagnosticInteger(robot.remote_flags,15),supported:typeof ctl.supported==='boolean'?ctl.supported:null,driveStatus:typeof ctl.drive_status==='string'?ctl.drive_status:'',mode:ctl.mode||robot.mode||'',navState:diagnosticInteger(robot.nav_state),nearMask:diagnosticInteger(robot.near_mask,7),ranges:[0,1,2].map(i=>{const r=robot.ranges?.[i]||{};return {state:typeof r.state==='string'?r.state.slice(0,48):'',error:diagnosticInteger(r.error),mm:diagnosticInteger(r.mm),near:r.near===true,rawZero:r.raw_zero_ignored===true,ageKnown:r.age_known===true,ageMin:diagnosticInteger(r.nonzero_age_min_ms),ageMax:diagnosticInteger(r.nonzero_age_max_ms)};}),imu:{status:typeof imu.status==='string'?imu.status:'',age:diagnosticInteger(imu.age_ms,4294967295)}};}
 function renderDiagnostics(fresh,online,ctl,robot,imu){
  if(state&&(state!==lastDiagnosticSource||!lastDiagnosticSnapshot||receivedAt!==lastDiagnosticSnapshot.at)){lastDiagnosticSnapshot=diagnosticSnapshot(ctl,robot,imu);lastDiagnosticSource=state;}
  const snapshot=lastDiagnosticSnapshot,button=$('diagnosticsButton'),label=$('diagnosticsSummary');
  const failed=!connected&&!connecting&&!!$('connectError').textContent;
  const current=!!state&&fresh,elapsed=snapshot?Math.max(0,Math.round(performance.now()-snapshot.at)):null;
  const prefix=current?'':'上次：',fb=snapshot?.feedback,reason=snapshot?.reason;
  const recovery=recoveryInfo(snapshot);
  const channels=fb===null||fb===undefined?[]:[0,1,2,3].filter(i=>fb&(1<<i)).map(i=>'M'+(i+1));
  let level='normal',summary='状态详情';
  if(!current){
   if(connecting){level='info';summary='正在连接';}
   else if(failed){level='stale';summary='通信失败 · 查看详情';}
   else if(connected&&!online){level='stale';summary='通信中断 · 上次数据';}
   else if(connected){level='stale';summary='遥测过期 · 上次数据';}
   else if(snapshot){level='stale';summary='未连接 · 上次数据';}
  }else if(recovery?.phase){level=recovery.phase===4?'danger':'warning';summary=recovery.text+' · '+recovery.retries+'/2';}
  else if(fb>0||reason===4){level='danger';summary='反馈异常 / 保护锁定'+(channels.length?' · '+channels.join('/'):'');}
  else if(snapshot.ranges.some(r=>r.state!=='stale'&&[1,2,3].includes(r.error))){level='danger';summary='测距接收错误';}
  else if(reason===10){level='info';summary='等待编码器初始化';}
  else if(snapshot.remoteFlags!==null&&(snapshot.remoteFlags&4)){level='warning';summary='遥控看门狗超时';}
  else if(snapshot.remoteFlags!==null&&(snapshot.remoteFlags&8)){level='info';summary='本地按键接管';}
  else if(reason===1||reason===2){level='warning';summary=diagnosticReasons[reason]+' · 保护';}
  else if(reason===3||snapshot.ranges.some(r=>r.state==='lost'||r.error===6)){level='warning';summary='测距失效或等待';}
  else if(reason===5||snapshot.ranges.some(r=>r.state==='hold'||r.error===7)){level='warning';summary='测距保持 / 观察';}
  else if(reason===8||reason===9){level='warning';summary=reason===8?'原地转向超时（旧版状态）· 自动复测':'方向安全保护';}
  else if(reason===6||reason===7){level='info';summary=diagnosticReasons[reason];}
  else if(snapshot.ranges.some(r=>r.state==='blind'||r.error===4)){level='warning';summary='测距盲区 · 保护';}
  else if(imu.status==='invalid'){level='warning';summary='IMU 数据异常';}
  else if(reason!==null&&reason>11){level='info';summary='未知原因码 '+reason;}
  button.dataset.level=level;label.textContent=summary;button.querySelector('.diagnostics-icon').textContent=level==='danger'||level==='stale'?'!':'i';
  button.setAttribute('aria-label',summary+'，查看状态与错误详情');
  renderEscapeDisplay(fresh,online,ctl,robot,imu);
  if(!$('diagnosticsDialog').open)return;
  const status=$('diagDataStatus');status.dataset.level=level;
  status.textContent=current?(demo||state?.simulated?'模拟遥测 · 不代表实车状态':'板端遥测新鲜 · 以下为本次回报'):failed?(snapshot?'通信失败；以下为上次数据（非实时）':'通信失败，尚无可用板端数据'):connecting?(snapshot?'正在连接；以下为上次数据（非实时）':'正在连接，等待板端数据'):snapshot?(connected?'遥测已过期 / 通信中断；以下为上次数据（非实时）':'当前未连接；以下为上次数据（非实时）'):'未连接，尚未收到板端数据';
  $('diagFreshness').textContent=snapshot?`距本页最后接收 ${elapsed} ms · 板端遥测标记：${state?(ctl.telemetry_fresh?'新鲜':'过期 / 不可用'):'当前无数据'}${current?'':' · 不能据此判断当前故障'}`:'没有历史遥测可显示';
  $('diagReason').textContent=snapshot&&reason!==null?prefix+'stop_reason='+reason+' · '+(diagnosticReasons[reason]||'未知原因（保留原始数值）'):'未上报 stop_reason';
  $('diagFeedbackRecovery').textContent=recovery?prefix+recovery.text+' · 已用额外重试 '+recovery.retries+'/2':prefix+'未上报恢复阶段（旧版遥测）';
  $('diagFeedback').textContent=snapshot&&fb!==null?prefix+'FB=0x'+fb.toString(16).toUpperCase()+' · '+(channels.length?channels.join('、')+(recovery?.phase?' 本轮反馈记录 · '+recovery.text:' 反馈异常 / 保护锁定（未检测到有效运动计数）'):'未触发保护'):'未上报 FB';
  for(let i=0;i<4;i++){const item=$('diagMotor'+i),reported=fb!==null&&fb!==undefined,active=reported&&!!(fb&(1<<i));item.textContent='M'+(i+1)+' · '+(!reported?'未上报':active?(current?(recovery?.phase?(recovery.phase===4?'重试用尽 / 已停机':'本轮反馈记录'):'反馈异常 / 保护锁定'):'上次反馈记录'):(current?'未触发保护':'上次未触发保护'));item.classList.toggle('fault',current&&active);item.classList.toggle('historical',!current&&reported);}
  for(let i=0;i<3;i++){
   const r=snapshot?.ranges[i],item=$('diagRange'+i);
   if(!r||(!r.state&&r.error===null&&r.mm===null)){item.textContent='未上报';continue;}
   const stale=!current||r.state==='stale',rprefix=stale?'上次：':'';
   const rangeName=diagnosticRangeStates[r.state]||'未知测距状态';
   const value=r.state==='max'?'＞450 cm':r.mm!==null&&r.mm>0?r.mm+' mm':'距离未上报';
   const ageText=r.ageKnown&&r.ageMin!==null?(r.ageMax===null?'≥'+r.ageMin+' ms':'['+r.ageMin+','+r.ageMax+') ms'):'未知 / 无可信非零缓存';
   item.textContent=rprefix+rangeName+' · state='+(r.state||'未上报')+' · '+value+' · error='+(r.error===null?'未上报':r.error)+'（'+(r.error===null?'含义未上报':diagnosticRangeErrors[r.error]||'未知错误码')+'）'+' · 非零样本上报年龄='+ageText+(r.near?' · 方向近障标记':'' )+(stale?' · 非实时':'');
  }
  const is=snapshot?.imu,imuNames={live:'有效姿态',invalid:'数据异常',waiting:'等待有效数据'};
  $('diagImu').textContent=is&&is.status?prefix+(imuNames[is.status]||'未知 IMU 状态')+' · status='+is.status+(is.age!==null?' · 板端 age_ms='+is.age:' · 未上报年龄')+(current&&is.status==='live'&&is.age!==null&&is.age+elapsed>=1000?' · 已过期':'')+(current?'':' · 非实时'):'未上报 IMU 状态';
  const flags=snapshot?.remoteFlags,remote=[];
  if(flags!==null&&flags!==undefined){remote.push('claimed='+((flags&1)?1:0)+'（'+((flags&1)?'已认领':'未认领')+'）');remote.push('live='+((flags&2)?1:0)+'（'+((flags&2)?'租约有效':'租约无效')+'）');remote.push('expired='+((flags&4)?1:0)+'（'+((flags&4)?'FPGA 报告看门狗超时':'未报告超时')+'）');remote.push('local_takeover='+((flags&8)?1:0)+'（'+((flags&8)?'本地按键接管':'未接管')+'）');}
  $('diagRemote').textContent=snapshot?prefix+(snapshot.supported===false?'控制服务未启用或遥测版本不支持；':'')+(flags===null?'remote_flags 未上报':'remote_flags=0x'+flags.toString(16).toUpperCase()+' · '+remote.join(' / ')):'未上报遥控会话';
  $('diagAction').textContent=snapshot&&snapshot.actual!==null?prefix+'actual='+snapshot.actual+' · 板端许可动作码，非实际轮速':'未上报动作码';
 }
 $('diagnosticsButton').addEventListener('click',openReadOnlyDiagnostics);
 $('closeDiagnostics').addEventListener('click',()=>{$('diagnosticsDialog').close();});
 // END FAULT DISPLAY
 // BEGIN ESCAPE8 DISPLAY
 let adviceBannerKey='',adviceDetailKey='';
 function escapeText(id,text){const el=$(id);if(el.textContent!==text)el.textContent=text;}
 function readOnlyAdvice(snapshot){
  const fb=snapshot?.feedback,reason=snapshot?.reason,ranges=snapshot?.ranges||[],channels=fb?[0,1,2,3].filter(i=>fb&(1<<i)).map(i=>'M'+(i+1)).join('、'):'';
  const make=(kind,level,title,known,possible,steps,show=true)=>({kind,level,title,known,possible,steps,show});
  const recovery=recoveryInfo(snapshot);
  if(recovery?.phase){
   const locked=recovery.phase===4;
   return make('feedback-recovery',locked?'danger':'warning',recovery.text+' · '+recovery.retries+'/2',
    '板端反馈阶段：'+recovery.text+'；本轮涉及 '+(channels||'未报告通道')+'，已使用额外重试 '+recovery.retries+'/2。',
    '反馈暂缺、低速起转不足、机械阻力或编码器链路异常均可能触发；阶段和 FB 无法单独定位硬件原因。',
    locked?['本轮最多两次额外重试已用尽，车子保持停止，不会继续自动试转。','检查对应电机、编码器连接与机械阻力；需要重新尝试时先明确点击停车，再重新启动。','用户STOP、真正控制断联与电机最终故障优先；手动不受测距和定位拦截。']:
    ['受限恢复最多300ms冻结输出；仍无反馈则停车等待至少1秒，再在原命令仍有效且测距许可时低速重试。','低速验证不超过20RPM，原目标更低时保持更低；所有驱动轮连续有效反馈并正常验证约2秒后才清除本轮记录。','可随时点击停车取消恢复；手动模式松手停车，AUTO不会因打开本页而被改成手动。']);
  }
  if(fb>0||reason===4)return make('feedback','danger','反馈异常 / 保护锁定','FPGA 已报告'+(channels?channels+' 逻辑通道':'电机通道')+'反馈保护，未检测到有效运动计数。','反馈计数未产生、反馈链路异常或驱动条件不足均可能出现该状态；仅凭 FB 无法确定供电、电机或接线损坏。',['先点击停车并记录 FB 原值和 M1–M4 逻辑通道；逻辑通道尚未标定到物理轮位。','完全断电后检查对应编码器 A/B 信号、供电 / 共地与插接，确认轮组没有机械卡滞，并检查电机驱动供电。','处理完成后完全断电上电清除锁存保护，再将车轮架空，手动按住一个方向作短时低速检查。','仍报错时停止试车，保留新的 FB 原值及板端日志；本页不会自动复位或试转。']);
  const errors=ranges.map((r,i)=>({r,i})).filter(x=>[1,2,3].includes(x.r.error));
  if(errors.length){const locations=['前方','左前','右前'];return make('range-receive','danger','测距接收错误',errors.map(x=>locations[x.i]+'：error='+x.r.error+'，'+diagnosticRangeErrors[x.r.error]).join('；')+'。','可能与串口接收、报文不完整或干扰有关，当前码不能确定某根线或某个传感器损坏。',['先查看对应测距通道是否恢复新鲜有效数据。','持续出现时检查该通道连接、串口条件及板端日志。','不把缓存距离当作新测量，不跳过现有运动保护。']);}
  if(snapshot?.remoteFlags!==null&&snapshot?.remoteFlags!==undefined&&(snapshot.remoteFlags&4))return make('watchdog','warning','遥控看门狗保护','FPGA 已报告遥控租约超时。','可能是手机进入后台、无线连接或控制服务未能及时续期，单凭此位不能定位链路根因。',['保持 App 在前台并确认手机与板子网络连接。','检查板端原运行终端的服务状态。','恢复连接后手动确认控制模式，不自动恢复旧运动。']);
  if(reason===3||ranges.some(r=>r.error===6||r.state==='lost'))return make('range-unavailable','warning','测距失效或等待保护','板端报告测距失效 / 等待；该状态也可能包含近障观察超时。','A2 遥测不能区分真实接收断流和所有观察超时原因，不能直接判断传感器断线。',['观察对应通道是否出现新的可信非零测距。','数据持续不可用时查看板端原终端的接收与状态日志。','保留当前避障和看门狗保护，不强行启动被禁止的方向。']);
  if(reason===1||reason===2||snapshot?.nearMask||ranges.some(r=>r.near))return make('near','warning','近障保护 · 可用方向脱困','板端已报告近障保护或对应方向不满足近障安全条件。','通常与当前障碍和测距结果有关；近障标记本身不是硬件故障。',['先观察真实环境，只使用圆盘上当前允许的方向。','左后退 / 右后退按车尾去向操作，按住行驶、松手停车。','等待有效清障数据，不自动退出 AUTO 或清除近障标记。']);
  if(reason===5||ranges.some(r=>r.error===7||r.state==='hold'))return make('range-hold','warning','数据保持 / 观察','板端正在使用缓存保持、近障确认或恢复状态。','同一个 A2 状态包含多种保持原因，无法判断正在第几笔确认；此状态不等于硬件故障。',['保持观察，允许方向仍由板端逐项决定。','等待新的有效非零数据，不强制恢复速度。','原观察时间、里程限制和看门狗继续生效。']);
  if(ranges.some(r=>r.error===4||r.state==='blind'))return make('blind','warning','测距盲区保护','对应测距通道报告盲区或非零过近。','可能是物体太近或回波不可用，不能仅凭盲区状态判定传感器损坏。',['观察传感器附近是否有过近物体。','使用当前允许的安全方向，不强行执行禁用方向。','等待可信测距恢复后再继续操作。']);
  if(reason===8||reason===9)return make('space-wait','warning',reason===8?'原地转向超时（旧版状态）· 自动复测':'方向安全条件不满足',reason===8?'板端报告兼容旧版的原地转向超时状态；新版会自动复测净空并寻找可用通路。':'当前请求方向的测距或空间条件不满足。','这是运动保护结果，不直接说明电机或传感器损坏。',['观察周围空间与三路测距状态。','仅使用当前允许的方向，必要时点击停车。','等待新数据重新评估，不自动重复原动作。']);
  if(reason===10)return make('encoder-wait','info','等待编码器初始化','板端仍在等待编码器初始化就绪。','这是启动 / 就绪状态，不代表需要先让车轮转动，也不能据此判断编码器损坏。',['先等待板端初始化完成。','持续等待时保留状态码并查看板端原终端日志。','不要为了初始化强行驱动车轮。']);
  if(snapshot?.imu?.status==='invalid')return make('imu','warning','IMU 数据异常','板端当前未提供有效的 IMU 姿态。','可能与姿态报文、IMU 会话或数据链路有关，当前字段不包含完整底层原因。',['保持 IMU 平稳并查看板端原运行终端日志。','等待有效姿态数据恢复。','本页不自动重新校准或重启采集。']);
  if(snapshot?.remoteFlags!==null&&snapshot?.remoteFlags!==undefined&&(snapshot.remoteFlags&8))return make('local','info','本地按键已接管','FPGA 已报告本地控制接管。','这是控制权状态，不是硬件故障。',['确认当前应由手机还是板端按键控制。','需恢复手机控制时手动建立控制会话。'],true);
  if(snapshot?.supported===false)return make('unsupported','info','手机控制尚未就绪','控制服务未启用，或当前遥测版本不支持手机控制。','该状态不能直接判断 FPGA 固件或硬件损坏。',['核对板端启动的是本次配套运行包。','在板端原终端查看服务与遥测状态。']);
  if(reason===6||reason===7)return make('normal-wait','info',reason===6?'低速恢复 / 减速运行':'自动复测净空','板端报告低速恢复或自动复测状态，按现有避障条件寻找可用通路。','这些是避障运行状态，不属于硬件故障。',['保持观察，等待板端条件满足。','需要停止时点击停车；本页不会自动改变模式。'],false);
  return make('normal','normal','未报告额外故障','当前没有收到需要额外处理的故障标记。','控制状态和动作码不等于实测车速，也不能证明车轮已经转动。',['继续观察画面、测距与板端状态。','按住方向键行驶、松手停车；AUTO 下可随时显式停车。'],false);
 }
 function driveStatusView(fresh,online,ctl,robot){
  if(!connected)return {status:'unknown',text:connecting?'正在连接':'未连接'};
  if(!online||!fresh)return {status:'unknown',text:online?'遥测过期':'通信中断'};
  const stopped=['停车','前方近障','侧方近障','测距保护','反馈锁定','保持等待','低速等待','清障等待','转向超时','方向保护','编码器初始化','手动停车'];
  const recovery=recoveryInfo(robot);
  if(recovery?.phase){if(robot.actual===0||recovery.phase===2||recovery.phase===4)return {status:'stopped',text:recovery.text};return {status:'slow',text:recovery.text};}
  if(robot.actual===0||robot.feedback>0||robot.stop_reason===4||ctl.drive_status==='stopped')return {status:'stopped',text:'停车'+(stopped[robot.stop_reason]&&robot.stop_reason!==0?' · '+stopped[robot.stop_reason]:'')};
  if(ctl.drive_status==='unknown')return {status:'unknown',text:'状态待确认'};
  if(ctl.drive_status==='slow')return {status:'slow',text:'减速运行'};
  if(ctl.drive_status==='normal')return {status:'normal',text:'正常运行'};
  if(!Number.isInteger(robot.actual)||robot.actual<=0)return {status:'unknown',text:'等待动作状态'};
  const auto=ctl.mode==='auto'||robot.mode==='auto';
  const slow=robot.stop_reason===5||robot.stop_reason===6||(auto&&[5,8].includes(robot.nav_state))||(auto&&[5,6].includes(robot.actual))||!!robot.near_mask;
  return {status:slow?'slow':'normal',text:slow?'减速 / 保护运行':'正常运行'};
 }
 function renderEscapeDisplay(fresh,online,ctl,robot,imu){
  const drive=driveStatusView(fresh,online,ctl,robot),indicator=$('driveIndicator');
  if(indicator.dataset.status!==drive.status)indicator.dataset.status=drive.status;
  escapeText('driveStatusText',drive.text);indicator.setAttribute('aria-label',drive.text+'，板端控制状态，非实测轮速');
  let advice=readOnlyAdvice(lastDiagnosticSnapshot),current=!!state&&fresh;
  const failed=!connected&&!connecting&&!!$('connectError').textContent;
  if(!current){
   const history=lastDiagnosticSnapshot;
   advice={kind:'stale-'+advice.kind,level:'stale',title:connecting?'正在连接':failed?'通信失败':online?'板端遥测过期':connected?'通信中断':'未连接',known:history?'上次回报：'+advice.known+' 这些是历史数据，不能据此判断当前状态。':'尚未收到可用板端遥测。',possible:'当前缺少新鲜数据，可能涉及网络、应用前台状态或板端数据服务；不能把历史码当成当前故障。',steps:['先确认手机与板子连接、App 前台状态以及板端原运行终端。','恢复新鲜遥测后再根据最新回报判断。','不根据历史数据自动复位、试转或恢复旧动作。'],show:failed||connected&&!connecting};
  }
  const key=JSON.stringify([advice.kind,advice.level,advice.title,advice.known,advice.possible,advice.steps,advice.show,current]);
  const banner=$('diagnosticAdviceBanner');
  if(key!==adviceBannerKey){
   adviceBannerKey=key;banner.hidden=!advice.show;banner.dataset.level=advice.level;
   escapeText('adviceTitle',(current?'':'历史 / 非实时 · ')+advice.title);
   escapeText('adviceSummary',advice.known);
   document.querySelector('.video-viewport').classList.toggle('has-advice',advice.show);
  }
  if($('diagnosticsDialog').open&&key!==adviceDetailKey){
   adviceDetailKey=key;escapeText('diagAdviceType',(current?'':'历史建议 · ')+advice.title);
   escapeText('diagAdviceKnown','已确定的回报：'+advice.known);
   escapeText('diagAdvicePossible','可能原因 / 能力边界：'+advice.possible);
   const list=$('diagAdviceSteps');clearChildren(list);for(const step of advice.steps){const li=document.createElement('li');li.textContent=step;list.append(li);}
  }
 }
 function openReadOnlyDiagnostics(){
  if(state?.control?.mode!=='auto'&&activePointer!==null)stop();
  adviceDetailKey='';if(!$('diagnosticsDialog').open)$('diagnosticsDialog').showModal();
  const ctl=state?.control||{},t=state?.telemetry||{};renderDiagnostics(dataFresh(),connected,ctl,t.robot||{},t.imu||{});
 }
 function diagnosticStop(){pendingMode=null;stop();message(connected?'停车指令已发送，等待板端回报':'未连接 · 已清除运动操作');}
 $('adviceDetails').addEventListener('click',openReadOnlyDiagnostics);
 $('diagnosticsStop').addEventListener('pointerdown',e=>{e.preventDefault();diagnosticStop();});
 $('diagnosticsStop').addEventListener('click',e=>{if(e.detail===0)diagnosticStop();});
 // END ESCAPE8 DISPLAY
 const saved=call('getConfig');if(saved)try{const config=JSON.parse(saved);$('hostInput').value=config.host||'';$('tokenInput').value=config.password||'123456';}catch(e){}
 // 按可用区域调整圆盘尺寸，保持不同手机上的圆形比例。
 const discStage=document.querySelector('.disc-stage'),disc=document.querySelector('.direction-disc');
 new ResizeObserver(()=>{const r=discStage.getBoundingClientRect();disc.style.setProperty('--disc-size',Math.floor(Math.min(r.width,r.height))+'px');}).observe(discStage);
 new ResizeObserver(resizeCanvas).observe(canvas);render();
 if(new URLSearchParams(location.search).get('demo')==='1'&&!native)startDemo();
})();
