 'use strict';
(()=>{const $=id=>document.getElementById(id);let pending=null,ackTimer=null,listenTimer=null;let busy=false,lastCandidates=-10000,lastCategory='',lastEpoch='';
const labels={idle:'等待开始',preparing:'准备中',permission:'等待麦克风授权',permission_granted:'授权成功 · 请再次点击',engine_check:'检查中文引擎',loading:'加载内置中文模型',ready:'请说话',listening:'正在听',recognizing:'正在识别',partial:'正在听 / 临时文字',result:'已取得最终文字',error:'需要处理',cancelled:'已取消'};
function clearTimers(){if(ackTimer!==null)clearTimeout(ackTimer);if(listenTimer!==null)clearTimeout(listenTimer);ackTimer=listenTimer=null;}
function render(state,message){$('voiceState').hidden=false;$('voiceState').dataset.state=state;$('voiceStage').textContent=labels[state]||'识别状态';$('voiceStatus').textContent=message;busy=['preparing','engine_check','loading','ready','listening','recognizing','partial'].includes(state);$('voiceListen').disabled=busy;$('voiceRetry').disabled=busy;$('voiceCancel').disabled=!busy;$('voiceNetwork').disabled=busy;$('voiceMic').textContent=['ready','listening','partial'].includes(state)?'● 正在录音':state==='loading'?'○ 模型加载中，尚未录音':state==='recognizing'?'○ 录音已结束，正在处理':'○ 麦克风未录音';if(!busy){clearTimers();$('voiceLevel').value=0;}}
const status=t=>render('error',t);
function nativeCancel(){try{if(window.Rover&&window.Rover.voiceCancel)window.Rover.voiceCancel();}catch(e){/* 桥接对象失效时仍释放本地状态。 */}}
window.voiceEvent=e=>{if(!e||typeof e.state!=='string')return;if(ackTimer!==null){clearTimeout(ackTimer);ackTimer=null;}if(e.state==='loading'){if(listenTimer!==null)clearTimeout(listenTimer);listenTimer=setTimeout(()=>{nativeCancel();status('内置中文模型加载超过45秒，已取消本次请求，请重试。');},45000);}if(e.state==='ready'){if(listenTimer!==null)clearTimeout(listenTimer);listenTimer=setTimeout(()=>{nativeCancel();status('本次离线识别等待超过17秒，已停止录音，请重试。');},17000);}if(e.state==='rms'){if(busy)$('voiceLevel').value=Math.max(0,Math.min(100,(Number(e.rms)+60)*100/60));return;}if(e.state==='partial'){$('voicePartial').textContent='临时文字：'+(e.text||'');render('partial',e.message||'临时文字仅显示，尚未理解命令');return;}if(e.text!=null){$('voiceTranscript').textContent='最终原文（未执行）：'+e.text;$('voiceText').value=e.text;}render(e.state,e.message||'');};
function listen(){clearTimers();$('voicePartial').textContent='临时文字：等待说话';$('voiceTranscript').textContent='最终原文：等待本次结果（上次文字仍可编辑）';$('voiceIntent').textContent='意图：等待最终结果';$('voiceOutcome').textContent='本次识别尚未提交命令';render('preparing','已点击说话，正在准备；首次使用请完成系统麦克风授权。');
 if(!window.Rover||!window.Rover.voiceListen){status('当前预览没有 Android 语音桥接服务。请使用包含离线模型的 APK，或直接输入文字。');return;}
 ackTimer=setTimeout(()=>{nativeCancel();status('Android 语音服务未确认请求；请返回前台重试，或输入文字。');},3000);
 listenTimer=setTimeout(()=>{nativeCancel();status('等待识别超过 17 秒，已请求释放麦克风。请重试或输入文字。');},17000);
 try{window.Rover.voiceListen($('voiceNetwork').checked);}catch(e){nativeCancel();status('无法调用 Android 语音服务，请重试或使用文字输入。');}
}
function refreshCandidates(){if(window.Rover&&window.Rover.voiceReadCandidates&&Date.now()-lastCandidates>=1000){lastCandidates=Date.now();window.Rover.voiceReadCandidates();}}
function openVoice(){refreshCandidates();if($('voiceDialog').open)$('voiceDialog').close();$('voiceDialog').showModal();if(window.Rover&&window.Rover.objectReadState)window.Rover.objectReadState();}
 for(const id of ['openVoice','objectVoiceEntry','navVoiceEntry'])$(id).addEventListener('click',openVoice);
$('closeVoice').addEventListener('click',()=>$('voiceDialog').close());$('voiceDialog').addEventListener('close',()=>{clearTimers();nativeCancel();render('cancelled','语音窗口已关闭，仅结束录音；已提交任务继续由RK执行。');});
$('voiceListen').addEventListener('click',listen);$('voiceRetry').addEventListener('click',listen);$('voiceCancel').addEventListener('click',()=>{clearTimers();nativeCancel();render('cancelled','已取消本次识别；可以重试，或编辑下方文字后提交。');});
$('voiceForm').addEventListener('submit',e=>{e.preventDefault();clearTimers();nativeCancel();render('recognizing','正在理解提交的文字…');if(window.Rover&&window.Rover.voiceText){ackTimer=setTimeout(()=>status('文字理解服务没有回应；请返回前台重试。'),3000);try{window.Rover.voiceText($('voiceText').value);}catch(error){status('文字桥接调用失败，请重试。');}}else status('当前预览没有 Android 文字理解桥接服务。');});
window.voiceStatus=status;

const modeNames={go:'前往',follow:'跟随',find:'寻找',select:'选择',selectPreview:'选择并预览',preview:'预览',start:'开始',stop:'停车',cancel:'取消',status:'查询状态'};
const colorNames={red:'红色',orange:'橙色',yellow:'黄色',green:'绿色',cyan:'青色',blue:'蓝色',purple:'紫色',pink:'粉色',brown:'棕色',white:'白色',gray:'灰色',black:'黑色',mixed:'多色'};const sizeNames={small:'较小',medium:'中等',large:'较大'};
const posNames={left:'左边',right:'右边',center:'前方/中间',near:'近处',far:'远处'};
function slots(i){const mask=Number(i&&i.mentioned)||0,parts=[];if(mask&1)parts.push('动作：'+(modeNames[i.action]||i.action));if(mask&2)parts.push('物体：'+(window.objectLabelText?window.objectLabelText(i.category):i.category));if(mask&4)parts.push('方位：'+(posNames[i.position]||i.position));if(mask&8)parts.push('序号：第'+i.ordinal+'个');if(mask&16)parts.push('指代：当前所选/上下文目标');if(i&&i.color)parts.push('颜色：'+(colorNames[i.color]||i.color));if(i&&i.size)parts.push('大小：'+(sizeNames[i.size]||i.size));return parts.join(' · ');}
function outcome(t){$('voiceOutcome').textContent='提交 / 执行：'+t;}
function state(){try{const value=window.Rover&&window.Rover.semanticState?JSON.parse(window.Rover.semanticState()):{};return value&&typeof value==='object'&&!Array.isArray(value)?value:{};}catch(e){return {};}}
window.semanticOwns=()=>state().client_owned===true;
function report(){const s=state(),i=s.spoken||{},text=slots(i);window.semanticView={...s,received_ms:Date.now()};if($('semanticSlots'))$('semanticSlots').textContent=text||'尚无已提交的语音条件';if($('semanticCard'))$('semanticCard').hidden=!s.desired_task_id;
 const age=Number(s.client_age_ms),same=String(s.task_id||'')===String(s.desired_task_id||''),fresh=same&&age>=0&&age<=1500;
 const stageNames={SEARCH:'持续寻找',SEARCH_TURN:'持续找回',SELECTING:'等待选中确认',REACQUIRE:'找回原目标',TRACKING:'同一物体已锁定',FOUND:'已找到，未请求运动',WAIT_LINK:'等待原会话心跳',WAIT_IMAGE:'等待新图像',WAIT_IDENTITY:'跟踪身份已改变',PREVIEW:'仅预览',ARRIVED:'已到达',CANCELLED:'已取消',WAIT_SENSORS:'等待新传感器',WAIT_FEEDBACK:'等待底盘反馈',WAIT_RECOVERY:'等待独立新扫描'};
 const status=fresh?(stageNames[s.stage]||s.stage)+'：'+(s.message||'等待RK具体原因'):(s.client_message||'等待RK回报');
 const detail=fresh?'任务 #'+s.task_id+' · 目标 #'+(s.track_id||'未锁定')+' · 选中尝试 '+s.selection_attempts+' · 请求 '+Number(s.requested_v||0).toFixed(2)+' m/s / '+Number(s.requested_w||0).toFixed(2)+' rad/s · 保护后 '+Number(s.protected_v||0).toFixed(2)+' m/s · '+(s.drive_enabled?'驾驶能力已启用':'预览能力'):'板端状态年龄 '+(age>=0?age+' ms':'未知')+'；未把旧回报冒充新状态';
 if($('semanticStatus'))$('semanticStatus').textContent=status;if($('semanticDetail'))$('semanticDetail').textContent=detail;$('voiceBoardStatus').textContent='板端实际回报：'+status+'；'+detail;
}
window.voiceFinal=(text,i)=>{clearTimers();render('result','最终文字已理解；新任务交给RK持续执行，查询和再次录音不会取消原任务。');$('voicePartial').textContent='临时文字：已结束';$('voiceTranscript').textContent='最终原文：'+text;$('voiceText').value=text;
 if(!i){$('voiceIntent').textContent='本句话未接受';outcome('未执行：未能理解单条明确命令；原任务保留。支持类别、颜色和大小组合；没有把无法理解的条件偷偷丢掉。');return;}
 $('voiceIntent').textContent=slots(i)||'按当前上下文理解；未添加未提及的语音条件';
 if(['stop','cancel'].includes(i.action)){if(window.Rover&&window.Rover.semanticCancel)window.Rover.semanticCancel();$('stop').dispatchEvent(new Event('pointerdown',{bubbles:true,cancelable:true}));outcome('已明确停车/取消；等待真实回报');return;}
 if(i.action==='status'){report();outcome('查询当前RK任务，没有重建或取消任务');return;}
 if(i.action==='start'&&window.semanticOwns()){window.Rover.semanticStart();outcome('当前任务明确开始请求已发送，等待RK接受');return;}
 if(i.action==='start'&&window.ACGObjectTaskActions&&$('objectDialog').open){outcome(window.ACGObjectTaskActions.start()?'已提交V8明确开始':'没有可开始的V8任务');return;}if(i.action==='start'){outcome(window.ACGNavActions&&window.ACGNavActions.start()?'已发送当前预览的明确开始':'未执行：没有可开始的当前任务；先选择并预览');return;}
 if(i.action==='preview'){if(window.ACGObjectTaskActions){outcome(window.ACGObjectTaskActions.prepare()?'V8预览已提交':'先选择当前物体再预览');return;}if(window.Rover&&window.Rover.objectReadScene)window.Rover.objectReadScene();if(window.Rover&&window.Rover.objectReadMap)window.Rover.objectReadMap();outcome(window.previewSelectedObject&&window.previewSelectedObject()?'已请求当前物体预览':'尚无可预览的已选物体');return;}
 if(!window.Rover||!window.Rover.semanticSubmit){outcome('需要配套V8语音配套版 Android桥接及RK程序');return;}
 i.distance_m=window.objectVoicePreferences?window.objectVoicePreferences(i.action==='follow'?'follow':'static'):.2;
 if(window.objectLaunchCancel)window.objectLaunchCancel("voice_takeover");window.Rover.semanticSubmit(JSON.stringify(i));outcome('已提交持续任务；没有首个目标时RK保留任务等待；锁定后由V8持续寻找，不要求手机窗口一直打开');report();
};
window.voiceTasksInvalidate=()=>{};
if($('semanticStop'))$('semanticStop').addEventListener('click',()=>{if(window.Rover&&window.Rover.semanticCancel)window.Rover.semanticCancel();});
setInterval(()=>{report();if($('voiceDialog').open){refreshCandidates();if(window.Rover&&window.Rover.objectReadState)window.Rover.objectReadState();const c=window.objectVoiceContext&&window.objectVoiceContext();$('voiceCandidates').textContent=c&&c.age<=800&&c.items.length?'当前可见候选：'+c.items.map((o,i)=>(i+1)+'. '+(window.objectLabelText?window.objectLabelText(o.label):o.label)+' #'+o.track_id).join('；'):'等待新鲜候选；RK任务独立持续执行';}},450);
})();
