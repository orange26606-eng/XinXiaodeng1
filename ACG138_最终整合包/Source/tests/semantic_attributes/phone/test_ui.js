const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const root=process.argv[2],voice=fs.readFileSync(path.join(root,'android/app/src/main/assets/voice.js'),'utf8'),objects=fs.readFileSync(path.join(root,'android/app/src/main/assets/objects.js'),'utf8');
let calls=[],els={};const el=id=>els[id]||(els[id]={id,hidden:false,dataset:{},textContent:'',value:'',checked:false,events:{},addEventListener(e,f){this.events[e]=f;},dispatchEvent(e){calls.push(['button',id,e.type]);},showModal(){this.open=true;},close(){this.open=false;}});
const win={Rover:{semanticState:()=>'{"client_owned":false}',semanticSubmit:s=>calls.push(['submit',JSON.parse(s)]),semanticCancel:()=>calls.push(['stop']),voiceCancel(){}},objectLaunchCancel(){},objectVoicePreferences:()=>.2,objectLabelText:s=>s};
vm.runInNewContext(voice,{window:win,document:{getElementById:el},setInterval(){},setTimeout(){return 1;},clearTimeout(){},console,Date,JSON,Math,Event:class{constructor(type){this.type=type;}}});
win.voiceFinal('找到红色大箱子',{action:'go',category:'box',color:'red',size:'large',position:'',ordinal:0,reference:false,mentioned:99});
assert.equal(calls[0][0],'submit');assert.equal(calls[0][1].color,'red');assert.equal(calls[0][1].size,'large');assert(el('voiceIntent').textContent.includes('红色')&&el('voiceIntent').textContent.includes('较大'));
win.voiceFinal('停车',{action:'stop'});assert(calls.some(x=>x[0]==='stop'));assert(calls.some(x=>x[0]==='button'&&x[1]==='stop'));
const helper=objects.match(/function objectDescription\(o\)\{[^\n]+\}/)[0];const context={objectColors:{red:'红色',blue:'蓝色'},objectLabel:s=>({bottle:'水瓶',box:'箱子'}[s]||s)};vm.createContext(context);vm.runInContext(helper,context);
assert.equal(context.objectDescription({label:'box',attributes:{color_valid:true,color:'red',size:'large',width_m:.8}}),'红色箱子');assert.equal(context.objectDescription({label:'bottle',attributes:{color_valid:false,color:'red'}}),'水瓶');
const draw=objects.match(/ctx\.fillText\([^;]+;/)[0];assert(draw.includes('objectDescription(o)')&&!draw.includes('track_id')&&!draw.includes('size'));
assert(objects.includes("title.textContent=ordinal+'. '+objectDescription(o)"));
const main=fs.readFileSync(path.join(root,'android/app/src/main/java/cn/acg138/rover/MainActivity.java'),'utf8'),svc=fs.readFileSync(path.join(root,'android/app/src/main/java/cn/acg138/rover/SemanticTaskService.java'),'utf8');
assert(main.includes('JSONObject.quote(intent.color)')&&main.includes('JSONObject.quote(intent.size)'));assert(main.includes('"window.objectReceiveState",262144,objectStatePending'));
assert(svc.includes('.put("color",i.optString("color",""))')&&svc.includes('.put("size",i.optString("size",""))'));assert(svc.includes('NetworkTiming.HEARTBEAT_MS')&&svc.includes('NetworkTiming.TASK_MS'));
console.log('PASS actual voice.js semantic dispatch+slots+STOP, actual canvas color/name helper and labels, Java bridge/payload attribute preservation,256KiB object cap');

// 执行完整实际任务绘制器，包含覆盖距离文字的诊断显示。
let taskEls={};const taskEl=id=>taskEls[id]||(taskEls[id]={id,value:'',open:false,textContent:'',hidden:false,disabled:false,setAttribute(){},addEventListener(){}});
const taskWindow={localStorage:{getItem(){return null;},setItem(){}},Rover:{},objectFlowDiagnostics:()=>({distance:'旧记忆距离0.20m',motion:'诊断运动',blocker:'诊断原因'})};
vm.runInNewContext(fs.readFileSync(path.join(root,'android/app/src/main/assets/object-dual.js'),'utf8'),{window:taskWindow,document:{getElementById:taskEl},setInterval(){},console,Date,JSON,Math,Number,String});
const task={version:1,mode:'static',epoch:'4',track_id:2,state:'VERIFY_ARRIVAL',active:true,enabled:true,prepared:true,drive_capable:true,near_hold:true,near_metric:false,near_reason:'TARGET_EXPANSION_VISUAL_NEAR_HOLD',message_zh:'视觉目标已经很近',standoff_m:.2,range_m:.2};
taskWindow.objectReceiveTask(task);assert(taskEl('objectTaskStatus').textContent.includes('目标已近，保持停车复核'));assert(taskEl('objectTaskReason').textContent.includes('视觉近距线索'));assert(taskEl('objectTaskEvidence').textContent.includes('未验证')||taskEl('objectTaskEvidence').textContent.includes('没有验证'));assert(taskEl('objectTaskDistance').textContent.includes('当前间距未可靠测得')&&!taskEl('objectTaskDistance').textContent.includes('旧记忆'));
taskWindow.objectReceiveTask({...task,state:'ARRIVED',active:false});assert(taskEl('objectTaskStatus').textContent.includes('视觉近距停靠完成'));assert(taskEl('objectTaskDistance').textContent.includes('不是实测到达精度'));assert(!taskEl('objectTaskDistance').textContent.includes('精确到达'));
taskWindow.objectReceiveTask({...task,state:'ARRIVED',active:false,near_metric:true,near_reason:'TARGET_SURFACE_NEAR_HOLD'});assert(taskEl('objectTaskStatus').textContent.includes('目标表面距离确认到达'));assert(taskEl('objectTaskEvidence').textContent.includes('目标表面距离支持停靠'));
console.log('PASS actual whole object-dual renderer: VERIFY_ARRIVAL Chinese, near_hold/coarse/metric evidence, coarse ARRIVED no false precision, diagnostics cannot restore old metric claim');

taskWindow.objectReceiveTask({...task,state:'OBSERVING_CANDIDATE',near_hold:false,near_metric:false,message_zh:'物理大小尚未可靠测得，正在对准候选'});assert(taskEl('objectTaskStatus').textContent.includes('正在对准候选，观察大小'));assert(!taskEl('objectTaskStatus').textContent.includes('OBSERVING_CANDIDATE'));
taskWindow.objectReceiveTask({...task,state:'FUTURE_UNLISTED_STATE',near_hold:false,near_metric:false,message_zh:'正在使用新观测调整方向'});assert(taskEl('objectTaskStatus').textContent.includes('正在使用新观测调整方向')&&!taskEl('objectTaskStatus').textContent.includes('FUTURE_UNLISTED_STATE'));
console.log('PASS actual renderer OBSERVING_CANDIDATE Chinese and unknown-state message_zh fallback; no dimensions added');
