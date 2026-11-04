'use strict';
// 分别处理传输失败、预览图缺失与目标有效性。
window.ACGAssociation={
 valid(a){return a&&a.version===1&&typeof a.epoch==='string'&&typeof a.frame_id==='string'&&typeof a.frame_time_us==='string'&&/^[0-9]{1,20}$/.test(a.epoch)&&/^[0-9]{1,20}$/.test(a.frame_id)&&/^[0-9]{1,20}$/.test(a.frame_time_us)&&Array.isArray(a.candidates)&&a.candidates.length<=16&&a.candidates.every(c=>Number.isInteger(c.id)&&Number.isFinite(c.range_m)&&Number.isFinite(c.score)&&Array.isArray(c.points)&&c.points.length<=64&&c.points.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)&&p[0]>=0&&p[0]<960&&p[1]>=0&&p[1]<1080));},
 paired(s){const a=s&&s.association;return this.valid(a)&&s.version===1&&s.epoch===a.epoch&&s.frame_id===a.frame_id&&s.frame_time_us===a.frame_time_us&&Number.isInteger(s.reference_eye)&&s.reference_eye>=0&&s.reference_eye<=1&&s.reference_eye===a.reference_eye&&s.selected_track_id===a.track_id&&s.snapshot_id===s.epoch+'-'+s.frame_id&&s.width===960&&s.height===1080&&Array.isArray(s.objects)&&s.objects.some(o=>o.track_id===a.track_id&&o.observed===true&&o.confirmed===true)&&s.stale!==true&&Number.isFinite(s.age_ms)&&s.age_ms<=500&&typeof s.image_data_url==='string'&&s.image_data_url.startsWith('data:image/png;base64,')&&s.image_data_url.length<3*1024*1024;},
 reason(code){const m={CONFIG_DISABLED:'关联功能尚未启用；原雷达建图仍可正常运行',INVALID_OR_UNVERIFIED_CALIBRATION:'单眼内参、相机安装参数或雷达高度尚未通过核对',LIDAR_MOUNT_NOT_VERIFIED:'雷达安装位置尚未核对',NO_SELECTED_TRACK:'尚未选择目标，请更新抓拍并点击检测框',SELECTION_OR_EPOCH_CHANGED:'目标或采集会话改变，正在重新确认',SELECTED_TRACK_NOT_FRESH_OBSERVED_CONFIRMED:'目标当前没有新鲜、已确认的观测，请检查跟踪或重新选择',OBJECT_FRAME_UNAVAILABLE:'等待摄像头检测数据',ORIGINAL_FRAME_STALE:'摄像头原始观测已过期',ORIGINAL_ASSOCIATION_EXPIRED:'关联所用原始观测已过期',ORIGINAL_OBSERVATION_EXPIRED:'关联所用原始观测已过期',BBOX_SCORE_OR_REFERENCE_EYE_INVALID:'检测框、识别分数或参考眼不符合要求',NO_FRESH_SCAN_WITHIN_PAIR_WINDOW:'没有与图像时间匹配的新鲜雷达扫描',HISTORICAL_POSE_OR_TIME_UNCERTAINTY_INVALID:'对应时刻的历史定位或时间误差未满足条件',LOCALIZATION_MAP_CONTEXT_UNAVAILABLE:'等待可用地图定位；请查看地图状态',EXACT_SAME_FRAME_SNAPSHOT_REQUIRED:'等待与检测结果严格同帧的叠加图片',EXACT_PAIRED_SNAPSHOT_UNAVAILABLE:'当前没有可用的同帧叠加图片',NO_SUPPORTED_PROJECTED_SURFACE:'目标区域内没有足够雷达回波，或目标不在扫描高度',COMPETING_SURFACE_CLUSTERS:'存在多个可能属于目标的点簇，距离暂未确认',SURFACE_CONTINUES_OUTSIDE_BBOX:'回波延伸到目标框外，可能来自背景',OVERLAP_OR_INSUFFICIENT_OBJECT_SUPPORT:'目标重叠或前景证据不足，暂不能确认距离',FOREGROUND_FLANK_SUPPORT_REQUIRED:'缺少目标两侧更远的背景回波，前景身份尚未确认',DISTINCT_IMAGE_SCAN_CONFIRMATION_PENDING:'正在等待不同图像和扫描的连续确认',DISTINCT_IMAGE_SCAN_SURFACE_SUPPORT:'目标表面位置已确认',OBJECT_ASSOCIATION_INVALID_OR_NO_EXACT_PAIR:'尚无同帧叠加图，不代表雷达断开',OBJECT_ASSOCIATION_SNAPSHOT_UNAVAILABLE:'关联图片暂不可用，先检查功能是否启用及目标是否选中',OBJECT_ASSOCIATION_PUBLICATION_STALE:'关联服务没有及时更新，请检查map进程和日志'};return m[code]||code||'等待关联状态';},
 describe(a){if(!a)return '等待关联状态';const labels={DISABLED:'未启用',CALIBRATION_REQUIRED:'待标定／安装核对',WAIT_TARGET:'等待选择目标',CANDIDATE:'正在确认候选',AMBIGUOUS:'存在歧义，距离未确认',CONFIRMED:'目标表面位置已确认',LOST:'目标暂时丢失',STALE:'原始观测已过期',UNAVAILABLE:'条件暂未满足'};return (labels[a.status]||a.status)+'：'+this.reason(a.reason);}
};
(()=>{const $=id=>document.getElementById(id),canvas=$('associationCanvas'),ctx=canvas.getContext('2d');let state=null,at=0,published='',pair=null,image=null,token=0,lastPairRequest=0,stateIssue='',pairIssue='';
function fresh(){return state&&state.stale!==true&&Number.isFinite(state.age_ms)&&state.age_ms>=0&&state.age_ms+Date.now()-at<=500;}
function confirmed(){return fresh()&&state.status==='CONFIRMED'&&state.target&&state.target.valid===true&&Number.isFinite(state.target.map_x_m)&&Number.isFinite(state.target.map_y_m);}
function eligiblePair(){return fresh()&&['CANDIDATE','AMBIGUOUS','CONFIRMED'].includes(state.status)&&state.track_id>0&&state.frame_id!=='0';}
window.currentObjectAssociation=()=>confirmed()?state:null;
function update(){
 const a=window.ACGAssociation,live=fresh(),setup=state&&['DISABLED','CALIBRATION_REQUIRED','WAIT_TARGET'].includes(state.status);
 let text=state?a.describe(state):'等待关联状态';
 if(state&&!setup&&!live)text+='（历史／过期，不用于导航）';
 if(state)text+=' ['+state.status+' / '+state.reason+']';
 if(stateIssue)text+='；状态读取：'+stateIssue;
 if(confirmed()&&Number.isFinite(state.target.range_m))text+='；雷达确认表面距离 '+state.target.range_m.toFixed(2)+' m · 地图 X '+state.target.map_x_m.toFixed(2)+' / Y '+state.target.map_y_m.toFixed(2)+' m';
 $('objectAssociationStatus').textContent=text;
 let candidates=state&&state.candidates.length?state.candidates.map(c=>'候选 '+c.id+'：'+c.range_m.toFixed(2)+' m / 支持分数 '+(c.score*100).toFixed(0)+'/100'+(c.confirmed?' 已确认':' 未确认')).join('；'):'尚无候选距离';
 if(pairIssue)candidates+='；叠加图片：'+pairIssue;
 $('associationCandidates').textContent=candidates;$('previewObjectNavigation').disabled=!confirmed();draw();
}
function draw(){ctx.fillStyle='#10171a';ctx.fillRect(0,0,canvas.width,canvas.height);if(!pair||!image)return;const r=window.ACGObjectGeometry.contain(960,1080,canvas.width,canvas.height);ctx.drawImage(image,r.x,r.y,r.width,r.height);const historical=Date.now()-pair.received+pair.age_ms>500;for(const c of pair.association.candidates){ctx.fillStyle=historical?'#9ba8ad':c.confirmed?'#3ee593':'#ffd24b';for(const p of c.points){ctx.beginPath();ctx.arc(r.x+p[0]*r.scale,r.y+p[1]*r.scale,3,0,Math.PI*2);ctx.fill();}}}
window.objectReceiveAssociation=a=>{if(!window.ACGAssociation.valid(a)){state=null;stateIssue='关联状态格式不兼容';update();return;}if(a.published_us!==published){published=a.published_us;at=Date.now();}state=a;stateIssue='';if(['DISABLED','CALIBRATION_REQUIRED','WAIT_TARGET'].includes(a.status)){pair=null;image=null;pairIssue='';token++;}update();};
window.objectReceiveAssociationSnapshot=s=>{
 if(!window.ACGAssociation.paired(s)){const reason=s&&s.association&&s.association.reason;window.objectAssociationError(reason?window.ACGAssociation.reason(reason):'当前叠加图未满足同帧或新鲜度条件，等待下一张');return;}
 const t=++token,im=new Image();im.onload=()=>{if(t!==token)return;if(im.naturalWidth!==960||im.naturalHeight!==1080)return window.objectAssociationError('同帧图片尺寸错误');pair={...s,received:Date.now()};image=im;pairIssue='';update();};im.onerror=()=>window.objectAssociationError('同帧图片解码失败');im.src=s.image_data_url;
};
window.objectAssociationError=message=>{pair=null;image=null;token++;pairIssue=message;update();};
function explainError(message){const code=String(message).match(/OBJECT_ASSOCIATION_[A-Z_]+/);return code?window.ACGAssociation.reason(code[0])+' ['+code[0]+']':String(message);}
const oldError=window.objectReceiveError;window.objectReceiveError=(path,message)=>{
 if(path.includes('association-snapshot')){window.objectAssociationError(explainError(message));return;}
 if(path.includes('association')){stateIssue=explainError(message);state=null;update();return;}
 oldError(path,message);
};
function request(){if(!$('objectDialog').open&&!$('navigationMapDialog').open&&!$('voiceDialog').open)return;if(window.Rover&&window.Rover.objectReadAssociation){window.Rover.objectReadAssociation();if($('objectDialog').open&&eligiblePair()&&Date.now()-lastPairRequest>=1500){lastPairRequest=Date.now();window.Rover.objectAssociationSnapshot();}}update();}
$('openObjects').addEventListener('click',request);$('refreshObjects').addEventListener('click',()=>{if(!eligiblePair()){pairIssue='请先启用并核对标定、选中目标，等待回波候选；原始抓拍仍可使用';update();return;}if(window.Rover&&window.Rover.objectAssociationSnapshot){lastPairRequest=Date.now();window.Rover.objectAssociationSnapshot();}});setInterval(request,350);
function preview(){if(!confirmed())return false;const a=state,d=Number($('objectStandoff').value);if(!Number.isFinite(d)||d<.2||d>3){$('objectAssociationStatus').textContent='停车距离必须为0.20～3.00米';return false;}if(window.ACGNavActions)return window.ACGNavActions.previewObject(a,d);return false;}
window.previewSelectedObject=preview;$('previewObjectNavigation').addEventListener('click',preview);$('objectMapRoute').addEventListener('click',()=>$('openNavigationMap').click());
const original=window.drawObjectMapMarkers;window.drawObjectMapMarkers=(c,v,w,h,mapId)=>{original(c,v,w,h,mapId);if(!state||state.map_id!==String(mapId)||!state.target||!state.target.valid||!Number.isFinite(state.target.map_x_m)||!Number.isFinite(state.target.map_y_m))return;const x=(.5+(state.target.map_x_m-v.x)/v.span)*w,y=(.5-(state.target.map_y_m-v.y)/v.span)*h;c.strokeStyle=confirmed()?'#3ee593':'#9ba8ad';c.beginPath();c.arc(x,y,10,0,Math.PI*2);c.stroke();c.fillStyle=c.strokeStyle;c.fillText('雷达实物 #'+state.track_id+(confirmed()?' 已确认':' 历史'),x+12,y);};
})();
