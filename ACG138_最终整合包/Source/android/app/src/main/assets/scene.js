"use strict";
/* 只读绘制RK生成的证据，不调用电机或目标接口。 */
(() => {
 const $=id=>document.getElementById(id);let scene=null,received=0,error='',range=12;
 const finite=Number.isFinite,xy=o=>o&&o.valid===true&&finite(o.map_x_m)&&finite(o.map_y_m);
 const labels={HELD:'保留控制器路线，当前停车或复核',ACTIVE:'控制器当前执行路线',READY:'路线可预览',PATH_READY:'路线可预览',OK:'路线可预览',NO_PATH:'尚无可通行路线',TARGET_OUTSIDE_MAP:'目标超出当前地图',DOCK_OUTSIDE_MAP:'停靠点超出当前地图',WAIT_TARGET:'等待目标位置',WAIT_MAP:'等待地图',WAIT_POSE:'等待定位',NO_TARGET:'尚未选择目标',UNAVAILABLE:'路线暂不可用',TARGET_DIRECTION_ONLY:'只有方向，尚无实测距离',SEARCH_LIMIT:'本轮规划达到计算上限',GOAL_OCCUPIED:'停靠点被占用',GOAL_UNKNOWN:'停靠点尚未观测'};
 function age(){return scene?Math.max(0,Number(scene.age_ms)||0)+Math.max(0,Number(scene.client_delay_ms)||0)+Math.max(0,Date.now()-received):Infinity;}
 function reason(code){return labels[code]||(window.objectReasonText?window.objectReasonText(code):code)||'等待数据';}
 function status(){
  let text=scene?(scene.message_zh||reason(scene.reason)):'等待真实雷达、视觉和定位输出';
  if(scene){text+='\n来源：'+(scene.source||'暂无')+' · 安装：'+(String(scene.mount_quality).startsWith('VERIFIED')?'已核对':'近似参数')+' · 数据年龄 '+Math.round(age())+'ms';
   if(xy(scene.target))text+='\n目标 X '+scene.target.map_x_m.toFixed(2)+' / Y '+scene.target.map_y_m.toFixed(2)+'m'+(finite(scene.target.range_m)?' · 位置估计距离（车体中心至表面） '+scene.target.range_m.toFixed(2)+'m':'')+' · 误差约 '+(finite(scene.target.error_m)?scene.target.error_m.toFixed(2):'未知')+'m'+(scene.target.historical||age()>8000?' · 历史位置':'');
   if(xy(scene.dock))text+='\n停靠点 X '+scene.dock.map_x_m.toFixed(2)+' / Y '+scene.dock.map_y_m.toFixed(2)+'m · 车头预留 '+(Number(scene.dock.standoff_m)||0).toFixed(2)+'m';
   const f=scene.fusion_diagnostics;if(f&&typeof f==='object'){
    text+='\n同目标雷达跟踪：'+(f.lidar_track_valid===true?'有效':f.lidar_track_valid===false?'尚未有效':'未上报')+' · 新扫描更新 '+(Number.isInteger(f.lidar_updates)?f.lidar_updates:'未知')+' · 候选 '+(Number.isInteger(f.lidar_candidates)?f.lidar_candidates:'未知');
    if(f.stereo_prior_only===true)text+='\n双目来源较旧，仅作粗距先验；不否定新雷达观测';
    if(f.stereo_conflict===true)text+='\n雷达与双目距离存在冲突，采用板端具体融合判断；不把旧深度当新距离';
   }
   if(window.objectFollowSceneDiagnostics){const follow=window.objectFollowSceneDiagnostics(scene);if(follow)text+='\n'+follow;}
   if(window.objectRadarAssociationDiagnostics){const radar=window.objectRadarAssociationDiagnostics(scene,Math.max(0,Date.now()-received));if(radar)text+='\n'+radar;}
   if(window.objectObstacleSensitivityDiagnostics){const obstacles=window.objectObstacleSensitivityDiagnostics(scene,Math.max(0,Date.now()-received));if(obstacles)text+='\n'+obstacles;}
   const p=scene.preview||{};text+='\n路线来源：'+({EXECUTING_REFERENCE_ROUTE:p.status==='HELD'?'保留控制器参考路线，当前停车或复核':'控制器实际执行参考路线',VALIDATED_ACTION:'已通过终检的局部动作路径',READONLY_PREVIEW:'只读路线预览，尚未执行',NONE:'暂无路线'}[p.kind]||p.kind||'旧接口未区分预览/执行')+' · 显示坐标 '+(p.frame||'未上报')+(p.source_frame?' · 原控制坐标 '+p.source_frame:'');text+='\n路线：'+(age()>8000?'历史路线已隐藏，等待新预览':reason(p.status))+(p.reason?' · '+reason(p.reason):'')+' · 实际路径点 '+(Array.isArray(p.path)?p.path.length:0);
  }
  if(error)text+='\n读取异常：'+error+'；保留最后结果';if(age()>8000&&scene)text+='\n历史结果，仅供查看，不代表当前可执行';
  for(const id of ['objectSceneStatus','navigationObjectSceneStatus'])if($(id))$(id).textContent=text;drawLocal();
 }
 function validPair(p){return Array.isArray(p)&&p.length>=2&&finite(p[0])&&finite(p[1]);}
 window.objectReceiveScene=data=>{
  if(!data||data.version!==1||typeof data.map_id!=='string'||typeof data.published_us!=='string'||!/^\d{1,20}$/.test(data.published_us)||!Array.isArray(data.preview?.path)||data.preview.path.length>4096||!data.preview.path.every(validPair)||!Array.isArray(data.front_cloud?.points)||data.front_cloud.points.length>8192||!data.front_cloud.points.every(p=>validPair(p)&&p.length>=3&&finite(p[2])&&p[2]>=0)){window.objectSceneError('目标场景格式不兼容，未绘制异常坐标');return;}
  if(scene&&scene.map_id===data.map_id&&scene.published_us){const a=data.published_us.replace(/^0+(?=\d)/,''),b=scene.published_us.replace(/^0+(?=\d)/,'');if(a.length<b.length||a.length===b.length&&a<b)return;}
  scene=data;received=Date.now();error='';status();if(window.navRedrawObjectMarkers)window.navRedrawObjectMarkers();
 };
 window.objectSceneError=message=>{error=message||'场景读取暂不可用';status();};
 window.objectSceneTick=status;
 function drawLocal(){const canvas=$('objectFrontCloud');if(!canvas)return;const c=canvas.getContext('2d');if(!c)return;const w=canvas.width,h=canvas.height,s=Math.min((w-30)/(2*range),(h-42)/(2*range)),ox=w/2,oy=h/2+8;c.fillStyle='#102029';c.fillRect(0,0,w,h);c.strokeStyle='#35515e';c.lineWidth=1;
  for(let d=2;d<=range;d+=2){c.beginPath();c.arc(ox,oy,d*s,0,Math.PI*2);c.stroke();}c.fillStyle='#85d5cd';c.fillRect(ox-4,oy-5,8,10);
  let count=0;if(scene)for(const p of scene.front_cloud.points){const d=Math.hypot(p[0],p[1]);if(d>range||p[2]+age()>800)continue;c.globalAlpha=Math.max(.25,1-(p[2]+age())/1000);c.fillStyle='#8fcfff';c.fillRect(ox-p[1]*s-1.5,oy-p[0]*s-1.5,3,3);count++;}c.globalAlpha=1;
  c.fillStyle='#d3e7ef';c.font='12px sans-serif';c.fillText('全向真实回波 · '+range+'米 · '+count+'点',8,15);c.fillText('前',ox-6,30);c.fillText('后',ox-6,h-5);c.fillText('左',8,oy+4);c.fillText('右',w-22,oy+4);if($('objectFrontCloudStatus'))$('objectFrontCloudStatus').textContent='全向360°；'+range+'米显示视野 · '+count+'个真实点；原始回波＋≤800ms补偿历史，不补造回波；不因目标排墙删障碍。';
 }
 window.drawObjectScene=(c,v,w,h,mapId,pose)=>{
  if(!scene||scene.map_id!==String(mapId))return;status();const historical=age()>8000;const pt=(x,y)=>[(.5+(x-v.x)/v.span)*w,(.5-(y-v.y)/v.span)*h],target=scene.target||{},dock=scene.dock||{};
  const cloudPose=scene.front_cloud.pose_valid===true&&Array.isArray(scene.front_cloud.pose)?scene.front_cloud.pose:pose;
  if(Array.isArray(cloudPose)&&cloudPose.length>=3&&cloudPose.every(finite)){
   const ca=Math.cos(cloudPose[2]),sa=Math.sin(cloudPose[2]);c.fillStyle='#84ccff';for(const p of scene.front_cloud.points){if(Math.hypot(p[0],p[1])>range||p[2]+age()>800)continue;const q=pt(cloudPose[0]+ca*p[0]-sa*p[1],cloudPose[1]+sa*p[0]+ca*p[1]);c.globalAlpha=Math.max(.2,1-(p[2]+age())/1000);c.fillRect(q[0]-1.5,q[1]-1.5,3,3);}c.globalAlpha=1;
  }
  if(xy(target)){const q=pt(target.map_x_m,target.map_y_m);c.strokeStyle=target.confirmed&&!target.historical&&!historical?'#48e59b':'#ffd15c';c.fillStyle=c.strokeStyle;c.lineWidth=2;if(c.setLineDash)c.setLineDash(target.historical||historical?[6,4]:[]);c.beginPath();c.arc(q[0],q[1],Math.max(7,(finite(target.error_m)?target.error_m:0)/v.span*w),0,Math.PI*2);c.stroke();c.fillRect(q[0]-3,q[1]-3,6,6);if(c.fillText){c.font='12px sans-serif';c.fillText('真实目标 #'+scene.track_id+(target.historical||historical?' 历史 '+Math.round(age())+'ms':''),q[0]+9,q[1]-8);}if(c.setLineDash)c.setLineDash([]);}
  if(!historical&&!xy(target)&&scene.bearing_valid===true&&finite(scene.bearing_rad)&&Array.isArray(pose)&&pose.length===3&&pose.every(finite)){const origin=pt(pose[0],pose[1]),length=Math.min(range,v.span*.35),heading=pose[2]+scene.bearing_rad;c.strokeStyle='#ffd15c';c.lineWidth=2;if(c.setLineDash)c.setLineDash([6,4]);for(const delta of [-.12,.12]){const end=pt(pose[0]+length*Math.cos(heading+delta),pose[1]+length*Math.sin(heading+delta));c.beginPath();c.moveTo(origin[0],origin[1]);c.lineTo(end[0],end[1]);c.stroke();}if(c.setLineDash)c.setLineDash([]);if(c.fillText){c.fillStyle='#ffd15c';c.fillText('仅目标方向 · 距离未知',origin[0]+10,origin[1]-15);}}
  if(xy(dock)){const q=pt(dock.map_x_m,dock.map_y_m);c.strokeStyle='#ef92ff';c.lineWidth=2;c.strokeRect(q[0]-6,q[1]-6,12,12);if(finite(dock.yaw_rad)){const ex=q[0]+17*Math.cos(dock.yaw_rad),ey=q[1]-17*Math.sin(dock.yaw_rad);c.beginPath();c.moveTo(q[0],q[1]);c.lineTo(ex,ey);c.stroke();}if(c.fillText){c.fillStyle='#ef92ff';c.fillText(historical?'历史停靠点':'停靠点',q[0]+9,q[1]+15);}}
  const path=!historical&&['READY','PATH_READY'].includes(scene.preview.status)?scene.preview.path:[];if(path.length>1){c.strokeStyle='#ed93ff';c.lineWidth=3;if(c.setLineDash)c.setLineDash([5,3]);c.beginPath();path.forEach((p,i)=>{const q=pt(p[0],p[1]);i?c.lineTo(q[0],q[1]):c.moveTo(q[0],q[1]);});c.stroke();if(c.setLineDash)c.setLineDash([]);}
 };
 const button=$('navigationFrontRange');if(button)button.addEventListener('click',()=>{range=range===8?12:8;button.textContent='全向'+range+'米';status();if(window.navRedrawObjectMarkers)window.navRedrawObjectMarkers();});
 window.objectSceneRange=()=>range;
})();
