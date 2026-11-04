'use strict';
// 仅显示定位，不访问Rover桥接、控制端点、模式或STOP接口。
(() => {
 const el=id=>document.getElementById(id);
 const canvas=el('odomCanvas'),ctx=canvas.getContext('2d');
 el('odomToggle').addEventListener('click',()=>{
  const details=el('odomDetails');details.hidden=!details.hidden;
  el('odomToggle').textContent=details.hidden?'展开轨迹':'收起轨迹';
  if(window.lastOdometry)window.renderOdometry(...window.lastOdometry);
 });
 window.renderOdometry=(value,online)=>{
  window.lastOdometry=[value,online];
  const valid=value&&Number.isFinite(value.x)&&Number.isFinite(value.y)&&Number.isFinite(value.heading_deg)&&Number.isFinite(value.distance_m)&&value.quality>0;
  const fresh=online&&valid&&value.age_ms<=500&&value.quality!==4;
  el('odomStatus').textContent=!online?'离线 · 保留历史轨迹':!value?'等待新版定位遥测':fresh?(value.status||'定位估计'):'定位数据过期 / 尚未就绪';
  el('odomStatus').classList.toggle('live',fresh&&value.calibrated===true);
  [['odomX','x',3],['odomY','y',3],['odomHeading','heading_deg',1],['odomDistance','distance_m',3]].forEach(([id,key,digits])=>{el(id).textContent=valid?value[key].toFixed(digits):'—';});
  el('odomNote').textContent=!valid?'初始前进为+X，左侧为+Y。定位不接管电机。':`相对起点 · 会话 ${value.session} · ${fresh?'当前估计':'历史快照'}${value.calibrated?'':' · 轮位/轮径未实车标定'}${value.gaps?' · 存在数据缺口，累计距离不完整':''}`;
  ctx.fillStyle='#111e2c';ctx.fillRect(0,0,800,360);
  const points=(Array.isArray(value?.trace)?value.trace:[]).slice(0,256).filter(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite));
  if(!valid){ctx.fillStyle='#c2d1e0';ctx.font='24px sans-serif';ctx.fillText('等待编码器与 IMU 定位数据',180,180);return;}
  let xs=[0,value.x,...points.map(p=>p[0])],ys=[0,value.y,...points.map(p=>p[1])];
  let xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys);
  const scale=Math.min(700/Math.max(1,xmax-xmin),250/Math.max(1,ymax-ymin));
  const cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  const project=p=>[400+(p[0]-cx)*scale,180-(p[1]-cy)*scale];
  const origin=project([0,0]);ctx.strokeStyle='#50647a';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(30,origin[1]);ctx.lineTo(770,origin[1]);ctx.moveTo(origin[0],30);ctx.lineTo(origin[0],330);ctx.stroke();
  ctx.fillStyle='#a9bdcf';ctx.font='16px sans-serif';ctx.fillText('+X',765,origin[1]-5);ctx.fillText('+Y',origin[0]+5,25);ctx.fillText('O',origin[0]+6,origin[1]+18);
  ctx.strokeStyle=fresh?'#48dbc1':'#8594a3';ctx.lineWidth=3;ctx.beginPath();
  points.forEach((p,i)=>{const q=project(p);if(i)ctx.lineTo(...q);else ctx.moveTo(...q);});ctx.stroke();
  const q=project([value.x,value.y]),theta=value.heading_deg*Math.PI/180;
  ctx.save();ctx.translate(...q);ctx.rotate(-theta);ctx.fillStyle=fresh?'#ffc267':'#8594a3';ctx.beginPath();ctx.moveTo(17,0);ctx.lineTo(-10,-9);ctx.lineTo(-10,9);ctx.closePath();ctx.fill();ctx.restore();
  ctx.fillStyle='#a9bdcf';ctx.fillText(`1 m = ${scale.toFixed(0)} px  · 同比例坐标`,35,345);
 };
 window.renderOdometry(null,false);
})();
