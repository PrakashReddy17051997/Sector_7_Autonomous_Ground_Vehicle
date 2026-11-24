#pragma once
// RouteMapPage - "Route map" card on the web page (loaded by /mission.js): the taught path (log_013,
// IMU + LiDAR map, no GPS), the walls, the Marks, and where the box BELIEVES it is: leg + effort
// fraction -> a point on that leg's part of the path, heading = route yaw, with the live LiDAR scan
// drawn around it. Live walls on the map walls = the box is where it thinks it is.
// Data: /route/map.json once (RouteWebMap.h), /route/live twice per second (Mission).
static const char kRouteMapJs[] PROGMEM = R"rawjs((function(){
const c=document.createElement('div');c.className='card';
c.innerHTML="<div class='sectionTitle'>Route map - taught path, believed position, live LiDAR (no GPS)</div>"
 +"<div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:6px'>"
 +"<button class='btnTiny' id='rmAll'>Whole route</button><button class='btnTiny' id='rmFol'>Follow rover</button>"
 +"<label class='label'><input type='checkbox' id='rmScan' checked> live scan</label></div>"
 +"<canvas id='rmC' style='width:100%;height:420px;background:#0d1117;border-radius:8px;touch-action:none'></canvas>"
 +"<div class='value' id='rmInfo' style='margin-top:6px'>loading map ...</div>"
 +"<div class='label'>Blue = taught path (M1 start, green), grey = walls of the teaching run, white = Marks, "
 +"orange = leg ends, red = believed rover position (leg + effort), yellow = live LiDAR at that position. "
 +"Position is an estimate from the leg and its effort, not a measurement.</div>";
const after=document.querySelector('#msS');const card=after?after.closest('.card'):null;
const w=document.querySelector('.wrap');w.insertBefore(c,card?card.nextSibling:w.children[2]);
const $=id=>document.getElementById(id),cv=$('rmC'),g=cv.getContext('2d');
let M=null,L=null,follow=false,view=null;
function fitAll(){let x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;for(let i=0;i<M.p.length;i+=2){x0=Math.min(x0,M.p[i]);x1=Math.max(x1,M.p[i]);y0=Math.min(y0,M.p[i+1]);y1=Math.max(y1,M.p[i+1]);}
 view={cx:(x0+x1)/20,cy:(y0+y1)/20,span:Math.max(x1-x0,y1-y0)/10+8};}
function pathPt(f){const n=M.p.length/2-1;f=Math.max(0,Math.min(n,f));const i=Math.floor(f),t=f-i,j=Math.min(n,i+1);
 return [(M.p[2*i]+(M.p[2*j]-M.p[2*i])*t)/10,(M.p[2*i+1]+(M.p[2*j+1]-M.p[2*i+1])*t)/10];}
function pose(){if(!L||!L.run||!M)return null;const lg=M.legs[L.leg];if(!lg)return null;
 if(L.lp===2)return {x:L.gx,y:L.gy,th:L.gth*Math.PI/180,lg:lg,loc:true};
 const fr=Math.max(0,Math.min(1.2,L.frac));const xy=pathPt(lg[0]+fr*(lg[1]-lg[0]));
 return {x:xy[0],y:xy[1],th:(L.ry+lg[2])*Math.PI/180,lg:lg};}
function draw(){const W=cv.clientWidth,H=cv.clientHeight;if(cv.width!==W*devicePixelRatio){cv.width=W*devicePixelRatio;cv.height=H*devicePixelRatio;}
 g.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);g.clearRect(0,0,W,H);if(!M)return;
 const P=pose();let v=view;if(follow&&P)v={cx:P.x,cy:P.y,span:16};
 const k=Math.min(W,H)/v.span,X=x=>W/2+(x-v.cx)*k,Y=y=>H/2-(y-v.cy)*k;
 g.fillStyle='#5b6573';for(let i=0;i<M.w.length;i+=2)g.fillRect(X(M.w[i]/10)-1,Y(M.w[i+1]/10)-1,2,2);
 g.strokeStyle='#4ea1ff';g.lineWidth=2;g.beginPath();for(let i=0;i<M.p.length;i+=2){const x=X(M.p[i]/10),y=Y(M.p[i+1]/10);i?g.lineTo(x,y):g.moveTo(x,y);}g.stroke();
 g.fillStyle='#ffb020';M.legs.forEach(l=>{const q=pathPt(l[1]);g.beginPath();g.arc(X(q[0]),Y(q[1]),3.5,0,7);g.fill();});
 g.fillStyle='#e6edf3';g.font='10px sans-serif';M.marks.forEach(m=>{const q=pathPt(m[1]);g.fillText('M'+m[0],X(q[0])+3,Y(q[1])-3);});
 const s0=pathPt(0);g.fillStyle='#3fb950';g.beginPath();g.arc(X(s0[0]),Y(s0[1]),6,0,7);g.fill();
 if(P){if(P.lg){g.strokeStyle='#ff5a5f';g.lineWidth=4;g.beginPath();for(let f=P.lg[0];f<=P.lg[1];f++){const q=pathPt(f);f===P.lg[0]?g.moveTo(X(q[0]),Y(q[1])):g.lineTo(X(q[0]),Y(q[1]));}g.globalAlpha=0.35;g.stroke();g.globalAlpha=1;}
  if($('rmScan').checked&&L.scan){g.fillStyle='#f2cc60';for(let i=0;i<90;i++){const r=L.scan[i]/100;if(r<0.3||r>8)continue;
   const a=P.th-(i*4+2)*Math.PI/180;g.fillRect(X(P.x+r*Math.cos(a))-1.5,Y(P.y+r*Math.sin(a))-1.5,3,3);}}
  if(P.loc&&L.wx!==undefined){g.strokeStyle='#3fb950';g.lineWidth=2;g.setLineDash([6,4]);g.beginPath();g.moveTo(X(P.x),Y(P.y));g.lineTo(X(L.wx),Y(L.wy));g.stroke();g.setLineDash([]);g.fillStyle='#3fb950';g.beginPath();g.arc(X(L.wx),Y(L.wy),4,0,7);g.fill();}
  const s=0.35;g.fillStyle=P.loc?'#3fb950':'#ff5a5f';g.beginPath();g.moveTo(X(P.x+s*1.6*Math.cos(P.th)),Y(P.y+s*1.6*Math.sin(P.th)));
  g.lineTo(X(P.x+s*Math.cos(P.th+2.5)),Y(P.y+s*Math.sin(P.th+2.5)));g.lineTo(X(P.x+s*Math.cos(P.th-2.5)),Y(P.y+s*Math.sin(P.th-2.5)));g.fill();}
 g.fillStyle='#8b949e';g.fillText('5 m',10,H-12);g.fillRect(10,H-8,5*k,2);}
async function live(){try{L=await (await fetch('/route/live',{cache:'no-store'})).json();
 const lg=M&&L.run?M.legs[L.leg]:null;
 $('rmInfo').textContent=!M?'loading map ...':(!L.mapOk?'map is from another teaching run - re-make RouteWebMap.h':
  (L.run?('leg '+(L.leg+1)+'/'+M.legs.length+'  '+(lg?lg[3]:'')+'  |  '+L.st+'  |  '+Math.round(100*L.frac)+' % of taught effort'
   +(L.lp===2?('  |  LiDAR located: '+L.ls.toFixed(1)+' / '+L.le.toFixed(1)+' m, '+L.off.toFixed(2)+' m off path (green)')
    :(L.lp===1?'  |  LiDAR NOT located - leg rules (red = estimate)':''))):'route not started'));
 draw();}catch(e){}}
fetch('/route/map.json',{cache:'force-cache'}).then(r=>r.json()).then(j=>{M=j;fitAll();draw();live();}).catch(()=>{$('rmInfo').textContent='no route map on the box';});
$('rmAll').onclick=()=>{follow=false;if(M)fitAll();draw();};$('rmFol').onclick=()=>{follow=true;draw();};$('rmScan').onchange=draw;
let drag=null;cv.onpointerdown=e=>{drag=[e.clientX,e.clientY];follow=false;};window.addEventListener('pointerup',()=>drag=null);
cv.onpointermove=e=>{if(!drag||!view)return;const k=Math.min(cv.clientWidth,cv.clientHeight)/view.span;view.cx-=(e.clientX-drag[0])/k;view.cy+=(e.clientY-drag[1])/k;drag=[e.clientX,e.clientY];draw();};
cv.onwheel=e=>{if(!view)return;e.preventDefault();view.span*=e.deltaY>0?1.2:1/1.2;draw();};
window.addEventListener('resize',draw);
setInterval(()=>{if(!document.hidden&&M)live();},500);
})();)rawjs";
