// ============================================================
//  웹페이지 (PROGMEM) — 폰 브라우저에서 http://<ESP IP>/ 로 열림
//  인터넷 되면 Leaflet+OSM 타일 지도, 안 되면(AP 모드) 첫 점 기준 미터 좌표 캔버스로 폴백
// ============================================================
#pragma once
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>FootTrack</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
html,body{margin:0;height:100%;overflow:hidden;font-family:system-ui,-apple-system,sans-serif;background:#111;color:#eee}
#map,#local{position:absolute;inset:0}
#local{display:none;background:#181c20}
#st{position:absolute;top:8px;left:8px;right:8px;z-index:1000;background:rgba(16,18,22,.88);padding:7px 10px;border-radius:8px;font-size:12px;line-height:1.4}
.ok{color:#5ad17a}.warn{color:#f5c542}.bad{color:#ff7b72}
#hud{position:absolute;left:0;right:0;bottom:0;z-index:1000;background:rgba(16,18,22,.93);border-top:1px solid #333;padding:10px 12px calc(10px + env(safe-area-inset-bottom,0px))}
.tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.tile{background:#22262b;border-radius:8px;padding:6px 8px;min-width:0}
.tile .l{font-size:11px;color:#9aa}
.tile .v{font-size:20px;font-weight:600;line-height:1.25;white-space:nowrap}
.tile .u{font-size:11px;color:#9aa;margin-left:2px}
#zones{display:flex;height:10px;border-radius:5px;overflow:hidden;margin:9px 0 3px;background:#2a2e33}
#zones div{height:100%;transition:width .3s}
.zl{display:flex;justify-content:space-between;font-size:10px;color:#9aa}
.row{display:flex;gap:8px;margin-top:9px}
button,a.btn{flex:1;padding:9px 4px;border:0;border-radius:8px;background:#2f6feb;color:#fff;font-size:14px;text-align:center;text-decoration:none;font-family:inherit}
.sec{background:#3a3f46!important}
</style></head><body>
<div id="map"></div><canvas id="local"></canvas>
<div id="st">연결 중…</div>
<div id="hud">
 <div class="tiles">
  <div class="tile"><div class="l">이동거리</div><div class="v"><span id="dist">0</span><span class="u" id="du">m</span></div></div>
  <div class="tile"><div class="l">현재속도</div><div class="v"><span id="spd">0.0</span><span class="u">km/h</span></div></div>
  <div class="tile"><div class="l">최고속도</div><div class="v"><span id="max">0.0</span><span class="u">km/h</span></div></div>
  <div class="tile"><div class="l">경과</div><div class="v"><span id="el">00:00</span></div></div>
 </div>
 <div id="zones"><div id="z0" style="background:#4b5563"></div><div id="z1" style="background:#22c55e"></div><div id="z2" style="background:#eab308"></div><div id="z3" style="background:#f97316"></div><div id="z4" style="background:#ef4444"></div></div>
 <div class="zl"><span>걷기</span><span>조깅</span><span>러닝</span><span>고속</span><span>스프린트 <b id="spr">0</b>회</span></div>
 <div class="row">
  <button class="sec" id="follow">따라가기: ON</button>
  <button id="reset">리셋</button>
  <a class="btn sec" href="/track.gpx" download="foottrack.gpx">GPX</a>
 </div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
(function(){
const $=id=>document.getElementById(id);
const hasL=!!window.L;
let pts=[],map=null,line=null,marker=null,follow=true,centered=false,last=null,dropped=false;
const pad=n=>(n<10?'0':'')+n;
const fmtEl=s=>pad(Math.floor(s/60))+':'+pad(s%60);

/* ---- Leaflet 지도 (인터넷 되는 경우: 폰 핫스팟 모드) ---- */
function initMap(){
  if(!hasL){$('map').style.display='none';$('local').style.display='block';return;}
  map=L.map('map',{zoomControl:false}).setView([37.5665,126.978],16);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap'}).addTo(map);
  line=L.polyline([],{color:'#2f6feb',weight:4,opacity:.9}).addTo(map);
  marker=L.circleMarker([0,0],{radius:7,color:'#fff',weight:2,fillColor:'#ef4444',fillOpacity:1}).addTo(map);
  map.on('dragstart',()=>setFollow(false));
}
function setFollow(f){follow=f;$('follow').textContent='따라가기: '+(f?'ON':'OFF');if(f&&last&&hasL)map.panTo(last,{animate:false});}

/* ---- 오프라인 캔버스 (AP 모드: 첫 점 기준 미터 좌표, 10/20/50m 격자) ---- */
const cv=$('local'),cx=cv.getContext('2d');let origin=null;
function toXY(p){if(!origin)origin=p;const R=6371000,d=Math.PI/180;return[(p[1]-origin[1])*d*R*Math.cos(origin[0]*d),(p[0]-origin[0])*d*R];}
function drawLocal(cur){
  if(hasL)return;
  const dpr=devicePixelRatio||1,W=cv.width=cv.clientWidth*dpr,H=cv.height=cv.clientHeight*dpr,hud=$('hud').offsetHeight*dpr;
  cx.fillStyle='#181c20';cx.fillRect(0,0,W,H);
  const xy=pts.map(toXY);if(cur)xy.push(toXY(cur));
  if(!xy.length){cx.fillStyle='#9aa';cx.font=(14*dpr)+'px system-ui';cx.fillText('오프라인 모드 · GPS fix 대기 중',16*dpr,60*dpr);return;}
  let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;for(const[x,y]of xy){x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);}
  const span=Math.max(x1-x0,y1-y0,20),s=Math.min(W-40*dpr,H-hud-70*dpr)/span;
  const P=p=>[W/2+(p[0]-(x0+x1)/2)*s,(H-hud)/2-(p[1]-(y0+y1)/2)*s];
  const g=span>300?50:span>100?20:10;cx.strokeStyle='#262b31';cx.lineWidth=dpr;
  for(let x=Math.floor(x0/g)*g;x<=x1+g;x+=g){const[X]=P([x,0]);cx.beginPath();cx.moveTo(X,0);cx.lineTo(X,H);cx.stroke();}
  for(let y=Math.floor(y0/g)*g;y<=y1+g;y+=g){const[,Y]=P([0,y]);cx.beginPath();cx.moveTo(0,Y);cx.lineTo(W,Y);cx.stroke();}
  cx.strokeStyle='#2f6feb';cx.lineWidth=3*dpr;cx.lineJoin='round';cx.lineCap='round';cx.beginPath();
  for(let i=0;i<pts.length;i++){const[X,Y]=P(xy[i]);i?cx.lineTo(X,Y):cx.moveTo(X,Y);}cx.stroke();
  if(cur){const[X,Y]=P(xy[xy.length-1]);cx.fillStyle='#ef4444';cx.beginPath();cx.arc(X,Y,7*dpr,0,7);cx.fill();}
  cx.fillStyle='#9aa';cx.font=(12*dpr)+'px system-ui';cx.fillText('오프라인 · 격자 '+g+'m · 범위 '+Math.round(span)+'m',12*dpr,58*dpr);
}

/* ---- 데이터 ---- */
function addPt(p){pts.push(p);if(hasL)line.addLatLng(p);}
function setPts(arr){pts=arr;origin=null;
  if(hasL){line.setLatLngs(arr);if(arr.length){map.fitBounds(line.getBounds(),{padding:[40,40],maxZoom:18});centered=true;}}
  else drawLocal(null);}
function update(d){
  const km=d.dist>=1000;$('dist').textContent=km?(d.dist/1000).toFixed(2):Math.round(d.dist);$('du').textContent=km?'km':'m';
  $('spd').textContent=d.spd.toFixed(1);$('max').textContent=d.max.toFixed(1);$('el').textContent=fmtEl(d.el);$('spr').textContent=d.spr;
  const zt=d.z.reduce((a,b)=>a+b,0)||1;d.z.forEach((v,i)=>$('z'+i).style.width=(v/zt*100)+'%');
  let s=d.gps?(d.fix?'<b class="ok">● FIX</b>':'<b class="warn">● 위성 탐색 중</b>'):'<b class="bad">● GPS 무응답</b> 배선/보레이트 확인';
  s+=' · 위성 '+d.sat+' · HDOP '+d.hdop.toFixed(1)+' · 점 '+d.n;
  if(d.imu)s+=' · PL '+d.pl.toFixed(0);
  $('st').innerHTML=s;
  if(d.fix){const p=[d.lat,d.lon];last=p;if(d.pt)addPt(p);
    if(hasL){marker.setLatLng(p);if(!centered){map.setView(p,17);centered=true;}else if(follow)map.panTo(p,{animate:false});}
    else drawLocal(p);}
}
function loadTrack(){return fetch('/track',{cache:'no-store'}).then(r=>r.json()).then(j=>setPts(j.pts.map(p=>[p[0],p[1]])));}
function connect(){
  const es=new EventSource('/events');
  es.addEventListener('fix',e=>update(JSON.parse(e.data)));
  es.onopen=()=>{if(dropped){dropped=false;loadTrack().catch(()=>{});}};
  es.onerror=()=>{dropped=true;$('st').innerHTML='<b class="bad">● 연결 끊김</b> 재접속 중…';};
}
initMap();
loadTrack().catch(()=>{}).then(connect);
$('follow').onclick=()=>setFollow(!follow);
$('reset').onclick=()=>{if(confirm('기록을 초기화할까요?'))fetch('/reset',{method:'POST'}).then(()=>{setPts([]);centered=false;last=null;});};
window.addEventListener('resize',()=>drawLocal(last));
})();
</script></body></html>
)HTML";
