#pragma once

// Embedded calibration portal (served by esp32dev_cal only).
namespace calib_pages {

inline constexpr const char kIndexHtml[] = R"RDWGHTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RoboDawg calibration</title>
<style>
:root{--bg:#0f1115;--panel:#1a1f2a;--text:#e8ecf1;--muted:#9aa7b6;--acc:#3dd6c6;--hip:#5ad8ff;--line:#3a4556;}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--text);line-height:1.45}
header{padding:16px 20px;border-bottom:1px solid #222;background:#12151c}
h1{margin:0;font-size:1.15rem;font-weight:650}
.sub{color:var(--muted);font-size:.88rem;margin-top:6px}
main{padding:16px;max-width:1100px;margin:0 auto}
.grid{display:grid;gap:14px}
@media(min-width:860px){.legs{grid-template-columns:repeat(2,1fr)}}
.card{background:var(--panel);border:1px solid #242a36;border-radius:12px;padding:14px}
.card h2{margin:0 0 8px;font-size:1rem}
.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
.ref{background:#141a24;border:1px solid #2a3344;border-radius:12px;padding:14px;margin-bottom:16px}
.ref h2{margin:0 0 8px;font-size:1rem}
.ref ul{margin:6px 0 0 18px;color:var(--muted)}
.ref .key{color:var(--text)}
svg{display:block;margin:6px auto 10px}
label{font-size:.78rem;color:var(--muted)}
input[type=range]{width:100%}
.val{font-variant-numeric:tabular-nums;color:var(--acc);min-width:44px;text-align:right}
button,.btn{background:#243044;color:var(--text);border:1px solid #3a4a62;border-radius:8px;padding:8px 12px;cursor:pointer;font:inherit}
button.primary{background:linear-gradient(135deg,#1c6b63,#2aa89a);border-color:#2c8f84}
button.danger{background:#4a2323;border-color:#6a3535}
button:disabled{opacity:.45;cursor:not-allowed}
.chk{display:flex;align-items:center;gap:6px;margin:4px 8px 4px 0}
.walkbar{display:grid;gap:10px}
#ip{font-family:ui-monospace,Menlo,Consolas,monospace;color:var(--acc)}
.status{font-size:.85rem;color:var(--muted)}
</style>
</head>
<body>
<header>
<h1>RoboDawg bench calibration</h1>
<div class="sub">Join WiFi AP <strong>RoboDawg-Cal</strong> then open <span id="ip">http://192.168.4.1</span> (this page).</div>
</header>
<main>
<section class="ref">
<h2>What “software 0°” means here</h2>
<ul>
<li><span class="key">Hip joint 0°</span> — IK hip angle 0. With default neutrals, the PCA command is about <strong>90°</strong> on that servo.</li>
<li><span class="key">Thigh joint 0°</span> — shoulder composition 0; commanded servo ≈ <strong>90°</strong>.</li>
<li><span class="key">Shin joint 0°</span> — knee offset 0; commanded servo ≈ <strong>90°</strong>.</li>
</ul>
<p class="status">Tap <em>Hold 0° bench</em> on a leg to command hip+thigh+shin to that neutral pose, then slide the matching <em>Trim (µs)</em> rows until the hardware matches your CAD / eyeball reference. <strong>Save NVS</strong> writes all trims to flash.</p>
</section>

<section class="grid legs">
<div class="card" data-leg="0"><h2>Front right</h2>
<svg viewBox="0 0 120 200" width="140" height="200" aria-hidden="true">
<line x1="60" y1="18" x2="60" y2="88" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="60" cy="18" r="7" fill="var(--hip)"/>
<circle cx="60" cy="92" r="7" fill="var(--hip)"/>
<line x1="60" y1="92" x2="78" y2="172" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="82" cy="176" r="14" fill="#222" stroke="#444" stroke-width="3"/>
</svg>
<div class="row"><button type="button" class="primary btnHold" data-slot="0">Hold 0° bench</button><button type="button" class="btnRelease" data-slot="0">Release hold</button></div>
</div>
<div class="card" data-leg="1"><h2>Front left</h2>
<svg viewBox="0 0 120 200" width="140" height="200" aria-hidden="true">
<line x1="60" y1="18" x2="60" y2="88" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="60" cy="18" r="7" fill="var(--hip)"/>
<circle cx="60" cy="92" r="7" fill="var(--hip)"/>
<line x1="60" y1="92" x2="42" y2="172" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="38" cy="176" r="14" fill="#222" stroke="#444" stroke-width="3"/>
</svg>
<div class="row"><button type="button" class="primary btnHold" data-slot="1">Hold 0° bench</button><button type="button" class="btnRelease" data-slot="1">Release hold</button></div>
</div>
<div class="card" data-leg="2"><h2>Back right</h2>
<svg viewBox="0 0 120 200" width="140" height="200" aria-hidden="true">
<line x1="60" y1="18" x2="56" y2="88" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="60" cy="18" r="7" fill="var(--hip)"/>
<circle cx="54" cy="92" r="7" fill="var(--hip)"/>
<line x1="54" y1="92" x2="74" y2="172" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="78" cy="176" r="14" fill="#222" stroke="#444" stroke-width="3"/>
</svg>
<div class="row"><button type="button" class="primary btnHold" data-slot="2">Hold 0° bench</button><button type="button" class="btnRelease" data-slot="2">Release hold</button></div>
</div>
<div class="card" data-leg="3"><h2>Back left</h2>
<svg viewBox="0 0 120 200" width="140" height="200" aria-hidden="true">
<line x1="60" y1="18" x2="64" y2="88" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="60" cy="18" r="7" fill="var(--hip)"/>
<circle cx="66" cy="92" r="7" fill="var(--hip)"/>
<line x1="66" y1="92" x2="46" y2="172" stroke="var(--line)" stroke-width="10" stroke-linecap="round"/>
<circle cx="42" cy="176" r="14" fill="#222" stroke="#444" stroke-width="3"/>
</svg>
<div class="row"><button type="button" class="primary btnHold" data-slot="3">Hold 0° bench</button><button type="button" class="btnRelease" data-slot="3">Release hold</button></div>
</div>
</section>

<section class="card" style="margin-top:16px">
<h2>PCA9685 trims (µs)</h2>
<p class="status">−200…+200 µs per channel. Bench “Hold” uses MODE_SERVO so gait will not overwrite these while you trim.</p>
<div id="allTrim" class="grid"></div>
<div class="row" style="margin-top:10px">
<button type="button" class="primary" id="btnSave">Save NVS</button>
<button type="button" class="danger" id="btnStop">STOP all</button>
<span class="status" id="msg"></span>
</div>
</section>

<section class="card" style="margin-top:16px">
<h2>Walk test (partial robot)</h2>
<p class="status">Uncheck legs or wheels you want electrically idle. Disabled legs are held at ~90° servo commands.</p>
<div class="row" id="legToggles"></div>
<div class="row" id="wheelToggles" style="margin-top:6px"></div>
<div class="walkbar" style="margin-top:12px">
<label>Forward / back (WALK x)</label><input type="range" id="wx" min="-100" max="100" value="0"/><div class="row"><span class="status">x</span><span class="val" id="wxv">0</span></div>
<label>Strafe (WALK y)</label><input type="range" id="wy" min="-100" max="100" value="0"/><div class="row"><span class="status">y</span><span class="val" id="wyv">0</span></div>
<label>Yaw rate (WALK yaw)</label><input type="range" id="ww" min="-100" max="100" value="0"/><div class="row"><span class="status">yaw</span><span class="val" id="wwv">0</span></div>
</div>
<div class="row" style="margin-top:12px">
<button type="button" class="primary" id="btnWalk">Apply walk / keep updating</button>
<button type="button" id="btnIdle">Idle (stand still)</button>
</div>
</section>
</main>
<script>
const LEGS=['FR','FL','BR','BL'];
const CH=[[0,4,8,12],[1,5,9,13],[2,6,10,14],[3,7,11,15]];
function post(path,body){return fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body}).then(r=>r.text());}
function flash(t){const m=document.getElementById('msg');m.textContent=t;setTimeout(()=>{if(m.textContent===t)m.textContent='';},1600);}
function mkSlider(ch,label){
const id='t'+ch;const host=document.getElementById('allTrim');const row=document.createElement('div');row.className='row';
row.innerHTML='<label style="flex:1">'+label+' (ch'+ch+')</label><input type="range" id="'+id+'" min="-200" max="200" value="0"/><span class="val" id="'+id+'v">0</span>';
host.appendChild(row);
const el=document.getElementById(id),ev=document.getElementById(id+'v');
let t=null;
const send=()=>{const v=+el.value;ev.textContent=v;post('/api/trim','ch='+ch+'&trim='+v).catch(()=>flash('net error'));};
el.addEventListener('input',()=>{ev.textContent=el.value;if(t)clearTimeout(t);t=setTimeout(send,45);});
el.addEventListener('change',send);
return el;
}
for(let s=0;s<4;s++)for(let j=0;j<4;j++){const nm=['hip','thigh','shin','wheel'][j];mkSlider(CH[s][j],LEGS[s]+' '+nm);}
const legT=document.getElementById('legToggles');
LEGS.forEach((n,i)=>{const w=document.createElement('label');w.className='chk';w.innerHTML='<input type="checkbox" class="legE" data-i="'+i+'" checked/> Leg '+n;legT.appendChild(w);});
const wheelT=document.getElementById('wheelToggles');
LEGS.forEach((n,i)=>{const w=document.createElement('label');w.className='chk';w.innerHTML='<input type="checkbox" class="whE" data-i="'+i+'" checked/> Wheel '+n;wheelT.appendChild(w);});
function maskFrom(cls){let m=0;document.querySelectorAll(cls).forEach(c=>{if(c.checked)m|=1<<(+c.dataset.i);});return m;}
function syncMask(){post('/api/mask','legs='+maskFrom('.legE')+'&wheels='+maskFrom('.whE')).catch(()=>{});}
document.querySelectorAll('.legE,.whE').forEach(e=>e.addEventListener('change',syncMask));
async function pull(){
const r=await fetch('/api/state');const j=await r.json();
for(let ch=0;ch<16;ch++){const el=document.getElementById('t'+ch);if(el)el.value=j.trim[ch];const ev=document.getElementById('t'+ch+'v');if(ev)ev.textContent=j.trim[ch];}
document.querySelectorAll('.legE').forEach(c=>{c.checked=!!(j.legMask&(1<<+c.dataset.i));});
document.querySelectorAll('.whE').forEach(c=>{c.checked=!!(j.wheelMask&(1<<+c.dataset.i));});
}
document.getElementById('btnSave').onclick=()=>post('/api/save','').then(()=>flash('Saved')).catch(()=>flash('save failed'));
document.getElementById('btnStop').onclick=()=>post('/api/stop','').then(()=>flash('Stopped')).catch(()=>flash('stop failed'));
document.querySelectorAll('.btnHold').forEach(b=>b.onclick=()=>post('/api/hold','slot='+b.dataset.slot+'&on=1').then(()=>flash('Hold leg '+b.dataset.slot)).catch(()=>{}));
document.querySelectorAll('.btnRelease').forEach(b=>b.onclick=()=>post('/api/hold','slot='+b.dataset.slot+'&on=0').then(()=>flash('Released')).catch(()=>{}));
function bindR(id,pv){const el=document.getElementById(id),v=document.getElementById(pv);const upd=()=>v.textContent=el.value;el.addEventListener('input',upd);upd();}
bindR('wx','wxv');bindR('wy','wyv');bindR('ww','wwv');
let walkTimer=null;
function pushWalk(){post('/api/walk','x='+document.getElementById('wx').value+'&y='+document.getElementById('wy').value+'&yaw='+document.getElementById('ww').value).catch(()=>{});}
document.getElementById('btnWalk').onclick=()=>{syncMask();pushWalk();if(walkTimer)clearInterval(walkTimer);walkTimer=setInterval(pushWalk,120);flash('Walk stream on');};
document.getElementById('btnIdle').onclick=()=>{if(walkTimer){clearInterval(walkTimer);walkTimer=null;}post('/api/idle','').then(()=>flash('Idle')).catch(()=>{});};
pull().catch(()=>{});
setInterval(pull,4000);
</script>
</body>
</html>)RDWGHTML";

}  // namespace calib_pages
