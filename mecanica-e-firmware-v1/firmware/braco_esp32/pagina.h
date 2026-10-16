/*
  pagina.h - a pagina de controle (pagina.html) guardada na memoria flash do ESP32.

  O conteudo abaixo e uma copia exata de pagina.html dentro de uma string C++ (raw string).
  Para atualizar depois de editar pagina.html, copie o HTML inteiro e cole entre
  R"HTML(<!doctype html>
<!--
  Pagina de controle do braco, servida pelo proprio ESP32 em http://192.168.4.1

  O QUE ELA FAZ
  - Le do ESP32 a lista de juntas e seus limites (/api/state) e monta um slider por junta,
    com botoes de -1 / +1 grau para ajuste fino.
  - Ao arrastar um slider, envia o novo angulo (/api/move). Os envios passam por uma fila:
    so existe um pedido por vez e, enquanto um esta em andamento, fica guardado apenas o
    ultimo valor de cada slider. Assim o ESP32 nao fica sobrecarregado com centenas de pedidos.
  - Botoes: PARAR (congela onde esta), Ligar/Soltar servos, Posicao home.
  - Velocidade global (5 a 100 %).
  - Poses: salvar a posicao atual, ir para uma pose, apagar, e executar todas em sequencia
    (com pausa em ms e opcao de repetir).
  - A cada 0,7 s consulta o estado para manter a tela em dia (ex.: durante uma sequencia),
    e mostra "sem conexao" se o ESP32 parar de responder.
  - Tema claro/escuro automatico, feita para tela de celular. Nao usa nada da internet
    (funciona so com a rede do ESP32).

  Depois de editar este arquivo, gere de novo o pagina.h (ver instrucao dentro dele).
-->
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="color-scheme" content="dark"><meta name="theme-color" content="#0D2648">
<title>Braço ESP32</title>
<style>
:root{--bg:#0D2648;--card:#0D2A4F;--view:#0A1F3D;--ink:#FFFFFF;--dim:#E3E5E8;--mute:#88A5D6;--line:rgba(136,165,214,.22);--acc:#038FFD;--accink:#FFFFFF;--deep:#053A96;--ok:#4ADE80;--bad:#F0525E;--track:rgba(136,165,214,.28);--arm:#38A8FF;--ghost:rgba(136,165,214,.75);--grid:rgba(136,165,214,.13);--box:#88A5D6;--pt:#4ADE80;--mono:ui-monospace,'IBM Plex Mono',SFMono-Regular,Consolas,monospace}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.4 "Space Grotesk",system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding:max(16px,env(safe-area-inset-top)) 16px 40px}
main{max-width:560px;margin:0 auto;display:grid;gap:14px}
header{display:flex;align-items:center;justify-content:space-between;gap:12px}
h1{font-size:20px;margin:0;font-weight:650;letter-spacing:-.01em}
.st{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--mute)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad)}
.dot.on{background:var(--ok)}
.card{background:var(--card);border:1px solid var(--line);border-radius:3px;padding:14px}
.row{display:flex;gap:8px;flex-wrap:wrap}
button{font:inherit;font-weight:600;border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:3px;padding:10px 14px;min-height:44px;cursor:pointer}
button:active{transform:translateY(1px)}
button.pri{background:var(--acc);border-color:var(--acc);color:var(--accink)}
button.stop{background:var(--bad);border-color:var(--bad);color:#fff;flex:1}
button.sm{min-height:36px;padding:4px 12px}
.row>button{flex:1}
.j{display:grid;gap:6px;padding:10px 0;border-top:1px solid var(--line)}
.j:first-child{border-top:0;padding-top:0}
.jh{display:flex;justify-content:space-between;align-items:baseline}
.jn{font-weight:600}
.jv{font-variant-numeric:tabular-nums;color:var(--mute)}
.jc{display:grid;grid-template-columns:44px 1fr 44px;gap:10px;align-items:center}
.jc button{padding:0;min-height:40px;font-size:20px}
input[type=range]{width:100%;height:40px;background:transparent;-webkit-appearance:none;appearance:none;touch-action:pan-y}
input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:3px;background:var(--track)}
input[type=range]::-moz-range-track{height:6px;border-radius:3px;background:var(--track)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;border-radius:50%;background:var(--acc);margin-top:-11px;border:3px solid var(--card);box-shadow:0 0 0 1px var(--acc)}
input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--acc);border:3px solid var(--card)}
input[type=range]:disabled{opacity:.45}
h2{font-size:13px;margin:0 0 10px;text-transform:uppercase;letter-spacing:.06em;color:var(--mute);font-weight:650}
.sp{display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center}
.poses{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}
.pose{display:flex;border:1px solid var(--line);border-radius:3px;overflow:hidden}
.pose button{border:0;border-radius:0;min-height:40px}
.pose button+button{border-left:1px solid var(--line);color:var(--bad);padding:0 12px}
.pose.cur{outline:2px solid var(--acc)}
.pl{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin-top:12px;color:var(--mute);font-size:14px}
.pl input[type=number]{width:84px;font:inherit;padding:8px;border-radius:3px;border:1px solid var(--line);background:var(--bg);color:var(--ink)}
.pl label{display:flex;align-items:center;gap:6px}
.msg{min-height:20px;font-size:13px;color:var(--mute)}
.warn{display:none;color:var(--bad);font-size:13px}
/* tema SETA */
h1{letter-spacing:-.01em}
.brand{font:500 11px/1 var(--mono);color:var(--acc);letter-spacing:.12em;text-transform:uppercase;margin-bottom:4px}
h2,.vl,.st,.jv,.tcp,.steps,.msg{font-family:var(--mono)}
h2{color:var(--acc);letter-spacing:.1em;font-size:12px}
button{background:transparent}
button:hover{border-color:var(--acc)}
button.pri:hover{background:var(--deep)}
.card{background:var(--card)}
.tabs button.on,.steps button.on{box-shadow:0 0 0 1px var(--acc)}
input[type=text],input[type=number],select{background:var(--view)}
</style>
</head>
<body>
<main>
<header>
  <div><div class="brand">SETA · UFMT</div><h1>Braço ESP32</h1></div>
  <div class="st"><span id="dot" class="dot"></span><span id="stt">conectando…</span></div>
</header>

<div class="row">
  <button class="stop" id="bStop">PARAR</button>
</div>

<section class="card">
  <div class="row" style="margin-bottom:12px">
    <button id="bEng" class="pri">Ligar servos</button>
    <button id="bRelax">Soltar servos</button>
    <button id="bHome">Posição home</button>
  </div>
  <div class="warn" id="warn">PCA9685 não encontrado no I2C. Confira SDA/SCL e a alimentação.</div>
  <div id="joints"></div>
</section>

<section class="card">
  <h2>Velocidade</h2>
  <div class="sp"><span>Lento</span><input type="range" id="speed" min="5" max="100" step="1" value="40"><span id="spv">40%</span></div>
</section>

<section class="card">
  <h2>Poses salvas</h2>
  <div class="poses" id="poses"></div>
  <div class="row"><button id="bSave">+ Salvar pose atual</button></div>
  <div class="pl">
    <button class="pri sm" id="bPlay">▶ Executar</button>
    <button class="sm" id="bPStop">■ Parar</button>
    <label>Pausa <input type="number" id="dwell" min="0" max="10000" step="100" value="600"> ms</label>
    <label><input type="checkbox" id="loop"> repetir</label>
  </div>
</section>
<div class="msg" id="msg"></div>
</main>

<script>
// ---------- utilidades e comunicacao com o ESP32 ----------
const $=id=>document.getElementById(id);
let S=null, drag=-1, busy=false, queue={}, lastOk=0;
function msg(t){$('msg').textContent=t;clearTimeout(msg.t);msg.t=setTimeout(()=>$('msg').textContent='',3500)}
async function api(path){
  const r=await fetch('/api/'+path,{cache:'no-store'});
  if(!r.ok) throw new Error(r.status);
  lastOk=Date.now(); return r.json();
}
// Monta os sliders a partir da lista de juntas que o ESP32 informou.
function build(){
  const box=$('joints'); box.innerHTML='';
  S.j.forEach((j,i)=>{
    const d=document.createElement('div'); d.className='j';
    d.innerHTML=`<div class="jh"><span class="jn">${j.n}</span><span class="jv" id="v${i}">${Math.round(j.t)}°</span></div>
    <div class="jc"><button data-i="${i}" data-d="-1" aria-label="menos 1 grau">−</button>
    <input type="range" id="s${i}" min="${j.min}" max="${j.max}" step="1" value="${j.t}">
    <button data-i="${i}" data-d="1" aria-label="mais 1 grau">+</button></div>`;
    box.appendChild(d);
    const s=d.querySelector('input');
    s.addEventListener('input',()=>{drag=i;$('v'+i).textContent=s.value+'°';queue[i]=s.value;pump()});
    const end=()=>{setTimeout(()=>{if(drag===i)drag=-1},400)};
    s.addEventListener('change',end); s.addEventListener('pointerup',end);
  });
  box.querySelectorAll('button[data-d]').forEach(b=>b.addEventListener('click',()=>{
    const i=+b.dataset.i, s=$('s'+i); let v=Math.max(+s.min,Math.min(+s.max,+s.value+(+b.dataset.d)));
    s.value=v;$('v'+i).textContent=v+'°';queue[i]=v;pump();
  }));
}
// Fila de envio: manda so o valor mais recente de cada slider, um pedido por vez.
async function pump(){
  if(busy) return; busy=true;
  try{
    while(Object.keys(queue).length){
      const q=queue; queue={};
      const p=Object.entries(q).map(([i,a])=>`j${i}=${a}`).join('&');
      S=await api('move?'+p); render(true);
      await new Promise(r=>setTimeout(r,30));
    }
  }catch(e){status(false)}
  busy=false;
}
function status(ok){
  $('dot').className='dot'+(ok?' on':''); $('stt').textContent=ok?(S&&S.engaged?'servos ligados':'servos soltos'):'sem conexão';
}
// Atualiza a tela com o estado recebido (sem mexer no slider que o usuario esta arrastando).
function render(keep){
  if(!S) return;
  status(true);
  $('warn').style.display=S.pca?'none':'block';
  $('spv').textContent=S.speed+'%'; if(document.activeElement!==$('speed')) $('speed').value=S.speed;
  S.j.forEach((j,i)=>{
    if(drag===i||queue[i]!==undefined) return;
    const s=$('s'+i); if(s){s.value=j.t;$('v'+i).textContent=Math.round(j.t)+'°'}
  });
  $('bEng').textContent=S.engaged?'Servos ligados ✓':'Ligar servos';
  const pz=$('poses'); pz.innerHTML='';
  S.poses.forEach(i=>{
    const e=document.createElement('div'); e.className='pose'+(S.playing&&S.cur===i?' cur':'');
    e.innerHTML=`<button data-go="${i}">Pose ${i+1}</button><button data-del="${i}" aria-label="apagar">×</button>`;
    pz.appendChild(e);
  });
  if(!S.poses.length) pz.innerHTML='<span style="color:var(--mute);font-size:14px">Nenhuma pose salva ainda.</span>';
}
// Consulta periodica do estado.
async function poll(){
  try{ if(!busy){const d=await api('state'); if(!S||!S.j||S.j.length!==d.j.length){S=d;build()}else S=d; render(); } }
  catch(e){status(false)}
}
async function act(path,okmsg){
  try{S=await api(path);render();if(okmsg)msg(okmsg)}catch(e){status(false);msg('Falha de comunicação')}
}
// ---------- botoes ----------
$('bStop').onclick=()=>act('stop');
$('bEng').onclick=()=>act('engage');
$('bRelax').onclick=()=>act('relax','Servos soltos (sem torque)');
$('bHome').onclick=()=>act('home');
$('bSave').onclick=()=>act('save','Pose salva');
$('bPlay').onclick=()=>act('play?loop='+($('loop').checked?1:0)+'&dwell='+(+$('dwell').value||0));
$('bPStop').onclick=()=>act('stop');
$('poses').addEventListener('click',e=>{
  const g=e.target.closest('[data-go]'),d=e.target.closest('[data-del]');
  if(g) act('go?i='+g.dataset.go); else if(d) act('del?i='+d.dataset.del,'Pose apagada');
});
let sp=null;
$('speed').addEventListener('input',()=>{$('spv').textContent=$('speed').value+'%';clearTimeout(sp);sp=setTimeout(()=>act('speed?v='+$('speed').value),120)});
poll(); setInterval(poll,700);
setInterval(()=>{if(Date.now()-lastOk>2500)status(false)},1000);
</script>
</body>
</html>
)HTML";
