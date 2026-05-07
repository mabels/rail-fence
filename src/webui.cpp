#include "globals.h"
#include <ArduinoJson.h>

static AsyncWebServer server(80);

static const char INDEX_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Slider</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font:14px monospace;background:#111;color:#ddd;padding:12px;max-width:460px}
h1{color:#4af;margin-bottom:14px;font-size:1.3em}
nav{display:flex;gap:8px;margin-bottom:14px}
nav button{flex:1;padding:8px;background:#1e1e1e;color:#888;border:1px solid #333;border-radius:4px;cursor:pointer;font:13px monospace}
nav button.on{background:#4af;color:#000;border-color:#4af}
section{display:none}.show{display:block!important}
.card{background:#181818;border:1px solid #2a2a2a;border-radius:6px;padding:12px;margin-bottom:12px}
.lbl{color:#666;font-size:.8em;text-transform:uppercase;letter-spacing:.06em;margin-bottom:4px}
.big{font-size:2em;color:#4fa;margin:2px 0;letter-spacing:-1px}
.big.neg{color:#f84}
.sub{font-size:1em;color:#888;margin:2px 0}
.badge{display:inline-block;padding:2px 10px;border-radius:3px;font-size:.8em;margin-top:6px}
.badge.idle{background:#0a2a0a;color:#4fa}
.badge.run{background:#2a1a00;color:#fa4;animation:blink 1s infinite}
.badge.wait{background:#1a1a2a;color:#88f;animation:blink 1s infinite}
.badge.fail{background:#2a0a0a;color:#f44}
@keyframes blink{50%{opacity:.5}}
.row{display:flex;gap:8px;margin-bottom:8px}
.step-btn{flex:1;padding:9px;background:#1e1e1e;color:#888;border:1px solid #333;border-radius:4px;cursor:pointer;font:13px monospace}
.step-btn.on{background:#4af;color:#000;border-color:#4af}
.jog-btn{flex:1;padding:16px;background:#1a2a1a;color:#8f8;border:none;border-radius:4px;cursor:pointer;font:bold 22px monospace}
.jog-btn:active{background:#2a4a2a}
input[type=number]{width:100%;padding:9px;background:#222;color:#eee;border:1px solid #333;border-radius:4px;margin:4px 0 8px;font:14px monospace}
.btn{width:100%;padding:11px;border:none;border-radius:4px;cursor:pointer;font:bold 14px monospace;margin-top:0}
.btn.blue{background:#1a3a5a;color:#4af}.btn.blue:hover{background:#2a4a7a}
.btn.red{background:#4a1010;color:#f88}.btn.red:hover{background:#6a1818}
.btn.amber{background:#4a3000;color:#fa8}.btn.amber:hover{background:#6a4400}
.btn.green{background:#0a3a0a;color:#8f8}.btn.green:hover{background:#1a5a1a}
.info{color:#555;line-height:1.9em;font-size:.9em}
</style>
</head>
<body>
<h1>Slider</h1>
<nav>
  <button class="on" onclick="tab('ctl',this)">Control</button>
  <button onclick="tab('inf',this)">Info</button>
</nav>

<!-- ── Control ── -->
<section id="ctl" class="show">
  <div class="card">
    <div class="lbl">Target</div>
    <div class="big" id="tgt">+0.0 mm</div>
    <div class="lbl" style="margin-top:8px">Position</div>
    <div class="sub" id="drv">+0.0 mm</div>
    <div style="margin-top:8px">
      <div class="lbl">Peer position</div>
      <div class="sub" id="peer-pos">—</div>
    </div>
    <span class="badge idle" id="badge">idle</span>
  </div>

  <div class="card">
    <div class="lbl">Step size</div>
    <div class="row">
      <button class="step-btn" id="s0" onclick="setStep(0)">0.1 mm</button>
      <button class="step-btn on" id="s1" onclick="setStep(1)">1 mm</button>
      <button class="step-btn" id="s2" onclick="setStep(2)">10 mm</button>
    </div>
    <div class="row" style="margin-top:4px">
      <button class="jog-btn" onclick="jog(-1)">&#9664;</button>
      <button class="jog-btn" onclick="jog(1)">&#9654;</button>
    </div>
  </div>

  <div class="card">
    <div class="lbl">Go to position (mm)</div>
    <input type="number" id="goto-mm" value="0" step="0.1" min="-9999" max="9999">
    <button class="btn blue" onclick="doGoto()">&#9654; Go</button>
  </div>

  <div class="card">
    <div class="row">
      <button class="btn amber" id="lock-btn" onclick="toggleLock()" style="margin:0;flex:1">Lock: ON</button>
      <div style="width:8px"></div>
      <button class="btn red" id="rst-btn" style="margin:0;flex:1;position:relative;overflow:hidden"
        onmousedown="holdStart(event)" ontouchstart="holdStart(event)"
        onmouseup="holdCancel()" onmouseleave="holdCancel()"
        ontouchend="holdCancel()" ontouchcancel="holdCancel()">
        <span id="rst-fill" style="position:absolute;left:0;top:0;height:100%;width:0%;background:rgba(255,80,80,.35);transition:none;pointer-events:none"></span>
        <span id="rst-lbl">Hold to reset</span>
      </button>
    </div>
  </div>
</section>

<!-- ── Info ── -->
<section id="inf">
  <div class="card">
    <div class="lbl">Device info</div>
    <div class="info" id="dev"></div>
  </div>
</section>

<script>
let locked=true,stepIdx=1;
const STEP_STEPS=[80,800,8000];

function tab(id,b){
  document.querySelectorAll('section').forEach(s=>s.classList.remove('show'));
  document.querySelectorAll('nav button').forEach(x=>x.classList.remove('on'));
  document.getElementById(id).classList.add('show');b.classList.add('on');
}
function fmm(v){return(v>=0?'+':'')+v.toFixed(1)+' mm';}

async function poll(){
  try{
    const d=await(await fetch('/api/status')).json();
    locked=!!d.locked;
    stepIdx=d.step_idx||1;

    const tEl=document.getElementById('tgt');
    tEl.textContent=fmm(d.target_mm||0);
    tEl.className='big'+((d.target_mm||0)<0?' neg':'');
    document.getElementById('drv').textContent=fmm(d.actual_mm||0);
    document.getElementById('peer-pos').textContent=fmm((d.peer_pos||0)/800);

    const badge=document.getElementById('badge');
    const st=d.sync_state||'idle';
    badge.textContent=st+(d.txid?(' #'+d.txid):'');
    badge.className='badge '+(st==='idle'?'idle':st==='moving'?'run':st==='waiting_peer'?'wait':'fail');

    [0,1,2].forEach(i=>{
      const el=document.getElementById('s'+i);
      if(el)el.className='step-btn'+(i===stepIdx?' on':'');
    });

    const lb=document.getElementById('lock-btn');
    if(lb){lb.textContent='Lock: '+(locked?'ON':'OFF');lb.className='btn '+(locked?'amber':'green');}

    document.getElementById('dev').innerHTML=
      'Hostname: <b>'+d.hostname+'</b><br>IP: <b>'+d.ip+'</b><br>MAC: <b>'+d.mac+'</b><br>'+
      'Transport: <b>'+d.transport+'</b><br>'+
      'Sync: <b>'+st+'</b> txid=<b>'+d.txid+'</b><br>i2c: <b>'+d.i2c_scan+'</b>';
  }catch(e){}
}

async function setStep(idx){
  stepIdx=idx;
  [0,1,2].forEach(i=>document.getElementById('s'+i).className='step-btn'+(i===idx?' on':''));
  await fetch('/api/set_step',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({idx})});
}
async function jog(dir){
  await fetch('/api/move',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({steps:STEP_STEPS[stepIdx]*dir})});
}
async function doGoto(){
  const mm=parseFloat(document.getElementById('goto-mm').value);
  if(isNaN(mm))return;
  await fetch('/api/goto',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({mm})});
}
async function toggleLock(){
  await fetch('/api/set_lock',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({locked:!locked})});
}
let holdRaf=null,holdStart_t=0;
const HOLD_MS=1500;
function holdStart(e){
  e.preventDefault();holdStart_t=Date.now();
  const fill=document.getElementById('rst-fill');
  const lbl=document.getElementById('rst-lbl');
  fill.style.transition='none';fill.style.width='0%';lbl.textContent='Hold…';
  function frame(){
    const pct=Math.min(100,(Date.now()-holdStart_t)/HOLD_MS*100);
    fill.style.width=pct+'%';
    if(pct<100){holdRaf=requestAnimationFrame(frame);}else{holdRaf=null;doReset();}
  }
  holdRaf=requestAnimationFrame(frame);
}
function holdCancel(){
  if(holdRaf){cancelAnimationFrame(holdRaf);holdRaf=null;}
  const fill=document.getElementById('rst-fill');
  const lbl=document.getElementById('rst-lbl');
  fill.style.transition='width .3s ease';fill.style.width='0%';lbl.textContent='Hold to reset';
}
async function doReset(){
  document.getElementById('rst-lbl').textContent='✓ Reset!';
  await fetch('/api/reset_pos',{method:'POST'});
  setTimeout(()=>{holdCancel();},800);
}

setInterval(poll,400);poll();
</script>
</body></html>
)html";

// ─────────────────────────────────────────────────────────────────────────────

void webserver_init() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        const char* sync_str = "idle";
        switch (g_sync_state) {
            case SyncState::IDLE:         sync_str = "idle";         break;
            case SyncState::MOVING:       sync_str = "moving";       break;
            case SyncState::WAITING_PEER: sync_str = "waiting_peer"; break;
            case SyncState::FAILED:       sync_str = "failed";       break;
        }
        JsonDocument doc;
        doc["sync_state"]   = sync_str;
        doc["transport"]    = (g_transport == Transport::ESPNOW) ? "espnow" : "udp";
        doc["is_initiator"] = g_is_initiator;
        doc["txid"]         = g_txid;
        doc["peer_pos"]     = (int32_t)g_peer_pos;
        doc["position"]     = stepper ? stepper->getCurrentPosition() : 0;
        doc["busy"]         = stepper ? stepper->isRunning() : false;
        doc["target_mm"]    = g_target_pos / STEPS_PER_MM;
        doc["actual_mm"]    = (stepper ? stepper->getCurrentPosition() : 0) / STEPS_PER_MM;
        doc["step_idx"]     = g_enc_step_idx;
        doc["locked"]       = g_locked;
        doc["hostname"]     = g_hostname;
        doc["ip"]           = WiFi.localIP().toString();
        doc["mac"]          = WiFi.macAddress();
        doc["i2c_scan"]     = g_i2c_scan;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    server.on("/api/move", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            req->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            if (g_sync_state != SyncState::IDLE) return;
            if (stepper && stepper->isRunning()) return;
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            const int32_t steps = doc["steps"] | 0;
            if (steps == 0) return;
            g_target_pos += steps;
            if (g_locked) {
                g_target_pos = constrain(g_target_pos,
                    (int32_t)(MIN_POS_MM * STEPS_PER_MM),
                    (int32_t)(MAX_POS_MM * STEPS_PER_MM));
            }
        }
    );

    server.on("/api/reset_pos", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (stepper) stepper->setCurrentPosition(0);
        g_target_pos    = 0;
        g_commanded_pos = 0;
        nvs_save_position(0);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/blink", HTTP_POST, [](AsyncWebServerRequest* req) {
        for (int i = 0; i < 5; i++) {
            neopixelWrite(48, 0, 60, 0); delay(200);
            neopixelWrite(48, 0,  0, 0); delay(200);
        }
        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/test_step", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!stepper) { req->send(503, "application/json", "{\"error\":\"no stepper\"}"); return; }
        if (stepper->isRunning()) { req->send(409, "application/json", "{\"error\":\"busy\"}"); return; }
        stepper->setSpeedInHz(1000);
        stepper->setAcceleration(2000);
        stepper->move(200);
        req->send(200, "application/json", "{\"ok\":true,\"steps\":200}");
    });

    server.on("/api/set_step", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            g_enc_step_idx = constrain((int8_t)(doc["idx"] | 1),
                                       (int8_t)0, (int8_t)(ENC_STEP_COUNT - 1));
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    server.on("/api/set_lock", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            g_locked = doc["locked"] | false;
            if (g_locked) {
                g_target_pos = constrain(g_target_pos,
                    (int32_t)(MIN_POS_MM * STEPS_PER_MM),
                    (int32_t)(MAX_POS_MM * STEPS_PER_MM));
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    server.on("/api/goto", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            if (g_sync_state != SyncState::IDLE) {
                req->send(200, "application/json", "{\"ok\":false,\"reason\":\"busy\"}");
                return;
            }
            JsonDocument doc;
            if (deserializeJson(doc, data, len)) return;
            int32_t target = (int32_t)((doc["mm"] | 0.0f) * STEPS_PER_MM);
            if (g_locked) {
                target = constrain(target,
                    (int32_t)(MIN_POS_MM * STEPS_PER_MM),
                    (int32_t)(MAX_POS_MM * STEPS_PER_MM));
            }
            g_target_pos = target;
            req->send(200, "application/json", "{\"ok\":true}");
        }
    );

    g_events.onConnect([](AsyncEventSourceClient* c) {
        c->send("connected", "log", millis());
    });
    server.addHandler(&g_events);

    server.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html",
            "<!doctype html><html><head>"
            "<meta charset=utf-8><title>Log</title>"
            "<style>body{background:#111;color:#0f0;font:13px/1.4 monospace;margin:0;padding:8px}"
            "#log{white-space:pre-wrap;word-break:break-all}</style></head><body>"
            "<b id=host></b> &mdash; live log (<a href=/ style=color:#08f>&#8592; back</a>)"
            "<hr><div id=log></div><script>"
            "document.getElementById('host').textContent=location.host;"
            "const d=document.getElementById('log');"
            "const es=new EventSource('/events');"
            "es.addEventListener('log',e=>{"
            "  d.textContent+=e.data;"
            "  if(!d.textContent.endsWith('\\n'))d.textContent+='\\n';"
            "  window.scrollTo(0,document.body.scrollHeight);"
            "});</script></body></html>"
        );
    });

    server.begin();
    Serial.println("[web] server started on :80");
}
