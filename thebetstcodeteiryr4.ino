/*
  ============================================================================
  ESP32-C3 — mașină cu comandă prin Wi-Fi — v3.1
  "Wi-Fi stabil + viteză variabilă pe PWM + auto-vindecare HTTP"
  ============================================================================

  NOU ÎN v3.1 (după simptomul "după un timp nu mai răspunde"):

    A. Cauza cea mai probabilă a fost AbortController(800 ms) din dashboard.
       Când o cerere e anulată de browser, ESP32 rămâne cu socketul în
       HC_WAIT_READ / HC_WAIT_CLOSE (până la 2 s per incident). La 5 cereri/s
       anulările se înmulțesc, serverul nu mai ajunge niciodată la HC_NONE și
       dashboard-ul moare definitiv. Abort-ul a fost ELIMINAT din JS.
    B. Viteza de retrimitere coborâtă la 300 ms, failsafe la 1500 ms — mai
       puține conexiuni TCP pe secundă (fiecare cerere HTTP = conexiune nouă
       pe ESP32, iar pool-ul lwIP e mic).
    C. /api/status separat ELIMINAT: răspunsul de la /api/control conține acum
       tot statusul, deci un singur flux de cereri în loc de două paralele.
    D. AP_INACTIVE_SEC ridicat de la 10 la 60 s — cu 10 s, un tab în fundal
       (Chrome limitează timer-ele) putea fi deconectat de AP.
    E. Auto-vindecare în doi pași: dacă un client e asociat dar nu mai vine
       nicio cerere 20 s, se reiau socket-urile de listen; la 60 s, restart
       curat al plăcii. Nu se activează dacă nu e niciun client conectat.
    F. TCP_NODELAY pe socket-ul HTTP — răspunsurile pleacă imediat.

  ============================================================================

  Ce s-a schimbat față de versiunea anterioară (cea cu delay-ul rezolvat):

  WI-FI / STABILITATE
    1. esp_wifi_set_inactive_time(AP, 10 s) — stațiile moarte sunt scoase din
       tabela AP. Fără asta, după 2-3 reconectări ale laptopului rămâi fără
       sloturi (AP_MAX_CLIENTS=4) și laptopul e refuzat la reconectare.
    2. Log explicit pentru fiecare connect/disconnect cu REASON code. Așa afli
       în 5 secunde dacă e rază (200), parolă/handshake (15), sau laptopul a
       plecat pe altă rețea (8).
    3. Putere TX setată DUPĂ softAP (înainte era ignorată de driver) + 15 dBm.
    4. Contorizare clienți conectați / pierduți, expusă în /api/status.
    5. Heartbeat-ul Serial arată și motivul ultimei deconectări.
    6. Dashboard-ul afișează motivul ultimei căderi + numărul de clienți, ca
       să nu mai ghicești.
    7. VIDEO: bucla principală nu mai cheltuie timp pe scrierea PWM dacă
       valoarea nu s-a schimbat (mai puțin jitter pe stack-ul Wi-Fi).

  MOTOARE / VITEZĂ  (nou)
    8. PWM real pe cei 4 pini DRV8833 (20 kHz, peste pragul audibil) —
       viteză variabilă 20-100%, nu doar full-on ca înainte.
    9. SOFT-START: duty-ul urcă rampat în ~0.45 s. Asta limitează vârful de
       curent la pornire, care e cauza #1 a reset-urilor de brownout — deci
       ajută DIRECT la stabilitatea Wi-Fi-ului.
   10. Ramp-down rapid la oprire (frânare lină, fără șoc în driver).
   11. TRIM_DREAPTA / TRIM_STANGA — compensezi dacă o roată fuge mai tare.
   12. MAX_PWM_PCT — limită dură de putere, de scăzut dacă ai brownout.
   13. Viteza se schimbă din slider în dashboard și din tastele [ și ].

  Cablaj (neschimbat):
    DRV8833: IN1=GPIO10, IN2=GPIO7, IN3=GPIO6, IN4=GPIO5
    Servo direcție: semnal=GPIO3
    LED: roșu=GPIO0, galben=GPIO1, verde=GPIO2
    MOTOR + SERVO pe sursă separată, GND comun cu ESP32!

  Necesită: arduino-esp32 core 3.x + librăria ESP32Servo.
  ============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include <esp_system.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define EV_AP_STACONN  ARDUINO_EVENT_WIFI_AP_STACONNECTED
  #define EV_AP_STADISC  ARDUINO_EVENT_WIFI_AP_STADISCONNECTED
#else
  #define EV_AP_STACONN  SYSTEM_EVENT_AP_STACONNECTED
  #define EV_AP_STADISC  SYSTEM_EVENT_AP_STADISCONNECTED
#endif

// ---------------------------------------------------------------- configurare
constexpr char AP_SSID[] = "ESP32_Car";
constexpr char AP_PASSWORD[] = "12345678";
constexpr uint8_t AP_CHANNEL = 6;        // 1 / 6 / 11 — alege canalul liber
constexpr uint8_t AP_MAX_CLIENTS = 4;
// 60 s, NU 10: cu 10 s, un tab trecut în fundal (Chrome limitează setInterval
// la 1/s sau mai rău) poate fi dat afară din AP, iar laptopul rămâne
// "conectat" fără să primească nimic.
constexpr uint16_t AP_INACTIVE_SEC = 60; // scoate stațiile moarte din tabelă

// Canal UDP de control — fără conexiune, fără handshake, fără stări de socket.
// Aici nu există "deconectare": un pachet pierdut e urmat de altul la 20 ms.
constexpr uint16_t UDP_PORT = 4210;
constexpr uint32_t UDP_PRIORITY_MS = 2000;  // cât timp UDP are prioritate față de browser

const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// 4 * 0.25 dBm => 60 qdbm = 15 dBm.
// Dacă vezi BROWNOUT în Serial, coboară la 52 (13 dBm) sau 44 (11 dBm):
// puterea de emisie dă vârfuri de curent, iar brownout-ul = AP-ul dispare
// și laptopul "se deconectează".
constexpr int8_t TX_POWER_QDBM = 60;

constexpr uint8_t PIN_IN1 = 10;
constexpr uint8_t PIN_IN2 = 7;
constexpr uint8_t PIN_IN3 = 6;
constexpr uint8_t PIN_IN4 = 5;
constexpr uint8_t PIN_SERVO = 3;
constexpr uint8_t LED_RED = 0;
constexpr uint8_t LED_YELLOW = 1;
constexpr uint8_t LED_GREEN = 2;

constexpr uint32_t COMMAND_TIMEOUT_MS = 1500;   // failsafe
constexpr uint32_t HEARTBEAT_MS = 2000;

// --- auto-vindecare HTTP ---------------------------------------------------
// Dacă un client e ASOCIAT dar nu mai vine nicio cerere HTTP, stack-ul
// WebServer/lwIP e blocat (nu Wi-Fi-ul). Reparăm în doi pași.
// Nu se declanșează când nu e niciun client conectat (laptop închis / plecat).
constexpr bool AUTO_RECOVERY = true;
constexpr uint32_t HTTP_WATCHDOG_MS = 20000;   // 20 s liniște -> refac socket-urile
constexpr uint32_t HTTP_ESCALATE_MS = 60000;   // 60 s liniște -> restart curat

constexpr int SERVO_LEFT = 55;
constexpr int SERVO_CENTER = 90;
constexpr int SERVO_RIGHT = 125;

// --------------------------------------------------------------- direcție
// Viraj PROGRESIV: când apeși stânga/dreapta, servo-ul pleacă de la un unghi
// mic și crește treptat până la maxim cât timp ții apăsat. La eliberare
// revine lin la centru. Așa poți face și corecții fine, și viraje strânse,
// cu aceeași tastă.
constexpr float STEER_MIN_FRAC = 0.16f;   // 16% din cursa maximă = ~6°
constexpr uint32_t STEER_RAMP_MS = 850;   // cât durează până la viraj maxim
constexpr float STEER_RELEASE_PER_S = 4.5f; // revenire la centru în ~0.2 s
// Unghiul maxim față de centru (35° la stânga, 35° la dreapta)
constexpr int STEER_FULL_RANGE_DEG = 35;

// ------------------------------------------------------------------ motoare
constexpr uint32_t MOTOR_PWM_HZ = 20000;   // 20 kHz: inaudibil, driver fericit
constexpr uint8_t MOTOR_PWM_BITS = 10;     // 0..1023

// Cât de repede urcă / coboară puterea. Urcarea lentă = fără vârf de curent.
constexpr float RAMP_UP_PCT_PER_SEC = 220.0f;    // 0 -> 100% în ~0.45 s
constexpr float RAMP_DOWN_PCT_PER_SEC = 700.0f;  // oprire în ~0.15 s

constexpr float MIN_SPEED_PCT = 20.0f;   // sub asta motorul poate sta

// Limită dură de putere. Coborâtă la 70-80 dacă ai brownout sau baterie slabă.
constexpr float MAX_PWM_PCT = 100.0f;
constexpr float DEFAULT_SPEED_PCT = 75.0f;

// Compensare stânga/dreapta dacă o roată trage mai tare (100 = fără corecție).
constexpr float TRIM_LEFT = 100.0f;
constexpr float TRIM_RIGHT = 100.0f;

// ------------------------------------------------------------------- globals
WebServer server(80);
Servo steeringServo;

// --- stare motoare
enum MotorMode : uint8_t { M_COAST = 0, M_FORWARD, M_BACKWARD, M_BRAKE };
MotorMode motorMode = M_COAST;
float speedPct = DEFAULT_SPEED_PCT;   // viteză cerută de utilizator
float currentDuty = 0.0f;             // duty real, rampat
uint32_t lastRampMs = 0;

// --- stare servo / comenzi
int lastServoAngle = SERVO_CENTER;
uint32_t lastCommandAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t commandCount = 0;
bool isFailSafe = true;

// --- direcție progresivă
int8_t steerDir = 0;            // -1 stânga, 0 drept, +1 dreapta
float steerOut = 0.0f;          // -1..+1, poziția reală a servo-ului
float steerOverride = 0.0f;     // 0 = progresiv; 20..100 = viraj fix (%)
uint32_t steerPressMs = 0;      // când a început ținerea tastei
uint32_t lastSteerMs = 0;

// --- diagnostic Wi-Fi
uint32_t stationsConnected = 0;
uint32_t stationsLost = 0;
uint8_t lastDiscReason = 0;
uint32_t lastDiscAtMs = 0;

// --- supraveghere HTTP
uint32_t lastRequestAt = 0;
bool recoveryLevel1Done = false;
uint32_t recoveryCount = 0;

// --- canal UDP
WiFiUDP udp;
IPAddress udpPeer;
uint16_t udpPeerPort = 0;
uint32_t udpLastHeard = 0;
uint32_t udpLastReplyMs = 0;
uint32_t udpPackets = 0;

// ------------------------------------------------------------- PWM compat
// Core 3.x: ledcAttach(pin, hz, bits) — alocă canalul automat, pe pin.
// Core 2.x: canale explicite; servo își ia canalul 0, restul merg pe 2..5.
struct PwmPin {
  uint8_t pin;
  uint8_t ch;
  uint8_t bits;
  int lastVal;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  void begin(uint32_t hz, uint8_t b) {
    bits = b; lastVal = -1;
    ledcAttach(pin, hz, b);
  }
  void writeRaw(uint32_t duty) { ledcWrite(pin, duty); }
#else
  void begin(uint32_t hz, uint8_t b) {
    bits = b; lastVal = -1;
    ledcSetup(ch, hz, b);
    ledcAttachPin(pin, ch);
  }
  void writeRaw(uint32_t duty) { ledcWrite(ch, duty); }
#endif
};

PwmPin pwmIn1{PIN_IN1, 2, MOTOR_PWM_BITS, -1};
PwmPin pwmIn2{PIN_IN2, 3, MOTOR_PWM_BITS, -1};
PwmPin pwmIn3{PIN_IN3, 4, MOTOR_PWM_BITS, -1};
PwmPin pwmIn4{PIN_IN4, 5, MOTOR_PWM_BITS, -1};

const char* motorModeName(MotorMode m) {
  switch (m) {
    case M_FORWARD:  return "forward";
    case M_BACKWARD: return "backward";
    case M_BRAKE:    return "brake";
    default:         return "stop";
  }
}

// percent -> duty, cu limită MAX_PWM_PCT
void writePercent(PwmPin& p, float percent) {
  if (percent < 0) percent = 0;
  if (percent > MAX_PWM_PCT) percent = MAX_PWM_PCT;
  int v = (int)(percent * 10.0f + 0.5f);   // 0..1000, pas de 0.1%
  if (v == p.lastVal) return;              // nu rescrie dacă nu s-a schimbat
  p.lastVal = v;
  uint32_t maxDuty = (1u << p.bits) - 1;
  p.writeRaw((uint32_t)((uint64_t)maxDuty * v / 1000));
}

// ------------------------------------------------------------------ dashboard
static const char DASHBOARD[] PROGMEM = R"HTML(
<!doctype html><html lang="ro"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>ESP32-C3 Car</title><style>
:root{color-scheme:dark;--ink:#eef6ff;--muted:#a7b9d6;--line:#385779;--panel:#111f35;
--go:#16ca8c;--stop:#ef6171;--warn:#f5c451}
*{box-sizing:border-box}body{margin:0;min-height:100vh;font:600 15px system-ui,sans-serif;
color:var(--ink);background:radial-gradient(circle at 50% 0,#234d7e,#07101e 65%)}
main{width:min(1100px,96vw);margin:12px auto}
h1{margin:0;font-size:1.3rem;letter-spacing:.04em}p.sub{margin:2px 0 10px;color:var(--muted);font-size:.78rem}
.status{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-bottom:10px}
.status div{background:#13233b;border:1px solid var(--line);border-radius:10px;padding:7px;
color:var(--muted);font-size:.62rem;letter-spacing:.04em}
.status b{display:block;margin-top:2px;color:var(--ink);font-size:.78rem;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.good{color:var(--go)!important}.warn{color:var(--warn)!important}.bad{color:var(--stop)!important}
.desk{display:grid;grid-template-columns:minmax(320px,1.2fr) minmax(290px,.8fr);gap:10px}
.scene,.controls{background:#0e1b2e;border:1px solid var(--line);border-radius:16px;padding:12px}
.road{position:relative;height:320px;overflow:hidden;border-radius:12px;background:#293448}
.road:before{content:"";position:absolute;inset:-100% 48%;
background:repeating-linear-gradient(0deg,transparent 0 25px,#dce9f6 25px 43px,transparent 43px 70px);opacity:.7}
.moving-forward .road:before{animation:roadForward .6s linear infinite}
.moving-backward .road:before{animation:roadBack .6s linear infinite}
@keyframes roadForward{to{transform:translateY(70px)}}@keyframes roadBack{to{transform:translateY(-70px)}}
.car{position:absolute;left:50%;top:50%;width:136px;height:214px;transform:translate(-50%,-50%);
border-radius:48% 48% 30% 30%;background:linear-gradient(90deg,#2bd3a3,#159172 50%,#2bd3a3);
box-shadow:0 0 0 8px #0b1626,0 12px 24px #0008}
.windshield{position:absolute;left:22px;right:22px;top:31px;height:62px;
border-radius:26px 26px 13px 13px;background:#9ed3ec}
.wheel{position:absolute;width:30px;height:55px;border-radius:15px;
background:repeating-linear-gradient(0deg,#080c12 0 7px,#485260 7px 11px);transition:transform .15s}
.wheel.fl{left:-15px;top:29px}.wheel.fr{right:-15px;top:29px}
.wheel.rl{left:-15px;bottom:29px}.wheel.rr{right:-15px;bottom:29px}
.moving-forward .wheel,.moving-backward .wheel{animation:spin .25s linear infinite}
@keyframes spin{to{background-position-y:18px}}
.steer-left .wheel.fl,.steer-left .wheel.fr{transform:rotate(-25deg)}
.steer-right .wheel.fl,.steer-right .wheel.fr{transform:rotate(25deg)}
.direction{position:absolute;left:50%;top:10px;transform:translateX(-50%);z-index:3;width:max-content;
padding:5px 9px;border-radius:999px;background:#07101ece;font-size:.75rem}
.braking .car{box-shadow:0 0 0 8px #0b1626,0 0 30px var(--stop)}
.controls{display:flex;flex-direction:column;gap:7px}
.controls h2{margin:0 0 1px;font-size:.72rem;color:var(--muted);letter-spacing:.09em}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.controls button{min-height:50px;border:1px solid var(--line);border-radius:11px;background:var(--panel);
color:var(--ink);font:700 .82rem system-ui;touch-action:none;cursor:pointer;user-select:none}
.controls button:hover,.controls button.active,.controls button:focus-visible{border-color:var(--go);background:#15574d;outline:none}
.controls button.brake{background:#432b36}
.controls button.brake:hover,.controls button.brake.active{background:#7d2b3c;border-color:var(--stop)}
.speedbox{background:#13233b;border:1px solid var(--line);border-radius:11px;padding:8px 10px}
.speedbox label{display:flex;justify-content:space-between;font-size:.72rem;color:var(--muted);letter-spacing:.05em}
.speedbox output{color:var(--go);font-weight:800}
input[type=range]{width:100%;margin-top:6px;accent-color:#16ca8c}
.legend{margin-top:auto;color:var(--muted);font-size:.7rem;line-height:1.45}
.legend b{color:var(--ink)}
@media(max-width:720px){.status{grid-template-columns:repeat(2,1fr)}.desk{grid-template-columns:1fr}
.road{height:240px}.scene{order:2}.controls{order:1}}
</style></head><body><main>
<h1>ESP32-C3 CAR</h1><p class="sub">Comenzile pleacă din acest browser către placa ESP32-C3, prin Wi-Fi.</p>
<section class="status">
  <div>LEGĂTURĂ<b id="link" class="warn">se verifică…</b></div>
  <div>CLIENȚI<b id="clients">—</b></div>
  <div>SURSĂ<b id="source">—</b></div>
  <div>COMANDĂ<b id="drive">STOP</b></div>
  <div>DIRECȚIE<b id="steer">DREPT</b></div>
  <div>ULTIMA CĂDERE<b id="drop">—</b></div>
</section>
<div class="desk">
  <section class="scene"><div id="visual" class="visual"><div id="arrow" class="direction">↑ STOP</div>
  <div class="road"><div class="car"><i class="wheel fl"></i><i class="wheel fr"></i>
  <i class="wheel rl"></i><i class="wheel rr"></i><i class="windshield"></i></div></div></div></section>
  <section class="controls">
    <h2>DIRECȚIE</h2>
    <div class="pair">
      <button data-steer="left">◀ A / ←</button>
      <button data-steer="right">D / → ▶</button>
    </div>
    <h2>MOTOARE</h2>
    <div class="pair">
      <button data-drive="forward">▲ ÎNAINTE</button>
      <button data-drive="backward">▼ ÎNAPOI</button>
    </div>
    <h2>VITEZĂ</h2>
    <div class="speedbox">
      <label>PUTERE MOTOR <output id="speedOut">75%</output></label>
      <input id="speed" type="range" min="20" max="100" step="5" value="75">
    </div>
    <h2>VIRAJ</h2>
    <div class="speedbox">
      <label>CÂT DE MULT VIRAJ <output id="steerAmtOut">PROGRESIV</output></label>
      <input id="steerAmt" type="range" min="0" max="100" step="5" value="0">
    </div>
    <h2>SIGURANȚĂ</h2>
    <button class="brake" data-drive="brake">■ FRÂNĂ (Space)</button>
    <div class="legend">
      <b>W</b>/<b>↑</b> înainte · <b>S</b>/<b>↓</b> înapoi · <b>A</b>/<b>←</b> · <b>D</b>/<b>→</b> direcție ·
      <b>Space</b> frână · <b>[</b> / <b>]</b> viteză · <b>1</b> activează · <b>1+2</b> oprire totală.<br>
      Virajul e <b>progresiv</b>: ții apăsat și crește de la mic la maxim.
      Controller PS4: △ activează · □ frână · R2/L2 mișcare · △+✕ oprire totală.<br>
      Comenzile se retrimit la 300 ms; dacă nu mai ajunge nimic 1,5 s, placa frânează singură.<br>
      Dacă serverul se blochează, placa se repară singură (20 s / 60 s).
    </div>
  </section>
</div></main><script>
'use strict';
var SEND_MS = 300;
var el = {
  link: document.getElementById('link'),
  clients: document.getElementById('clients'),
  source: document.getElementById('source'),
  drive: document.getElementById('drive'),
  steer: document.getElementById('steer'),
  drop: document.getElementById('drop'),
  arrow: document.getElementById('arrow'),
  visual: document.getElementById('visual'),
  speed: document.getElementById('speed'),
  speedOut: document.getElementById('speedOut'),
  steerAmt: document.getElementById('steerAmt'),
  steerAmtOut: document.getElementById('steerAmtOut')
};
var RO = { forward:'ÎNAINTE', backward:'ÎNAPOI', brake:'FRÂNĂ', stop:'STOP',
           left:'STÂNGA', right:'DREAPTA', center:'DREPT',
           keyboard:'⌨ TASTATURĂ', controller:'🎮 CONTROLER', none:'—' };
var REASON = { 2:'AUTH_EXPIRE', 3:'ASSOC_EXPIRE', 8:'laptop a plecat',
               15:'handshake slab', 200:'rază / timeout', 204:'handshake timeout',
               206:'AP_TSF_RESET', 208:'assoc comeback', 209:'SA_QUERY (PMF)' };

var command = { drive:'stop', steer:'center' };
var speed = 75;
var steerAmt = 0;   // 0 = progresiv (crește cât ții apăsat), 20..100 = fix
var keys = new Set();
var activeSource = 'none';
var lastKey = '', lastAt = 0, failStreak = 0, links = 0;

// Viteza minimă între două cereri forțate. Fără ea, două taste apăsate în
// același timp produceau două cereri în aceeași milisecundă, iar ESP32 are
// o singură conexiune HTTP activă — al doilea socket rămânea orfan.
var MIN_GAP_MS = 90;
var inflight = false, pending = null;
var heldButton = null;

function describeTrajectory(s){
  if (s.drive === 'forward') return s.steer === 'left' ? '↖ ÎNAINTE STÂNGA'
    : s.steer === 'right' ? '↗ ÎNAINTE DREAPTA' : '↑ ÎNAINTE';
  if (s.drive === 'backward') return s.steer === 'left' ? '↘ ÎNAPOI DREAPTA'
    : s.steer === 'right' ? '↙ ÎNAPOI STÂNGA' : '↓ ÎNAPOI';
  return s.drive === 'brake' ? '■ FRÂNĂ' : '↑ STOP';
}

function paint(){
  el.drive.textContent = RO[command.drive] || 'STOP';
  el.steer.textContent = RO[command.steer] || 'DREPT';
  el.source.textContent = RO[activeSource] || '—';
  el.source.className = activeSource === 'none' ? 'bad' : 'good';
  el.arrow.textContent = describeTrajectory(command) + ' · ' + Math.round(speed) + '%';
  var cls = 'visual';
  if (command.drive === 'forward') cls += ' moving-forward';
  else if (command.drive === 'backward') cls += ' moving-backward';
  else if (command.drive === 'brake') cls += ' braking';
  if (command.steer === 'left') cls += ' steer-left';
  if (command.steer === 'right') cls += ' steer-right';
  el.visual.className = cls;
  el.speedOut.textContent = Math.round(speed) + '%';
  el.steerAmtOut.textContent = steerAmt > 0 ? (Math.round(steerAmt) + '%') : 'PROGRESIV';
  el.steerAmtOut.style.color = steerAmt > 0 ? 'var(--warn)' : 'var(--go)';
  document.querySelectorAll('.controls button').forEach(function(b){
    var on = (b.dataset.drive && b.dataset.drive === command.drive) ||
             (b.dataset.steer && b.dataset.steer === command.steer);
    b.classList.toggle('active', !!on);
  });
}

// O singură cerere în zbor. NU folosim AbortController: anularea unei cereri
// lasă serverul ESP32 în HC_WAIT_READ/CLOSE până la 2 s per incident, iar la
// ritmul de mai multe cereri pe secundă serverul nu mai iese niciodată din
// starea aceea. În schimb: dacă o cerere e blocată > 4 s, eliberăm slotul
// local și trimitem din nou; răspunsul vechi e ignorat prin numărul de ordine.
var seqNo = 0, inflightSince = 0, releasedCount = 0;

function pump(){
  if (inflight){
    if ((Date.now() - inflightSince) < 4000) return;
    inflight = false;
    releasedCount++;
  }
  if (!pending) return;
  var c = pending; pending = null;
  var my = ++seqNo;
  inflight = true; inflightSince = Date.now();
  fetch('/api/control?drive=' + c.drive + '&steer=' + c.steer +
        '&speed=' + Math.round(c.speed) + '&steerAmt=' + Math.round(c.steerAmt),
        { cache:'no-store' })
    .then(function(r){ if (!r.ok) throw new Error('http'); return r.json(); })
    .then(function(j){
      if (my !== seqNo) return;          // răspuns vechi, ignorat
      failStreak = 0; links++;
      el.link.textContent = 'OK · ' + links;
      el.link.className = 'good';
      el.clients.textContent = j.stations + ' / ' + j.connects + ' total';
      el.clients.className = j.stations > 0 ? 'good' : 'bad';
      showDrop(j.reason);
    })
    .catch(function(){
      if (my !== seqNo) return;
      if (++failStreak >= 2){
        el.link.textContent = 'FĂRĂ RĂSPUNS';
        el.link.className = 'bad';
      }
      // ~3 s fără niciun răspuns: pagina se reîncarcă singură, ca să pornim
      // cu o conexiuni curate (nu rămâne în starea blocată).
      if (failStreak === 8){
        setTimeout(function(){ if (failStreak >= 8) location.reload(); }, 1200);
      }
    })
    .then(function(){
      if (my === seqNo) inflight = false;
      pump();
    });
}

function send(force){
  var key = command.drive + '|' + command.steer + '|' + Math.round(speed) + '|' + Math.round(steerAmt);
  var now = Date.now();
  if (key !== lastKey){
    // schimbare reală de comandă: pleacă imediat, fără să aștepte nimic
    paint();
    lastKey = key; lastAt = now;
    pending = { drive: command.drive, steer: command.steer, speed: speed, steerAmt: steerAmt };
    pump();
    return;
  }
  // aceeași comandă: fie heartbeat (300 ms), fie reîmprospătare forțată,
  // dar niciodată mai des de MIN_GAP_MS
  var gap = force ? MIN_GAP_MS : SEND_MS;
  if ((now - lastAt) < gap) return;
  lastAt = now;
  pending = { drive: command.drive, steer: command.steer, speed: speed, steerAmt: steerAmt };
  pump();
}

// --- tastatură: ordinea apăsării, ULTIMA tastă câștigă -------------------
// Ții A, apeși D  -> merge dreapta. Ridici D -> revine pe stânga (A e încă
// jos). La fel pentru înainte/înapoi și frână.
var pressOrder = [];

function pressAction(a){
  var i = pressOrder.indexOf(a);
  if (i !== -1) pressOrder.splice(i, 1);  // re-apăsare = urcă în vârf
  pressOrder.push(a);
}
function releaseAction(a){
  var i = pressOrder.indexOf(a);
  if (i !== -1) pressOrder.splice(i, 1);
}
function lastPressed(list){
  for (var i = pressOrder.length - 1; i >= 0; i--){
    if (list.indexOf(pressOrder[i]) !== -1) return pressOrder[i];
  }
  return null;
}

function readKeyboard(){
  var d  = lastPressed(['brake', 'forward', 'backward']);
  var st = lastPressed(['left', 'right']);
  return { drive: d || 'stop', steer: st || 'center' };
}

function padButton(g, n){ return !!(g.buttons[n] && g.buttons[n].pressed); }

function readPad(g){
  var s = { drive:'stop', steer:'center' };
  if (padButton(g, 2)) s.drive = 'brake';
  else if (padButton(g, 6)) s.drive = 'forward';
  else if (padButton(g, 7)) s.drive = 'backward';
  var ax = g.axes[0] || 0;
  if (padButton(g, 14) || ax < -0.4) s.steer = 'left';
  else if (padButton(g, 15) || ax > 0.4) s.steer = 'right';
  return s;
}

function padHasInput(g){
  return padButton(g, 2) || padButton(g, 6) || padButton(g, 7) ||
         padButton(g, 14) || padButton(g, 15) || Math.abs(g.axes[0] || 0) > 0.4;
}

var padTriangleWas = false, padChordWas = false;

function stopEverything(){
  keys.clear();
  pressOrder.length = 0;
  heldButton = null;
  activeSource = 'none';
  command = { drive:'stop', steer:'center' };
  lastKey = ''; send(true);
}

function tick(){
  var pads = navigator.getGamepads ? navigator.getGamepads() : [];
  var g = null;
  for (var i = 0; i < pads.length; i++){ if (pads[i]) { g = pads[i]; break; } }

  if (g){
    var tri = padButton(g, 3), cross = padButton(g, 0), chord = tri && cross;
    if (chord && !padChordWas) stopEverything();
    else if (tri && !padTriangleWas && activeSource !== 'controller'){
      activeSource = 'controller'; keys.clear(); pressOrder.length = 0;
    }
    padChordWas = chord; padTriangleWas = tri;
    if (activeSource !== 'controller' && padHasInput(g)){
      activeSource = 'controller'; keys.clear(); pressOrder.length = 0;
    }
    if (activeSource === 'controller'){
      command = readPad(g);
      send(false);
      requestAnimationFrame(tick);
      return;
    }
  }

  if (heldButton){
    command = heldButton;
    send(false);
    requestAnimationFrame(tick);
    return;
  }

  if (activeSource === 'keyboard' || activeSource === 'none'){
    var s = readKeyboard();
    if (s.drive !== 'stop' || s.steer !== 'center'){
      activeSource = 'keyboard';
      command = s;
    } else if (activeSource === 'keyboard'){
      command = s;
    }
    send(false);
  }
  requestAnimationFrame(tick);
}

// ---- tastatură
var KEYMAP = {
  KeyW:'forward', ArrowUp:'forward',
  KeyS:'backward', ArrowDown:'backward',
  KeyA:'left', ArrowLeft:'left',
  KeyD:'right', ArrowRight:'right',
  Space:'brake'
};
var oneHeld = false, twoHeld = false;

window.addEventListener('keydown', function(e){
  if (e.code === 'Digit1'){
    e.preventDefault();
    oneHeld = true;
    if (twoHeld) stopEverything();
    else { activeSource = 'keyboard'; keys.clear(); paint(); }
    return;
  }
  if (e.code === 'Digit2'){ twoHeld = true; if (oneHeld) stopEverything(); return; }
  if (e.code === 'BracketLeft' || e.code === 'BracketRight'){
    e.preventDefault();
    speed = Math.max(20, Math.min(100, speed + (e.code === 'BracketLeft' ? -5 : 5)));
    el.speed.value = speed;
    send(true);
    return;
  }
  var a = KEYMAP[e.code];
  if (!a) return;
  e.preventDefault();
  activeSource = 'keyboard';
  heldButton = null;
  pressAction(a);
  command = readKeyboard();
  if (!e.repeat) send(true);   // o singură cerere per apăsare reală
  else send(false);
});

window.addEventListener('keyup', function(e){
  if (e.code === 'Digit1') oneHeld = false;
  if (e.code === 'Digit2') twoHeld = false;
  var a = KEYMAP[e.code];
  if (!a) return;
  e.preventDefault();
  releaseAction(a);
  command = readKeyboard();
  send(true);
});

// ---- slider viteză
el.speed.addEventListener('input', function(){
  speed = parseFloat(el.speed.value) || 75;
  paint();
  send(true);
});

// ---- slider viraj (0 = progresiv)
el.steerAmt.addEventListener('input', function(){
  steerAmt = parseFloat(el.steerAmt.value) || 0;
  paint();
  send(true);
});

// ---- butoane touch / mouse
document.querySelectorAll('.controls button').forEach(function(b){
  var pressedDrive = b.dataset.drive || null;
  var pressedSteer = b.dataset.steer || null;

  b.addEventListener('pointerdown', function(e){
    e.preventDefault();
    if (b.setPointerCapture) { try { b.setPointerCapture(e.pointerId); } catch (err) {} }
    activeSource = 'keyboard';
    keys.clear();
    pressOrder.length = 0;
    heldButton = { drive: pressedDrive || 'stop', steer: pressedSteer || 'center' };
    command = heldButton;
    send(true);
  });

  function release(e){
    if (e && e.type === 'pointerleave' && b.hasPointerCapture &&
        b.hasPointerCapture(e.pointerId)) return;
    heldButton = null;
    command = { drive:'stop', steer:'center' };
    send(true);
  }
  ['pointerup','pointercancel','pointerleave'].forEach(function(ev){
    b.addEventListener(ev, release);
  });
});

function releaseAll(){
  keys.clear();
  pressOrder.length = 0;
  heldButton = null;
  if (activeSource !== 'none'){
    command = { drive:'stop', steer:'center' };
    send(true);
  }
}
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', function(){ if (document.hidden) releaseAll(); });

// ---- heartbeat + diagnostic
// Un singur flux de cereri: statusul vine în răspunsul de la /api/control,
// deci nu mai facem polling separat pe /api/status (a doua conexiune
// paralelă omora serverul).
function showDrop(code){
  if (!code) { el.drop.textContent = '—'; el.drop.className = 'good'; return; }
  el.drop.textContent = code + ' · ' + (REASON[code] || 'necunoscut');
  el.drop.className = (code === 8) ? 'warn' : 'bad';
}

// Ține legătura vie ȘI cu tab-ul în fundal. Dacă nu trimitem nimic, AP-ul
// poate considera stația inactivă și o scoate — apoi "totul se blochează".
// Comanda trimisă e oricum STOP în fundal, deci mașina nu se mișcă.
setInterval(function(){ send(false); }, 300);

// Senzor de blocare: dacă am trimis și nimic nu vine în 5 s, arată clar.
setInterval(function(){
  if (inflight && (Date.now() - inflightSince) > 5000){
    el.link.textContent = 'BLOCAT · ' + releasedCount;
    el.link.className = 'bad';
  }
}, 1000);

paint();
tick();
</script></body></html>
)HTML";

// ------------------------------------------------------------------- motoare
// Frână electronică pe DRV8833: ambele intrări HIGH = înfășurările sunt
// scurtcircuitate, motorul se oprește rapid, fără consum din sursă.
void applyBrake() {
  writePercent(pwmIn1, 100);
  writePercent(pwmIn2, 100);
  writePercent(pwmIn3, 100);
  writePercent(pwmIn4, 100);
}

// Coast: ambele LOW = motorul se învârte liber.
void applyCoast() {
  writePercent(pwmIn1, 0);
  writePercent(pwmIn2, 0);
  writePercent(pwmIn3, 0);
  writePercent(pwmIn4, 0);
}

void applyDrive() {
  switch (motorMode) {
    case M_FORWARD:
      writePercent(pwmIn1, currentDuty * TRIM_LEFT / 100.0f);
      writePercent(pwmIn2, 0);
      writePercent(pwmIn3, currentDuty * TRIM_RIGHT / 100.0f);
      writePercent(pwmIn4, 0);
      digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, LOW);
      break;

    case M_BACKWARD:
      writePercent(pwmIn1, 0);
      writePercent(pwmIn2, currentDuty * TRIM_LEFT / 100.0f);
      writePercent(pwmIn3, 0);
      writePercent(pwmIn4, currentDuty * TRIM_RIGHT / 100.0f);
      digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, LOW);
      break;

    case M_BRAKE:
      applyBrake();
      digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, HIGH);
      break;

    default:  // M_COAST
      applyCoast();
      digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED, HIGH);
      break;
  }
}

// Rampa: duty-ul real urmărește ținta cu viteză limitată. Asta e ce salvează
// alimentarea (și deci Wi-Fi-ul) la pornirea motoarelor.
void updateRamp() {
  uint32_t now = millis();
  float dt = (now - lastRampMs) / 1000.0f;
  lastRampMs = now;
  if (dt <= 0) return;
  if (dt > 0.1f) dt = 0.1f;   // protecție la primul apel / pauze

  float target = (motorMode == M_FORWARD || motorMode == M_BACKWARD) ? speedPct : 0.0f;
  float rate = (target > currentDuty) ? RAMP_UP_PCT_PER_SEC : RAMP_DOWN_PCT_PER_SEC;
  float step = rate * dt;

  if (currentDuty < target) {
    currentDuty += step;
    if (currentDuty > target) currentDuty = target;
  } else if (currentDuty > target) {
    currentDuty -= step;
    if (currentDuty < target) currentDuty = target;
  }
  applyDrive();
}

void steerTo(int angle) {
  if (angle == lastServoAngle) return;   // nu rescrie servo la fiecare cerere
  lastServoAngle = angle;
  steeringServo.write(angle);
}

// ------------------------------------------------------------------- handleri
// Aplică o comandă venită din ORICE sursă (HTTP sau UDP). Un singur loc
// decide starea motoarelor, ca cele două canale să nu se contrazică.
void applyCommand(char d, char s, uint8_t pct) {
  switch (d) {
    case 'F': motorMode = M_FORWARD;  break;
    case 'B': motorMode = M_BACKWARD; break;
    case 'R': motorMode = M_BRAKE; currentDuty = 0; break;
    default:  motorMode = M_COAST;  break;   // 'X'
  }

  int8_t want = (s == 'L') ? -1 : (s == 'R') ? 1 : 0;
  if (want != steerDir) {
    // Pornim imediat cu un viraj MIC, ca să nu smucească mașina.
    // (dacă trecem direct stânga<->dreapta, lăsăm rampa să treacă prin 0)
    if (steerDir == 0 && want != 0) {
      float k = (steerOverride > 0.0f) ? (steerOverride / 100.0f) : STEER_MIN_FRAC;
      steerOut = want * k;
    }
    steerDir = want;
    steerPressMs = millis();
  }

  if (pct < (uint8_t)MIN_SPEED_PCT) pct = (uint8_t)MIN_SPEED_PCT;
  if (pct > (uint8_t)MAX_PWM_PCT)   pct = (uint8_t)MAX_PWM_PCT;
  speedPct = (float)pct;

  lastCommandAt = millis();
  commandCount++;
  isFailSafe = false;
}

// Mișcă servo-ul spre țintă cu viteză limitată:
//  - ținut apăsat -> de la STEER_MIN_FRAC până la 1.0 în STEER_RAMP_MS
//  - eliberat      -> revine la 0 în ~0.2 s
void updateSteer() {
  uint32_t now = millis();
  float dt = (now - lastSteerMs) / 1000.0f;
  lastSteerMs = now;
  if (dt <= 0.0f) return;
  if (dt > 0.1f) dt = 0.1f;

  float target = 0.0f;
  if (steerDir != 0) {
    if (steerOverride > 0.0f) {
      target = steerDir * (steerOverride / 100.0f);
    } else {
      float held = (float)(now - steerPressMs) / (float)STEER_RAMP_MS;
      if (held > 1.0f) held = 1.0f;
      target = steerDir * (STEER_MIN_FRAC + (1.0f - STEER_MIN_FRAC) * held);
    }
  }

  float rate;
  if (fabsf(target) > fabsf(steerOut)) {
    // creștere: exact ritmul rampei (dacă nu e override, ținta crește singură)
    rate = (1.0f - STEER_MIN_FRAC) / ((float)STEER_RAMP_MS / 1000.0f);
  } else {
    rate = STEER_RELEASE_PER_S;   // reducere / revenire la centru
  }
  float step = rate * dt;

  if (steerOut < target) {
    steerOut += step;
    if (steerOut > target) steerOut = target;
  } else if (steerOut > target) {
    steerOut -= step;
    if (steerOut < target) steerOut = target;
  }

  int angle = SERVO_CENTER + (int)lroundf(steerOut * (float)STEER_FULL_RANGE_DEG);
  if (angle < SERVO_LEFT)  angle = SERVO_LEFT;
  if (angle > SERVO_RIGHT) angle = SERVO_RIGHT;
  steerTo(angle);
}

void buildStatusJson(char* buf, size_t n) {
  snprintf(buf, n,
           "{\"ok\":true,\"drive\":\"%s\",\"speed\":%u,\"duty\":%u,"
           "\"uptime\":%lu,\"heap\":%lu,\"stations\":%u,"
           "\"connects\":%lu,\"drops\":%lu,\"reason\":%u,\"recoveries\":%lu,"
           "\"udp\":%lu,\"failsafe\":%s,\"cmds\":%lu}",
           motorModeName(motorMode), (unsigned)(speedPct + 0.5f),
           (unsigned)(currentDuty + 0.5f),
           (unsigned long)(millis() / 1000UL),
           (unsigned long)ESP.getFreeHeap(),
           (unsigned)WiFi.softAPgetStationNum(),
           (unsigned long)stationsConnected,
           (unsigned long)stationsLost,
           (unsigned)lastDiscReason,
           (unsigned long)recoveryCount,
           (unsigned long)udpPackets,
           isFailSafe ? "true" : "false",
           (unsigned long)commandCount);
}

void sendStatusJson() {
  char buf[360];
  buildStatusJson(buf, sizeof(buf));
  server.client().setNoDelay(true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleControl() {
  // Dacă aplicația UDP vorbește, comenzile din browser sunt ignorate — altfel
  // tab-ul deschis în fundal trimite STOP la 300 ms și oprește mașina.
  if (udpPeerPort && (millis() - udpLastHeard) < UDP_PRIORITY_MS) {
    sendStatusJson();
    return;
  }

  char d = 0, s = 0;
  String drive = server.hasArg("drive") ? server.arg("drive") : String("");
  String steer = server.hasArg("steer") ? server.arg("steer") : String("");

  if (drive == "forward")       d = 'F';
  else if (drive == "backward") d = 'B';
  else if (drive == "brake")    d = 'R';
  else if (drive == "stop")     d = 'X';

  if (steer == "left")        s = 'L';
  else if (steer == "right")  s = 'R';
  else if (steer == "center") s = 'C';

  if (!d && !s) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  uint8_t pct = (uint8_t)(speedPct + 0.5f);
  if (server.hasArg("speed")) {
    long v = server.arg("speed").toInt();
    if (v < (long)MIN_SPEED_PCT) v = (long)MIN_SPEED_PCT;
    if (v > (long)MAX_PWM_PCT)   v = (long)MAX_PWM_PCT;
    pct = (uint8_t)v;
  }

  // 0 sau absent = viraj progresiv (cât ții apăsat); 20..100 = viraj fix
  if (server.hasArg("steerAmt")) {
    long sa = server.arg("steerAmt").toInt();
    if (sa < 0)   sa = 0;
    if (sa > 100) sa = 100;
    steerOverride = (float)sa;
  }

  lastRequestAt = millis();
  recoveryLevel1Done = false;
  applyCommand(d ? d : 'X', s ? s : 'C', pct);
  sendStatusJson();
}

void handleStatus() {
  lastRequestAt = millis();
  recoveryLevel1Done = false;
  sendStatusJson();
}

void handleRoot() {
  lastRequestAt = millis();
  recoveryLevel1Done = false;
  server.client().setNoDelay(true);
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", DASHBOARD);
}

// ------------------------------------------------------------- evenimente Wi-Fi
const char* reasonText(uint8_t r) {
  switch (r) {
    case 2:   return "AUTH_EXPIRE";
    case 3:   return "ASSOC_EXPIRE";
    case 8:   return "ASSOC_LEAVE (laptopul a plecat pe alt SSID)";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT (semnal slab / parola)";
    case 200: return "BEACON_TIMEOUT (afara din raza sau client adormit)";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    case 206: return "AP_TSF_RESET";
    case 208: return "ASSOC_COMEBACK_TIME_TOO_LONG";
    case 209: return "SA_QUERY_TIMEOUT (AP a pierdut confirmarea clientului)";
    default:  return "alt motiv";
  }
}

void onStaConnected() {
  stationsConnected++;
  Serial.printf("[WiFi] STA conectat. Total sesiune: %lu, clienți acum: %u\n",
                (unsigned long)stationsConnected,
                (unsigned)WiFi.softAPgetStationNum());
}

void onStaDisconnected(uint8_t reason) {
  stationsLost++;
  lastDiscReason = reason;
  lastDiscAtMs = millis();
  Serial.printf("[WiFi] STA plecat: reason=%u (%s), clienți rămași: %u\n",
                (unsigned)reason, reasonText(reason),
                (unsigned)WiFi.softAPgetStationNum());
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void cbStaConnected(WiFiEvent_t e, WiFiEventInfo_t info) { (void)e; (void)info; onStaConnected(); }
void cbStaDisconnected(WiFiEvent_t e, WiFiEventInfo_t info) {
  (void)e;
  onStaDisconnected(info.wifi_ap_stadisconnected.reason);
}
#else
void cbStaConnected(system_event_id_t e, system_event_info_t info) { (void)e; (void)info; onStaConnected(); }
void cbStaDisconnected(system_event_id_t e, system_event_info_t info) {
  (void)e;
  onStaDisconnected(info.disconnected.reason);
}
#endif

// ------------------------------------------------------- canal UDP de control
// Protocol (ASCII, ca să-l poți testa și cu netcat):
//   "CFL75"  -> C = command, F=forward / B=backward / R=brake / X=stop,
//               L/C/R = direcția, 75 = putere în procente (două cifre)
//   "P"      -> ping, placa răspunde cu o linie de telemetrie
// Răspuns (≈10 Hz): "S,<uptime>,<duty>,<mod>,<clienți>,<heap>"
//
// De ce UDP: nu are conexiune, deci nu are ce să se "deconecteze". Nu există
// handshake, ACK, TIME_WAIT, coadă de accept sau socket de server care să se
// blocheze. Un pachet pierdut e urmat de altul 50 ms mai târziu.
void replyUdpStatus() {
  if (!udpPeerPort) return;
  char buf[72];
  snprintf(buf, sizeof(buf), "S,%lu,%u,%s,%u,%lu\n",
           (unsigned long)(millis() / 1000UL),
           (unsigned)(currentDuty + 0.5f),
           motorModeName(motorMode),
           (unsigned)WiFi.softAPgetStationNum(),
           (unsigned long)ESP.getFreeHeap());
  udp.beginPacket(udpPeer, udpPeerPort);
  udp.write((const uint8_t*)buf, strlen(buf));
  udp.endPacket();
}

void handleUdp() {
  int size = udp.parsePacket();
  if (size <= 0) return;

  char buf[32];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = 0;

  // reținem expeditorul ca să putem trimite telemetria înapoi
  udpPeer = udp.remoteIP();
  udpPeerPort = udp.remotePort();
  udpLastHeard = millis();
  udpPackets++;

  if (buf[0] == 'P') { replyUdpStatus(); return; }

  // "C" + drive + steer + 2 cifre de putere [+ 2 cifre viraj fix, opțional]
  if (buf[0] != 'C' || n < 5) return;

  uint8_t pct = 0;
  if (buf[3] >= '0' && buf[3] <= '9') pct = (uint8_t)((buf[3] - '0') * 10);
  if (buf[4] >= '0' && buf[4] <= '9') pct = (uint8_t)(pct + (buf[4] - '0'));

  // opțional "CFL7550" -> viraj fix la 50%; absent -> viraj progresiv
  if (n >= 7 && buf[5] >= '0' && buf[5] <= '9' && buf[6] >= '0' && buf[6] <= '9') {
    uint8_t sa = (uint8_t)((buf[5] - '0') * 10 + (buf[6] - '0'));
    if (sa > 100) sa = 100;
    steerOverride = (float)sa;
  }

  applyCommand(buf[1], buf[2], pct);

  // telemetrie la ~10 Hz, ca să nu umplem canalul
  uint32_t now = millis();
  if (now - udpLastReplyMs >= 100) {
    udpLastReplyMs = now;
    replyUdpStatus();
  }
}

// ----------------------------------------------------------------------- misc
const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON (alimentare pornită)";
    case ESP_RST_EXT:      return "EXT (pin de reset)";
    case ESP_RST_SW:       return "SW (restart software)";
    case ESP_RST_PANIC:    return "PANIC (excepție / crash)";
    case ESP_RST_INT_WDT:  return "INT_WDT (watchdog întreruperi)";
    case ESP_RST_TASK_WDT: return "TASK_WDT (loop blocat)";
    case ESP_RST_WDT:      return "WDT (watchdog)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (tensiune prea mică!)";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    default:               return "necunoscut";
  }
}

void hardStop() {
  motorMode = M_BRAKE;      // frână, nu coast: mașina nu trebuie să ruleze
  currentDuty = 0;
  applyBrake();
  steerDir = 0;
  steerOut = 0.0f;
  lastServoAngle = SERVO_CENTER;
  steeringServo.write(SERVO_CENTER);
  isFailSafe = true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32-C3 masina Wi-Fi v3 ===");
  Serial.printf("Core: %d.x\n", (int)ESP_ARDUINO_VERSION_MAJOR);
  Serial.printf("Ultimul reset: %s\n", resetReasonText(esp_reset_reason()));

  pinMode(LED_RED, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_YELLOW, LOW);

  // Servo ÎNTÂI (pe core 2.x își ia canalul 0, ca să nu se bată cu motoarele).
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(PIN_SERVO, 500, 2400);
  steeringServo.write(SERVO_CENTER);

  // Motoare pe PWM, 20 kHz.
  pwmIn1.begin(MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  pwmIn2.begin(MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  pwmIn3.begin(MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  pwmIn4.begin(MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  motorMode = M_BRAKE;
  applyBrake();

  // --- Wi-Fi AP
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  // Evenimente ÎNAINTE de a porni AP-ul, altfel pierzi primele conectări.
  WiFi.onEvent(cbStaConnected,    EV_AP_STACONN);
  WiFi.onEvent(cbStaDisconnected, EV_AP_STADISC);

  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, AP_MAX_CLIENTS)) {
    Serial.println("softAP a eșuat!");
  }

  // PMF (802.11w) oprit explicit pe AP, ÎNAINTE de restul setărilor de radio
  // (esp_wifi_set_config repornește AP-ul și ar reseta puterea TX).
  // Cu PMF capabil/required, AP-ul trimite SA Query ca să confirme stația;
  // dacă laptopul nu răspunde la timp, AP-ul îl scoate cu reason=209
  // (SA_QUERY_TIMEOUT) — exact ce ai văzut. Pe un AP local nu avem nevoie.
  {
    wifi_config_t cfg = {};
    if (esp_wifi_get_config(WIFI_IF_AP, &cfg) == ESP_OK) {
      cfg.ap.pmf_cfg.required = false;
      cfg.ap.pmf_cfg.capable  = false;
      esp_err_t p = esp_wifi_set_config(WIFI_IF_AP, &cfg);
      if (p != ESP_OK) Serial.printf("pmf_cfg: %d\n", (int)p);
      else Serial.println("PMF off (fara SA Query -> fara reason 209)");
    }
  }

  // Ordinea contează: setările de radio se aplică DUPĂ ce AP-ul e pornit.
  esp_err_t err;

  err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (err != ESP_OK) Serial.printf("set_ps: %d\n", (int)err);

  err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  if (err != ESP_OK) Serial.printf("set_bandwidth: %d\n", (int)err);

  err = esp_wifi_set_max_tx_power(TX_POWER_QDBM);
  if (err != ESP_OK) Serial.printf("set_tx_power: %d\n", (int)err);

  // Cheia stabilității: scoate stațiile moarte din tabela AP.
  // Fără asta, după câteva reconectări ale laptopului rămâi fără sloturi
  // și AP-ul refuză orice client nou.
  err = esp_wifi_set_inactive_time(WIFI_IF_AP, AP_INACTIVE_SEC);
  if (err != ESP_OK) Serial.printf("set_inactive_time: %d\n", (int)err);

  int8_t txRead = 0;
  esp_wifi_get_max_tx_power(&txRead);
  Serial.printf("AP: %s / %s  canal %u  TX %d dBm (cerut %d)\n",
                AP_SSID, AP_PASSWORD, AP_CHANNEL,
                (int)(txRead / 4), (int)(TX_POWER_QDBM / 4));
  Serial.printf("Deschide http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("Inactive timeout: %u s, max clienți: %u\n",
                (unsigned)AP_INACTIVE_SEC, (unsigned)AP_MAX_CLIENTS);
  Serial.printf("Motor PWM: %lu Hz, %u biți, viteză implicită %d%%\n",
                (unsigned long)MOTOR_PWM_HZ, (unsigned)MOTOR_PWM_BITS,
                (int)DEFAULT_SPEED_PCT);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/control", HTTP_GET, handleControl);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();

  udp.begin(UDP_PORT);
  Serial.printf("UDP control pe portul %u (aplicația drive_app.py)\n", UDP_PORT);

  lastCommandAt = millis();
  lastHeartbeatAt = millis();
  lastRampMs = millis();
  lastRequestAt = millis();
  lastRampMs = millis();
  lastSteerMs = millis();
  currentDuty = 0;
  speedPct = DEFAULT_SPEED_PCT;
  steerOut = 0.0f;
  steerDir = 0;
  steerOverride = 0.0f;
}

void loop() {
  handleUdp();          // canalul de control principal (fără conexiune)
  server.handleClient();

  // 1) niciun client conectat -> mașina nu are voie să meargă singură
  if (!isFailSafe && WiFi.softAPgetStationNum() == 0) {
    Serial.println("FAILSAFE: niciun client conectat -> frână");
    hardStop();
  }

  // 2) failsafe: nu a mai venit nicio comandă
  if (!isFailSafe && (millis() - lastCommandAt) > COMMAND_TIMEOUT_MS) {
    Serial.println("FAILSAFE: timeout comenzi -> frână");
    hardStop();
  }

  // 3) rampa de putere (soft-start / soft-stop)
  updateRamp();

  // 3a) viraj progresiv: servo-ul urmărește ținta cu viteză limitată
  updateSteer();

  // 3b) auto-vindecare dacă serverul HTTP se blochează (client asociat, dar
  //     nu mai vine nicio cerere). NU se aplică dacă nu e nimeni conectat.
  if (AUTO_RECOVERY && WiFi.softAPgetStationNum() > 0) {
    uint32_t silent = millis() - lastRequestAt;
    if (!recoveryLevel1Done && silent > HTTP_WATCHDOG_MS) {
      recoveryLevel1Done = true;
      recoveryCount++;
      Serial.println("RECOVERY L1: HTTP blocat -> reiau socket-urile de listen");
      server.stop();
      delay(20);
      server.begin();
    }
    if (silent > HTTP_ESCALATE_MS) {
      Serial.println("RECOVERY L2: tot blocat -> restart ESP32");
      Serial.flush();
      ESP.restart();
    }
  } else if (recoveryLevel1Done) {
    recoveryLevel1Done = false;
  }

  // 4) heartbeat de diagnoză
  if (millis() - lastHeartbeatAt >= HEARTBEAT_MS) {
    lastHeartbeatAt = millis();
    Serial.printf("uptime=%lus heap=%lu clienți=%u con=%lu drop=%lu cmd=%lu "
                  "viteza=%d%% duty=%d%% %s\n",
                  (unsigned long)(millis() / 1000UL),
                  (unsigned long)ESP.getFreeHeap(),
                  (unsigned)WiFi.softAPgetStationNum(),
                  (unsigned long)stationsConnected,
                  (unsigned long)stationsLost,
                  (unsigned long)commandCount,
                  (int)(speedPct + 0.5f),
                  (int)(currentDuty + 0.5f),
                  isFailSafe ? "STOP" : "ACTIV");
  }

  delay(1);   // lasă stack-ul Wi-Fi/TCP să ruleze
}
