/**
 * FYSETC E4 v1.3 - WiFi AP + Web Interface con Gradi (+1, +10, +90...)
 * + StallGuard homing su asse Y (Altitude) con offset/limiti configurabili
 *   e salvataggio in NVS (Preferences).
 *
 * --- ISTRUZIONI WIFI ---
 * Nome Rete: APA
 * Password:  Apa1234567
 * Indirizzo: 192.168.4.1
 *   - Web UI:            http://192.168.4.1   (porta 80)
 *   - Canale comandi:    TCP 192.168.4.1:23   (stesso protocollo della seriale USB,
 *                        usato da platedual.py in modalita' "WiFi")
 *
 * --- COMANDI ESTESI (prefisso ":") ---
 *  :HOMEY         -> Avvia homing Y (StallGuard + offset)
 *  :OFFY <n>      -> Imposta offset (step) tra punto di stallo e home
 *  :MINY <n>      -> Limite minimo Y (step, relativo a home)
 *  :MAXY <n>      -> Limite massimo Y (step, relativo a home)
 *  :SGY  <0-255>  -> Sensibilita StallGuard (piu alto = piu sensibile)
 *  :DIRY <-1|1>   -> Direzione di homing
 *  :HSPY <step/s> -> Velocita durante l'homing
 *  :SPDX <step/s> -> Velocita massima asse X (movimenti normali)
 *  :SPDY <step/s> -> Velocita massima asse Y (movimenti normali)
 *  :ACCX <step/s2>-> Accelerazione asse X
 *  :ACCY <step/s2>-> Accelerazione asse Y
 *  :SAVE          -> Salva tutta la config in NVS
 *  :STATUS        -> Restituisce stato corrente (pos, busy, limiti, SG_RESULT, DIAG...)
 *  :STOP          -> Stop immediato di entrambi gli assi
 *  :RESETY        -> Imposta posizione Y a 0 (home manuale senza stall)
 *  :DREAD         -> Diagnostica: legge pin DIAG_Y e registro SG_RESULT
 *  :SGTEST [n]    -> Test SG: muove Y di n step (default 4000 nella dir di homing)
 *                    e riporta range di SG_RESULT + se DIAG e' mai andato HIGH.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#define ENABLE_PIN 25

// --- INDIRIZZI TMC2209 ---
#define X_ADDR      1
#define Y_ADDR      3

// --- PINOUT ---
#define X_STEP_PIN 27
#define X_DIR_PIN  26
#define Y_STEP_PIN 33
#define Y_DIR_PIN  32

// StallGuard: DIAG del TMC2209 dell'asse Y.
// Sulla FYSETC E4 il pin Y_STOP (endstop Y) e' collegato al DIAG del driver Y = GPIO 35.
// GPIO 35 e' input-only sull'ESP32 (ok per DIAG, che e' push-pull).
#define Y_DIAG_PIN 35

// --- UART ---
#define SERIAL_PORT Serial1
#define DRIVER_UART_RX 15
#define DRIVER_UART_TX 15
#define R_SENSE 0.11f

// --- OGGETTI ---
TMC2209Stepper driverX(&SERIAL_PORT, R_SENSE, X_ADDR);
TMC2209Stepper driverY(&SERIAL_PORT, R_SENSE, Y_ADDR);

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// --- WIFI ---
const char* ssid = "APA";
const char* password = "Apa1234567";

WebServer server(80);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
// Buffer di riga per il canale TCP: i comandi possono arrivare spezzati su piu'
// pacchetti, quindi accumuliamo i byte fino al newline (lettura NON bloccante).
String telnetBuf;

// --- STATO ---
int x_run_ma = 900;
float x_hold_mult = 0.2;
int x_microsteps = 16;

int y_run_ma = 900;
float y_hold_mult = 0.2;
int y_microsteps = 16;

// Profilo di moto (max speed e accelerazione) per ciascun asse. NVS-persistenti.
int x_max_speed = 1500;
int x_accel     = 500;
int y_max_speed = 1500;
int y_accel     = 500;

// LIMITI SICURI per NEMA 17 con TMC2209 sotto carico.
// Oltre questi valori il motore tende a stallare sotto coppia.
#define MOTION_SPEED_MIN  50
#define MOTION_SPEED_MAX  2500
#define MOTION_ACCEL_MIN  10
#define MOTION_ACCEL_MAX  1500

// Helper: clamp di un int in un range, con log seriale se viene corretto.
int clampMotion(const char *name, int v, int lo, int hi) {
  if (v < lo) {
    Serial.printf("[WARN] %s=%d below MIN, clamped to %d\n", name, v, lo);
    return lo;
  }
  if (v > hi) {
    Serial.printf("[WARN] %s=%d above MAX, clamped to %d\n", name, v, hi);
    return hi;
  }
  return v;
}

// --- STATO STALLGUARD / HOMING / LIMITI (solo Y) ---
Preferences prefs;

long  y_home_offset   = 200;        // step da percorrere DOPO lo stall per arrivare a "home"
long  y_min_limit     = -100000L;   // limite soft inferiore (step relativi a home)
long  y_max_limit     =  100000L;   // limite soft superiore (step relativi a home)
uint8_t y_sgthrs      = 80;         // soglia StallGuard (0-255: piu alto = piu sensibile)
int   y_homing_speed  = 1500;       // step/s durante l'homing (alza se SG4 non rileva: SG4 e' inaffidabile <50 full step/s)
int   y_homing_dir    = -1;         // -1 = homing verso step negativi, +1 verso positivi
bool  y_homed         = false;      // true dopo un homing riuscito o un :RESETY

volatile bool y_stall_flag    = false;
volatile bool y_stall_enabled = false;  // gate ISR: setta il flag solo durante l'homing

void IRAM_ATTR onYDiag() {
  if (y_stall_enabled) y_stall_flag = true;
}

// --- INTERFACCIA WEB (HTML/CSS/JS) ---
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>FYSETC E4 Control</title>
<style>
  body { font-family: sans-serif; text-align: center; background-color: #1a1a1a; color: #eee; margin: 0; padding: 10px; }
  h2 { color: #f39c12; margin: 10px 0; }
  .card { background-color: #2d2d2d; padding: 10px; margin: 10px auto; max-width: 400px; border-radius: 8px; border: 1px solid #444; }
  
  /* Griglia bottoni gradi */
  .deg-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 10px; }
  .btn-deg { padding: 10px 0; background-color: #34495e; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; }
  .btn-deg:active { background-color: #2980b9; transform: translateY(2px); }
  .pos { background-color: #27ae60; }
  .neg { background-color: #c0392b; }

  /* Controlli settings */
  .ctrl-row { display: flex; justify-content: space-between; margin-top: 8px; }
  input, select { padding: 8px; width: 60%; background: #444; color: white; border: 1px solid #555; border-radius: 4px; }
  .btn-set { width: 35%; background-color: #f39c12; border: none; border-radius: 4px; color: white; font-weight: bold; }
  
  .log { font-family: monospace; color: #00ff00; font-size: 12px; margin-top: 5px; min-height: 15px; }
</style>
<script>
  // Configurazione Motori (Standard 1.8 gradi = 200 step/giro)
  const STEPS_PER_REV = 200; 
  
  // Teniamo traccia dei microstep lato JS per fare i calcoli giusti
  var microX = 16;
  var microY = 16;

  function sendCmd(cmd) {
    fetch("/cmd?val=" + encodeURIComponent(cmd))
      .then(r => r.text())
      .then(t => document.getElementById("status").innerText = "> " + t);
  }

  // Muove in gradi: Calcola i passi basandosi sui microstep attuali
  function moveDeg(axis, deg) {
    let u = (axis === 'X') ? microX : microY;
    // Formula: (Gradi / 360) * (StepMotore * Microsteps)
    let steps = Math.round((deg / 360.0) * (STEPS_PER_REV * u));
    sendCmd(axis + steps);
  }

  // Imposta microstep e aggiorna variabile JS
  function setMicro(axis) {
    let val = document.getElementById('m' + axis).value;
    val = parseInt(val);
    if(axis === 'X') microX = val;
    else microY = val;
    sendCmd('S' + axis + val);
  }

  function setCurrent(axis, type) {
    let id = (type === 'C') ? 'c' : 'h'; // C=Run, H=Hold
    let val = document.getElementById(id + axis).value;
    sendCmd(type + axis + val);
  }
</script>
</head>
<body>
  <h2>FYSETC E4 WiFi</h2>
  <div id="status" class="log">Ready</div>

  <div class="card">
    <h3>AXIS X</h3>
    <div class="deg-grid">
      <button class="btn-deg neg" onclick="moveDeg('X', -360)">-360&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('X', -90)">-90&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('X', -10)">-10&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('X', -1)">-1&deg;</button>
      
      <button class="btn-deg pos" onclick="moveDeg('X', 1)">+1&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('X', 10)">+10&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('X', 90)">+90&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('X', 360)">+360&deg;</button>
    </div>

    <div class="ctrl-row">
      <input type="number" id="cX" placeholder="Run mA (600)">
      <button class="btn-set" onclick="setCurrent('X', 'C')">SET mA</button>
    </div>
    <div class="ctrl-row">
      <select id="mX" onchange="setMicro('X')">
        <option value="16" selected>16 Steps</option>
        <option value="32">32 Steps</option>
        <option value="64">64 Steps</option>
        <option value="0">Full</option>
      </select>
      <button class="btn-set" onclick="setMicro('X')">SET STP</button>
    </div>
  </div>

  <div class="card">
    <h3>AXIS Y</h3>
    <div class="deg-grid">
      <button class="btn-deg neg" onclick="moveDeg('Y', -360)">-360&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('Y', -90)">-90&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('Y', -10)">-10&deg;</button>
      <button class="btn-deg neg" onclick="moveDeg('Y', -1)">-1&deg;</button>
      
      <button class="btn-deg pos" onclick="moveDeg('Y', 1)">+1&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('Y', 10)">+10&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('Y', 90)">+90&deg;</button>
      <button class="btn-deg pos" onclick="moveDeg('Y', 360)">+360&deg;</button>
    </div>

    <div class="ctrl-row">
      <input type="number" id="cY" placeholder="Run mA (600)">
      <button class="btn-set" onclick="setCurrent('Y', 'C')">SET mA</button>
    </div>
    <div class="ctrl-row">
      <select id="mY" onchange="setMicro('Y')">
        <option value="16" selected>16 Steps</option>
        <option value="32">32 Steps</option>
        <option value="64">64 Steps</option>
        <option value="0">Full</option>
      </select>
      <button class="btn-set" onclick="setMicro('Y')">SET STP</button>
    </div>
  </div>
</body></html>
)rawliteral";


// --- FUNZIONI DRIVER ---
void apply_current_x() { driverX.rms_current(x_run_ma, x_hold_mult); }
void apply_current_y() { driverY.rms_current(y_run_ma, y_hold_mult); }

// --- PERSISTENZA NVS ---
// Posizione Y ripristinata al boot (vale per "ricordare dove si trova"
// dopo spegnimento; ovviamente solo se nessuno ha mosso il motore a mano).
long y_saved_pos = 0;
bool y_pos_was_saved = false;

void loadConfig() {
  prefs.begin("aapa", true); // read-only
  x_run_ma       = prefs.getInt ("x_ma",   x_run_ma);
  y_run_ma       = prefs.getInt ("y_ma",   y_run_ma);
  x_microsteps   = prefs.getInt ("x_ms",   x_microsteps);
  y_microsteps   = prefs.getInt ("y_ms",   y_microsteps);
  y_home_offset  = prefs.getLong("y_home", y_home_offset);
  y_min_limit    = prefs.getLong("y_min",  y_min_limit);
  y_max_limit    = prefs.getLong("y_max",  y_max_limit);
  y_sgthrs       = prefs.getUChar("y_sg",  y_sgthrs);
  y_homing_dir   = prefs.getInt ("y_dir",  y_homing_dir);
  y_homing_speed = prefs.getInt ("y_hsp",  y_homing_speed);
  // Profilo di moto: leggi e CLAMPA in range sicuro (NVS corrotta o vecchia non danneggia).
  x_max_speed = clampMotion("NVS x_spd", prefs.getInt("x_spd", x_max_speed),
                            MOTION_SPEED_MIN, MOTION_SPEED_MAX);
  x_accel     = clampMotion("NVS x_acc", prefs.getInt("x_acc", x_accel),
                            MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
  y_max_speed = clampMotion("NVS y_spd", prefs.getInt("y_spd", y_max_speed),
                            MOTION_SPEED_MIN, MOTION_SPEED_MAX);
  y_accel     = clampMotion("NVS y_acc", prefs.getInt("y_acc", y_accel),
                            MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
  // Posizione Y salvata + flag "era valida"
  y_pos_was_saved = prefs.getBool("y_psv", false);
  y_saved_pos     = prefs.getLong("y_pos", 0);
  prefs.end();
}

void saveConfig() {
  prefs.begin("aapa", false);
  prefs.putInt  ("x_ma",   x_run_ma);
  prefs.putInt  ("y_ma",   y_run_ma);
  prefs.putInt  ("x_ms",   x_microsteps);
  prefs.putInt  ("y_ms",   y_microsteps);
  prefs.putLong ("y_home", y_home_offset);
  prefs.putLong ("y_min",  y_min_limit);
  prefs.putLong ("y_max",  y_max_limit);
  prefs.putUChar("y_sg",   y_sgthrs);
  prefs.putInt  ("y_dir",  y_homing_dir);
  prefs.putInt  ("y_hsp",  y_homing_speed);
  // Profilo di moto
  prefs.putInt  ("x_spd",  x_max_speed);
  prefs.putInt  ("x_acc",  x_accel);
  prefs.putInt  ("y_spd",  y_max_speed);
  prefs.putInt  ("y_acc",  y_accel);
  // Salva ANCHE la posizione corrente (solo se Y e' homato).
  if (y_homed) {
    prefs.putLong("y_pos", stepperY.currentPosition());
    prefs.putBool("y_psv", true);
  }
  prefs.end();
}

// --- STALLGUARD Y ---
void setupStallGuardY() {
  // TCOOLTHRS alto = StallGuard sempre attivo durante il movimento.
  driverY.TCOOLTHRS(0xFFFFF);
  driverY.SGTHRS(y_sgthrs);
  pinMode(Y_DIAG_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Y_DIAG_PIN), onYDiag, RISING);
}

// Limita il target Y ai soft-limits (solo se gia' homato).
long clampYTarget(long target) {
  if (!y_homed) return target;
  if (target < y_min_limit) return y_min_limit;
  if (target > y_max_limit) return y_max_limit;
  return target;
}

// Procedura di homing su Y con StallGuard + offset.
String homeY() {
  if (stepperY.isRunning() || stepperX.isRunning()) return "ERR: busy";

  // Reset latch di DIAG: dopo uno stall precedente il TMC2209 puo' tenere DIAG
  // asserito a motore fermo (SG_RESULT non aggiornato finche' il motore non gira).
  // Disabilitando temporaneamente StallGuard (TCOOLTHRS=0) la linea torna LOW.
  driverY.TCOOLTHRS(0);
  delay(30);

  // Adesso il check ha senso: se DIAG e' ANCORA high con StallGuard disabilitato,
  // la linea e' davvero sospesa (jumper JP2 aperto / pull-up R23 che vince).
  if (digitalRead(Y_DIAG_PIN) == HIGH) {
    driverY.TCOOLTHRS(0xFFFFF); // ripristina
    return "ERR: DIAG_Y still HIGH after SG disable -> jumper JP2 open or HW fault";
  }

  // Riabilita StallGuard per l'homing.
  driverY.TCOOLTHRS(0xFFFFF);

  // Salva i parametri di moto correnti per ripristinarli a fine homing.
  float saved_max_speed = stepperY.maxSpeed();
  float saved_accel     = stepperY.acceleration();

  y_stall_flag    = false;
  y_stall_enabled = false;

  stepperY.setMaxSpeed(y_homing_speed);
  stepperY.setAcceleration(y_homing_speed * 2 );   // accel moderata (era *4)

  // Target enorme nella direzione di homing: il moto verra' interrotto dallo stall.
  long big_target = (long)y_homing_dir * 1000000L;
  stepperY.move(big_target);

  // Fase di accelerazione iniziale: niente rilevamento stall (evita falsi positivi).
  unsigned long t0 = millis();
  while (millis() - t0 < 400) {
    stepperY.run();
    yield();
  }

  // Ora arma il rilevamento.
  y_stall_flag    = false;
  y_stall_enabled = true;

  unsigned long t_timeout = millis();
  while (!y_stall_flag && (millis() - t_timeout < 60000UL)) {
    stepperY.run();
    yield();
  }

  y_stall_enabled = false;

  // Stop "morbido" (decelera con l'accel impostata sopra).
  stepperY.stop();
  while (stepperY.isRunning()) { stepperY.run(); yield(); }

  // Ripristina parametri di moto.
  // Ripristina ai valori user (sempre dentro range sicuro).
  // Non usiamo saved_max_speed/saved_accel per evitare contaminazione da stati sporchi.
  stepperY.setMaxSpeed(y_max_speed);
  stepperY.setAcceleration(y_accel);

  if (!y_stall_flag) {
    return "ERR: homing timeout (no stall)";
  }

  // Punto di stallo = posizione 0.
  stepperY.setCurrentPosition(0);

  // Allontanati di y_home_offset step nella direzione OPPOSTA all'homing.
  long back_off = -(long)y_homing_dir * y_home_offset;
  stepperY.moveTo(back_off);
  while (stepperY.isRunning()) { stepperY.run(); yield(); }

  // Questa e' la nuova "home" = posizione 0 logica.
  stepperY.setCurrentPosition(0);
  y_homed = true;

  return "OK: Y homed";
}

// --- COMANDI ESTESI (prefisso ":") ---
String parseColonCmd(String cmd) {
  cmd.trim();
  String name, args;
  int sp = cmd.indexOf(' ');
  if (sp < 0) { name = cmd; args = ""; }
  else        { name = cmd.substring(0, sp); args = cmd.substring(sp + 1); args.trim(); }
  name.toUpperCase();

  if (name == "HOMEY")  return homeY();
  if (name == "OFFY")   { y_home_offset = args.toInt(); return "OK OFFY=" + String(y_home_offset); }
  if (name == "MINY")   { y_min_limit   = args.toInt(); return "OK MINY=" + String(y_min_limit); }
  if (name == "MAXY")   { y_max_limit   = args.toInt(); return "OK MAXY=" + String(y_max_limit); }
  if (name == "SGY")    { y_sgthrs = (uint8_t)args.toInt(); driverY.SGTHRS(y_sgthrs); return "OK SGY=" + String(y_sgthrs); }
  if (name == "DIRY")   { y_homing_dir = (args.toInt() >= 0) ? 1 : -1; return "OK DIRY=" + String(y_homing_dir); }
  if (name == "HSPY")   { y_homing_speed = max(50, (int)args.toInt()); return "OK HSPY=" + String(y_homing_speed); }
  // Profilo di moto (per movimenti normali, NON l'homing).
  // Tutti i valori sono clampati in range sicuro per NEMA 17.
  if (name == "SPDX") {
    x_max_speed = clampMotion("SPDX", args.toInt(), MOTION_SPEED_MIN, MOTION_SPEED_MAX);
    stepperX.setMaxSpeed(x_max_speed);
    return "OK SPDX=" + String(x_max_speed);
  }
  if (name == "SPDY") {
    y_max_speed = clampMotion("SPDY", args.toInt(), MOTION_SPEED_MIN, MOTION_SPEED_MAX);
    stepperY.setMaxSpeed(y_max_speed);
    return "OK SPDY=" + String(y_max_speed);
  }
  if (name == "ACCX") {
    x_accel = clampMotion("ACCX", args.toInt(), MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
    stepperX.setAcceleration(x_accel);
    return "OK ACCX=" + String(x_accel);
  }
  if (name == "ACCY") {
    y_accel = clampMotion("ACCY", args.toInt(), MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
    stepperY.setAcceleration(y_accel);
    return "OK ACCY=" + String(y_accel);
  }
  // Emergenza: torna ai default safe (utile se hai messo valori troppo aggressivi).
  if (name == "RESETSPD") {
    x_max_speed = 1500; x_accel = 500;
    y_max_speed = 1500; y_accel = 500;
    stepperX.setMaxSpeed(x_max_speed); stepperX.setAcceleration(x_accel);
    stepperY.setMaxSpeed(y_max_speed); stepperY.setAcceleration(y_accel);
    return "OK reset speed/accel to 1500/500";
  }
  if (name == "SAVE")   { saveConfig(); return "OK SAVED"; }
  if (name == "STOP")   { stepperX.stop(); stepperY.stop(); return "OK STOPPED"; }
  if (name == "RESETY") {
    stepperY.setCurrentPosition(0);
    y_homed = true;
    // Invalida la posizione salvata: dovra' essere ri-salvata con :SAVE
    prefs.begin("aapa", false); prefs.putBool("y_psv", false); prefs.end();
    return "OK Y=0 (manual home, NVS pos invalidated)";
  }

  // --- DIAGNOSTICA STALLGUARD ---
  // :DREAD  -> lettura istantanea del pin DIAG_Y e di SG_RESULT
  if (name == "DREAD") {
    return "DIAGY_PIN=" + String(digitalRead(Y_DIAG_PIN)) +
           " SG_RESULT=" + String(driverY.SG_RESULT()) +
           " SGTHRS=" + String(y_sgthrs);
  }
  // :SGTEST [steps]  -> muove Y a velocita' di homing e riporta min/max SG_RESULT
  //   e se DIAG e' mai andato HIGH. Default: 4000 step nella direzione di homing.
  if (name == "SGTEST") {
    long steps = args.toInt();
    if (steps == 0) steps = (long)y_homing_dir * 4000;
    if (stepperY.isRunning() || stepperX.isRunning()) return "ERR: busy";

    // 1) Stato DIAG a motore FERMO (deve essere LOW se JP2 e' saldato).
    int diag_at_rest = digitalRead(Y_DIAG_PIN);

    // Salva ma clampa in range sicuro nel caso AccelStepper restituisca valori sporchi.
    int saved_max = clampMotion("SGTEST saved_max", (int)stepperY.maxSpeed(),
                                MOTION_SPEED_MIN, MOTION_SPEED_MAX);
    int saved_acc = clampMotion("SGTEST saved_acc", (int)stepperY.acceleration(),
                                MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
    int sgtest_speed = clampMotion("SGTEST speed", y_homing_speed,
                                    MOTION_SPEED_MIN, MOTION_SPEED_MAX);
    int sgtest_accel = clampMotion("SGTEST accel", sgtest_speed / 2,
                                    MOTION_ACCEL_MIN, MOTION_ACCEL_MAX);
    stepperY.setMaxSpeed(sgtest_speed);
    stepperY.setAcceleration(sgtest_accel);
    stepperY.move(steps);

    unsigned long t_warm = millis();
    while (millis() - t_warm < 300) { stepperY.run(); yield(); }

    int min_sg = 9999, max_sg = 0;
    bool diag_seen = false;
    unsigned long t_start = millis();
    while (stepperY.isRunning() && (millis() - t_start < 30000)) {
      stepperY.run();
      int sg = driverY.SG_RESULT();
      if (sg < min_sg) min_sg = sg;
      if (sg > max_sg) max_sg = sg;
      if (digitalRead(Y_DIAG_PIN) == HIGH) diag_seen = true;
      yield();
    }
    // Ripristina ai valori user (non al "saved" sporco): cosi' siamo certi del profilo dopo SGTEST.
    stepperY.setMaxSpeed(y_max_speed);
    stepperY.setAcceleration(y_accel);

    // Verdetto leggibile.
    String verdict;
    if (diag_at_rest == HIGH) {
      verdict = " VERDICT: JP2_OPEN (DIAG stuck high at rest)";
    } else if (!diag_seen && max_sg == 0) {
      verdict = " VERDICT: TMC2209_UART_FAIL (SG_RESULT sempre 0)";
    } else if (!diag_seen) {
      verdict = " VERDICT: SG_OK_but_no_stall (alza SGTHRS o spingi piu forte)";
    } else {
      verdict = " VERDICT: OK_stall_detected";
    }

    return "SGTEST: DIAG_rest=" + String(diag_at_rest) +
           " SG_min=" + String(min_sg) +
           " SG_max=" + String(max_sg) +
           " DIAG_seen=" + String(diag_seen ? 1 : 0) +
           " trig_below=" + String(y_sgthrs * 2) +
           verdict;
  }

  if (name == "STATUS") {
    String s  = "POSX:"  + String(stepperX.currentPosition());
    s += " POSY:"  + String(stepperY.currentPosition());
    s += " BUSYX:" + String(stepperX.isRunning() ? 1 : 0);
    s += " BUSYY:" + String(stepperY.isRunning() ? 1 : 0);
    s += " HOMED:" + String(y_homed ? 1 : 0);
    s += " OFFY:"  + String(y_home_offset);
    s += " MINY:"  + String(y_min_limit);
    s += " MAXY:"  + String(y_max_limit);
    s += " SGY:"   + String(y_sgthrs);
    s += " DIRY:"  + String(y_homing_dir);
    s += " HSPY:"  + String(y_homing_speed);
    s += " SGRES:" + String(driverY.SG_RESULT());
    s += " DIAGY:" + String(digitalRead(Y_DIAG_PIN));
    s += " SPDX:"  + String(x_max_speed);
    s += " ACCX:"  + String(x_accel);
    s += " SPDY:"  + String(y_max_speed);
    s += " ACCY:"  + String(y_accel);
    s += " MSX:"   + String(x_microsteps);
    s += " MSY:"   + String(y_microsteps);
    return s;
  }
  return "ERR: unknown :cmd";
}

// --- PARSER COMANDI ---
String parseCommand(String input) {
  input.trim();
  if (input.length() == 0) return "ERR";

  // Comandi estesi con prefisso ":"
  if (input.charAt(0) == ':') {
    return parseColonCmd(input.substring(1));
  }

  String response = "OK";
  if (input.length() < 2) return "ERR";

  char firstChar = input.charAt(0);

  // Movimento Diretto (es. X800 calcolato dalla web ui per 90 gradi)
  if ((firstChar == 'X' || firstChar == 'x' || firstChar == 'Y' || firstChar == 'y') &&
      (isdigit(input.charAt(1)) || input.charAt(1) == '-')) {
      long steps = input.substring(1).toInt();
      if (firstChar == 'X' || firstChar == 'x') {
        stepperX.move(steps);
        response = "X: " + String(steps);
      } else {
        // Y con clamping ai soft-limit (solo se homed).
        long current = stepperY.currentPosition();
        long target  = clampYTarget(current + steps);
        long actual  = target - current;
        stepperY.move(actual);
        if (actual != steps) {
          response = "Y CLAMPED: " + String(actual) + " (req " + String(steps) + ")";
        } else {
          response = "Y: " + String(actual);
        }
      }
      return response;
  }

  // Comandi Config (CX, SX, HX...)
  if (input.length() > 2) {
      char type = firstChar; char axis = input.charAt(1);
      int val = input.substring(2).toInt();

      if (type == 'C' || type == 'c') {
        if (axis == 'X' || axis == 'x') { x_run_ma = val; apply_current_x(); response = "X mA: " + String(val); }
        else { y_run_ma = val; apply_current_y(); response = "Y mA: " + String(val); }
      }
      else if (type == 'H' || type == 'h') {
        float mult = val / 100.0;
        if (axis == 'X' || axis == 'x') { x_hold_mult = mult; apply_current_x(); response = "X Hold: " + String(val) + "%"; }
        else { y_hold_mult = mult; apply_current_y(); response = "Y Hold: " + String(val) + "%"; }
      }
      else if (type == 'S' || type == 's') {
        if (axis == 'X' || axis == 'x') {
          driverX.microsteps(val); x_microsteps = val;
          // Auto-persist: la microstep e' un valore "permanente" e deve sopravvivere al reboot.
          prefs.begin("aapa", false); prefs.putInt("x_ms", x_microsteps); prefs.end();
          response = "X Micro: " + String(val) + " (saved)";
        } else {
          driverY.microsteps(val); y_microsteps = val;
          prefs.begin("aapa", false); prefs.putInt("y_ms", y_microsteps); prefs.end();
          response = "Y Micro: " + String(val) + " (saved)";
        }
      }
  }
  return response;
}

// --- HANDLERS ---
void handleRoot() { server.send(200, "text/html", HTML_PAGE); }
void handleCmd() {
  if (server.hasArg("val")) {
    server.send(200, "text/plain", parseCommand(server.arg("val")));
  } else server.send(400, "text/plain", "Err");
}

// Canale comandi via WiFi (TCP porta 23). Lettura NON bloccante: consuma solo i
// byte gia' disponibili e accumula in telnetBuf fino al newline, poi esegue il
// comando. Cosi' il loop dei motori (stepperX/Y.run()) non si impunta mai.
void handleTelnet() {
  // Accetta un nuovo client; ne tiene UNO solo alla volta.
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.setNoDelay(true);
      telnetBuf = "";
    } else {
      telnetServer.available().stop();  // rifiuta connessioni extra
    }
  }

  while (telnetClient && telnetClient.connected() && telnetClient.available()) {
    char c = (char)telnetClient.read();
    if (c == '\n' || c == '\r') {
      if (telnetBuf.length() > 0) {
        telnetClient.println(parseCommand(telnetBuf));
        telnetBuf = "";
      }
    } else {
      telnetBuf += c;
      if (telnetBuf.length() > 200) telnetBuf = "";  // guard anti-overflow
    }
  }
}

void setup() {
  Serial.begin(115200);
  SERIAL_PORT.begin(115200, SERIAL_8N1, DRIVER_UART_RX, DRIVER_UART_TX);

  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);

  // Carica config persistente PRIMA di inizializzare i driver
  // cosi' microstep/correnti/SGTHRS partono gia' dai valori salvati.
  loadConfig();

  driverX.begin(); driverX.toff(5); driverX.microsteps(x_microsteps); driverX.pwm_autoscale(true); apply_current_x();
  driverY.begin(); driverY.toff(5); driverY.microsteps(y_microsteps); driverY.pwm_autoscale(true); apply_current_y();

  // StallGuard solo su Y.
  setupStallGuardY();

  // Profilo di moto: caricato da NVS (vedi loadConfig).
  // Comandi runtime: :SPDX/:SPDY (max speed) e :ACCX/:ACCY (accel).
  stepperX.setMaxSpeed(x_max_speed); stepperX.setAcceleration(x_accel);
  stepperY.setMaxSpeed(y_max_speed); stepperY.setAcceleration(y_accel);

  // Ripristina la posizione Y se era stata salvata (sopravvive al reboot).
  // Marca anche il sistema come "homed" cosi' i soft-limit sono subito attivi.
  if (y_pos_was_saved) {
    stepperY.setCurrentPosition(y_saved_pos);
    y_homed = true;
    Serial.print("Restored Y position from NVS: "); Serial.println(y_saved_pos);
  }

  // WIFI RESET & START
  WiFi.disconnect(true); WiFi.softAPdisconnect(true); delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  Serial.println("\n--- FYSETC E4 READY ---");
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
  telnetServer.begin(); telnetServer.setNoDelay(true);
}

void loop() {
  stepperX.run();
  stepperY.run();

  if (Serial.available()) Serial.println(parseCommand(Serial.readStringUntil('\n')));
  server.handleClient();
  handleTelnet();
}
