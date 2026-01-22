// ==========================================================
//  WĘDZARNIA – ESP32 (wersja z 3 grzałkami schodkowymi)
//  OLED + MENU + ENKODER + WEB + MQTT + Bezpieczeństwo drzwi
//  + PID Autotune + Tryby mocy + Zapis w NVS
//
//  Wersja stabilizowana – styczeń 2026
//  • asynchroniczny DS18B20
//  • non-blocking buzzer
//  • OLED w osobnym tasku FreeRTOS
//  • throttlowane: mqtt.loop(), OLED, sterowanie grzałkami
//  • monitorowanie heapu + logowanie
//  • getStatusJSON z snprintf (mniej fragmentacji)
//  • watchdog 15 s + prosty soft-watchdog
// ==========================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// ─────────────────────────────────────────────────────────────
//          DEFINICJE PINÓW
// ─────────────────────────────────────────────────────────────
#define PIN_SSR1       26
#define PIN_SSR2       15
#define PIN_SSR3        2
#define PIN_OVEN_FAN   14
#define PIN_SMOKE_FAN  27
#define PIN_BUZZER     12
#define PIN_DOOR       13
#define PIN_ONEWIRE     4
#define PIN_SD_CS       5

#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_ADDR    0x3C
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64

#define ENC_A          32
#define ENC_B          33
#define ENC_SW         25
#define BTN_STARTSTOP  34
#define BTN_PAUSERES   35

#define MAX_TEMP      120.0

// ─────────────────────────────────────────────────────────────
//          KONFIGURACJA WIFI I MQTT
// ─────────────────────────────────────────────────────────────
const char* ssid       = "SSID";                // ← Zmień
const char* pass       = "HASLO";               // ← Zmień
const char* mqttServer = "192.168.1.10";

// ─────────────────────────────────────────────────────────────
//          TEMPERATURA + PID + AUTOTUNE
// ─────────────────────────────────────────────────────────────
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

double Input       = 0.0;
double Output      = 0.0;
double Setpoint    = 65.0;
double lastGoodTemp = 25.0;

double Kp = 20.0;
double Ki = 0.5;
double Kd = 100.0;

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

PID_AutoTune autoTune(&Input, &Output);
bool tuning = false;
double aTuneStep = 255.0;
double aTuneNoise = 1.0;
unsigned int aTuneLookBack = 20;

// ─────────────────────────────────────────────────────────────
//          TRYBY MOCY
// ─────────────────────────────────────────────────────────────
enum PowerMode { POWER_LOW = 1, POWER_MEDIUM = 2, POWER_FULL = 3 };
PowerMode powerMode = POWER_FULL;

// ─────────────────────────────────────────────────────────────
//          NVS + PROFIL
// ─────────────────────────────────────────────────────────────
Preferences prefs;

struct Step {
  int   timeMin;
  float temp;
  bool  smoke;
  int   fan;
};

Step steps[10];
int stepCount     = 0;
int currentStep   = 0;
unsigned long stepStart = 0;

bool running = false;
bool paused  = false;

// ─────────────────────────────────────────────────────────────
//          GLOBALNE ZMIENNE
// ─────────────────────────────────────────────────────────────
bool lastDoorOpen = false;

unsigned long windowSize  = 4000;
unsigned long windowStart = 0;

AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────────────────────
//          MENU + ENKODER
// ─────────────────────────────────────────────────────────────
enum MenuState {
  SCREEN_MAIN, MENU_MAIN, MENU_PROFILE, MENU_STEPS, MENU_MANUAL, MENU_POWER
};
MenuState menuState = SCREEN_MAIN;
int menuIndex = 0;

const int MAX_PROFILES = 10;
String profileNames[MAX_PROFILES];
int profileCount = 0;
int selectedProfileIndex = 0;
int selectedStepIndex = 0;

bool editSetpoint  = false;
bool manualHeater  = false;
bool manualSmoke   = false;
bool manualFan     = false;

int selectedPowerIndex = 2;

volatile int16_t encPos = 0;
int16_t lastEncPos = 0;
int lastEncA = 0;
int lastEncB = 0;

unsigned long lastBtnEnc   = 0;
unsigned long lastBtnStart = 0;
unsigned long lastBtnPause = 0;
const unsigned long debounceMs = 200;

// ─────────────────────────────────────────────────────────────
//          NON-BLOCKING BUZZER
// ─────────────────────────────────────────────────────────────
bool buzzerActive = false;
unsigned long buzzerOffTime = 0;

void beep(int ms = 150) {
  if (buzzerActive) return;
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerOffTime = millis() + ms;
  buzzerActive = true;
}

// ─────────────────────────────────────────────────────────────
//          ASYNCHRONICZNY DS18B20
// ─────────────────────────────────────────────────────────────
unsigned long lastTempRequest = 0;
unsigned long lastTempReadPossible = 0;
const unsigned long TEMP_REQUEST_INTERVAL = 1200;
const unsigned long TEMP_CONVERSION_TIME  = 850;

void requestTemperature() {
  unsigned long now = millis();
  if (now - lastTempRequest >= TEMP_REQUEST_INTERVAL) {
    sensors.requestTemperatures();
    lastTempRequest = now;
    lastTempReadPossible = now + TEMP_CONVERSION_TIME;
  }
}

void readTemperature() {
  unsigned long now = millis();
  if (lastTempReadPossible > 0 && now >= lastTempReadPossible) {
    float t = sensors.getTempCByIndex(0);
    if (t > -50 && t < 150) {
      Input = t;
      lastGoodTemp = t;
    } else {
      Input = lastGoodTemp;
    }
    lastTempReadPossible = 0;
  }
}

// ─────────────────────────────────────────────────────────────
//          HTML (bez zmian)
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Wędzarnia</title>
<style>
body { font-family:sans-serif; background:#111; color:#eee; padding:20px; }
.card { background:#222; padding:15px; margin-bottom:15px; border-radius:8px; }
button { padding:8px 16px; margin:4px; background:#444; color:#eee; border:none; border-radius:4px; cursor:pointer; }
button:hover { background:#666; }
select { padding:6px; margin:4px; background:#333; color:#eee; border:1px solid #555; }
.label { color:#aaa; font-size:0.9em; }
canvas { background:#000; border:1px solid #444; width:100%; max-width:600px; height:300px; }
.paused { color:#ff4444; font-weight:bold; font-size:1.6em; margin:10px 0; text-align:center; display:none; }
</style>
</head>
<body>
<h1>Wędzarnia</h1>
<div class="card">
  <div class="label">Temperatura</div>
  <div><span id="temp">--</span> °C</div>
  <div class="label">Setpoint</div>
  <div><span id="setpoint">--</span> °C</div>
  <div class="label">Stan</div>
  <div>
    running: <span id="running">false</span><br>
    <span id="pauseStatus" class="paused">PAUZA</span>
  </div>
  <div class="label">Moc grzałek</div>
  <div><span id="powerMode">Full</span></div>
  <div class="label">Krok</div>
  <div><span id="currentStep">0</span> / <span id="stepCount">0</span></div>
</div>
<div class="card">
  <div class="label">Wykres temperatury (ostatnie ~5 min)</div>
  <canvas id="chart"></canvas>
</div>
<div class="card">
  <div class="label">Sterowanie</div>
  <button onclick="sendCmd('START')">START</button>
  <button onclick="sendCmd('STOP')">STOP</button>
  <button onclick="sendCmd('PAUSE')">PAUSE</button>
  <button onclick="sendCmd('RESUME')">RESUME</button>
  <button onclick="sendCmd('AUTOTUNE')">AUTOTUNE PID</button>
</div>
<div class="card">
  <div class="label">Moc grzałek</div>
  <select id="powerSelect">
    <option value="LOW">Low (1 grzałka)</option>
    <option value="MEDIUM">Medium (2 grzałki)</option>
    <option value="FULL">Full (3 grzałki)</option>
  </select>
  <button onclick="setPowerMode()">Ustaw moc</button>
</div>
<div class="card">
  <div class="label">Profil</div>
  <select id="profileSelect"></select>
  <button onclick="loadProfile()">Załaduj profil</button>
</div>
<script>
let historyTemp = [];
let historySet  = [];
let historyMaxPoints = 150;
const tempEl       = document.getElementById('temp');
const setpointEl   = document.getElementById('setpoint');
const runningEl    = document.getElementById('running');
const pauseStatusEl= document.getElementById('pauseStatus');
const powerModeEl  = document.getElementById('powerMode');
const currentStepEl= document.getElementById('currentStep');
const stepCountEl  = document.getElementById('stepCount');
const profileSelect= document.getElementById('profileSelect');
const powerSelect  = document.getElementById('powerSelect');
const canvas       = document.getElementById('chart');
const ctx          = canvas.getContext('2d');

function resizeCanvas(){
  const rect = canvas.getBoundingClientRect();
  canvas.width  = rect.width;
  canvas.height = rect.height;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function drawChart(){
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0,0,w,h);
  if(historyTemp.length < 2) return;
  let minVal = Math.min(...historyTemp, ...historySet);
  let maxVal = Math.max(...historyTemp, ...historySet);
  if(minVal == maxVal){
    minVal -= 1;
    maxVal += 1;
  }
  function yFor(v){
    return h - ( (v - minVal) / (maxVal - minVal) ) * (h-20) - 10;
  }
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, yFor(minVal));
  ctx.lineTo(w, yFor(minVal));
  ctx.moveTo(0, yFor(maxVal));
  ctx.lineTo(w, yFor(maxVal));
  ctx.stroke();
  ctx.strokeStyle = '#ff4444';
  ctx.lineWidth = 2;
  ctx.beginPath();
  historyTemp.forEach((v,i)=>{
    const x = (i/(historyTemp.length-1))*w;
    const y = yFor(v);
    if(i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.stroke();
  ctx.strokeStyle = '#44aaff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  historySet.forEach((v,i)=>{
    const x = (i/(historySet.length-1))*w;
    const y = yFor(v);
    if(i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.stroke();
}

async function refreshStatus(){
  try{
    const r = await fetch('/status');
    const j = await r.json();
    tempEl.textContent       = j.temp.toFixed(1);
    setpointEl.textContent   = j.setpoint.toFixed(1);
    runningEl.textContent    = j.running;
    if (j.paused) {
      pauseStatusEl.textContent = "PAUZA";
      pauseStatusEl.style.display = "block";
    } else {
      pauseStatusEl.textContent = "";
      pauseStatusEl.style.display = "none";
    }
    powerModeEl.textContent  = j.powerMode;
    currentStepEl.textContent= j.currentStep;
    stepCountEl.textContent  = j.stepCount;
    historyTemp.push(j.temp);
    historySet.push(j.setpoint);
    if(historyTemp.length > historyMaxPoints){
      historyTemp.shift();
      historySet.shift();
    }
    drawChart();
  }catch(e){}
}

async function sendCmd(cmd){
  await fetch('/cmd?c=' + encodeURIComponent(cmd));
  refreshStatus();
}

async function setPowerMode(){
  const mode = powerSelect.value;
  if(!mode) return;
  await fetch('/cmd?c=' + encodeURIComponent('POWER:' + mode));
  refreshStatus();
}

async function loadProfile(){
  const name = profileSelect.value;
  if(!name) return;
  await fetch('/cmd?c=' + encodeURIComponent('PROFILE:' + name));
  refreshStatus();
}

async function loadProfiles(){
  try{
    const r = await fetch('/profiles');
    const j = await r.json();
    profileSelect.innerHTML = "";
    j.profiles.forEach(p=>{
      const opt=document.createElement('option');
      opt.value=p.replace(".json","");
      opt.textContent=p.replace(".json","");
      profileSelect.appendChild(opt);
    });
  }catch(e){}
}

setInterval(refreshStatus,2000);
refreshStatus();
loadProfiles();
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────────
//          FUNKCJE POMOCNICZE
// ─────────────────────────────────────────────────────────────

void turnOffHeaters() {
  digitalWrite(PIN_SSR1, LOW);
  digitalWrite(PIN_SSR2, LOW);
  digitalWrite(PIN_SSR3, LOW);
}

void doorSafety() {
  bool doorOpen = digitalRead(PIN_DOOR) == HIGH;
  if (doorOpen != lastDoorOpen) {
    if (doorOpen) {
      beep(300);
      running = false;
      tuning = false;
      turnOffHeaters();
    } else {
      beep(80);
    }
    lastDoorOpen = doorOpen;
  }
}

void controlHeatersStepped() {
  unsigned long now = millis();
  if (now - windowStart >= windowSize) {
    windowStart = now;
  }

  double onTime1 = 0, onTime2 = 0, onTime3 = 0;
  const double pidStep = 255.0 / 3.0;

  if (Output > 0 && Output <= pidStep) {
    onTime1 = (Output / pidStep) * windowSize;
  } else if (Output <= 2 * pidStep) {
    onTime1 = windowSize;
    onTime2 = ((Output - pidStep) / pidStep) * windowSize;
  } else {
    onTime1 = windowSize;
    onTime2 = windowSize;
    onTime3 = ((Output - 2 * pidStep) / pidStep) * windowSize;
  }

  if (powerMode == POWER_LOW)      { onTime2 = onTime3 = 0; }
  else if (powerMode == POWER_MEDIUM) { onTime3 = 0; }

  unsigned long elapsed = now - windowStart;
  digitalWrite(PIN_SSR1, elapsed < onTime1);
  digitalWrite(PIN_SSR2, elapsed < onTime2);
  digitalWrite(PIN_SSR3, elapsed < onTime3);
}

void startAutotune() {
  if (running || paused || lastDoorOpen || tuning) return;
  autoTune.SetOutputStep(aTuneStep);
  autoTune.SetNoiseBand(aTuneNoise);
  autoTune.SetLookbackSec(aTuneLookBack);
  tuning = true;
  Output = 0;
  Serial.println("Autotune START");
  beep(400);
}

void handleAutotune() {
  int val = autoTune.Runtime();
  if (val != 0) {
    tuning = false;
    Kp = autoTune.GetKp(); Ki = autoTune.GetKi(); Kd = autoTune.GetKd();
    pid.SetTunings(Kp, Ki, Kd);
    saveSettings();
    Serial.printf("Autotune done: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
    beep(700);
  } else {
    bool on = Output > 127;
    digitalWrite(PIN_SSR1, on);
    digitalWrite(PIN_SSR2, on);
    digitalWrite(PIN_SSR3, on);
  }
}

void saveSettings() {
  prefs.begin("settings", false);
  prefs.putDouble("Kp", Kp);
  prefs.putDouble("Ki", Ki);
  prefs.putDouble("Kd", Kd);
  prefs.putInt("powerMode", (int)powerMode);
  prefs.end();
  beep(80);
}

void loadSettings() {
  prefs.begin("settings", true);
  Kp = prefs.getDouble("Kp", 20.0);
  Ki = prefs.getDouble("Ki", 0.5);
  Kd = prefs.getDouble("Kd", 100.0);
  powerMode = (PowerMode)prefs.getInt("powerMode", (int)POWER_FULL);
  prefs.end();
  pid.SetTunings(Kp, Ki, Kd);
}

// ─────────────────────────────────────────────────────────────
//          PROFIL
// ─────────────────────────────────────────────────────────────

bool loadProfile(const char* path) {
  File f = SD.open(path);
  if (!f) return false;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  stepCount = 0;
  for (JsonObject s : doc["steps"].as<JsonArray>()) {
    if (stepCount >= 10) break;
    steps[stepCount].timeMin = s["time"];
    steps[stepCount].temp    = s["temp"];
    steps[stepCount].smoke   = s["smoke"];
    steps[stepCount].fan     = s["fan"];
    stepCount++;
  }
  return true;
}

void loadProfileList() {
  profileCount = 0;
  File dir = SD.open("/profiles");
  if (!dir) return;
  File entry = dir.openNextFile();
  while (entry && profileCount < MAX_PROFILES) {
    String name = entry.name();
    if (name.endsWith(".json")) {
      name.replace("/profiles/", "");
      profileNames[profileCount++] = name;
    }
    entry = dir.openNextFile();
  }
  dir.close();
}

void startProfile() {
  if (stepCount == 0) return;
  currentStep = 0;
  stepStart = millis();
  Setpoint = steps[0].temp;
  running = true;
  paused = false;
  pid.SetMode(MANUAL); Output = 0; pid.SetMode(AUTOMATIC);
}

void nextStep() {
  currentStep++;
  if (currentStep >= stepCount) {
    running = false;
    turnOffHeaters();
    beep(800);
    return;
  }
  stepStart = millis();
  Setpoint = steps[currentStep].temp;
  pid.SetMode(MANUAL); Output = 0; pid.SetMode(AUTOMATIC);
}

void runStep() {
  Step &s = steps[currentStep];
  digitalWrite(PIN_SMOKE_FAN, s.smoke ? HIGH : LOW);
  digitalWrite(PIN_OVEN_FAN, s.fan > 0 ? HIGH : LOW);

  unsigned long elapsedMin = (millis() - stepStart) / 60000UL;
  if (elapsedMin >= (unsigned long)s.timeMin) nextStep();
}

// ─────────────────────────────────────────────────────────────
//          MQTT
// ─────────────────────────────────────────────────────────────

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String cmd;
  for (unsigned int i = 0; i < len; i++) cmd += (char)payload[i];

  if (cmd == "START") startProfile();
  else if (cmd == "STOP") { running = false; turnOffHeaters(); }
  else if (cmd == "PAUSE") paused = true;
  else if (cmd == "RESUME") paused = false;
  else if (cmd == "AUTOTUNE") startAutotune();
  else if (cmd.startsWith("PROFILE:")) {
    String name = cmd.substring(8);
    String path = "/profiles/" + name + ".json";
    loadProfile(path.c_str());
  }
  else if (cmd.startsWith("POWER:")) {
    String m = cmd.substring(6);
    if (m == "LOW") powerMode = POWER_LOW;
    else if (m == "MEDIUM") powerMode = POWER_MEDIUM;
    else if (m == "FULL") powerMode = POWER_FULL;
    saveSettings();
  }
}

char statusJsonBuf[480];
String getStatusJSON() {
  snprintf(statusJsonBuf, sizeof(statusJsonBuf),
    "{\"temp\":%.1f,\"setpoint\":%.1f,\"running\":%d,\"paused\":%d,\"tuning\":%d,\"powerMode\":\"%s\",\"currentStep\":%d,\"stepCount\":%d}",
    Input, Setpoint,
    running ? 1 : 0, paused ? 1 : 0, tuning ? 1 : 0,
    powerMode == POWER_LOW ? "Low" : (powerMode == POWER_MEDIUM ? "Medium" : "Full"),
    currentStep, stepCount
  );
  return String(statusJsonBuf);
}

unsigned long lastMqttPublish = 0;
void mqttPublish() {
  unsigned long now = millis();
  if (now - lastMqttPublish < 5000) return;
  if (mqtt.connected()) {
    mqtt.publish("wedzarnia/status", getStatusJSON().c_str());
    lastMqttPublish = now;
  }
}

// ─────────────────────────────────────────────────────────────
//          OLED – wszystkie funkcje rysujące
// ─────────────────────────────────────────────────────────────

void drawMainScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (paused && running) {
    display.setTextSize(3);
    display.setCursor(10, 20);
    display.print("PAUZA");
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("T: "); display.print(Input, 1); display.print("C");

    display.setCursor(0, 10);
    display.print("SP: "); display.print(Setpoint, 1); display.print("C");

    display.setCursor(0, 20);
    display.print("Step: "); display.print(currentStep); display.print("/"); display.print(stepCount);

    display.setCursor(0, 30);
    display.print("Run:"); display.print(running ? "ON " : "OFF");
    display.print(" Pa:"); display.print(paused ? "YES" : "NO ");

    display.setCursor(0, 40);
    display.print("Tune:"); display.print(tuning ? "YES" : "NO ");

    display.setCursor(0, 50);
    display.print("Power:");
    display.print(powerMode == POWER_LOW ? "LOW" : (powerMode == POWER_MEDIUM ? "MED" : "FULL"));

    display.setCursor(0, 60);
    display.print("Door:"); display.print(lastDoorOpen ? "OPEN" : "CLOSED");
  }
  display.display();
}

void drawMenuMain() {
  const char* items[] = {"1) Wybierz profil", "2) Podglad krokow", "3) R. sterowanie", "4) Ustaw moc", "5) Powrot"};
  int itemCount = 5;

  menuIndex = constrain(menuIndex, 0, itemCount-1);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("MENU");

  for (int i = 0; i < itemCount; i++) {
    display.setCursor(0, 12 + i * 10);
    display.print((i == menuIndex) ? ">" : " ");
    display.print(items[i]);
  }
  display.display();
}

void drawMenuProfile() {
  if (profileCount == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Brak profili");
    display.display();
    return;
  }

  selectedProfileIndex = constrain(selectedProfileIndex, 0, profileCount-1);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Wybierz profil:");

  int start = max(0, selectedProfileIndex - 1);
  int end = min(profileCount, start + 3);

  int line = 0;
  for (int i = start; i < end; i++) {
    display.setCursor(0, 12 + line * 10);
    display.print((i == selectedProfileIndex) ? ">" : " ");
    display.print(profileNames[i]);
    line++;
  }
  display.display();
}

void drawMenuSteps() {
  if (stepCount == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Brak krokow");
    display.display();
    return;
  }

  selectedStepIndex = constrain(selectedStepIndex, 0, stepCount-1);
  Step &s = steps[selectedStepIndex];

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Krok "); display.print(selectedStepIndex); display.print("/"); display.print(stepCount-1);

  display.setCursor(0, 12); display.print("t: "); display.print(s.timeMin); display.print(" min");
  display.setCursor(0, 22); display.print("T: "); display.print(s.temp, 1); display.print("C");
  display.setCursor(0, 32); display.print("Smoke: "); display.print(s.smoke ? "ON" : "OFF");
  display.setCursor(0, 42); display.print("Fan: "); display.print(s.fan > 0 ? "ON" : "OFF");

  display.display();
}

void drawMenuManual() {
  const char* items[] = {"Setpoint", "Grzalka", "Dym", "Wentylator", "Autotune PID", "Powrot"};
  int itemCount = 6;

  menuIndex = constrain(menuIndex, 0, itemCount-1);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("RECZNE STEROWANIE");

  for (int i = 0; i < itemCount; i++) {
    display.setCursor(0, 12 + i * 10);
    display.print((i == menuIndex) ? "> " : "  ");
    display.print(items[i]);

    if (i == 0) { display.print(": "); display.print(Setpoint, 1); if (editSetpoint) display.print("*"); }
    if (i == 1) { display.print(": "); display.print(manualHeater ? "ON" : "OFF"); }
    if (i == 2) { display.print(": "); display.print(manualSmoke ? "ON" : "OFF"); }
    if (i == 3) { display.print(": "); display.print(manualFan ? "ON" : "OFF"); }
  }
  display.display();
}

void drawMenuPower() {
  const char* items[] = {"Low (1 grz.)", "Medium (2 grz.)", "Full (3 grz.)"};
  int itemCount = 3;

  selectedPowerIndex = constrain(selectedPowerIndex, 0, itemCount-1);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Ustaw moc:");

  for (int i = 0; i < itemCount; i++) {
    display.setCursor(0, 12 + i * 10);
    display.print((i == selectedPowerIndex) ? ">" : " ");
    display.print(items[i]);
  }

  display.setCursor(0, 52);
  display.print("ENC: wybierz / powrot");
  display.display();
}

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 500;

void drawOLED() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_INTERVAL) return;
  lastDisplayUpdate = now;

  switch (menuState) {
    case SCREEN_MAIN:  drawMainScreen();  break;
    case MENU_MAIN:    drawMenuMain();    break;
    case MENU_PROFILE: drawMenuProfile(); break;
    case MENU_STEPS:   drawMenuSteps();   break;
    case MENU_MANUAL:  drawMenuManual();  break;
    case MENU_POWER:   drawMenuPower();   break;
  }
}

// ─────────────────────────────────────────────────────────────
//          ENKODER + PRZYCISKI
// ─────────────────────────────────────────────────────────────

void readEncoderRaw() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a != lastEncA) {
    if (a == HIGH) {
      if (b == LOW) encPos++;
      else encPos--;
    }
    lastEncA = a;
    lastEncB = b;
  }
}

void handleEncoderDelta(int diff) {
  if (diff == 0) return;

  switch (menuState) {
    case MENU_MAIN:      menuIndex            += diff; break;
    case MENU_PROFILE:   selectedProfileIndex += diff; break;
    case MENU_STEPS:     selectedStepIndex    += diff; break;
    case MENU_MANUAL:
      if (editSetpoint && menuIndex == 0) {
        Setpoint += diff;
        Setpoint = constrain(Setpoint, 20, 120);
      } else {
        menuIndex += diff;
      }
      break;
    case MENU_POWER:     selectedPowerIndex   += diff; break;
  }
}

void handleEncoderButton() {
  unsigned long now = millis();
  if (digitalRead(ENC_SW) != LOW || (now - lastBtnEnc) < debounceMs) return;
  lastBtnEnc = now;
  beep(80);

  switch (menuState) {
    case SCREEN_MAIN:
      menuState = MENU_MAIN;
      menuIndex = 0;
      break;
    case MENU_MAIN:
      if (menuIndex == 0) { loadProfileList(); menuState = MENU_PROFILE; }
      else if (menuIndex == 1) { menuState = MENU_STEPS; selectedStepIndex = 0; }
      else if (menuIndex == 2) { menuState = MENU_MANUAL; menuIndex = 0; editSetpoint = false; }
      else if (menuIndex == 3) {
        menuState = MENU_POWER;
        selectedPowerIndex = (powerMode == POWER_LOW ? 0 : (powerMode == POWER_MEDIUM ? 1 : 2));
      }
      else menuState = SCREEN_MAIN;
      break;
    case MENU_PROFILE:
      if (profileCount > 0) {
        String path = "/profiles/" + profileNames[selectedProfileIndex];
        loadProfile(path.c_str());
      }
      menuState = SCREEN_MAIN;
      break;
    case MENU_STEPS:
      menuState = SCREEN_MAIN;
      break;
    case MENU_MANUAL:
      if      (menuIndex == 0) editSetpoint = !editSetpoint;
      else if (menuIndex == 1) manualHeater = !manualHeater;
      else if (menuIndex == 2) manualSmoke  = !manualSmoke;
      else if (menuIndex == 3) manualFan    = !manualFan;
      else if (menuIndex == 4) startAutotune();
      else if (menuIndex == 5) menuState = SCREEN_MAIN;
      break;
    case MENU_POWER:
      powerMode = (selectedPowerIndex == 0 ? POWER_LOW : (selectedPowerIndex == 1 ? POWER_MEDIUM : POWER_FULL));
      saveSettings();
      menuState = SCREEN_MAIN;
      break;
  }
}

void handleButtons() {
  unsigned long now = millis();

  if (digitalRead(BTN_STARTSTOP) == LOW && now - lastBtnStart > debounceMs) {
    lastBtnStart = now;
    if (running) {
      running = false;
      turnOffHeaters();
      beep(150);
    } else {
      startProfile();
      beep(150);
    }
  }

  if (digitalRead(BTN_PAUSERES) == LOW && now - lastBtnPause > debounceMs) {
    lastBtnPause = now;
    if (running) {
      paused = !paused;
      beep(80);
    }
  }
}

// ─────────────────────────────────────────────────────────────
//          TASK OLED
// ─────────────────────────────────────────────────────────────
void taskOLED(void *parameter) {
  for (;;) {
    drawOLED();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ─────────────────────────────────────────────────────────────
//          SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== WĘDZARNIA ESP32 – START ===\n");

  pinMode(PIN_SSR1, OUTPUT);
  pinMode(PIN_SSR2, OUTPUT);
  pinMode(PIN_SSR3, OUTPUT);
  pinMode(PIN_OVEN_FAN, OUTPUT);
  pinMode(PIN_SMOKE_FAN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DOOR, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(BTN_STARTSTOP, INPUT_PULLUP);
  pinMode(BTN_PAUSERES, INPUT_PULLUP);

  turnOffHeaters();

  sensors.begin();
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  pid.SetSampleTime(1000);

  loadSettings();

  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("!!! BŁĄD KARTY SD !!!");
  }

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("!!! BŁĄD OLED !!!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("WEDZARNIA START");
  display.display();
  delay(1200);

  WiFi.begin(ssid, pass);
  unsigned long wifiTimer = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiTimer < 15000) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK → %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nBrak WiFi");
  }

  mqtt.setServer(mqttServer, 1883);
  mqtt.setCallback(mqttCallback);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getStatusJSON());
  });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("c")) {
      request->send(400, "text/plain", "NO CMD");
      return;
    }
    String c = request->getParam("c")->value();
    if      (c == "START")    startProfile();
    else if (c == "STOP")     { running = false; turnOffHeaters(); }
    else if (c == "PAUSE")    paused = true;
    else if (c == "RESUME")   paused = false;
    else if (c == "AUTOTUNE") startAutotune();
    else if (c.startsWith("PROFILE:")) {
      String name = c.substring(8);
      String path = "/profiles/" + name + ".json";
      loadProfile(path.c_str());
    }
    else if (c.startsWith("POWER:")) {
      String mode = c.substring(6);
      if      (mode == "LOW")    powerMode = POWER_LOW;
      else if (mode == "MEDIUM") powerMode = POWER_MEDIUM;
      else if (mode == "FULL")   powerMode = POWER_FULL;
      saveSettings();
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/profiles", HTTP_GET, [](AsyncWebServerRequest *request){
    File dir = SD.open("/profiles");
    StaticJsonDocument<512> doc;
    JsonArray arr = doc.createNestedArray("profiles");
    if (dir && dir.isDirectory()) {
      File entry = dir.openNextFile();
      while (entry) {
        String name = entry.name();
        if (name.endsWith(".json")) {
          name.replace("/profiles/", "");
          arr.add(name);
        }
        entry = dir.openNextFile();
      }
    }
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.begin();
  Serial.println("Serwer WWW uruchomiony");

  esp_task_wdt_init(15, true);   // 15 sekund
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(taskOLED, "OLED", 4096, NULL, 1, NULL, 0);

  Serial.println("=== SETUP ZAKOŃCZONY ===\n");
}

// ─────────────────────────────────────────────────────────────
//          GŁÓWNA PĘTLA
// ─────────────────────────────────────────────────────────────
unsigned long lastHeapLog       = 0;
unsigned long lastHeaterControl = 0;
unsigned long lastLoopCheck     = 0;
uint32_t      loopStallCounter  = 0;

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // ─── Bezpieczeństwo + temperatura ───
  doorSafety();
  requestTemperature();
  readTemperature();

  if (Input > MAX_TEMP) {
    running = tuning = false;
    turnOffHeaters();
    beep(500);
    Serial.println("!!! AWARYJNE WYŁĄCZENIE – MAX TEMP !!!");
  }

  // ─── WiFi reconnect ───
  static unsigned long lastWifi = 0;
  if (now - lastWifi > 30000) {
    lastWifi = now;
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }

  // ─── MQTT ───
  static unsigned long lastMqtt = 0;
  if (now - lastMqtt >= 20) {
    mqtt.loop();
    lastMqtt = now;
  }
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    if (mqtt.connect("wedzarnia-esp32")) mqtt.subscribe("wedzarnia/cmd");
  }
  mqttPublish();

  // ─── Enkoder + przyciski ───
  readEncoderRaw();
  int16_t diff = encPos - lastEncPos;
  if (diff != 0) {
    lastEncPos = encPos;
    handleEncoderDelta(diff);
  }
  handleEncoderButton();
  handleButtons();

  // ─── Sterowanie grzałkami (throttlowane) ───
  if (now - lastHeaterControl >= 100) {
    lastHeaterControl = now;

    if (!lastDoorOpen) {
      if (running && !paused && menuState != MENU_MANUAL) {
        runStep();
        if (!tuning) pid.Compute();
        controlHeatersStepped();
      } else if (tuning) {
        handleAutotune();
      } else if (menuState == MENU_MANUAL) {
        digitalWrite(PIN_SSR1, manualHeater ? HIGH : LOW);
        digitalWrite(PIN_SSR2, manualHeater ? HIGH : LOW);
        digitalWrite(PIN_SSR3, manualHeater ? HIGH : LOW);
        digitalWrite(PIN_SMOKE_FAN, manualSmoke  ? HIGH : LOW);
        digitalWrite(PIN_OVEN_FAN,  manualFan    ? HIGH : LOW);
      }
    } else {
      turnOffHeaters();
    }
  }

  // ─── Buzzer ───
  if (buzzerActive && now >= buzzerOffTime) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerActive = false;
  }

  // ─── Monitor heapu ───
  if (now - lastHeapLog >= 60000) {
    lastHeapLog = now;
    uint32_t free = ESP.getFreeHeap();
    uint32_t minf = ESP.getMinFreeHeap();
    Serial.printf("[HEAP] %u B wolne | min kiedykolwiek %u B\n", free, minf);

    if (free < 28000) {
      Serial.println("!!! NISKIE PAMIĘCI – RYZYKO ZAWIESZENIA !!!");
      // ewentualnie: beep(100); lub publish MQTT
    }
  }

  // ─── Bardzo prosty soft-watchdog ───
  if (now - lastLoopCheck > 30000) {
    Serial.println("!!! SOFT-WATCHDOG – restart !!!");
    ESP.restart();
  }
  lastLoopCheck = now;

  vTaskDelay(1);
}
