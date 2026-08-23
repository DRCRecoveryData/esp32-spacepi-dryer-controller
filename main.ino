#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "time.h"

#define WDT_TIMEOUT_SECONDS 5

// ===== FORWARD DECLARATIONS =====
void drawK1BaseLayout();
void renderK1DualChart();
void drawK1AdjustPage();
void updateK1Telemetry();
void applyFanState();
void startDryingLogic();
void stopDryingLogic();
void loadSettings();
void saveSettings();
void markSettingsChanged();

// Đọc cảm biến nhiệt độ chip ESP32
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

float getCpuTemperature() {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    return temperatureRead();
  #else
    uint8_t raw = temprature_sens_read();
    if (raw == 0) return 0.0;
    return (raw - 32) / 1.8;
  #endif
}

const char* MDNS_NAME = "spacepi"; 

// Cấu hình NTP Server (GMT+7)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;

// ===== CHÂN PHẦN CỨNG CHUẨN (CYD 2.8") =====
#define TFT_BL_PIN    21   // Đèn nền màn hình
#define PIN_HEATER    17   // Pad LED1 Blue (Kích Opto EL3063)
#define PIN_FAN       16   // Pad LED1 Green (PWM điều tốc Mosfet Quạt)
#define I2C_SDA       27   // Cổng CN1 (Chân 3)
#define I2C_SCL       22   // Cổng CN1 (Chân 2)

// Cảm ứng XPT2046
#define XPT2046_IRQ   36   
#define XPT2046_MOSI  32   
#define XPT2046_MISO  39   
#define XPT2046_CLK   25   
#define XPT2046_CS    33   

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define FAN_PWM_FREQ  5000
#define FAN_PWM_RES   8
#define FAN_PWM_CH    2

#define HEATER_ON     LOW   
#define HEATER_OFF    HIGH  

// ===== BẢNG MÀU CREALITY K1 OS =====
#define K1_BG         0x0842  
#define K1_CARD_BG    0x18C5  
#define K1_BORDER     0x2969  
#define K1_SIDEBAR_BG 0x0821  
#define K1_SIDEBAR_HL 0x2949  
#define K1_PILL_BG    0x10A3  
#define K1_CYAN_LINE  0x07FF  
#define K1_CYAN_FILL  0x03EB  
#define K1_ORANGE     0xFC64  
#define K1_WHITE      0xFFFF  
#define K1_TEXT_DIM   0x8C71  
#define K1_ACCENT_ON  0x07E0  
#define K1_RED        0xF8A6  
#define K1_GOLD       0xFEA0  

// ===== THÔNG SỐ ĐIỀU NHIỆT & MẶC ĐỊNH HỆ THỐNG =====
float targetTemp = 50.0;          
const float MAX_SAFE_TEMP = 55.0; // Ngưỡng dừng khẩn cấp bảo vệ máy (50-55°C)
const float COOLDOWN_TEMP = 40.0; // Ngưỡng quạt tự ngắt sau khi hạ nhiệt
int fanSpeedPercent = 50;         // Mặc định quạt 50%
bool fanEnabled = false;          
bool isCoolingDown = false;       // Cờ xả nhiệt 100% sau khi ấn STOP
bool screenBacklight = true;

// Mặc định băm xung 500ms và 0% công suất
unsigned long windowSizeMs = 500; 
unsigned long windowStartTime = 0;
float heaterDutyPercent = 0.0;
bool manualPwmMode = false;
float manualDutyPercent = 0.0;
bool isOverheatCooling = false;   // Cờ ngắt tạm thời khi vượt Profile

// ===== BẢO VỆ FLASH NVS (DEBOUNCE WRITE) =====
bool settingsNeedSave = false;
unsigned long lastSettingChange = 0;

// ===== CHẾ ĐỘ SẤY & HẸN GIỜ =====
bool isDrying = false;
bool isAutoMode = false;
float targetHum = 20.0;           
float dryingHours = 12.0;         

unsigned long dryingDuration = 12 * 3600 * 1000; 
unsigned long startTime = 0;
unsigned long remainingTime = 0;

float currentTemp = 0.0;
float currentHum = 0.0;
float cpuTemp = 0.0;
bool bmeOk = false;
int bmeErrorCount = 0;

bool isPortalMode = false;
DNSServer dnsServer;
Preferences prefs;

int currentTab = 0; 

const int MAX_POINTS = 30;
float chartTemp[MAX_POINTS];
float chartHum[MAX_POINTS];
int chartIndex = 0;

unsigned long lastTftUpdate = 0;
unsigned long lastChartPush = 0;
unsigned long lastTouchDebounce = 0;
unsigned long lastSensorRead = 0;

Adafruit_BME280 bme;
WebServer server(80);
TFT_eSPI tft = TFT_eSPI();

// ===== QUẢN LÝ LƯU TRỮ FLASH NVS =====
void loadSettings() {
  prefs.begin("dryer_cfg", true);
  targetTemp      = prefs.getFloat("targetTemp", 65.0);
  dryingHours     = prefs.getFloat("dryingHours", 12.0);
  targetHum       = prefs.getFloat("targetHum", 20.0);
  fanSpeedPercent = prefs.getInt("fanSpeed", 50);
  windowSizeMs    = prefs.getULong("windowMs", 500);
  prefs.end();
}

void saveSettings() {
  prefs.begin("dryer_cfg", false);
  prefs.putFloat("targetTemp", targetTemp);
  prefs.putFloat("dryingHours", dryingHours);
  prefs.putFloat("targetHum", targetHum);
  prefs.putInt("fanSpeed", fanSpeedPercent);
  prefs.putULong("windowMs", windowSizeMs);
  prefs.end();
}

void markSettingsChanged() {
  settingsNeedSave = true;
  lastSettingChange = millis();
}

// ===== TIỆN ÍCH THỜI GIAN & PHẦN CỨNG =====
String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--:--:--";
  char timeStr[20];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
  return String(timeStr);
}

String getFullFormattedDateTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "--/--/---- • --:--:--";
  char timeStr[35];
  strftime(timeStr, sizeof(timeStr), "%d/%m/%Y • %H:%M:%S", &timeinfo);
  return String(timeStr);
}

void applyFanState() {
  int duty = 0;
  if (isCoolingDown) {
    duty = 255;
  } else if (fanEnabled && !isOverheatCooling) {
    duty = map(fanSpeedPercent, 0, 100, 0, 255);
  }
  
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_FAN, duty);
  #else
    ledcWrite(FAN_PWM_CH, duty);
  #endif
}

void setBacklight(bool state) {
  screenBacklight = state;
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, screenBacklight ? HIGH : LOW);
}

void startDryingLogic() {
  manualPwmMode = false;
  isDrying = true;
  fanEnabled = true;
  isCoolingDown = false;
  isOverheatCooling = false;
  applyFanState();
  dryingDuration = dryingHours * 3600 * 1000;
  startTime = millis();
  windowStartTime = millis();
}

void stopDryingLogic() {
  isDrying = false;
  isAutoMode = false;
  manualPwmMode = false;
  isOverheatCooling = false;
  heaterDutyPercent = 0.0;
  manualDutyPercent = 0.0;
  digitalWrite(PIN_HEATER, HEATER_OFF);
  
  if (currentTemp > COOLDOWN_TEMP) {
    isCoolingDown = true;
    fanEnabled = false;
  } else {
    isCoolingDown = false;
    fanEnabled = false;
  }
  applyFanState();
}

// ===== THUẬT TOÁN ĐIỀU NHIỆT & BẢO VỆ FAILSAFE =====
void updateHeaterControl() {
  if (isCoolingDown) {
    if (currentTemp <= COOLDOWN_TEMP && currentTemp > 0.0) {
      isCoolingDown = false;
      applyFanState();
    }
  }

  // 1. FAILSAFE: Cảm biến ngắt kết nối / Lỗi đọc dữ liệu -> NGẮT SƯỞI NGAY
  if (!bmeOk || isnan(currentTemp) || currentTemp <= 5.0 || bmeErrorCount > 3) {
    heaterDutyPercent = 0.0;
    digitalWrite(PIN_HEATER, HEATER_OFF);
    return;
  }

  if (!isDrying && !manualPwmMode) {
    heaterDutyPercent = 0.0;
    digitalWrite(PIN_HEATER, HEATER_OFF);
    return;
  }

  // 2. Chạm ngưỡng nguy hiểm: Dừng khẩn cấp
  if (currentTemp >= MAX_SAFE_TEMP) {
    stopDryingLogic();
    if (currentTab == 1) drawK1AdjustPage();
    return;
  }

  if (manualPwmMode) {
    heaterDutyPercent = manualDutyPercent;
  } else {
    if (isAutoMode && currentHum <= targetHum && currentHum > 0.0) {
      heaterDutyPercent = 0.0;
      stopDryingLogic();
      return;
    }

    if (currentTemp >= targetTemp) {
      if (!isOverheatCooling) {
        isOverheatCooling = true;
        heaterDutyPercent = 0.0;
        applyFanState();
      }
    } else if (currentTemp <= (targetTemp - 2.0)) {
      if (isOverheatCooling) {
        isOverheatCooling = false;
        applyFanState();
      }
    }

    if (isOverheatCooling) {
      heaterDutyPercent = 0.0;
    } else {
      float error = targetTemp - currentTemp;
      if (error <= 0.0) {
        heaterDutyPercent = 0.0;
      } else if (error >= 3.0) {
        heaterDutyPercent = 100.0;
      } else if (error >= 1.0) {
        heaterDutyPercent = 50.0;
      } else {
        heaterDutyPercent = 20.0;
      }
    }
  }

  unsigned long now = millis();
  if (now - windowStartTime >= windowSizeMs) {
    windowStartTime = now;
  }

  unsigned long onDuration = (unsigned long)((heaterDutyPercent / 100.0) * windowSizeMs);
  if (now - windowStartTime < onDuration && heaterDutyPercent > 0.0 && !isOverheatCooling) {
    digitalWrite(PIN_HEATER, HEATER_ON);
  } else {
    digitalWrite(PIN_HEATER, HEATER_OFF);
  }
}

// ===== GIAO DIỆN WEB TIẾNG ANH CHUẨN =====
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Creality SpacePi Dryer</title>
  <style>
    :root {
      --bg: #090d16;
      --card-bg: rgba(20, 27, 45, 0.75);
      --border: rgba(255, 255, 255, 0.07);
      --accent-orange: #ff5e3a;
      --accent-cyan: #00f2fe;
      --accent-green: #00e676;
      --accent-red: #ff1744;
      --accent-gold: #ffd600;
      --text: #f0f4f8;
      --text-dim: #7e8b9b;
    }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: radial-gradient(circle at 50% 0%, #16203a 0%, var(--bg) 80%);
      color: var(--text);
      margin: 0; padding: 12px;
      min-height: 100vh;
      display: flex; justify-content: center;
    }
    .wrapper { width: 100%; max-width: 480px; }
    .header { display: flex; justify-content: space-between; align-items: flex-start; margin-bottom: 10px; }
    .header h1 { font-size: 1.2rem; margin: 0; font-weight: 800; }
    .current-clock { font-size: 0.78rem; color: var(--accent-cyan); font-family: monospace; margin-top: 2px; }
    
    .badge { padding: 4px 10px; border-radius: 20px; font-size: 0.72rem; font-weight: 700; text-transform: uppercase; border: 1px solid transparent; }
    .badge-off { background: rgba(255,255,255,0.05); color: var(--text-dim); border-color: var(--border); }
    .badge-run { background: rgba(0, 230, 118, 0.15); color: var(--accent-green); border-color: var(--accent-green); animation: pulse 2s infinite; }
    .badge-auto { background: rgba(255, 214, 0, 0.15); color: var(--accent-gold); border-color: var(--accent-gold); animation: pulse 2s infinite; }
    .badge-cool { background: rgba(0, 242, 254, 0.15); color: var(--accent-cyan); border-color: var(--accent-cyan); animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 0.7; } 50% { opacity: 1; } }

    .btn-ota {
      background: rgba(0, 242, 254, 0.15); border: 1px solid var(--accent-cyan); color: var(--accent-cyan);
      padding: 3px 8px; border-radius: 6px; font-size: 0.7rem; font-weight: 700; text-decoration: none; display: inline-block; margin-top: 3px;
    }

    .card {
      background: var(--card-bg);
      backdrop-filter: blur(16px);
      -webkit-backdrop-filter: blur(16px);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
      margin-bottom: 10px;
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.4);
    }
    .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; margin-bottom: 8px; }
    .metric {
      background: rgba(0, 0, 0, 0.25);
      border: 1px solid rgba(255, 255, 255, 0.03);
      border-radius: 10px;
      padding: 8px 4px;
      text-align: center;
    }
    .metric-title { font-size: 0.65rem; font-weight: 600; color: var(--text-dim); text-transform: uppercase; margin-bottom: 2px; }
    .metric-value { font-size: 1.45rem; font-weight: 800; line-height: 1.1; }
    .metric-sub { font-size: 0.68rem; margin-top: 2px; color: var(--text-dim); font-weight: 700; }

    .temp-val { color: var(--accent-orange); }
    .hum-val { color: var(--accent-cyan); }
    .cpu-val { color: var(--accent-gold); }

    .mode-tabs { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 10px; background: rgba(0,0,0,0.3); padding: 3px; border-radius: 10px; }
    .tab-btn { background: transparent; border: none; color: var(--text-dim); padding: 6px; font-weight: 700; font-size: 0.8rem; border-radius: 6px; cursor: pointer; }
    .tab-btn.active { background: rgba(255,255,255,0.1); color: #fff; }

    .timer-container {
      background: rgba(0,0,0,0.25); border-radius: 10px; padding: 8px 12px; margin: 6px 0;
    }
    .timer-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 4px; }
    .timer-digit { font-size: 1rem; font-weight: 700; color: var(--accent-gold); font-family: monospace; }
    .progress-bar-bg { width: 100%; height: 5px; background: rgba(255,255,255,0.08); border-radius: 4px; overflow: hidden; }
    .progress-bar-fill { height: 100%; width: 0%; background: linear-gradient(90deg, var(--accent-cyan), var(--accent-green)); border-radius: 4px; }

    .status-row { display: flex; justify-content: space-between; font-size: 0.75rem; color: var(--text-dim); margin-top: 6px; }
    .indicator { display: inline-flex; align-items: center; gap: 4px; }
    .spin { animation: spin 1.5s linear infinite; display: inline-block; }
    @keyframes spin { 100% { transform: rotate(360deg); } }

    .preset-group { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin: 6px 0 8px 0; }
    .btn-preset {
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid var(--border);
      color: var(--text);
      padding: 8px 0;
      font-size: 0.8rem; font-weight: 700;
      border-radius: 8px; cursor: pointer; }
    .btn-preset.active { background: rgba(255, 94, 58, 0.2); border-color: var(--accent-orange); color: var(--accent-orange); }

    .input-row { display: flex; justify-content: space-between; align-items: center; margin: 8px 0; font-size: 0.8rem; }
    .input-row label { color: var(--text-dim); font-weight: 500; }
    input[type="number"], select.custom-select {
      background: rgba(0,0,0,0.3); border: 1px solid var(--border);
      color: #fff; font-size: 0.9rem; font-weight: 600;
      width: 80px; padding: 5px; border-radius: 6px; text-align: center; outline: none;
    }

    .fan-control-box, .pwm-control-box {
      background: rgba(0,0,0,0.2);
      border: 1px solid rgba(255,255,255,0.03);
      padding: 8px 10px;
      border-radius: 10px;
      margin: 8px 0;
    }
    
    .toggle-group {
      display: inline-flex;
      background: rgba(0, 0, 0, 0.4);
      border-radius: 6px;
      padding: 2px;
      border: 1px solid var(--border);
    }
    .btn-toggle {
      background: transparent;
      border: none;
      color: var(--text-dim);
      font-size: 0.72rem;
      font-weight: 700;
      padding: 3px 8px;
      border-radius: 4px;
      cursor: pointer;
      transition: all 0.2s;
    }
    .btn-toggle.active-on { background: var(--accent-green); color: #000; }
    .btn-toggle.active-off { background: rgba(255, 255, 255, 0.15); color: var(--text); }

    .slider {
      -webkit-appearance: none; width: 100%; height: 5px; border-radius: 4px;
      background: rgba(255,255,255,0.1); outline: none; margin-top: 8px;
    }
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none; appearance: none; width: 16px; height: 16px; border-radius: 50%;
      background: var(--accent-cyan); cursor: pointer;
    }

    .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 10px; }
    .btn { border: none; padding: 11px; border-radius: 10px; font-size: 0.85rem; font-weight: 700; cursor: pointer; }
    .btn-start { background: linear-gradient(135deg, #00e676, #00b248); color: #000; }
    .btn-stop { background: linear-gradient(135deg, #ff1744, #b2102f); color: #fff; }

    canvas { width: 100%; height: 130px; background: rgba(0, 0, 0, 0.2); border-radius: 8px; }
    .chart-legend { display: flex; justify-content: center; gap: 14px; font-size: 0.72rem; margin-top: 6px; }
  </style>
</head>
<body>
  <div class="wrapper">
    <div class="header">
      <div>
        <h1>Creality SpacePi Dryer</h1>
        <div id="liveClock" class="current-clock">--/--/---- • --:--:--</div>
        <a href="/update" class="btn-ota">⚡ OTA UPDATE</a>
      </div>
      <div id="statusBadge" class="badge badge-off">READY</div>
    </div>

    <div class="card">
      <div class="grid-3">
        <div class="metric">
          <div class="metric-title">Chamber</div>
          <div class="metric-value temp-val"><span id="temp">--</span><span style="font-size: 0.75rem;">°C</span></div>
          <div class="metric-sub">T: <b id="dispTarget">65</b>°C</div>
        </div>
        <div class="metric">
          <div class="metric-title">Humidity</div>
          <div class="metric-value hum-val"><span id="hum">--</span><span style="font-size: 0.75rem;">%</span></div>
          <div class="metric-sub" id="dryState">--</div>
        </div>
        <div class="metric">
          <div class="metric-title">CPU</div>
          <div class="metric-value cpu-val"><span id="cpuTemp">--</span><span style="font-size: 0.75rem;">°C</span></div>
          <div class="metric-sub">Duty: <b id="dutyDisp" style="color:var(--accent-cyan);">0%</b></div>
        </div>
      </div>

      <div class="timer-container">
        <div class="timer-row">
          <span id="modeLabel" style="font-size: 0.75rem; color: var(--text-dim);">Progress (<span id="progPercent">0</span>%)</span>
          <span id="timer" class="timer-digit">00:00:00</span>
        </div>
        <div class="progress-bar-bg">
          <div id="progressFill" class="progress-bar-fill"></div>
        </div>
      </div>

      <div class="status-row">
        <span class="indicator">Heater: <span id="heaterIcon">⚪</span> <b id="heaterStatus">OFF</b></span>
        <span class="indicator">Fan: <span id="fanIcon">⚪</span> <b id="fanStatus">OFF</b> (<span id="fanDutyTxt">50%</span>)</span>
      </div>
    </div>

    <div class="card">
      <div class="mode-tabs">
        <button id="tabTimer" class="tab-btn active" onclick="switchControlMode(false)">⏱ TIMER MODE</button>
        <button id="tabAuto" class="tab-btn" onclick="switchControlMode(true)">💧 AUTO HUMIDITY</button>
      </div>

      <div style="font-size: 0.75rem; font-weight: 600; color: var(--text-dim); text-transform: uppercase;">Filament Profiles</div>
      <div class="preset-group">
        <button type="button" class="btn-preset active" onclick="setPreset(65, 12, this)">ABS</button>
        <button type="button" class="btn-preset" onclick="setPreset(60, 8, this)">PETG</button>
        <button type="button" class="btn-preset" onclick="setPreset(50, 8, this)">PLA</button>
      </div>

      <div class="input-row">
        <label>Drying Temperature (°C)</label>
        <input type="number" id="setTemp" value="65" min="35" max="66">
      </div>

      <div id="rowHours" class="input-row">
        <label>Drying Duration (Hours)</label>
        <input type="number" id="setHours" value="12" min="0.5" max="24" step="0.5">
      </div>

      <div id="rowAutoHum" class="input-row" style="display: none;">
        <label>Cut-off Humidity Threshold (%)</label>
        <input type="number" id="setAutoHum" value="20" min="10" max="40" step="1">
      </div>

      <div class="input-row">
        <label>PTC Cycle Window</label>
        <select id="selWindow" class="custom-select" onchange="sendWindowSize(this.value)">
          <option value="500" selected>500ms (Default / Smooth)</option>
          <option value="1000">1000ms (Fast)</option>
          <option value="3000">3000ms (Balanced)</option>
          <option value="5000">5000ms (Stock / Powerful)</option>
        </select>
      </div>

      <!-- BẢNG BĂM XUNG PTC HEATER POWER -->
      <div class="pwm-control-box">
        <div style="display:flex; justify-content:space-between; align-items:center;">
          <span style="font-weight:600; font-size:0.8rem;">PTC Heater Power</span>
          <div style="display:flex; align-items:center; gap:8px;">
            <div class="toggle-group">
              <button id="btnPwmOn" class="btn-toggle" onclick="setPwmModeState(true)">ON</button>
              <button id="btnPwmOff" class="btn-toggle active-off" onclick="setPwmModeState(false)">OFF</button>
            </div>
            <b id="dispPwmPercent" style="color: var(--accent-orange); font-size:0.8rem; min-width:35px; text-align:right;">0%</b>
          </div>
        </div>
        <input type="range" id="pwmSlider" class="slider" min="0" max="100" value="0" oninput="updatePwmLabel(this.value)" onchange="sendPwmDuty(this.value)">
      </div>

      <!-- BẢNG ĐIỀU KHIỂN QUẠT -->
      <div class="fan-control-box">
        <div style="display:flex; justify-content:space-between; align-items:center;">
          <span style="font-weight:600; font-size:0.8rem;">Fan</span>
          <div style="display:flex; align-items:center; gap:8px;">
            <div class="toggle-group">
              <button id="btnFanOn" class="btn-toggle" onclick="handleFanToggle(true)">ON</button>
              <button id="btnFanOff" class="btn-toggle active-off" onclick="handleFanToggle(false)">OFF</button>
            </div>
            <b id="dispFanPercent" style="color: var(--accent-cyan); font-size:0.8rem; min-width:35px; text-align:right;">50%</b>
          </div>
        </div>
        <input type="range" id="setFanSpeed" class="slider" min="30" max="100" value="50" oninput="updateFanLabel(this.value)" onchange="sendFanSpeed(this.value)">
      </div>

      <div class="actions">
        <button class="btn btn-start" onclick="startDrying()">START</button>
        <button class="btn btn-stop" onclick="stopDrying()">STOP</button>
      </div>
    </div>

    <div class="card">
      <div style="font-size: 0.75rem; font-weight: 600; color: var(--text-dim); text-transform: uppercase; margin-bottom: 4px;">Temperature & Humidity Graph</div>
      <canvas id="chartCanvas" width="420" height="130"></canvas>
      <div class="chart-legend">
        <span style="color: var(--accent-orange);">■ Temp (20-80°C)</span>
        <span style="color: var(--accent-cyan);">■ Humidity (0-100%)</span>
      </div>
    </div>
  </div>

<script>
  var isAutoTab = false;
  var tempList = [];
  var humList = [];
  var maxPoints = 35;
  var isDraggingPwm = false;

  function updateFanLabel(val) { document.getElementById('dispFanPercent').innerText = val + "%"; }
  function handleFanToggle(enable) { fetch('/toggleFan?enable=' + (enable ? 1 : 0)).then(updateData); }
  function sendFanSpeed(val) { fetch('/setFan?speed=' + val).then(updateData); }
  
  function updatePwmLabel(val) { 
    isDraggingPwm = true;
    document.getElementById('dispPwmPercent').innerText = val + "%"; 
    document.getElementById('dutyDisp').innerText = val + "%"; 
  }
  
  function setPwmModeState(enable) {
    var val = document.getElementById('pwmSlider').value;
    if (enable && Number(val) === 0) {
      val = 100;
      document.getElementById('pwmSlider').value = 100;
    }
    fetch('/setPwm?manual=' + (enable ? 1 : 0) + '&duty=' + val).then(updateData); 
  }

  function sendPwmDuty(val) { 
    isDraggingPwm = false;
    fetch('/setPwm?manual=1&duty=' + val).then(updateData); 
  }

  function sendWindowSize(val) {
    fetch('/setPwm?window=' + val).then(updateData);
  }

  function switchControlMode(isAuto) {
    isAutoTab = isAuto;
    document.getElementById('tabTimer').className = isAuto ? "tab-btn" : "tab-btn active";
    document.getElementById('tabAuto').className = isAuto ? "tab-btn active" : "tab-btn";
    document.getElementById('rowHours').style.display = isAuto ? "none" : "flex";
    document.getElementById('rowAutoHum').style.display = isAuto ? "flex" : "none";
  }

  function setPreset(temp, hours, btn) {
    document.getElementById('setTemp').value = temp;
    document.getElementById('setHours').value = hours;
    var all = document.querySelectorAll('.btn-preset');
    for (var i = 0; i < all.length; i++) all[i].classList.remove('active');
    if (btn) btn.classList.add('active');
  }

  function drawChart() {
    var canvas = document.getElementById('chartCanvas');
    if (!canvas) return;
    var ctx = canvas.getContext('2d');
    var w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    for (var i = 1; i <= 3; i++) {
      var y = (h / 4) * i;
      ctx.beginPath(); ctx.moveTo(30, y); ctx.lineTo(w - 30, y); ctx.stroke();
    }

    function renderCurve(data, strokeColor, gradStart, minVal, maxVal) {
      if (data.length < 2) return;
      var left = 32, usableWidth = (w - 32) - left;
      var stepX = usableWidth / (maxPoints - 1);
      ctx.beginPath();
      for (var i = 0; i < data.length; i++) {
        var norm = Math.max(0, Math.min(1, (data[i] - minVal) / (maxVal - minVal)));
        var y = h - (norm * (h - 20) + 10);
        if (i === 0) ctx.moveTo(left + i * stepX, y);
        else ctx.lineTo(left + i * stepX, y);
      }
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 2;
      ctx.stroke();
    }
    renderCurve(tempList, '#ff5e3a', 'rgba(255, 94, 58, 0.25)', 20, 80);
    renderCurve(humList, '#00f2fe', 'rgba(0, 242, 254, 0.15)', 0, 100);
  }

  function updateData() {
    fetch('/status').then(r => r.json()).then(data => {
      if (data.dateTime) document.getElementById('liveClock').innerText = data.dateTime;
      document.getElementById('temp').innerText = Number(data.temp).toFixed(1);
      document.getElementById('hum').innerText = Number(data.hum).toFixed(1);
      document.getElementById('cpuTemp').innerText = Number(data.cpuTemp).toFixed(1);
      document.getElementById('dispTarget').innerText = Number(data.target).toFixed(0);
      document.getElementById('dutyDisp').innerText = Number(data.duty).toFixed(0) + "%";
      
      var btnPwmOn = document.getElementById('btnPwmOn');
      var btnPwmOff = document.getElementById('btnPwmOff');
      if (data.manualMode) {
        btnPwmOn.className = "btn-toggle active-on";
        btnPwmOff.className = "btn-toggle";
      } else {
        btnPwmOn.className = "btn-toggle";
        btnPwmOff.className = "btn-toggle active-off";
      }

      if (!isDraggingPwm) {
        document.getElementById('pwmSlider').value = data.duty;
        document.getElementById('dispPwmPercent').innerText = Number(data.duty).toFixed(0) + "%";
      }

      var btnFanOn = document.getElementById('btnFanOn');
      var btnFanOff = document.getElementById('btnFanOff');
      if (data.fan) {
        btnFanOn.className = "btn-toggle active-on";
        btnFanOff.className = "btn-toggle";
      } else {
        btnFanOn.className = "btn-toggle";
        btnFanOff.className = "btn-toggle active-off";
      }

      if (data.windowMs) {
        document.getElementById('selWindow').value = data.windowMs;
      }

      var dryBadge = document.getElementById('dryState');
      dryBadge.innerText = (data.hum >= 13.0) ? "Wet" : "Dry";
      dryBadge.style.color = (data.hum >= 13.0) ? "var(--accent-red)" : "var(--accent-green)";

      var badge = document.getElementById('statusBadge');
      if (data.cooling) {
        badge.className = "badge badge-cool";
        badge.innerText = "COOLING (<40°C)";
      } else if (data.manualMode) {
        badge.className = "badge badge-auto";
        badge.innerText = "MANUAL PWM";
      } else if (data.isDrying) {
        badge.className = data.isAuto ? "badge badge-auto" : "badge badge-run";
        badge.innerText = data.isAuto ? "AUTO HUMIDITY" : "DRYING ACTIVE";
      } else {
        badge.className = "badge badge-off";
        badge.innerText = "READY";
      }

      document.getElementById('heaterStatus').innerText = data.heater ? "ON" : "OFF";
      document.getElementById('heaterStatus').style.color = data.heater ? "var(--accent-orange)" : "var(--text-dim)";
      document.getElementById('heaterIcon').innerText = data.heater ? "🔥" : "⚪";

      document.getElementById('fanStatus').innerText = data.fan ? "ON" : "OFF";
      document.getElementById('fanStatus').style.color = data.fan ? "var(--accent-cyan)" : "var(--text-dim)";
      document.getElementById('fanDutyTxt').innerText = data.fan ? (data.cooling ? "100%" : data.fanSpeed + "%") : "0%";

      if (data.isAuto) {
        document.getElementById('modeLabel').innerText = "Hold Humidity < " + data.targetHum + "%";
        document.getElementById('timer').innerText = data.hum <= data.targetHum ? "TARGET REACHED" : "DEHUMIDIFYING";
      } else {
        var sec = Math.floor(data.remaining / 1000);
        var h = String(Math.floor(sec / 3600)).padStart(2, '0');
        var m = String(Math.floor((sec % 3600) / 60)).padStart(2, '0');
        var s = String(sec % 60).padStart(2, '0');
        document.getElementById('timer').innerText = data.isDrying ? (h + ":" + m + ":" + s) : "00:00:00";
        var percent = (data.isDrying && data.duration > 0) ? Math.min(100, Math.max(0, Math.floor(((data.duration - data.remaining) / data.duration) * 100))) : 0;
        document.getElementById('progPercent').innerText = percent;
        document.getElementById('progressFill').style.width = percent + "%";
      }

      tempList.push(data.temp);
      humList.push(data.hum);
      if (tempList.length > maxPoints) tempList.shift();
      if (humList.length > maxPoints) humList.shift();
      drawChart();
    });
  }

  function startDrying() {
    var t = document.getElementById('setTemp').value;
    var url = '/start?temp=' + t + '&auto=' + (isAutoTab ? 1 : 0);
    url += isAutoTab ? ('&targetHum=' + document.getElementById('setAutoHum').value) : ('&hours=' + document.getElementById('setHours').value);
    fetch(url).then(updateData);
  }

  function stopDrying() { fetch('/stop').then(updateData); }
  setInterval(updateData, 1000);
  updateData();
</script>
</body>
</html>
)rawliteral";

// ===== TRANG WEB CẬP NHẬT OTA =====
const char OTA_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>SpacePi • OTA Firmware Update</title>
  <style>
    body { font-family: sans-serif; background: #090d16; color: #f0f4f8; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; }
    .box { width: 100%; max-width: 400px; background: rgba(20,27,45,0.9); border: 1px solid rgba(255,255,255,0.1); border-radius: 16px; padding: 24px; text-align: center; }
    h2 { margin-top: 0; color: #00f2fe; }
    input[type="file"] { width: 100%; margin: 20px 0; color: #7e8b9b; }
    .btn { background: #00e676; color: #000; border: none; padding: 12px 24px; border-radius: 10px; font-weight: bold; cursor: pointer; width: 100%; }
    .progress-bar { width: 100%; height: 10px; background: rgba(255,255,255,0.1); border-radius: 5px; margin: 15px 0; overflow: hidden; display: none; }
    .progress-fill { width: 0%; height: 100%; background: #00f2fe; }
  </style>
</head>
<body>
  <div class="box">
    <h2>⚡ OTA UPDATE</h2>
    <p style="font-size:0.85rem;color:#7e8b9b;">Select compiled firmware file (.bin)</p>
    <form method="POST" action="/update" enctype="multipart/form-data" id="upload_form">
      <input type="file" name="update" required>
      <div class="progress-bar" id="prg"><div class="progress-fill" id="bar"></div></div>
      <button type="submit" class="btn" id="btnSub">FLASH FIRMWARE</button>
    </form>
    <p><a href="/" style="color:#7e8b9b;font-size:0.8rem;text-decoration:none;">← Back to Dashboard</a></p>
  </div>
  <script>
    document.getElementById('upload_form').onsubmit = function(){
      document.getElementById('prg').style.display = 'block';
      document.getElementById('btnSub').innerText = 'FLASHING...';
      document.getElementById('btnSub').disabled = true;
    };
  </script>
</body>
</html>
)rawliteral";

// ===== PORTAL SETUP WIFI =====
void handlePortalRoot() {
  int n = WiFi.scanNetworks();
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>SpacePi WiFi Setup</title><style>";
  html += "body{font-family:sans-serif;background:#090d16;color:#f0f4f8;padding:20px;text-align:center;}";
  html += ".box{max-width:380px;margin:auto;background:rgba(20,27,45,0.9);padding:22px;border-radius:14px;border:1px solid rgba(255,255,255,0.1);}";
  html += "input,select{width:100%;padding:10px;margin:10px 0;box-sizing:border-box;background:#000;border:1px solid #333;color:#fff;border-radius:6px;}";
  html += "button{width:100%;padding:12px;background:#00e676;border:none;border-radius:6px;font-weight:bold;cursor:pointer;}";
  html += "</style></head><body><div class='box'>";
  html += "<h2>SpacePi WiFi Setup</h2><p style='font-size:0.85rem;color:#7e8b9b;'>Select your home WiFi network</p>";
  html += "<form action='/savewifi' method='POST'>";
  html += "<select name='ssid' onchange='if(this.value===\"custom\"){document.getElementById(\"c_ssid\").style.display=\"block\";}else{document.getElementById(\"c_ssid\").style.display=\"none\";}'>";
  
  for (int i = 0; i < n; ++i) {
    html += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
  }
  html += "<option value='custom'>-- Enter other SSID --</option></select>";
  html += "<input type='text' id='c_ssid' name='custom_ssid' placeholder='Manual SSID' style='display:none;'>";
  html += "<input type='password' name='pass' placeholder='WiFi Password'>";
  html += "<button type='submit'>SAVE & CONNECT</button></form></div></body></html>";

  server.send(200, "text/html", html);
}

void handleSaveWifi() {
  String s = server.arg("ssid");
  if (s == "custom" && server.hasArg("custom_ssid")) s = server.arg("custom_ssid");
  String p = server.arg("pass");

  prefs.begin("wifi_cfg", false);
  prefs.putString("ssid", s);
  prefs.putString("pass", p);
  prefs.end();

  server.send(200, "text/html", "<body style='background:#090d16;color:#00e676;text-align:center;padding-top:40px;'><h3>Configuration Saved! SpacePi is restarting...</h3></body>");
  delay(1500);
  ESP.restart();
}

void startCaptivePortal() {
  isPortalMode = true;
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(200);
  
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  
  IPAddress apIP(192, 168, 4, 1);
  IPAddress netMsk(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP("SpacePi_Setup", NULL, 1, 0, 4);

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP);

  server.on("/", handlePortalRoot);
  server.on("/savewifi", HTTP_POST, handleSaveWifi);
  server.onNotFound(handlePortalRoot);
  server.begin();

  tft.fillScreen(K1_BG);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_ORANGE, K1_BG);
  tft.drawString("WIFI SETUP MODE", 160, 50, 4);
  tft.setTextColor(K1_WHITE, K1_BG);
  tft.drawString("Connect to WiFi AP:", 160, 100, 2);
  tft.setTextColor(K1_CYAN_LINE, K1_BG);
  tft.drawString("SpacePi_Setup", 160, 130, 4);
  tft.setTextColor(K1_GOLD, K1_BG);
  tft.drawString("(Open Network - No Password)", 160, 160, 2);
  tft.setTextColor(K1_TEXT_DIM, K1_BG);
  tft.drawString("Open browser: 192.168.4.1", 160, 195, 2);
}

// ===== GIAO DIỆN MÀN HÌNH CYD =====
void playBootAnimation() {
  tft.fillScreen(K1_BG);
  int cx = 160, cy = 95;

  for (int frame = 0; frame < 36; frame++) {
    esp_task_wdt_reset();
    tft.drawRoundRect(cx - 50, cy - 40, 100, 75, 16, K1_BORDER);
    tft.drawCircle(cx, cy - 10, 38, K1_WHITE);
    tft.fillRect(cx - 45, cy - 35, 90, 60, K1_BG);

    tft.drawCircle(cx, cy - 10, 26, K1_CYAN_LINE);
    tft.drawCircle(cx, cy - 10, 12, K1_CARD_BG);
    tft.fillCircle(cx, cy - 10, 7, K1_BORDER);

    float rad = frame * 0.35;
    for (int i = 0; i < 4; i++) {
      float a = rad + i * 1.57;
      int x1 = cx + cos(a) * 8, y1 = (cy - 10) + sin(a) * 8;
      int x2 = cx + cos(a) * 25, y2 = (cy - 10) + sin(a) * 25;
      tft.drawLine(x1, y1, x2, y2, K1_CYAN_LINE);
    }

    int waveOffset = (frame * 3) % 18;
    for (int i = -2; i <= 2; i++) {
      int hx = cx + (i * 18);
      int hy = (cy + 26) - (waveOffset / 2);
      tft.drawPixel(hx, hy, K1_ORANGE);
      tft.drawPixel(hx + 1, hy - 1, K1_GOLD);
      tft.drawPixel(hx - 1, hy - 2, K1_ORANGE);
    }
    tft.drawFastHLine(cx - 48, cy + 32, 96, K1_WHITE);

    int barW = map(frame, 0, 35, 0, 180);
    tft.fillRoundRect(cx - 90, 185, 180, 6, 3, K1_PILL_BG);
    tft.fillRoundRect(cx - 90, 185, barW, 6, 3, K1_CYAN_LINE);

    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(K1_WHITE, K1_BG);
    tft.drawString("CREALITY SPACE PI", cx, 168, 2);
    delay(30);
  }
}

void drawSidebar() {
  tft.fillRect(0, 0, 44, 240, K1_SIDEBAR_BG);
  tft.drawFastVLine(44, 0, 240, K1_BORDER);

  if (currentTab == 0) tft.fillRoundRect(5, 38, 34, 44, 8, K1_SIDEBAR_HL);
  else tft.fillRect(5, 38, 34, 44, K1_SIDEBAR_BG);
  tft.fillTriangle(22, 46, 11, 58, 33, 58, K1_WHITE);
  tft.fillRect(14, 57, 16, 14, K1_WHITE);
  tft.fillRect(19, 63, 6, 8, currentTab == 0 ? K1_SIDEBAR_HL : K1_SIDEBAR_BG);

  if (currentTab == 1) tft.fillRoundRect(5, 158, 34, 44, 8, K1_SIDEBAR_HL);
  else tft.fillRect(5, 158, 34, 44, K1_SIDEBAR_BG);
  tft.drawCircle(22, 180, 9, K1_WHITE);
  tft.drawCircle(22, 180, 8, K1_WHITE);
  tft.fillCircle(22, 180, 4, K1_WHITE);
  for (int a = 0; a < 8; a++) {
    float rad = a * 0.785;
    tft.fillCircle(22 + cos(rad) * 11, 180 + sin(rad) * 11, 2, K1_WHITE);
  }
}

void drawFlameIcon(int x, int y) {
  tft.fillCircle(x + 6, y + 9, 5, K1_ORANGE);
  tft.fillTriangle(x + 6, y, x + 1, y + 8, x + 11, y + 8, K1_ORANGE);
  tft.fillCircle(x + 6, y + 10, 2, K1_GOLD);
  tft.fillTriangle(x + 6, y + 4, x + 4, y + 10, x + 8, y + 10, K1_GOLD);
}

void drawWindows11WifiIcon(int x, int y) {
  bool connected = (WiFi.status() == WL_CONNECTED);
  uint16_t c = connected ? K1_CYAN_LINE : K1_TEXT_DIM;
  tft.fillCircle(x, y + 10, 1, c);
  tft.drawFastHLine(x - 2, y + 7, 5, c);
  tft.drawFastHLine(x - 4, y + 4, 9, c);
  tft.drawFastHLine(x - 7, y + 1, 15, c);
}

void drawK1BaseLayout() {
  tft.fillScreen(K1_BG);
  drawSidebar();

  tft.fillRoundRect(50, 6, 178, 228, 10, K1_CARD_BG);
  tft.drawRoundRect(50, 6, 178, 228, 10, K1_BORDER);

  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(K1_TEXT_DIM, K1_CARD_BG);
  tft.drawString("100", 75, 32);
  tft.drawString("75", 75, 76);
  tft.drawString("50", 75, 120);
  tft.drawString("25", 75, 164);
  tft.drawString("0", 75, 204);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(K1_ORANGE, K1_CARD_BG);
  tft.drawString("TEMP", 82, 14);
  tft.setTextColor(K1_CYAN_LINE, K1_CARD_BG);
  tft.drawString("HUM", 135, 14);
  
  drawWindows11WifiIcon(214, 10);

  tft.fillRoundRect(234, 6, 80, 228, 10, K1_CARD_BG);
  tft.drawRoundRect(234, 6, 80, 228, 10, K1_BORDER);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_WHITE, K1_CARD_BG);
  tft.drawString("CREALITY", 274, 14, 1);
  tft.setTextColor(K1_CYAN_LINE, K1_CARD_BG);
  tft.drawString("SpacePi", 274, 24, 1);
  
  tft.setTextColor(K1_TEXT_DIM, K1_CARD_BG);
  tft.drawString("CPU:" + String((int)cpuTemp) + "C", 274, 33, 1);

  for (int i = 0; i < 3; i++) {
    tft.fillRoundRect(238, 42 + i * 38, 72, 33, 16, K1_PILL_BG);
    tft.drawRoundRect(238, 42 + i * 38, 72, 33, 16, K1_BORDER);
  }

  drawFlameIcon(244, 51);
  tft.drawTriangle(250, 89, 246, 97, 254, 97, K1_CYAN_LINE);
  tft.fillCircle(250, 97, 4, K1_CYAN_LINE);
  tft.drawCircle(250, 135, 6, K1_GOLD);
  tft.drawLine(250, 135, 250, 131, K1_GOLD);
  tft.drawLine(250, 135, 253, 135, K1_GOLD);

  tft.drawFastHLine(238, 164, 72, K1_BORDER);
  tft.fillRoundRect(242, 172, 64, 54, 8, screenBacklight ? tft.color565(40, 40, 15) : K1_PILL_BG);
  tft.drawRoundRect(242, 172, 64, 54, 8, screenBacklight ? K1_GOLD : K1_BORDER);
  
  int lx = 274, ly = 199;
  tft.drawCircle(lx, ly, 11, screenBacklight ? K1_GOLD : K1_WHITE);
  tft.drawFastHLine(lx - 5, ly + 13, 10, screenBacklight ? K1_GOLD : K1_WHITE);
  tft.drawFastHLine(lx - 3, ly + 16, 6, screenBacklight ? K1_GOLD : K1_WHITE);
}

void renderK1DualChart() {
  int chartX = 80, chartY = 32, chartW = 140, chartH = 185;
  tft.fillRect(chartX, chartY, chartW, chartH, K1_CARD_BG);

  for (int i = 0; i <= 4; i++) {
    tft.drawFastHLine(chartX, chartY + (chartH / 4) * i, chartW, 0x1927);
  }

  if (chartIndex < 2) return;
  float stepX = (float)chartW / (MAX_POINTS - 1);

  for (int i = 0; i < chartIndex - 1; i++) {
    float norm1 = constrain(chartTemp[i] / 100.0, 0.0, 1.0);
    float norm2 = constrain(chartTemp[i + 1] / 100.0, 0.0, 1.0);
    int y1 = chartY + chartH - (int)(norm1 * chartH);
    int y2 = chartY + chartH - (int)(norm2 * chartH);
    int x1 = chartX + (int)(i * stepX);
    int x2 = chartX + (int)((i + 1) * stepX);

    tft.fillTriangle(x1, y1, x2, y2, x1, chartY + chartH, tft.color565(40, 15, 10));
    tft.fillTriangle(x2, y2, x2, chartY + chartH, x1, chartY + chartH, tft.color565(40, 15, 10));
    tft.drawLine(x1, y1, x2, y2, K1_ORANGE);
  }

  for (int i = 0; i < chartIndex - 1; i++) {
    float norm1 = constrain(chartHum[i] / 100.0, 0.0, 1.0);
    float norm2 = constrain(chartHum[i + 1] / 100.0, 0.0, 1.0);
    int y1 = chartY + chartH - (int)(norm1 * chartH);
    int y2 = chartY + chartH - (int)(norm2 * chartH);
    int x1 = chartX + (int)(i * stepX);
    int x2 = chartX + (int)((i + 1) * stepX);
    tft.drawLine(x1, y1, x2, y2, K1_CYAN_LINE);
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, K1_CYAN_LINE);
  }
}

void updateK1Telemetry() {
  tft.setTextDatum(MR_DATUM);
  tft.setTextSize(2);

  tft.setTextColor(K1_ORANGE, K1_PILL_BG);
  tft.drawString(String((int)currentTemp) + "C", 305, 57);

  tft.setTextColor(K1_CYAN_LINE, K1_PILL_BG);
  tft.drawString(String((int)currentHum) + "%", 305, 95);

  tft.fillRect(258, 122, 48, 26, K1_PILL_BG);
  tft.setTextDatum(MR_DATUM);
  tft.setTextSize(1);

  if (isCoolingDown) {
    tft.setTextColor(K1_CYAN_LINE, K1_PILL_BG);
    tft.drawString("COOL", 304, 134, 2);
  } else if (isDrying || manualPwmMode) {
    tft.setTextColor(K1_GOLD, K1_PILL_BG);
    unsigned long sec = remainingTime / 1000;
    char tBuf[10];
    sprintf(tBuf, "%02lu:%02lu", sec / 3600, (sec % 3600) / 60);
    tft.drawString(manualPwmMode ? "PWM" : String(tBuf), 304, 134, 2);
  } else {
    tft.setTextColor(K1_TEXT_DIM, K1_PILL_BG);
    tft.drawString("00:00", 304, 134, 2);
  }
}

void drawK1AdjustPage() {
  tft.fillScreen(K1_BG);
  drawSidebar();

  tft.fillRoundRect(50, 6, 264, 228, 8, K1_CARD_BG);
  tft.drawRoundRect(50, 6, 264, 228, 8, K1_BORDER);

  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(K1_CYAN_LINE, K1_CARD_BG);
  tft.drawString("TIME: " + getFormattedTime() + " | CPU:" + String((int)cpuTemp) + "C", 58, 14, 2);

  tft.fillRoundRect(236, 10, 68, 20, 4, tft.color565(40, 20, 10));
  tft.drawRoundRect(236, 10, 68, 20, 4, K1_ORANGE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_ORANGE, tft.color565(40, 20, 10));
  tft.drawString("WIFI AP", 270, 20, 1);

  auto drawPreset = [](int x, const char* name, bool active) {
    tft.fillRoundRect(x, 36, 76, 26, 4, active ? K1_CYAN_FILL : K1_PILL_BG);
    tft.drawRoundRect(x, 36, 76, 26, 4, active ? K1_CYAN_LINE : K1_BORDER);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(active ? K1_WHITE : K1_TEXT_DIM, active ? K1_CYAN_FILL : K1_PILL_BG);
    tft.drawString(name, x + 38, 49, 2);
  };
  drawPreset(60, "ABS", (targetTemp == 50.0 && dryingHours == 12.0));
  drawPreset(144, "PETG", (targetTemp == 50.0 && dryingHours == 8.0));
  drawPreset(228, "PLA", (targetTemp == 45.0 && dryingHours == 8.0));

  tft.fillRoundRect(60, 68, 118, 46, 6, K1_PILL_BG);
  tft.drawRoundRect(60, 68, 118, 46, 6, K1_BORDER);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(K1_TEXT_DIM, K1_PILL_BG);
  tft.drawString("TEMP (C)", 66, 72, 1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_WHITE, K1_PILL_BG);
  tft.drawString("-", 76, 95, 2);
  tft.setTextColor(K1_ORANGE, K1_PILL_BG);
  tft.drawString(String((int)targetTemp), 119, 94, 2);
  tft.setTextColor(K1_WHITE, K1_PILL_BG);
  tft.drawString("+", 162, 95, 2);

  tft.fillRoundRect(186, 68, 118, 46, 6, K1_PILL_BG);
  tft.drawRoundRect(186, 68, 118, 46, 6, K1_BORDER);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(K1_TEXT_DIM, K1_PILL_BG);
  tft.drawString("DURATION (H)", 192, 72, 1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_WHITE, K1_PILL_BG);
  tft.drawString("-", 202, 95, 2);
  tft.setTextColor(K1_CYAN_LINE, K1_PILL_BG);
  tft.drawString(String((int)dryingHours), 245, 94, 2);
  tft.setTextColor(K1_WHITE, K1_PILL_BG);
  tft.drawString("+", 288, 95, 2);

  bool fanIsVisualOn = (fanEnabled && !isOverheatCooling) || isCoolingDown;
  tft.fillRoundRect(60, 120, 118, 42, 6, fanIsVisualOn ? K1_CYAN_FILL : K1_PILL_BG);
  tft.drawRoundRect(60, 120, 118, 42, 6, fanIsVisualOn ? K1_CYAN_LINE : K1_BORDER);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fanIsVisualOn ? K1_WHITE : K1_TEXT_DIM, fanIsVisualOn ? K1_CYAN_FILL : K1_PILL_BG);
  tft.drawString(fanIsVisualOn ? "FAN: ON" : "FAN: OFF", 119, 141, 2);

  tft.fillRoundRect(186, 120, 118, 42, 6, K1_PILL_BG);
  tft.drawRoundRect(186, 120, 118, 42, 6, K1_BORDER);
  tft.setTextColor(K1_CYAN_LINE, K1_PILL_BG);
  tft.drawString(String(fanSpeedPercent) + "%", 245, 141, 2);

  tft.fillRoundRect(60, 170, 118, 52, 6, (isDrying || manualPwmMode) ? K1_PILL_BG : K1_ACCENT_ON);
  tft.drawRoundRect(60, 170, 118, 52, 6, (isDrying || manualPwmMode) ? K1_BORDER : K1_ACCENT_ON);
  tft.setTextColor((isDrying || manualPwmMode) ? K1_TEXT_DIM : TFT_BLACK, (isDrying || manualPwmMode) ? K1_PILL_BG : K1_ACCENT_ON);
  tft.drawString("START", 119, 196, 2);

  tft.fillRoundRect(186, 170, 118, 52, 6, (isDrying || manualPwmMode) ? K1_RED : K1_PILL_BG);
  tft.drawRoundRect(186, 170, 118, 52, 6, (isDrying || manualPwmMode) ? K1_RED : K1_BORDER);
  tft.setTextColor((isDrying || manualPwmMode) ? K1_WHITE : K1_TEXT_DIM, (isDrying || manualPwmMode) ? K1_RED : K1_PILL_BG);
  tft.drawString("STOP", 245, 196, 2);
}

void handleTouchEvents() {
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    int touchX = constrain(map(p.x, 200, 3700, 1, 320), 1, 320);
    int touchY = constrain(map(p.y, 240, 3800, 1, 240), 1, 240);

    if (millis() - lastTouchDebounce < 250) return;
    lastTouchDebounce = millis();

    if (touchX <= 46) {
      if (touchY <= 120 && currentTab != 0) {
        currentTab = 0;
        drawK1BaseLayout();
        renderK1DualChart();
      } else if (touchY > 120 && currentTab != 1) {
        currentTab = 1;
        drawK1AdjustPage();
      }
      return;
    }

    if (currentTab == 0) {
      if (touchX >= 240 && touchX <= 310 && touchY >= 170 && touchY <= 230) {
        setBacklight(!screenBacklight);
        drawK1BaseLayout();
        renderK1DualChart();
      }
    } else if (currentTab == 1) {
      if (touchX >= 230 && touchY <= 32) {
        startCaptivePortal();
        return;
      }
      if (touchY >= 34 && touchY <= 64) {
        if (touchX >= 58 && touchX <= 138) { targetTemp = 65.0; dryingHours = 12.0; markSettingsChanged(); }
        else if (touchX >= 142 && touchX <= 222) { targetTemp = 60.0; dryingHours = 8.0; markSettingsChanged(); }
        else if (touchX >= 226 && touchX <= 306) { targetTemp = 50.0; dryingHours = 8.0; markSettingsChanged(); }
        drawK1AdjustPage();
      } else if (touchY >= 68 && touchY <= 116 && touchX <= 180) {
        if (touchX <= 100) targetTemp = constrain(targetTemp - 1.0, 35.0, 66.0);
        else targetTemp = constrain(targetTemp + 1.0, 35.0, 66.0);
        markSettingsChanged();
        drawK1AdjustPage();
      } else if (touchY >= 68 && touchY <= 116 && touchX > 180) {
        if (touchX <= 230) dryingHours = constrain(dryingHours - 0.5, 0.5, 24.0);
        else dryingHours = constrain(dryingHours + 0.5, 0.5, 24.0);
        markSettingsChanged();
        drawK1AdjustPage();
      } else if (touchY >= 118 && touchY <= 164) {
        if (touchX <= 180) { fanEnabled = !fanEnabled; isCoolingDown = false; applyFanState(); }
        else { 
          fanSpeedPercent = (fanSpeedPercent == 100) ? 50 : 100; 
          markSettingsChanged();
          applyFanState(); 
        }
        drawK1AdjustPage();
      } else if (touchY >= 168 && touchY <= 226) {
        if (touchX <= 180 && !isDrying && !manualPwmMode) startDryingLogic();
        else if (touchX > 180 && (isDrying || manualPwmMode)) stopDryingLogic();
        drawK1AdjustPage();
      }
    }
  }
}

// ===== API & ENDPOINTS =====
void handleRoot() { server.send_P(200, "text/html", HTML_PAGE); }

void handleStatus() {
  String json = "{";
  json += "\"dateTime\":\"" + getFullFormattedDateTime() + "\",";
  json += "\"temp\":" + String(currentTemp, 1) + ",";
  json += "\"hum\":" + String(currentHum, 1) + ",";
  json += "\"cpuTemp\":" + String(cpuTemp, 1) + ",";
  json += "\"target\":" + String(targetTemp, 0) + ",";
  json += "\"targetHum\":" + String(targetHum, 0) + ",";
  json += "\"isDrying\":" + String(isDrying ? "true" : "false") + ",";
  json += "\"isAuto\":" + String(isAutoMode ? "true" : "false") + ",";
  json += "\"manualMode\":" + String(manualPwmMode ? "true" : "false") + ",";
  json += "\"cooling\":" + String(isCoolingDown ? "true" : "false") + ",";
  json += "\"duty\":" + String(heaterDutyPercent, 0) + ",";
  json += "\"windowMs\":" + String(windowSizeMs) + ",";
  json += "\"heater\":" + String((digitalRead(PIN_HEATER) == HEATER_ON && !isOverheatCooling) ? "true" : "false") + ",";
  json += "\"fan\":" + String(((fanEnabled && !isOverheatCooling) || isCoolingDown) ? "true" : "false") + ",";
  json += "\"fanSpeed\":" + String(fanSpeedPercent) + ",";
  json += "\"fanEnabled\":" + String(fanEnabled ? "true" : "false") + ",";
  json += "\"duration\":" + String(dryingDuration) + ",";
  json += "\"remaining\":" + String(isDrying ? remainingTime : 0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleToggleFan() {
  if (server.hasArg("enable")) {
    fanEnabled = (server.arg("enable").toInt() == 1);
    isCoolingDown = false;
    applyFanState();
    if (currentTab == 1) drawK1AdjustPage();
  }
  server.send(200, "text/plain", "OK");
}

void handleSetFan() {
  if (server.hasArg("speed")) {
    fanSpeedPercent = server.arg("speed").toInt();
    markSettingsChanged();
    applyFanState();
    if (currentTab == 1) drawK1AdjustPage();
  }
  server.send(200, "text/plain", "OK");
}

void handleSetPwm() {
  if (server.hasArg("window")) {
    windowSizeMs = constrain(server.arg("window").toInt(), 200, 10000);
    markSettingsChanged();
  }
  if (server.hasArg("duty")) {
    manualDutyPercent = constrain(server.arg("duty").toFloat(), 0.0, 100.0);
    heaterDutyPercent = manualDutyPercent;
  }
  if (server.hasArg("manual")) {
    manualPwmMode = (server.arg("manual").toInt() == 1);
    if (manualPwmMode) {
      isDrying = false;
      isCoolingDown = false;
      fanEnabled = true;
      applyFanState();
    } else {
      manualDutyPercent = 0.0;
      stopDryingLogic();
    }
  }
  if (currentTab == 1) drawK1AdjustPage();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  manualPwmMode = false;
  if (server.hasArg("temp")) targetTemp = server.arg("temp").toFloat();
  if (server.hasArg("auto")) isAutoMode = (server.arg("auto").toInt() == 1);

  if (isAutoMode) {
    if (server.hasArg("targetHum")) targetHum = server.arg("targetHum").toFloat();
  } else {
    if (server.hasArg("hours")) dryingHours = server.arg("hours").toFloat();
  }

  markSettingsChanged();
  startDryingLogic();
  if (currentTab == 1) drawK1AdjustPage();
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopDryingLogic();
  if (currentTab == 1) drawK1AdjustPage();
  server.send(200, "text/plain", "STOPPED");
}

// ===== SETUP & MAIN LOOP =====
void setup() {
  Serial.begin(115200);

  // 1. Cấu hình bảo vệ ngắt gia nhiệt ngay lập tức khi cấp nguồn
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, HEATER_OFF);

  // 2. Nạp cấu hình từ Flash
  loadSettings();
  setBacklight(true);

  // 3. Khởi tạo Watchdog Timer (WDT) 5 giây tương thích ESP32 Core v2.x & v3.x
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&wdt_config);
  #else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  #endif
  esp_task_wdt_add(NULL);

  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);

  tft.init();
  tft.setRotation(1);

  playBootAnimation();

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_FAN, FAN_PWM_FREQ, FAN_PWM_RES);
  #else
    ledcSetup(FAN_PWM_CH, FAN_PWM_FREQ, FAN_PWM_RES);
    ledcAttachPin(PIN_FAN, FAN_PWM_CH);
  #endif
  applyFanState();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50); // Timeout chống lock bus I2C
  bmeOk = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);

  prefs.begin("wifi_cfg", true);
  String savedSSID = prefs.getString("ssid", "");
  String savedPass = prefs.getString("pass", "");
  prefs.end();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);

  if (savedSSID.length() == 0) {
    startCaptivePortal();
    return;
  }

  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(K1_WHITE, K1_BG);
  tft.drawString("CONNECTING WIFI...", 160, 210, 2);

  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPass.c_str());
  
  unsigned long startWifiTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startWifiTime < 6000)) {
    esp_task_wdt_reset();
    delay(200);
  }

  if (WiFi.status() != WL_CONNECTED) {
    startCaptivePortal();
    return;
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  MDNS.begin(MDNS_NAME);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/setFan", handleSetFan);
  server.on("/toggleFan", handleToggleFan);
  server.on("/setPwm", handleSetPwm);

  server.on("/update", HTTP_GET, []() {
    server.send_P(200, "text/html", OTA_PAGE);
  });
  
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", (Update.hasError()) ? "<body style='background:#090d16;color:#ff1744;text-align:center;padding-top:40px;'><h3>OTA FLASH FAILED!</h3></body>" : "<body style='background:#090d16;color:#00e676;text-align:center;padding-top:40px;'><h3>OTA UPDATE SUCCESSFUL! Restarting...</h3></body>");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      digitalWrite(PIN_HEATER, HEATER_OFF);
      tft.fillScreen(K1_BG);
      tft.setTextSize(1);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(K1_ORANGE, K1_BG);
      tft.drawString("OTA FIRMWARE FLASHING...", 160, 100, 2);
      tft.drawRoundRect(60, 130, 200, 14, 4, K1_BORDER);

      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      esp_task_wdt_reset();
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
      int progress = (Update.progress() * 100) / Update.size();
      int w = map(progress, 0, 100, 0, 196);
      tft.fillRoundRect(62, 132, w, 10, 2, K1_CYAN_LINE);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        tft.setTextColor(K1_ACCENT_ON, K1_BG);
        tft.drawString("OTA SUCCESS! REBOOTING...", 160, 170, 2);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  drawK1BaseLayout();
  windowStartTime = millis();
}

void loop() {
  // Feed Watchdog Timer
  esp_task_wdt_reset();

  if (isPortalMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    handleTouchEvents();
    delay(10);
    return;
  }

  server.handleClient();
  handleTouchEvents();

  // Kiểm tra lưu cấu hình an toàn cho Flash sau 3s dừng thao tác
  if (settingsNeedSave && (millis() - lastSettingChange > 3000)) {
    saveSettings();
    settingsNeedSave = false;
  }

  updateHeaterControl();

  // Đọc BME280 chu kỳ 500ms
  if (millis() - lastSensorRead >= 500) {
    lastSensorRead = millis();
    if (bmeOk) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      if (!isnan(t) && !isnan(h) && t > 0.0) {
        currentTemp = t;
        currentHum = h;
        bmeErrorCount = 0;
      } else {
        bmeErrorCount++;
      }
    } else {
      bmeOk = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
    }
    cpuTemp = getCpuTemperature();
  }

  // Quản lý thời gian chạy chế độ sấy
  if (isDrying && !isAutoMode && !manualPwmMode) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed >= dryingDuration) {
      stopDryingLogic();
      if (currentTab == 1) drawK1AdjustPage();
    } else {
      remainingTime = dryingDuration - elapsed;
    }
  }

  // Cập nhật biểu đồ nhiệt độ chu kỳ 3s
  if (millis() - lastChartPush >= 3000) {
    lastChartPush = millis();
    if (chartIndex < MAX_POINTS) {
      chartTemp[chartIndex] = currentTemp;
      chartHum[chartIndex] = currentHum;
      chartIndex++;
    } else {
      for (int i = 0; i < MAX_POINTS - 1; i++) {
        chartTemp[i] = chartTemp[i + 1];
        chartHum[i] = chartHum[i + 1];
      }
      chartTemp[MAX_POINTS - 1] = currentTemp;
      chartHum[MAX_POINTS - 1] = currentHum;
    }
    if (currentTab == 0) renderK1DualChart();
  }

  // Cập nhật màn hình chính chu kỳ 500ms
  if (millis() - lastTftUpdate >= 500) {
    lastTftUpdate = millis();
    if (currentTab == 0) {
      updateK1Telemetry();
    }
  }

  delay(5);
}
