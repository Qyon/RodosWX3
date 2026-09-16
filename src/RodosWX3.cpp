///////////////////////////////////////////////////////////////////////////////////////////////////
// RodosWX3.ino
// 
// Integracja: Bresser + ESP8266/ESP32 + BME280/BMP280 + CC1101/SX1276/SX1262 -> APRS
// + Zapis PEŁNEJ konfiguracji w LittleFS przez Panel WWW
// + Tryb Access Point (Fallback) przy braku WiFi
// + Watchdog (Restart po 5 min bez ramek radiowych)
// + Dynamiczne definiowanie pinów (OLED na szynie 0, Baro na szynie 1)
// + Czytelny interfejs OLED z dużymi czcionkami
// + Algorytm opadów NTP - niezależny od interwału ramek radiowych
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

// --- WYKRYWANIE PLATFORMY I BIBLIOTEKI SIECIOWE ---
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #error "Ten kod obsluguje wylacznie mikrokontrolery z rodziny ESP8266 oraz ESP32!"
#endif

#include <time.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <Adafruit_BMP280.h> 
#include <LittleFS.h> 
#include <ESPAsyncWebServer.h>

// --- BIBLIOTEKI DLA EKRANU OLED ---
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool display_available = false;
unsigned long last_display_time = 0;
int display_page = 0;

// --- STRUKTURA KONFIGURACJI ---
struct AppConfig {
    uint32_t sensor_id = 0;
    String wifi_ssid = "";
    String wifi_pass = "";
    String aprs_callsign = "N0CALL";
    int aprs_ssid = 13;
    String aprs_passcode = "12345";
    int report_interval_min = 15;
    double lat = 52.0000;
    double lon = 16.0000;
    bool use_kiss = false;
    String server_host = "rotate.aprs2.net";
    uint16_t server_port = 14580;
    
    int oled_sda = -1;
    int oled_scl = -1;
    int oled_rst = -1;
    int oled_pwr = -1;
    int oled_interval = 4;
    
    int bme_sda = -1;
    int bme_scl = -1;
} config;

void saveConfig() {
    File f = LittleFS.open("/config_v5.txt", "w");
    if (f) {
        f.println(String(config.sensor_id, HEX));
        f.println(config.wifi_ssid);
        f.println(config.wifi_pass);
        f.println(config.aprs_callsign);
        f.println(String(config.aprs_ssid));
        f.println(config.aprs_passcode);
        f.println(String(config.report_interval_min));
        f.println(String(config.lat, 4));
        f.println(String(config.lon, 4));
        f.println(config.use_kiss ? "1" : "0");
        f.println(config.server_host);
        f.println(String(config.server_port));
        
        f.println(String(config.oled_sda));
        f.println(String(config.oled_scl));
        f.println(String(config.oled_rst));
        f.println(String(config.oled_pwr));
        f.println(String(config.oled_interval));
        f.println(String(config.bme_sda));
        f.println(String(config.bme_scl));
        
        f.close();
        Serial.println(F("[SYSTEM] Konfiguracja zapisana."));
    }
}

void loadConfig() {
    if (LittleFS.exists("/config_v5.txt")) {
        File f = LittleFS.open("/config_v5.txt", "r");
        if (f) {
            config.sensor_id = strtoul(f.readStringUntil('\n').c_str(), NULL, 16);
            config.wifi_ssid = f.readStringUntil('\n'); config.wifi_ssid.trim();
            config.wifi_pass = f.readStringUntil('\n'); config.wifi_pass.trim();
            config.aprs_callsign = f.readStringUntil('\n'); config.aprs_callsign.trim();
            config.aprs_ssid = f.readStringUntil('\n').toInt();
            config.aprs_passcode = f.readStringUntil('\n'); config.aprs_passcode.trim();
            config.report_interval_min = f.readStringUntil('\n').toInt();
            config.lat = f.readStringUntil('\n').toFloat();
            config.lon = f.readStringUntil('\n').toFloat();
            config.use_kiss = (f.readStringUntil('\n').toInt() == 1);
            config.server_host = f.readStringUntil('\n'); config.server_host.trim();
            config.server_port = f.readStringUntil('\n').toInt();
            
            String s_osda = f.readStringUntil('\n'); if(s_osda.length() > 0) config.oled_sda = s_osda.toInt();
            String s_oscl = f.readStringUntil('\n'); if(s_oscl.length() > 0) config.oled_scl = s_oscl.toInt();
            String s_orst = f.readStringUntil('\n'); if(s_orst.length() > 0) config.oled_rst = s_orst.toInt();
            String s_opwr = f.readStringUntil('\n'); if(s_opwr.length() > 0) config.oled_pwr = s_opwr.toInt();
            String s_oint = f.readStringUntil('\n'); if(s_oint.length() > 0) config.oled_interval = s_oint.toInt();
            String s_bsda = f.readStringUntil('\n'); if(s_bsda.length() > 0) config.bme_sda = s_bsda.toInt();
            String s_bscl = f.readStringUntil('\n'); if(s_bscl.length() > 0) config.bme_scl = s_bscl.toInt();
            
            f.close();
            Serial.println(F("[SYSTEM] Wczytano konfiguracje z pamieci."));
        }
    } else {
        Serial.println(F("[SYSTEM] Brak pliku konfiguracyjnego v5. Ladowanie domyslnych."));
        saveConfig();
    }
}

#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC  0xDD

WeatherSensor ws;

// --- ZMIENNE CZUJNIKA CISNIENIA ---
Adafruit_BME280 bme;
Adafruit_BMP280 *bmp = nullptr;
int baro_sensor_type = 0; // 0 = brak, 1 = BME280, 2 = BMP280

AsyncWebServer server(80);

float wind_speed_sum = 0.0;
int wind_sample_count = 0;
float wind_gust_max_period = 0.0;
int baro_hpa = 0;
bool shouldReboot = false;
bool in_ap_mode = false;

struct WeatherData {
  float temp_c = NAN;
  uint8_t humidity = 0;
  float wind_dir = NAN;
  float rain_total_mm = NAN;
  float uv_index = NAN;
  float light_klx = NAN;
  int radio_rssi = -255;
  bool battery_ok = true; 
  bool valid_data = false;
} current_wx;

// =========================================================================
// --- SYSTEM REJESTRACJI OPADÓW OPARTY NA ZEGARZE NTP ---------------------
// =========================================================================
#define RAIN_SAMPLES 144
struct RainRecord {
    uint32_t timestamp;
    float cum_rain;
};
RainRecord rain_history[RAIN_SAMPLES];
int rain_hist_idx = 0;
uint32_t last_rain_save_time = 0;

int calc_r1h = 0;
int calc_p24h = 0;
int calc_lum = 0;

unsigned long last_report_time = 0;
unsigned long last_frame_time = 0; 
static const unsigned long NO_DATA_TIMEOUT_MS = 5UL * 60UL * 1000UL; 

#define MAX_SEEN_IDS 5
uint32_t seen_ids[MAX_SEEN_IDS] = {0};

void addSeenId(uint32_t id) {
    if (id == 0) return;
    for (int i = 0; i < MAX_SEEN_IDS; i++) {
        if (seen_ids[i] == id) return; 
    }
    for (int i = 0; i < MAX_SEEN_IDS; i++) {
        if (seen_ids[i] == 0) { 
            seen_ids[i] = id; 
            return; 
        }
    }
    seen_ids[MAX_SEEN_IDS - 1] = id; 
}

float get_rain_delta(uint32_t target_sec, float current_rain) {
    uint32_t min_diff = 0xFFFFFFFF;
    float base_rain = current_rain;
    bool found_close = false;
    
    for (int i=0; i<RAIN_SAMPLES; i++) {
        uint32_t t = rain_history[i].timestamp;
        if (t > 0) {
            uint32_t diff = (t > target_sec) ? (t - target_sec) : (target_sec - t);
            if (diff < min_diff && diff <= 3600) { 
                min_diff = diff;
                base_rain = rain_history[i].cum_rain;
                found_close = true;
            }
        }
    }
    
    if (!found_close) {
        uint32_t oldest_t = 0xFFFFFFFF;
        for (int i=0; i<RAIN_SAMPLES; i++) {
            uint32_t t = rain_history[i].timestamp;
            if (t > 0 && t < oldest_t) {
                oldest_t = t;
                base_rain = rain_history[i].cum_rain;
            }
        }
    }
    
    float diff = current_rain - base_rain;
    if (diff < 0) diff = current_rain; 
    return diff;
}

void updateDisplay() {
    if (!display_available) return;
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    switch(display_page) {
        case 0:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- RodosWX3 ---"));
            display.println("");
            display.printf("WiFi: %s\n", in_ap_mode ? "Tryb AP (Setup)" : (WiFi.status() == WL_CONNECTED ? "Polaczono" : "Brak"));
            if (in_ap_mode) {
                display.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
            } else if (WiFi.status() == WL_CONNECTED) {
                display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
            }
            display.printf("Nadajnik: %08X\n", config.sensor_id);
            display.printf("Siec: %s\n", config.use_kiss ? "KISS TNC" : "APRS-IS");
            break;
            
        case 1:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- TEMPERATURA ---"));
            display.setTextSize(3);
            display.setCursor(0, 25);
            display.printf("%.1f C", current_wx.temp_c);
            break;
            
        case 2:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- WILGOTNOSC ---"));
            display.setTextSize(3);
            display.setCursor(0, 25);
            display.printf("%d %%", current_wx.humidity);
            break;
            
        case 3:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- CISNIENIE ---"));
            display.setTextSize(2);
            display.setCursor(0, 25);
            if (baro_hpa > 0) {
                display.printf("%.1f hPa", baro_hpa / 10.0);
            } else {
                display.println(F("-- hPa"));
            }
            break;
            
        case 4: {
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- WIATR ---"));
            float avg_w = (wind_sample_count > 0) ? (wind_speed_sum / wind_sample_count) : 0.0;
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.printf("%.1f m/s\n", avg_w);
            display.printf("Kier: %.0f", current_wx.wind_dir);
            break;
        }
            
        case 5:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- OPAD ---"));
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.printf("1h: %.1f mm\n", calc_r1h * 0.254);
            display.printf("24h: %.1f mm", calc_p24h * 0.254);
            break;
            
        case 6:
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println(F("--- SLONCE ---"));
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.printf("%d W/m2\n", calc_lum);
            if (!isnan(current_wx.uv_index)) {
                display.printf("UV: %.1f", current_wx.uv_index);
            }
            break;
    }
    display.display();
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RodosWX3 - Panel Stacji</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; margin: 0; padding: 20px; }
    h2 { text-align: center; color: #4CAF50; border-bottom: 2px solid #333; padding-bottom: 10px;}
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-top: 20px; }
    .card { background-color: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); text-align: center; }
    .card h3 { margin-top: 0; color: #aaaaaa; font-size: 1.1rem; border-bottom: 1px solid #333; padding-bottom: 5px; }
    .value { font-size: 2rem; font-weight: bold; color: #4CAF50; }
    .unit { font-size: 1rem; color: #888; }
    .status { margin-top: 20px; padding: 15px; background: #1e1e1e; border-radius: 10px; font-size: 0.9rem; color: #bbb; }
    .alert { color: #f44336; font-weight: bold; }
    .cfg-group { text-align: left; margin: 10px 0; }
    .cfg-group label { display: block; margin-bottom: 5px; color: #aaa; }
    select, button, input { width: 100%; box-sizing: border-box; padding: 10px; margin-bottom: 15px; border-radius: 5px; border: 1px solid #555; background: #333; color: white; }
    button { background: #4CAF50; border: none; cursor: pointer; font-weight: bold; font-size: 1.1rem; transition: background 0.3s; }
    button:hover { background: #45a049; }
    .cfg-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    @media (max-width: 600px) { .cfg-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <h2>📡 RodosWX3 Dashboard</h2>
  <div class="grid">
    <div class="card"><h3>Temperatura</h3><span class="value" id="temp">--</span><span class="unit"> &deg;C</span></div>
    <div class="card"><h3>Wilgotność</h3><span class="value" id="hum">--</span><span class="unit"> %</span></div>
    <div class="card"><h3>Ciśnienie</h3><span class="value" id="baro">--</span><span class="unit"> hPa</span></div>
    <div class="card"><h3>Wiatr (Poryw)</h3><span class="value" id="wind">--</span><span class="unit"> m/s</span></div>
    <div class="card"><h3>Kierunek Wiatru</h3><span class="value" id="dir">--</span><span class="unit"> &deg;</span></div>
    <div class="card"><h3>Opad (1h)</h3><span class="value" id="rain1">--</span><span class="unit"> mm</span></div>
    <div class="card"><h3>Opad (24h)</h3><span class="value" id="rain24">--</span><span class="unit"> mm</span></div>
    <div class="card"><h3>Słońce</h3><span class="value" id="lum">--</span><span class="unit"> W/m&sup2;</span></div>
    <div class="card"><h3>Indeks UV</h3><span class="value" id="uv">--</span><span class="unit"> </span></div>
    
    <div class="card" style="grid-column: 1 / -1;">
      <h3>Zarządzanie Stacją (Wybór nadajnika)</h3>
      <p>Nasłuchiwana stacja: <strong class="value" id="current_id" style="font-size:1.5rem;">Dowolna</strong></p>
      <div class="cfg-group">
        <label>Wykryte w okolicy:</label>
        <select id="id_selector">
            <option value="0">Dowolna (Słuchaj wszystkich)</option>
        </select>
        <button onclick="setStationId()">Zapisz wybrane ID stacji</button>
      </div>
    </div>
    
    <div class="card" style="grid-column: 1 / -1;">
      <h3>Główna Konfiguracja Stacji</h3>
      <div class="cfg-grid">
          <div class="cfg-group"><label>WiFi SSID:</label><input type="text" id="cfg_ssid"></div>
          <div class="cfg-group"><label>Hasło WiFi:</label><input type="password" id="cfg_pass"></div>
          <div class="cfg-group"><label>Znak (Callsign):</label><input type="text" id="cfg_call"></div>
          <div class="cfg-group"><label>SSID (np. 13):</label><input type="number" id="cfg_call_ssid"></div>
          <div class="cfg-group"><label>Passcode APRS:</label><input type="text" id="cfg_passcode"></div>
          <div class="cfg-group"><label>Interwał APRS (minuty):</label><input type="number" id="cfg_interval"></div>
          <div class="cfg-group"><label>Lat (Szerokość):</label><input type="number" step="0.0001" id="cfg_lat"></div>
          <div class="cfg-group"><label>Lon (Długość):</label><input type="number" step="0.0001" id="cfg_lon"></div>
          <div class="cfg-group">
            <label>Tryb Sieciowy:</label>
            <select id="cfg_mode">
                <option value="0">APRS-IS (TCP)</option>
                <option value="1">KISS (Share-TNC)</option>
            </select>
          </div>
          <div class="cfg-group"><label>Serwer/IP:</label><input type="text" id="cfg_host"></div>
          <div class="cfg-group"><label>Port:</label><input type="number" id="cfg_port"></div>
      </div>
    </div>

    <div class="card" style="grid-column: 1 / -1;">
      <h3>Konfiguracja Sprzętowa (Piny I2C)</h3>
      <p style="font-size: 0.9rem; color: #aaa; margin-bottom: 15px;">Wpisz -1, aby użyć domyślnych pinów dla wybranej płytki.</p>
      <div class="cfg-grid">
          <div class="cfg-group"><label>OLED SDA (Wewnętrzny):</label><input type="number" id="cfg_osda"></div>
          <div class="cfg-group"><label>OLED SCL (Wewnętrzny):</label><input type="number" id="cfg_oscl"></div>
          <div class="cfg-group"><label>BARO SDA (Zewnętrzny):</label><input type="number" id="cfg_bsda"></div>
          <div class="cfg-group"><label>BARO SCL (Zewnętrzny):</label><input type="number" id="cfg_bscl"></div>
          <div class="cfg-group"><label>OLED Reset (RST):</label><input type="number" id="cfg_rst"></div>
          <div class="cfg-group"><label>OLED Zasilanie (Vext):</label><input type="number" id="cfg_pwr"></div>
          <div class="cfg-group"><label>Czas ekranu OLED (sekundy):</label><input type="number" id="cfg_oled_int" min="1"></div>
      </div>
      <button onclick="saveFullConfig()" style="margin-top: 15px;">💾 ZAPISZ KONFIGURACJĘ I ZRESTARTUJ</button>
    </div>

  </div>
  
  <div class="status">
    <strong>Status Systemu:</strong><br><br>
    Sygnał radiowy (RSSI): <span id="rssi" style="color:#4CAF50">--</span> dBm<br>
    Bateria czujnika: <span id="bat">--</span><br>
    Czas od ostatniej ramki: <span id="last_seen">--</span> sek.<br>
    Następny raport APRS za: <span id="aprs_time">--</span> sek.<br>
    Tryb WiFi: <span id="wifi_mode">--</span>
  </div>

<script>
let configLoaded = false;
setInterval(function() {
  fetch('/api/data').then(response => response.json()).then(data => {
    document.getElementById('temp').innerHTML = data.temp;
    document.getElementById('hum').innerHTML = data.hum;
    document.getElementById('baro').innerHTML = data.baro;
    document.getElementById('wind').innerHTML = data.wind;
    document.getElementById('dir').innerHTML = data.dir;
    document.getElementById('rain1').innerHTML = data.rain1;
    document.getElementById('rain24').innerHTML = data.rain24;
    document.getElementById('lum').innerHTML = data.lum;
    document.getElementById('uv').innerHTML = data.uv;
    
    document.getElementById('rssi').innerHTML = data.rssi;
    document.getElementById('bat').innerHTML = data.bat ? "OK 🔋" : "<span class='alert'>SŁABA 🪫</span>";
    document.getElementById('wifi_mode').innerHTML = data.is_ap ? "<span class='alert'>Access Point</span>" : "Station";
    
    let seen = Math.floor(data.last_seen / 1000);
    let seenElem = document.getElementById('last_seen');
    seenElem.innerHTML = seen;
    if(seen > 180) seenElem.className = 'alert'; else seenElem.className = '';
    
    document.getElementById('aprs_time').innerHTML = Math.floor(data.aprs_time / 1000);
    document.getElementById('current_id').innerHTML = (data.target_id === "0") ? "Dowolna" : data.target_id.toUpperCase();
    
    if(!configLoaded) {
        document.getElementById('cfg_ssid').value = data.cfg_ssid;
        document.getElementById('cfg_pass').value = data.cfg_pass;
        document.getElementById('cfg_call').value = data.cfg_call;
        document.getElementById('cfg_call_ssid').value = data.cfg_call_ssid;
        document.getElementById('cfg_passcode').value = data.cfg_passcode;
        document.getElementById('cfg_interval').value = data.cfg_interval;
        document.getElementById('cfg_lat').value = data.cfg_lat;
        document.getElementById('cfg_lon').value = data.cfg_lon;
        document.getElementById('cfg_mode').value = data.cfg_mode;
        document.getElementById('cfg_host').value = data.cfg_host;
        document.getElementById('cfg_port').value = data.cfg_port;
        
        document.getElementById('cfg_osda').value = data.cfg_osda;
        document.getElementById('cfg_oscl').value = data.cfg_oscl;
        document.getElementById('cfg_bsda').value = data.cfg_bsda;
        document.getElementById('cfg_bscl').value = data.cfg_bscl;
        document.getElementById('cfg_rst').value = data.cfg_rst;
        document.getElementById('cfg_pwr').value = data.cfg_pwr;
        document.getElementById('cfg_oled_int').value = data.cfg_oled_int;
        
        configLoaded = true;
    }
    
    let sel = document.getElementById('id_selector');
    let currentOptions = Array.from(sel.options).map(opt => opt.value);
    data.seen_ids.forEach(id => {
        let idStr = id.toUpperCase();
        if(!currentOptions.includes(idStr)) {
            let opt = document.createElement('option');
            opt.value = idStr;
            opt.innerHTML = idStr;
            sel.appendChild(opt);
        }
    });
  });
}, 2000);

function setStationId() {
    let id = document.getElementById('id_selector').value;
    fetch('/api/setid?id=' + encodeURIComponent(id)).then(r => alert('Zapisano.'));
}

function saveFullConfig() {
    let params = new URLSearchParams();
    params.append('ssid', document.getElementById('cfg_ssid').value);
    params.append('pass', document.getElementById('cfg_pass').value);
    params.append('call', document.getElementById('cfg_call').value);
    params.append('call_ssid', document.getElementById('cfg_call_ssid').value);
    params.append('passcode', document.getElementById('cfg_passcode').value);
    params.append('interval', document.getElementById('cfg_interval').value);
    params.append('lat', document.getElementById('cfg_lat').value);
    params.append('lon', document.getElementById('cfg_lon').value);
    params.append('mode', document.getElementById('cfg_mode').value);
    params.append('host', document.getElementById('cfg_host').value);
    params.append('port', document.getElementById('cfg_port').value);
    
    params.append('osda', document.getElementById('cfg_osda').value);
    params.append('oscl', document.getElementById('cfg_oscl').value);
    params.append('bsda', document.getElementById('cfg_bsda').value);
    params.append('bscl', document.getElementById('cfg_bscl').value);
    params.append('rst', document.getElementById('cfg_rst').value);
    params.append('pwr', document.getElementById('cfg_pwr').value);
    params.append('oled_int', document.getElementById('cfg_oled_int').value);
    
    fetch('/api/savecfg?' + params.toString()).then(r => {
        alert('Zapisano. Stacja się restartuje.');
        setTimeout(() => location.reload(), 5000);
    });
}
</script>
</body>
</html>)rawliteral";

String safeNum(float v) { return isnan(v) ? "\"--\"" : String(v, 1); }

String get_sensor_json() {
  String json = "{";
  json += "\"temp\":" + safeNum(current_wx.temp_c) + ",";
  json += "\"hum\":" + String(current_wx.humidity) + ",";
  json += "\"baro\":" + (baro_hpa > 0 ? String(baro_hpa / 10.0, 1) : "\"--\"") + ",";
  json += "\"wind\":" + safeNum(wind_gust_max_period) + ",";
  json += "\"dir\":" + safeNum(current_wx.wind_dir) + ",";
  json += "\"rain1\":" + String((calc_r1h * 0.254), 1) + ",";
  json += "\"rain24\":" + String((calc_p24h * 0.254), 1) + ",";
  json += "\"lum\":" + String(calc_lum) + ",";
  json += "\"uv\":" + safeNum(current_wx.uv_index) + ",";
  json += "\"rssi\":" + String(current_wx.radio_rssi) + ",";
  json += "\"bat\":" + String(current_wx.battery_ok ? "true" : "false") + ",";
  
  unsigned long now = millis();
  unsigned long interval_ms = config.report_interval_min * 60000UL;
  long time_to_aprs = interval_ms - (now - last_report_time);
  if (time_to_aprs < 0) time_to_aprs = 0;

  json += "\"last_seen\":" + String(now - last_frame_time) + ",";
  json += "\"aprs_time\":" + String(time_to_aprs) + ",";
  json += "\"is_ap\":" + String(in_ap_mode ? "true" : "false") + ",";
  json += "\"target_id\":\"" + String(config.sensor_id, HEX) + "\",";
  
  json += "\"cfg_ssid\":\"" + config.wifi_ssid + "\",";
  json += "\"cfg_pass\":\"" + config.wifi_pass + "\",";
  json += "\"cfg_call\":\"" + config.aprs_callsign + "\",";
  json += "\"cfg_call_ssid\":" + String(config.aprs_ssid) + ",";
  json += "\"cfg_passcode\":\"" + config.aprs_passcode + "\",";
  json += "\"cfg_interval\":" + String(config.report_interval_min) + ",";
  json += "\"cfg_lat\":" + String(config.lat, 4) + ",";
  json += "\"cfg_lon\":" + String(config.lon, 4) + ",";
  json += "\"cfg_mode\":" + String(config.use_kiss ? 1 : 0) + ",";
  json += "\"cfg_host\":\"" + config.server_host + "\",";
  json += "\"cfg_port\":" + String(config.server_port) + ",";
  
  json += "\"cfg_osda\":" + String(config.oled_sda) + ",";
  json += "\"cfg_oscl\":" + String(config.oled_scl) + ",";
  json += "\"cfg_bsda\":" + String(config.bme_sda) + ",";
  json += "\"cfg_bscl\":" + String(config.bme_scl) + ",";
  json += "\"cfg_rst\":" + String(config.oled_rst) + ",";
  json += "\"cfg_pwr\":" + String(config.oled_pwr) + ",";
  json += "\"cfg_oled_int\":" + String(config.oled_interval) + ",";
  
  json += "\"seen_ids\":[";
  bool first = true;
  for(int i = 0; i < MAX_SEEN_IDS; i++) {
      if(seen_ids[i] != 0) {
          if(!first) json += ",";
          json += "\"" + String(seen_ids[i], HEX) + "\"";
          first = false;
      }
  }
  json += "]";
  json += "}";
  return json;
}

int c_to_f(float c) { return (int)lround(c * 1.8 + 32); }
int ms_to_mph(float ms) { return (int)lround(ms * 2.23694); }
int mm_to_hin(float mm) { return (int)lround(mm * 3.93701); }

String format_lat(double lat) {
  char b[20]; char h = (lat >= 0) ? 'N' : 'S'; lat = fabs(lat);
  int d = (int)lat; double m = (lat - d) * 60.0;
  snprintf(b, sizeof(b), "%02d%05.2f%c", d, m, h);
  return String(b);
}
String format_lon(double lon) {
  char b[20]; char h = (lon >= 0) ? 'E' : 'W'; lon = fabs(lon);
  int d = (int)lon; double m = (lon - d) * 60.0;
  snprintf(b, sizeof(b), "%03d%05.2f%c", d, m, h);
  return String(b);
}
String p3(int v) { char b[5]; snprintf(b, sizeof(b), "%03d", (v<0?0:(v>999?999:v))); return String(b); }

String get_timestamp() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buff[10];
  snprintf(buff, sizeof(buff), "%02d%02d%02dz", t->tm_mday, t->tm_hour, t->tm_min);
  return String(buff);
}

void send_kiss_byte(WiFiClient &client, uint8_t b) {
    if (b == FEND) { client.write((uint8_t)FESC); client.write((uint8_t)TFEND);
    } else if (b == FESC) { client.write((uint8_t)FESC); client.write((uint8_t)TFESC);
    } else { client.write((uint8_t)b); }
}

void send_ax25_frame(const String &payload) {
    WiFiClient client;
    if (!client.connect(config.server_host.c_str(), config.server_port)) {
        Serial.println(F("BLAD: Brak polaczenia z serwerem KISS TCP!"));
        return;
    }
    client.write((uint8_t)FEND); 
    client.write((uint8_t)0x00); 

    char dest_padded[7]; memset(dest_padded, ' ', 6);
    for(int i=0; i<6; i++) dest_padded[i] = (i < strlen("APRS")) ? "APRS"[i] : ' ';
    for(int i=0; i<6; i++) send_kiss_byte(client, dest_padded[i] << 1);
    send_kiss_byte(client, 0x60 | ((0 & 0x0F) << 1)); 

    char src_padded[7]; memset(src_padded, ' ', 6);
    for(int i=0; i<6; i++) src_padded[i] = (i < config.aprs_callsign.length()) ? config.aprs_callsign[i] : ' ';
    for(int i=0; i<6; i++) send_kiss_byte(client, src_padded[i] << 1);
    send_kiss_byte(client, 0x60 | ((config.aprs_ssid & 0x0F) << 1)); 
    
    const char* digi = "WIDE1";
    char digi_padded[7]; memset(digi_padded, ' ', 6);
    for(int i=0; i<6; i++) digi_padded[i] = (i < strlen(digi)) ? digi[i] : ' ';
    for(int i=0; i<6; i++) send_kiss_byte(client, digi_padded[i] << 1);
    send_kiss_byte(client, 0x60 | (1 << 1) | 0x01); 

    send_kiss_byte(client, 0x03); 
    send_kiss_byte(client, 0xF0); 

    for (int i = 0; i < payload.length(); i++) {
        send_kiss_byte(client, (uint8_t)payload[i]);
    }

    client.write((uint8_t)FEND);
    client.stop();
    Serial.println(F("-> OK: Wyslano ramke KISS"));
}

void send_aprs(String custom_comment = "") {
  if (in_ap_mode) return;
  
  Serial.println(F("\n[APRS] Generowanie raportu..."));
  float avg_w = (wind_sample_count > 0) ? (wind_speed_sum / wind_sample_count) : 0.0;
  
  String ts = get_timestamp();
  String body = "@" + ts + format_lat(config.lat) + "/" + format_lon(config.lon) + "_";
  
  int wd = (int)current_wx.wind_dir;
  body += p3(wd <= 0 ? 0 : wd) + "/" + p3(ms_to_mph(avg_w));
  body += "g" + p3(ms_to_mph(wind_gust_max_period));
  body += "t" + p3(c_to_f(current_wx.temp_c));
  
  if (calc_r1h > 0) body += "r" + p3(calc_r1h);
  if (calc_p24h > 0) body += "p" + p3(calc_p24h);

  if (current_wx.humidity > 0) { char hb[5]; snprintf(hb, sizeof(hb), "h%02d", (int)current_wx.humidity); body += hb; }
  if (baro_hpa > 0) { char bb[10]; snprintf(bb, sizeof(bb), "b%05d", baro_hpa); body += bb; }
  
  if (calc_lum > 0) {
      int send_lum = (calc_lum > 999) ? 999 : calc_lum;
      char lb[6]; snprintf(lb, sizeof(lb), "L%03d", send_lum);
      body += lb;
  }

  String comment = "";
  if (custom_comment.length() > 0) {
      comment = " " + custom_comment;
  } else {
      comment = " RodosWX_3";
      comment += " Sig:" + String(current_wx.radio_rssi) + "dBm";
      if (!isnan(current_wx.uv_index)) comment += " UV:" + String(current_wx.uv_index, 1);
      comment += current_wx.battery_ok ? " Bat:OK" : " Bat:LOW";
  }

  if (config.use_kiss) {
      send_ax25_frame(body + comment);
      if (custom_comment.length() == 0) {
          wind_speed_sum = 0; wind_sample_count = 0; wind_gust_max_period = 0;
      }
  } else {
      String call_full = config.aprs_callsign;
      if (config.aprs_ssid > 0) call_full += "-" + String(config.aprs_ssid);
      
      String packet = call_full + ">APRS,TCPIP*:" + body + comment;

      WiFiClient cl;
      if (cl.connect(config.server_host.c_str(), config.server_port)) {
        cl.printf("user %s pass %s vers RodosBME 1.9\n", call_full.c_str(), config.aprs_passcode.c_str());
        delay(200);
        cl.println(packet);
        delay(500);
        cl.stop();
        Serial.println("-> " + packet);
        
        if (custom_comment.length() == 0) {
            wind_speed_sum = 0; wind_sample_count = 0; wind_gust_max_period = 0;
        }
      } else {
        Serial.println(F("BLAD: Brak polaczenia z serwerem APRS-IS!"));
      }
  }
}

void setup() {
  delay(1000); 
  Serial.begin(115200);
  Serial.println(F("\n\n--- START RodosWX_3 ---"));

  #if defined(ESP32)
      if(!LittleFS.begin(true)) Serial.println(F("LittleFS Mount Failed"));
  #else
      LittleFS.begin();
  #endif
  
  loadConfig();

  int o_sda = config.oled_sda;
  int o_scl = config.oled_scl;
  int o_rst = config.oled_rst;
  int o_pwr = config.oled_pwr;
  int b_sda = config.bme_sda;
  int b_scl = config.bme_scl;

  if (o_pwr == -1) {
      #if defined(ARDUINO_HELTEC_WIFI_LORA_32_V3) || defined(ARDUINO_HELTEC_VISION_MASTER_T190) || defined(ARDUINO_HELTEC_WIRELESS_STICK_V3)
          o_pwr = 36;
      #endif
  }
  
  if (o_rst == -1) {
      #if defined(ARDUINO_HELTEC_WIFI_LORA_32_V2) || defined(ARDUINO_TTGO_LoRa32_V1) || defined(ARDUINO_TTGO_LoRa32_V2)
          o_rst = 16;
      #elif defined(ARDUINO_HELTEC_WIFI_LORA_32_V3) || defined(ARDUINO_HELTEC_WIRELESS_STICK_V3)
          o_rst = 21;
      #endif
  }
  
  if (o_sda == -1 || o_scl == -1) {
      #if defined(ARDUINO_HELTEC_WIFI_LORA_32_V3) || defined(ARDUINO_HELTEC_VISION_MASTER_T190) || defined(ARDUINO_HELTEC_WIRELESS_STICK_V3)
          o_sda = 17; o_scl = 18;
      #elif defined(ESP8266)
          o_sda = 2; o_scl = 5;
      #endif
  }

  if (b_sda == -1 || b_scl == -1) {
      b_sda = o_sda;
      b_scl = o_scl;
  }

  if (o_pwr >= 0) {
      pinMode(o_pwr, OUTPUT);
      digitalWrite(o_pwr, LOW);
      delay(50);
  }

  if (o_sda >= 0 && o_scl >= 0) {
      Wire.begin(o_sda, o_scl);
  } else {
      #if defined(ESP8266)
          Wire.begin(2, 5); 
      #else
          Wire.begin(); 
      #endif
  }
  Wire.setClock(100000);

  TwoWire* baroWire = &Wire;
  bool use_wire1 = false;
  #if defined(ESP32)
  if (b_sda != -1 && b_scl != -1 && (b_sda != o_sda || b_scl != o_scl)) {
      Wire1.begin(b_sda, b_scl);
      Wire1.setClock(100000); 
      baroWire = &Wire1;
      use_wire1 = true;
  }
  #endif

  if (o_rst >= 0) {
      pinMode(o_rst, OUTPUT);
      digitalWrite(o_rst, LOW);
      delay(20);
      digitalWrite(o_rst, HIGH);
      delay(20);
  }
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("OLED: NIE WYKRYTO EKRANU SSD1306"));
  } else {
      Serial.println(F("OLED: OK"));
      display_available = true;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 10);
      display.println(F("RodosWX3 Uruchamianie..."));
      display.display();
  }
  
  Serial.println(F("\n--- Skanowanie fizycznej szyny I2C barometru ---"));
  byte detected_addr = 0;
  byte detected_chipid = 0;

  for(int addr = 0x76; addr <= 0x77; addr++) {
      baroWire->beginTransmission(addr);
      if (baroWire->endTransmission() == 0) {
          baroWire->beginTransmission(addr);
          baroWire->write(0xD0);
          baroWire->endTransmission();
          baroWire->requestFrom(addr, 1);
          if (baroWire->available()) {
              detected_chipid = baroWire->read();
              detected_addr = addr;
              Serial.printf("-> ZNALEZIONO FIZYCZNIE CZUJNIK: Adres 0x%02X, Chip ID: 0x%02X\n", detected_addr, detected_chipid);
              break;
          }
      }
  }

  if (detected_addr != 0) {
      if (detected_chipid == 0x60) {
          Serial.println(F("Uruchamianie sterownika BME280..."));
          if (bme.begin(detected_addr, baroWire)) {
              baro_sensor_type = 1;
              Serial.println(F("BME280 ZAINICJALIZOWANY POPRAWNIE"));
          } else {
              Serial.println(F("BME280: BLAD STARTU BIBLIOTEKI"));
          }
      } 
      else if (detected_chipid == 0x58 || detected_chipid == 0x56 || detected_chipid == 0x57) {
          Serial.println(F("Uruchamianie sterownika BMP280..."));
          bmp = new Adafruit_BMP280((TwoWire*)baroWire);
          if (bmp->begin(detected_addr, detected_chipid)) {
              baro_sensor_type = 2;
              Serial.println(F("BMP280 ZAINICJALIZOWANY POPRAWNIE"));
          } else {
              Serial.println(F("BMP280: BLAD STARTU BIBLIOTEKI"));
          }
      } 
      else {
          Serial.println(F("BME/BMP: Nieobslugiwany lub uszkodzony czujnik (Zle Chip ID)."));
      }
  } else {
      Serial.println(F("UWAGA: Nie wykryto zadnego czujnika I2C na tych pinach!"));
  }
  
  Serial.println(F("\nRozpoczynam inicjalizacje radia (BresserReceiver)..."));
  ws.begin();
  Serial.println(F("Radio zainicjalizowane pomyslnie."));

  Serial.println(F("Laczenie WiFi..."));
  WiFi.mode(WIFI_STA);
  if (config.wifi_ssid.length() > 0) {
      WiFi.begin(config.wifi_ssid.c_str(), config.wifi_pass.c_str());
  }
  
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout < 30) { 
      delay(500); Serial.print("."); wifi_timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
      Serial.println(F("\nWiFi OK"));
      configTime(0, 0, "pool.ntp.org");
  } else {
      Serial.println(F("\nBLAD WiFi! Uruchamiam Access Point dla konfiguracji."));
      WiFi.mode(WIFI_AP);
      WiFi.softAP("RodosWX3_Setup");
      in_ap_mode = true;
  }
  
  for(int i=0; i<RAIN_SAMPLES; i++) {
      rain_history[i].timestamp = 0;
      rain_history[i].cum_rain = 0.0;
  }
  
  last_frame_time = millis(); 
  last_display_time = millis();

  // --- SERWER WWW ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", get_sensor_json());
  });

  server.on("/api/setid", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("id")) {
        config.sensor_id = strtoul(request->getParam("id")->value().c_str(), NULL, 16);
        saveConfig();
        request->send(200, "text/plain", "OK");
    }
  });

  server.on("/api/savecfg", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("ssid")) config.wifi_ssid = request->getParam("ssid")->value();
    if(request->hasParam("pass")) config.wifi_pass = request->getParam("pass")->value();
    if(request->hasParam("call")) config.aprs_callsign = request->getParam("call")->value();
    if(request->hasParam("call_ssid")) config.aprs_ssid = request->getParam("call_ssid")->value().toInt();
    if(request->hasParam("passcode")) config.aprs_passcode = request->getParam("passcode")->value();
    if(request->hasParam("interval")) config.report_interval_min = request->getParam("interval")->value().toInt();
    if(request->hasParam("lat")) config.lat = request->getParam("lat")->value().toFloat();
    if(request->hasParam("lon")) config.lon = request->getParam("lon")->value().toFloat();
    if(request->hasParam("mode")) config.use_kiss = (request->getParam("mode")->value() == "1");
    if(request->hasParam("host")) config.server_host = request->getParam("host")->value();
    if(request->hasParam("port")) config.server_port = request->getParam("port")->value().toInt();
    
    if(request->hasParam("osda")) config.oled_sda = request->getParam("osda")->value().toInt();
    if(request->hasParam("oscl")) config.oled_scl = request->getParam("oscl")->value().toInt();
    if(request->hasParam("bsda")) config.bme_sda = request->getParam("bsda")->value().toInt();
    if(request->hasParam("bscl")) config.bme_scl = request->getParam("bscl")->value().toInt();
    if(request->hasParam("rst")) config.oled_rst = request->getParam("rst")->value().toInt();
    if(request->hasParam("pwr")) config.oled_pwr = request->getParam("pwr")->value().toInt();
    if(request->hasParam("oled_int")) config.oled_interval = request->getParam("oled_int")->value().toInt();
    
    saveConfig();
    request->send(200, "text/plain", "OK");
    shouldReboot = true;
  });

  server.begin();
  Serial.println(F("Serwer WWW dziala w tle."));
}

void loop() {
  if (shouldReboot) {
      delay(1000);
      ESP.restart();
  }

  unsigned long d_interval = (config.oled_interval > 0 ? config.oled_interval : 4) * 1000UL;
  if (millis() - last_display_time > d_interval) {
      display_page++;
      if (display_page > 6) display_page = 0;
      updateDisplay();
      last_display_time = millis();
  }

  ws.clearSlots();
  if (ws.getMessage() == DECODE_OK && ws.sensor[0].valid) {
      uint32_t inc_id = ws.sensor[0].sensor_id;
      addSeenId(inc_id); 

      if (config.sensor_id != 0 && inc_id != config.sensor_id) {
          Serial.printf("[RADIO] Odrzucono ramke obcej stacji. ID: %X\n", inc_id);
      } else {
          last_frame_time = millis(); 
          Serial.printf("[RADIO] Odebrano dane (ID: %X)\n", inc_id);
          
          if (ws.sensor[0].w.temp_ok) current_wx.temp_c = ws.sensor[0].w.temp_c;
          if (ws.sensor[0].w.humidity_ok) current_wx.humidity = ws.sensor[0].w.humidity;
          if (ws.sensor[0].w.rain_ok) current_wx.rain_total_mm = ws.sensor[0].w.rain_mm;
          
          #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
            if (ws.sensor[0].w.uv_ok) current_wx.uv_index = ws.sensor[0].w.uv;
          #endif
          #ifdef BRESSER_7_IN_1
            if (ws.sensor[0].w.light_ok) current_wx.light_klx = ws.sensor[0].w.light_klx;
          #endif

          current_wx.radio_rssi = ws.sensor[0].rssi;
          current_wx.battery_ok = ws.sensor[0].battery_ok;

          if (ws.sensor[0].w.wind_ok) {
              current_wx.wind_dir = ws.sensor[0].w.wind_direction_deg;
              float s = ws.sensor[0].w.wind_avg_meter_sec;
              float g = ws.sensor[0].w.wind_gust_meter_sec;
              wind_speed_sum += s; wind_sample_count++;
              if (g > wind_gust_max_period) wind_gust_max_period = g;
          }
          current_wx.valid_data = true;

          if (!isnan(current_wx.rain_total_mm)) {
              uint32_t now_sec = time(nullptr);
              if (now_sec > 100000) { 
                  if (last_rain_save_time == 0 || abs((long)(now_sec - last_rain_save_time)) >= 600) {
                      rain_history[rain_hist_idx].timestamp = now_sec;
                      rain_history[rain_hist_idx].cum_rain = current_wx.rain_total_mm;
                      rain_hist_idx = (rain_hist_idx + 1) % RAIN_SAMPLES;
                      last_rain_save_time = now_sec;
                  }
                  
                  float diff_1h = get_rain_delta(now_sec - 3600, current_wx.rain_total_mm);
                  calc_r1h = mm_to_hin(diff_1h);
                  
                  float diff_24h = get_rain_delta(now_sec - 86400, current_wx.rain_total_mm);
                  calc_p24h = mm_to_hin(diff_24h);
              }
          }

          // --- BEZPIECZNY ODCZYT CISNIENIA ---
          if (baro_sensor_type > 0) {
              float p = NAN;
              if (baro_sensor_type == 1) {
                  p = bme.readPressure();
              } else if (baro_sensor_type == 2 && bmp != nullptr) {
                  p = bmp->readPressure();
              }
              
              if (!isnan(p) && p > 80000.0 && p < 120000.0) {
                  baro_hpa = (int)(p / 10.0);
              }
          }
          
          if (!isnan(current_wx.light_klx)) {
              calc_lum = (int)(current_wx.light_klx * 7.9);
          }
      }
  }

  // --- WYSYŁKA APRS ---
  unsigned long interval_ms = config.report_interval_min * 60000UL;
  bool aprs_allowed = (!in_ap_mode || config.use_kiss);
  // aprs frame is send if enough time passed, and we use KISS OR we are connected to external wifi
  if (aprs_allowed && (millis() - last_report_time >= interval_ms)) {
      if (current_wx.valid_data && (WiFi.status() == WL_CONNECTED || config.use_kiss)) {
          send_aprs();
      }
      last_report_time = millis();
  }

  // --- WATCHDOG ---
  if (aprs_allowed && config.sensor_id != 0 && (millis() - last_frame_time >= NO_DATA_TIMEOUT_MS)) {
      Serial.println(F("\n[WATCHDOG] Brak ramek radiowych od 5 minut! Wysylam komunikat i wymuszam restart..."));
      if (WiFi.status() == WL_CONNECTED || config.use_kiss) {
          send_aprs("RodosWX3: Brak sygnalu ze stacji");
      }
      delay(1000); 
      ESP.restart();
  }

  delay(50);
}
