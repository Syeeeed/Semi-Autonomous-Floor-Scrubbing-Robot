/**
 * ============================================================
 *  SEMI-AUTONOMOUS FLOOR SCRUBBING ROBOT
 *  ESP32-CAM — Wi-Fi Bridge, Live Video Stream & RC Control
 * ============================================================
 *  Responsibilities:
 *    - Host Wi-Fi Access Point (AP) for RC Car Mobile App
 *    - Stream live MJPEG video from OV2640 camera
 *    - Receive RC joystick/button commands from mobile app
 *    - Forward commands to Arduino Mega via UART (Serial)
 *    - Display robot status on a web dashboard
 *
 *  Communication:
 *    ESP32 Serial (TX=GPIO1, RX=GPIO3) ↔ Mega Serial1
 *    Wi-Fi AP    SSID: "RobotControl"
 *               Pass: "robot1234"
 *    Web Server  Port: 80   (dashboard + video stream)
 *    Video Stream Port: 81 (MJPEG /stream endpoint)
 *
 *  Hardware:
 *    Board     : Thinker ESP32-CAM
 *    Camera    : OV2640
 *    Flash LED : GPIO4 (built-in)
 *
 *  Target  : ESP32-CAM (Thinker module)
 * ============================================================
 */

#include "Arduino.h"
#include "WiFi.h"
#include "WebServer.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"    // Disable brownout detector

// ============================================================
//  CAMERA PIN DEFINITIONS  (AI-Thinker ESP32-CAM)
// ============================================================
#define PWDN_GPIO_NUM      32
#define RESET_GPIO_NUM     -1
#define XCLK_GPIO_NUM       0
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27
#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        21
#define Y4_GPIO_NUM        19
#define Y3_GPIO_NUM        18
#define Y2_GPIO_NUM         5
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22
#define FLASH_LED_PIN       4

// ============================================================
//  Wi-Fi CONFIGURATION  (Access Point Mode)
// ============================================================
const char* AP_SSID     = "RobotControl";
const char* AP_PASSWORD = "robot1234";
IPAddress   AP_IP(192, 168, 4, 1);
IPAddress   AP_GATEWAY(192, 168, 4, 1);
IPAddress   AP_SUBNET(255, 255, 255, 0);

// ============================================================
//  WEB SERVERS
// ============================================================
WebServer server(80);       // Dashboard + control endpoint
WebServer streamServer(81); // MJPEG video stream

// ============================================================
//  SERIAL TO ARDUINO MEGA
//  ESP32-CAM GPIO1=TX, GPIO3=RX — shared with USB prog.
//  Use Serial (Serial0) to communicate with Mega.
//  During upload use GPIO1/GPIO3 jumpers; during runtime use for UART.
// ============================================================
#define MEGA_SERIAL_BAUD     115200

// ============================================================
//  ROBOT STATUS  (updated from Mega status strings)
// ============================================================
struct RobotStatus {
    String state      = "IDLE";
    bool   autoMode   = true;
    bool   vacuum     = false;
    bool   mop        = false;
    bool   spray      = false;
    bool   obstacle   = false;
    uint32_t lastUpdate = 0;
};

RobotStatus robotStatus;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
bool     initCamera();
void     initWiFiAP();
void     initWebServer();
void     initStreamServer();

// Web handlers
void     handleRoot();
void     handleControl();
void     handleStatus();
void     handleFlashToggle();
void     handleNotFound();
void     handleStream();

// Mega communication
void     sendCommandToMega(const String& cmd);
void     processMegaData();
void     parseStatusJSON(const String& json);

// Utilities
String   buildDashboardHTML();
String   buildStatusJSON();
void     setFlashLED(bool on);

// ============================================================
//  SETUP
// ============================================================
void setup() {
    // Disable brownout detector (ESP32-CAM can brownout on motor noise)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(MEGA_SERIAL_BAUD);   // UART to Arduino Mega

    delay(500);
    Serial.println("[ESP32] Booting...");

    // Flash LED init
    pinMode(FLASH_LED_PIN, OUTPUT);
    setFlashLED(false);

    // Initialise camera
    if (!initCamera()) {
        Serial.println("[ESP32] ERROR: Camera init failed!");
        // Continue without camera — control still works
    }

    // Start Wi-Fi Access Point
    initWiFiAP();

    // Start web servers
    initWebServer();
    initStreamServer();

    Serial.println("[ESP32] Ready. Connect to Wi-Fi: " + String(AP_SSID));
    Serial.println("[ESP32] Dashboard: http://192.168.4.1");
    Serial.println("[ESP32] Video:     http://192.168.4.1:81/stream");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    // Handle HTTP clients
    server.handleClient();
    streamServer.handleClient();

    // Read status updates from Arduino Mega
    processMegaData();

    delay(5);
}

// ============================================================
//  CAMERA INITIALISATION
// ============================================================
bool initCamera() {
    camera_config_t config;
    config.ledc_channel    = LEDC_CHANNEL_0;
    config.ledc_timer      = LEDC_TIMER_0;
    config.pin_d0          = Y2_GPIO_NUM;
    config.pin_d1          = Y3_GPIO_NUM;
    config.pin_d2          = Y4_GPIO_NUM;
    config.pin_d3          = Y5_GPIO_NUM;
    config.pin_d4          = Y6_GPIO_NUM;
    config.pin_d5          = Y7_GPIO_NUM;
    config.pin_d6          = Y8_GPIO_NUM;
    config.pin_d7          = Y9_GPIO_NUM;
    config.pin_xclk        = XCLK_GPIO_NUM;
    config.pin_pclk        = PCLK_GPIO_NUM;
    config.pin_vsync       = VSYNC_GPIO_NUM;
    config.pin_href        = HREF_GPIO_NUM;
    config.pin_sscb_sda    = SIOD_GPIO_NUM;
    config.pin_sscb_scl    = SIOC_GPIO_NUM;
    config.pin_pwdn        = PWDN_GPIO_NUM;
    config.pin_reset       = RESET_GPIO_NUM;
    config.xclk_freq_hz    = 20000000;
    config.pixel_format    = PIXFORMAT_JPEG;
    config.frame_size      = FRAMESIZE_VGA;   // 640×480
    config.jpeg_quality    = 12;              // 0–63 (lower = better)
    config.fb_count        = 2;

    // Lower resolution if no PSRAM
    if (!psramFound()) {
        config.frame_size   = FRAMESIZE_QVGA;  // 320×240
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[Camera] Init error 0x%x\n", err);
        return false;
    }

    // Adjust sensor settings
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 0);        // -2 to 2
    s->set_contrast(s, 0);          // -2 to 2
    s->set_saturation(s, 0);        // -2 to 2
    s->set_special_effect(s, 0);    // 0=No Effect
    s->set_whitebal(s, 1);          // AWB on
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);           // 0=Auto
    s->set_exposure_ctrl(s, 1);     // AEC on
    s->set_gain_ctrl(s, 1);         // AGC on
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);

    Serial.println("[Camera] Initialised OK");
    return true;
}

// ============================================================
//  Wi-Fi ACCESS POINT
// ============================================================
void initWiFiAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());
}

// ============================================================
//  DASHBOARD WEB SERVER  (port 80)
// ============================================================
void initWebServer() {
    server.on("/",          HTTP_GET,  handleRoot);
    server.on("/control",   HTTP_POST, handleControl);
    server.on("/status",    HTTP_GET,  handleStatus);
    server.on("/flash",     HTTP_POST, handleFlashToggle);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("[WebServer] Started on port 80");
}

// ============================================================
//  VIDEO STREAM SERVER  (port 81)
// ============================================================
void initStreamServer() {
    streamServer.on("/stream", HTTP_GET, handleStream);
    streamServer.begin();
    Serial.println("[StreamServer] Started on port 81");
}

// ============================================================
//  WEB HANDLERS
// ============================================================

/** Serve the RC dashboard HTML page */
void handleRoot() {
    server.send(200, "text/html", buildDashboardHTML());
}

/**
 * Handle RC commands from the mobile app
 * POST /control
 * Body: cmd=FWD  (or BWD, LFT, RGT, STP, AUTO, MAN, VAC, MOP, SPR)
 */
void handleControl() {
    if (server.hasArg("cmd")) {
        String cmd = server.arg("cmd");
        cmd.toUpperCase();
        cmd.trim();

        Serial.println("[Web] Control cmd: " + cmd);
        sendCommandToMega(cmd);

        server.send(200, "application/json",
                    "{\"ok\":true,\"cmd\":\"" + cmd + "\"}");
    } else {
        server.send(400, "application/json", "{\"error\":\"Missing cmd\"}");
    }
}

/** Return robot status as JSON */
void handleStatus() {
    server.send(200, "application/json", buildStatusJSON());
}

/** Toggle flash LED */
void handleFlashToggle() {
    static bool flashOn = false;
    flashOn = !flashOn;
    setFlashLED(flashOn);
    server.send(200, "application/json",
                "{\"flash\":" + String(flashOn ? "true" : "false") + "}");
}

void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

/**
 * MJPEG video stream handler.
 * Continuously sends camera frames as multipart JPEG stream.
 * Compatible with <img src="http://192.168.4.1:81/stream"> in browsers.
 */
void handleStream() {
    WiFiClient client = streamServer.client();

    // Send multipart MIME header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Cache-Control: no-cache");
    client.println();

    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[Stream] Camera frame capture failed");
            delay(100);
            continue;
        }

        // Frame boundary header
        client.println("--frame");
        client.println("Content-Type: image/jpeg");
        client.print("Content-Length: ");
        client.println(fb->len);
        client.println();

        // Send JPEG data
        client.write(fb->buf, fb->len);
        client.println();

        esp_camera_fb_return(fb);   // Return frame buffer

        delay(33);   // ~30 FPS target
    }
}

// ============================================================
//  MEGA COMMUNICATION
// ============================================================

/** Send a command string to Arduino Mega via UART */
void sendCommandToMega(const String& cmd) {
    Serial.println(cmd);   // Sends via UART TX (GPIO1) to Mega
    Serial.flush();
}

/** Read and parse status JSON from Arduino Mega */
void processMegaData() {
    static String buffer = "";

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            buffer.trim();
            if (buffer.startsWith("{")) {
                parseStatusJSON(buffer);
            }
            buffer = "";
        } else {
            buffer += c;
            if (buffer.length() > 256) buffer = "";   // Overflow guard
        }
    }
}

/**
 * Minimal JSON parser — extracts values from the Mega's status string.
 * Format: {"state":"SCANNING","auto":true,"vacuum":false,...}
 */
void parseStatusJSON(const String& json) {
    auto extractBool = [&](const String& key) -> bool {
        int idx = json.indexOf("\"" + key + "\":");
        if (idx == -1) return false;
        int valStart = idx + key.length() + 3;
        return json.substring(valStart, valStart + 4) == "true";
    };

    auto extractString = [&](const String& key) -> String {
        String search = "\"" + key + "\":\"";
        int idx = json.indexOf(search);
        if (idx == -1) return "";
        int start = idx + search.length();
        int end   = json.indexOf("\"", start);
        return json.substring(start, end);
    };

    robotStatus.state    = extractString("state");
    robotStatus.autoMode = extractBool("auto");
    robotStatus.vacuum   = extractBool("vacuum");
    robotStatus.mop      = extractBool("mop");
    robotStatus.spray    = extractBool("spray");
    robotStatus.obstacle = extractBool("obstacle");
    robotStatus.lastUpdate = millis();
}

// ============================================================
//  UTILITIES
// ============================================================
void setFlashLED(bool on) {
    digitalWrite(FLASH_LED_PIN, on ? HIGH : LOW);
}

String buildStatusJSON() {
    String json = "{";
    json += "\"state\":\"" + robotStatus.state + "\",";
    json += "\"auto\":"    + String(robotStatus.autoMode ? "true" : "false") + ",";
    json += "\"vacuum\":"  + String(robotStatus.vacuum   ? "true" : "false") + ",";
    json += "\"mop\":"     + String(robotStatus.mop      ? "true" : "false") + ",";
    json += "\"spray\":"   + String(robotStatus.spray    ? "true" : "false") + ",";
    json += "\"obstacle\":"+ String(robotStatus.obstacle ? "true" : "false") + ",";
    json += "\"ip\":\"192.168.4.1\"";
    json += "}";
    return json;
}

// ============================================================
//  DASHBOARD HTML  (RC Car Mobile App Interface)
// ============================================================
String buildDashboardHTML() {
    return R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Floor Robot Control</title>
<style>
  :root {
    --bg: #0f0f1a;
    --card: #1a1a2e;
    --accent: #00d4ff;
    --green: #00ff88;
    --red: #ff4466;
    --yellow: #ffd700;
    --text: #e0e0ff;
    --muted: #6060a0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Segoe UI', Arial, sans-serif;
    min-height: 100vh;
    padding: 12px;
    display: flex;
    flex-direction: column;
    gap: 12px;
  }
  h1 {
    text-align: center;
    font-size: 1.1rem;
    color: var(--accent);
    letter-spacing: 2px;
    text-transform: uppercase;
    padding: 8px 0;
  }
  /* Live Video */
  .video-container {
    background: #000;
    border-radius: 12px;
    overflow: hidden;
    border: 1px solid var(--accent);
    aspect-ratio: 4/3;
    display: flex;
    align-items: center;
    justify-content: center;
  }
  .video-container img {
    width: 100%; height: 100%; object-fit: cover;
  }
  /* Status Row */
  .status-bar {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 6px;
  }
  .status-chip {
    background: var(--card);
    border-radius: 8px;
    padding: 6px 4px;
    text-align: center;
    font-size: 0.65rem;
    font-weight: 600;
    text-transform: uppercase;
    color: var(--muted);
    border: 1px solid #333;
    transition: all 0.3s;
  }
  .status-chip.active { color: var(--green); border-color: var(--green); }
  .status-chip.warning { color: var(--yellow); border-color: var(--yellow); }
  .status-chip.danger  { color: var(--red);    border-color: var(--red);   }
  /* Mode Buttons */
  .mode-row {
    display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
  }
  .btn-mode {
    padding: 10px; border-radius: 10px; border: none; cursor: pointer;
    font-size: 0.85rem; font-weight: 700; letter-spacing: 1px;
    transition: all 0.2s;
  }
  .btn-auto {
    background: linear-gradient(135deg, #00d4ff22, #00d4ff44);
    border: 1px solid var(--accent); color: var(--accent);
  }
  .btn-manual {
    background: linear-gradient(135deg, #ffd70022, #ffd70044);
    border: 1px solid var(--yellow); color: var(--yellow);
  }
  .btn-mode:active { transform: scale(0.95); opacity: 0.8; }
  /* D-Pad */
  .dpad {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr 1fr;
    gap: 6px;
    max-width: 200px;
    margin: 0 auto;
  }
  .dpad-btn {
    background: var(--card);
    border: 1px solid #333;
    border-radius: 10px;
    color: var(--text);
    font-size: 1.4rem;
    padding: 14px;
    cursor: pointer;
    text-align: center;
    transition: all 0.15s;
    user-select: none;
    -webkit-user-select: none;
  }
  .dpad-btn:active, .dpad-btn.pressed {
    background: var(--accent);
    color: #000;
    border-color: var(--accent);
    transform: scale(0.92);
  }
  .dpad-stop {
    background: #2a0a0a;
    border-color: var(--red);
    color: var(--red);
    font-size: 0.7rem;
    font-weight: 700;
  }
  /* Tool buttons */
  .tools-row {
    display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px;
  }
  .btn-tool {
    padding: 12px 6px; border-radius: 10px;
    border: 1px solid #444; background: var(--card);
    color: var(--muted); font-size: 0.75rem;
    font-weight: 600; cursor: pointer; text-align: center;
    transition: all 0.2s;
  }
  .btn-tool span { display: block; font-size: 1.4rem; margin-bottom: 4px; }
  .btn-tool.on {
    border-color: var(--green);
    color: var(--green);
    background: rgba(0,255,136,0.08);
  }
  .btn-tool:active { transform: scale(0.94); }
  /* Flash */
  .btn-flash {
    width: 100%; padding: 10px;
    background: #1a1000; border: 1px solid var(--yellow);
    color: var(--yellow); border-radius: 10px;
    font-size: 0.85rem; font-weight: 700; cursor: pointer;
  }
  /* Labels */
  .section-label {
    font-size: 0.65rem;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 1px;
    margin-bottom: -4px;
  }
  .state-display {
    text-align: center; font-size: 0.85rem;
    color: var(--accent); letter-spacing: 1px;
    padding: 4px;
  }
</style>
</head>
<body>

<h1>&#129302; Floor Robot Control</h1>

<!-- Live Video Stream -->
<div class="video-container">
  <img id="liveStream" src="http://192.168.4.1:81/stream" 
       alt="Live Camera Feed"
       onerror="this.style.display='none'; document.getElementById('noStream').style.display='flex'">
  <div id="noStream" style="display:none;color:#666;flex-direction:column;align-items:center;gap:8px;">
    <span style="font-size:2rem;">📷</span>
    <span style="font-size:0.8rem;">No Video Signal</span>
  </div>
</div>

<!-- Status Chips -->
<div class="status-bar">
  <div class="status-chip" id="chipState">STATE</div>
  <div class="status-chip" id="chipVacuum">VAC</div>
  <div class="status-chip" id="chipMop">MOP</div>
  <div class="status-chip" id="chipSpray">SPRAY</div>
  <div class="status-chip" id="chipObstacle">CLEAR</div>
</div>

<div class="state-display" id="stateDisplay">Connecting...</div>

<!-- Mode Selection -->
<div class="section-label">Control Mode</div>
<div class="mode-row">
  <button class="btn-mode btn-auto"   onclick="sendCmd('AUTO')">🤖 Auto Mode</button>
  <button class="btn-mode btn-manual" onclick="sendCmd('MAN')">🕹️ Manual</button>
</div>

<!-- D-Pad -->
<div class="section-label">Navigation</div>
<div class="dpad">
  <div></div>
  <div class="dpad-btn" id="btnFwd" ontouchstart="hold('FWD')" ontouchend="release()"
       onmousedown="hold('FWD')" onmouseup="release()">▲</div>
  <div></div>

  <div class="dpad-btn" id="btnLft" ontouchstart="hold('LFT')" ontouchend="release()"
       onmousedown="hold('LFT')" onmouseup="release()">◀</div>
  <div class="dpad-btn dpad-stop" onclick="sendCmd('STP')">STOP</div>
  <div class="dpad-btn" id="btnRgt" ontouchstart="hold('RGT')" ontouchend="release()"
       onmousedown="hold('RGT')" onmouseup="release()">▶</div>

  <div></div>
  <div class="dpad-btn" id="btnBwd" ontouchstart="hold('BWD')" ontouchend="release()"
       onmousedown="hold('BWD')" onmouseup="release()">▼</div>
  <div></div>
</div>

<!-- Cleaning Tool Toggles -->
<div class="section-label">Cleaning Tools</div>
<div class="tools-row">
  <div class="btn-tool" id="btnVacuum" onclick="toggleTool('VAC','btnVacuum')">
    <span>🌀</span>Vacuum
  </div>
  <div class="btn-tool" id="btnMop" onclick="toggleTool('MOP','btnMop')">
    <span>🧹</span>Mop
  </div>
  <div class="btn-tool" id="btnSpray" onclick="toggleTool('SPR','btnSpray')">
    <span>💧</span>Spray
  </div>
</div>

<!-- Flash LED -->
<button class="btn-flash" onclick="toggleFlash()">⚡ Flash LED</button>

<script>
  let holdInterval = null;

  function sendCmd(cmd) {
    fetch('/control', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'cmd=' + cmd
    }).catch(e => console.warn('Send failed:', e));
  }

  function hold(cmd) {
    sendCmd(cmd);
    holdInterval = setInterval(() => sendCmd(cmd), 150);
  }

  function release() {
    clearInterval(holdInterval);
    sendCmd('STP');
  }

  function toggleTool(cmd, btnId) {
    sendCmd(cmd);
    // Visual toggle handled by status update
  }

  function toggleFlash() {
    fetch('/flash', { method: 'POST' }).catch(e => {});
  }

  // Polling status every 800ms
  function updateStatus() {
    fetch('/status')
      .then(r => r.json())
      .then(d => {
        document.getElementById('stateDisplay').textContent = d.state || '---';

        const chipState = document.getElementById('chipState');
        chipState.textContent = d.auto ? 'AUTO' : 'MANUAL';
        chipState.className = 'status-chip ' + (d.auto ? 'active' : 'warning');

        setChip('chipVacuum', 'VAC',   d.vacuum);
        setChip('chipMop',    'MOP',   d.mop);
        setChip('chipSpray',  'SPRAY', d.spray);

        const obs = document.getElementById('chipObstacle');
        obs.textContent = d.obstacle ? 'BLOCK' : 'CLEAR';
        obs.className = 'status-chip ' + (d.obstacle ? 'danger' : '');

        // Sync tool buttons
        syncBtn('btnVacuum', d.vacuum);
        syncBtn('btnMop',    d.mop);
        syncBtn('btnSpray',  d.spray);
      })
      .catch(() => {
        document.getElementById('stateDisplay').textContent = 'Disconnected';
      });
  }

  function setChip(id, label, active) {
    const el = document.getElementById(id);
    el.textContent = label;
    el.className = 'status-chip ' + (active ? 'active' : '');
  }

  function syncBtn(id, isOn) {
    const el = document.getElementById(id);
    if (isOn) el.classList.add('on'); else el.classList.remove('on');
  }

  setInterval(updateStatus, 800);
  updateStatus();

  // Prevent page scroll on touch for D-pad
  document.querySelector('.dpad').addEventListener('touchmove', e => {
    e.preventDefault();
  }, { passive: false });
</script>
</body>
</html>
)rawhtml";
}
