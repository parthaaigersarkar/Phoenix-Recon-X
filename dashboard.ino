/*
  RESQ-VISION | ESP32-CAM  ->  camera + live dashboard
  ----------------------------------------------------
  This board:
    - makes its own WiFi network  RESQ-CAM  (password rescue123)
    - streams the camera
    - RECEIVES sensor readings from the ESP32-S3 node over WiFi
    - serves one page showing the camera AND the sensor data together

  No wires between the two boards. The S3 joins this network and posts its
  readings here once a second.

  NO LIBRARIES TO INSTALL.

  ARDUINO IDE  (Tools menu)
    Board ............ AI Thinker ESP32-CAM
    Partition Scheme . Huge APP (3MB No OTA)
    Upload Speed ..... 115200

  UPLOAD on the ESP32-CAM-MB base:
    click Upload; if it hangs on "Connecting...", hold IO0, tap RST, release IO0.
    When done, tap RST.

  USE
    Phone WiFi -> RESQ-CAM  (password rescue123)
    Browser    -> http://192.168.4.1

  ENDPOINTS
    port 80   /             dashboard page
              /api/data     latest sensor reading (JSON)
              /api/sensor   the S3 posts its readings here
              /jpg          one snapshot
    port 81   /stream       live MJPEG stream (used by the dashboard)
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---------------- network ----------------
const char *AP_SSID = "RESQ-CAM";
const char *AP_PASS = "rescue123";            // at least 8 characters

// true only for bench tests on a weak USB port; fix the power supply instead
const bool DISABLE_BROWNOUT = false;

// ---------------- AI-Thinker ESP32-CAM pins ----------------
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FLASH_LED_PIN   4
#define RED_LED_PIN    33                     // active LOW

httpd_handle_t webServer = NULL;
httpd_handle_t streamServer = NULL;
const char *sensorName = "unknown";

// latest reading from the S3 node
static char     sensorJson[640] = "{}";
static uint32_t sensorAt = 0;
static uint32_t sensorCount = 0;
portMUX_TYPE    sensorMux = portMUX_INITIALIZER_UNLOCKED;

#define BOUNDARY "resqvisionframe"
static const char *STREAM_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *PART_HEAD   = "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
                                 "Content-Length: %u\r\n\r\n";

// ---------------- camera ----------------
bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;   c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 10000000;                  // 10 MHz: lower current draw
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    c.frame_size = FRAMESIZE_VGA;  c.jpeg_quality = 12;  c.fb_count = 2;
  } else {
    c.frame_size = FRAMESIZE_QVGA; c.jpeg_quality = 15;  c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x  (reseat the ribbon, check power)\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  switch (s->id.PID) {
    case OV2640_PID: sensorName = "OV2640"; break;
    case OV3660_PID: sensorName = "OV3660"; break;
    case OV5640_PID: sensorName = "OV5640"; break;
    default:         sensorName = "other";  break;
  }
  if (s->id.PID == OV3660_PID || s->id.PID == OV5640_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -1);
  }
  s->set_framesize(s, FRAMESIZE_VGA);
  Serial.printf("Camera ready: %s\n", sensorName);
  return true;
}

// ---------------- dashboard page ----------------
static const char PAGE[] = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RESQ-VISION</title><style>
:root{--navy:#132a4c;--teal:#0f6e56;--coral:#993c1d;--amber:#b07a14;--edge:#dfe3ea;--dim:#77766f}
*{box-sizing:border-box}body{margin:0;background:#f4f6f9;color:#1a1a18;
font:15px/1.45 -apple-system,Segoe UI,Roboto,Helvetica,Arial}
header{background:var(--navy);color:#fff;padding:12px 16px;display:flex;justify-content:space-between;align-items:center}
header h1{margin:0;font-size:17px}header span{font-size:12px;color:#9fb8d6}
.wrap{padding:12px;max-width:980px;margin:0 auto}
#alert{display:none;background:var(--coral);color:#fff;border-radius:10px;padding:12px 14px;
font-weight:700;margin-bottom:12px;text-align:center}
.cam{background:#000;border-radius:10px;overflow:hidden;position:relative}
.cam img{width:100%;display:block;min-height:180px}
.tag{position:absolute;left:10px;top:10px;background:rgba(0,0,0,.55);color:#fff;
font-size:12px;padding:4px 8px;border-radius:6px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;margin-top:12px}
.card{background:#fff;border:1px solid var(--edge);border-radius:10px;padding:12px}
.card h2{margin:0 0 6px;font-size:11px;letter-spacing:.8px;color:var(--dim);text-transform:uppercase}
.big{font-size:24px;font-weight:700;line-height:1.15}
.sub{font-size:12px;color:var(--dim);margin-top:3px}
.ok{color:var(--teal)}.bad{color:var(--coral)}.warn{color:var(--amber)}.idle{color:var(--dim)}
a{color:var(--navy)}
</style></head><body>
<header><h1>RESQ-VISION</h1><span id="clock">camera + sensor node</span></header>
<div class="wrap">
 <div id="alert"></div>
 <div class="cam"><img id="cam" alt="camera"><div class="tag" id="camtag">live camera</div></div>
 <div class="grid">
  <div class="card"><h2>Motion</h2><div id="m" class="big idle">--</div><div id="ms" class="sub">&nbsp;</div></div>
  <div class="card"><h2>Gas</h2><div id="g" class="big idle">--</div><div id="gs" class="sub">&nbsp;</div></div>
  <div class="card"><h2>Position</h2><div id="p" class="big idle">--</div><div id="ps" class="sub">&nbsp;</div></div>
  <div class="card"><h2>Sensor node</h2><div id="n" class="big idle">--</div><div id="ns" class="sub">&nbsp;</div></div>
 </div>
</div><script>
document.getElementById('cam').src='http://'+location.hostname+':81/stream';
function $(i){return document.getElementById(i)}
async function tick(){
 try{
  const r=await(await fetch('/api/data',{cache:'no-store'})).json();
  const d=r.node||{}, age=r.age_ms;
  const live = age>=0 && age<5000;
  $('n').textContent = live ? 'online' : (age<0?'waiting':'lost');
  $('n').className = 'big '+(live?'ok':(age<0?'idle':'bad'));
  $('ns').textContent = age<0 ? 'no data from the S3 yet'
      : ('last update '+(age/1000).toFixed(1)+' s ago · LoRa '+(d.lora?'on':'off'));
  if(!live){$('alert').style.display='none';return;}

  if(d.ready===false){
   $('g').textContent='warm-up';$('g').className='big warn';
   $('gs').textContent=(d.warm_left||0)+' s left';
  }else{
   $('g').textContent=(d.gas_ratio||0).toFixed(2)+'x';
   $('g').className='big '+(d.gas_alarm?'bad':(d.gas_ratio>1.3?'warn':'ok'));
   $('gs').textContent=(d.gas_alarm?'ELEVATED · ':'')+'vs clean-air baseline';
  }
  $('m').textContent=d.motion?'MOTION':'clear';
  $('m').className='big '+(d.motion?'bad':'ok');
  $('ms').textContent=(d.events||0)+' events';
  if(d.fix){
   $('p').innerHTML='<span style="font-size:17px">'+d.lat.toFixed(5)+', '+d.lon.toFixed(5)+'</span>';
   $('p').className='big ok';
   $('ps').innerHTML=d.sats+' sats · <a target="_blank" href="https://maps.google.com/?q='+d.lat+','+d.lon+'">maps</a>';
  }else{
   $('p').textContent='no fix';$('p').className='big idle';$('ps').textContent=(d.sats||0)+' satellites';
  }
  const msgs=[];
  if(d.motion) msgs.push('MOTION DETECTED');
  if(d.gas_alarm) msgs.push('GAS ELEVATED '+(d.gas_ratio||0).toFixed(2)+'x');
  $('alert').style.display=msgs.length?'block':'none';
  $('alert').textContent=msgs.join('  ·  ')+(msgs.length&&d.fix?'  ·  '+d.lat.toFixed(5)+', '+d.lon.toFixed(5):'');
 }catch(e){$('n').textContent='--';}
}
tick();setInterval(tick,1000);
</script></body></html>)HTML";

static esp_err_t page_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

// the S3 posts its JSON reading here
static esp_err_t sensor_post_handler(httpd_req_t *req) {
  int len = req->content_len;
  if (len <= 0 || len >= (int)sizeof(sensorJson)) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
  }
  char buf[640];
  int got = 0;
  while (got < len) {
    int r = httpd_req_recv(req, buf + got, len - got);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (r <= 0) return ESP_FAIL;
    got += r;
  }
  buf[got] = '\0';

  portENTER_CRITICAL(&sensorMux);
  memcpy(sensorJson, buf, got + 1);
  sensorAt = millis();
  sensorCount++;
  portEXIT_CRITICAL(&sensorMux);

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

// the dashboard reads the latest reading from here
static esp_err_t data_handler(httpd_req_t *req) {
  char copy[640];
  uint32_t at, count;
  portENTER_CRITICAL(&sensorMux);
  memcpy(copy, sensorJson, sizeof(copy));
  at = sensorAt;
  count = sensorCount;
  portEXIT_CRITICAL(&sensorMux);

  long age = (count == 0) ? -1 : (long)(millis() - at);
  static char out[760];
  snprintf(out, sizeof(out), "{\"age_ms\":%ld,\"count\":%lu,\"camera\":\"%s\",\"node\":%s}",
           age, (unsigned long)count, sensorName, count ? copy : "{}");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t jpg_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[80];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), PART_HEAD, fb->len);
    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

void startServers() {
  // port 80: page + data + the S3's posts
  httpd_config_t web = HTTPD_DEFAULT_CONFIG();
  web.server_port = 80;
  web.ctrl_port = 32768;
  web.max_uri_handlers = 6;
  httpd_uri_t u_page = {"/",           HTTP_GET,  page_handler,        NULL};
  httpd_uri_t u_data = {"/api/data",   HTTP_GET,  data_handler,        NULL};
  httpd_uri_t u_post = {"/api/sensor", HTTP_POST, sensor_post_handler, NULL};
  httpd_uri_t u_jpg  = {"/jpg",        HTTP_GET,  jpg_handler,         NULL};
  if (httpd_start(&webServer, &web) == ESP_OK) {
    httpd_register_uri_handler(webServer, &u_page);
    httpd_register_uri_handler(webServer, &u_data);
    httpd_register_uri_handler(webServer, &u_post);
    httpd_register_uri_handler(webServer, &u_jpg);
  }

  // port 81: the stream on its own server so it never blocks the data
  httpd_config_t st = HTTPD_DEFAULT_CONFIG();
  st.server_port = 81;
  st.ctrl_port = 32769;
  st.max_uri_handlers = 1;
  httpd_uri_t u_stream = {"/stream", HTTP_GET, stream_handler, NULL};
  if (httpd_start(&streamServer, &st) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &u_stream);
  }
  Serial.println(F("Web servers running on ports 80 and 81."));
}

void setup() {
  if (DISABLE_BROWNOUT) WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(600);
  Serial.println(F("\nRESQ-VISION | ESP32-CAM dashboard"));

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);            // flash LED off: it starves WiFi
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);

  if (!startCamera()) {
    while (true) { digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN)); delay(150); }
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  Serial.println(F("========================================"));
  Serial.printf("  WiFi     : %s  /  %s\n", AP_SSID, AP_PASS);
  Serial.print(F("  open     : http://")); Serial.println(WiFi.softAPIP());
  Serial.println(F("  S3 posts : http://192.168.4.1/api/sensor"));
  Serial.println(F("========================================"));

  startServers();
  digitalWrite(RED_LED_PIN, LOW);              // steady red = running
}

void loop() {
  static uint32_t last = 0;
  static uint32_t lastCount = 0;
  if (millis() - last > 5000) {
    last = millis();
    uint32_t c; portENTER_CRITICAL(&sensorMux); c = sensorCount; portEXIT_CRITICAL(&sensorMux);
    Serial.printf("clients %d  |  sensor posts received %lu (+%lu in 5 s)\n",
                  WiFi.softAPgetStationNum(), (unsigned long)c, (unsigned long)(c - lastCount));
    lastCount = c;
  }
  delay(100);
}