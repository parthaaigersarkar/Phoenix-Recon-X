/*
  RESQ-VISION | ESP32-S3 sensor node  ->  ESP32-CAM (WiFi)  +  LoRa
  -----------------------------------------------------------------
  Reads PIR, MQ-2 and NEO-6M GPS, then:
    - posts every reading to the ESP32-CAM over WiFi, once a second, and
      immediately when motion starts -> shown live on the camera dashboard
    - transmits a compact packet over LoRa every 5 s (1.5 s during an alarm)

  Libraries (Library Manager):
    LoRa          by Sandeep Mistry
    TinyGPSPlus   by Mikal Hart

  ORDER: flash and power the ESP32-CAM first; this board joins its network.

  ARDUINO IDE  (Tools menu)
    Board ............ ESP32S3 Dev Module
    USB CDC On Boot .. Enabled

  WIRING (unchanged from before)
    PIR    OUT  -> GPIO4
    MQ-2   AOUT -> 10k/10k divider -> GPIO5          (MANDATORY)
    NEO-6M TX   -> GPIO18        NEO-6M RX -> GPIO17
    LoRa   SCK 12, MISO 13, MOSI 11, NSS 10, RST 14, DIO0 9, VCC 3.3V ONLY
    LED         -> GPIO2 (optional)
    PIR and MQ-2 VCC -> 5V.  GPS VCC -> 3.3V.  GND common.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ---------------- network: must match the ESP32-CAM ----------------
const char *CAM_SSID = "RESQ-CAM";
const char *CAM_PASS = "rescue123";
const char *POST_URL = "http://192.168.4.1/api/sensor";
const uint8_t NODE_ID = 1;

// ---------------- pins ----------------
#define PIR_PIN     4
#define MQ2_AOUT    5
#define LED_PIN     2
#define GPS_RX_PIN  18
#define GPS_TX_PIN  17
#define LORA_SCK    12
#define LORA_MISO   13
#define LORA_MOSI   11
#define LORA_NSS    10
#define LORA_RST    14
#define LORA_DIO0    9

// ---------------- LoRa (must match the ground station) ----------------
const long LORA_FREQ = 433E6;
const int  LORA_SF   = 9;
const long LORA_BW   = 125E3;
const int  LORA_CR   = 5;

// ---------------- timing & sensors ----------------
const float    DIVIDER_RATIO    = 2.0;
const uint32_t WARMUP_MS        = 60000;
const uint16_t BASELINE_SAMPLES = 100;
const uint8_t  ADC_AVG          = 16;
const float    ALARM_RATIO      = 2.0;
const uint32_t POST_MS          = 1000;     // WiFi update to the camera
const uint32_t LORA_MS          = 5000;     // routine LoRa packet
const uint32_t LORA_ALERT_MS    = 1500;     // faster while something is happening
const uint32_t GPS_FIX_MAX_AGE  = 2000;
const uint32_t PIR_DEBOUNCE_MS  = 200;
const uint32_t PIR_CLEAR_MS     = 2500;

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

volatile bool     pirEdge = false;
volatile uint32_t pirLastEdge = 0;

bool     motionActive = false;
uint32_t motionStart = 0, pirLastHigh = 0, motionEvents = 0;
float    baselineV = 0, lastVolts = 0, lastRatio = 0;
bool     calibrated = false, gasAlarm = false, loraOK = false;
uint32_t bootMs = 0, lastPost = 0, lastLora = 0, lastWifiTry = 0;
uint32_t seq = 0, loraCount = 0, postOK = 0, postFail = 0;

void IRAM_ATTR pirISR() {
  uint32_t now = millis();
  if (now - pirLastEdge > PIR_DEBOUNCE_MS) { pirLastEdge = now; pirEdge = true; }
}

float readSensorVolts() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < ADC_AVG; i++) { acc += analogReadMilliVolts(MQ2_AOUT); delayMicroseconds(200); }
  return ((acc / (float)ADC_AVG) / 1000.0) * DIVIDER_RATIO;
}

void feedGPS() { while (GPS_Serial.available()) gps.encode(GPS_Serial.read()); }

bool freshFix() { return gps.location.isValid() && gps.location.age() < GPS_FIX_MAX_AGE; }

void calibrateBaseline() {
  Serial.println(F("Calibrating MQ-2 baseline in clean air..."));
  double acc = 0;
  for (uint16_t i = 0; i < BASELINE_SAMPLES; i++) { acc += readSensorVolts(); delay(20); feedGPS(); }
  baselineV = acc / BASELINE_SAMPLES;
  calibrated = true;
  Serial.printf("Baseline = %.3f V\n", baselineV);
}

// returns true when motion has just started
bool servicePIR(uint32_t now) {
  bool started = false;
  bool pinHigh = digitalRead(PIR_PIN) == HIGH;
  if (pinHigh) pirLastHigh = now;
  if ((pirEdge || pinHigh) && !motionActive && calibrated) {
    motionActive = true; motionStart = now; motionEvents++; started = true;
    digitalWrite(LED_PIN, HIGH);
    Serial.printf("*** MOTION  event #%lu\n", (unsigned long)motionEvents);
  }
  pirEdge = false;
  if (motionActive && !pinHigh && (now - pirLastHigh > PIR_CLEAR_MS)) {
    motionActive = false; digitalWrite(LED_PIN, LOW);
    Serial.printf("    cleared after %.1f s\n", (pirLastHigh - motionStart) / 1000.0);
  }
  return started;
}

void keepWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (now - lastWifiTry < 5000) return;
  lastWifiTry = now;
  Serial.println(F("WiFi: joining RESQ-CAM..."));
  WiFi.disconnect();
  WiFi.begin(CAM_SSID, CAM_PASS);
}

void postToCamera(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) { postFail++; return; }

  bool fix = freshFix();
  uint32_t warmLeft = calibrated ? 0 : (WARMUP_MS - (now - bootMs)) / 1000;
  char body[400];
  int n = snprintf(body, sizeof(body),
    "{\"node\":%u,\"seq\":%lu,\"uptime\":%lu,\"ready\":%s,\"warm_left\":%lu,"
    "\"motion\":%s,\"events\":%lu,\"gas_ratio\":%.2f,\"gas_volts\":%.3f,\"gas_alarm\":%s,"
    "\"fix\":%s,\"sats\":%d,\"lora\":%s,\"lora_tx\":%lu,\"rssi\":%d",
    NODE_ID, (unsigned long)seq, (unsigned long)(now / 1000),
    calibrated ? "true" : "false", (unsigned long)warmLeft,
    motionActive ? "true" : "false", (unsigned long)motionEvents,
    lastRatio, lastVolts, gasAlarm ? "true" : "false",
    fix ? "true" : "false", (int)gps.satellites.value(),
    loraOK ? "true" : "false", (unsigned long)loraCount, (int)WiFi.RSSI());
  if (fix) {
    n += snprintf(body + n, sizeof(body) - n, ",\"lat\":%.6f,\"lon\":%.6f",
                  gps.location.lat(), gps.location.lng());
  }
  snprintf(body + n, sizeof(body) - n, "}");

  HTTPClient http;
  http.setConnectTimeout(800);
  http.setTimeout(800);
  http.begin(POST_URL);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)body, strlen(body));
  http.end();

  if (code == 200) postOK++;
  else { postFail++; Serial.printf("post failed (%d)\n", code); }
}

void sendLora() {
  if (!loraOK) return;
  bool fix = freshFix();
  char pkt[128];
  int n = snprintf(pkt, sizeof(pkt), "RV,%u,%lu,%lu,%d,%lu,%.2f,%d,%d,",
                   NODE_ID, (unsigned long)seq, (unsigned long)(millis() / 1000),
                   motionActive ? 1 : 0, (unsigned long)motionEvents,
                   lastRatio, gasAlarm ? 1 : 0, fix ? 1 : 0);
  if (fix) snprintf(pkt + n, sizeof(pkt) - n, "%.6f,%.6f,%d",
                    gps.location.lat(), gps.location.lng(), gps.satellites.value());
  else     snprintf(pkt + n, sizeof(pkt) - n, ",,%d", gps.satellites.value());
  LoRa.beginPacket(); LoRa.print(pkt); LoRa.endPacket();
  loraCount++;
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  bootMs = millis();

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);
#if defined(ADC_ATTEN_DB_12)
  analogSetPinAttenuation(MQ2_AOUT, ADC_ATTEN_DB_12);
#else
  analogSetPinAttenuation(MQ2_AOUT, ADC_11db);
#endif

  GPS_Serial.setRxBufferSize(1024);           // keeps GPS data safe while WiFi posts
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirISR, RISING);

  Serial.println(F("\nRESQ-VISION | S3 sensor node -> camera + LoRa"));

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setTxPower(17);
    LoRa.enableCrc();
    loraOK = true;
    Serial.println(F("LoRa ready."));
  } else {
    Serial.println(F("LoRa init FAILED - check wiring and 3.3V. Continuing without it."));
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(CAM_SSID, CAM_PASS);
  lastWifiTry = millis();
  Serial.printf("Joining %s ... warm-up %lu s\n", CAM_SSID, (unsigned long)(WARMUP_MS / 1000));
}

void loop() {
  feedGPS();
  uint32_t now = millis();
  keepWifi(now);

  static bool announced = false;
  if (WiFi.status() == WL_CONNECTED && !announced) {
    announced = true;
    Serial.printf("WiFi connected to camera. Signal %d dBm\n", WiFi.RSSI());
  }
  if (WiFi.status() != WL_CONNECTED) announced = false;

  if (!calibrated && now - bootMs >= WARMUP_MS) {
    calibrateBaseline();
    pirEdge = false;
    motionActive = false;
  }

  bool motionStarted = servicePIR(now);

  if (calibrated && (now - lastPost >= POST_MS || motionStarted)) {
    lastVolts = readSensorVolts();
    lastRatio = (baselineV > 0.01) ? lastVolts / baselineV : 0.0;
    bool nowAlarm = lastRatio >= ALARM_RATIO;
    if (nowAlarm && !gasAlarm) Serial.printf("*** GAS %.2fx baseline\n", lastRatio);
    gasAlarm = nowAlarm;
  }

  if (now - lastPost >= POST_MS || motionStarted) {
    lastPost = now;
    seq++;
    postToCamera(now);
    feedGPS();
  }

  uint32_t loraPeriod = (motionActive || gasAlarm) ? LORA_ALERT_MS : LORA_MS;
  if (calibrated && now - lastLora >= loraPeriod) {
    lastLora = now;
    sendLora();
  }

  static uint32_t lastLog = 0;
  if (now - lastLog >= 5000) {
    lastLog = now;
    Serial.printf("%s | PIR %s | gas %.2fx | GPS %s sats %d | posts ok %lu fail %lu | LoRa %lu\n",
                  calibrated ? "running" : "warm-up",
                  motionActive ? "MOTION" : "clear", lastRatio,
                  freshFix() ? "fix" : "no fix", gps.satellites.value(),
                  (unsigned long)postOK, (unsigned long)postFail, (unsigned long)loraCount);
    Serial.printf("GPS bytes: %lu | valid sentences: %lu\n",
                  gps.charsProcessed(),
                  gps.passedChecksum());
  }
}