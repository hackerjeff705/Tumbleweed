#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>
#include "FS.h"
#include "SD.h"
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>

// --- HARDWARE & SENSOR CONFIGURATION ---
#define BMP280_ADDRESS 0x76
#define SWITCH_PIN 4
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// --- WEB SERVER CONFIGURATION ---
const char* ssid = "Tumbleweed-FC-AP";
const char* password = "12345678";

// --- GLOBAL OBJECTS & VARIABLES ---
Adafruit_BMP280 bmp;
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
WebServer server(80);
using namespace BLA;

// --- FLIGHT & LOGGING STATE VARIABLES ---
bool isLogging = false;
// Master logging flag
bool webLogging = false;     // Flag for web UI record command
String currentFilename = "flight_log.csv";
float seaLevelPressurehPa = 1013.25; // Default value, updatable from UI

/* Accelerometer variables */
float ax, ay, az;
float AngleRoll, AnglePitch;
/* Gyroscope variables */
float RateRoll, RatePitch, RateYaw;
float RateCalRoll, RateCalPitch, RateCalYaw;
int RateCalNum;
/* 1D Kalman Filter (Roll/Pitch) variables */
float kfAngleRoll = 0, kfUncertaintyAngleRoll = 2 * 2;
float kfAnglePitch = 0, kfUncertaintyAnglePitch = 2 * 2;
float kf1DOutput[] = {0, 0};
/* 2D Kalman Filter (Altitude) variables */
float AltBaro;
float AltKF, VelVerticalKF;
float AccZInertial;
// GPS variables
double gpsLat = 0.0, gpsLon = 0.0;
double gpsAltitude = 0.0, gpsSpeed = 0.0;
uint32_t gpsSatellites = 0;
uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;

// Kalman Filter Matrices
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P;
BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
/* Timer variables */
unsigned long LoopTimer;

/* WiFi variables */
IPAddress IP;

// --- FUNCTION PROTOTYPES ---
void handleRoot();
void handleUpdate();
void handleRecord();
void handleData();

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // Configure physical switch

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  // --- HARDWARE INITIALIZATION ---
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Serial.println(F("Initializing OLED..."));
  oled_setup();
  Serial.println(F("Calibrating MPU-6050...")); calibrate_gryo();
  Serial.println(F("Initializing BMP280...")); baro_setup();
  Serial.println(F("Initializing GPS...")); gps_setup();
  Serial.println(F("Initializing SD card...")); sd_card_setup();
  // --- WEB SERVER INITIALIZATION ---
  Serial.print("Setting up Access Point...");
  WiFi.softAP(ssid, password);
  IP = WiFi.softAPIP();
  Serial.print(" Done! AP IP address: ");
  Serial.println(IP);

  // Display IP on OLED
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("Tumbleweed FC Ready");
  delay(1000);
  
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/record", HTTP_POST, handleRecord);
  server.begin();
  Serial.println("HTTP server started");

  kf_2d_setup();

  Serial.println("\n------------------------------------");
  Serial.println("System ready. Control logging from Web UI or switch.");
  Serial.println("------------------------------------");

  LoopTimer = micros();
}

void loop() {
  server.handleClient();
  if (micros() - LoopTimer >= 4000) {
    LoopTimer = micros();
    // --- DETERMINE LOGGING STATE (SWITCH OR WEB) ---
    bool switchLogging = (digitalRead(SWITCH_PIN) == LOW);
    isLogging = switchLogging || webLogging;

    // --- TELEMETRY GATHERING ---
    updateGPS();
    get_imu_data();
    
    RateRoll -= RateCalRoll;
    RatePitch -= RateCalPitch;
    RateYaw -= RateCalYaw;

    AngleRoll = atan(ay / sqrt(ax * ax + az * az)) * 180.0 / PI;
    AnglePitch = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / PI;

    kf_1d(kfAngleRoll, kfUncertaintyAngleRoll, RateRoll, AngleRoll);
    kfAngleRoll = kf1DOutput[0];
    kfUncertaintyAngleRoll = kf1DOutput[1];

    kf_1d(kfAnglePitch, kfUncertaintyAnglePitch, RatePitch, AnglePitch);
    kfAnglePitch = kf1DOutput[0];
    kfUncertaintyAnglePitch = kf1DOutput[1];
    AccZInertial = -sin(AnglePitch*(3.142/180))*ax + cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))*ay + cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*az;   
    AccZInertial = (AccZInertial-1)*9.81;

    AltBaro = bmp.readAltitude(seaLevelPressurehPa);
    kf_2d();
    // --- DATA LOGGING ---
    if (isLogging) {
      String dataString = String(micros()) + "," + 
                          String(kfAngleRoll) + "," + 
                          String(kfAnglePitch) + "," + 
                          String(AltKF) + "," +
                          String(gpsLat, 6) + "," +
                          String(gpsLon, 6) + "," +
                          String(gpsSatellites) + 
                          "\n";
      appendFile(SD, ("/" + currentFilename).c_str(), dataString.c_str());
      Serial.print("LOGGING: "); // Also log to serial monitor
      Serial.print(dataString);
    }
    
    // -- UPDATE OLED DISPLAY ---
    updateOLED();
  }
}

// --- WEB SERVER HANDLER FUNCTIONS ---

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<link rel=\"icon\" href=\"data:,\">";
  html += "<style>";
  html += "html { font-family: Helvetica, Arial, sans-serif; display: inline-block; margin: 0px auto; text-align: center;}";
  html += "body { margin-top: 20px; }";
  html += "h1 { color: #0F3376; }";
  html += "table { border-collapse: collapse; width: 80%; max-width: 600px; margin-left:auto; margin-right:auto; margin-bottom: 20px;}";
  html += "th { padding: 12px; background-color: #0043af; color: white; }";
  html += "tr { border: 1px solid #ddd; padding: 8px; }";
  html += "td { border: none; padding: 12px; text-align: left; font-size: 18px;}";
  html += ".sensor { color:white; font-weight: bold; background-color: #555555; padding: 4px 8px; border-radius: 5px; float: right;}";
  html += ".form-container { display: flex; justify-content: center; align-items: center; margin-top: 15px; gap: 10px; }";
  html += ".form-label { width: 160px; text-align: center; font-size: 18px; }";
  html += "input[type=number], input[type=text] { width: 150px; padding: 10px; font-size: 16px; border-radius: 8px; border: 1px solid #ccc; }";
  html += ".button { width: 120px; border: none; color: white; padding: 12px 0; text-align: center; text-decoration: none; font-size: 16px; cursor: pointer; border-radius: 8px;}";
  html += ".update-btn { background-color: #4CAF50; }";
  html += ".record-btn-off { background-color: #808080; }";
  html += ".record-btn-on { background-color: #f44336; }";
  html += "</style></head>";
  html += "<body><h1>Tumbleweed Flight Telemetry</h1>";
  html += "<table>";
  html += "<tr><th>MEASUREMENT</th><th>VALUE</th></tr>";
  html += "<tr><td>Roll / Pitch / Yaw</td><td><span class=\"sensor\" id=\"atti\">" + String(kfAngleRoll, 1) + "&deg; / " + String(kfAnglePitch, 1) + "&deg; / " + String(RateYaw, 1) + "&deg;/s</span></td></tr>";
  html += "<tr><td>Altitude (KF)</td><td><span class=\"sensor\" id=\"alt\">" + String(AltKF, 2) + " m</span></td></tr>";
  html += "<tr><td>GPS Sats</td><td><span class=\"sensor\" id=\"sats\">" + String(gpsSatellites) + "</span></td></tr>";
  html += "<tr><td>GPS Coords</td><td><span class=\"sensor\" id=\"coords\">" + String(gpsLat, 4) + " / " + String(gpsLon, 4) + "</span></td></tr>";
  html += "<tr><td>GPS Alt / Speed</td><td><span class=\"sensor\" id=\"gps_as\">" + String(gpsAltitude, 1) + "m / " + String(gpsSpeed, 1) + "km/h</span></td></tr>";
  html += "<tr><td>GPS Time (UTC)</td><td><span class=\"sensor\" id=\"gps_time\">--:--:--</span></td></tr>";
  html += "</table>";
  html += "<form method='POST'>";
  html += "<div class=\"form-container\">";
  html += "<label for='seaLevelPressure' class='form-label'>MSL Press (mB):</label>";
  html += "<input type='number' id='seaLevelPressure' name='seaLevelPressure' step='0.01' value='" + String(seaLevelPressurehPa, 2) + "'>";
  html += "<button type='submit' formaction='/update' class='button update-btn'>Update</button>";
  html += "</div>";
  html += "<div class=\"form-container\">";
  html += "<label for='filename' class='form-label'>Filename:</label>";
  html += "<input type='text' id='filename' name='filename' value='" + currentFilename + "'>";
  if (webLogging) { // Button color is based on web command state only
    html += "<button type='submit' formaction='/record' class='button record-btn-on'>Stop</button>";
  } else {
    html += "<button type='submit' formaction='/record' class='button record-btn-off'>Record</button>";
  }
  html += "</div></form>";
  html += "<script>";
  html += "function formatTime(h,m,s){ let hr=h<10?'0'+h:h; let min=m<10?'0'+m:m; let sec=s<10?'0'+s:s; return hr+':'+min+':'+sec; }";
  html += "setInterval(function() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.getElementById('atti').innerHTML = data.roll.toFixed(1) + '&deg; / ' + data.pitch.toFixed(1) + '&deg; / ' + data.yaw.toFixed(1) + '&deg;/s';";
  html += "      document.getElementById('alt').innerHTML = data.alt.toFixed(2) + ' m';";
  html += "      document.getElementById('sats').innerHTML = data.sats;";
  html += "      document.getElementById('coords').innerHTML = data.lat.toFixed(4) + ' / ' + data.lon.toFixed(4);";
  html += "      document.getElementById('gps_as').innerHTML = data.gps_alt.toFixed(1) + 'm / ' + data.speed.toFixed(1) + 'km/h';";
  html += "      document.getElementById('gps_time').innerHTML = formatTime(data.hour, data.min, data.sec);";
  html += "    });";
  html += "}, 1000);";
  html += "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"roll\":" + String(kfAngleRoll);
  json += ",\"pitch\":" + String(kfAnglePitch);
  json += ",\"yaw\":" + String(RateYaw);
  json += ",\"alt\":" + String(AltKF);
  json += ",\"sats\":" + String(gpsSatellites);
  json += ",\"lat\":" + String(gpsLat, 6);
  json += ",\"lon\":" + String(gpsLon, 6);
  json += ",\"gps_alt\":" + String(gpsAltitude);
  json += ",\"speed\":" + String(gpsSpeed);
  json += ",\"hour\":" + String(gpsHour);
  json += ",\"min\":" + String(gpsMinute);
  json += ",\"sec\":" + String(gpsSecond);
  json += "}";
  server.send(200, "application/json", json);
}

void handleUpdate() {
  if (server.hasArg("seaLevelPressure")) {
    seaLevelPressurehPa = server.arg("seaLevelPressure").toFloat();
    Serial.print("Sea Level Pressure updated to: ");
    Serial.println(seaLevelPressurehPa);
  }
  if (server.hasArg("filename")) {
      currentFilename = server.arg("filename");
  }
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting...");
}

void handleRecord() {
  if (server.hasArg("seaLevelPressure")) {
    seaLevelPressurehPa = server.arg("seaLevelPressure").toFloat();
  }
  if (server.hasArg("filename")) {
    currentFilename = server.arg("filename");
    if (currentFilename == "") {
      currentFilename = "flight_log.csv";
    }
  }
  webLogging = !webLogging;
  // Toggle the web logging command
  if(webLogging){
    Serial.println("Web record command: ON");
  } else {
    Serial.println("Web record command: OFF");
  }
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting...");
}

// --- SUPPORTIVE FUNCTIONS ---

void calibrate_gryo() {
  for (RateCalNum = 0; RateCalNum < 2000; RateCalNum++) {
    get_imu_data();
    RateCalRoll += RateRoll;
    RateCalPitch += RatePitch;
    RateYaw += RateYaw;
    delay(1);
  }
  RateCalRoll /= 2000;
  RateCalPitch /= 2000;
  RateCalYaw /= 2000;
}

void get_imu_data(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  RateRoll = (float)gx / 65.5;
  RatePitch = (float)gy / 65.5;
  RateYaw = (float)gz / 65.5;

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t axLSB = Wire.read() << 8 | Wire.read();
  int16_t ayLSB = Wire.read() << 8 | Wire.read();
  int16_t azLSB = Wire.read() << 8 | Wire.read();
  ax = (float)axLSB / 4096;
  ay = (float)ayLSB / 4096;
  az = (float)azLSB / 4096;
}

void kf_1d(float kfState, float kfUncertainty, float kfInput, float kfMeasurement) {
  kfState += 0.004 * kfInput;
  kfUncertainty += 0.004 * 0.004 * 4 * 4;
  float kfGain = kfUncertainty * 1 / (kfUncertainty + 3 * 3);
  kfState += kfGain * (kfMeasurement - kfState);
  kfUncertainty *= (1 - kfGain);
  kf1DOutput[0] = kfState;
  kf1DOutput[1] = kfUncertainty;
}

void baro_setup() {
  if (!bmp.begin(BMP280_ADDRESS)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void kf_2d_setup() {
  F = {1, 0.004, 0, 1};
  G = {0.5 * 0.004 * 0.004, 0.004};
  H = {1, 0};
  I = {1, 0, 0, 1};
  Q = G * ~G * 100.0f;
  R = {30 * 30};
  P = {0, 0, 0, 0};
  S = {0, 0};
}

void kf_2d() {
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  Invert(L);
  K = P * ~H * L;
  M = {AltBaro};
  S = S + K * (M - H * S);
  AltKF = S(0, 0); 
  VelVerticalKF = S(1, 0);
  P = (I - K * H) * P;
}

void sd_card_setup() {
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed. Logging will be disabled.");
  } else {
    Serial.println("SD card initialized.");
    File file = SD.open(("/" + currentFilename).c_str(), FILE_WRITE);
    if(file){
      file.println("Timestamp,Roll,Pitch,Altitude_KF,Latitude,Longitude,Satellites");
      file.close();
      Serial.println("Log file ready.");
    } else {
      Serial.println("Failed to create log file.");
    }
  }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    isLogging = false;
    return;
  }
  if(!file.print(message)){
    // Error
  }
  file.close();
}

void oled_setup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    Serial.println(F("OLED display initialized."));
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("System Booting...");
    display.display();
    delay(1000);
  }
}

void gps_setup() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

void updateGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
      }
      if (gps.altitude.isValid()) {
        gpsAltitude = gps.altitude.meters();
      }
      if (gps.speed.isValid()) {
        gpsSpeed = gps.speed.kmph();
      }
      if (gps.satellites.isValid()) {
        gpsSatellites = gps.satellites.value();
      }
      if (gps.time.isValid()){
        gpsHour = gps.time.hour();
        gpsMinute = gps.time.minute();
        gpsSecond = gps.time.second();
      }
    }
  }
}

// --- Original OLED display function ---
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Telemetry Status
  display.print(F("TLM Rec:")); display.println(isLogging ? F("ON ") : F("OFF"));
  display.print(F("IP:")); display.println(IP.toString());
  // GPS info
  display.print(F("Sats:")); display.print(gpsSatellites);
  if (gps.time.isValid()) {
    if (gpsHour < 10) display.print(F("0")); display.print(gpsHour); display.print(F(":"));
    if (gpsMinute < 10) display.print(F("0")); display.print(gpsMinute); display.print(F(":"));
    if (gpsSecond < 10) display.print(F("0")); display.println(gpsSecond);
  } else {
    display.println(F("--:--:--"));
  }

  // Barometer Altitude
  display.print(F("Baro ALT: ")); display.print(AltKF, 1); display.println(F("m"));

  // IMU RPY
  display.print(F("R:")); display.print(kfAngleRoll, 1);
  display.print(F(" P:")); display.print(kfAnglePitch, 1);
  display.print(F(" Y:")); display.println(RateYaw, 1);

  // GPS Coordinates
  display.print(F("lng:")); display.print(gpsLon, 1);
  display.print(F(" lat:")); display.println(gpsLat, 1);
  // GPS Altitude & Velocity
  display.print(F("ALT:")); display.print(gpsAltitude, 1);
  display.print(F("m V:")); display.print(gpsSpeed, 1); display.println(F("kmh"));
  
  display.display();
}