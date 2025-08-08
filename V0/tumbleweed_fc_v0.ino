#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>
#include "FS.h"
#include "SD.h"
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// --- HARDWARE & SENSOR CONFIGURATION ---
#define BMP280_ADDRESS 0x76
#define SWITCH_PIN 4
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

// IMPORTANT: Update this with your local sea level pressure for accurate altitude.
const float SEA_LEVEL_PRESSURE_HPA = 1027; 

// --- GLOBAL OBJECTS & VARIABLES ---
Adafruit_BMP280 bmp;
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
using namespace BLA;

bool isLogging = false;
const char* dataLogFilePath = "/flight_log.csv";

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

// GPS-MOD: Global variables to hold the latest GPS data
double gpsLat = 0.0, gpsLon = 0.0;
double gpsAltitude = 0.0, gpsSpeed = 0.0;
uint32_t gpsSatellites = 0;
uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;

// Kalman Filter Matrices
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

/* Timer variables */
unsigned long LoopTimer;


void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  // --- INITIALIZATION ---
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Serial.println(F("Initializing OLED...")); oled_setup();
  Serial.println(F("Calibrating MPU-6050...")); calibrate_gryo();
  Serial.println(F("Initializing BMP280...")); baro_setup();
  Serial.println(F("Initializing GPS...")); gps_setup();
  Serial.println(F("Initializing SD card...")); sd_card_setup();

  kf_2d_setup();

  Serial.println("\n------------------------------------");
  Serial.println("System ready. Toggle switch to control logging.");
  Serial.println("------------------------------------");

  LoopTimer = micros();
}

void loop() {
  // Check the state of the physical switch
  if (digitalRead(SWITCH_PIN) == LOW) {
    if (!isLogging) {
      Serial.println("--- Logging STARTED ---");
      isLogging = true;
    }
  } else {
    if (isLogging) {
      Serial.println("--- Logging STOPPED ---");
      isLogging = false;
    }
  }

  // --- TELEMETRY ---
  updateGPS(); // Check for new GPS data
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

  AltBaro = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  kf_2d();

  // --- DATA LOGGING ---
  if (isLogging) {
    String dataString = String(micros()) + "," + 
                        String(kfAngleRoll) + "," + 
                        String(kfAnglePitch) + "," + 
                        String(AltKF) + "," +
                        String(gpsLat, 6) + "," +
                        String(gpsLon, 6) + "," +
                        String(gpsSatellites) + "\n";
    appendFile(SD, dataLogFilePath, dataString.c_str());
  }
  
  // -- UPDATE DISPLAY ---
  updateOLED();

  /* Ensures 250 Hz loop cycle */
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

/* --- SUPPORTIVE FUNCTIONS --- */
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
  unsigned status = bmp.begin(BMP280_ADDRESS);
  if (!status) {
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
    File file = SD.open(dataLogFilePath, FILE_WRITE);
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
    // Error message removed
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

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // Row 1: Telemetry Status
  display.print(F("TLM rec:")); display.print(isLogging ? F("ON ") : F("OFF"));
  display.print(F(" sats:")); display.println(gpsSatellites);

  // Row 2: Barometer Altitude
  display.print(F("Baro ALT: ")); display.print(AltKF, 1); display.println(F("m"));

  // Row 3: IMU Attitude
  display.print(F("R:")); display.print(AngleRoll, 1);
  display.print(F(" P:")); display.print(AnglePitch, 1);
  display.print(F(" Y:")); display.println(RateYaw, 1);

  // Row 4: GPS Coordinates
  display.print(F("lng:")); display.print(gpsLon, 1);
  display.print(F(" lat:")); display.println(gpsLat, 1);

  // Row 5: GPS Altitude & Velocity
  display.print(F("ALT:")); display.print(gpsAltitude, 1);
  display.print(F("m V:")); display.print(gpsSpeed, 1); display.println(F("kmh"));

  // Row 6: GPS Time
  if (gps.time.isValid()) {
    if (gpsHour < 10) display.print(F("0")); display.print(gpsHour); display.print(F(":"));
    if (gpsMinute < 10) display.print(F("0")); display.print(gpsMinute); display.print(F(":"));
    if (gpsSecond < 10) display.print(F("0")); display.print(gpsSecond);
  } else {
    display.print(F("--:--:--"));
  }
  
  display.display();
}