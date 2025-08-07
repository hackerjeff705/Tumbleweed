#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>
#include "FS.h"   // Added for file system
#include "SD.h"   // Added for SD card

// --- HARDWARE & SENSOR CONFIGURATION ---
#define BMP280_ADDRESS 0x76
#define SWITCH_PIN 4 // SWITCH-MOD: GPIO pin for the logging switch. Changed to GPIO 4.

// IMPORTANT: Update this with your local sea level pressure for accurate altitude.
const float SEA_LEVEL_PRESSURE_HPA = 1027; 

// --- GLOBAL OBJECTS & VARIABLES ---
Adafruit_BMP280 bmp;
using namespace BLA;

// SWITCH-MOD: This variable is now controlled by the physical switch.
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
  
  // SWITCH-MOD: Configure the switch pin as an input with an internal pull-up resistor.
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  // --- SENSOR INITIALIZATION ---
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Serial.println(F("Calibrating MPU-6050..."));
  calibrate_gryo();
  
  Serial.println(F("Initializing BMP280..."));
  baro_setup();
  
  kf_2d_setup(); // Initialize 2D Kalman filter matrices

  // --- SD CARD INITIALIZATION ---
  Serial.println("\nInitializing SD card...");
  sd_card_setup();

  Serial.println("\n------------------------------------");
  Serial.println("System ready. Toggle switch to control logging.");
  Serial.println("------------------------------------");

  LoopTimer = micros();
}

void loop() {
  // --- SWITCH-MOD: Check the state of the physical switch ---
  // digitalRead is LOW when the switch connects the pin to GND.
  if (digitalRead(SWITCH_PIN) == LOW) {
    if (!isLogging) {
      // Announce when logging starts, just once.
      Serial.println("--- Logging STARTED ---");
      isLogging = true;
    }
  } else { // Switch is HIGH (open)
    if (isLogging) {
      // Announce when logging stops, just once.
      Serial.println("--- Logging STOPPED ---");
      isLogging = false;
    }
  }

  // --- FLIGHT CONTROL CALCULATIONS ---
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

  // --- LOG DATA IF ENABLED ---
  if (isLogging) {
    String dataString = String(micros()) + "," + 
                        String(kfAngleRoll) + "," + 
                        String(kfAnglePitch) + "," + 
                        String(AltKF) + "\n";
    appendFile(SD, dataLogFilePath, dataString.c_str());

    /* Optional: Print data for live debugging */
    Serial.print("Roll [°] "); Serial.print(kfAngleRoll);
    Serial.print(" Pitch [°] "); Serial.print(kfAnglePitch);
    Serial.print(" Altitude [m] "); Serial.println(AltKF);
  }
  
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
  if(!SD.begin(5)){ // Pin 5 is the default CS pin for many ESP32 boards
    Serial.println("Card Mount Failed. Logging will be disabled.");
  } else {
    Serial.println("SD card initialized.");
    File file = SD.open(dataLogFilePath, FILE_WRITE);
    if(file){
      file.println("Timestamp,Roll,Pitch,Altitude_KF"); // CSV header
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
    isLogging = false; // Stop logging if file access fails
    return;
  }
  if(!file.print(message)){
    // Error message removed to keep serial monitor clean during flight
  }
  file.close();
}