#include <Wire.h>
#include <SPI.h>                  // BARO: Added for BMP280
#include <Adafruit_BMP280.h>      // BARO: Added for BMP280
#include <BasicLinearAlgebra.h>   // For Kalman filter matrices

// BARO: Define sensor address and create BMP object
#define BMP280_ADDRESS 0x76
Adafruit_BMP280 bmp; // use I2C interface
using namespace BLA;

// IMPORTANT: Update this value with your local sea level pressure in hPa for accurate altitude readings.
// You can get this from a local weather station or online weather service.
const float SEA_LEVEL_PRESSURE_HPA = 1021; 

/* Accelerometer variables */
float ax, ay, az;
float AngleRoll, AnglePitch;

/* Gyroscope variables */
float RateRoll, RatePitch, RateYaw;
float RateCalRoll, RateCalPitch, RateCalYaw;
int RateCalNum;

/* 1D Kalman Filter variables*/
float kfAngleRoll = 0, kfUncertaintyAngleRoll = 2 * 2;
float kfAnglePitch = 0, kfUncertaintyAnglePitch = 2 * 2;
float kf1DOutput[] = {0, 0}; // {angle prediction, uncertainty of prediction}

/* 2D Kalman Filter variables */
// Barometer & Kalman Filter Variables
float AltBaro; // Will store the current sea-level altitude
float AltKF, VelVerticalKF;
float AccZInertial;

// Kalman Filter Matrices
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

/* Barometer variables */
float alt = 0; // BARO: Variable to store alt data

/* Timer variables */
unsigned long LoopTimer; // Changed to unsigned long for micros()

/* Main code*/
void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz (fast mode)
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  /* Calibrate sensors */
  Serial.println(F("Calibrating MPU-6050...")); calibrate_gryo();
  Serial.println(F("Initializing BMP280...")); baro_setup();
  
  /* Initialize loop timer at the end of setup */
  LoopTimer = micros(); // Initialize loop timer at the end of setup
}

void loop() {
  /* Get IMU data */
  get_imu_data();

  /* Adjust Gyro bias */
  RateRoll -= RateCalRoll;
  RatePitch -= RateCalPitch;
  RateYaw -= RateCalYaw;

  /* Calculate Pitch and Roll from Accelerometer */
  AngleRoll = atan(ay / sqrt(ax * ax + az * az)) * 180.0 / PI;
  AnglePitch = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / PI;

  /* Calculate KF for Roll */
  kf_1d(kfAngleRoll, kfUncertaintyAngleRoll, RateRoll, AngleRoll);
  kfAngleRoll = kf1DOutput[0];
  kfUncertaintyAngleRoll = kf1DOutput[1];

  /* Calculate KF for Pitch */
  kf_1d(kfAnglePitch, kfUncertaintyAnglePitch, RatePitch, AnglePitch);
  kfAnglePitch = kf1DOutput[0];
  kfUncertaintyAnglePitch = kf1DOutput[1];

  /* Calculate the KF matrix for altitude*/
  AccZInertial = -sin(AnglePitch*(3.142/180))*ax + cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))*ay + cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*az;   
  AccZInertial = (AccZInertial-1)*9.81;

  // BARO: Read altitude from BMP280
  AltBaro = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  kf_2d();

  /* Print data for debugging */
  Serial.print("Roll [°] "); Serial.print(kfAngleRoll);
  Serial.print(" Pitch [°] "); Serial.print(kfAnglePitch);
  Serial.print(" Altitude [m] "); Serial.println(AltBaro);


  /* Ensures 250 Hz loop cycle */
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

/* Supportive Functions */
void calibrate_gryo() {
  for (RateCalNum = 0; RateCalNum < 2000; RateCalNum++) {
    get_imu_data();
    RateCalRoll += RateRoll;
    RateCalPitch += RatePitch;
    RateCalYaw += RateYaw;
    delay(1);
  }
  /* Calculate gyro bias */
  RateCalRoll /= 2000;
  RateCalPitch /= 2000;
  RateCalYaw /= 2000;
}

void get_imu_data(void) {
  /* Gyroscope */
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

  /* Accelerometer */
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
  /* 1. Predict the current state of the system */
  kfState += 0.004 * kfInput;

  /* 2. Calculate uncertainty*/
  kfUncertainty += 0.004 * 0.004 * 4 * 4;

  /* 3. Calculate the Kalman gain*/
  float kfGain = kfUncertainty * 1 / (kfUncertainty + 3 * 3);

  /* 4. Update the predicted state with the measured state through the gain*/
  kfState += kfGain * (kfMeasurement - kfState);

  /* 5. Update uncertainty of predicted state */
  kfUncertainty *= (1 - kfGain);

  /* 6. KF output */
  kf1DOutput[0] = kfState;
  kf1DOutput[1] = kfUncertainty;
}

void baro_setup() {
  unsigned status = bmp.begin(BMP280_ADDRESS);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    while (1) delay(10); // Stop code execution if the sensor is not found.
  }

  /* BARO: Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void kf_2d_setup() {
  // Initialize Kalman Filter Matrices
  F = {1, 0.004,
       0, 1};  
  G = {0.5 * 0.004 * 0.004,
       0.004};
  H = {1, 0};
  I = {1, 0,
       0, 1};
  Q = G * ~G * 100.0f;
  R = {30 * 30};
  P = {0, 0,
       0, 0};
  S = {0,
       0};
}

void kf_2d() {
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  Invert(L);  // Invert L in place
  K = P * ~H * L;   // Use inverted L
  M = {AltBaro};
  S = S + K * (M - H * S);
  AltKF = S(0, 0); 
  VelVerticalKF = S(1, 0); 
  P = (I - K * H) * P;
}
