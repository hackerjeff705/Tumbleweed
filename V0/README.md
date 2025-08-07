# Tumbleweed Flight Computer V0

This project code serves as the core of a flight computer, designed to accurately determine an object's orientation (roll and pitch) and altitude. It fuses data from an MPU-6050 Inertial Measurement Unit (IMU) and a BMP280 barometric pressure sensor using two separate Kalman filters. This process results in smooth and reliable estimations that are far more accurate than using any single sensor alone.

-----

## Features 📋

  * **Sensor Fusion**: Combines accelerometer, gyroscope, and barometer data for robust state estimation.
  * **Roll & Pitch Estimation**: Provides stable angle measurements using a 1D Kalman filter.
  * **Altitude & Vertical Velocity Estimation**: Provides stable altitude and vertical speed measurements using a 2D Kalman filter.
  * **Gyroscope Calibration**: Includes a startup routine to calibrate the gyroscope and remove its inherent bias.
  * **Data Logging**: Records key flight data (angles, altitude, velocity, etc.) to a Micro SD card for post-flight analysis.
  * **Real-time Display**: Shows critical flight information on an OLED screen for immediate feedback.
  * **Fixed Loop Rate**: Ensures consistent performance by maintaining a 250 Hz loop cycle.

-----

## Hardware & Libraries 🛠️

### Required Hardware

  * **Microcontroller**: ESP32-WROOM
  * **IMU**: MPU-6050 6-axis Accelerometer + Gyroscope
  * **Barometer**: BMP280 Barometric Pressure + Temperature sensor
  * **Storage**: HW-125 Micro SD card reader
  * **Display**: OLED Display (SSD1306)
  * **Power**: BreadVolt PSU

### Required Libraries

You'll need to install the following libraries through the Arduino IDE Library Manager:

  * `Wire.h` (included with the ESP32 core)
  * `SPI.h`
  * `Adafruit BMP280 Library`
  * `Adafruit GFX Library`
  * `Adafruit SSD1306`
  * `Adafruit Unified Sensor` (dependency for the BMP280 library)
  * `BasicLinearAlgebra` by Tom Stewart

-----

## How It Works 🧠

This code tackles a classic problem in navigation: sensors are imperfect.

  * **Accelerometers** can determine angles relative to gravity, but they are very sensitive to external forces and vibration (noise).
  * **Gyroscopes** measure the rate of rotation very well, but they suffer from **drift**, where small errors accumulate over time, causing the angle estimate to become inaccurate.
  * **Barometers** can measure altitude based on air pressure, but the readings can be noisy and fluctuate with weather changes.

To get the best of all worlds, we use **sensor fusion** powered by a **Kalman Filter**.

### Roll & Pitch Estimation (1D Kalman Filter)

For both the roll and pitch axes, a simple 1D Kalman Filter is used. Here's the logic:

1.  **Predict**: The gyroscope measures the *rate of change* of the angle ($RateRoll$, $RatePitch$). We use this to predict what the *next* angle will be, knowing that this prediction will slowly drift.
2.  **Measure**: The accelerometer measures the *absolute* angle ($AngleRoll$, $AnglePitch$) based on the direction of gravity. This measurement is noisy but doesn't drift.
3.  **Correct**: The Kalman filter intelligently combines the drifted prediction from the gyro with the noisy measurement from the accelerometer. It produces a final estimate ($kfAngleRoll$, $kfAnglePitch$) that is more accurate than either sensor could provide on its own.

This process is handled by the `kf_1d()` function.

### Altitude Estimation (2D Kalman Filter)

A more complex 2D Kalman filter is used to determine altitude ($AltKF$) and vertical velocity ($VelVerticalKF$).

1.  **Predict**: The accelerometer's Z-axis reading ($AccZInertial$), once corrected for the device's tilt, tells us the vertical acceleration. By integrating this acceleration over time, we can predict the change in altitude and vertical velocity. Like the gyro, this is prone to drift.
2.  **Measure**: The BMP280 barometer provides a direct measurement of the current altitude ($AltBaro$) based on air pressure. This measurement is noisy but provides an absolute reference.
3.  **Correct**: The 2D Kalman filter, implemented in the `kf_2d()` function, uses linear algebra (matrices) to fuse the predicted altitude (from the accelerometer) with the measured altitude (from the barometer). This produces a smooth, stable estimate of both the current altitude and vertical velocity.

-----

## Setup & Usage 🚀

### 1\. Wiring

Connect the sensors and peripherals to your ESP32.

**I2C Devices (MPU-6050, BMP280, SSD1306 OLED)**
  * **SDA** -> GPIO 21
  * **SCL** -> GPIO 22
  * **VCC** -> 3.3V
  * **GND** -> GND

**SPI Device (Micro SD Card Reader)**
  * **CS**   -> GPIO 5
  * **SCK**  -> GPIO 18
  * **MISO** -> GPIO 19
  * **MOSI** -> GPIO 23
  * **VCC**  -> 3.3V
  * **GND**  -> GND

### 2\. Configure Sea Level Pressure

For accurate altitude readings from the BMP280, you **must** update the `SEA_LEVEL_PRESSURE_HPA` constant in the code.

```cpp
// IMPORTANT: Update this value with your local sea level pressure in hPa.
// You can get this from a local weather station or online weather service.
const float SEA_LEVEL_PRESSURE_HPA = 1013.25; // Example: 1021 hPa
```

You can find your local sea-level pressure from an online weather service or a nearby airport's METAR report.

### 3\. Upload and Run

1.  Install the required libraries.
2.  Upload the sketch to your ESP32.
3.  Keep the device **flat and stationary** for the first few seconds while the gyroscope calibrates.
4.  Watch the OLED display for real-time data.
5.  For detailed debugging, open the **Serial Monitor** at a baud rate of **115200**.
6.  After use, you can retrieve the `flight_data.csv` file from the SD card for analysis.

<!-- end list -->

```
Calibrating MPU-6050...
Initializing BMP280...
Roll [°] 0.03 Pitch [°] -0.15 Altitude [m] 123.45
Roll [°] 0.04 Pitch [°] -0.14 Altitude [m] 123.46
...
```