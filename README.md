markdown
# INS  – High‑Precision 15‑State EKF Inertial Navigation System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

This project implements a **GPS‑augmented Inertial Navigation System (INS)** on an **ESP32** using a **15‑state Extended Kalman Filter (EKF)**. It fuses data from:

- **MPU6050** (accelerometer + gyroscope) – raw I2C with rate‑of‑change rejection  
- **BMP280** (barometer) – altitude correction  
- **QMC5883L** (magnetometer) – heading (yaw) reference  
- **NEO‑6M** (GPS) – absolute position & velocity correction  

All sensor fusion, ZUPT (Zero Velocity Update), and drift countermeasures run on the ESP32. A **built‑in web dashboard** (Wi‑Fi AP) provides real‑time 3D tracking, live trail map, target navigation, bump test, and full telemetry.

---

## Features

- **15‑state EKF** (posN/E/D, velN/E/D, roll/pitch/yaw, accel bias x3, gyro bias x3)
- **Thermal calibration** (60 s median gyro bias, robust against vibration)
- **Adaptive ZUPT** with variance‑based stationary detection
- **GPS jam detection** (fallback to pure INS)
- **BMP280 cross‑calibration** against phone origin
- **Live web dashboard**:
  - Artificial horizon with pitch ladder and nose‑up/down indicator
  - 360° compass rose
  - Turn rate indicator
  - Live trail map (canvas, zoom/clear)
  - Target coordinates entry with arrival beep & LED (GPIO26)
  - Bump test (drift measurement, 0.1m resolution)
  - Unit toggle (m/s, km/h, mph, knots)
  - Auto time sync from mobile browser
- **NMEA output** ($GPGGA, $GPRMC) at 5Hz for Pixhawk / flight controller
- **LED status indicators** (on‑board LED and external pins)

---

## Hardware Requirements

| Component       | Model / Pinout                        |
|----------------|---------------------------------------|
| MCU             | ESP32‑N4 (or any ESP32 with sufficient RAM) |
| IMU             | MPU6050 (I2C address 0x68)            |
| Barometer       | BMP280 (I2C address 0x76 or 0x77)     |
| Magnetometer    | QMC5883L (I2C address 0x0D)           |
| GPS             | NEO‑6M (UART1: RX=GPIO16, TX=GPIO17)  |
| Pixhawk         | UART2: TX=GPIO33, RX=GPIO32 (optional)|
| Buzzer/LED      | GPIO26 (optional, for target arrival) |
| LEDs (optional) | GPIO2 (built‑in), GPIO4, GPIO5        |

### Wiring (I2C & UART)


ESP32 MPU6050 / BMP280 / QMC5883L


3.3V ──── VCC
GND ──── GND
GPIO21 ──── SDA
GPIO22 ──── SCL


ESP32 NEO‑6M


GPIO16 ──── TX (GPS -> ESP)
GPIO17 ──── RX (ESP -> GPS) optional
GND ──── GND
3.3V ──── VCC (or 5V if module tolerant)


ESP32 Pixhawk (GPS2 port)


GPIO33 ──── RX (Pixhawk GPS port)
GPIO32 ──── TX (optional, not used)
GND ──── GND



## Software Setup

1. Install **Arduino IDE** (or PlatformIO) with **ESP32 board support** (≥2.0.0).
2. Install libraries via Library Manager:
   - `Adafruit BMP280`
   - `QMC5883LCompass`
   - `TinyGPS++`
   - (No Adafruit MPU6050 – the code uses raw I2C)
3. Open `INS.ino` and select your ESP32 board (e.g., `ESP32 Dev Module`).
4. Compile and upload.

---

## Operation

1. Power the ESP32. Wait for the 60‑second thermal calibration (keep device **perfectly still** on a flat surface).
2. Connect to the Wi‑Fi access point:  
   **SSID:** `INS by Jubair`  
   **Password:** `98765ins`
3. Open a browser at `http://192.168.4.1`
4. On the setup page:
   - Click `USE PHONE GPS` to get your current location, or manually enter origin coordinates.
   - (Optional) Set a target latitude/longitude and arrival radius.
   - Click `START INS`.
5. The dashboard will show live position, velocity, attitude, a moving trail on the map, and GPS status.
6. To test drift: place device still, press `BUMP TEST`, move ~1 m, return, and read the error.

---

## Troubleshooting

| Symptom                     | Likely cause                                   | Fix                                   |
|-----------------------------|------------------------------------------------|---------------------------------------|
| No Wi‑Fi AP                 | Power supply insufficient                     | Use a stable 5V/2A USB supply         |
| GPS not connected           | Wiring or baud rate (9600)                    | Check GPS TX → GPIO16, GND             |
| Yaw drifts to 39845 degrees | MPU6050 gyro bias not calibrated (vibration)  | Re‑calibrate on a foam pad, no movement|
| Velocity > 1000 m/s         | I2C corrupt readings                          | Add 4.7k pull‑ups on SDA/SCL, use 3.3V |
| MPU6050 not found           | AD0 floating or 5V power                      | Connect AD0 to GND, use 3.3V           |

---

## Theory of Operation

The EKF maintains a 15‑state vector:

\[
\mathbf{x} = \begin{bmatrix}
p_n & p_e & p_d & v_n & v_e & v_d & \phi & \theta & \psi & b_{ax} & b_{ay} & b_{az} & b_{gx} & b_{gy} & b_{gz}
\end{bmatrix}^T
\]

- **Prediction** uses the IMU (accelerometer + gyro) with a direction‑cosine matrix to propagate position, velocity, and attitude.
- **Update** uses:
  - **ZUPT** (Zero Velocity Update) – when variance of accelerometer magnitude is low, velocity is forced to zero and position covariance is clamped.
  - **Barometer** – corrects vertical (Z) position.
  - **GPS** – corrects horizontal (N/E) position (HDOP‑weighted).
- **Bias estimation** – both accelerometer and gyro biases are tracked in the state vector.
- **GPS jam handling** – if the GPS position differs from the INS prediction by more than 2000 m, GPS is ignored until the error drops.

The median‑based gyro calibration at boot rejects vibration outliers, and the rate‑of‑change filter in `mpuRead()` discards physically impossible I2C glitches.

---

## License

MIT – see [LICENSE](LICENSE) file.

---

## Author

**Jubair Al Berune**  
Project from extensive debugging collaboration with Claude.

---

## Repository

[GitHub – INS](https://github.com/Jubairalberune/INS)
