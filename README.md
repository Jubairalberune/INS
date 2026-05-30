# INS – High‑Precision Inertial Navigation System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32-blue)]()

> A full‑featured inertial navigation system for ESP32, fusing MPU6050, BMP280, QMC5883L and NEO‑6M with a 15‑state Extended Kalman Filter. Includes a live web dashboard, target navigation, bump test, drift monitoring, and Pixhawk NMEA output.

---

## 📸 Preview


|:---:|
| ![SETUP](images/setup.png) |
| ![Wi-fi](images/wifi.png) |
| ![Dashboard](images/dashboard.png) | 
| ![Map](images/trail_map.png) | 
| ![Bump](images/bump_test.png) |
| ![Hardware](images/wiring1.jpeg) |
| ![Hardware](images/wiring.jpeg) |

---

## 🚀 Features

- **15‑state EKF** – Position (N/E/D), Velocity (N/E/D), Attitude (Roll/Pitch/Yaw), plus 6 bias states.
- **Thermal calibration** – 60‑second median‑based gyro bias calibration, immune to vibration spikes.
- **Adaptive ZUPT** – Variance‑based stationary detection with hard velocity clamping.
- **GPS jam detection** – Automatic fallback to pure INS when GPS error exceeds 2000 m.
- **BMP280 cross‑calibration** – Altitude offset corrected against phone origin.
- **Live web dashboard** (Wi‑Fi AP) with:
  - Artificial horizon (pitch ladder, nose up/down indicator)
  - 360° compass rose
  - Turn rate indicator
  - Live trail map (zoom/clear)
  - Target coordinates entry with arrival beep & LED
  - Bump test (drift measurement, rating 0.1 m resolution)
  - Unit toggle (m/s, km/h, mph, knots)
  - Auto time sync from mobile browser
- **NMEA output** – $GPGGA and $GPRMC at 5 Hz, compatible with Pixhawk / ArduPilot.
- **LED status** – Built‑in LED and optional external LEDs for GPS fix, ZUPT activity, and drift warning.
- **Drift logging** – 5‑second debug and 1‑minute drift summary over Serial.

---

## 🧰 Hardware Requirements

| Component       | Model / Pinout                        |
|----------------|---------------------------------------|
| MCU             | ESP32‑N4 (or any ESP32 with ≥4 MB flash, ≥300 KB free RAM) |
| IMU             | MPU6050 (I2C address 0x68)            |
| Barometer       | BMP280 (I2C address 0x76 or 0x77)     |
| Magnetometer    | QMC5883L (I2C address 0x0D)           |
| GPS             | NEO‑6M (UART1: RX=GPIO16, TX=GPIO17)  |
| Pixhawk (opt.)  | UART2: TX=GPIO33, RX=GPIO32           |
| Buzzer/LED      | GPIO26 (active low buzzer or LED)     |
| LEDs (optional) | GPIO2 (built‑in), GPIO4, GPIO5        |

### Wiring Diagram

![Wiring](images/wiring.png)

**I2C connections (all sensors share the same bus):**

| ESP32 | MPU6050 | BMP280 | QMC5883L |
|-------|---------|--------|----------|
| 3.3V  | VCC     | VCC    | VCC      |
| GND   | GND     | GND    | GND      |
| GPIO21| SDA     | SDA    | SDA      |
| GPIO22| SCL     | SCL    | SCL      |

**UART connections:**

| ESP32 | NEO‑6M | Pixhawk (optional) |
|-------|--------|--------------------|
| GPIO16| TX     | –                  |
| GPIO17| RX     | –                  |
| GPIO33| –      | GPS port RX        |
| GPIO32| –      | GPS port TX (unused)|

*Ground must be shared between all modules.*

---

## 📦 Software Setup

1. **Install Arduino IDE** (≥1.8.19) or PlatformIO with **ESP32 board support** (≥2.0.0).
2. **Install libraries** via Library Manager:
   - `Adafruit BMP280`
   - `QMC5883LCompass`
   - `TinyGPS++`
   - *(No Adafruit MPU6050 – this code uses raw I2C)*
3. **Download the code** – open `INS_v14.6.ino`.
4. **Select your board** – e.g., `ESP32 Dev Module`, **Partition Scheme: `Huge App`** (to fit the large HTML).
5. **Compile and upload** to your ESP32.

> ⚠️ **Important:** The HTML dashboard is stored in PROGMEM and is ~20 KB. Ensure your partition scheme has enough space. If you get a “sketch too big” error, use `Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)`.

---

## 🔧 Operation

1. **Power the ESP32**. Wait **60 seconds** for the thermal calibration – keep the device perfectly still on a flat surface.
2. **Connect to Wi‑Fi** – SSID: `INS by Jubair`, Password: `98765ins`.
3. **Open a browser** at `http://192.168.4.1`.
4. **On the setup page**:
   - Click **`USE PHONE GPS`** to automatically fill your current coordinates (requires HTTPS – works on mobile browsers).
   - Or manually enter **Origin Latitude**, **Origin Longitude**, **Altitude**.
   - *Optional:* Enter **Target Latitude/Longitude** and arrival radius (default 10 m). A beep and LED will signal arrival.
   - Click **`START INS`**.
5. **Dashboard** shows live:
   - Position, velocity, attitude
   - Live trail map (pan/zoom, clear)
   - Horizon and compass instruments
   - GPS status and environment data
6. **Bump test** – press `BUMP TEST`, move the device ~1 m away, then press `RETURN TO START`. The displayed error tells you the drift.
7. **Reset** – press the red `RESET` button to stop INS and return to setup (confirmation countdown).

---

## 📊 Bump Test Interpretation

| Drift (meters) | Rating      | Action |
|----------------|-------------|--------|
| < 0.10         | EXCELLENT   | Perfect – your INS is highly accurate. |
| 0.10 – 0.29    | GOOD        | Acceptable for most applications. |
| 0.30 – 0.49    | ACCEPTABLE  | Tune ZUPT or reduce vibration. |
| ≥ 0.50         | FIX ZUPT/Q  | Re‑calibrate, check sensor mounting, or replace MPU6050. |

---

## 🛠 Troubleshooting

| Symptom                     | Most likely cause                          | How to fix |
|-----------------------------|--------------------------------------------|------------|
| **No Wi‑Fi AP appears**     | Power supply insufficient (voltage drop)   | Use a stable 5V/2A USB power supply. |
| **GPS shows “NOT CONNECTED”**| Wiring or baud rate mismatch               | Check GPS TX → GPIO16, GND, power. Ensure `SerialGPS.begin(9600,...)`. |
| **Yaw drifts to 39845 degrees**| Gyro bias calibration corrupted by vibration | Re‑calibrate on a thick foam pad. Ensure no movement during 60 s. |
| **Velocity > 1000 m/s while still**| I2C corrupt readings (noise)           | Add 4.7 kΩ pull‑up resistors on SDA/SCL lines. Use short wires. |
| **MPU6050 not found**       | AD0 pin floating or 5 V power              | Connect AD0 to GND (address 0x68). Power MPU6050 from 3.3 V only. |
| **ESP32 reboots during calibration** | Watchdog timeout                      | Code already feeds WDT – if persists, increase `timeout_ms` to 20000. |
| **Web page empty / white**  | Raw string literal parsing issue           | The code uses a normal C string – re‑upload. Clear browser cache. |

---

## 📈 Theory of Operation

### Extended Kalman Filter (15 states)

The state vector is:

\[
\mathbf{x} = \begin{bmatrix}
p_n & p_e & p_d & v_n & v_e & v_d & \phi & \theta & \psi & b_{ax} & b_{ay} & b_{az} & b_{gx} & b_{gy} & b_{gz}
\end{bmatrix}^T
\]

**Prediction step** – uses IMU acceleration and gyro rates with a direction‑cosine matrix to propagate position, velocity and attitude.

**Update steps**:
- **ZUPT** – When acceleration variance is low, velocity is forced to zero and position covariance is clamped.
- **Barometer** – corrects the vertical (Z) position.
- **GPS** – corrects horizontal (N/E) position with HDOP‑weighted gain.
- **Bias estimation** – Both accelerometer and gyro biases are estimated online.

**GPS jam handling** – If the INS prediction differs from GPS by more than 2000 m, GPS is ignored until the error drops below that threshold.

### Median Calibration

During 60 seconds of stillness, the raw gyro Z‑axis values are stored in an array, sorted, and the median is taken as the bias. This rejects outliers caused by mechanical vibration.

### Rate‑of‑Change Rejection

`mpuRead()` compares consecutive gyro readings; if the change exceeds 2 rad/s (physically impossible for a stationary sensor at 200 Hz), the reading is considered corrupt and the previous good value is reused. A low‑pass filter further smooths the data.

---

## 🖥 Web Dashboard API

The ESP32 serves a JSON API at the following endpoints:

| Endpoint       | Method | Description |
|----------------|--------|-------------|
| `/data`        | GET    | Returns live telemetry (lat,lon,alt,vel,att,etc.) |
| `/status`      | GET    | Returns `{running, calDone, uptime}` |
| `/caldata`     | GET    | Returns calibration progress `{done, p}` |
| `/env`         | GET    | Returns `{al, tc, pp}` (baro altitude, temp, pressure) |
| `/reset`       | POST   | Stops the INS and resets internal state |
| `/bump_start`  | POST   | Starts a bump test, records current position |
| `/bump_return` | POST   | Computes drift and returns JSON error |
| `/start`       | POST   | Receives origin and optional target coordinates |

---

## 🔬 Performance & Drift Expectations

With a **healthy MPU6050** on a vibration‑isolated mount, the expected performance is:

| Condition | Position drift per hour |
|-----------|------------------------|
| Stationary, ZUPT active | < 5 m |
| Walking at 1 m/s, no GPS | 30‑100 m |
| With GPS (HDOP < 1.5) | < 2 m (bounded) |
| With GPS jammed for 30 s | < 15 m error when GPS returns |

If you observe **>1 km drift per hour while stationary**, your MPU6050 is defective – replace it or upgrade to a BNO055.

---

## 📝 License

This project is open source under the **MIT License** – see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Jubair Al Berune**  
GitHub: [@jubairalberune](https://github.com/jubairalberune) *(replace with your actual handle)*

---

## 🙏 Acknowledgments

- Claude AI for extensive debugging collaboration.
- The open‑source community for the libraries (Adafruit BMP280, QMC5883LCompass, TinyGPS++).
- The ESP32 team for a capable and affordable platform.

---

## ⭐ Star the repo

If you find this project useful, please **star** it on GitHub and share your results!
