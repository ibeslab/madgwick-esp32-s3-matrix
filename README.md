# ESP32-S3-Matrix Madgwick Orientation Demo


https://github.com/user-attachments/assets/460e96f9-d29d-4ff2-88bf-eb5580e405ce


A classroom-oriented implementation of **6-axis orientation estimation**
on the Waveshare ESP32-S3-Matrix using its onboard **QMI8658 accelerometer
and gyroscope**.

The project reads the IMU directly over I2C, estimates orientation using a
hand-written **Madgwick filter**, converts the resulting quaternion to
**roll, pitch, and yaw (RPY)**, and streams the result over USB serial.

A Python viewer is included to visualize the moving body coordinate frame
in 3D.

The project is intended primarily for classroom demo:

- vectors and coordinate frames,
- accelerometer and gyroscope measurements,
- quaternion orientation,
- rotation matrices,
- sensor fusion,
- Madgwick filtering,
- roll-pitch-yaw representation,
- and simple FreeRTOS task coordination.

---

## Hardware

This project targets the:

**Waveshare ESP32-S3-Matrix**

with:

- ESP32-S3 dual-core MCU
- onboard QMI8658 6-axis IMU
  - 3-axis accelerometer
  - 3-axis gyroscope
- USB-C interface
- 4 MB flash
- 2 MB PSRAM

No external IMU is required.

### IMU connection

The onboard QMI8658 is accessed through I2C:

| Signal | ESP32-S3 GPIO |
|---|---:|
| SDA | GPIO 11 |
| SCL | GPIO 12 |

The firmware probes both common QMI8658 addresses:

```text
0x6B
0x6A
```

---

# What this demo does

The complete data path is:

```text
QMI8658
   │
   │ ax, ay, az
   │ gx, gy, gz
   ▼
IMU task @ 100 Hz
   │
   ▼
FreeRTOS queue
   │
   ▼
Madgwick filter
   │
   ▼
Quaternion
[qw, qx, qy, qz]
   │
   ▼
Quaternion → RPY
   │
   ▼
roll, pitch, yaw
   │
   ▼
USB Serial
   │
   ▼
Python 3D viewer
```

The serial output is intentionally simple:

```text
roll,pitch,yaw
```

Example:

```text
12.31,-4.62,18.20
12.35,-4.57,18.24
12.40,-4.52,18.28
```

Angles are given in degrees.

---

# Why this implementation?

This project __deliberately avoids hiding the important mathematics inside
large sensor or orientation libraries__.

The firmware uses:

- Arduino `Wire` for I2C
- direct QMI8658 register access
- native ESP32 FreeRTOS tasks and queues
- a small explicit Madgwick implementation
- an explicit quaternion-to-RPY function

It does **not** require:

- SensorQMI8658
- Adafruit sensor libraries
- a Madgwick library
- an AHRS library

The intention is that students can follow the complete path from the six
raw IMU measurements to the final orientation.

---

# Madgwick module

The orientation mathematics is separated from the application:

```text
madgwick.h
madgwick.cpp
```

The main update is intentionally visible:

```cpp
q = madgwickUpdateImu(
    q,
    s.ax, s.ay, s.az,
    s.gx, s.gy, s.gz,
    dt
);
```

The resulting quaternion is converted to roll, pitch, and yaw using:

```cpp
float roll, pitch, yaw;

quaternionToRpyDeg(
    q,
    roll,
    pitch,
    yaw
);
```

This makes the firmware data flow explicit:

```text
IMU
 ↓
madgwickUpdateImu()
 ↓
Quaternion
 ↓
quaternionToRpyDeg()
 ↓
Roll, Pitch, Yaw
```

---

# Madgwick tuning

The main filter tuning parameter is defined globally in `madgwick.h`:

```cpp
static constexpr float MADGWICK_BETA = 0.10f;
```

Conceptually,

```text
smaller beta
    → trust the gyroscope more
    → smoother short-term response
    → slower gravity correction

larger beta
    → stronger accelerometer correction
    → faster tilt correction
    → more sensitivity to vibration and linear acceleration
```

For the classroom demo:

```cpp
MADGWICK_BETA = 0.10f;
```

is used as the starting value.

---

# Coordinate-frame convention

The board carries a moving **body frame** \(B\).

The Python visualization uses a fixed **world frame** \(W\).

The estimated rotation is interpreted as a body-to-world rotation:

```text
v_W = R_WB v_B
```

using the ZYX Euler-angle convention:

```text
R_WB = Rz(yaw) Ry(pitch) Rx(roll)
```

with

```text
roll  = rotation about X
pitch = rotation about Y
yaw   = rotation about Z
```

This can be interpreted equivalently as:

```text
Intrinsic ZYX:
yaw → pitch → roll
about moving/body axes
```

or

```text
Extrinsic XYZ:
roll → pitch → yaw
about fixed/world axes
```

---

# Important: yaw is relative

The QMI8658 on this board is a **6-axis IMU**:

```text
accelerometer + gyroscope
```

There is no magnetometer.

Gravity provides a long-term reference for:

```text
roll
pitch
```

but gravity contains no compass-heading information.

Therefore:

```text
Roll  → gravity-corrected
Pitch → gravity-corrected
Yaw   → relative only and will drift
```

The initial yaw should therefore be interpreted as an arbitrary reference,
not as geographic north.

---

# Startup calibration

At startup, keep the board stationary for approximately 3 seconds.

The firmware estimates:

```text
gyro X bias
gyro Y bias
gyro Z bias
```

and uses the average accelerometer vector to initialize the quaternion so
that the measured gravity direction is aligned with the world vertical.

Example startup output:

```text
ESP32-S3-Matrix raw QMI8658 + FreeRTOS + Madgwick
QMI8658 OK at I2C address 0x6B
Keep board still: calibrating gyro for ~3 seconds...
Gyro bias dps: ...
Streaming CSV: roll,pitch,yaw
```

Do not move the board during this calibration period.

---

# FreeRTOS architecture

Two tasks are used.

### IMU task

```text
Core 0
Priority 2
100 Hz
```

Responsibilities:

```text
read QMI8658
→ convert raw values
→ remove gyro bias
→ timestamp sample
→ place latest sample in queue
```

### Fusion task

```text
Core 1
Priority 1
```

Responsibilities:

```text
receive IMU sample
→ determine dt
→ execute Madgwick update
→ quaternion → RPY
→ send serial output
```

The queue contains only one sample.

Therefore the fusion task always processes the newest available IMU
measurement rather than accumulating an old backlog.

---

# Arduino IDE

Arduino IDE is the easiest way to use this project in class.

Place these three files in the same Arduino sketch directory:

```text
ESP32_S3_Matrix_Madgwick_RTOS/
├── ESP32_S3_Matrix_Madgwick_RTOS.ino
├── madgwick.h
└── madgwick.cpp
```

Open:

```text
ESP32_S3_Matrix_Madgwick_RTOS.ino
```

in Arduino IDE.

Select an ESP32-S3 board configuration and upload through USB.

Open the Serial Monitor at:

```text
115200 baud
```

Keep the board stationary during the initial calibration.

After calibration, serial output will contain:

```text
roll,pitch,yaw
```

---

# PlatformIO

A PlatformIO project is also included.

The important configuration is:

```ini
[env:esp32-s3-matrix]
platform = espressif32@6.12.0
board = esp32-s3-devkitc-1
framework = arduino

monitor_speed = 115200

board_build.flash_mode = dio
board_build.flash_size = 4MB
board_upload.flash_size = 4MB
board_build.partitions = partitions_ffat.csv

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
```

Build and upload with:

```bash
pio run -t upload
```

Then monitor:

```bash
pio device monitor
```

---

# Python 3D viewer

The repository includes:

```text
tool/rpy_viewer.py
```

The viewer reads the serial RPY stream and displays the board coordinate
frame in 3D.

Install the required packages:

```bash
pip install pyserial numpy matplotlib
```

Run, for example:

```bash
python tool/rpy_viewer.py /dev/ttyACM0
```

On Windows this might instead be:

```bash
python tool/rpy_viewer.py COM5
```

The viewer draws:

```text
Xb
Yb
Zb
```

as the rotating body-frame axes inside the fixed world coordinate system.

---

## Resetting the viewer reference

Press:

```text
R
```

inside the Python viewer to make the current orientation the new displayed
zero orientation.

Internally this uses:

```text
R_display = R0^T R_current
```

This does **not**:

- restart the ESP32,
- reset the IMU,
- or restart the Madgwick filter.

It simply selects a new reference coordinate frame.

This is useful for demonstrating that orientation is always defined
relative to a chosen frame.

---

# From IMU to quaternion

At every sample the filter receives:

```text
ax, ay, az
gx, gy, gz
dt
previous quaternion
```

and produces:

```text
qw, qx, qy, qz
```

Conceptually:

```text
accelerometer
      │
      └──── gravity correction ────┐
                                    │
gyroscope                           ▼
      └──── angular prediction → Madgwick
                                    │
                                    ▼
                              quaternion
```

The Madgwick update can be summarized as:

```text
quaternion derivative
=
gyro prediction
-
beta × gravity-correction gradient
```

followed by:

```text
q(k+1) = q(k) + q_dot dt
```

and quaternion normalization.

---

# Quaternion to roll-pitch-yaw

For

```text
q = [qw, qx, qy, qz]
```

the firmware computes the ZYX roll-pitch-yaw angles.

The function is available directly as:

```cpp
quaternionToRpyDeg(
    q,
    roll,
    pitch,
    yaw
);
```

so the mathematical representation and the human-readable representation
remain clearly separated.

---

# Suggested classroom sequence

One possible learning sequence is:

1. Observe the board X, Y and Z axes.
2. Read accelerometer values.
3. Identify the gravity vector.
4. Read gyroscope angular velocity.
5. Integrate gyro values and observe drift.
6. Introduce quaternion orientation.
7. Run Madgwick sensor fusion.
8. Convert quaternion to RPY.
9. Visualize the moving body coordinate frame.
10. Observe yaw drift.
11. Press `R` in the viewer and discuss reference frames.
12. Change `MADGWICK_BETA` and observe filter behavior.

---

# Repository structure

A typical project layout is:

```text
.
├── ESP32_S3_Matrix_Madgwick_RTOS.ino
├── madgwick.h
├── madgwick.cpp
├── platformio.ini
├── partitions_ffat.csv
├── src/
│   ├── main.cpp
│   ├── madgwick.h
│   └── madgwick.cpp
├── tool/
│   └── rpy_viewer.py
└── tests/
    └── test_madgwick.cpp
```

The root `.ino`, `madgwick.h`, and `madgwick.cpp` are provided for
Arduino IDE users.

The `src/` directory is the PlatformIO version.

---

# Educational purpose

This repository is intentionally more explicit than a typical production
IMU application.

The goal is not merely to obtain an orientation angle.

The goal is to make the complete transformation visible:

```text
sensor registers
      ↓
physical IMU quantities
      ↓
coordinate frames
      ↓
angular velocity + gravity
      ↓
sensor fusion
      ↓
quaternion
      ↓
roll-pitch-yaw
      ↓
3D visualization
```

---

# References

- Waveshare ESP32-S3-Matrix documentation
- QMI8658 6-axis inertial sensor
- Sebastian O. H. Madgwick,
  *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*

---

## Notes

This is a teaching/demo implementation rather than a navigation-grade
attitude reference system.

For applications requiring absolute heading, an additional heading
reference such as a magnetometer or another external reference is required.
