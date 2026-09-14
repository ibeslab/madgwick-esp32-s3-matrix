# ESP32-S3-Matrix: Raw QMI8658 + FreeRTOS + Madgwick + Python 3D Viewer

Teaching demo for the Waveshare ESP32-S3-Matrix.

## Firmware

- Arduino framework
- `Wire` only for I2C transport
- direct QMI8658 register access (no sensor library)
- native ESP32 FreeRTOS tasks and a one-element queue
- hand-written 6-axis Madgwick update
- 100 Hz IMU task
- 25 Hz serial output: `roll,pitch,yaw`

Pins match the board/old project:

- SDA: GPIO 11
- SCL: GPIO 12

At boot, keep the board stationary for about 3 seconds while gyro bias is estimated.

Build/upload with PlatformIO:

```bash
cd firmware
pio run -t upload
pio device monitor
```

Expected data after the startup messages:

```text
1.23,-4.56,12.78
1.25,-4.51,12.81
```

The third number is **relative yaw**. With only accelerometer + gyroscope, yaw has no absolute reference and will drift.

## Python viewer

```bash
cd python
python -m pip install -r requirements.txt
python rpy_viewer.py /dev/ttyACM0
```

Windows example:

```bash
python rpy_viewer.py COM5
```

The viewer uses the same convention as the firmware:

- roll = rotation about body X
- pitch = rotation about body Y
- yaw = rotation about body Z
- body-to-world matrix: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`

A 3-axis body triad is rendered rather than a single arrow, because a single arrow cannot show rotation about its own axis.
