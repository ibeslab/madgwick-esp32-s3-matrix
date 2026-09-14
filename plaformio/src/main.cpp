#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "madgwick.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// -----------------------------------------------------------------------------
// Waveshare ESP32-S3-Matrix + onboard QMI8658
// Direct I2C register access, no IMU library, no external Madgwick library.
//
// Serial output is exactly:
//     roll_deg,pitch_deg,yaw_deg
//
// Roll/pitch are corrected by gravity. Yaw is RELATIVE only and will drift
// because this board has no magnetometer / absolute heading reference.
// -----------------------------------------------------------------------------

static constexpr int PIN_IMU_SDA = 11;
static constexpr int PIN_IMU_SCL = 12;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr uint8_t QMI_ADDR_LOW  = 0x6B;
static constexpr uint8_t QMI_ADDR_HIGH = 0x6A;
static constexpr uint8_t QMI_WHOAMI_VALUE = 0x05;

// QMI8658 register addresses used by this demo.
static constexpr uint8_t REG_WHO_AM_I = 0x00;
static constexpr uint8_t REG_CTRL1    = 0x02;
static constexpr uint8_t REG_CTRL2    = 0x03;
static constexpr uint8_t REG_CTRL3    = 0x04;
static constexpr uint8_t REG_CTRL5    = 0x06;
static constexpr uint8_t REG_CTRL7    = 0x08;
static constexpr uint8_t REG_AX_L     = 0x35;  // 12 bytes: accel XYZ + gyro XYZ

// Match the old project configuration:
// accel: +/-4 g, nominal 125 Hz (117.5 Hz in 6DOF mode)
// gyro : +/-1024 dps, 112.1 Hz
static constexpr uint8_t CTRL2_4G_125HZ       = 0x16;
static constexpr uint8_t CTRL3_1024DPS_112HZ  = 0x66;
static constexpr uint8_t CTRL5_LPF_OFF        = 0x00;
static constexpr uint8_t CTRL7_ACC_GYR_ENABLE = 0x03;

static constexpr float ACC_SCALE_G_PER_LSB = 4.0f / 32768.0f;
static constexpr float GYR_SCALE_DPS_PER_LSB = 1024.0f / 32768.0f;

static constexpr float SAMPLE_HZ = 100.0f;
static constexpr uint32_t SAMPLE_PERIOD_MS = 10;
static constexpr uint32_t SERIAL_DECIMATION = 4; // 100 Hz / 4 = 25 Hz serial

static uint8_t qmiAddress = QMI_ADDR_LOW;

struct ImuSample {
    float ax, ay, az; // g
    float gx, gy, gz; // deg/s, gyro bias already removed
    uint32_t tUs;
};

static QueueHandle_t imuQueue = nullptr;
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;
static Quaternion initialQ{1.0f, 0.0f, 0.0f, 0.0f};

// -----------------------------------------------------------------------------
// Naked QMI8658 I2C access
// -----------------------------------------------------------------------------

static bool qmiWrite8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(qmiAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool qmiRead8(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(qmiAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((int)qmiAddress, 1) != 1) return false;
    value = Wire.read();
    return true;
}

static bool qmiReadBytes(uint8_t startReg, uint8_t *dst, size_t len) {
    Wire.beginTransmission(qmiAddress);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;

    const size_t got = Wire.requestFrom((int)qmiAddress, (int)len);
    if (got != len) return false;

    for (size_t i = 0; i < len; ++i) dst[i] = (uint8_t)Wire.read();
    return true;
}

static bool qmiProbeAt(uint8_t address) {
    qmiAddress = address;
    uint8_t who = 0;
    return qmiRead8(REG_WHO_AM_I, who) && who == QMI_WHOAMI_VALUE;
}

static bool qmiInit() {
    if (!qmiProbeAt(QMI_ADDR_LOW) && !qmiProbeAt(QMI_ADDR_HIGH)) return false;

    // Enable register auto-increment (bit 6), force little-endian output
    // (bit 5 = 0), and keep the internal oscillator enabled (bit 0 = 0).
    uint8_t ctrl1 = 0;
    if (!qmiRead8(REG_CTRL1, ctrl1)) return false;
    ctrl1 |= 0x40;
    ctrl1 &= (uint8_t)~0x20;
    ctrl1 &= (uint8_t)~0x01;
    if (!qmiWrite8(REG_CTRL1, ctrl1)) return false;

    // Disable sensors while changing ranges / ODRs.
    if (!qmiWrite8(REG_CTRL7, 0x00)) return false;
    if (!qmiWrite8(REG_CTRL2, CTRL2_4G_125HZ)) return false;
    if (!qmiWrite8(REG_CTRL3, CTRL3_1024DPS_112HZ)) return false;
    if (!qmiWrite8(REG_CTRL5, CTRL5_LPF_OFF)) return false;
    if (!qmiWrite8(REG_CTRL7, CTRL7_ACC_GYR_ENABLE)) return false;

    // Gyroscope startup is much slower than accelerometer startup.
    delay(200);
    return true;
}

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool qmiReadScaled(float &ax, float &ay, float &az,
                          float &gx, float &gy, float &gz) {
    uint8_t raw[12];
    if (!qmiReadBytes(REG_AX_L, raw, sizeof(raw))) return false;

    const int16_t rax = le16(raw + 0);
    const int16_t ray = le16(raw + 2);
    const int16_t raz = le16(raw + 4);
    const int16_t rgx = le16(raw + 6);
    const int16_t rgy = le16(raw + 8);
    const int16_t rgz = le16(raw + 10);

    ax = rax * ACC_SCALE_G_PER_LSB;
    ay = ray * ACC_SCALE_G_PER_LSB;
    az = raz * ACC_SCALE_G_PER_LSB;

    gx = rgx * GYR_SCALE_DPS_PER_LSB;
    gy = rgy * GYR_SCALE_DPS_PER_LSB;
    gz = rgz * GYR_SCALE_DPS_PER_LSB;
    return true;
}

// -----------------------------------------------------------------------------
// Madgwick orientation math lives in madgwick.h / madgwick.cpp.
// This keeps the application data flow explicit and the filter reusable.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Startup calibration
// -----------------------------------------------------------------------------

static bool calibrateStationary() {
    static constexpr int N = 300; // about 3 seconds at ~100 Hz

    double sumGx = 0.0, sumGy = 0.0, sumGz = 0.0;
    double sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;
    int good = 0;

    Serial.println("Keep board still: calibrating gyro for ~3 seconds...");

    for (int i = 0; i < N; ++i) {
        float ax, ay, az, gx, gy, gz;
        if (qmiReadScaled(ax, ay, az, gx, gy, gz)) {
            sumAx += ax; sumAy += ay; sumAz += az;
            sumGx += gx; sumGy += gy; sumGz += gz;
            ++good;
        }
        delay(10);
    }

    if (good < N * 9 / 10) return false;

    const float inv = 1.0f / (float)good;
    gyroBiasX = (float)(sumGx * inv);
    gyroBiasY = (float)(sumGy * inv);
    gyroBiasZ = (float)(sumGz * inv);

    const float ax0 = (float)(sumAx * inv);
    const float ay0 = (float)(sumAy * inv);
    const float az0 = (float)(sumAz * inv);
    initialQ = quatAlignAccelToWorldZ(ax0, ay0, az0);

    Serial.printf("Gyro bias dps: %.4f, %.4f, %.4f\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);
    return true;
}

// -----------------------------------------------------------------------------
// FreeRTOS tasks
// -----------------------------------------------------------------------------

static void imuTask(void *parameter) {
    (void)parameter;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);

    for (;;) {
        float ax, ay, az, gx, gy, gz;
        if (qmiReadScaled(ax, ay, az, gx, gy, gz)) {
            ImuSample s;
            s.ax = ax; s.ay = ay; s.az = az;
            s.gx = gx - gyroBiasX;
            s.gy = gy - gyroBiasY;
            s.gz = gz - gyroBiasZ;
            s.tUs = micros();

            // Queue length is one: the fusion task always receives the newest sample.
            xQueueOverwrite(imuQueue, &s);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

static void fusionTask(void *parameter) {
    (void)parameter;
    Quaternion q = initialQ;
    uint32_t previousUs = 0;
    uint32_t printCounter = 0;

    for (;;) {
        ImuSample s;
        if (xQueueReceive(imuQueue, &s, portMAX_DELAY) != pdTRUE) continue;

        float dt = 1.0f / SAMPLE_HZ;
        if (previousUs != 0) {
            const uint32_t deltaUs = s.tUs - previousUs; // rollover-safe unsigned subtraction
            dt = deltaUs * 1.0e-6f;
            if (dt < 0.002f || dt > 0.050f) dt = 1.0f / SAMPLE_HZ;
        }
        previousUs = s.tUs;

        q = madgwickUpdateImu(q, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, dt);

        if (++printCounter >= SERIAL_DECIMATION) {
            printCounter = 0;
            float roll, pitch, yaw;
            quaternionToRpyDeg(q, roll, pitch, yaw);
            Serial.printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
        }
    }
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("ESP32-S3-Matrix raw QMI8658 + FreeRTOS + Madgwick");

    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire.setClock(I2C_HZ);
    delay(20);

    if (!qmiInit()) {
        Serial.println("ERROR: QMI8658 not found/configured.");
        while (true) delay(1000);
    }
    Serial.printf("QMI8658 OK at I2C address 0x%02X\n", qmiAddress);

    if (!calibrateStationary()) {
        Serial.println("ERROR: gyro calibration failed.");
        while (true) delay(1000);
    }

    imuQueue = xQueueCreate(1, sizeof(ImuSample));
    if (!imuQueue) {
        Serial.println("ERROR: could not create IMU queue.");
        while (true) delay(1000);
    }

    // Sampling gets the higher priority. Fusion is computationally light.
    xTaskCreatePinnedToCore(imuTask, "IMU_100Hz", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(fusionTask, "Madgwick", 4096, nullptr, 1, nullptr, 1);

    Serial.println("Streaming CSV: roll,pitch,yaw   (yaw is relative and will drift)");
}

void loop() {
    // The application lives in FreeRTOS tasks. Arduino loop is intentionally idle.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
