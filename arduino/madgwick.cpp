#include "madgwick.h"

#include <math.h>

namespace {

constexpr float DEG2RAD = 0.01745329251994329577f;
constexpr float RAD2DEG = 57.295779513082320876f;
constexpr float HALF_PI = 1.57079632679489661923f;

Quaternion qNormalize(Quaternion q) {
    const float n2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (!isfinite(n2) || n2 < 1e-20f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    const float inv = 1.0f / sqrtf(n2);
    q.w *= inv;
    q.x *= inv;
    q.y *= inv;
    q.z *= inv;
    return q;
}

}  // namespace

Quaternion quatAlignAccelToWorldZ(float ax, float ay, float az) {
    const float n2 = ax*ax + ay*ay + az*az;
    if (!isfinite(n2) || n2 < 1e-12f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    const float inv = 1.0f / sqrtf(n2);
    const float ux = ax * inv;
    const float uy = ay * inv;
    const float uz = az * inv;
    const float d = uz;

    if (d < -0.999999f) {
        // Nearly upside down: choose a stable axis for a 180-degree rotation.
        const float bx = (fabsf(ux) < 0.9f) ? 1.0f : 0.0f;
        const float by = (fabsf(ux) < 0.9f) ? 0.0f : 1.0f;
        const float cx = -uz * by;
        const float cy =  uz * bx;
        const float cz =  ux * by - uy * bx;
        const float cn = sqrtf(cx*cx + cy*cy + cz*cz);
        if (cn < 1e-8f) {
            return {1.0f, 0.0f, 0.0f, 0.0f};
        }
        return {0.0f, cx/cn, cy/cn, cz/cn};
    }

    // cross(measured_gravity, world_Z) = [uy, -ux, 0]
    return qNormalize({1.0f + d, uy, -ux, 0.0f});
}

Quaternion madgwickUpdateImu(Quaternion q,
                             float ax, float ay, float az,
                             float gxDps, float gyDps, float gzDps,
                             float dt) {
    // 1) Gyroscope: deg/s -> rad/s.
    const float gx = gxDps * DEG2RAD;
    const float gy = gyDps * DEG2RAD;
    const float gz = gzDps * DEG2RAD;

    float q1 = q.w;
    float q2 = q.x;
    float q3 = q.y;
    float q4 = q.z;

    // 2) Quaternion derivative predicted by angular velocity.
    float qDot1 = 0.5f * (-q2*gx - q3*gy - q4*gz);
    float qDot2 = 0.5f * ( q1*gx + q3*gz - q4*gy);
    float qDot3 = 0.5f * ( q1*gy - q2*gz + q4*gx);
    float qDot4 = 0.5f * ( q1*gz + q2*gy - q3*gx);

    // 3) Accelerometer: normalize it and use gravity for gradient correction.
    const float an2 = ax*ax + ay*ay + az*az;
    if (isfinite(an2) && an2 > 1e-12f) {
        const float invA = 1.0f / sqrtf(an2);
        ax *= invA;
        ay *= invA;
        az *= invA;

        const float _2q1 = 2.0f*q1;
        const float _2q2 = 2.0f*q2;
        const float _2q3 = 2.0f*q3;
        const float _2q4 = 2.0f*q4;
        const float _4q1 = 4.0f*q1;
        const float _4q2 = 4.0f*q2;
        const float _4q3 = 4.0f*q3;
        const float _8q2 = 8.0f*q2;
        const float _8q3 = 8.0f*q3;
        const float q1q1 = q1*q1;
        const float q2q2 = q2*q2;
        const float q3q3 = q3*q3;
        const float q4q4 = q4*q4;

        float s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay;
        float s2 = _4q2*q4q4 - _2q4*ax + 4.0f*q1q1*q2 - _2q1*ay
                 - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az;
        float s3 = 4.0f*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay
                 - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az;
        float s4 = 4.0f*q2q2*q4 - _2q2*ax + 4.0f*q3q3*q4 - _2q3*ay;

        const float sn2 = s1*s1 + s2*s2 + s3*s3 + s4*s4;
        if (isfinite(sn2) && sn2 > 1e-20f) {
            const float invS = 1.0f / sqrtf(sn2);
            s1 *= invS;
            s2 *= invS;
            s3 *= invS;
            s4 *= invS;

            // q_dot = gyro prediction - beta * normalized gradient.
            qDot1 -= MADGWICK_BETA * s1;
            qDot2 -= MADGWICK_BETA * s2;
            qDot3 -= MADGWICK_BETA * s3;
            qDot4 -= MADGWICK_BETA * s4;
        }
    }

    // 4) Integrate one sample and normalize the quaternion.
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;
    return qNormalize(q);
}

void quaternionToRpyDeg(const Quaternion &q,
                        float &rollDeg, float &pitchDeg, float &yawDeg) {
    const float sinrCosp = 2.0f * (q.w*q.x + q.y*q.z);
    const float cosrCosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    float roll = atan2f(sinrCosp, cosrCosp);

    const float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(HALF_PI, sinp);
    } else {
        pitch = asinf(sinp);
    }

    const float sinyCosp = 2.0f * (q.w*q.z + q.x*q.y);
    const float cosyCosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    float yaw = atan2f(sinyCosp, cosyCosp);

    rollDeg  = roll  * RAD2DEG;
    pitchDeg = pitch * RAD2DEG;
    yawDeg   = yaw   * RAD2DEG;
}
