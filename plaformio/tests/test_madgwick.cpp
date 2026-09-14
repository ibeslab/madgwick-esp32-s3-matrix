#include <cassert>
#include <cmath>
#include <cstdio>

#include "../src/madgwick.h"

static bool nearf(float a, float b, float tol) {
    return std::fabs(a - b) <= tol;
}

int main() {
    {
        Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
        float roll = 99.0f, pitch = 99.0f, yaw = 99.0f;
        quaternionToRpyDeg(q, roll, pitch, yaw);
        assert(nearf(roll, 0.0f, 1e-4f));
        assert(nearf(pitch, 0.0f, 1e-4f));
        assert(nearf(yaw, 0.0f, 1e-4f));
    }

    {
        const float s = std::sqrt(0.5f);
        Quaternion q{s, s, 0.0f, 0.0f};  // +90 deg roll
        float roll, pitch, yaw;
        quaternionToRpyDeg(q, roll, pitch, yaw);
        assert(nearf(roll, 90.0f, 0.05f));
        assert(nearf(pitch, 0.0f, 0.05f));
        assert(nearf(yaw, 0.0f, 0.05f));
    }

    {
        Quaternion q = quatAlignAccelToWorldZ(0.0f, 0.0f, 1.0f);
        assert(nearf(q.w, 1.0f, 1e-5f));
        assert(nearf(q.x, 0.0f, 1e-5f));
        assert(nearf(q.y, 0.0f, 1e-5f));
        assert(nearf(q.z, 0.0f, 1e-5f));
    }

    {
        Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
        q = madgwickUpdateImu(q,
                              0.0f, 0.0f, 1.0f,
                              0.0f, 0.0f, 0.0f,
                              0.01f);
        assert(nearf(q.w, 1.0f, 1e-5f));
        assert(nearf(q.x, 0.0f, 1e-5f));
        assert(nearf(q.y, 0.0f, 1e-5f));
        assert(nearf(q.z, 0.0f, 1e-5f));
    }

    {
        // One second of +90 deg/s yaw, integrated at 100 Hz.  With no valid
        // accelerometer vector, this tests the gyro/quaternion path alone.
        Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 100; ++i) {
            q = madgwickUpdateImu(q,
                                  0.0f, 0.0f, 0.0f,
                                  0.0f, 0.0f, 90.0f,
                                  0.01f);
        }
        float roll, pitch, yaw;
        quaternionToRpyDeg(q, roll, pitch, yaw);
        assert(nearf(roll, 0.0f, 0.1f));
        assert(nearf(pitch, 0.0f, 0.1f));
        assert(nearf(yaw, 90.0f, 0.2f));
    }

    std::puts("madgwick tests passed");
    return 0;
}
