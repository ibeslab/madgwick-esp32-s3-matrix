#pragma once

// -----------------------------------------------------------------------------
// Minimal 6-axis Madgwick IMU filter used by the teaching demo.
//
// Inputs to madgwickUpdateImu():
//   - accelerometer: ax, ay, az (any consistent unit, e.g. g)
//   - gyroscope:     gx, gy, gz (degrees/second)
//   - dt:            sample interval in seconds
//   - q:             previous body-to-world orientation quaternion
//
// The filter normalizes the accelerometer internally and returns a normalized
// quaternion.  With a 6-axis IMU, gravity corrects roll and pitch; yaw remains
// relative and will drift because there is no absolute heading reference.
// -----------------------------------------------------------------------------

struct Quaternion {
    float w, x, y, z;
};

// Main tuning parameter for the classroom demo.
// Larger values apply stronger accelerometer/gravity correction.
static constexpr float MADGWICK_BETA = 0.10f;

// Build an initial quaternion that aligns the measured gravity direction in
// the body frame with world +Z.  Initial yaw remains arbitrary.
Quaternion quatAlignAccelToWorldZ(float ax, float ay, float az);

// One 6-axis Madgwick update.
Quaternion madgwickUpdateImu(Quaternion q,
                             float ax, float ay, float az,
                             float gxDps, float gyDps, float gzDps,
                             float dt);

// Convert q=[w,x,y,z] to ZYX roll-pitch-yaw angles, in degrees.
// Convention: R_WB = Rz(yaw) * Ry(pitch) * Rx(roll).
void quaternionToRpyDeg(const Quaternion &q,
                        float &rollDeg, float &pitchDeg, float &yawDeg);
