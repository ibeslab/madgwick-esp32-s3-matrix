Arduino IDE classroom version
=============================

Open:
  ESP32_S3_Matrix_Madgwick_RTOS/ESP32_S3_Matrix_Madgwick_RTOS.ino

Keep these three files in the same sketch folder:
  ESP32_S3_Matrix_Madgwick_RTOS.ino
  madgwick.h
  madgwick.cpp

The Madgwick tuning constant is intentionally easy to find in madgwick.h:
  MADGWICK_BETA = 0.10f

The sketch calls:
  madgwickUpdateImu(...)       -> quaternion
  quaternionToRpyDeg(...)      -> roll, pitch, yaw

The filter is hand-written and uses no external Madgwick library.
