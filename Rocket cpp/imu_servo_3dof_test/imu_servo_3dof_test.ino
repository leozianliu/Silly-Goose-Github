#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BNO08X_RESET -1

const int servo_angle_max = 70;
const float servo_ang_to_fin_ang = 0.35;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    float sin_pitch = -2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr);
    if (abs(sin_pitch) >= 1.0) {  // gimbal lock
      ypr->yaw = 0.0;
      ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      ypr->roll = 0.0;
    }
    else {
      ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
      ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
      ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
    }

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float pidControl(float target, float measure, long dt, float kp, float ki, float kd) {
  static float error_pre = 0;
  static float error_sum = 0;
  float error = target - measure;
  float error_dot = (error - error_pre) / (dt * 1e-3);
  error_sum = error_sum + error * (dt * 1e-3);
  error_pre = error; 
  return kp * error + ki * error_sum + kd * error_dot;
}

int outputSaturation(int demand, int limit) {
  if (demand > 0) {
    return min(demand, limit);
  } 
  else {
    return max(demand, -limit);
  }
}


void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long t_last = 0; // t_pre
    static float x_pitch_pre = 0;
    static float y_roll_pre = 0;
    static float z_yaw_pre = 0;

    long t_now = micros();
    long dt_ms = t_now - t_last;
    t_last = t_now;

    Serial.print(dt_ms);             Serial.print("\t");

    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);

    // in rocket frame, with z up
    float x_pitch = ypr.yaw;
    float y_roll = ypr.pitch;
    float z_yaw = ypr.yaw;

    float z_dot_yaw = (z_yaw - z_yaw_pre) / (dt_ms * 1e-3);

    float output_z_yaw = pidControl(0, z_dot_yaw, dt_ms, 1., 0., 0.);
    float output_y_roll = pidControl(0, y_roll, dt_ms, 1., 0., 0.);
    float output_x_pitch = pidControl(0, x_pitch, dt_ms, 1., 0., 0.);

    float fin1_ang = output_z_yaw + 0. + output_x_pitch;
    float fin2_ang = output_z_yaw + output_y_roll + 0.;
    float fin3_ang = output_z_yaw + 0. - output_x_pitch;
    float fin4_ang = output_z_yaw - output_y_roll + 0.;

    // raw actuator commands
    int servo1_ang = (int)(fin1_ang / servo_ang_to_fin_ang);
    int servo2_ang = (int)(fin2_ang / servo_ang_to_fin_ang);
    int servo3_ang = (int)(fin3_ang / servo_ang_to_fin_ang);
    int servo4_ang = (int)(fin4_ang / servo_ang_to_fin_ang);

    // real actuator commands
    int servo1_ang_out = outputSaturation(servo1_ang, servo_angle_max);
    int servo2_ang_out = outputSaturation(servo2_ang, servo_angle_max);
    int servo3_ang_out = outputSaturation(servo3_ang, servo_angle_max);
    int servo4_ang_out = outputSaturation(servo4_ang, servo_angle_max);
  }

}
