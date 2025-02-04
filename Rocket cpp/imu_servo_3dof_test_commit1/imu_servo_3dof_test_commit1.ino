#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BNO08X_RESET -1

// Initialize the PWM driver using the default address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse length constants for servos
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// 0 fin angles
const int servo1_iang = 75;
const int servo2_iang = 90;
const int servo3_iang = 75;
const int servo4_iang = 90;

const int servo_angle_max = 70;
const float servo_ang_to_fin_ang = 0.35;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

struct euler_t ypr;
struct euler_t ypr_dot;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Top frequency is about 250Hz but this report is more accurate
long reportIntervalUs = 5000;


void setReports(long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize IMU
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

  // Servo control
  Serial.println("Servo control initializing!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz

  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    // ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    // ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    // ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

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

// Helper function to map an angle (0-180) to the pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to set angles for multiple servos
// 0 to 90 is servo cw, 90 to 180 is servo ccw
void realServoAngles(int servo_num, int angle) {
  int pulseLength = angleToPulse(angle);
  pwm.setPWM(servo_num, 0, pulseLength); // Set PWM for servo
  Serial.print("Servo set to angle ");
  Serial.println(angle);
  }

void setIdealAngles(int angles[]) {
  // Subtract the initial biases to set fin angles to 0, fin angles are between -90 and +90
  int servo1 = angles[0] + servo1_iang;
  int servo2 = angles[1] + servo2_iang;
  int servo3 = angles[2] + servo3_iang;
  int servo4 = angles[3] + servo4_iang;

  // Now drive servos
  realServoAngles(0, servo1);  
  realServoAngles(1, servo2);  
  realServoAngles(2, servo3);  
  realServoAngles(3, servo4);  
  }

float pidControl(float target, float measure, long dt, float kp, float ki, float kd) {
  static float error_pre = 0;
  static float error_sum = 0;
  float error = target - measure;
  float error_dot = (error - error_pre) / (dt * 1e-6);
  error_sum = error_sum + error * (dt * 1e-6);
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
    setReports(reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true); // output in degrees
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        ypr_dot.roll = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        ypr_dot.pitch = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        ypr_dot.yaw = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        break;
    }}

    static long t_last = 0; // t_pre
    static float x_pitch_pre = 0;
    static float y_roll_pre = 0;
    static float z_yaw_pre = 0;

    long t_now = micros();
    long dt_us = t_now - t_last;
    t_last = t_now;

    Serial.print(dt_us);             Serial.print("\t");

    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);             Serial.print("\t");
    Serial.print(ypr_dot.yaw);            Serial.print("\t");
    Serial.print(ypr_dot.pitch);          Serial.print("\t");
    Serial.println(ypr_dot.roll);

    // in rocket frame, with z up
    float x_pitch = ypr.roll;
    float y_roll = ypr.pitch;
    float z_yaw = ypr.yaw;
    float z_yaw_dot = ypr_dot.yaw;

    float output_z_yaw = pidControl(0, z_yaw_dot, dt_us, 0.1, 0., 0.);
    float output_y_roll = pidControl(0, y_roll, dt_us, 1., 0., 0.);
    float output_x_pitch = pidControl(0, x_pitch, dt_us, 1., 0., 0.);

    float fin1_ang = (output_z_yaw + 0. + output_x_pitch);
    float fin2_ang = (output_z_yaw + output_y_roll + 0.);
    float fin3_ang = (output_z_yaw + 0. - output_x_pitch);
    float fin4_ang = (output_z_yaw - output_y_roll + 0.);

    // raw actuator commands
    int servo1_ang = (int)(- fin1_ang / servo_ang_to_fin_ang);
    int servo2_ang = (int)(- fin2_ang / servo_ang_to_fin_ang);
    int servo3_ang = (int)(- fin3_ang / servo_ang_to_fin_ang);
    int servo4_ang = (int)(- fin4_ang / servo_ang_to_fin_ang);

    // real actuator commands
    int servo1_ang_out = outputSaturation(servo1_ang, servo_angle_max);
    int servo2_ang_out = outputSaturation(servo2_ang, servo_angle_max);
    int servo3_ang_out = outputSaturation(servo3_ang, servo_angle_max);
    int servo4_ang_out = outputSaturation(servo4_ang, servo_angle_max);

      // dfine desired angles for the 4 servos
    int servo_angles_out[4] = {servo1_ang_out, servo2_ang_out, servo3_ang_out, servo4_ang_out}; 
    setIdealAngles(servo_angles_out);

    delay(50); // 20Hz
  }

