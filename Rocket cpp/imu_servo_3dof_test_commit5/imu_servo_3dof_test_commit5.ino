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
const float servo_ang_to_fin_ang = 0.356;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

struct cartesian_t {
  float x;
  float y;
  float z;
};

struct pid_error_t {
  float error_pre;
  float error_sum;
};

struct euler_t ypr;
struct euler_t ypr_dot;

struct cartesian_t body_acc;
struct cartesian_t enu_acc;
struct cartesian_t enu_vel;
struct cartesian_t enu_pos;

struct pid_error_t x_pitch_rate_pid = {0, 0};
struct pid_error_t y_roll_rate_pid = {0, 0};
struct pid_error_t z_yaw_pid = {0, 0};
struct pid_error_t x_pitch_pid = {0, 0};
struct pid_error_t y_roll_pid = {0, 0};

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Top frequency is about 250Hz but this report is more accurate
long reportIntervalUs = 4000;


void setReports(long report_interval) {
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED
  pinMode(3, OUTPUT); // Buzzer connected to GP3 (5)


  Serial.begin(115200);

  // Try to initialize IMU
  while (!bno08x.begin_I2C(0x4B)) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100); 
    digitalWrite(LED_BUILTIN, LOW);
    delay(100); 
  }

  setReports(reportIntervalUs);

  delay(100);

  // Servo control
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz

  delay(100);

  // Finish initialization; beep
  for (int i=0; i<3; i++) {
  digitalWrite(3, HIGH);
  delay(300);
  digitalWrite(3,LOW);
  delay(300);}
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

void acc_ZYX_body_to_inertial(float x_ang, float y_ang, float z_ang, cartesian_t* body_acc, cartesian_t* enu_acc) { // ANGLES IN RAD!!!!!
    // Rotation matrix for ZYX Euler angles
    float R_mat[3][3] = {
        {cos(y_ang) * cos(z_ang), -cos(x_ang) * sin(z_ang) + sin(x_ang) * sin(y_ang) * cos(z_ang), sin(x_ang) * sin(z_ang) + cos(x_ang) * sin(y_ang) * cos(z_ang)},
        {cos(y_ang) * sin(z_ang), cos(x_ang) * cos(z_ang) + sin(x_ang) * sin(y_ang) * sin(z_ang), -sin(x_ang) * cos(z_ang) + cos(x_ang) * sin(y_ang) * sin(z_ang)},
        {-sin(y_ang), sin(x_ang) * cos(y_ang), cos(x_ang) * cos(y_ang)}
    };

    float acc_body[3] = {body_acc->x, body_acc->y, body_acc->z};

    float acc_enu[3];
    matrixTimesVector3D(R_mat, acc_body, acc_enu);  // Pass acc_enu to be filled by the function

    // Store the results in enu_acc
    enu_acc->x = acc_enu[0];
    enu_acc->y = acc_enu[1];
    enu_acc->z = acc_enu[2];
}

void matrixTimesVector3D(float mat[3][3], float vec[3], float result[3]) {
    // Perform matrix-vector multiplication and store the result in the result array
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;  // Initialize the result cell
        for (int j = 0; j < 3; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
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

float pidControl(pid_error_t* error_save, float target, float measure, float dt, float kp, float ki, float kd, bool anti_wind, bool saturation) { // dt in micro second
  float error_pre = error_save->error_pre;
  float error_sum = error_save->error_sum;
  float error = target - measure;
  float error_dot = (error - error_pre) / (dt * 1e-6);
  
  error_save->error_pre = error;

  // Anti-windup
  if (!(saturation && anti_wind)) {
    error_save->error_sum = error_sum + error * (dt * 1e-6); // No saturation
  }

  float output = kp * error + ki * error_save->error_sum + kd * error_dot;
  return output;
}

int outputSaturation(int demand, int limit) {
  if (demand > 0) {
    return min(demand, limit);
  } 
  else {
    return max(demand, -limit);
  }
}

bool saturationDetection(int demand, int limit) {
  if (demand > 0) {
    return (demand > limit);
  } 
  else {
    return (demand < -limit);
  }
}

void order2integrator(cartesian_t* acc, float dt_us, cartesian_t* pos_out, cartesian_t* vel_out) {
    static cartesian_t acc_old = {0, 0, 0};
    static cartesian_t vel_old = {0, 0, 0};
    static cartesian_t pos_old = {0, 0, 0};
    
    cartesian_t vel = {
        vel_old.x + (acc_old.x + acc->x) / 2 * dt_us * 1e-6f,
        vel_old.y + (acc_old.y + acc->y) / 2 * dt_us * 1e-6f,
        vel_old.z + (acc_old.z + acc->z) / 2 * dt_us * 1e-6f
    };
    
    cartesian_t pos = {
        pos_old.x + (vel_old.x + vel.x) / 2 * dt_us * 1e-6f,
        pos_old.y + (vel_old.y + vel.y) / 2 * dt_us * 1e-6f,
        pos_old.z + (vel_old.z + vel.z) / 2 * dt_us * 1e-6f
    };
    
    acc_old = *acc;
    vel_old = vel;
    pos_old = pos;
    
    *pos_out = pos;
    *vel_out = vel;
}

// float actuationFactor(float relative_height, float vertical_speed) {
//     if (relative_height < 8) { // 7m is the height of the launch rail
//         return 0; // fins should not move when the rocket is on the launch rail
//     }
//     else if (vertical_speed > 10) {
//         return (30 / vertical_speed) ** 2; // PID tuned at 30 m/s
//     }
//     else {
//         return 9 // max k_act is 3^2
//     }
// }

float actuationFactor(float vertical_speed) {
    if (vertical_speed > 10) {
        return pow((30 / vertical_speed), 2); // PID tuned at 30 m/s
    }
    else {
        return 9; // max k_act is 3^2
    }
}

void loop() {

  if (bno08x.wasReset()) {
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
      case SH2_LINEAR_ACCELERATION:
        body_acc.x = sensorValue.un.linearAcceleration.x;
        body_acc.y = sensorValue.un.linearAcceleration.y;
        body_acc.z = sensorValue.un.linearAcceleration.z;
        break;
    }}

    static long t_last = 0; // t_pre

    long t_now = micros();
    float dt_us = t_now - t_last;
    t_last = t_now;

    // in rocket frame, with z up; rotation is ZYX
    float x_pitch = ypr.roll; // x is still x, just a differrent name
    float y_roll = ypr.pitch; // y is still y, just a differrent name
    float z_yaw = ypr.yaw;
    float z_yaw_dot = ypr_dot.yaw;

    acc_ZYX_body_to_inertial(x_pitch / RAD_TO_DEG, y_roll / RAD_TO_DEG, z_yaw / RAD_TO_DEG, &body_acc, &enu_acc); // ANGLE INPUTS IN RAD!!!
    float acc_enu_x = enu_acc.x;
    float acc_enu_y = enu_acc.y;
    float acc_enu_z = enu_acc.z;

    //order2integrator(&enu_acc, dt_us, &enu_pos, &enu_vel);

    // if (Serial){
    // // Serial.print(t_now);           Serial.print(",");
    // // Serial.print(x_pitch);          Serial.print(",");
    // // Serial.print(y_roll);          Serial.print(",");
    // // Serial.print(z_yaw);          Serial.print(",");

    // // Serial.print(enu_acc.x);          Serial.print(",");
    // // Serial.print(enu_acc.y);          Serial.print(",");
    // Serial.println(enu_acc.z);
    // }

    // Actuation factor
    float vertical_speed = 30; // FILLER!!!
    float actuation_factor = actuationFactor(vertical_speed);

    // Initialize saturation flags
    static bool x_pitch_saturation = false;
    static bool y_roll_saturation = false;
    static int servo1_ang = 0;
    static int servo2_ang = 0;
    static int servo3_ang = 0;
    static int servo4_ang = 0;

    // saturation detection
    bool servo1_saturation = saturationDetection(servo1_ang, servo_angle_max);
    bool servo2_saturation = saturationDetection(servo2_ang, servo_angle_max);
    bool servo3_saturation = saturationDetection(servo3_ang, servo_angle_max);
    bool servo4_saturation = saturationDetection(servo4_ang, servo_angle_max);
    if (servo1_saturation || servo3_saturation) {
        x_pitch_saturation = true;
        Serial.println("Saturation 13");
    }
    else {
        x_pitch_saturation = false;
    }
    if (servo2_saturation || servo4_saturation) {
        y_roll_saturation = true;
        Serial.println("Saturation 24");
    }
    else {
        y_roll_saturation = false;
    }

    // PID control
    float output_z_yaw = pidControl(&z_yaw_pid, 0, z_yaw_dot, dt_us, 0.05, 0., 0., false, false);

    //float output_y_roll_rate = pidControl(0, y_roll, dt_us, 10., 4., 0., true, y_roll_saturation);
    float output_y_roll_rate = pidControl(&y_roll_pid, 0, y_roll, dt_us, 5., 0.5, 0., true, y_roll_saturation);
    float output_y_roll = pidControl(&y_roll_rate_pid, output_y_roll_rate, 0, dt_us, 0.1, 0., 0., false, false);

    //float output_x_pitch_rate = pidControl(0, x_pitch, dt_us, 10., 4., 0., true, x_pitch_saturation);
    float output_x_pitch_rate = pidControl(&x_pitch_pid, 0, x_pitch, dt_us, 5., 0.5, 0., true, x_pitch_saturation);
    float output_x_pitch = pidControl(&x_pitch_rate_pid, output_x_pitch_rate, 0, dt_us, 0.1, 0., 0., false, false);

    float fin1_ang = (output_z_yaw + 0. + output_x_pitch);
    float fin2_ang = (output_z_yaw + output_y_roll + 0.);
    float fin3_ang = (output_z_yaw + 0. - output_x_pitch);
    float fin4_ang = (output_z_yaw - output_y_roll + 0.);

    if (Serial){
    Serial.print(fin1_ang);           Serial.print(",");
    Serial.print(fin2_ang);          Serial.print(",");
    Serial.print(fin3_ang);          Serial.print(",");
    Serial.println(fin4_ang);
    }

    // Scaling factors to account for variable speed
    fin1_ang = fin1_ang * actuation_factor;
    fin2_ang = fin2_ang * actuation_factor;
    fin3_ang = fin3_ang * actuation_factor;
    fin4_ang = fin4_ang * actuation_factor;

    // raw actuator commands
    servo1_ang = (int)(- fin1_ang / servo_ang_to_fin_ang);
    servo2_ang = (int)(- fin2_ang / servo_ang_to_fin_ang);
    servo3_ang = (int)(- fin3_ang / servo_ang_to_fin_ang);
    servo4_ang = (int)(- fin4_ang / servo_ang_to_fin_ang);

    // real actuator commands
    int servo1_ang_out = outputSaturation(servo1_ang, servo_angle_max);
    int servo2_ang_out = outputSaturation(servo2_ang, servo_angle_max);
    int servo3_ang_out = outputSaturation(servo3_ang, servo_angle_max);
    int servo4_ang_out = outputSaturation(servo4_ang, servo_angle_max);

    // dfine desired angles for the 4 servos
    int servo_angles_out[4] = {servo1_ang_out, servo2_ang_out, servo3_ang_out, servo4_ang_out}; 
    setIdealAngles(servo_angles_out);

  }

