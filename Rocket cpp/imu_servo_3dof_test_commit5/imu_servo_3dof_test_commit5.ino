#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MS5611.h>


#define BNO08X_RESET -1
MS5611 MS5611(0x77);

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

// Samples for altitude initialization
const int n_samples = 100;
// Kalman parameters
float R = 0.01; // Measurement noise covariance
float acc_std = 0.1; // Acceleration noise standard deviation

// This might need to be relocated in the final FSM code
float alt_init = 0; // Define initial altitude variable

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

float actuationFactor(float relative_height, float vertical_speed) {
  const int min_control_speed = 10; // Minimum vertical speed for control
  const float height_rail = 8; // Height of the launch rail in meters

  if (relative_height < height_rail) { // 7m is the height of the launch rail
      return 0.; // fins should not move when the rocket is on the launch rail
  }
  else if (vertical_speed > min_control_speed) {
      return pow((30 / vertical_speed), 2); // PID tuned at 30 m/s
  }
  else {
      return 9.; // max k_act is 3^2, since min speed is 10 m/s
  }
}

// float actuationFactor(float vertical_speed) {
//     if (vertical_speed > 10) {
//         return pow((30 / vertical_speed), 2); // PID tuned at 30 m/s
//     }
//     else {
//         return 9; // max k_act is 3^2
//     }
// }

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

void initial_altitude(float* alt_init){
  static int counter = 1; // Counter must not be 0 due to division
  bool run_initial_altitude = true;

  while (run_initial_altitude == true) {
    // Get pressure altitude
    int result = MS5611.read();
    float pressure = MS5611.getPressure();
    float raw_altitude = pressure_altitude(pressure);
        
    if (counter <= n_samples) {
      *alt_init = *alt_init * (counter - 1) / counter + raw_altitude / counter;
      counter = counter + 1;
    }
    else if (counter > n_samples) {
        counter = 1; // Reset counter and done
        run_initial_altitude = false;
    }
  }
}

// Kalman filter
//-------------------------------------------------------------------------
void kalman_filter(float baro_alt, unsigned long time_new, float s_state_out[3]) {
  // Tuning parameters

  static bool initialized = false;
  static float s_state_prev[3] = {0};       // [altitude, vertical velocity, acceleration]
  static float P_prev[3][3] = {{0}};        // Covariance matrix
  static unsigned long time_old = 0;

  // ===== Initialization =====
  if (!initialized) {
      s_state_prev[0] = baro_alt;           // Initialize altitude with first measurement
      s_state_prev[1] = 0;                  // Initial vertical velocity (unknown, assume 0)
      s_state_prev[2] = 0;                  // Initial acceleration (unknown, assume 0)
      
      // Initial covariance - high uncertainty in velocity and bias
      P_prev[0][0] = 1.0f;                 // Altitude variance
      P_prev[1][1] = 1.0f;                // Velocity variance
      P_prev[2][2] = 1.0f;                 // Avveration variance
      
      time_old = time_new;
      initialized = true;
      return;  // Skip first update to initialize properly
  }

  // ===== Time Step Handling =====
  if (time_new <= time_old) {
      return;  // Skip invalid time steps
  }
  
  float dt = (time_new - time_old) / 1e6f;  // Convert microseconds to seconds
  time_old = time_new;
  
  // Limit maximum dt for stability
  if (dt > 0.1f) dt = 0.1f;

  // ===== Prediction Step =====
  // State transition matrix
  float F[3][3] = {
      {1, dt, 0.5f * dt * dt},  // Position update
      {0, 1, dt},               // Velocity update
      {0, 0, 1}                  // Acceleration is assumed constant
  };

  // Control input matrix (how acceleration affects state)
  float G[3] = {0.5f * dt * dt, dt, 1};

  // Process noise covariance
  float Q[3][3] = {
      {G[0]*G[0]*acc_std*acc_std, G[0]*G[1]*acc_std*acc_std, G[0]*G[2]*acc_std*acc_std},
      {G[1]*G[0]*acc_std*acc_std, G[1]*G[1]*acc_std*acc_std, G[1]*G[2]*acc_std*acc_std},
      {G[2]*G[0]*acc_std*acc_std, G[2]*G[1]*acc_std*acc_std, G[2]*G[2]*acc_std*acc_std}
  };

  // Predict state: x_pred = F*x_prev + G*u
  float s_state_pred[3] = {
      F[0][0]*s_state_prev[0] + F[0][1]*s_state_prev[1] + F[0][2]*s_state_prev[2],
      F[1][0]*s_state_prev[0] + F[1][1]*s_state_prev[1] + F[1][2]*s_state_prev[2],
      F[2][0]*s_state_prev[0] + F[2][1]*s_state_prev[1] + F[2][2]*s_state_prev[2]
  };

  // Predict covariance: P_pred = F*P_prev*F' + Q
  float F_T[3][3], temp[3][3], P_pred[3][3];
  
  // Compute F transpose
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          F_T[i][j] = F[j][i];
      }
  }
  
  // temp = F*P_prev
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          temp[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              temp[i][j] += F[i][k] * P_prev[k][j];
          }
      }
  }
  
  // P_pred = temp*F_T + Q
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          P_pred[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              P_pred[i][j] += temp[i][k] * F_T[k][j];
          }
          P_pred[i][j] += Q[i][j];
      }
  }

  // ===== Update Step =====
  // Measurement matrix (we only measure altitude directly)
  float H[3] = {1, 0, 0};

  // Innovation covariance: S = H*P_pred*H' + R
  float S = 0;
  for (int i = 0; i < 3; i++) {
      S += H[i] * P_pred[0][i];  // Since H is [1,0,0], this simplifies to P_pred[0][0]
  }
  S += R;

  // Kalman gain: K = P_pred*H'/S
  float K[3];
  for (int i = 0; i < 3; i++) {
      K[i] = 0;
      for (int j = 0; j < 3; j++) {
          K[i] += P_pred[i][j] * H[j];
      }
      K[i] /= S;
  }

  // Innovation: y = z - H*x_pred
  float innov = baro_alt - s_state_pred[0];

  // State update: x = x_pred + K*innov
  float s_state[3] = {
      s_state_pred[0] + K[0] * innov,
      s_state_pred[1] + K[1] * innov,
      s_state_pred[2] + K[2] * innov
  };

  // Covariance update: P = (I - K*H)*P_pred
  float KH[3][3], I[3][3] = {{1,0,0},{0,1,0},{0,0,1}}, P[3][3];
  
  // Compute KH = K*H
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          KH[i][j] = K[i] * H[j];
      }
  }
  
  // Compute P = (I - KH)*P_pred
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          P[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              P[i][j] += (I[i][k] - KH[i][k]) * P_pred[k][j];
          }
      }
  }

  // ===== Post-Update Checks =====
  // Ensure covariance matrix remains positive definite
  for (int i = 0; i < 3; i++) {
      if (P[i][i] < 0) P[i][i] = 0;
  }
  
  // Symmetrize covariance matrix
  for (int i = 0; i < 3; i++) {
      for (int j = i+1; j < 3; j++) {
          P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
      }
  }

  // ===== Update Persistent Variables =====
  for (int i = 0; i < 3; i++) {
      s_state_prev[i] = s_state[i];
      s_state_out[i] = s_state[i];
      for (int j = 0; j < 3; j++) {
          P_prev[i][j] = P[i][j];
      }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED
  pinMode(3, OUTPUT); // Buzzer connected to GP3 (5)

  Serial.begin(115200);
  Wire.begin();

  // Try to initialize IMU
  while (!bno08x.begin_I2C(0x4B)) {
    digitalWrite(3, HIGH);
    delay(100); 
    digitalWrite(3, LOW);
    delay(100);}

  setReports(reportIntervalUs);

  delay(100);

  // Servo control
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz

  delay(100);

  // Try to initialize Barometer
  while (!MS5611.begin()){
    digitalWrite(3, HIGH);
    delay(100); 
    digitalWrite(3, LOW);
    delay(100); }

  MS5611.reset(1);
  MS5611.setOversampling(OSR_ULTRA_HIGH);
  initial_altitude(&alt_init); // Initialize altitude

  // Finish setup; beep
  for (int i=0; i<2; i++) {
    digitalWrite(3, HIGH);
    delay(300);
    digitalWrite(3,LOW);
    delay(300);}
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

    // in rocket frame, with z up; rotation is ZYX
    float x_pitch = ypr.roll; // x is still x, just a differrent name
    float y_roll = ypr.pitch; // y is still y, just a differrent name
    float z_yaw = ypr.yaw;

    float x_pitch_dot = ypr_dot.roll;
    float y_roll_dot = ypr_dot.pitch;
    float z_yaw_dot = ypr_dot.yaw;

    acc_ZYX_body_to_inertial(x_pitch / RAD_TO_DEG, y_roll / RAD_TO_DEG, z_yaw / RAD_TO_DEG, &body_acc, &enu_acc); // ANGLE INPUTS IN RAD!!!
    float acc_enu_x = enu_acc.x;
    float acc_enu_y = enu_acc.y;
    float acc_enu_z = enu_acc.z;

    // Get pressure altitude
    int result = MS5611.read();
    float pressure = MS5611.getPressure();
    float raw_altitude = pressure_altitude(pressure);
    float adjusted_altitude = raw_altitude - alt_init;

    // Clock tick tick
    static long t_last = 0; // t_pre
    long t_now = micros();
    float dt_us = t_now - t_last;
    t_last = t_now;

    static float s_state_out[3] = {0, 0, 0}; // [altitude, vertical speed, accelerometer bias]
    kalman_filter(adjusted_altitude, t_now, s_state_out); // t_now in microseconds or 1e-6 seconds
    float estimated_altitude = s_state_out[0];
    float estimated_vertical_speed = s_state_out[1];
    float estimated_acceleration = s_state_out[2]; // no IMU used

    // if (Serial){
    // // Serial.print(t_now);           Serial.print(",");
    // // Serial.print(x_pitch);          Serial.print(",");
    // // Serial.print(y_roll);          Serial.print(",");
    // // Serial.print(z_yaw);          Serial.print(",");
    // // Serial.print(enu_acc.x);          Serial.print(",");
    // // Serial.print(enu_acc.y);          Serial.print(",");
    // Serial.print(enu_acc.z);          Serial.print(",");
    // Serial.print(estimated_acceleration); Serial.print(",");
    // Serial.print(estimated_vertical_speed);          Serial.print(",");
    // Serial.println(estimated_altitude);
    // }

    // Actuation factor
    float actuation_factor = actuationFactor(estimated_altitude ,estimated_vertical_speed);

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
    float output_z_yaw = 0.;
    float output_y_roll_rate = 0.;
    float output_y_roll = 0.;
    float output_x_pitch_rate = 0.;
    float output_x_pitch = 0.;

    if (abs(actuation_factor) < 1e-3) { // When the rocket is on the launch rail
        // All outputs already initialized to 0
    }
    else {
        //pidControl(pid_error_t* error_save, float target, float measure, float dt_us, float kp, float ki, float kd, bool anti_wind, bool saturation)
        output_z_yaw = pidControl(&z_yaw_pid, 0, z_yaw_dot, dt_us, 0.05, 0., 0., false, false);

        output_y_roll_rate = pidControl(&y_roll_pid, 0, y_roll, dt_us, 5., 0.5, 0., true, y_roll_saturation);
        output_y_roll = pidControl(&y_roll_rate_pid, output_y_roll_rate, y_roll_dot, dt_us, 0.1, 0., 0., false, false);

        output_x_pitch_rate = pidControl(&x_pitch_pid, 0, x_pitch, dt_us, 5., 0.5, 0., true, x_pitch_saturation);
        output_x_pitch = pidControl(&x_pitch_rate_pid, output_x_pitch_rate, x_pitch_dot, dt_us, 0.1, 0., 0., false, false);
    }

    float fin1_ang = (output_z_yaw + 0. + output_x_pitch);
    float fin2_ang = (output_z_yaw + output_y_roll + 0.);
    float fin3_ang = (output_z_yaw + 0. - output_x_pitch);
    float fin4_ang = (output_z_yaw - output_y_roll + 0.);

    if (Serial){
    Serial.print(fin1_ang);           Serial.print(",");
    Serial.print(fin2_ang);          Serial.print(",");
    Serial.print(fin3_ang);          Serial.print(",");
    Serial.print(fin4_ang);          Serial.print(",");
    Serial.println(actuation_factor);
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

