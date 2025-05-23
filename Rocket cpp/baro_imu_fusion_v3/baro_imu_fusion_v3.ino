#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <MS5611.h>

#define BNO08X_RESET -1
MS5611 MS5611(0x77);

const int n_samples = 100;
float R = 0.01; // Measurement noise covariance
float acc_std = 0.5; // Acceleration noise standard deviation

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

struct euler_t ypr;
struct euler_t ypr_dot;

struct cartesian_t body_acc;
struct cartesian_t enu_acc;
struct cartesian_t enu_vel;
struct cartesian_t enu_pos;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Top frequency is about 250Hz but this report is more accurate
long reportIntervalUs = 4000;

// Enable IMU reports
void setReports(long report_interval) {
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
  bno08x.enableReport(SH2_ACCELEROMETER, reportIntervalUs);
}

// Setup function
//----------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED
  pinMode(3, OUTPUT); // Buzzer connected to GP3 (5)

  Serial.begin(115200);
  Wire.begin();

  // Try to initialize IMU
  while (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to initialize BNO08x!");
    delay(10); 
  }

  setReports(reportIntervalUs);

  delay(100);

  // Try to initialize Barometer
  while (!MS5611.begin())
    {
    Serial.println("MS5611 sensor not found!");
    delay(10);}

  MS5611.reset(1);
  MS5611.setOversampling(OSR_ULTRA_HIGH);

  // Finish initialization; beep
  for (int i = 0; i < 3; i++) {
    digitalWrite(3, HIGH);
    delay(300);
    digitalWrite(3, LOW);
    delay(300);
  }
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

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

// Kalman filter with bias estimation
//-------------------------------------------------------------------------
void kalman_filter_fusion(float baro_alt, float acc_z, unsigned long time_new, float s_state_out[3]) {
  // Tuning parameters - these need to be set appropriately!
  const float acc_std = 0.1f; // Process noise standard deviation
  const float R = 1.0f;      // Measurement noise variance

  static float s_state_prev[3] = {0, 0, 0}; // [altitude, vertical speed, accelerometer bias]
  static float P_prev[3][3] = {{0.5, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
  static unsigned long time_old = 0;

  // Time step (convert to seconds)
  float dt = (time_new - time_old) / 1e6f;
  time_old = time_new;

  // State transition matrix
  float F[3][3] = {
      {1, dt, 0.5f * dt * dt},
      {0, 1, dt},
      {0, 0, 1}
  };

  // Control input matrix
  float G[3] = {0.5f * dt * dt, dt, 0};

  // Measurement matrix (we only measure altitude)
  float H[3] = {1, 0, 0};

  // Process noise covariance matrix
  float Q[3][3] = {
      {G[0] * G[0] * acc_std * acc_std, G[0] * G[1] * acc_std * acc_std, G[0] * G[2] * acc_std * acc_std},
      {G[1] * G[0] * acc_std * acc_std, G[1] * G[1] * acc_std * acc_std, G[1] * G[2] * acc_std * acc_std},
      {G[2] * G[0] * acc_std * acc_std, G[2] * G[1] * acc_std * acc_std, G[2] * G[2] * acc_std * acc_std}
  };

  // ===== Prediction Step =====
  // Predict state
  float s_state_pred[3] = {
      F[0][0] * s_state_prev[0] + F[0][1] * s_state_prev[1] + F[0][2] * s_state_prev[2] + G[0] * acc_z,
      F[1][0] * s_state_prev[0] + F[1][1] * s_state_prev[1] + F[1][2] * s_state_prev[2] + G[1] * acc_z,
      F[2][0] * s_state_prev[0] + F[2][1] * s_state_prev[1] + F[2][2] * s_state_prev[2] + G[2] * acc_z
  };

  // Predict covariance: F * P * F' + Q
  float F_T[3][3]; // Transpose of F
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          F_T[i][j] = F[j][i];
      }
  }

  float temp[3][3];
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          temp[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              temp[i][j] += F[i][k] * P_prev[k][j];
          }
      }
  }

  float P_pred[3][3];
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
  // Innovation covariance
  float S = 0;
  for (int i = 0; i < 3; i++) {
      S += H[i] * P_pred[0][i]; // H is [1,0,0], so this simplifies to P_pred[0][0]
  }
  S += R;

  // Kalman gain
  float K[3];
  for (int i = 0; i < 3; i++) {
      K[i] = 0;
      for (int j = 0; j < 3; j++) {
          K[i] += P_pred[i][j] * H[j];
      }
      K[i] /= S;
  }

  // Innovation
  float innov = baro_alt - s_state_pred[0]; // Since H is [1,0,0]

  // State update
  float s_state[3] = {
      s_state_pred[0] + K[0] * innov,
      s_state_pred[1] + K[1] * innov,
      s_state_pred[2] + K[2] * innov
  };

  // Covariance update: P = (I - K*H) * P_pred
  float KH[3][3];
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          KH[i][j] = K[i] * H[j];
      }
  }

  float I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  float P[3][3];
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          P[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              P[i][j] += (I[i][k] - KH[i][k]) * P_pred[k][j];
          }
      }
  }

  // Update persistent variables
  for (int i = 0; i < 3; i++) {
      s_state_prev[i] = s_state[i];
      s_state_out[i] = s_state[i];
      for (int j = 0; j < 3; j++) {
          P_prev[i][j] = P[i][j];
      }
  }
}

// Main loop
void loop() {
  if (bno08x.wasReset()) {
    setReports(reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true); // output in degrees
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        ypr_dot.roll = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        ypr_dot.pitch = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        ypr_dot.yaw = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        break;
      case SH2_ACCELEROMETER:
        body_acc.x = sensorValue.un.accelerometer.x;
        body_acc.y = sensorValue.un.accelerometer.y;
        body_acc.z = sensorValue.un.accelerometer.z;
        break;
    }
  }

  // In rocket frame, with z up; rotation is ZYX
  float x_pitch = ypr.roll; // x is still x, just a different name
  float y_roll = ypr.pitch; // y is still y, just a different name
  float z_yaw = ypr.yaw;

  acc_ZYX_body_to_inertial(x_pitch / RAD_TO_DEG, y_roll / RAD_TO_DEG, z_yaw / RAD_TO_DEG, &body_acc, &enu_acc); // ANGLE INPUTS IN RAD!!!
  float acc_enu_z = enu_acc.z;

  // Get pressure altitude
  int result = MS5611.read();
  float pressure = MS5611.getPressure();
  float raw_altitude = pressure_altitude(pressure);

  static int counter = 1;
  static float alt_init = 0;
  
  if (counter < n_samples) {
    alt_init = alt_init * (counter - 1) / counter + raw_altitude / counter;
    counter = counter + 1;  
  }
  else {
    if (counter == n_samples) { // Beep after getting the initial altitude
      counter = counter + 1; // Prevent from beeping again
      digitalWrite(3, HIGH);
      delay(1000);
      digitalWrite(3, LOW);
      delay(100);
    }
    float adjusted_altitude = raw_altitude - alt_init;

    static float s_state_out[3] = {0, 0, 0}; // [altitude, vertical speed, accelerometer bias]

    static long t_last = 0; // t_pre
    long t_now = micros();
    float dt_us = t_now - t_last;
    t_last = t_now;

    // Run the Kalman filter with bias estimation
    kalman_filter_fusion(adjusted_altitude, acc_enu_z, t_now, s_state_out); // t_now in microseconds or 1e-6 seconds
    // Output the estimated altitude, vertical speed, and bias
    Serial.print(s_state_out[0]); // Estimated altitude in meters
    Serial.print(',');
    Serial.print(s_state_out[1]); // Estimated vertical speed in m/s
    Serial.print(',');
    Serial.print(s_state_out[2]); //Estimated accelerometer bias in m/s^2
    Serial.print(',');
    Serial.print(adjusted_altitude);
    Serial.print(',');
    Serial.println(acc_enu_z); // IMU acc z
  }

  // Add a delay or other logic as needed
  delay(10); // Delay is for the weak
}