#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define BNO08X_RESET -1

const int n_samples = 500;
float R = 0.01; // Measurement noise covariance
float acc_std = 0.08; // Acceleration noise standard deviation

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

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// Top frequency is about 250Hz but this report is more accurate
long reportIntervalUs = 4000;

// Enable IMU reports
void setReports(long report_interval) {
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
}

// Setup function
//----------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED
  pinMode(3, OUTPUT); // Buzzer connected to GP3 (5)

  Serial.begin(115200);

  // Try to initialize IMU
  while (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to initialize BNO08x!");
    delay(10); 
  }

  setReports(reportIntervalUs);

  delay(100);

  // Try to initialize Barometer
  unsigned bmp_state = bmp.begin(0x76);

  // Try to initialize BMP280
  while (!bmp_state) {
    Serial.println(F("Could not find any BMP280 sensor!"));
    delay(100);
  }

  /* Default settings from Bosch 280 datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  bmp_temp->printSensorDetails();

  // Finish initialization; beep
  // for (int i = 0; i < 3; i++) {
  //   digitalWrite(3, HIGH);
  //   delay(300);
  //   digitalWrite(3, LOW);
  //   delay(300);
  // }
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

void acc_ZYX_body_to_inertial(float x_ang, float y_ang, float z_ang, cartesian_t* body_acc, cartesian_t* enu_acc) {
    // Rotation matrix for ZYX Euler angles
    float R_mat[3][3] = {
        {cos(y_ang) * cos(z_ang), -cos(x_ang) * sin(z_ang) + sin(x_ang) * sin(y_ang) * cos(z_ang), sin(x_ang) * sin(z_ang) + cos(x_ang) * sin(y_ang) * cos(z_ang)},
        {cos(y_ang) * sin(z_ang), cos(x_ang) * cos(z_ang) + sin(x_ang) * sin(y_ang) * sin(z_ang), -sin(x_ang) * cos(z_ang) + cos(x_ang) * sin(y_ang) * sin(z_ang)},
        {-sin(y_ang), sin(x_ang) * cos(y_ang), cos(x_ang) * cos(y_ang)}
    };

    float acc_body[3] = {body_acc->x, body_acc->y, body_acc->z};

    float acc_enu[3];
    for (int i = 0; i < 3; i++) {
        acc_enu[i] = 0;
        for (int j = 0; j < 3; j++) {
            acc_enu[i] += R_mat[i][j] * acc_body[j];
        }
    }

    enu_acc->x = acc_enu[0];
    enu_acc->y = acc_enu[1];
    enu_acc->z = acc_enu[2];
}

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

// Kalman filter with bias estimation
//-------------------------------------------------------------------------
void kalman_filter_fusion(float baro_alt, float acc_z, unsigned long time_new, float s_state_out[3]) {
    static float s_state_prev[3] = {0, 0, 0}; // [altitude, vertical speed, accelerometer bias]
    static float P_prev[3][3] = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.1}}; // Initial covariance matrix
    static unsigned long time_old = 0; // Previous time in microseconds

    // Time step (convert to seconds)
    float dt = (time_new - time_old) / 1e6;
    time_old = time_new;

    // Ensure dt is within a reasonable range
    if (dt < 1e-6) dt = 1e-6;
    if (dt > 1.0) dt = 1.0;

    // State transition matrix
    float F[3][3] = {
        {1, dt, 0.5 * dt * dt},
        {0, 1, dt},
        {0, 0, 1}
    };

    // Control input matrix
    float G[3] = {0.5 * dt * dt, dt, 0};

    // Measurement matrix
    float H[3] = {1, 0, 0};

    // Process noise covariance matrix
    float Q[3][3] = {
        {G[0] * G[0] * acc_std * acc_std, G[0] * G[1] * acc_std * acc_std, G[0] * G[2] * acc_std * acc_std},
        {G[1] * G[0] * acc_std * acc_std, G[1] * G[1] * acc_std * acc_std, G[1] * G[2] * acc_std * acc_std},
        {G[2] * G[0] * acc_std * acc_std, G[2] * G[1] * acc_std * acc_std, G[2] * G[2] * acc_std * acc_std}
    };

    // Prediction step
    float s_state_pred[3] = {
        F[0][0] * s_state_prev[0] + F[0][1] * s_state_prev[1] + F[0][2] * s_state_prev[2] + G[0] * acc_z,
        F[1][0] * s_state_prev[0] + F[1][1] * s_state_prev[1] + F[1][2] * s_state_prev[2] + G[1] * acc_z,
        F[2][0] * s_state_prev[0] + F[2][1] * s_state_prev[1] + F[2][2] * s_state_prev[2] + G[2] * acc_z
    };

    float P_pred[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                P_pred[i][j] += F[i][k] * P_prev[k][j];
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] += Q[i][j];
        }
    }

    // Normalize P_pred to prevent overflow
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (P_pred[i][j] > 1e10) {
                P_pred[i][j] = 1e10;
            }
        }
    }

    // Calculate innovation covariance (L)
    float L = 0;
    for (int i = 0; i < 3; i++) {
        L += H[i] * P_pred[i][0] * H[0];
    }
    L += R;

    // Kalman gain
    float K[3] = {
        P_pred[0][0] * H[0] / L,
        P_pred[1][0] * H[0] / L,
        P_pred[2][0] * H[0] / L
    };

    // Innovation
    float innov = baro_alt - (H[0] * s_state_pred[0] + H[1] * s_state_pred[1] + H[2] * s_state_pred[2]);

    // State update
    float s_state[3] = {
        s_state_pred[0] + K[0] * innov,
        s_state_pred[1] + K[1] * innov,
        s_state_pred[2] + K[2] * innov
    };

    // Covariance update
    float P[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = P_pred[i][j] - K[i] * H[j] * P_pred[i][j];
        }
    }

    // Update persistent variables for the next iteration
    for (int i = 0; i < 3; i++) {
        s_state_prev[i] = s_state[i];
        s_state_out[i] = s_state[i];
        for (int j = 0; j < 3; j++) {
            P_prev[i][j] = P[i][j];
        }
    }

    // Debug output
    // Serial.print("K: "); Serial.println(K[0]);
    // Serial.print("State: "); Serial.print(s_state_prev[0]); Serial.print(", ");
    // Serial.print(s_state_prev[1]); Serial.print(", "); Serial.println(s_state_prev[2]);
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
      case SH2_LINEAR_ACCELERATION:
        body_acc.x = sensorValue.un.linearAcceleration.x;
        body_acc.y = sensorValue.un.linearAcceleration.y;
        body_acc.z = sensorValue.un.linearAcceleration.z;
        break;
    }
  }

  static long t_last = 0; // t_pre
  long t_now = micros();
  float dt_us = t_now - t_last;
  t_last = t_now;

  // In rocket frame, with z up; rotation is ZYX
  float x_pitch = ypr.roll; // x is still x, just a different name
  float y_roll = ypr.pitch; // y is still y, just a different name
  float z_yaw = ypr.yaw;

  acc_ZYX_body_to_inertial(x_pitch / RAD_TO_DEG, y_roll / RAD_TO_DEG, z_yaw / RAD_TO_DEG, &body_acc, &enu_acc); // ANGLE INPUTS IN RAD!!!
  float acc_enu_z = enu_acc.z;

  // Get pressure altitude
  sensors_event_t temp_event, pressure_event;
  bmp_pressure->getEvent(&pressure_event);

  float pressure = pressure_event.pressure;
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
    // Run the Kalman filter with bias estimation
    kalman_filter_fusion(adjusted_altitude, acc_enu_z, t_now, s_state_out); // t_now in microseconds or 1e-6 seconds
    // Output the estimated altitude, vertical speed, and bias
    Serial.print(s_state_out[0]); // Estimated altitude in meters
    Serial.print(',');
    Serial.print(s_state_out[1]); // Estimated vertical speed in m/s
    Serial.print(',');
    Serial.println(s_state_out[2]); //Estimated accelerometer bias in m/s^2
  }

  // Add a delay or other logic as needed
  delay(10); // Delay is for the weak
}