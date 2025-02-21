#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define BNO08X_RESET -1

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
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100); 
    digitalWrite(LED_BUILTIN, LOW);
    delay(100); 
  }

  setReports(reportIntervalUs);

  delay(100);

  // Try to initialize Barometer
  unsigned bmp_state = bmp.begin(0x76);

  while (!bmp_state) 
    {Serial.println(F("Could not find any BMP280 sensor!"));
    delay(10);}

  /* Default settings from Bosch 280 datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  bmp_temp->printSensorDetails();

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

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

// Kalman filter stuff
//-------------------------------------------------------------------------
void mat21_multiply_mat12(float A[2][1], float B[1][2], float C[2][2]) {
    C[0][0] = A[0][0] * B[0][0];
    C[0][1] = A[0][0] * B[0][1];
    C[1][0] = A[1][0] * B[0][0];
    C[1][1] = A[1][0] * B[0][1];
}

void mat_elemwise_multiply22D(float A[2][2], float B, float C[2][2]) {
    C[0][0] = A[0][0] * B;
    C[0][1] = A[0][1] * B;
    C[1][0] = A[1][0] * B;
    C[1][1] = A[1][1] * B;
}

void mat_elemwise_multiply21D(float A[2][1], float B, float C[2][1]) {
    C[0][0] = A[0][0] * B;
    C[1][0] = A[1][0] * B;
}

void mat_vec_multiply2D(float A[2][2], float B[2][1], float C[2][1]) {
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
}

void vec_add2D(float A[2][1], float B[2][1], float C[2][1]) {
    C[0][0] = A[0][0] + B[0][0];
    C[1][0] = A[1][0] + B[1][0];
}

void mat_add22D(float A[2][2], float B[2][2], float C[2][2]) {
    C[0][0] = A[0][0] + B[0][0];
    C[1][0] = A[1][0] + B[1][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][1] = A[1][1] + B[1][1];
}

void mat_multiply22D(float A[2][2], float B[2][2], float C[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void compute_FPFt(float F[2][2], float P[2][2], float P_1[2][2]) {
    float FP[2][2];   // Intermediate matrix FP
    float Ft[2][2];   // Transpose of F

    mat_multiply22D(F, P, FP);

    Ft[0][0] = F[0][0]; Ft[0][1] = F[1][0];
    Ft[1][0] = F[0][1]; Ft[1][1] = F[1][1];

    mat_multiply22D(FP, Ft, P_1);
}

void kalman_filter_fusion(float dt_us, float a_imu, float z_baro, float acc_std, float R, float s_out[2][1]) {
    // Convert time to seconds
    float dt = dt_us * 1e-6;

    // State vector and covariance matrix (persistent between calls)
    static float s[2][1] = {{0}, {0}};
    static float P[2][2] = {{0, 0}, {0, 0}};

    // State transition matrix
    float F[2][2] = {{1, dt}, {0, 1}};

    // Control input matrix
    float G[2][1] = {{0.5 * pow(dt, 2)}, {dt}};

    // Measurement matrix
    float H[1][2] = {1, 0};

    // Process noise covariance matrix
    float Q[2][2];
    float G_T[1][2] = {G[0][0], G[1][0]};
    mat21_multiply_mat12(G, G_T, Q); // Q = G * G^T
    float acc_std2 = pow(acc_std, 2);
    mat_elemwise_multiply22D(Q, acc_std2, Q); // Q = Q * acc_std^2

    // Prediction step
    float s_1[2][1], s_2[2][1];
    mat_vec_multiply2D(F, s, s_1); // s_1 = F * s
    mat_elemwise_multiply21D(G, a_imu, s_2); // s_2 = G * a_imu
    vec_add2D(s_1, s_2, s); // s = s_1 + s_2

    float P_1[2][2];
    compute_FPFt(F, P, P_1); // P_1 = F * P * F^T
    mat_add22D(P_1, Q, P); // P = P_1 + Q

    // Update step
    float L = H[0][0] * P[0][0] * H[0][0] + R; // L = H * P * H^T + R

    float K[2][1];
    float H_T[2][1] = {H[0][0], H[1][0]};
    mat_vec_multiply2D(P, H_T, K); // K = P * H^T
    K[0][0] /= L; // K = P * H^T / L
    K[1][0] /= L;

    float Hs = H[0][0] * s[0][0]; // Hs = H * s
    float Innov = z_baro - Hs; // Innov = z_baro - H * s

    float delta_s[2][1];
    mat_elemwise_multiply21D(K, Innov, delta_s); // delta_s = K * Innov
    vec_add2D(s, delta_s, s); // s = s + delta_s

    // Update covariance matrix
    float KH[2][2] = {{K[0][0] * H[0][0], K[0][0] * H[0][1]},
                     {K[1][0] * H[0][0], K[1][0] * H[0][1]}};
    float I_minus_KH[2][2] = {{1 - KH[0][0], -KH[0][1]},
                              {-KH[1][0], 1 - KH[1][1]}};
    float P_new[2][2];
    mat_multiply22D(I_minus_KH, P, P); // P = (I - K * H) * P

    // Copy output
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 1; j++) {
            s_out[i][j] = s[i][j];
        }
    }
}
// End of Kalman filter stuff
//-------------------------------------------------------------------------

// For bias removal of altitude, begins at 0m
float initial_altitude = 0;
bool altitude_initialized = false;
const int INIT_SAMPLES = 1000; // Number of samples to average for initial altitude
int sample_count = 0;
float altitude_sum = 0;



// Main loop
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
    static float x_pitch_pre = 0;
    static float y_roll_pre = 0;
    static float z_yaw_pre = 0;

    long t_now = micros();
    float dt_us = t_now - t_last;
    t_last = t_now;

    // In rocket frame, with z up; rotation is ZYX
    float x_pitch = ypr.roll; // x is still x, just a differrent name
    float y_roll = ypr.pitch; // y is still y, just a differrent name
    float z_yaw = ypr.yaw;
    float z_yaw_dot = ypr_dot.yaw;

    acc_ZYX_body_to_inertial(x_pitch / RAD_TO_DEG, y_roll / RAD_TO_DEG, z_yaw / RAD_TO_DEG, &body_acc, &enu_acc); // ANGLE INPUTS IN RAD!!!
    float acc_enu_x = enu_acc.x;
    float acc_enu_y = enu_acc.y;
    float acc_enu_z = enu_acc.z;

    // Get pressure altitude
    sensors_event_t temp_event, pressure_event;
    bmp_pressure->getEvent(&pressure_event);

    float pressure = pressure_event.pressure;
    float raw_altitude = pressure_altitude(pressure);
    static float s_kalman[2][1] = {{0}, {0}}; // Height, vertical speed initialization
 
    // Initialize altitude bias or apply correction
    if (!altitude_initialized) {
      // Accumulate samples for averaging
      altitude_sum += raw_altitude;
      sample_count++;

      // If we've collected enough samples, calculate the average and set it as the initial altitude
      if (sample_count >= INIT_SAMPLES) {
        initial_altitude = altitude_sum / INIT_SAMPLES;
        altitude_initialized = true;
        
        // Beep to indicate calibration is complete
        digitalWrite(3, HIGH);
        delay(1000);
        digitalWrite(3, LOW);
        
        Serial.print("Initial altitude reference: ");
        Serial.print(initial_altitude);
        Serial.println(" meters");
      }
    } 
    else { // Already intialized, execute the rest
      // Remove the bias by subtracting the initial altitude
      float altitude = raw_altitude - initial_altitude;
      
      // Now feed the bias-corrected altitude to the Kalman filter
      float acc_std = 0.01;
      float R = 0.01; // Measurement noise covariance
      kalman_filter_fusion(dt_us, acc_enu_z, altitude, acc_std, R, s_kalman);

      float alt_kalm = s_kalman[0][0];
      float vert_spd = s_kalman[1][0];

      if (Serial) {
        // Serial.print(t_now);           Serial.print(",");
        // Serial.print(x_pitch);          Serial.print(",");
        // Serial.print(y_roll);          Serial.print(",");
        // Serial.print(z_yaw);          Serial.print(",");

        // Serial.print(enu_acc.x);          Serial.print(",");
        // Serial.print(enu_acc.y);          Serial.print(",");
        Serial.print(enu_acc.z);          Serial.print(",");

        Serial.print(pressure_event.pressure * 100);     Serial.print(","); // Pressure in Pa
        Serial.print(altitude);      Serial.print(",");
        Serial.println(vert_spd);
      }
    }
}