#include <WiFi.h>
#include <WebServer.h>
#include <cstring>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_PWMServoDriver.h>
#include <MS5611.h>



// Define important global parameters
//----------------------------------------------------
// Servo parameters
const int servo1_iang = 75; // servo angle when 0 degree fin angle
const int servo2_iang = 90; // servo angle when 0 degree fin angle
const int servo3_iang = 75; // servo angle when 0 degree fin angle
const int servo4_iang = 90; // servo angle when 0 degree fin angle
// Kalman parameters
const int n_samples = 100; // Number of samples for altitude initialization
float R = 0.01; // Measurement noise covariance
float acc_std = 0.1; // Acceleration noise standard deviation
// Control parameters
const int min_control_speed = 10; // Minimum vertical speed for control
const float height_rail = 7.5; // Height of the launch rail in meters
// Detection parameters
//float alpha_baro = 0.15; // Smoothing factor for low pass filter
float acc_z_threshold = 3; // Threshold for launch detection
const int launch_n_samples = 10; // Number of samples for launch detection
const int end_angle_n_samples = 2; // Number of samples for angles used to detect end of flight
const float end_angle_threshold = 45; // Threshold in deg for recovery condition
const float end_height_from_apogee = 5; // Distance in meters from the recorded height to detect descent
const int end_height_n_samples = 10; // Number of samples for height used to detect end of flight
// SD file parameters
const int chipSelect = 17; // for SD card reader
const char filename[] = "FLIGHTLOG.csv";


// Define global state machine variables
//----------------------------------------------------
int state = 0;                   // state: 0=Idle, 1=Initial, 2=Armed, 3=Launch, 4=Recovery
int initialize_cmd = 0;          // Arm command flag
int abort_cmd = 0;               // Abort command flag
int reset_cmd = 0;               // Reset command flag
int launch_detect = 0;           // Sensor flags
int descent_detect = 0;
int init_done = 0;

// Wifi's AP mode credentials
const char* ap_ssid = "sillygoose";
const char* ap_password = "sillygoose";

// Define Wifi parameters
WebServer server(80);
const int buzzerPin = 3;
bool hatchOpen = LOW;
char state_disp[20] = "Idle"; // Use a char array for state display
bool IMUState = LOW;
bool BaroState = LOW;

// State handler prototype functions (just fillers will be updated below)
//----------------------------------------------------
void idle_state();
void initial_state();
void armed_state();
void launch_state();
void recovery_state();

// Define data structures
//----------------------------------------------------
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

// Initialization
//----------------------------------------------------
// Servo initialize
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
const int servo_angle_max = 70;
const float servo_ang_to_fin_ang = 0.356;

// IMU initialize
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
long reportIntervalUs = 4000; // Top frequency is about 250Hz but this report is more accurate

// Baro initialize
float alt_init = 0; // Define initial altitude variable
MS5611 MS5611(0x77); // Create an instance

// Define initialization functions
//----------------------------------------------------
void setReports(long report_interval) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
  }

void IMU_init() {
    while (!bno08x.begin_I2C(0x4B)) {
      digitalWrite(buzzerPin, HIGH);
      delay(100); 
      digitalWrite(buzzerPin, LOW);
      delay(100);
    }
    setReports(reportIntervalUs);
    delay(100);
}

void Baro_init() {
    while (!MS5611.begin()){
      digitalWrite(buzzerPin, HIGH);
      delay(100); 
      digitalWrite(buzzerPin, LOW);
      delay(100); 
    }
    MS5611.reset(1);
    MS5611.setOversampling(OSR_ULTRA_HIGH);
    delay(100);
}

void Servo_init() {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz
    delay(100);
}

void SD_init() {
    while (!SD.begin(chipSelect)) {
      delay(100);
    }
    File dataFile = SD.open(filename, FILE_WRITE); // Write header
    if (dataFile) {
      dataFile.println("time,raw_altitude,estimated_speed,acc_enu_z,x_pitch,y_roll,z_yaw,x_pitch_dot,y_roll_dot,z_yaw_dot,pid2_x_output,pid2_y_out,pid1_z_output,servo1_ang,servo2_ang,servo3_ang,servo4_ang");
      dataFile.close();
    }
}

void Wifi_init() {
    // Set up Access Point
    WiFi.softAPConfig(IPAddress(192,168,1,1), IPAddress(192,168,1,1), IPAddress(255,255,255,0));
    WiFi.softAP(ap_ssid, ap_password);
    
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    // Server endpoints
    server.on("/", handleRoot);
    server.on("/openh", openHatch);
    server.on("/closeh", closeHatch);
    server.on("/arm", initialArm);
    server.on("/reset", reset);
    server.on("/abort", []() { abort_cmd = 1; server.sendHeader("Location", "/"); server.send(303); });
    server.on("/refresh", refresh);
    
    server.begin();
    Serial.println("HTTP server started");
    delay(100);
}

void disableWiFi() {
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
}

// Define helper functions
//----------------------------------------------------
float lowPassFilter(float input, float alpha) {
    static float prevOutput = 0.0;
    float output = alpha * input + (1 - alpha) * prevOutput;
    prevOutput = output;
    return output;
}

float pressure_altitude(float pressure_hpa) {
    const float p0_hpa = 1013.25;
    float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
    return altitude;
  }

// Define math functions
//----------------------------------------------------
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

int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void realServoAngles(int servo_num, int angle) { // 0 to 90 is servo cw, 90 to 180 is servo ccw
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
  if (relative_height < height_rail) { // 7m is the height of the launch rail
      return 0.; // fins should not move when the rocket is on the launch rail
  }
  else if (vertical_speed > min_control_speed) {
      return pow((30 / vertical_speed), 2); // PID tuned at 30 m/s
  }
  else {
      return pow((30 / min_control_speed), 2);; // max k_act is 3^2, since min speed is 10 m/s
  }
}

void initial_altitude(float *alt_init){
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

void writeDataSD(double time_ms, float raw_altitude, float estimated_speed, float acc_enu_z, 
                 float x_pitch, float y_roll, float z_yaw, float x_pitch_dot, float y_roll_dot, 
                 float z_yaw_dot, float pid2_x_output, float pid2_y_output, float pid1_z_output,
                 int servo1_ang, int servo2_ang, int servo3_ang, int servo4_ang) {
  String dataString = String(time_ms)  + "," + 
                      String(raw_altitude) + "," +
                      String(estimated_speed) + "," +
                      String(acc_enu_z) + "," +
                      String(x_pitch) + "," +
                      String(y_roll) + "," +
                      String(z_yaw) + "," +
                      String(x_pitch_dot) + "," +
                      String(y_roll_dot) + "," +
                      String(z_yaw_dot) + "," +
                      String(pid2_x_output) + "," +
                      String(pid2_y_output) + "," +
                      String(pid1_z_output) + "," +
                      String(servo1_ang) + "," +
                      String(servo2_ang) + "," +
                      String(servo3_ang) + "," +
                      String(servo4_ang);

  // Open file and append data
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
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
  
void hatchServo(bool hatchOpen) {
  int servo_num_hatch = 8;
  if (hatchOpen) {
    realServoAngles(servo_num_hatch, 50);
  } else {
    realServoAngles(servo_num_hatch, 110);
  }
}

void beep1() {
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
}

void beep3() {
    for (int i = 0; i < 3; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(300);
      digitalWrite(buzzerPin, LOW);
      delay(300);
    }
}

// Web handlers functions
//----------------------------------------------------
void handleRoot() {
    String html = "<html><head><style>";
    html += "h1 { font-size: 2.5em; }";
    html += "p { font-size: 1.5em; }";
    html += "button { font-size: 1.5em; padding: 10px 20px; }";
    html += "</style></head><body>";
    html += "<h1>Silly Goose Wifi Control</h1>";
    html += "<p>Current State: <b>" + String(state_disp) + "</b></p>";
    html += "<p>IMU: <b>" + String(IMUState ? "Ok" : "Off") + "</b></p>";
    html += "<p>Baro: <b>" + String(BaroState ? "Ok" : "Off") + "</b></p>";
    html += "<p>Hatch: <b>" + String(hatchOpen ? "Open" : "Closed") + "</b></p>";
    html += "<a href='/arm'><button>InitArm</button></a>&nbsp;";
    html += "<a href='/abort'><button>Abort</button></a>&nbsp;";
    html += "<a href='/reset'><button>Reset</button></a>&nbsp;";
    html += "<a href='/refresh'><button>Refresh</button></a>";
    html += "<br>";
    html += "<br>";
    html += "<a href='/openh'><button>Open hatch</button></a>&nbsp;";
    html += "<a href='/closeh'><button>Close hatch</button></a>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  }
  
  void openHatch() {
    if (state == 0) { // Only in idle
    hatchOpen = HIGH;
    hatchServo(hatchOpen);
    server.sendHeader("Location", "/");
    server.send(303);
    }
  }
  
  void closeHatch() {
    if (state == 0) { // Only in idle
    hatchOpen = LOW;
    hatchServo(hatchOpen);
    server.sendHeader("Location", "/");
    server.send(303);
    }
  }
  
  void initialArm() {
    initialize_cmd = 1; // Change state from idle to initial
    server.sendHeader("Location", "/");
    server.send(303);
  }
  
  void reset() {
    reset_cmd = 1; // Change state from armed or recovery to idle
    server.sendHeader("Location", "/");
    server.send(303);
  }
  
  void refresh() {
    server.sendHeader("Location", "/");
    server.send(303);
  }

// Define state functions
//----------------------------------------------------
void idle_state() { // Wait for command and open hatch

}

void initial_state(float *alt_init, int *init_done) { // Initialize sensors
    initial_altitude(alt_init); // Initialize altitude
    *init_done = 1;
}

void armed_state(float acc_enu_z, int *launch_detect) { // Wait for launch detection
    launchDetect(acc_enu_z, launch_detect);
    
    int servo_angles_out[4] = {0, 0, 0, 0}; // set target fin angles to 0
    setIdealAngles(servo_angles_out);
}

void launch_state(int *descent_detect, float estimated_altitude, float estimated_vertical_speed, 
                  float z_yaw, float z_yaw_dot, float y_roll, float y_roll_dot, float x_pitch, float x_pitch_dot, 
                  float dt_us, long time_us, float raw_altitude, float acc_enu_z) { // Ascend and apogee
    // Actuation factor
    float actuation_factor = actuationFactor(estimated_altitude, estimated_vertical_speed);

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
    }
    else {
        x_pitch_saturation = false;
    }
    if (servo2_saturation || servo4_saturation) {
        y_roll_saturation = true;
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

    // if (Serial){
    // Serial.print(fin1_ang);           Serial.print(",");
    // Serial.print(fin2_ang);          Serial.print(",");
    // Serial.print(fin3_ang);          Serial.print(",");
    // Serial.print(fin4_ang);          Serial.print(",");
    // Serial.println(actuation_factor);
    // }

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

    // define desired angles for the 4 servos
    int servo_angles_out[4] = {servo1_ang_out, servo2_ang_out, servo3_ang_out, servo4_ang_out}; 
    setIdealAngles(servo_angles_out);
  
    // detect descent to trigger recovery state
    descentDetect(estimated_altitude, x_pitch, y_roll, descent_detect);

    // write data to SD card
    writeDataSD(millis(), raw_altitude, estimated_vertical_speed, acc_enu_z, 
    x_pitch, y_roll, z_yaw, x_pitch_dot, y_roll_dot, z_yaw_dot,
    output_x_pitch, output_y_roll, output_z_yaw,
    fin1_ang, fin2_ang, fin3_ang, fin4_ang);
}

void recovery_state() { // Descend and deploy parachute
    long time = millis();
    static long time_pre = 0;

    hatchOpen = HIGH;
    hatchServo(hatchOpen);

    // Fin aerobraking
    float fin1_ang = 30;
    float fin2_ang = -30;
    float fin3_ang = 30;
    float fin4_ang = -30;

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

    if ((time - time_pre) > 30000) {
        beep3();
        beep3();
        time_pre = time;
    }
}

// Define functions for event conditions
//----------------------------------------------------
void launchDetect(float acc_enu_z, int *launch_detect) {
    static float acc_buffer[launch_n_samples] = {0};
    static int index = 0;
    static int count = 0;
    float sum = 0;

    acc_buffer[index] = acc_enu_z;
    index = (index + 1) % launch_n_samples;
    if (count < launch_n_samples) count++;

    for (int i = 0; i < count; i++) {
        sum += acc_buffer[i];
    }
    float average_acc = sum / count;

    if (average_acc > acc_z_threshold) {
        *launch_detect = 1;
    }
}

void descentDetect(float baro_alt, float ang_x, float ang_y, int *descent_detect) {
    static float ang_x_buffer[end_angle_n_samples] = {0};
    static float ang_y_buffer[end_angle_n_samples] = {0};
    static int index = 0;
    static int count = 0;
    static float alt_apogee = 0;
    int judge_x = 0, judge_y = 0;

    if (baro_alt > alt_apogee) {
        alt_apogee = baro_alt;
    }

    ang_x_buffer[index] = ang_x;
    ang_y_buffer[index] = ang_y;
    index = (index + 1) % end_angle_n_samples;
    if (count < end_angle_n_samples) count++;

    for (int i = 0; i < count; i++) {
        if (ang_x_buffer[i] > end_angle_threshold) judge_x++;
        if (ang_y_buffer[i] > end_angle_threshold) judge_y++;
    }

    if (judge_x == end_angle_n_samples || judge_y == end_angle_n_samples) {
        *descent_detect = 1;
    } 
    else if (baro_alt + end_height_from_apogee < alt_apogee) { 
        *descent_detect = 1; 
    } 
    else {
        *descent_detect = 0;
    }
}

// Setup function
//----------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(buzzerPin, OUTPUT);
    IMU_init();
    Baro_init();
    Servo_init();
    SD_init();
    Wifi_init();
}

// Loop function
//----------------------------------------------------
void loop() {
    // IMU data and procrssing
    //----------------------------------------------------
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
      }}
    server.handleClient(); // Handle incoming client requests, WIFI

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
    
    switch(state) {
      case 0: // Idle
        strcpy(state_disp, "Idle"); // Update state display
        idle_state();
        if(initialize_cmd and !hatchOpen) {
          state = 1;
          initialize_cmd = 0;
          beep3();
        } else {
          initialize_cmd = 0; // Cancel operation if hatch is open
        }
        break;
        
      case 1: // Initialization
        strcpy(state_disp, "Initialization"); // Update state display
        initial_state(&alt_init, &init_done); // Pass address of alt_init
        if (init_done == 1) { // Fixed comparison operator
          state = 2;  // Move to armed after initialization
          init_done = 0;
          beep3();
        }
        break;
              
      case 2: // Armed
        strcpy(state_disp, "Armed"); // Update state display
        armed_state(acc_enu_z, &launch_detect);
        if(launch_detect) { // Launch detection
          disableWiFi(); // Disable WiFi before entering launch state
          state = 3;
          launch_detect = 0;
        } else if(abort_cmd) { // Abort command
          state = 0;
          abort_cmd = 0;
          beep1();
        }
        break;
        
      case 3: // Launch
        strcpy(state_disp, "Launched"); // Update state display
        launch_state(&descent_detect, estimated_altitude, estimated_vertical_speed, z_yaw,
                      z_yaw_dot, y_roll, y_roll_dot, x_pitch, x_pitch_dot, dt_us, t_now, adjusted_altitude, acc_enu_z);
        if(descent_detect) {
          state = 4;
          descent_detect = 0;
        }
        break;
        
      case 4: // Recovery
        strcpy(state_disp, "Recovery"); // Update state display
        recovery_state();
        if(reset_cmd) {
          state = 0;
          reset_cmd = 0;
        }
        break;
    }
  }