#include <WiFi.h>
#include <WebServer.h>
#include <cstring>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PWMServoDriver.h>

// Define data structures
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

// Define important global parameters
//----------------------------------------------------
const int n_samples = 100; // Number of samples for initialization
float alpha_baro = 0.15; // Smoothing factor for low pass filter
float acc_z_threshold = 3; // Threshold for launch detection
const int launch_n_samples = 10; // Number of samples for launch detection
const int end_angle_n_samples = 5; // Number of samples for angles used to detect end of flight
const float end_angle_threshold = 45; // Threshold in deg for recovery condition
const float end_height_from_apogee = 2; // Distance in meters from the recorded height to detect descent
const int end_height_n_samples = 10; // Number of samples for angles used to detect end of flight


// Define Global State Machine Variables
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
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// Define initialization functions
//----------------------------------------------------
void setReports(long report_interval) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
  }

void IMU_init() {
  while (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to initialize BNO08x!");
    delay(100); 
  }
  setReports(reportIntervalUs);
  delay(100);
}

void Baro_init() {
    unsigned bmp_state = bmp.begin(0x76);
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
    delay(100);
}

void Servo_init() {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz
    delay(100);
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

int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void realServoAngles(int servo_num, int angle) { // 0 to 90 is servo cw, 90 to 180 is servo ccw
    int pulseLength = angleToPulse(angle);
    pwm.setPWM(servo_num, 0, pulseLength); // Set PWM for servo
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

void initial_state(float raw_altitude, float* alt_init, int* init_done) { // Initialize sensors
    static int counter = 1; // Counter must not be 0 due to division
    
    if (counter <= n_samples) {
      *alt_init = *alt_init * (counter - 1) / counter + raw_altitude / counter;
      counter = counter + 1;
    }
    else if (counter > n_samples) {
        counter = 1; // Reset counter and move to next state
        *init_done = 1;
    }
}

void armed_state(float acc_enu_z, int *launch_detect) { // Wait for launch detection
    launchDetect(acc_enu_z, launch_detect);
}

void launch_state(float baro_alt, float ang_x, float ang_y, int *descent_detect) { // Ascend and apogee
    descentDetect(baro_alt, ang_x, ang_y, descent_detect);
}

void recovery_state() { // Descend and deploy parachute
    long time = millis();
    static long time_pre = 0;

    hatchOpen = HIGH;
    hatchServo(hatchOpen);

    if ((time - time_pre) > 10000) {
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
    IMU_init();
    Baro_init();
    Servo_init();
    Wifi_init();
    pinMode(buzzerPin, OUTPUT);
    delay(100);
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
    delay(10); // 100 Hz

    server.handleClient();

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

    // Baro data and processing
    //----------------------------------------------------
    sensors_event_t temp_event, pressure_event;
    bmp_pressure->getEvent(&pressure_event);

    float pressure = pressure_event.pressure;
    float raw_altitude = pressure_altitude(pressure); // Get pressure altitude
    static float alt_init = 0; // Let's intialize the initial altitude to zero
    float adjusted_altitude = raw_altitude - alt_init;
    float filtered_altitude = lowPassFilter(adjusted_altitude, alpha_baro);

    Serial.print(adjusted_altitude);          Serial.print(",");
    Serial.println(acc_enu_z);
    
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
        initial_state(raw_altitude, &alt_init, &init_done); // Pass address of alt_init
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
        launch_state(filtered_altitude, x_pitch, y_roll, &descent_detect);
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