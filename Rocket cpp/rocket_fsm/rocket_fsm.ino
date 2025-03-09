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

// Define Global State Machine Variables
//----------------------------------------------------
int state = 0;                   // state: 0=Idle, 1=Initial, 2=Armed, 3=Launch, 4=Recovery
int initialize_cmd = 0;          // Arm command flag
int abort_cmd = 0;               // Abort command flag
int reset_cmd = 0;               // Reset command flag
int launch_detect = 0;           // Sensor flags
int descent_detect = 0;

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
    delay(10); 
  }
  setReports(reportIntervalUs);
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
}

void Servo_init() {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz
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
}

// Define helper functions
//----------------------------------------------------
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
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    delay(500);
    digitalWrite(3, LOW);
    delay(500);
}

void beep3() {
    for (int i = 0; i < 3; i++) {
      digitalWrite(3, HIGH);
      delay(300);
      digitalWrite(3, LOW);
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

void initial_state() { // Initialize sensors
    
}

void armed_state() { // Wait for launch detection

}

void launch_state() { // Ascend and apogee

}

void recovery_state() { // Descend and deploy parachute

}

// Setup function
//----------------------------------------------------
void setup() {
    Serial.begin(115200);
    IMU_init();
    Baro_init();
    Servo_init();
    Wifi_init()
    sleep(100);
}

// Loop function
//----------------------------------------------------
void loop() {
    server.handleClient();
    
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
        initial_state();
        state = 2;  // Move to armed after initialization
        beep3();
        break;
        
      case 2: // Armed
        strcpy(state_disp, "Armed"); // Update state display
        armed_state();
        if(launch_detect) {
          state = 3;
          launch_detect = 0;
        } else if(abort_cmd) {
          state = 0;
          abort_cmd = 0;
          beep1();
        }
        break;
        
      case 3: // Launch
        strcpy(state_disp, "Launched"); // Update state display
        launch_state();
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