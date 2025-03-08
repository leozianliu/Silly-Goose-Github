#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

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

// Define global parameters
int state = 0 // state: 0=Idle, 1=Initial, 2=Armed, 3=Launch, 4=Recovery
bool initialize_cmd = 0;
bool launch_detect = 0;
bool abort_cmd = 0;
bool reset_cmd = 0;

// Initialization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
const int servo_angle_max = 70;
const float servo_ang_to_fin_ang = 0.356;

#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
long reportIntervalUs = 4000; // Top frequency is about 250Hz but this report is more accurate

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
    beep3();
}

// Define helper functions
//----------------------------------------------------
void beep3() {
    for (int i = 0; i < 3; i++) {
      digitalWrite(3, HIGH);
      delay(300);
      digitalWrite(3, LOW);
      delay(300);
    }
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
    sleep(100);
}

// Loop function
//----------------------------------------------------
void loop() {
    if (state == 0) {
        idle_state();
        if (initialize_cmd == 1) {
            state = 1;
        }
    }
    else if (state == 1) {
        initial_state();
        state = 2; // Go to armed state after done
    } 
    else if (state == 2) {
        armed_state();
        if (launch_detect == 1) {
            state == 3;
        }
        else if (abort_cmd == 1) {
            state == 0;
        }
    }
    else if (state == 3) {
        launch_state();
        if (descent_detect == 1) {
            state == 4;
        }
    }
    else if (state == 4) {
        recovery_state();
        if (reset_cmd == 1) {
            state == 0;
        }
    }
}