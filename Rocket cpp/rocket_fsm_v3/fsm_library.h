#ifndef FSM_LIBRARY_H
#define FSM_LIBRARY_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_PWMServoDriver.h>
#include <MS5611.h>

// Data structures
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

// Wifi class for communication and control
class WiFiControl {
  public:
    void init();
    void disableWiFi();
    void handleRoot();
    void openHatch();
    void closeHatch();
    void initialArm();
    void reset();
    void refresh();
    
  private:
    const char* ap_ssid;
    const char* ap_password;
    WebServer server;
    int buzzerPin;
    bool hatchOpen;
    char state_disp[20];
};

// Helper class for various utility functions
class Helper {
  public:
    float lowPassFilter(float input, float alpha);
    float pressure_altitude(float pressure_hpa);
    void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
    void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
    void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);    
    void acc_ZYX_body_to_inertial(float x_pitch, float y_roll, float z_yaw, cartesian_t* body_acc, cartesian_t* enu_acc);
    void matrixTimesVector3D(float mat[3][3], float vec[3], float result[3]);
    void writeDataSD(double time_ms, float raw_altitude, float estimated_speed, float acc_enu_z, 
                     float x_pitch, float y_roll, float z_yaw, float x_pitch_dot, float y_roll_dot, 
                     float z_yaw_dot, float pid2_x_output, float pid2_y_output, float pid1_z_output,
                     int servo1_ang, int servo2_ang, int servo3_ang, int servo4_ang);
    void hatchServo(bool hatchOpen);
    void beep1();
    void beep3();
};

// Class for detecting launch and descent events
class Detect {
  public:
    void launchDetect(float acc_enu_z, int *launch_detect);
    void descentDetect(float estimated_altitude, float x_pitch, float y_roll, int *descent_detect);
};

// Class for control algorithms
class Control {
  public:
    float pidControl(pid_error_t* error_save, float target, float measure, float dt_us, float kp, float ki, float kd, bool anti_wind, bool saturation);
    int angleToPulse(int angle);
    void realServoAngles(int servo_num, int angle);
    void setIdealAngles(int angles[]);
    int outputSaturation(int demand, int limit);
    bool saturationDetection(int demand, int limit);
    float actuationFactor(float relative_height, float vertical_speed);
    void kalman_filter(float baro_alt, unsigned long time_new, float s_state_out[3]);
};

// Function prototypes for initializing system components
void IMU_init();
void Baro_init();
void Servo_init();
void SD_init();
void initial_altitude(float *alt_init);

// State handler function prototypes
void idle_state();
void initial_state(float *alt_init, int *init_done);
void armed_state(float acc_enu_z, int *launch_detect);
void launch_state(int *descent_detect, float estimated_altitude, float estimated_vertical_speed, 
                  float z_yaw, float z_yaw_dot, float y_roll, float y_roll_dot, float x_pitch, float x_pitch_dot, 
                  float dt_us, long time_us, float raw_altitude, float acc_enu_z);
void recovery_state();

// Global constants defined elsewhere in implementation
extern const int servo1_iang;
extern const int servo2_iang;
extern const int servo3_iang;
extern const int servo4_iang;
extern const int n_samples;
extern float R;
extern float acc_std;
extern const int min_control_speed;
extern const float height_rail;
extern float acc_z_threshold;
extern const int launch_n_samples;
extern const int end_angle_n_samples;
extern const float end_angle_threshold;
extern const float end_height_from_apogee;
extern const int end_height_n_samples;
extern const int chipSelect;
extern const char filename[];

// Global state machine variables
extern int state;
extern int initialize_cmd;
extern int abort_cmd;
extern int reset_cmd;
extern int launch_detect;
extern int descent_detect;
extern int init_done;
extern const char* ap_ssid;
extern const char* ap_password;
extern const int buzzerPin;
extern bool hatchOpen;
extern char state_disp[20];

#endif // FSM_LIBRARY_H