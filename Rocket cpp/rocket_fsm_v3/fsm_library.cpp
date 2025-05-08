#include "FSM_Library.h"

// Initialize global constants
const int servo1_iang = 75;
const int servo2_iang = 90;
const int servo3_iang = 75;
const int servo4_iang = 90;
const int n_samples = 100;
float R = 0.01;
float acc_std = 0.1;
const int min_control_speed = 10;
const float height_rail = 7.5;
float acc_z_threshold = 3;
const int launch_n_samples = 10;
const int end_angle_n_samples = 2;
const float end_angle_threshold = 45;
const float end_height_from_apogee = 5;
const int end_height_n_samples = 10;
const int chipSelect = 17;
const char filename[] = "FLIGHTLOG.csv";

// Initialize global state machine variables
int state = 0;
int initialize_cmd = 0;
int abort_cmd = 0;
int reset_cmd = 0;
int launch_detect = 0;
int descent_detect = 0;
int init_done = 0;
const char* ap_ssid = "sillygoose";
const char* ap_password = "sillygoose";
const int buzzerPin = 3;
bool hatchOpen = LOW;
char state_disp[20] = "Idle";

// WiFiControl Class Implementation
void WiFiControl::init() {
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

void WiFiControl::disableWiFi() {
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
}

void WiFiControl::handleRoot() {
    String html = "<html><head><style>";
    html += "h1 { font-size: 2.5em; }";
    html += "p { font-size: 1.5em; }";
    html += "button { font-size: 1.5em; padding: 10px 20px; }";
    html += "</style></head><body>";
    html += "<h1>Silly Goose Wifi Control</h1>";
    html += "<p>Current State: <b>" + String(state_disp) + "</b></p>";
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

void WiFiControl::openHatch() {
    if (state == 0) {
        hatchOpen = HIGH;
        hatchServo(hatchOpen);
        server.sendHeader("Location", "/");
        server.send(303);
    }
}

void WiFiControl::closeHatch() {
    if (state == 0) {
        hatchOpen = LOW;
        hatchServo(hatchOpen);
        server.sendHeader("Location", "/");
        server.send(303);
    }
}

void WiFiControl::initialArm() {
    initialize_cmd = 1;
    server.sendHeader("Location", "/");
    server.send(303);
}

void WiFiControl::reset() {
    reset_cmd = 1;
    server.sendHeader("Location", "/");
    server.send(303);
}

void WiFiControl::refresh() {
    server.sendHeader("Location", "/");
    server.send(303);
}

// Helper Class Implementation
float Helper::lowPassFilter(float input, float alpha) {
    static float prevOutput = 0.0;
    float output = alpha * input + (1 - alpha) * prevOutput;
    prevOutput = output;
    return output;
}

float Helper::pressure_altitude(float pressure_hpa) {
    const float p0_hpa = 1013.25;
    float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255));
    return altitude;
}

void Helper::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    float sin_pitch = -2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr);
    if (abs(sin_pitch) >= 1.0) {
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

void Helper::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void Helper::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void Helper::acc_ZYX_body_to_inertial(float x_pitch, float y_roll, float z_yaw, cartesian_t* body_acc, cartesian_t* enu_acc) {
    float R_mat[3][3] = {
        {cos(y_roll) * cos(z_yaw), -cos(x_pitch) * sin(z_yaw) + sin(x_pitch) * sin(y_roll) * cos(z_yaw), sin(x_pitch) * sin(z_yaw) + cos(x_pitch) * sin(y_roll) * cos(z_yaw)},
        {cos(y_roll) * sin(z_yaw), cos(x_pitch) * cos(z_yaw) + sin(x_pitch) * sin(y_roll) * sin(z_yaw), -sin(x_pitch) * cos(z_yaw) + cos(x_pitch) * sin(y_roll) * sin(z_yaw)},
        {-sin(y_roll), sin(x_pitch) * cos(y_roll), cos(x_pitch) * cos(y_roll)}
    };

    float acc_body[3] = {body_acc->x, body_acc->y, body_acc->z};
    float acc_enu[3];
    matrixTimesVector3D(R_mat, acc_body, acc_enu);

    enu_acc->x = acc_enu[0];
    enu_acc->y = acc_enu[1];
    enu_acc->z = acc_enu[2];
}

void Helper::matrixTimesVector3D(float mat[3][3], float vec[3], float result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;
        for (int j = 0; j < 3; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

void Helper::writeDataSD(double time_ms, float raw_altitude, float estimated_speed, float acc_enu_z, 
                       float x_pitch, float y_roll, float z_yaw, float x_pitch_dot, float y_roll_dot, 
                       float z_yaw_dot, float pid2_x_output, float pid2_y_output, float pid1_z_output,
                       int servo1_ang, int servo2_ang, int servo3_ang, int servo4_ang) {
    String dataString = String(time_ms) + "," + 
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

    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
    }
}

void Helper::hatchServo(bool hatchOpen) {
    int servo_num_hatch = 8;
    if (hatchOpen) {
        realServoAngles(servo_num_hatch, 50);
    } else {
        realServoAngles(servo_num_hatch, 110);
    }
}

void Helper::beep1() {
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
}

void Helper::beep3() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(buzzerPin, HIGH);
        delay(300);
        digitalWrite(buzzerPin, LOW);
        delay(300);
    }
}

// Detect Class Implementation
void Detect::launchDetect(float acc_enu_z, int *launch_detect) {
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

void Detect::descentDetect(float estimated_altitude, float ang_x, float ang_y, int *descent_detect) {
    static float ang_x_buffer[end_angle_n_samples] = {0};
    static float ang_y_buffer[end_angle_n_samples] = {0};
    static int index = 0;
    static int count = 0;
    static float alt_apogee = 0;
    int judge_x = 0, judge_y = 0;

    if (estimated_altitude > alt_apogee) {
        alt_apogee = estimated_altitude;
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
    else if (estimated_altitude + end_height_from_apogee < alt_apogee) { 
        *descent_detect = 1; 
    } 
    else {
        *descent_detect = 0;
    }
}

// Control Class Implementation
float Control::pidControl(pid_error_t* error_save, float target, float measure, float dt_us, float kp, float ki, float kd, bool anti_wind, bool saturation) {
    float error_pre = error_save->error_pre;
    float error_sum = error_save->error_sum;
    float error = target - measure;
    float error_dot = (error - error_pre) / (dt_us * 1e-6);
    
    error_save->error_pre = error;

    if (!(saturation && anti_wind)) {
        error_save->error_sum = error_sum + error * (dt_us * 1e-6);
    }

    float output = kp * error + ki * error_save->error_sum + kd * error_dot;
    return output;
}

int Control::angleToPulse(int angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void Control::realServoAngles(int servo_num, int angle) {
    int pulseLength = angleToPulse(angle);
    pwm.setPWM(servo_num, 0, pulseLength);
}

void Control::setIdealAngles(int angles[]) {
    int servo1 = angles[0] + servo1_iang;
    int servo2 = angles[1] + servo2_iang;
    int servo3 = angles[2] + servo3_iang;
    int servo4 = angles[3] + servo4_iang;

    realServoAngles(0, servo1);  
    realServoAngles(1, servo2);  
    realServoAngles(2, servo3);  
    realServoAngles(3, servo4);  
}

int Control::outputSaturation(int demand, int limit) {
    if (demand > 0) {
        return min(demand, limit);
    } 
    else {
        return max(demand, -limit);
    }
}

bool Control::saturationDetection(int demand, int limit) {
    if (demand > 0) {
        return (demand > limit);
    } 
    else {
        return (demand < -limit);
    }
}

float Control::actuationFactor(float relative_height, float vertical_speed) {
    if (relative_height < height_rail) {
        return 0.;
    }
    else if (vertical_speed > min_control_speed) {
        return pow((30 / vertical_speed), 2);
    }
    else {
        return pow((30 / min_control_speed), 2);
    }
}

void Control::kalman_filter(float baro_alt, unsigned long time_new, float s_state_out[3]) {
    static bool initialized = false;
    static float s_state_prev[3] = {0};
    static float P_prev[3][3] = {{0}};
    static unsigned long time_old = 0;

    if (!initialized) {
        s_state_prev[0] = baro_alt;
        s_state_prev[1] = 0;
        s_state_prev[2] = 0;
        
        P_prev[0][0] = 1.0f;
        P_prev[1][1] = 1.0f;
        P_prev[2][2] = 1.0f;
        
        time_old = time_new;
        initialized = true;
        return;
    }

    if (time_new <= time_old) {
        return;
    }
    
    float dt = (time_new - time_old) / 1e6f;
    time_old = time_new;
    
    if (dt > 0.1f) dt = 0.1f;

    float F[3][3] = {
        {1, dt, 0.5f * dt * dt},
        {0, 1, dt},
        {0, 0, 1}
    };

    float G[3] = {0.5f * dt * dt, dt, 1};

    float Q[3][3] = {
        {G[0]*G[0]*acc_std*acc_std, G[0]*G[1]*acc_std*acc_std, G[0]*G[2]*acc_std*acc_std},
        {G[1]*G[0]*acc_std*acc_std, G[1]*G[1]*acc_std*acc_std, G[1]*G[2]*acc_std*acc_std},
        {G[2]*G[0]*acc_std*acc_std, G[2]*G[1]*acc_std*acc_std, G[2]*G[2]*acc_std*acc_std}
    };

    float s_state_pred[3] = {
        F[0][0]*s_state_prev[0] + F[0][1]*s_state_prev[1] + F[0][2]*s_state_prev[2],
        F[1][0]*s_state_prev[0] + F[1][1]*s_state_prev[1] + F[1][2]*s_state_prev[2],
        F[2][0]*s_state_prev[0] + F[2][1]*s_state_prev[1] + F[2][2]*s_state_prev[2]
    };

    float F_T[3][3], temp[3][3], P_pred[3][3];
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_T[i][j] = F[j][i];
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += F[i][k] * P_prev[k][j];
            }
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                P_pred[i][j] += temp[i][k] * F_T[k][j];
            }
            P_pred[i][j] += Q[i][j];
        }
    }

    float H[3] = {1, 0, 0};
    float S = 0;
    for (int i = 0; i < 3; i++) {
        S += H[i] * P_pred[0][i];
    }
    S += R;

    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = 0;
        for (int j = 0; j < 3; j++) {
            K[i] += P_pred[i][j] * H[j];
        }
        K[i] /= S;
    }

    float innov = baro_alt - s_state_pred[0];
    float s_state[3] = {
        s_state_pred[0] + K[0] * innov,
        s_state_pred[1] + K[1] * innov,
        s_state_pred[2] + K[2] * innov
    };

    float KH[3][3], I[3][3] = {{1,0,0},{0,1,0},{0,0,1}}, P[3][3];
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            KH[i][j] = K[i] * H[j];
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                P[i][j] += (I[i][k] - KH[i][k]) * P_pred[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        if (P[i][i] < 0) P[i][i] = 0;
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = i+1; j < 3; j++) {
            P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
        }
    }

    for (int i = 0; i < 3; i++) {
        s_state_prev[i] = s_state[i];
        s_state_out[i] = s_state[i];
        for (int j = 0; j < 3; j++) {
            P_prev[i][j] = P[i][j];
        }
    }
}

// Initialization functions
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
    pwm.setPWMFreq(SERVO_FREQ);
    delay(100);
}

void SD_init() {
    while (!SD.begin(chipSelect)) {
        delay(100);
    }
    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.println("time,raw_altitude,estimated_speed,acc_enu_z,x_pitch,y_roll,z_yaw,x_pitch_dot,y_roll_dot,z_yaw_dot,pid2_x_output,pid2_y_out,pid1_z_output,servo1_ang,servo2_ang,servo3_ang,servo4_ang");
        dataFile.close();
    }
}

void initial_altitude(float *alt_init) {
    static int counter = 1;
    bool run_initial_altitude = true;

    while (run_initial_altitude == true) {
        int result = MS5611.read();
        float pressure = MS5611.getPressure();
        float raw_altitude = pressure_altitude(pressure);
            
        if (counter <= n_samples) {
            *alt_init = *alt_init * (counter - 1) / counter + raw_altitude / counter;
            counter = counter + 1;
        }
        else if (counter > n_samples) {
            counter = 1;
            run_initial_altitude = false;
        }
    }
}

// State handler functions
void idle_state() {
    strcpy(state_disp, "Idle");
    if(initialize_cmd && !hatchOpen) {
        state = 1;
        initialize_cmd = 0;
        beep3();
    } else {
        initialize_cmd = 0;
    }
}

void initial_state(float *alt_init, int *init_done) {
    strcpy(state_disp, "Initialization");
    initial_altitude(alt_init);
    *init_done = 1;
    state = 2;
    init_done = 0;
    beep3();
}

void armed_state(float acc_enu_z, int *launch_detect) {
    strcpy(state_disp, "Armed");
    Detect detect;
    detect.launchDetect(acc_enu_z, launch_detect);
    
    int servo_angles_out[4] = {0, 0, 0, 0};
    Control control;
    control.setIdealAngles(servo_angles_out);
    
    if(*launch_detect) {
        WiFiControl wifi;
        wifi.disableWiFi();
        state = 3;
        *launch_detect = 0;
    } else if(abort_cmd) {
        state = 0;
        abort_cmd = 0;
        beep1();
    }
}

void launch_state(int *descent_detect, float estimated_altitude, float estimated_vertical_speed, 
                float z_yaw, float z_yaw_dot, float y_roll, float y_roll_dot, float x_pitch, float x_pitch_dot, 
                float dt_us, long time_us, float raw_altitude, float acc_enu_z) {
    strcpy(state_disp, "Launched");
    
    Control control;
    Detect detect;
    Helper helper;
    
    float actuation_factor = control.actuationFactor(estimated_altitude, estimated_vertical_speed);

    static bool x_pitch_saturation = false;
    static bool y_roll_saturation = false;
    static int servo1_ang = 0;
    static int servo2_ang = 0;
    static int servo3_ang = 0;
    static int servo4_ang = 0;

    bool servo1_saturation = control.saturationDetection(servo1_ang, servo_angle_max);
    bool servo2_saturation = control.saturationDetection(servo2_ang, servo_angle_max);
    bool servo3_saturation = control.saturationDetection(servo3_ang, servo_angle_max);
    bool servo4_saturation = control.saturationDetection(servo4_ang, servo_angle_max);
    
    x_pitch_saturation = (servo1_saturation || servo3_saturation);
    y_roll_saturation = (servo2_saturation || servo4_saturation);

    float output_z_yaw = 0.;
    float output_y_roll_rate = 0.;
    float output_y_roll = 0.;
    float output_x_pitch_rate = 0.;
    float output_x_pitch = 0.;

    if (abs(actuation_factor) >= 1e-3) {
        output_z_yaw = control.pidControl(&z_yaw_pid, 0, z_yaw_dot, dt_us, 0.02, 0., 0., false, false);
        output_y_roll_rate = control.pidControl(&y_roll_pid, 0, y_roll, dt_us, 15., 1.0, 0., true, y_roll_saturation);
        output_y_roll = control.pidControl(&y_roll_rate_pid, output_y_roll_rate, y_roll_dot, dt_us, 0.12, 0., 0., false, false);
        output_x_pitch_rate = control.pidControl(&x_pitch_pid, 0, x_pitch, dt_us, 15., 1.0, 0., true, x_pitch_saturation);
        output_x_pitch = control.pidControl(&x_pitch_rate_pid, output_x_pitch_rate, x_pitch_dot, dt_us, 0.12, 0., 0., false, false);
    }

    float fin1_ang = (output_z_yaw + output_x_pitch);
    float fin2_ang = (output_z_yaw + output_y_roll);
    float fin3_ang = (output_z_yaw - output_x_pitch);
    float fin4_ang = (output_z_yaw - output_y_roll);

    fin1_ang *= actuation_factor;
    fin2_ang *= actuation_factor;
    fin3_ang *= actuation_factor;
    fin4_ang *= actuation_factor;

    servo1_ang = (int)(-fin1_ang / servo_ang_to_fin_ang);
    servo2_ang = (int)(-fin2_ang / servo_ang_to_fin_ang);
    servo3_ang = (int)(-fin3_ang / servo_ang_to_fin_ang);
    servo4_ang = (int)(-fin4_ang / servo_ang_to_fin_ang);

    int servo1_ang_out = control.outputSaturation(servo1_ang, servo_angle_max);
    int servo2_ang_out = control.outputSaturation(servo2_ang, servo_angle_max);
    int servo3_ang_out = control.outputSaturation(servo3_ang, servo_angle_max);
    int servo4_ang_out = control.outputSaturation(servo4_ang, servo_angle_max);

    int servo_angles_out[4] = {servo1_ang_out, servo2_ang_out, servo3_ang_out, servo4_ang_out}; 
    control.setIdealAngles(servo_angles_out);
  
    detect.descentDetect(estimated_altitude, x_pitch, y_roll, descent_detect);
    if(*descent_detect) {
        state = 4;
        *descent_detect = 0;
    }

    helper.writeDataSD(millis(), raw_altitude, estimated_vertical_speed, acc_enu_z, 
                     x_pitch, y_roll, z_yaw, x_pitch_dot, y_roll_dot, z_yaw_dot,
                     output_x_pitch, output_y_roll, output_z_yaw,
                     fin1_ang, fin2_ang, fin3_ang, fin4_ang);
}

void recovery_state() {
    strcpy(state_disp, "Recovery");
    long time = millis();
    static long time_pre = 0;

    hatchOpen = HIGH;
    Helper helper;
    helper.hatchServo(hatchOpen);

    Control control;
    float fin1_ang = 30;
    float fin2_ang = -30;
    float fin3_ang = 30;
    float fin4_ang = -30;

    int servo1_ang = (int)(-fin1_ang / servo_ang_to_fin_ang);
    int servo2_ang = (int)(-fin2_ang / servo_ang_to_fin_ang);
    int servo3_ang = (int)(-fin3_ang / servo_ang_to_fin_ang);
    int servo4_ang = (int)(-fin4_ang / servo_ang_to_fin_ang);

    int servo1_ang_out = control.outputSaturation(servo1_ang, servo_angle_max);
    int servo2_ang_out = control.outputSaturation(servo2_ang, servo_angle_max);
    int servo3_ang_out = control.outputSaturation(servo3_ang, servo_angle_max);
    int servo4_ang_out = control.outputSaturation(servo4_ang, servo_angle_max);

    int servo_angles_out[4] = {servo1_ang_out, servo2_ang_out, servo3_ang_out, servo4_ang_out}; 
    control.setIdealAngles(servo_angles_out);

    if ((time - time_pre) > 30000) {
        helper.beep3();
        helper.beep3();
        time_pre = time;
    }

    if(reset_cmd) {
        state = 0;
        reset_cmd = 0;
    }
}