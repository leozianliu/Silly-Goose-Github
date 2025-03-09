#include <WiFi.h>
#include <WebServer.h>
#include <cstring>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// AP mode credentials
const char* ap_ssid = "sillygoose";
const char* ap_password = "sillygoose";

// Global State Machine Variables
int state = 0;                   // Initial state: idle
int initialize_cmd = 0;          // Arm command flag
int abort_cmd = 0;               // Abort command flag
int reset_cmd = 0;               // Reset command flag
int launch_detect = 0;           // Sensor flags
int descent_detect = 0;

WebServer server(80);
const int buzzerPin = 3;
bool hatchOpen = LOW;
char state_disp[20] = "Idle"; // Use a char array for state display
bool IMUState = LOW;
bool BaroState = LOW;

// Initialize the PWM driver using the default address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Helper functions
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

void Servo_init() {
  while (!pwm.begin()) {
    Serial.println("Failed to initialize PWM controller!");
    delay(10);
  }
  pwm.setOscillatorFrequency(27000000);  // Ensure this matches your hardware
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz
  Serial.println("Servo initialized successfully.");
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
    delay(100);
    digitalWrite(3, LOW);
    delay(100);
  }
}

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// State handler prototypes
void idle_state();
void initial_state();
void armed_state();
void launch_state();
void recovery_state();

void setup() {
  Serial.begin(115200);

  Servo_init();

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

// Web handlers
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

// FSM states
void idle_state() {
  // Default idle operations
}

void initial_state() {
  delay(5000); // Simulate initialization delay
  // Initialization sequence
}

void armed_state() {
  // Armed state operations
}

void launch_state() {
  // Launch detection handling
}

void recovery_state() {
  // Recovery operations
}

// Loop function
// -------------------------------------------------------
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