#include <WiFi.h>
#include <WebServer.h>
#include <cstring> // For strcpy()

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
bool buzzerState = LOW;
char state_disp[20] = "Idle"; // Use a char array for state display
bool IMUState = LOW;
bool BaroState = LOW;

// State handler prototypes
void idle_state();
void initial_state();
void armed_state();
void launch_state();
void recovery_state();

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, buzzerState);

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
  String html = "<html><body>";
  html += "<h1>Silly Goose Wifi Control</h1>";
  html += "<p>Current State: <b>" + String(state_disp) + "</b></p>";
  html += "<p>IMU: <b>" + String(IMUState ? "Off" : "Ok") + "</b></p>";
  html += "<p>Baro: <b>" + String(BaroState ? "Off" : "Ok") + "</b></p>";
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
  buzzerState = HIGH;
  digitalWrite(buzzerPin, buzzerState);
  server.sendHeader("Location", "/");
  server.send(303);
}

void closeHatch() {
  buzzerState = LOW;
  digitalWrite(buzzerPin, buzzerState);
  server.sendHeader("Location", "/");
  server.send(303);
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

// State handlers (implement your logic here)
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
      if(initialize_cmd) {
        state = 1;
        initialize_cmd = 0;
      }
      break;
      
    case 1: // Initialization
      strcpy(state_disp, "Initialization"); // Update state display
      initial_state();
      state = 2;  // Move to armed after initialization
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