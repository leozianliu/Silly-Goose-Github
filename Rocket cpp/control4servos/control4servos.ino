/*************************************************** 
  Example for Adafruit 16-channel PWM & Servo driver
  Encapsulated servo angle control into a function

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  Modified to include custom functions for setting servo angles.
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PWM driver using the default address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse length constants for servos
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const int servo1_iang = 85;
const int servo2_iang = 96;
const int servo3_iang = 75;
const int servo4_iang = 80;

void setup() {
  // Serial.begin(9600);
  // Serial.println("Servo control using functions!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency to 50 Hz

  delay(10);
}

// Helper function to map an angle (0-180) to the pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to set angles for multiple servos
// 0 to 90 is servo cw, 90 to 180 is servo ccw
void realServoAngles(int servo_num, int angle) {
  int pulseLength = angleToPulse(angle);
  pwm.setPWM(servo_num, 0, pulseLength); // Set PWM for servo
  // Serial.print("Servo set to angle ");
  // Serial.println(angle);
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

void loop() {
  // Define desired angles for the first 4 servos
  int angles1[4] = {0, 0, 0, 0}; 

  // Set servo angles
  setIdealAngles(angles1);

  delay(500); // Hold positions for 0.5 second

  // // // Reset all servos to 0 degrees
  // int angles2[4] = {40, 40, 40, 40};
  // setIdealAngles(angles2);

  // delay(200); // Hold positions for 0.5 second

  // // Reset all servos to 0 degrees
  // int angles3[4] = {0, 0, 0, 0};
  // setIdealAngles(angles3);

  // delay(500); // Hold positions for 1 second

  // // Reset all servos to 0 degrees
  // int angles4[4] = {-60, 60, -60, 60};
  // setIdealAngles(angles4);

  // delay(500); // Hold positions for 1 second

  //   // Reset all servos to 0 degrees
  // int angles5[4] = {-60, 0, 60, 0};
  // setIdealAngles(angles5);

  // delay(500); // Hold positions for 1 second

  //   // Reset all servos to 0 degrees
  // int angles6[4] = {60, 0, -60, 0};
  // setIdealAngles(angles6);

  // delay(200);

  // // Reset all servos to 0 degrees
  // int angles7[4] = {-60, 0, 60, 0};
  // setIdealAngles(angles7);

  // delay(200); // Hold positions for 1 second

  // // Reset all servos to 0 degrees
  // int angles8[4] = {0, 0, 0, 0};
  // setIdealAngles(angles8);

  // delay(500); // Hold positions for 1 second
}
