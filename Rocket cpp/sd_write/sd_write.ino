#include <SPI.h>
#include <SD.h>

const int chipSelect = 17; // for SD card reader
const char filename[] = "FLIGHTLOG.csv";

void setup() {
  Serial.begin(115200);
  
  // Initialize SD card
  while (!SD.begin(chipSelect)) {
    delay(100);
  }
  
  // Write initial file
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,raw_altitude,estimated_speed,acc_enu_z,x_pitch,y_roll,z_yaw,x_pitch_dot,y_roll_dot,z_yaw_dot,pid2_x_output,pid2_y_out,pid1_z_output,servo1_ang,servo2_ang,servo3_ang,servo4_ang");
    dataFile.close();
  }
}

void loop() {
  String dataString = String(millis())  + "," + 
                      String(altitude) + "," +
                      String(estimated_speed) + "," +
                      String(acc_enu_z) + "," +
                      String(x_pitch) + "," +
                      String(y_roll) + "," +
                      String(z_yaw) + "," +
                      String(x_pitch_dot) + "," +
                      String(y_roll_dot) + "," +
                      String(z_yaw_dot) + "," +
                      String(pid_x_output) + "," +
                      String(pid_y_out) + "," +
                      String(pid_z_output) + "," +
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