#include <Wire.h>

void setup() {
  // Start the I2C bus
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); // Wait for serial to initialize

  Serial.println("\nI2C Sweep:");
}

void loop() {
  byte error, address;
  int nDevices;

  nDevices = 0;
  
  // Scan through all possible I2C addresses (0-127)
  for (address = 1; address < 128; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      // Device found
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print(nDevices);
    Serial.println(" devices found");
  }

}
