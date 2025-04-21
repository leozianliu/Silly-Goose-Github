#include <Wire.h>
#include <MS5611.h>

MS5611 MS5611(0x77);

/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
*/

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

void setup() {
  Serial.begin(115200);

  Wire.begin();

  while (!MS5611.begin())
    {
    Serial.println("MS5611 sensor not found!");
    delay(10);}

  MS5611.reset(1);
  MS5611.setOversampling(OSR_ULTRA_HIGH);
}

void loop() {
  int result = MS5611.read();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }

  unsigned long t_now = micros();
  float pressure = MS5611.getPressure();
  float altitude = pressure_altitude(pressure);
  Serial.print(t_now); // in 1e-6 seconds
  Serial.print(",");
  Serial.println(altitude);

  delay(20);
}
