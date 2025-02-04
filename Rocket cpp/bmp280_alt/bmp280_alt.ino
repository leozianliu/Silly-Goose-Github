#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

float pressure_altitude(float pressure_hpa) {
  const float p0_hpa = 1013.25;
  float altitude = 44330.0 * (1.0 - pow(pressure_hpa / p0_hpa, 1.0 / 5.255)); // in meters
  return altitude;
}

void setup() {
  Serial.begin(115200);

  unsigned bmp_state = bmp.begin(0x76);

  while (!bmp_state) 
    {Serial.println(F("Could not find any BMP280 sensor!"));
    delay(10);}

  /* Default settings from Bosch 280 datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  bmp_temp->printSensorDetails();
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  // Serial.print(F("Temperature = "));
  // Serial.print(temp_event.temperature);
  // Serial.println(" *C");

  unsigned long t_now = micros();
  float pressure = pressure_event.pressure;
  float altitude = pressure_altitude(pressure);
  Serial.print(t_now); // in 1e-6 seconds
  Serial.print(",");
  Serial.print(pressure_event.pressure * 100); // Pressure in Pa
  Serial.print(",");
  Serial.println(altitude);

  //delay(100);
}
