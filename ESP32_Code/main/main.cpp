#include "Arduino.h"
#include <Wire.h>

#include <vector>

#include <sensors.hpp>

#define sda_pin 17
#define scl_pin 16
#define sda_pin1 23
#define scl_pin1 22

// Define sensor refresh rate (IMU, Magnetron, GPS, PublishRate)
Sensors sensors(20.0,20.0,1.0,20.0);

void setup() {
  Serial.begin(115200);

  Wire.begin(sda_pin, scl_pin);
  Wire1.begin(sda_pin1, scl_pin1);

  sensors.initSensors(Wire, Wire1);
  
}

void loop() {

  sensors.readSensors();
  sensors.publishSensorData();

}
