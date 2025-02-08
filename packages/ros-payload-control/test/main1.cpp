// void PayloadControl::tofSensorSetup() {
//     Wire.begin();
//     tof_sensor_.init();
//     tof_sensor_.configureDefault();
//     tof_sensor_.setTimeout(500);

// }
// void PayloadControl::tofRead() {
//   waterlevel_ = (float)tof_sensor_.readRangeSingleMillimeters();
// } implement the above but not ros and serial output

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

VL6180X tof_sensor;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  tof_sensor.init();
  tof_sensor.configureDefault();
  tof_sensor.setTimeout(500);
  tof_sensor.writeReg16Bit(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 0x32);
}

void loop() {
    // send back return rate and water level
    Serial.print("Water level: ");
    Serial.println(tof_sensor.readRangeSingleMillimeters());
    delay(50);
}