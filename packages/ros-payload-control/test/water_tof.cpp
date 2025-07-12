// // void PayloadControl::tofSensorSetup() {
// //     Wire.begin();
// //     tof_sensor_.init();
// //     tof_sensor_.configureDefault();
// //     tof_sensor_.setTimeout(500);

// // }
// // void PayloadControl::TofRead() {
// //   waterlevel_ = (float)tof_sensor_.readRangeSingleMillimeters();
// // } implement the above but not ros and serial output

// #include <Arduino.h>
// #include <Wire.h>
// #include <VL6180X.h>

// VL6180X tof_sensor;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(21,22);
//   tof_sensor.init();
//   tof_sensor.configureDefault();
//   //tof_sensor.setTimeout(500);
//   //tof_sensor.writeReg16Bit(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
//   //tof_sensor.writeReg16Bit(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 0x32);
//   //set bit one of sysrange check ignore
//   tof_sensor.writeReg(VL6180X::SYSRANGE__RANGE_CHECK_ENABLES, 0x01);
//   tof_sensor.writeReg16Bit(VL6180X::SYSRANGE__RANGE_IGNORE_THRESHOLD, 0x634B);
//   tof_sensor.writeReg16Bit(VL6180X::SYSRANGE__CROSSTALK_COMPENSATION_RATE, 0xC26);
// }

// void loop() {
//     // send back return rate and water level
//     Serial.print("Water level: ");
//     Serial.println(tof_sensor.readRangeSingleMillimeters());

//     // read return signal rate from address
//     delay(50);
// }