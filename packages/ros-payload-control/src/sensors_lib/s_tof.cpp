#include "payload_control.hpp"

void PayloadControl::TofSensorSetup() {
    Wire.begin(21, 22);
    tof_sensor_.init();
    tof_sensor_.configureDefault();
    //tof_sensor_.writeReg16Bit(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 0x32);

}
void PayloadControl::TofRead() {
  waterlevel_ = (float)tof_sensor_.readRangeSingleMillimeters();
}