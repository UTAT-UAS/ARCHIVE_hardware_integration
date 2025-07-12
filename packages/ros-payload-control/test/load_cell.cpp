#include <Arduino.h>
#include <HX711.h>

HX711 loadcell_;
const int LOADCELL_DOUT_PIN = 18;
const int LOADCELL_SCK_PIN = 5;
const long LOADCELL_OFFSET = 0;
const long LOADCELL_DIVIDER = -2220;
void setup()
{
    Serial.begin(115200);
    loadcell_.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    loadcell_.set_scale(LOADCELL_DIVIDER);
    loadcell_.set_offset(LOADCELL_OFFSET);
    loadcell_.tare();
}
void loop()
{
    Serial.print("Load cell reading: ");
    if (loadcell_.wait_ready_timeout(1000)) {
        long reading = loadcell_.get_units(1);
        Serial.println(reading);
    } else {
        Serial.println("HX711 not found.");
    }
}