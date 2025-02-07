// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>
// #include "Adafruit_VL6180X.h"
// #include "payload_control.hpp"

// // Create sensor object
// Adafruit_VL6180X vl6180x = Adafruit_VL6180X();

// void setup() {
//     Serial.begin(115200);  // Start Serial Monitor
    
//     // Initialize I2C with default pins
//     Wire.begin();  

//     // Initialize the sensor
//     if (!vl6180x.begin()) {
//         Serial.println("VL6180X sensor not found! Check wiring.");
//         while (1);  // Stop execution
//     }
//     Serial.println("VL6180X sensor initialized.");
// }

// void loop() {
//     // Read the distance in mm
//     uint8_t distance = vl6180x.readRange();
//     uint8_t status = vl6180x.readRangeStatus();

//     if (status == 0) {  // No error
//         Serial.print("Distance: ");
//         Serial.print(distance);
//         Serial.println(" mm");
//     } else {
//         Serial.println("Error reading distance!");
//     }

//     delay(16);  // Wait 500ms before next reading
// }


// // add function to map distance to water amount