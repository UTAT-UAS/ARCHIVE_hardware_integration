#include "payload_control.hpp"

void PayloadControl::contServoAttach()
{ 
    pinMode(contServoPin_, OUTPUT);

    // this setup sets the pin timer to 50 Hz and is only valid for 16 MHz clock
    TCCR1A = 0;  // Clear Timer1 control register A
    TCCR1B = 0;  // Clear Timer1 control register B
    TCNT1 = 0;   // Initialize counter

    ICR1 = 39999;  // Set TOP for 50 Hz (16 MHz / (8 * (39999 + 1))) 
    TCCR1A = (1 << WGM11);           // Fast PWM, mode 14 (ICR1 is TOP)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    TCCR1A |= (1 << COM1A1);         // Non-inverting mode on pin 9
}

void PayloadControl::contServoWrite(float rps)
{
    int us = 1000 + ((rps - (-maxSpd_)) * (2000 - 1000)) / (maxSpd_*2);
	if (us < 1000) us = 1000;
	if (us > 2000) us = 2000;

	uint32_t dutyCycle = (uint32_t) (us * 2); // converts microseconds to timer ticks (ICR1 = 400000 -> 20 ms / .5 us ticks)
	OCR1A = dutyCycle;
}
