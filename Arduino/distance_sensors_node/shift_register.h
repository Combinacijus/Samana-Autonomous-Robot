/*
	74HC595N shift register library
	Pin values are hardcoded with #define for FastGPIO to work
	Pin values can be change to any pins

	\/ - usually low,  /\ - usually high, MR - Master Reset, QE - Output Enable, --- - Wire

	<--- Notch left
	1    2   3    4    5     6    7     8
	VCC/\ Q0 DATA QE\/ LATCH CLOCK MR/\  Q7'
	----------------------------------------
	)                                      |
	----------------------------------------
	Q1   Q2   Q3   Q4   Q5    Q6   Q7   GND\/

	Also connect: VCC --- 10uF Capacitor --- GND

	Linking 2nd Register: 1.Q7' --- 2.DATA. Linked: VCC QE LATCH CLOCK MR GND
*/

#pragma once

#include <FastGPIO.h>

// NOTE: pin values
#define PIN_DATA A0
#define PIN_LATCH A1
#define PIN_CLOCK A2
#define PIN_MR A3	// Master reset pin
#define PIN_IN 13	// Input pin (multiplex for all sensors)
// #define DEBUG // Uncomment for debugging

class ShiftReg
{
public:
	ShiftReg(int _bytes_count = 1);
	void clear();
	void write_bit(byte _bit);
	void write_single_bit(byte _bit);
	void write_all(int _data, bool _reversed = false);
	int read_all();
	// void write_from_serial();

	uint8_t bytes_count = 1; // Number of shift register
};
