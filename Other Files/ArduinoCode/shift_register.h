#pragma once

#include "globals.h"

/*
<--- Notch left
\/ - usually low,  /\ - usually high, MR - Master Reset, QE - Output Enable
 1    2   3    4    5     6    7     8
 VCC/\ Q0 DATA OE\/ LATCH CLOCK MR/\  Q7'
 Q1   Q2  Q3   Q4   Q5    Q6   Q7   GND\/
VCC --- 10uF Capacitor --- GND
2nd Register: 1.Q7' to 2.DATA. Linked: VCC QE LATCH CLOCK MR
*/

class ShiftReg
{
public:
	ShiftReg();
	void update();
	void write_bit(byte _bit);
	void write_byte(byte _data, bool _reversed = false);
	int read_byte(int _bytes_num = 1);

private:
	const int PIN_DATA = 26;
	const int PIN_CLOCK = 25;
	const int PIN_LATCH = 32;
	const int PIN_MR = 35;

	int PIN_BTN_IN = 35;
};
