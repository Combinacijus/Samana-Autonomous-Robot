#include "Arduino.h"
#include "shift_register.h"


/*
	@param _bytes_count: number of bytes combined shift registers have
*/
ShiftReg::ShiftReg(int _bytes_count)
{
	bytes_count = _bytes_count;

	FastGPIO::Pin<PIN_DATA>::setOutputLow();
	FastGPIO::Pin<PIN_CLOCK>::setOutputLow();
	FastGPIO::Pin<PIN_LATCH>::setOutputLow();
	FastGPIO::Pin<PIN_MR>::setOutputHigh();
	FastGPIO::Pin<PIN_IN>::setInput();	// Should be pulled down with resistor ~10k
}

/*
	Shifts all zeros to the shift registers
*/
void ShiftReg::clear()
{
	write_all(0);
	// Master reset doesn't work for some reason
}

/*
	Writes one bit without latching it's faster for multi bit writes
	To actully do something set latch low, call this function and set latch low

	@param _bit: LOW or HIGH, 0 or 1
*/
void ShiftReg::write_bit(byte _bit)
{
	FastGPIO::Pin<PIN_CLOCK>::setOutputValueLow();

	if (_bit == 0)
		FastGPIO::Pin<PIN_DATA>::setOutputValueLow();
	else
		FastGPIO::Pin<PIN_DATA>::setOutputValueHigh();

	FastGPIO::Pin<PIN_CLOCK>::setOutputValueHigh();
}

/*
	Writes and latches one bit

	@param _bit: LOW or HIGH, 0 or 1
*/
void ShiftReg::write_single_bit(byte _bit)
{
	// Latch low starts saving data and latch high makes physical change
	FastGPIO::Pin<PIN_LATCH>::setOutputValueLow();
	write_bit(_bit);
	FastGPIO::Pin<PIN_LATCH>::setOutputValueHigh();
}

/*
	Writes and latches all bytes

	@param _data: (int) bits to write
	@param _reversed: (bool) if true bits are writen in reverse order
*/
void ShiftReg::write_all(int _data, bool _reversed)
{
	FastGPIO::Pin<PIN_LATCH>::setOutputValueLow(); // Ready to read data
	if (_reversed)
	{
		// Write in reverse first bit pushed first
		for (int i = 0; i < 8 * bytes_count; ++i) // in 00101 out 00101
			write_bit(0 != (_data & 1 << i));

		#ifdef DEBUG
			Serial.print("Write rev:  ");
			for (int i = 8 * bytes_count - 1; i >= 0; --i) // in 00101 out 10100
				Serial.print(0 != (_data & 1 << i));
			Serial.println(' ');
		#endif
	}
	else
	{
		// Write normal last bit pushed first
		for (int i = 8 * bytes_count - 1; i >= 0; --i) // in 00101 out 10100
			write_bit(0 != (_data & 1 << i));

		#ifdef DEBUG
			Serial.print("Write norm: ");
			for (int i = 0; i < 8 * bytes_count; ++i) // in 00101 out 00101
				Serial.print(0 != (_data & 1 << i));
			Serial.println(' ');
		#endif
	}

	FastGPIO::Pin<PIN_LATCH>::setOutputValueHigh(); // Latch data
}

/*
	Multiplexes through all bytes
	of shift register and reads values on PIN_IN

	@return: int value (in binary) with 1 if input high or 0 if low
*/
int ShiftReg::read_all()
{
	int input = 0;

	clear();
	for (int i = 0; i < 8 * bytes_count; ++i)
	{
		write_single_bit(i == 0); // First bit is 1 and shifts to the end
		delayMicroseconds(2); // Wait for register to update (if delay 1us register skips one read)
		input = input | (FastGPIO::Pin<PIN_IN>::isInputHigh() << i);
	}

	return input;
}

/*
	Waits for serial input and writes that number
*/
// void ShiftReg::write_from_serial()
// {
	//Serial.println("Input");
	//while (true)
	//	if (Serial.available())
	//	{
	//		num = Serial.readString().toInt();
	//		Serial.println("Num: " + String(num));
	//		break;
	//	}
	//write_all(num);
// }