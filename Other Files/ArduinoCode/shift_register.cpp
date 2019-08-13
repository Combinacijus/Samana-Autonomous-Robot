#include "Arduino.h"
#include "shift_register.h"

void stop()
{
	while (!Serial.available())
		delay(10);
	Serial.read();
}

ShiftReg::ShiftReg()
{
	Serial.println("Shift register init");
	pinMode(PIN_DATA, OUTPUT);
	pinMode(PIN_CLOCK, OUTPUT);
	pinMode(PIN_LATCH, OUTPUT);

	pinMode(PIN_BTN_IN, INPUT_PULLDOWN);
}


void ShiftReg::update()
{
	//Serial.println("Input");
	//while (true)
	//	if (Serial.available())
	//	{
	//		num = Serial.readString().toInt();
	//		Serial.println("Num: " + String(num));
	//		break;
	//	}

	//write_byte(num);

	int input = read_byte(2);
	func::print_bin(input, 2);
	//Serial.printf("%s\n", func::get_bin(input, 2));
}

void ShiftReg::write_bit(byte _bit)
{
	//digitalWrite(PIN_LATCH, LOW);
	digitalWrite(PIN_CLOCK, LOW);
	digitalWrite(PIN_DATA, _bit);
	digitalWrite(PIN_CLOCK, HIGH);
	//digitalWrite(PIN_LATCH, HIGH);
}

void ShiftReg::write_byte(byte _data, bool _reversed)
{
	digitalWrite(PIN_LATCH, LOW); // Ready to read data
	if (_reversed)
	{
		//Serial.print("REV BIN: ");
		//Serial.println(_data, BIN);
		for (int i = 7; i >= 0; --i) // in 00101 out 10100
		{
			//Serial.print(0 != (_data & 1 << i));
			write_bit(0 != (_data & 1 << i));
		}
	}
	else
	{
		//Serial.print("BIN: ");
		//Serial.println(_data, BIN);
		for (int i = 0; i < 8; ++i) // in 00101 out 00101
		{
			//Serial.print(0 != (_data & 1 << i));
			write_bit(0 != (_data & 1 << i));
		}
	}
	//Serial.println(' ');

	digitalWrite(PIN_LATCH, HIGH); // Latch data
}

int ShiftReg::read_byte(int _bytes_num)
{
	int input = 0;
	for (int i = 0; i < _bytes_num; ++i)
		write_byte(0); // Crude register clearing TODO with MR pin
	// TODO
	//digitalWrite(PIN_MR, LOW);
	//digitalWrite(PIN_MR, HIGH);

	for (int i = 0; i < 8 * _bytes_num; ++i)
	{
		digitalWrite(PIN_LATCH, LOW);
		write_bit(i == 0); // First bit is 1 and shifts to the end
		digitalWrite(PIN_LATCH, HIGH);
		delayMicroseconds(1); // Wait for register to update
		input = input | (digitalRead(PIN_BTN_IN) << i);
	}

	return input;
}