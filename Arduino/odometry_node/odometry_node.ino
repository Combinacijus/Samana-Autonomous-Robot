/*
  Gintaras Grebliunas
  combinacijus@gmail.com

  Odometry ROS node
  CW
  A 11001100
  B 01100110
  + 13201320
  A+2B
*/

int pin_a = 6;
int pin_b = 7;
unsigned long last_update_time = 0;

int counter = 0;
byte curr_state = 0b00000000;
byte last_state = 0b00000000;

volatile int value = 0;

void setup()
{
	Serial.begin(115200);
	//   delay(10);
	Serial.print("Start");

	encoder_setup();
}

void loop()
{
	static int c = 0;
	//  Serial.print(digitalRead(pin_a));
	//	Serial.println(value);
	Serial.println(counter);
	delay(150);
}

void encoder_setup()
{
	pinMode(pin_a, INPUT_PULLUP);
	pinMode(pin_b, INPUT_PULLUP);

	cli();
	// Pin change interrupt register for ports
	PCICR |= 0b00000001; // turn on port b  13-8
						 //	PCICR |= 0b00000010;    // turn on port c  A5-A0
	PCICR |= 0b00000100; // turn on port d  7-0(2)

	// Pin change mask for pins of the port
	PCMSK0 |= 0b11111111; // 13-8
	PCMSK1 |= 0b11111111; // A5-A0
	PCMSK2 |= 0b11111100; // 7-0(2)
	sei();
}

// PortB interrupt
ISR(PCINT0_vect)
{
}

// PortD interrupt
ISR(PCINT2_vect)
{
	//	CW
	//  A 11001100
	//  B 01100110
	//  + 13201320
	// D6 and D7 PIND
	// 0 64 192 128 0 64 192 128

	last_state = curr_state;
	curr_state = PIND & 0b11000000; // D7 and D6

	if (last_state == 0)
	{
		if (curr_state == 64)
			++counter;
		else if (curr_state == 128)
			--counter;
	}

	if (last_state == 64)
	{
		if (curr_state == 192)
			++counter;
		else if (curr_state == 0)
			--counter;
	}

	if (last_state == 192)
	{
		if (curr_state == 128)
			++counter;
		else if (curr_state == 64)
			--counter;
	}

	if (last_state == 128)
	{
		if (curr_state == 0)
			++counter;
		else if (curr_state == 192)
			--counter;
	}

	// if (last_state ^ curr_state < 0b00000011)
	// {
	// 	// Serial.println("g");
	// }
	// else
	// {
	// 	// Serial.println("bad");
	// 	// Serial.println("a" + String(a));
	// 	// Serial.println("b" + String(b));
	// 	// Serial.println("ls" + String(last_state));
	// }
	// Serial.println("cs" + String(curr_state));
}

// PortC interrupt
// ISR(PCINT1_vect)
// {
//  Serial.print("C");
//  Serial.println(PINC);
// }
