/*
	Gintaras Grebliunas
	combinacijus@gmail.com

	Odometry ROS node
  
	For pins check interrupt function ISR(). Input pins need internal pullup

	Connections:
		Encoder1: A(GREEN) - D8, B(WHITE) - D9, GND(BLACK) - GND, VCC(RED) - 5V
		Encoder2: A(GREEN) - D6, B(WHITE) - D7, GND(BLACK) - GND, VCC(RED) - 5V

	It uses pin change interrupts on ports B and D and hardcoded pins.
	Clockwise direction is positive.
	Max rotations per second 38 but above ~20rps it starts to skip steps
	also no skip is not guaranteed.
	(Tested with only main loop printinh speeds and counts every 150ms)

	TODO: to handle counter overflow (mess up speed)
*/

#include <ros_gps_odom.h>
#include <samana_msgs/OdometrySmall.h>


ros::NodeHandle nh;
samana_msgs::OdometrySmall odometry_msg;
ros::Publisher odometry_pub("odom", &odometry_msg);
const int BAUD_RATE = 115200;

int pin_a = 6;
int pin_b = 7;
unsigned long last_update_time = 0;

volatile int ticks1 = 0;
volatile int ticks2 = 0;

const float ticks_to_rot = 1000000.0 / 2400.0;

void setup()
{
	// Enable pullup for all pins
	DDRB = 0b00000000;   // Input
	DDRC = 0b00000000;   // Input
	DDRD = 0b00000000;   // Input
	PORTB |= 0b11111111; // Pullup
	PORTC |= 0b11111111; // Pullup
	PORTD |= 0b11111100; // Pullup leave last bits because it's TX and RX

	// Serial.begin(115200);
	// Serial.print("Start");

	encoderSetup();

	initROS();
}

void loop()
{
	static unsigned long last_time = 0;
	static unsigned long dt = -1;
	static unsigned int last_ticks1 = 0;
	static unsigned int last_ticks2 = 0;
	static long dc1 = 0;
	static long dc2 = 0;
	static float speed1 = 0;
	static float speed2 = 0;

	//  Serial.print(digitalRead(pin_a));
	//	Serial.println(value);

	dt = micros() - last_time;
	last_time = micros();
	dc1 = ticks1 - last_ticks1;
	last_ticks1 = ticks1;
	dc2 = ticks2 - last_ticks2;
	last_ticks2 = ticks2;

	speed1 = dc1 * ticks_to_rot / dt;
	speed2 = dc2 * ticks_to_rot / dt;

	// Serial.println("A" + String(speed1));
	// Serial.println("B" + String(speed2));
	// Serial.println("A" + String((long)last_count1));
	// Serial.println("B" + String((long)last_count2));
	// Serial.println("");

	odometry_msg.ticks1 = last_ticks1;
	odometry_msg.ticks2 = last_ticks2;
	odometry_msg.speed1 = speed1;
	odometry_msg.speed2 = speed2;
	odometry_pub.publish(&odometry_msg);
	nh.spinOnce();

	delay(100);
}

void encoderSetup()
{
	cli();
	// NOTE: hardcoded
	// Pin change interrupt register for ports
	PCICR |= 0b00000001; // turn on port b  13-8
	PCICR |= 0b00000100; // turn on port d  7-0(2)
	//	PCICR |= 0b00000010;    // turn on port c  A5-A0

	// Pin change mask for pins of the port
	PCMSK0 |= 0b00000011; // 13-8
	PCMSK2 |= 0b11000000; // 7-0(2)
	// PCMSK1 |= 0b11111111; // A5-A0
	sei();
}

// PortB interrupt. Max speed - 38 rotations per second
ISR(PCINT0_vect)
{
	/*
		CW rotation 013201320
		Decision matrix:
		               Y            X
		decisions[last_state][current_state] =
		.|  0| 1| 2| 3| X
		--------------
		0|  0| 1|-1| *|
		1| -1| 0| *| 1|
		2|  1| *| 0|-1|
		3|  *|-1| 1| 0|
		Y

		* means impossible transition for implementation should be 0
	*/

	static byte curr_state = 0b00000000;
	static byte last_state = 0b00000000;
	static const int decision_matrix[4][4] = {
		{0, 1, -1, 0},
		{-1, 0, 0, 1},
		{1, 0, 0, -1},
		{0, -1, 1, 0}};

	last_state = curr_state;
	curr_state = (PINB & 0b0000011); // Read D9 and D8. NOTE: hardcoded
	// Serial.println(curr_state);
	ticks1 += decision_matrix[last_state][curr_state];
}

// PortD interrupt. Max speed - 38 rotations per second
ISR(PCINT2_vect)
{
	/*
		CW rotation 013201320
		Decision matrix:
		               Y            X
		decisions[last_state][current_state] =
		.|  0| 1| 2| 3| X
		--------------
		0|  0| 1|-1| *|
		1| -1| 0| *| 1|
		2|  1| *| 0|-1|
		3|  *|-1| 1| 0|
		Y

		* means impossible transition for implementation should be 0
	*/
	static byte curr_state = 0b00000000;
	static byte last_state = 0b00000000;
	static const int decision_matrix[4][4] = {
		{0, 1, -1, 0},
		{-1, 0, 0, 1},
		{1, 0, 0, -1},
		{0, -1, 1, 0}};

	last_state = curr_state;
	curr_state = (PIND & 0b11000000) >> 6; // Read D7 and D6 and push to LSB. NOTE: hardcoded
	// Serial.println(curr_state);
	ticks2 += decision_matrix[last_state][curr_state];
}

/*
    Inits node, advertises topics
*/
void initROS()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();

    nh.advertise(odometry_pub);

    while (!nh.connected()) // Wait until connected
    {
        nh.spinOnce();
    }

    // Debuging parameters. Brake if not enough dynamic memory
    // sprintf(log_msg, "%d", recalibrate);
    // nh.logwarn(log_msg);
}