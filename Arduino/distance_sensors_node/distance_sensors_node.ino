/*
    Gintaras Grebliunas
    Combinacijus@gmail.com
    ROS node which handles many ultrasonic sensors

    Connections:
    VCC-5V  GND-GND  TRIG-PIN_TRIG_ALL  ECHO-PIN_ECHO_X
    10k Pulldown resistor to every ECHO pin
    Single Arduino Nano should be able to handle 10 sensors no problem

    Times:
    digitalRead()                                 -  4us
    FastGPIO::Pin<IO_D1>::setOutputValueHigh()    -  0.16us

    FastGPIO requires knowledge on pin numbers in compile time
    so we have a lot of copy paste code

    Use find to read all 'NOTE:' comments
*/

#include <FastGPIO.h>
#include <TimerOne.h>
#include <ros.h>
#include <samana_msgs/Sonar.h>
#include <std_msgs/UInt16MultiArray.h>

#define BAUD_RATE 1000000
#define SENORS_COUNT 4  // NOTE: Number of sensors

/*
    Speed of sound = (331.3 + 0.606 * temp) m/s
    At 40 deg ~ 355 m/s, -10 ~ 337 m/s
*/
#define MAX_SOUND_SP 355
#define MIN_SOUND_SP 337

/*
    In milliseconds. Should be long enough to not catch previuos echo
    At lowest speed in 50ms max distance 8.4m, 40ms - 6.7m
 */
#define TRIGGER_PERIOD 40

/*
    ISR_PERIOD: In microseconds. Should be long enough to execute
    pin readouts and leave time for other code(bare minimum 6us per pin)
    60us might introduce up to 1cm measurement error
*/
#define ISR_PERIOD 50
#define STOP -1            // Pin state when we don't watch pulse state
#define UNDEF_TIME 1234567 // Undefined dtX time in microseconds
#define UNDEF_DIST -1      // Undefined distance

// NOTE: Single trig pin triggers all sensors
#define PIN_TRIG_ALL 4

// NOTE: Define echo pins for every sensor
#define PIN_ECHO_1 5
#define PIN_ECHO_2 6
#define PIN_ECHO_3 7
#define PIN_ECHO_4 8

volatile unsigned long dt[SENORS_COUNT]; // Elapsed times in HIGH state
volatile unsigned long tt;

int state_echo[SENORS_COUNT]; // Old echo pin states

// ROS variables
ros::NodeHandle nh;
samana_msgs::Sonar sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);
int16_t distances[SENORS_COUNT];
int8_t temperature = 20; // Assuming temperature


void setup()
{
    for (int i = 0; i < SENORS_COUNT; ++i)
        state_echo[i] = STOP;
    
    FastGPIO::Pin<PIN_TRIG_ALL>::setOutput(LOW);

    // NOTE: Set echo pins to INPUT mode
    FastGPIO::Pin<PIN_ECHO_1>::setInput();
    FastGPIO::Pin<PIN_ECHO_2>::setInput();
    FastGPIO::Pin<PIN_ECHO_3>::setInput();
    FastGPIO::Pin<PIN_ECHO_4>::setInput();

    // Serial.begin(230400);
    // while (!Serial)
    //     ;

    // ROS serial node setup
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(sonar_pub);
    while (!nh.connected())
        nh.spinOnce();
    sonar_msg.dist_length = SENORS_COUNT;
    sonar_msg.header.frame_id = "base_link";

    // Init timer interupt
    Timer1.initialize(ISR_PERIOD);
    Timer1.attachInterrupt(multiPulseIn);
}

void loop()
{
    // NOTE: Reset dtX values to UNDEF_TIME so it's easy to spot bad data
    for (int i = 0; i < SENORS_COUNT; ++i)
        dt[i] = UNDEF_TIME;

    // Generates 10us trigger for all sensors
    FastGPIO::Pin<PIN_TRIG_ALL>::setOutput(HIGH);
    delayMicroseconds(10);
    FastGPIO::Pin<PIN_TRIG_ALL>::setOutput(LOW);

    // Set echo pin states LOW for all sensors
    for (int i = 0; i < SENORS_COUNT; ++i)
        state_echo[i] = LOW;
    // Now timer interrupt will act like pulseIn() for all echo pins

    delay(TRIGGER_PERIOD);
    // After delay data is collected and can be used

    for (int i = 0; i < SENORS_COUNT; ++i)
        distances[i] = getDistance(dt[i]);

    // NOTE: Publish data to ros (WARNING: requires custom message for more sensors)
    sonar_msg.header.stamp = nh.now();
    sonar_msg.dist = distances;
    sonar_pub.publish(&sonar_msg);
    nh.spinOnce();
}

/*
    Interrupt service routine as non-blocking pulseIn for multiple pins
    Runs on timer interrupt and checks for pins changes
    Tracks how long pin was in HIGH state
*/
void multiPulseIn()
{
    //    tt = micros();
    // TODO: REDUCE REPETITION
    /*
        NOTE: Make this block for every sensor change X to a number
        if (FastGPIO::Pin<PIN_ECHO_X>::isInputHigh() && state_echo_X == LOW) // Rising
        {
            state_echo_X = HIGH;
            dtX = micros();
        }
        else if(!FastGPIO::Pin<PIN_ECHO_X>::isInputHigh() && state_echo_X == HIGH) // Falling
        {
            state_echo_X = LOW;
            dtX = micros() - dtX;
        }
    */

    if (state_echo[1] == LOW && FastGPIO::Pin<PIN_ECHO_1>::isInputHigh()) // Rising
    {
        // High state started record time
        state_echo[1] = HIGH;
        dt[1] = micros();
    }
    else if (state_echo[1] == HIGH &&!FastGPIO::Pin<PIN_ECHO_1>::isInputHigh()) // Falling
    {
        // High state ended don't change dt until next loop
        state_echo[1] = STOP;
        dt[1] = micros() - dt[1];
    }

    if (state_echo[2] == LOW && FastGPIO::Pin<PIN_ECHO_2>::isInputHigh()) // Rising
    {
        state_echo[2] = HIGH;
        dt[2] = micros();
    }
    else if (state_echo[2] == HIGH && !FastGPIO::Pin<PIN_ECHO_2>::isInputHigh()) // Falling
    {
        state_echo[2] = STOP;
        dt[2] = micros() - dt[2];
    }

    if (state_echo[3] == LOW && FastGPIO::Pin<PIN_ECHO_3>::isInputHigh()) // Rising
    {
        state_echo[3] = HIGH;
        dt[3] = micros();
    }
    else if (state_echo[3] == HIGH && !FastGPIO::Pin<PIN_ECHO_3>::isInputHigh()) // Falling
    {
        state_echo[3] = STOP;
        dt[3] = micros() - dt[3];
    }

    if (state_echo[4] == LOW && FastGPIO::Pin<PIN_ECHO_4>::isInputHigh()) // Rising
    {
        state_echo[4] = HIGH;
        dt[4] = micros();
    }
    else if (state_echo[4] == HIGH && !FastGPIO::Pin<PIN_ECHO_4>::isInputHigh()) // Falling
    {
        state_echo[4] = STOP;
        dt[4] = micros() - dt[4];
    }

    // tt = micros() - tt;
}

/*
    Returns distanc (mm) given time (us)
    TODO: add temperature to equation

    @param delta_time: elapsed time between pulse and echo in microseconds (us)
    @return: distance in mm or -1 if data is invalid
*/
int getDistance(long delta_time)
{
    /*
        Check validity of trigger time
        if 
    */
    // if (delta_time >= TRIGGER_PERIOD * 1000 ||
    //     delta_time <= 80)
    //     return UNDEF_DIST;

    // Speed of sound = (331.3 + 0.606 * temp) m/s
    return (delta_time * (331.3 + 0.606 * temperature)) / 2000.0;
}