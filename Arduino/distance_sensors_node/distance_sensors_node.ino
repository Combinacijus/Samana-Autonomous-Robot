/*
    Gintaras Grebliunas
    Combinacijus@gmail.com
    ROS node which handles many ultrasonic and bump sensors

    Connections ultrasonic:
        VCC-5V  GND-GND  TRIG-PIN_TRIG_ALL  ECHO-PIN_ECHO_X
        10k (used 6k8) Pulldown resistor to every ECHO pin (else random values)
        A row of pins used from D2 to D12

    Single Arduino Nano should be able to handle 10 distance sensors no problem

    Connections shift register:
        See shift_register.h defined values
        (Might be) A0-A3, D13

    Times:
    digitalRead()                                 -  4us
    FastGPIO::Pin<IO_D1>::setOutputValueHigh()    -  0.16us

    FastGPIO requires knowledge on pin numbers in compile time
    so we have a lot of copy paste code

    TODO: read current temperature from ROS topic

    Use find to read all 'NOTE:' comments
*/

#include <FastGPIO.h>
#include <TimerOne.h>
// ros_sonar.h -> typedef NodeHandle_<ArduinoHardware, 3, 3, 40, 150> NodeHandle;
#include <ros_sonar.h> // Special ros.h for this library
#include <samana_msgs/Int16Array.h>
#include <samana_msgs/Bump.h>
#include "shift_register.h"

#define BAUD_RATE 57600 // Even at 2.5k looses sync
#define SENORS_COUNT 10 // NOTE: Number of sensors

/*
    Speed of sound = (331.3 + 0.606 * temp) m/s
    At 40 deg ~ 355 m/s, -10 ~ 337 m/s
*/
#define MIN_SOUND_SP 337
#define MAX_SOUND_SP 355
#define MIN_DIST 20 // Minimum distance ultrasonic can measure

/*
    In milliseconds. Should be long enough to not catch previuos echo
    At lowest speed in 50ms max distance 8.4m, 40ms - 6.7m
 */
#define TRIGGER_PERIOD 40 // In ms

/*
    NOTE:
    ISR_PERIOD: In microseconds. Should be long enough to execute
    pin readouts and leave time for other code(bare minimum 6us per pin)
    60us might introduce up to 1cm measurement error
    Measured: 10 sensors = ~40us
*/
#define ISR_PERIOD 60      // In us
#define STOP -1            // Pin state when we don't watch pulse state
#define UNDEF_TIME 1234567 // Undefined dtX time in microseconds
#define UNDEF_DIST -1      // Undefined distance

// NOTE: Single trig pin triggers all sensors
#define PIN_TRIG_ALL 2

// NOTE: Define echo pins for every sensor
#define PIN_ECHO_1 3
#define PIN_ECHO_2 4
#define PIN_ECHO_3 5
#define PIN_ECHO_4 6
#define PIN_ECHO_5 7
#define PIN_ECHO_6 8
#define PIN_ECHO_7 9
#define PIN_ECHO_8 10
#define PIN_ECHO_9 11
#define PIN_ECHO_10 12

volatile unsigned long dt[SENORS_COUNT]; // Elapsed times in HIGH state
volatile unsigned long tt;

int state_echo[SENORS_COUNT]; // Old echo pin states

// ROS variables
ros::NodeHandle nh;
// Distance sensors
samana_msgs::Int16Array sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);
int16_t distances[SENORS_COUNT];
int8_t temperature = 20; // Assuming temperature
// Bump sensors
samana_msgs::Bump bump_msg;
ros::Publisher bump_pub("bump", &bump_msg);
ShiftReg shift_reg(2);    // NOTE: change pins in library header defines
int16_t bump_sensor_bits; // Each bit means if sensor is triggered or not

void setup()
{
    for (int i = 0; i < SENORS_COUNT; ++i)
        state_echo[i] = STOP;

    FastGPIO::Pin<PIN_TRIG_ALL>::setOutputLow();

    // NOTE: Set echo pins to INPUT mode
    FastGPIO::Pin<PIN_ECHO_1>::setInput();
    FastGPIO::Pin<PIN_ECHO_2>::setInput();
    FastGPIO::Pin<PIN_ECHO_3>::setInput();
    FastGPIO::Pin<PIN_ECHO_4>::setInput();
    FastGPIO::Pin<PIN_ECHO_5>::setInput();
    FastGPIO::Pin<PIN_ECHO_6>::setInput();
    FastGPIO::Pin<PIN_ECHO_7>::setInput();
    FastGPIO::Pin<PIN_ECHO_8>::setInput();
    FastGPIO::Pin<PIN_ECHO_9>::setInput();
    FastGPIO::Pin<PIN_ECHO_10>::setInput();

    // ROS serial node setup
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.advertise(sonar_pub);
    nh.advertise(bump_pub);
    while (!nh.connected())
        nh.spinOnce();
    sonar_msg.data_length = SENORS_COUNT;
    sonar_msg.header.frame_id = "base_link";
    bump_msg.header.frame_id = "base_link";

    // Init timer interupt. MAIN LOOP IS AN INTERRUPT multiPulseIn
    Timer1.initialize(ISR_PERIOD);
    Timer1.attachInterrupt(multiPulseIn);

    // Serial.begin(BAUD_RATE);
    // while (!Serial)
    //     ;
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

    // Delay for TRIGGER_PERIOD meanwhile doing bump sensor updates
    unsigned long long tmp_time = millis();
    while (millis() - tmp_time < TRIGGER_PERIOD - 1)
    {
        bump_sensor_update();
    }

    // After delay data is already collected by multiPulseIn and can be used

    for (int i = 0; i < SENORS_COUNT; ++i)
        distances[i] = getDistance(dt[i]);

    // NOTE: Publish data to ros (WARNING: requires custom message for more sensors)
    sonar_msg.header.stamp = nh.now();
    sonar_msg.data = distances;
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
    // tt = micros();

    for (int i = 0; i < SENORS_COUNT; ++i)
    {
        if (state_echo[i] == LOW && pinIsHigh(i))
        {
            // High state started record time
            state_echo[i] = HIGH;
            dt[i] = micros();
        }
        else if (state_echo[i] == HIGH && !pinIsHigh(i)) // Falling
        {
            // High state ended don't change dt until next loop
            state_echo[i] = STOP;
            dt[i] = micros() - dt[i];
        }
    }

    // tt = micros() - tt;
    // char log_msg[20];
    // sprintf(log_msg, "%d", tt);
    // nh.loginfo(log_msg);
    // nh.spinOnce();
}

/*
    Returns if pins is high using fastGPIO library and predefined pin values
    pinIsHigh(0)  ->  return FastGPIO::Pin<PIN_ECHO_1>::isInputHigh(); etc.
*/
bool pinIsHigh(int ind)
{
    // NOTE: Add cases for every sensor
    switch (ind)
    {
    case 0:
        return FastGPIO::Pin<PIN_ECHO_1>::isInputHigh();
    case 1:
        return FastGPIO::Pin<PIN_ECHO_2>::isInputHigh();
    case 2:
        return FastGPIO::Pin<PIN_ECHO_3>::isInputHigh();
    case 3:
        return FastGPIO::Pin<PIN_ECHO_4>::isInputHigh();
    case 4:
        return FastGPIO::Pin<PIN_ECHO_5>::isInputHigh();
    case 5:
        return FastGPIO::Pin<PIN_ECHO_6>::isInputHigh();
    case 6:
        return FastGPIO::Pin<PIN_ECHO_7>::isInputHigh();
    case 7:
        return FastGPIO::Pin<PIN_ECHO_8>::isInputHigh();
    case 8:
        return FastGPIO::Pin<PIN_ECHO_9>::isInputHigh();
    case 9:
        return FastGPIO::Pin<PIN_ECHO_10>::isInputHigh();

    default: // Index not in range
        return 0;
    }
}

/*
    Returns distanc (mm) given time (us)

    @param delta_time: elapsed time between pulse and echo in microseconds (us)
    @return: distance in mm or -1 if data is invalid
*/
int getDistance(long delta_time)
{
    // Check validity of trigger time

    // Speed of sound = (331.3 + 0.606 * temp) m/s
    int speed_of_sound = 331.3 + 0.606 * temperature;
    int dist = (delta_time * speed_of_sound) / 2000.0; // In mm

    int max_dist = (TRIGGER_PERIOD * speed_of_sound) / 2.0; // In mm
    if (dist >= max_dist || dist < MIN_DIST)
        return UNDEF_DIST;

    return dist;
}

/*
    Reads bump sensor data by multiplexing with shift register
    and after reading sets outputs as inputs to light up leds
    Takes about 400us
*/
void bump_sensor_update()
{
    shift_reg.clear();
    bump_sensor_bits = shift_reg.read_all();
    shift_reg.write_all(bump_sensor_bits);

    // Ros publishing
    bump_msg.header.stamp = nh.now();
    bump_msg.bump_bits = bump_sensor_bits;
    bump_pub.publish(&bump_msg);
    nh.spinOnce();
}