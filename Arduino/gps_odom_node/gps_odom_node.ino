/*
    Gintaras Grebliunas
	combinacijus@gmail.com

    GPS and Odometry ROS node

    Read odomentry data from STM32 via SPI

    Connections (Arduino - Logic level converter - STM32):
        (SDA) A4 ---5V---3V3--- B7
        (SCL) A5 ---5V---3V3--- 67
        GND - GND

    digitalRead max speed 28RPS and >24RPS without skips with both encoders (good communication)
    ~18RPS without noticable skips with both encoders on I2C with 4bytes every 10ms = 400B/s
*/

#include <ros_gps_odom.h>
#include <Wire.h>

#define BAUD_RATE 115200
#define ODOM_SLAVE_ADDR 8   // NOTE: check if address correct
#define MAIN_LOOP_PERIOD 50 // In ms
#define MIN_DELAY 2         // In ms

void setup()
{
    Serial.begin(BAUD_RATE);
    Wire.begin();
}

void loop()
{
    static unsigned long last_time = 0;
    static unsigned long loop_start_time = 0;
    static int16_t last_ticks1 = 0;
    static int16_t last_ticks2 = 0;
    static int16_t ticks1 = 0;
    static int16_t ticks2 = 0;
    static int16_t k1 = 0;
    static int16_t k2 = 0;
    // Rotations per second
    static float rps1 = 0;
    static float rps2 = 0;

    // Read ticks from STM32
    Wire.requestFrom(ODOM_SLAVE_ADDR, 4);
    ticks1 = Wire.read();
    ticks1 += Wire.read() << 8;
    ticks2 = Wire.read();
    ticks2 += Wire.read() << 8;

    // Rotations per second. delta_ticks / (ticks_per_rotation * delta_time)
    // Overflow handles itself: delta_ticks is correct on overflow
    int16_t delta_ticks1 = ticks1 - last_ticks1;
    int16_t delta_ticks2 = ticks2 - last_ticks2;
    rps1 = delta_ticks1 / (2.4 * (millis() - last_time));
    rps2 = delta_ticks2 / (2.4 * (millis() - last_time));

    // Count how many times counter overflowed
    if (delta_ticks1 > 0 && ticks1 < last_ticks1)
        ++k1;
    else if (delta_ticks1 < 0 && ticks1 > last_ticks1)
        --k1;
    if (delta_ticks2 > 0 && ticks2 < last_ticks2)
        ++k2;
    else if (delta_ticks2 < 0 && ticks2 > last_ticks2)
        --k2;

    last_ticks1 = ticks1;
    last_ticks2 = ticks2;
    last_time = millis();

    Serial.print(String(ticks1) + " " + String(ticks2) + " ");
    Serial.println(String(rps1) + " " + String(rps2));
    Serial.println(String(k1) + "|" + String(k2));

    // Keeping same rate with variable delay
    int sleep_time = max(MIN_DELAY, MAIN_LOOP_PERIOD - (millis() - loop_start_time));
    Serial.println("Sleep time: " + String(sleep_time));
    delay(sleep_time);
    loop_start_time = millis();
}