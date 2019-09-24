#include "Arduino.h"
#include "current_sensor.h"

/*
    For variables description read the header file
*/
CurrentSensor::CurrentSensor(int _pin, int _dir, float _calib, int _mid, int _offset = 0)
{
    pin_input = _pin;
    dir = _dir;
    calib_mult = _calib;
    mid_val = _mid;
    offset = _offset;
}

/*
    Reads current values for some time and sets average to 0
*/
void CurrentSensor::calibrate()
{
    long counter = 0;
    double cur_now = 0;

    for (int i = 0; i < 200; ++i)
    {
        value = analogRead(pin_input) - mid_val;
        cur_now += map(value, 0, 1024 * dir, 0, 10000 * calib_mult) + offset;
        counter++;
        delayMicroseconds(250);
    }

    cur_now /= counter; // Average current
    Serial.println("Ofdset delta: " + String(cur_now));
    offset -= cur_now;  // Update offset
    Serial.println("Ofdset: " + String(offset));
}

/*
    Reads value from current sensor
    Calculates amp draw and smooths it
*/
void CurrentSensor::update()
{
    value = analogRead(pin_input) - mid_val;
    float cur_now = map(value, 0, 1024 * dir, 0, 10000 * calib_mult) + offset;
    current = SMOOTHING * cur_now + (1 - SMOOTHING) * current;
}

/*
    Getter for current in miliamps
    Don't forget to use update() before this
*/
float CurrentSensor::get_current()
{
    return current;
}

float CurrentSensor::clamp(float x, float min, float max)
{
    x < min ? x = min : x;
    x > max ? x = max : x;

    return x;
}