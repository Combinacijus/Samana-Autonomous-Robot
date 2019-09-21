#include "Arduino.h"
#include "current_sensor.h"

/*
    For variables description read the header file
*/
CurrentSensor::CurrentSensor(int _pin, int _dir, float _calib, int _mid)
{
    pin_input = _pin;
    dir = _dir;
    calib_mult = _calib;
    mid_val = _mid;
}

/*
    Reads value from current sensor
    Calculates amp draw and smooths it
*/
void CurrentSensor::update()
{
    value = analogRead(pin_input) - mid_val;
    float cur_now = map(value, 0, 1024 * dir, 0, 10000 * calib_mult);
    current = SMOOTHING * cur_now + (1 - SMOOTHING) * current;
}

float clamp(float x, float min, float max)
{
  x < min ? x = min : x;
  x > max ? x = max : x;

  return x;
}
