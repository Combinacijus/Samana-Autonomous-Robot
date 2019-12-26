/*
    Gintaras Grebliunas
    combinacijus@gmail.com

    Library for converting current sensor data to current in miliamps
    Calibration with constructor:
        Set _mid to sensor value when no current
        Set _dir to -1 or 1 if you get negative current
        _calib used for calibrating output by scaling it(1.5 would mean output multiplied by 1.5)
        _offset to offset value in mA
    How to use:
        Periodically call update() function to read and update sensor values
        current: public value of the current (which can be computed with calibrate())
*/

#pragma once

class CurrentSensor
{
private:
    const float SMOOTHING = 0.25; // For complimentary filter (lower - more smoothing)
    const float VCC = 5.0;        // Voltage of Vcc pin

    int pin_input;    // Input pin name
    int value;        // Raw value read from sensor
    int mid_val;      // Value of 0 current
    int offset;       // Offset to add to current in mA
    int dir;          // Direction of possitive current
    float calib_mult; // Calibration multiplyer if sensor (if on by 100% set calib_mult to 2)
    float current = 0; // Current in miliamperes

    float clamp(float x, float min, float max);

public:

    CurrentSensor(int _pin, int _dir = -1, float _calib = 2.3, int _mid = 512, int _offset = 0);
    void calibrate();
    void update();
    float get_current();
};