#pragma once

/*
    Library for converting current sensor data to current in miliamps
    Calibration with constructor:
        Set _mid to sensor value when no current
        Set _dir to -1 or 1 if you get negative current
        _calib used for calibrating output by scaling it(1.5 would mean output multiplied by 1.5)
    How to use:
        Periodically call update() function to read and update sensor values
        current: public value of the current
*/
class CurrentSensor
{
private:
    const float SMOOTHING = 0.25; // For complimentary filter (lower - more smoothing)
    const float VCC = 5.0;        // Voltage of Vcc pin

    int pin_input;    // Input pin name
    int value;        // Raw value read from sensor
    int mid_val;      // Value of 0 current
    int dir;          // Direction of possitive current
    float calib_mult; // Calibration multiplyer if sensor (if on by 100% set calib_mult to 2)

public:
    float current = 0; // Current in miliamperes

    CurrentSensor(int _pin, int _dir = -1, float _calib = 2.3, int _mid = 512);
    void update();
};