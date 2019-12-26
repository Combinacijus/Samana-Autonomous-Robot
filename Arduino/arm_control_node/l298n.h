/*
    Gintaras Grebliunas
    combinacijus@gmail.com
    Library for driving L298N H-Bridge for motor control with pwm

    NOTE: Is pin is not PWM capable analogWrite(pwm) is eqivalent to digitalWrite(1) if pwm > 128
*/

#pragma once

class L298N
{
private:
    const int pin_ena;
    const int pin_enb;
    const int pin_in1;
    const int pin_in2;
    const int pin_in3;
    const int pin_in4;
    int dir1; // To reverse direction set it to 1 or -1
    int dir2; // To reverse direction set it to 1 or -1

    float clamp(float x, float min, float max);

public:
    L298N(int ena, int enb, int in1, int in2, int in3, int in4, int dir1 = 1, int dir2 = 1);
    void run_motor_pwm1(int pwm, int dir = 1);
    void run_motor1(int dir = 1);
    void run_motor_pwm2(int pwm, int dir = 1);
    void run_motor2(int dir = 1);
    void stop_motor1();
    void stop_motor2();
};