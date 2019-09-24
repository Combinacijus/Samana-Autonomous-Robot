#include "Arduino.h"
#include "l298n.h"

/*
    Set all en and in pins also you change default direction of motors
*/
L298N::L298N(int ena, int enb, int in1, int in2, int in3, int in4, int dir1, int dir2)
    : pin_ena(ena), pin_enb(enb), pin_in1(in1), pin_in2(in2), pin_in3(in3), pin_in4(in4)
{
    dir1 >= 0 ? this->dir1 = 1 : this->dir1 = -1;
    dir2 >= 0 ? this->dir2 = 1 : this->dir2 = -1;

    pinMode(pin_ena, OUTPUT);
    pinMode(pin_enb, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_in3, OUTPUT);
    pinMode(pin_in4, OUTPUT);

    digitalWrite(pin_ena, LOW);
    digitalWrite(pin_enb, LOW);
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    digitalWrite(pin_in3, LOW);
    digitalWrite(pin_in4, LOW);
}

/*
    Run motor on OUT1 and OUT2 pins
    pwm: -255 - 255
    dir: -1, 1
*/
void L298N::run_motor1(int pwm, int dir)
{
    dir *= dir1; // Direction calibration

    if (pwm < 0) // Reverse direction if pwm is negative
        dir *= -1;
    pwm = clamp(abs(pwm), 0, 255);

    if (dir > 0)
    {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
    }
    else if (dir < 0)
    {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);

    }
    else // dir == 0
    {
        stop_motor1();
        return;
    }

    analogWrite(pin_ena, pwm);
}

/*
    Run motor on OUT3 and OUT4 pins
    pwm: -255 - 255
    dir: -1, 1
*/
void L298N::run_motor2(int pwm, int dir)
{
    dir *= dir2; // Direction calibration

    if (pwm < 0) // Reverse direction if pwm is negative
        dir *= -1;
    pwm = clamp(abs(pwm), 0, 255);

    if (dir > 0)
    {
        digitalWrite(pin_in3, HIGH);
        digitalWrite(pin_in4, LOW);
    }
    else if (dir < 0)
    {
        digitalWrite(pin_in3, LOW);
        digitalWrite(pin_in4, HIGH);
    }
    else // dir == 0
    {
        stop_motor2();
        return;
    }

    analogWrite(pin_enb, pwm);
}

/*
    Stop motor on OUT1 and OUT2 pins
*/
void L298N::stop_motor1()
{
    digitalWrite(pin_ena, LOW);
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
}

/*
    Stop motor on OUT3 and OUT4 pins
*/
void L298N::stop_motor2()
{
    digitalWrite(pin_enb, LOW);
    digitalWrite(pin_in3, LOW);
    digitalWrite(pin_in4, LOW);
}

float L298N::clamp(float x, float min, float max)
{
    x < min ? x = min : x;
    x > max ? x = max : x;

    return x;
}