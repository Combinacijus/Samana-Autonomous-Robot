#include "Arduino.h"
#include "arm_abstraction.h"

L298N l298n(PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, 1, -1);

//-----------------------GRABBER-------------------------
void grabber_open()
{
  l298n.run_motor2(1);
}

void grabber_close()
{
  l298n.run_motor2(-1);
}

void grabber_stop()
{
  l298n.stop_motor2();
}

bool is_grabber_opened()
{
  return !digitalRead(PIN_GO);
}

bool is_grabber_closed()
{
  return !digitalRead(PIN_GC);
}

bool is_grabber_moving()
{
  return !is_grabber_closed() && !is_grabber_opened();
}

//------------------------LIFTER---------------------------

void lifter_up()
{
  l298n.run_motor1(-1);
}

void lifter_down()
{
  l298n.run_motor1(1);
}

void lifter_stop()
{
  l298n.stop_motor1();
}

bool is_lifter_up()
{
  return !digitalRead(PIN_LU);
}

bool is_lifter_down()
{
  return !digitalRead(PIN_LD);
}

bool is_lifter_moving()
{
  return !is_lifter_down() && !is_lifter_up();
}