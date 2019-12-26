/* 
  Gintaras Grebliunas
  combinacijus@gmail.com
  Node for arm mechanism motor control with current measurements

  Connections:
    Current sensors:
      GND - A_GND
      VCC - A_5V
      1.OUT - A0
      2.OUT - A1

    H-bridge:
      Jumper next to +12V and GND removed! For seperate 5V logic supply
      +12V - 12V?
      GND - A_GND & BAT_GND
      +5V - A_5V

      OUT2 & OUT3 - through current sensor to motors
      OUT1 & OUT$ - to motors

      ENA - D5 (PWM) Be carefull with pwm pins because TimerOne disables some
      ENB - D6 (PWM)
      IN1 - D7
      IN2 - D8
      IN3 - D9
      IN4 - D10

    Read all NOTE: for more info
*/

#include "current_sensor.h"
#include "l298n.h"
#include <TimerOne.h>

#define SERIAL_BAUD 115200

// NOTE: pins
#define PIN_CUR_SENS_1 A0
#define PIN_CUR_SENS_2 A1
#define PIN_ENA 5
#define PIN_ENB 6
#define PIN_IN1 7
#define PIN_IN2 8
#define PIN_IN3 9
#define PIN_IN4 10

#define INTERRUPT_PERIOD 3000 // Timer interrupt period in micro seconds

// NOTE: curent sensors calibration
CurrentSensor ampmeter1(PIN_CUR_SENS_1, -1, 2.3, 512);
CurrentSensor ampmeter2(PIN_CUR_SENS_2, 1, 2.3, 512);

L298N l298n(PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, 1, -1);

/*
  Periodically called funtion by timer1
  Updates current reading
*/
void timer_interrupt()
{
  // Update current reading
  ampmeter1.update();
  ampmeter2.update();

  // Serial.print(ampmeter1.get_current());
  // Serial.print(" ");
  // Serial.println(ampmeter2.get_current());
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;
  Serial.println("START");

  // Calibrate current sesors
  l298n.stop_motor1();
  l298n.stop_motor2();
  delay(100);
  ampmeter1.calibrate();
  ampmeter2.calibrate();

  // Timer interrupt for updateing current reading
  Timer1.initialize(INTERRUPT_PERIOD);
  Timer1.attachInterrupt(timer_interrupt);
}

void loop()
{
  // Current sensor updates in timer interrupt routine

  // TODO: Connect it to computer or to master Arduino
  // TODO: Current limiting?

  Serial.print(ampmeter1.get_current());
  Serial.print(" ");
  Serial.println(ampmeter2.get_current());

  l298n.run_motor1(255);
  l298n.run_motor2(255);
  delay(200);
  l298n.stop_motor1();
  l298n.stop_motor2();
  delay(500);

  delay(1);
}