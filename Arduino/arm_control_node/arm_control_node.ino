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

      ENA - D4
      ENB - D5
      IN1 - D6
      IN2 - D7
      IN3 - D8
      IN4 - D9
*/

#include "current_sensor.h"

#define SERIAL_BAUD 115200
#define PIN_CUR_SENS_1 A0
#define PIN_CUR_SENS_2 A1

CurrentSensor ampmeter1(PIN_CUR_SENS_1, -1, 2.3, 512, -21);
CurrentSensor ampmeter2(PIN_CUR_SENS_2, -1, 2.3, 512, -105);

void setup()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;
  Serial.println("START");
}

void loop()
{
  ampmeter1.update();
  ampmeter2.update();
  Serial.print(ampmeter1.get_current());
  Serial.print(" ");
  Serial.println(ampmeter2.get_current());

  delay(1);
}