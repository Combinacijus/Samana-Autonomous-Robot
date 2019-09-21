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
*/

#include "current_sensor.h"

#define SERIAL_BAUD 115200
#define PIN_CUR_SENS_1 A0
#define PIN_CUR_SENS_2 A1

CurrentSensor current1(PIN_CUR_SENS_1, -1, 2.3);
CurrentSensor current2(PIN_CUR_SENS_2, -1, 2.3);

void setup()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;
  Serial.println("START");
}

void loop()
{
  current1.update();
  Serial.println(current1.current);

  delay(1);
}