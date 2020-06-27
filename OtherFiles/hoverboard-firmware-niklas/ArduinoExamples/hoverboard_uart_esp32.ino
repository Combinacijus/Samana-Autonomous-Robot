/*
  ESP32 serial passthrough 
  Receiving data from hoverboard UART3 and
  sending to the computer via USB
*/

#define BAUD_RATE 115200 // Same as in hoveboard firmware
#define RXD2 16          // TX2 pin to hoverboard RX pin
#define TXD2 17          // RX2 pin to hoverboard TX pin

void setup()
{
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{
  if (Serial.available())
  {                               // If anything comes in Serial (USB),
    Serial2.write(Serial.read()); // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial2.available())
  {                               // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(Serial2.read()); // read it and send it out Serial (USB)
  }
}
