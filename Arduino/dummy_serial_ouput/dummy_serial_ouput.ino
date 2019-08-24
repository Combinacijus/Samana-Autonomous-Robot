/*
  Emulates serial debug data for development 
*/

char uart_buf[100];
int mult = 1;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
}

void loop()
{
  int ch_buf[10];

  for (int i = 0; i < 10; ++i)
  {
    ch_buf[i] = (i+2) * mult;
  }

  mult += 50;
  mult = (mult % 5000) + 1;
  
  sprintf(uart_buf, "<1:%i><2:%i><3:%i><4:%i><5:%i><6:%i><7:%i><8:%i>\r\n", ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);
  Serial.print(uart_buf);
  delay(100);
}
