volatile int state = LOW;

void setup()
{
  pinMode(PA0, INPUT);
  pinMode(PC13, OUTPUT);
    attachInterrupt(PA0, blink, CHANGE);
}

void loop()
{
    digitalWrite(PC13, state);
}

void blink() 
{
    if (state == HIGH) 
    {
        state = LOW;
    } 
    else 
    { // state must be LOW
        state = HIGH;
    }
}
