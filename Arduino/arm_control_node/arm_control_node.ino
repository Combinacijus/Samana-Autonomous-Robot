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
      OUT1 & OUT4 - to motors

      ENA - D8 (NON-PWM) Be carefull with pwm pins because TimerOne disables some
      IN1 - D9
      IN2 - D10

      ENB - D5 (PWM)
      IN3 - D7
      IN4 - D6

    Limit Switches (PULLED UP normally HIHG):
      A2 - Lifter up
      A3 - Lifter down
      A4 - Grabber open
      A5 - Grabber closed

    Read all NOTE: for more info
    NOTE: read arm_abstraction.* files
*/

#include "current_sensor.h"
#include "arm_abstraction.h"
#include <TimerOne.h>

// #define DEBUG
#define INTERRUPT_PERIOD 10000 // Timer interrupt period in micro seconds
#define MAIN_LOOP_PERIOD 50    // In miliseconds
#define SERIAL_BAUD 115200

// NOTE: pins
#define PIN_CUR_SENS_1 A0
#define PIN_CUR_SENS_2 A1

// Arm control states
#define GRABBER_STOP 0
#define GRABBER_OPEN 1
#define GRABBER_CLOSE 2
#define LIFTER_STOP 0
#define LIFTER_RAISE 1
#define LIFTER_LOWER 2

// NOTE: Stall current depends on motor voltage
#define OVERCURRENT_GRABBER 1500 // In mA
#define OVERCURRENT_LIFTER 1900  // In mA
#define OVERCURRENT_TIMEOUT 300  // In ms
#define UNDEF -1 // Undefined overcurrent time

int grabber_action = 0; // Tells which action grabber is doing
int lifter_action = 0;  // Tells which action lifter is doing
unsigned long overcurrent_grabber_time = UNDEF;
unsigned long overcurrent_lifter_time = UNDEF;

// NOTE: curent sensors calibration
CurrentSensor ampmeter_grabber(PIN_CUR_SENS_2, 1, 2.3, 512);
CurrentSensor ampmeter_lifter(PIN_CUR_SENS_1, -1, 2.3, 512);

/*
  Periodically called funtion by timer1
  Updates current reading
*/
void timer_interrupt()
{
  ampmeter_lifter.update();
  ampmeter_grabber.update();
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    ;
  Serial.println("START");

  // Set limit swithches input pins
  pinMode(PIN_GO, INPUT_PULLUP);
  pinMode(PIN_GC, INPUT_PULLUP);
  pinMode(PIN_LU, INPUT_PULLUP);
  pinMode(PIN_LD, INPUT_PULLUP);

  // Calibrate current sesors
  l298n.stop_motor1();
  l298n.stop_motor2();
  delay(250);
  ampmeter_lifter.calibrate();
  ampmeter_grabber.calibrate();

  lifter_up();

  // Timer interrupt for updating current reading
  Timer1.initialize(INTERRUPT_PERIOD);
  Timer1.attachInterrupt(timer_interrupt);
}

void loop()
{
  // Current sensor updates in timer interrupt routine

  // Control arm via serial port
  if (Serial.available())
  {
    char c = Serial.read() - '0';
    if (c >= 0 && c <= 2)
      lifter_action = c;
    if (c >= 5 && c <= 8)
      grabber_action = c - 5;
  }
  // Serial.println("G:" + String(grabber_action) + " L:" + String(lifter_action));

  
  overcurrent_protection();
  arm_actions_controller();

  Serial.println("Amp: " + String(ampmeter_grabber.get_current()) + " " + String(ampmeter_lifter.get_current()));
  // Serial.println("Amp: " + String(ampmeter_lifter.get_current()));
#ifdef DEBUG

  Serial.println("GO: " + String(is_grabber_opened()));
  Serial.println("GC: " + String(is_grabber_closed()));
  Serial.println("GM: " + String(is_grabber_moving()));
  Serial.println("LU: " + String(is_lifter_up()));
  Serial.println("LD: " + String(is_lifter_down()));
  Serial.println("LM: " + String(is_lifter_moving()));
  Serial.println("------");
#endif

  delay(MAIN_LOOP_PERIOD);
}

/*
  Overcurrent/Stall protection. Disables lifter or grabber motor if in stall for some time
*/
void overcurrent_protection()
{
  // Grabber
  if (abs(ampmeter_grabber.get_current()) > OVERCURRENT_GRABBER)
  {
    if (overcurrent_grabber_time == UNDEF)
      overcurrent_grabber_time = millis();

    // Stop grabber when overcurrent time is exceeded
    if (millis() - overcurrent_grabber_time > OVERCURRENT_TIMEOUT)
      grabber_action = GRABBER_STOP;
  }
  else
  {
    overcurrent_grabber_time = UNDEF;
  }

  // Lifter
  if (abs(ampmeter_lifter.get_current()) > OVERCURRENT_LIFTER)
  {
    if (overcurrent_lifter_time == UNDEF)
      overcurrent_lifter_time = millis();

    // Stop lifter when overcurrent time is exceeded
    if (millis() - overcurrent_lifter_time > OVERCURRENT_TIMEOUT)
      lifter_action = LIFTER_STOP;
  }
  else
  {
    overcurrent_lifter_time = UNDEF;
  }
}

/*
  Given grabber or lifter action decides what to do
  Spins motor to specified position until limit switch is reached
*/
void arm_actions_controller()
{
  switch (grabber_action)
  {
  case GRABBER_STOP:
    grabber_stop();
    break;

  case GRABBER_OPEN:
    grabber_open();
    if (is_grabber_opened())
      grabber_action = GRABBER_STOP;
    break;

  case GRABBER_CLOSE:
    grabber_close();
    if (is_grabber_closed())
      grabber_action = GRABBER_STOP;
    break;

  default:
    grabber_stop();
  }

  switch (lifter_action)
  {
  case LIFTER_STOP:
    lifter_stop();
    break;

  case LIFTER_RAISE:
    lifter_up();
    if (is_lifter_up())
      lifter_action = LIFTER_STOP;
    break;

  case LIFTER_LOWER:
    lifter_down();
    if (is_lifter_down())
      lifter_action = LIFTER_STOP;
    break;

  default:
    lifter_stop();
  }
}