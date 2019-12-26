#pragma once
#include "l298n.h"

#define PIN_ENA 8
#define PIN_ENB 5
#define PIN_IN1 9
#define PIN_IN2 10
#define PIN_IN3 7
#define PIN_IN4 6

#define PIN_LU A2  // Lifter up
#define PIN_LD A3  // Lifter down
#define PIN_GO A4  // Grabber open
#define PIN_GC A5  // Grabber closed

extern L298N l298n;

//-----------------------GRABBER-------------------------
void grabber_open();
void grabber_close();
void grabber_stop();
bool is_grabber_opened();
bool is_grabber_closed();
bool is_grabber_moving();

//------------------------LIFTER---------------------------
void lifter_up();
void lifter_down();
void lifter_stop();
bool is_lifter_up();
bool is_lifter_down();
bool is_lifter_moving();