/*
    Gintaras Grebliunas
	combinacijus@gmail.com

    Code for STM32F103C8 to read optical encoder position
    and send it to master arduino on request

    Connections:
		Encoder1: A(GREEN) - A0, B(WHITE) - A1, GND(BLACK) - GND, VCC(RED) - 5V
		Encoder2: A(GREEN) - A2, B(WHITE) - A3, GND(BLACK) - GND, VCC(RED) - 5V
    
        (Arduino - Logic level converter - STM32):
        (SDA) A4 ---5V---3V3--- B7
        (SCL) A5 ---5V---3V3--- 67
        GND - GND

    digitalRead max speed 28RPS and >24RPS without skips with both encoders (good communication)
    ~18RPS without noticable skips with both encoders on I2C with 4bytes every 10ms = 400B/s
*/

#include <Wire_slave.h>

#define PIN_A1 PA0
#define PIN_B1 PA1
#define PIN_A2 PA2
#define PIN_B2 PA3
#define BAUD_RATE 115200
#define ODOM_SLAVE_ADDR 8 // NOTE: must be same as in master

/*
    CW rotation 013201320
    Decision matrix:
                        Y            X
    decision_matrix[last_state][current_state] =
    .|  0| 1| 2| 3| X
    --------------
    0|  0| 1|-1| *|
    1| -1| 0| *| 1|
    2|  1| *| 0|-1|s
    3|  *|-1| 1| 0|
    Y

    * means impossible transition for implementation should be 0
*/
const int decision_matrix[4][4] = {
    {0, 1, -1, 0},
    {-1, 0, 0, 1},
    {1, 0, 0, -1},
    {0, -1, 1, 0}};
volatile int16_t state_1_curr = 0;
volatile int16_t state_1_last = 0;
volatile int16_t ticks1 = 0;
volatile int16_t state_2_curr = 0;
volatile int16_t state_2_last = 0;
volatile int16_t ticks2 = 0;

void setup()
{
    pinMode(PIN_A1, INPUT_PULLUP);
    pinMode(PIN_B1, INPUT_PULLUP);
    pinMode(PIN_A2, INPUT_PULLUP);
    pinMode(PIN_B2, INPUT_PULLUP);

    // Serial.begin(BAUD_RATE);
    // while (!Serial)
    //     ;

    Wire.begin(ODOM_SLAVE_ADDR);
    Wire.onRequest(request_event);

    attachInterrupt(PIN_A1, a1_change_int, CHANGE);
    attachInterrupt(PIN_B1, b1_change_int, CHANGE);
    attachInterrupt(PIN_A2, a2_change_int, CHANGE);
    attachInterrupt(PIN_B2, b2_change_int, CHANGE);
}

// void loop(){} // Nothing to do

/* 
    When master Arduino requests data
    sends tick counts from both encoders
    During this function some ticks might be skipped
*/
void request_event()
{
    // Send ticks 16bit x2
    Wire.write(ticks1);
    Wire.write(ticks1 >> 8);
    Wire.write(ticks2);
    Wire.write(ticks2 >> 8);
    Serial.println(String(ticks1) + " | " + String(ticks2));
}

void a1_change_int()
{
    if (digitalRead(PIN_A1))
    {
        state_1_curr |= 1;
    }
    else
    {
        state_1_curr &= ~1;
    }

    ticks1 += decision_matrix[state_1_last][state_1_curr];
    state_1_last = state_1_curr;
}

void b1_change_int()
{
    if (digitalRead(PIN_B1))
    {
        state_1_curr |= 2;
    }
    else
    {
        state_1_curr &= ~2;
    }

    ticks1 += decision_matrix[state_1_last][state_1_curr];
    state_1_last = state_1_curr;
}

void a2_change_int()
{
    if (digitalRead(PIN_A2))
    {
        state_2_curr |= 1;
    }
    else
    {
        state_2_curr &= ~1;
    }

    ticks2 += decision_matrix[state_2_last][state_2_curr];
    state_2_last = state_2_curr;
}

void b2_change_int()
{
    if (digitalRead(PIN_B2))
    {
        state_2_curr |= 2;
    }
    else
    {
        state_2_curr &= ~2;
    }

    ticks2 += decision_matrix[state_2_last][state_2_curr];
    state_2_last = state_2_curr;
}
