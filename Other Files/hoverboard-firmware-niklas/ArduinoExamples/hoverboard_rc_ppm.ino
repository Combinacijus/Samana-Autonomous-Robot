/*
    Gintaras Grebliunas
    combinacijus@gmail.com

    CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
    CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
*/

#include <PPMReader.h>

// Channel indexs
#define CH_THROT 3
#define CH_YAW 4
#define CH_ROLL 1
#define CH_PITCH 2
#define CH_KNOB 7

#define BAUD_RATE 115200 // Same as in hoveboard firmware

#define DELAY 20        // Loop delay
#define PIN_INTERRUPT 2 // For RC PPM
#define CHANNEL_COUNT 7 // For RC PPM

#define COMMAND_START 12838 // arbitrary value signals that next 2 values will be steer and speed 011001000100110
#define COMMAND_END 3416    // arbitrary value signals that data ended                            000110101011000

PPMReader ppm(PIN_INTERRUPT, CHANNEL_COUNT);
int pitch, roll, knob; // Placeholders for channel values
float power_coef = 1;
int cmd_rnd = 0;

void setup()
{
    // Init serial communication
    Serial.begin(BAUD_RATE);
    while (!Serial)
        ;
}

void loop()
{
    // Read RC signal
    pitch = getCommand(CH_PITCH);
    roll = getCommand(CH_ROLL);
    knob = getCommand(CH_KNOB) + 1000; // [0 .. 2000]
    power_coef = knob / 2000.0;

    // Set motor speed and steering
    uint16_t steer = (uint16_t)(roll * power_coef);
    uint16_t speed = (uint16_t)(pitch * power_coef);

    // Send data via UART to hoverboard
    // Serial.println((int16_t)steer);
    // Serial.println((int16_t)speed);
    sendCommand(steer, speed);

    delay(DELAY);
}

/*
    @return val: pwm command value from RC channel between -1000 and 1000
*/
int getCommand(int channel)
{
    int value = ppm.latestValidChannelValue(channel, 0);
    if (value == 0)
        return 0;

    value = (value - 1500) * 2; // Convert to range [-1000 .. 1000]
    value = clamp(value, -1000, 1000);

    return value;
}

/*
  Clamps value between min and max
  @return val: clamped value
 */
double clamp(double val, double min, double max)
{
    if (val > max)
        val = max;
    if (val < min)
        val = min;

    return val;
}

/*
    Sends steer and speed command to the hoverboard
*/
void sendCommand(uint16_t _steer, uint16_t _speed)
{
    static uint16_t cmd_start = COMMAND_START;
    static uint16_t cmd_end = COMMAND_END;
    static uint16_t cmd_rnd = 0;

    // Inverting front direction
    _speed *= -1;

    cmd_rnd = 1000 + (cmd_rnd % 200); // Generate different value
    ++cmd_rnd;

    // Send data two time for redundancy
    for (int i = 0; i < 2; ++i)
    {
        Serial.write((uint8_t *)&cmd_start, sizeof(cmd_start));
        Serial.write((uint8_t *)&_steer, sizeof(_steer));
        Serial.write((uint8_t *)&_speed, sizeof(_speed));
        Serial.write((uint8_t *)&cmd_rnd, sizeof(cmd_rnd));
        Serial.write((uint8_t *)&cmd_end, sizeof(cmd_end));
    }
}