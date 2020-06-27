/*
    Gintaras Grebliunas
    combinacijus@gmail.com
*/

#define DELAY 20            // Loop delay
#define BAUD_RATE 115200    // Same as in hoveboard firmware
#define COMMAND_START 12838 // Arbitrary value signals that next 2 values will be steer and speed 011001000100110
#define COMMAND_END 3416    // Arbitrary value signals that data ended                            000110101011000

void setup()
{
    // Init serial communication
    Serial.begin(BAUD_RATE);
    while (!Serial)
        ;
}

void loop()
{
    // Set motor speed and steering
    uint16_t steer = 70;
    uint16_t speed = 100;

    // Send data via UART to hoverboard
    sendCommand(steer, speed);

    delay(DELAY);
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