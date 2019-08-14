# 1 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino"
# 1 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino"
/*
    Gintaras Grebliunas
    Combinacijus@gmail.com
    ROS node which handles many ultrasonic sensors

    Connections:
    VCC-5V  GND-GND  TRIG-PIN_TRIG_ALL  ECHO-PIN_ECHO_X
    10k Pulldown resistor to every ECHO pin (else random values)

    Single Arduino Nano should be able to handle 10 sensors no problem

    Times:
    digitalRead()                                 -  4us
    FastGPIO::Pin<IO_D1>::setOutputValueHigh()    -  0.16us

    FastGPIO requires knowledge on pin numbers in compile time
    so we have a lot of copy paste code

    Use find to read all 'NOTE:' comments
*/

# 23 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino" 2
# 24 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino" 2
# 25 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino" 2
# 26 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino" 2
# 27 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino" 2




/*
    Speed of sound = (331.3 + 0.606 * temp) m/s
    At 40 deg ~ 355 m/s, -10 ~ 337 m/s
*/



/*
    In milliseconds. Should be long enough to not catch previuos echo
    At lowest speed in 50ms max distance 8.4m, 40ms - 6.7m
 */


/*
    NOTE:
    ISR_PERIOD: In microseconds. Should be long enough to execute
    pin readouts and leave time for other code(bare minimum 6us per pin)
    60us might introduce up to 1cm measurement error
*/





// NOTE: Single trig pin triggers all sensors


// NOTE: Define echo pins for every sensor
# 70 "/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/distance_sensors_node/distance_sensors_node.ino"
volatile unsigned long dt[10 /* NOTE: Number of sensors*/]; // Elapsed times in HIGH state
volatile unsigned long tt;

int state_echo[10 /* NOTE: Number of sensors*/]; // Old echo pin states

// ROS variables
ros::NodeHandle nh;
samana_msgs::Sonar sonar_msg;
ros::Publisher sonar_pub("sonar", &sonar_msg);
int16_t distances[10 /* NOTE: Number of sensors*/];
int8_t temperature = 20; // Assuming temperature

void setup()
{
    for (int i = 0; i < 10 /* NOTE: Number of sensors*/; ++i)
        state_echo[i] = -1 /* Pin state when we don't watch pulse state*/;

    FastGPIO::Pin<2>::setOutput(0x0);

    // NOTE: Set echo pins to INPUT mode
    FastGPIO::Pin<3>::setInput();
    FastGPIO::Pin<4>::setInput();
    FastGPIO::Pin<5>::setInput();
    FastGPIO::Pin<6>::setInput();
    FastGPIO::Pin<7>::setInput();
    FastGPIO::Pin<8>::setInput();
    FastGPIO::Pin<9>::setInput();
    FastGPIO::Pin<10>::setInput();
    FastGPIO::Pin<11>::setInput();
    FastGPIO::Pin<12>::setInput();

    // Serial.begin(230400);
    // while (!Serial)
    //     ;

    // ROS serial node setup
    nh.getHardware()->setBaud(57600 /* 1M looses sync*/);
    nh.initNode();
    nh.advertise(sonar_pub);
    while (!nh.connected())
        nh.spinOnce();
    sonar_msg.dist_length = 10 /* NOTE: Number of sensors*/;
    sonar_msg.header.frame_id = "base_link";

    // Init timer interupt. MAIN LOOP IS AN INTERRUPT multiPulseIn
    Timer1.initialize(60 /* In us*/);
    Timer1.attachInterrupt(multiPulseIn);
}

void loop()
{
    // NOTE: Reset dtX values to UNDEF_TIME so it's easy to spot bad data
    for (int i = 0; i < 10 /* NOTE: Number of sensors*/; ++i)
        dt[i] = 1234567 /* Undefined dtX time in microseconds*/;

    // Generates 10us trigger for all sensors
    FastGPIO::Pin<2>::setOutput(0x1);
    delayMicroseconds(10);
    FastGPIO::Pin<2>::setOutput(0x0);

    // Set echo pin states LOW for all sensors
    for (int i = 0; i < 10 /* NOTE: Number of sensors*/; ++i)
        state_echo[i] = 0x0;
    // Now timer interrupt will act like pulseIn() for all echo pins

    delay(40 /* In ms*/);
    // After delay data is collected and can be used

    for (int i = 0; i < 10 /* NOTE: Number of sensors*/; ++i)
        distances[i] = getDistance(dt[i]);

    // NOTE: Publish data to ros (WARNING: requires custom message for more sensors)
    sonar_msg.header.stamp = nh.now();
    sonar_msg.dist = distances;
    sonar_pub.publish(&sonar_msg);
    nh.spinOnce();
}

/*
    Interrupt service routine as non-blocking pulseIn for multiple pins
    Runs on timer interrupt and checks for pins changes
    Tracks how long pin was in HIGH state
*/
void multiPulseIn()
{
    tt = micros();
    /*
        TODO: remove
        NOTE: Make this block for every sensor change X to a number
        if (FastGPIO::Pin<PIN_ECHO_X>::isInputHigh() && state_echo_X == LOW) // Rising
        {
            state_echo_X = HIGH;
            dtX = micros();
        }
        else if(!FastGPIO::Pin<PIN_ECHO_X>::isInputHigh() && state_echo_X == HIGH) // Falling
        {
            state_echo_X = LOW;
            dtX = micros() - dtX;
        }
    */

    for (int i = 0; i < 10 /* NOTE: Number of sensors*/; ++i)
    {
        if (state_echo[i] == 0x0 && pinIsHigh(i))
        {
            // High state started record time
            state_echo[i] = 0x1;
            dt[i] = micros();
        }
        else if (state_echo[i] == 0x1 && !pinIsHigh(i)) // Falling
        {
            // High state ended don't change dt until next loop
            state_echo[i] = -1 /* Pin state when we don't watch pulse state*/;
            dt[i] = micros() - dt[i];
        }
    }

    // if (state_echo[1] == LOW && FastGPIO::Pin<PIN_ECHO_1>::isInputHigh()) // Rising
    // {
    //     // High state started record time
    //     state_echo[1] = HIGH;
    //     dt[1] = micros();
    // }
    // else if (state_echo[1] == HIGH && !FastGPIO::Pin<PIN_ECHO_1>::isInputHigh()) // Falling
    // {
    //     // High state ended don't change dt until next loop
    //     state_echo[1] = STOP;
    //     dt[1] = micros() - dt[1];
    // }

    // if (state_echo[2] == LOW && FastGPIO::Pin<PIN_ECHO_2>::isInputHigh()) // Rising
    // {
    //     state_echo[2] = HIGH;
    //     dt[2] = micros();
    // }
    // else if (state_echo[2] == HIGH && !FastGPIO::Pin<PIN_ECHO_2>::isInputHigh()) // Falling
    // {
    //     state_echo[2] = STOP;
    //     dt[2] = micros() - dt[2];
    // }

    // if (state_echo[3] == LOW && FastGPIO::Pin<PIN_ECHO_3>::isInputHigh()) // Rising
    // {
    //     state_echo[3] = HIGH;
    //     dt[3] = micros();
    // }
    // else if (state_echo[3] == HIGH && !FastGPIO::Pin<PIN_ECHO_3>::isInputHigh()) // Falling
    // {
    //     state_echo[3] = STOP;
    //     dt[3] = micros() - dt[3];
    // }

    // if (state_echo[4] == LOW && FastGPIO::Pin<PIN_ECHO_4>::isInputHigh()) // Rising
    // {
    //     state_echo[4] = HIGH;
    //     dt[4] = micros();
    // }
    // else if (state_echo[4] == HIGH && !FastGPIO::Pin<PIN_ECHO_4>::isInputHigh()) // Falling
    // {
    //     state_echo[4] = STOP;
    //     dt[4] = micros() - dt[4];
    // }

    tt = micros() - tt;
    // char log_msg[20];
    // sprintf(log_msg, "%d", tt);
    // nh.loginfo(log_msg);
    // nh.spinOnce();
}

/*
    Returns if pins is high using fastGPIO library and predefined pin values
    pinIsHigh(0)  ->  return FastGPIO::Pin<PIN_ECHO_1>::isInputHigh(); etc.
*/
bool pinIsHigh(int ind)
{
    // NOTE: Add cases for every sensor
    switch (ind)
    {
    case 0:
        return FastGPIO::Pin<3>::isInputHigh();
    case 1:
        return FastGPIO::Pin<4>::isInputHigh();
    case 2:
        return FastGPIO::Pin<5>::isInputHigh();
    case 3:
        return FastGPIO::Pin<6>::isInputHigh();
    case 4:
        return FastGPIO::Pin<7>::isInputHigh();
    case 5:
        return FastGPIO::Pin<8>::isInputHigh();
    case 6:
        return FastGPIO::Pin<9>::isInputHigh();
    case 7:
        return FastGPIO::Pin<10>::isInputHigh();
    case 8:
        return FastGPIO::Pin<11>::isInputHigh();
    case 9:
        return FastGPIO::Pin<12>::isInputHigh();

    default:
        // Index not in range
        return 0;
        break;
    }
}

/*
    Returns distanc (mm) given time (us)
    TODO: add temperature to equation

    @param delta_time: elapsed time between pulse and echo in microseconds (us)
    @return: distance in mm or -1 if data is invalid
*/
int getDistance(long delta_time)
{
    /*
        Check validity of trigger time
        if 
    */
    // if (delta_time >= TRIGGER_PERIOD * 1000 ||
    //     delta_time <= 80)
        // return UNDEF_DIST;

    // Speed of sound = (331.3 + 0.606 * temp) m/s
    return (delta_time * (331.3 + 0.606 * temperature)) / 2000.0;
}
