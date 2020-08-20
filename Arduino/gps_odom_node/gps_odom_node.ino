/*
    Gintaras Grebliunas
	combinacijus@gmail.com

    GPS and Odometry ROS node

    Read odomentry data from STM32 via SPI

    Connections:
        [Arduino -- Logic level converter -- STM32]
            (SDA) A4 ---5V---3V3--- B7
            (SCL) A5 ---5V---3V3--- B6
            GND - GND
        [Arduino -- GPS module Neo6MV2]
            D4 - TX
            D3 - RX
            5V - VCC
            GND - GND

    digitalRead max speed 28RPS and >24RPS without skips with both encoders (good communication)
    ~18RPS without noticable skips with both encoders on I2C with 4bytes every 10ms = 400B/s

    [Before GPS]
    Sketch uses 12428 bytes (40%) && Global variables use 1208 bytes (58%) of dynamic memory
*/

#include <ros_gps_odom.h>
#include <samana_msgs/OdometrySmall.h>
#include <sensor_msgs/NavSatFix.h>
#include <Wire.h>
#include <NeoSWSerial.h>
#include <TinyGPS.h>

#define BAUD_RATE 115200
#define ODOM_SLAVE_ADDR 8          // NOTE: check if address is correct
#define MAIN_LOOP_PERIOD 10        // In ms
#define MIN_DELAY 2                // In ms
#define SS_BAUD 9600               // Softserial baud rate
#define PIN_SS_RX 4                // Softserial RX pin
#define PIN_SS_TX 3                // Softserial TX pin
#define IDEAL_MES_ERR 1.5          // Ideal measurement error for gps module (guessing)
#define FORCED_GPS_MSG_PERIOD 2000 // Period when even with no new data gps message would be pubslished in ms
#define DEBUG_SERIAL 0             // If true/1 disables ROS and prints debug data via serial port

ros::NodeHandle nh;
samana_msgs::OdometrySmall odometry_msg;
sensor_msgs::NavSatFix gps_msg;
ros::Publisher odometry_pub("odom_data", &odometry_msg);
ros::Publisher gps_pub("fix", &gps_msg);

NeoSWSerial ss(PIN_SS_RX, PIN_SS_TX);
TinyGPS gps;

void setup()
{
    if (DEBUG_SERIAL)
        Serial.begin(BAUD_RATE);
        
    Serial.println("Started");
    Wire.begin();
    ss.begin(SS_BAUD);
    
    if (!DEBUG_SERIAL)
        init_ros();
}

/*
    Inits node, advertises topics
*/
void init_ros()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();

    nh.advertise(odometry_pub);
    nh.advertise(gps_pub);

    while (!nh.connected()) // Wait until connected
    {
        nh.spinOnce();
    }

    // nh.setSpinTimeout(10000); // 10s timeout
    // nh.logwarn("Odom start");
}

void loop()
{
    static unsigned long loop_start_time = 0; // In ms

    handle_odometry_messages();
    handle_gps_messages();

    // Keeping same rate with variable delay
    int sleep_time = MAIN_LOOP_PERIOD - (millis() - loop_start_time);
    if (sleep_time < MIN_DELAY)
        sleep_time = MIN_DELAY;

    // char log_msg[20];
    // sprintf(log_msg, "S:%d", sleep_time);
    // nh.logwarn(log_msg);
    // nh.spinOnce();
    // Serial.println("d " + String(sleep_time));
    delay(sleep_time);
    loop_start_time = millis();
}

/*
    Reads odometry data form STM32
    And sends ROS message
*/
void handle_odometry_messages()
{
    static unsigned long last_time = 0; // In us
    static int16_t last_ticks1 = 0;
    static int16_t last_ticks2 = 0;
    static int16_t delta_ticks1;
    static int16_t delta_ticks2;
    static int16_t ticks1 = 0;
    static int16_t ticks2 = 0;
    // static int16_t k1 = 0;
    // static int16_t k2 = 0;
    static float rps1 = 0; // Rotations per second
    static float rps2 = 0; // Rotations per second
    static float dt = 0;   // Delta time in us
    static uint8_t rnd_byte = 0; // Should be different byte every read for health check
    static uint8_t rnd_byte_prev = 0;

    // Read ticks from STM32
    Wire.requestFrom(ODOM_SLAVE_ADDR, 4);
    if (Wire.available())
    {
        ticks1 = Wire.read();
        ticks1 += Wire.read() << 8;
        ticks2 = Wire.read();
        ticks2 += Wire.read() << 8;
    }
    else  // No bytes recieved don't send to ROS
    {
        if (DEBUG_SERIAL)
            Serial.println("Odom Failed");

        last_time = micros();
        return;
    }

    // Rotations per second. delta_ticks / (ticks_per_rotation * delta_time_s)
    // Overflow handles itself: delta_ticks is correct on overflow
    delta_ticks1 = ticks1 - last_ticks1;
    delta_ticks2 = ticks2 - last_ticks2;
    dt = micros() - last_time;
    last_time = micros();
    rps1 = delta_ticks1 / (0.0024 * dt);
    rps2 = delta_ticks2 / (0.0024 * dt);

    // Count how many times counter overflowed
    // if (delta_ticks1 > 0 && ticks1 < last_ticks1)
    //     ++k1;
    // else if (delta_ticks1 < 0 && ticks1 > last_ticks1)
    //     --k1;
    // if (delta_ticks2 > 0 && ticks2 < last_ticks2)
    //     ++k2;
    // else if (delta_ticks2 < 0 && ticks2 > last_ticks2)
    //     --k2;

    last_ticks1 = ticks1;
    last_ticks2 = ticks2;

    // Output odom message to ROS
    odometry_msg.header.stamp = nh.now();
    odometry_msg.header.frame_id = "odom";
    odometry_msg.delta_ticks1 = delta_ticks1;
    odometry_msg.delta_ticks2 = -delta_ticks2;
    odometry_msg.rps1 = rps1;
    odometry_msg.rps2 = -rps2;
    odometry_msg.dt = dt;

    if (!DEBUG_SERIAL)
    {
        odometry_pub.publish(&odometry_msg);
        nh.spinOnce();
    }
    
    // Serial debug
    if (DEBUG_SERIAL)
        Serial.print(String(ticks1) + " " + String(ticks2) + " ");
    // Serial.println(String(rps1) + " " + String(rps2));
    // Serial.println(String(k1) + "|" + String(k2));
}

/*
    Reads gps data with soft serial if available
    And sends ROS message
*/
void handle_gps_messages()
{
    static float lat = 0, lon = 0;
    static float last_lat = 0, last_lon = 0;
    static unsigned long fix_age = gps.GPS_INVALID_FIX_TIME, last_msg_time = 0;

    // Update gps serial data
    while (ss.available())
        gps.encode(ss.read());

    // Decode gps data
    gps.f_get_position(&lat, &lon, &fix_age);

    // If data have changed or some time have passed send ROS message
    if (last_lat != lat || last_lon != lon || millis() - last_msg_time > FORCED_GPS_MSG_PERIOD)
    {
        last_lat = lat;
        last_lon = lon;
        last_msg_time = millis();

        unsigned long hdop = gps.hdop();

        //  Debug
        if (DEBUG_SERIAL)
        {
            Serial.print(gps.satellites());
            Serial.print("  ");
            Serial.println(gps.hdop());
            Serial.print(lat, 12);
            Serial.print("  ");
            Serial.println(lon, 12);
        }

        // Setup and send ROS message
        gps_msg.header.stamp = nh.now();
        gps_msg.header.frame_id = "gps";

        if (fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > 2500 || millis() - last_msg_time > FORCED_GPS_MSG_PERIOD)
            gps_msg.status.status = gps_msg.status.STATUS_NO_FIX;
        else
            gps_msg.status.status = gps_msg.status.STATUS_FIX;

        gps_msg.status.service = gps_msg.status.SERVICE_GPS;
        gps_msg.latitude = lat;
        gps_msg.longitude = lon;
        gps_msg.altitude = gps.f_altitude();

        // Fix for rviz because lat = 1000 is invalid for a plugin
        if (millis() - last_msg_time > FORCED_GPS_MSG_PERIOD && gps_msg.status.status == gps_msg.status.STATUS_NO_FIX)
        {
            gps_msg.latitude = 0;
            gps_msg.longitude = 0;
        }

        // Aproximating covariance matrix from hdop
        float var = IDEAL_MES_ERR * hdop; // Standard deviation
        var *= var;                       // Convert to variance

        // Diagonal of covariance matrix
        gps_msg.position_covariance[0] = var;
        gps_msg.position_covariance[4] = var;
        gps_msg.position_covariance[8] = var;

        gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_APPROXIMATED;

        if (!DEBUG_SERIAL)
            gps_pub.publish(&gps_msg);
    }

    nh.spinOnce();
}
