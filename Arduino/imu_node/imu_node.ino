/*
    Gintaras Grebliunas
    combinacijus@gmail.com

    Reads transformation data from IMU BNO055 sensor
    and sends it via serial to ROS

    CONNECTIONS:
    VCC 5V | GND GND | SCL A5 + 4K7 PULLUP | SDA A4 + 4K7 PULLUP | ard_RST D2
    I2C address 0X28 or 0x29

    NOTE: ros_imu.h: typedef NodeHandle_<ArduinoHardware, 1, 2, 30, 90> NodeHandle;

    Won't publish /imu_data until full calibration is reached (all four states == 3 for some time)

    @param bool _recal: if true on reset IMU will do full recalibration
    @service reset: resets Arduino

    BUG: If IMU looses power Arduino get stuck in infinite loop 
    until IMU gets powered again
    BUG: If in loop Arduino won't reset by itself but will timeout ROS
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <ros_imu.h> // typedef NodeHandle_<ArduinoHardware, 1, 2, 30, 90> NodeHandle;
#include <samana_msgs/ImuSmall.h>
#include <samana_msgs/ImuCalib.h>
#include <TimerOne.h>
#include <std_srvs/Empty.h>

#define IMU_ID 42                // IMU sensor id
#define EE_ADDRESS 0             // Starting EEPROM addres for calibration data
#define SAMPLERATE_DELAY_US 3000 // Delay between samples (>4k slows updates)
#define BAUD_RATE 115200         // Serial communication speed. 115200 is minimal for maximal 100hz update rate
#define ISR_PERIOD 40000         // Interrupt service routine period in micro sec
#define PIN_RESET 2              // GPIO pin which is connected to RST
#define IMU_UPDATE_TIMEOUT 500   // Restart after inactivity in ms

void reset_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    reset();
}

bool found_calib = false;                  // Is calibration found in EEPROM
bool recalibrate = false;                  // Value read from param server if false don't read EEPROM
unsigned long long last_update_time = 0;   // In milis
unsigned long long last_setup_up_time = 0; // In milis

Adafruit_BNO055 bno = Adafruit_BNO055();
ros::NodeHandle nh; // ROS node
samana_msgs::ImuSmall imu_data;
ros::Publisher imu_data_pub("imu_data", &imu_data); // IMU data publisher
samana_msgs::ImuCalib imu_calib;
ros::Publisher imu_calib_pub("imu_calib", &imu_calib); // IMU calibration status publisher
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> srv_reset("reset", &reset_service);

uint8_t sys, gyro, acc, mag;    // Data for /imu_calib
sensors_event_t angVel, linAcc; // Data for /imu_data
imu::Quaternion quat;           // Data for /imu_data

String log_str;   // Temporary variable to set log_msg
char log_msg[20]; // Placeholder for log messages

void setup(void)
{
    pinMode(PIN_RESET, INPUT_PULLUP); // Pull high so it won't reset Arduino

    setupRoutine();

    // Initializing interrupt service routine for healthCheck()
    Timer1.initialize(ISR_PERIOD);
    Timer1.attachInterrupt(healthCheck);

    nh.spinOnce();
}

void loop(void)
{
    sendNewData();
    sendCalibrationStatus();
    last_update_time = millis();

    delayMicroseconds(SAMPLERATE_DELAY_US);
}

void reset()
{
    log_str = F("Arduino Restarting");
    strcpy(log_msg, log_str.c_str());
    nh.logfatal(log_msg);
    delay(50);

    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);
}

void setupRoutine()
{
    initROS();
    delay(100);

    // Needs longer timeout for IMU to initialize
    nh.setSpinTimeout(10000); // 10s timeout

    connectToBNO055();

    // CALIBRATION
    if (recalibrate)
    {
        log_str = F("Recalibrate");
        strcpy(log_msg, log_str.c_str());
        nh.logwarn(log_msg);
    }
    else
    {
        found_calib = readCalibrationFromEEPROM();
    }

    delay(1);
    // \/ Crystal must be configured after loading calibration
    bno.setExtCrystalUse(true);

    fullyCalibrate();
    if (!found_calib || recalibrate)
        saveCalibrationToEEPROM();

    // nh.setSpinTimeout(100); // Reset timeout
}

/*
    Inits node, advertises topics, gets params
*/
void initROS()
{
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();

    nh.advertise(imu_data_pub);
    nh.advertise(imu_calib_pub);
    nh.advertiseService(srv_reset);

    while (!nh.connected()) // Wait until connected
    {
        nh.spinOnce();
    }

    if (!nh.getParam("~recal", &recalibrate)) // Gets local param
    {
        recalibrate = false;
    }
    // Debuging parameters. Brake if not enough dynamic memory
    // sprintf(log_msg, "%d", recalibrate);
    // nh.logwarn(log_msg);
}

/*
    Sends custom IMU message to ROS
*/
void sendNewData()
{
    nh.spinOnce();
    quat = bno.getQuat();
    bno.getEvent(&angVel, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linAcc, Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Set data to send
    imu_data.header.stamp = nh.now();
    imu_data.header.frame_id = "imu";
    imu_data.quaternion_x = quat.x();
    imu_data.quaternion_y = quat.y();
    imu_data.quaternion_z = quat.z();
    imu_data.quaternion_w = quat.w();
    imu_data.linear_acceleration_x = linAcc.acceleration.x;
    imu_data.linear_acceleration_y = linAcc.acceleration.y;
    imu_data.linear_acceleration_z = linAcc.acceleration.z;
    imu_data.angular_velocity_x = angVel.gyro.x;
    imu_data.angular_velocity_y = angVel.gyro.y;
    imu_data.angular_velocity_z = angVel.gyro.z;
    imu_data_pub.publish(&imu_data);
    nh.spinOnce();
}
/*
    Sends calibration massage to ROS
*/
void sendCalibrationStatus()
{
    nh.spinOnce();
    uint8_t sys, gyr, acc, mag;
    bno.getCalibration(&sys, &gyr, &acc, &mag);

    imu_calib.sys = sys;
    imu_calib.gyr = gyr;
    imu_calib.acc = acc;
    imu_calib.mag = mag;
    imu_calib.temp = bno.getTemp();
    imu_calib_pub.publish(&imu_calib);
    nh.spinOnce();
}

/*
    Connects to IMU with address 0x28 0x29 or restarts Arduino
 */
void connectToBNO055()
{
    // Try to connect to BNO055 with address 0x28
    bno = Adafruit_BNO055(IMU_ID, 0x28);
    if (bno.begin())
    {
        log_str = F("IMU 0x28");
        strcpy(log_msg, log_str.c_str());
        nh.loginfo(log_msg);
    }
    else
    {
        // Try to connect to BNO055 with address 0x29
        bno = Adafruit_BNO055(IMU_ID, 0x29);
        if (bno.begin())
        {
            log_str = F("IMU 0x29");
            strcpy(log_msg, log_str.c_str());
            nh.loginfo(log_msg);
        }
        else
        {
            log_str = F("IMU NOT FOUND!");
            strcpy(log_msg, log_str.c_str());
            nh.logfatal(log_msg);
            delay(50);
            reset();
        }
    }
}

/*
    Reads calibration data from EEPROM if found
*/
bool readCalibrationFromEEPROM()
{
    long eepromID; // ID from EEPROM memory
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    EEPROM.get(EE_ADDRESS, eepromID); // Read ID

    bno.getSensor(&sensor); // Gets sensor info
    if (eepromID != sensor.sensor_id)
    {
        log_str = F("No calib data");
        strcpy(log_msg, log_str.c_str());
        nh.loginfo(log_msg);
        return false;
    }
    else
    {
        log_str = F("Calib data found");
        strcpy(log_msg, log_str.c_str());
        nh.loginfo(log_msg);
        EEPROM.get(EE_ADDRESS + sizeof(long), calibrationData);

        bno.setSensorOffsets(calibrationData);

        return true;
    }
}

/*
    Checks if IMU disconected
    Assuming data changes to same values when IMU disconnects
    @param update: if true updates variables to check
*/
bool isDataStuck(bool update = false)
{
    if (update)
    {
        quat = bno.getQuat();
        bno.getEvent(&angVel, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linAcc, Adafruit_BNO055::VECTOR_LINEARACCEL);
    }

    return quat.x() == quat.y() && quat.y() == quat.z() &&
           linAcc.acceleration.x == linAcc.acceleration.y &&
           linAcc.acceleration.y == linAcc.acceleration.z &&
           angVel.gyro.x == angVel.gyro.y &&
           angVel.gyro.y == angVel.gyro.z;
}

void fullyCalibrate()
{
    static unsigned int counter = 0;
    sensors_event_t event;

    // Always recalibrate the magnetometer as it goes out of calibration very often
    while (!isStableCalibrated())
    {
        logCalibStatus();
        sendCalibrationStatus();
        nh.spinOnce();

        // Check if event is valid or should we reset Arduino
        if (isDataStuck(true))
        {
            counter++;
            if (counter >= 3) // To filter accidental resets
                reset();
        }
        else
        {
            counter = 0;
        }
        nh.spinOnce();

        delay(250);
    }

    log_str = F("IMU Calibrated");
    strcpy(log_msg, log_str.c_str());
    nh.loginfo(log_msg);
}

void saveCalibrationToEEPROM()
{
    sensor_t sensor;

    bno.getSensor(&sensor);
    EEPROM.put(EE_ADDRESS, sensor.sensor_id);

    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    EEPROM.put(EE_ADDRESS + sizeof(long), newCalib);

    log_str = F("Calibration stored");
    strcpy(log_msg, log_str.c_str());
    nh.loginfo(log_msg);
}

/*
    Checks if full calibration lasted for some time
    @return status of stable calibration
 */
bool isStableCalibrated()
{
    const int stabilityTime = 1000;
    static unsigned long firstCalibTime = millis();
    static bool isCalib = false;

    if (bno.isFullyCalibrated())
    {
        if (!isCalib)
            firstCalibTime = millis();
        isCalib = true;
    }
    else
    {
        isCalib = false;
    }

    if (isCalib && millis() - firstCalibTime >= stabilityTime)
        return true;

    return false;
}

void logCalibStatus()
{
    bno.getCalibration(&sys, &gyro, &acc, &mag);

    sprintf(log_msg, "S%dG%dA%dM%d", sys, gyro, acc, mag);
    nh.logdebug(log_msg);
}

/*
    Checks if IMU still working while in main loop()
    If not resets Arduino
 */
void healthCheck()
{
    static int counter = -2; // Because first 2 check always bad
    // If all data equal to each other or updates don't accure
    if (isDataStuck(false) || millis() - last_update_time > IMU_UPDATE_TIMEOUT)
    {                     // IMU disconnected
        if (counter >= 0) // For debuging
        {
            log_str = F("Bad data");
            strcpy(log_msg, log_str.c_str());
            nh.logwarn(log_msg);
        }

        counter++;
        if (counter >= 5) // If 5 samples lost then restart
        {
            reset();
        }
    }
    else
    { // IMU working
        counter = 0;
    }

    nh.spinOnce();
}
