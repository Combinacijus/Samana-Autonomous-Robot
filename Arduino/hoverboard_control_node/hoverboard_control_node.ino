/*
  Gintaras Grebliunas
  combinacijus@gmail.com

  ROS node which:
    reads radio reciever data and sends it to ROS master
    reads hoverboard debug info and sends it to ROS master
    receives ROS master motor commands and sends to hoverboard
    recieves ROS master camera servo commands and controls servo

  To boot in no ROS mode restart Arduino while pressing button (on pin A2)
  no ROS mode is good because you can drive hoverboard without computer
  if no ROS mode is activated builtin LED will be on

  AltSoftSerial always uses these pins: TX 9, RX 8 also D10 PWM is unusable
  
  Note: 5V to 3.3V bidirectional logic converted needed between Arduino and hoverboard

  CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
  CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob

  Connections (using only UART2 (no button side)):
    HOV-GND -> ARD-GND
    HOV-GREEN(RX) -> 3.3V TO 5V -> D9(TX)
    HOV-BLUE(TX) -> 3.3v TO 5V -> D8(RX)
    HOV-RED -x (none)
    RC-PPM -> D2
    RC -> 5V and GND
    RC-SBUS -x (none)
    Logic converter 5V, 3.3V and GND to coresponding ARD
    SERVO-GND - GND
    SERVO-5V - 5V
    SERVO-SIGNAL - D9 (Only D9 works)

  Read all "NOTE:" comments for any modifications
*/

#include <AltSoftSerial.h>
#include <PPMReader.h>
#include <PWMServo.h>
#include <ros_motor.h>
#include <samana_msgs/Int16Array.h>
#include <samana_msgs/Teleop.h>
#include <std_msgs/UInt8.h>

#define BAUD_ROS 115200 // Baud rate for communication with ROS master (NOTE: same as in ROS master software)
#define BAUD_HOV 9600   // Baud rate for hoverboard UART2 communication (NOTE: same as in hoveboard software)
// 19200, 38400

#define DELAY 20          // Loop delay NOTE: it slows down RC update rate because it's not needed
#define PIN_BUTTON A2     // Button to boot in no ROS mode
#define PIN_SERVO 6       // For camera servo (must be pwm capable pin)
#define PIN_INTERRUPT 2   // For RC PPM
#define CHANNEL_COUNT 7   // For RC PPM
#define HOV_DATA_COUNT 8  // Number of hoverboard debug data points
#define MIN_SERVO 5       // Min servo value deg
#define MAX_SERVO 175     // Max servo value deg
#define SWITCH_TRIG 900   // Value when it considered that switch channel is triggered
#define SPEED_DEADZONE 20 // Deadzone for hoverboard speed

#define COMMAND_START 12838 // Arbitrary value signals that next 2 values will be steer and speed 011001000100110
#define COMMAND_END 3416    // Arbitrary value signals that data ended                            000110101011000

// NOTE: format <1:%i><2:%i>..
#define START_CHAR '<'  // Of hoverboard data
#define END_CHAR '>'    // Of hoverboard data
#define SPACER_CHAR ':' // Of hoverboard data

AltSoftSerial softSerial; // Pins TX9 RX8 hardcoded in library
PPMReader ppm(PIN_INTERRUPT, CHANNEL_COUNT);

PWMServo servo; // Servo motor mounted to the camera

ros::NodeHandle nh;
// Hoverboard debug data received via uart
samana_msgs::Int16Array hov_msg;
ros::Publisher hov_pub("hov", &hov_msg);
int16_t hov_data[HOV_DATA_COUNT];
// Radio controller input
samana_msgs::Int16Array rc_msg;
ros::Publisher rc_pub("rc", &rc_msg);
int16_t rc_data[CHANNEL_COUNT];
// Teleoperation commands for controlling motors
void teleopCb(const samana_msgs::Teleop &msg) { sendCommand(msg.steer, msg.speed); }
ros::Subscriber<samana_msgs::Teleop> teleop_sub("teleop", &teleopCb);

bool ros_mode = true; // If true connect to ROS

// Servo control
// void servoCb(const std_msgs::UInt8 &msg) { controlServo(msg.data); }
// ros::Subscriber<std_msgs::UInt8> servo_sub("servo", &servoCb);

void setup()
{
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  ros_mode = getRosMode();
  digitalWrite(LED_BUILTIN, !ros_mode);

  softSerial.begin(BAUD_HOV);
  // servo.attach(PIN_SERVO);

  // ros_mode = true;
  if (ros_mode)
  {
    // ROS serial node setup
    nh.getHardware()->setBaud(BAUD_ROS);
    nh.initNode();
    nh.advertise(hov_pub);
    nh.advertise(rc_pub);
    nh.subscribe(teleop_sub);
    // nh.subscribe(servo_sub);

    while (!nh.connected())
      nh.spinOnce();

    // Standard messages info
    hov_msg.data_length = HOV_DATA_COUNT;
    hov_msg.header.frame_id = "base_link";
    rc_msg.data_length = CHANNEL_COUNT;
    rc_msg.header.frame_id = "base_link";
  }
}

void loop()
{
  if (ros_mode)
  {
    publishHovData();
    publishRCData();
    // Motor control is done by subscriber callback function teleopCb()
    // Servo control is done by subscriber callback function servoCb()

    // Serial passthrough test
    // while (softSerial.available())
    //   Serial.write(softSerial.read());

    nh.spinOnce();
    delay(DELAY);
  }
  else // No ROS mode
  {
    static int16_t speed = 0, steer = 0;
    static bool armed = false;
    static float power_coef = 0;

    readRC();
    armed = rc_data[5] >= SWITCH_TRIG; // Switch arm by 2 pos-switch

    if (armed)
    {
      power_coef = (rc_data[6] + 1000) / 2000.0; // Knob channel
      speed = rc_data[1] * power_coef;           // Pitch
      steer = rc_data[0] * power_coef;           // Roll

      // Add deadzone
      if (abs(speed) < SPEED_DEADZONE)
        speed = 0;
      if (abs(steer) < SPEED_DEADZONE)
        steer = 0;

      // Clamp values to accepted range
      speed = clamp(speed, -1000, 1000);
      steer = clamp(steer, -1000, 1000);
    }
    else
    {
      speed = 0;
      steer = 0;
    }

    sendCommand(steer, speed);
    delay(DELAY);
  }
}

/*
  Returns false if button is pressed
  Note that button is pulled-up by default
*/
bool getRosMode()
{
  bool mode = true;

  delay(50);
  if (digitalRead(PIN_BUTTON) == HIGH)
    mode = true;
  else
    mode = false;

  return mode;
}

/*
  Continuously reading hoverboard debug data 
  and after receiving full line publishes to ROS
*/
void publishHovData()
{
  static char unit_buf[8];
  static int ind = 0;
  static bool still_reading_unit = false;
  char c;

  while (softSerial.available()) // Iterate available bytes
  {
    c = softSerial.read();

    if (c == '\n') // Full string received, end loop
    {
      // Serial.print("LINE END. Avail() = ");
      // Serial.println(softSerial.available());

      // for (int i = 0; i < 8; ++i)
      // {
      //   Serial.print(hov_data[i]);
      //   Serial.print(", ");
      // }
      // Serial.println("");

      // Publish data to ROS
      hov_msg.header.stamp = nh.now();
      hov_msg.data = hov_data;
      hov_pub.publish(&hov_msg);
      nh.spinOnce();

      break;
    }
    else // Not full string keep reading
    {
      if (c == START_CHAR) // Start of the data unit
      {
        still_reading_unit = true;
        continue;
      }
      else if (c == END_CHAR) // Full data unit received
      {
        still_reading_unit = false;
        unit_buf[ind] = '\0';
        ind = 0;

        // Serial.print("Unit read: ");
        // Serial.println(unit_buf);
        // Serial.print("Unit read1: ");
        // Serial.println(&unit_buf[2]);

        int unit_num = atoi(unit_buf);
        int unit_value = atoi(&unit_buf[2]); // Start array from '2' element

        hov_data[unit_num - 1] = unit_value;
        continue;
      }

      if (still_reading_unit) // Read bytes to the data unit
      {
        unit_buf[ind] = c;
        ind++;
      }
    }
  }

  nh.spinOnce();
}

/*
  Reads input from remote controller to a global rc_data array
*/
int16_t *readRC()
{
  static int16_t rc_data_old[CHANNEL_COUNT]; // For checking if RC RX is disconnected
  static int16_t rc_data_same_counter = 0;   // For checking if RC RX is disconnected
  static bool failsafe_on = false;

  // Get RC channel values
  for (int i = 0; i < CHANNEL_COUNT; ++i)
    rc_data[i] = getCommand(i + 1);

  // For failsafe
  rc_data_same_counter++;
  for (int i = 0; i < CHANNEL_COUNT; ++i)
  {
    if (rc_data[i] != rc_data_old[i])
    {
      rc_data_same_counter -= 2;
      break;
    }
  }

  // Set old rc_data
  for (int i = 0; i < CHANNEL_COUNT; ++i)
    rc_data_old[i] = rc_data[i];

  // Toogle failsafe
  if (rc_data_same_counter >= 10)
  {
    rc_data_same_counter = 10; // To stop overflow
    if (!failsafe_on)
      nh.logerror("FAILSAFE!");
    failsafe_on = true;
  }
  else if (rc_data_same_counter <= 0)
  {
    rc_data_same_counter = 0; // To stop overflow
    if (failsafe_on)
      nh.loginfo("RC RECONNECTED");
    failsafe_on = false;
  }

  // On failsafe do
  if (failsafe_on)
  {
    // Do a failsafe (NOTE: same as in a controller)
    rc_data[0] = rc_data[1] = rc_data[3] = rc_data[4] = rc_data[6] = 0;
    rc_data[2] = rc_data[5] = -1000;
  }
}

/*
  Publishes rc input to ROS
*/
void publishRCData()
{
  readRC();
  
  // Publish data to ROS
  rc_msg.header.stamp = nh.now();
  rc_msg.data = rc_data;
  rc_pub.publish(&rc_msg);
  nh.spinOnce();
}

/*
  Receives commands from ROS topic clamps it
  And controls servo
*/
void controlServo(uint8_t deg)
{
  deg = clamp(deg, MIN_SERVO, MAX_SERVO);
  servo.write(deg);
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
    softSerial.write((uint8_t *)&cmd_start, sizeof(cmd_start));
    softSerial.write((uint8_t *)&_steer, sizeof(_steer));
    softSerial.write((uint8_t *)&_speed, sizeof(_speed));
    softSerial.write((uint8_t *)&cmd_rnd, sizeof(cmd_rnd));
    softSerial.write((uint8_t *)&cmd_end, sizeof(cmd_end));
  }
}

/*
  @param channel: 1 to CHANNEL_COUNT
  @return val: pwm command value from RC channel between -1000 and 1000
*/
int getCommand(int channel)
{
  int value = ppm.latestValidChannelValue(channel, 0);
  // int value = ppm.rawChannelValue(channel);
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
