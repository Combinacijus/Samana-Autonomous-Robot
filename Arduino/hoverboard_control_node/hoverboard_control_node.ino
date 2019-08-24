/*
  Gintaras Grebliunas
  combinacijus@gmail.com

  ROS node which:
    reads radio reciever data and sends it to ROS master
    reads hoverboard debug info and sends it to ROS master
    receives ROS master motor commands and sends to hoverboard

  AltSoftSerial always uses these pins: TX 9, RX 8 also D10 PWM is unusable
  
  Note: 5V to  3.3V bidirecrional logic converted needed between Arduino and hoverboard

  CH1: Roll | CH2: Pitch | CH3: Throtlle | CH4: Yaw
  CH5: Switches | CH6: 2-Pos-Switch | CH7: Knob
*/
#define CH_THROT 3
#define CH_YAW 4
#define CH_ROLL 1
#define CH_PITCH 2
#define CH_KNOB 7
#define DELAY 20        // Loop delay
#define PIN_INTERRUPT 2 // For RC PPM
#define CHANNEL_COUNT 7 // For RC PPM
int pitch, roll, knob;  // Placeholders for channel values
float power_coef = 1;
int cmd_rnd = 0;

#include <AltSoftSerial.h>
#include <PPMReader.h>
#include <ros_motor.h>
#include <std_msgs/Int16MultiArray.h>

#define BAUD_ROS 115200 // Baud rate for communication with ROS master (NOTE: same as in ROS master software)
#define BAUD_HOV 9600   // Baud rate for hoverboard UART2 communication (NOTE: same as in hoveboard software)

#define COMMAND_START 12838 // Arbitrary value signals that next 2 values will be steer and speed 011001000100110
#define COMMAND_END 3416    // Arbitrary value signals that data ended                            000110101011000

// Format <1:%i><2:%i>..
#define START_CHAR '<'  // Of hoverboard data
#define END_CHAR '>'    // Of hoverboard data
#define SPACER_CHAR ':' // Of hoverboard data

ros::NodeHandle nh;


AltSoftSerial softSerial; // Pins TX9 RX8 hardcoded in library
PPMReader ppm(PIN_INTERRUPT, CHANNEL_COUNT);

int hov_data[8]; // Hoverboard data received via uart

void setup()
{
  Serial.begin(BAUD_ROS);
  while (!Serial)
    ;

  softSerial.begin(BAUD_HOV);
}

void loop()
{
  // Passthrough hoverbord data to usb serial
  // while (softSerial.available())
  //   Serial.write(softSerial.read());

  stripDebugData();

  // pitch = getCommand(CH_PITCH);
  // roll = getCommand(CH_ROLL);
  // knob = getCommand(CH_KNOB) + 1000; // [0 .. 2000]
  // power_coef = knob / 2000.0;

  // // Set motor speed and steering
  // uint16_t steer = (uint16_t)(roll * power_coef);
  // uint16_t speed = (uint16_t)(pitch * power_coef);

  // sendCommand(steer, speed);

  delay(DELAY);
}

void stripDebugData()
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

      //TODO sent to ROS

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