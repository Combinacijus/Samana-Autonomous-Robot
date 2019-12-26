#ifndef _ROS_samana_msgs_ArmCmd_h
#define _ROS_samana_msgs_ArmCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace samana_msgs
{

  class ArmCmd : public ros::Msg
  {
    public:
      typedef int8_t _grabber_cmd_type;
      _grabber_cmd_type grabber_cmd;
      typedef int8_t _lifter_cmd_type;
      _lifter_cmd_type lifter_cmd;

    ArmCmd():
      grabber_cmd(0),
      lifter_cmd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_grabber_cmd;
      u_grabber_cmd.real = this->grabber_cmd;
      *(outbuffer + offset + 0) = (u_grabber_cmd.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grabber_cmd);
      union {
        int8_t real;
        uint8_t base;
      } u_lifter_cmd;
      u_lifter_cmd.real = this->lifter_cmd;
      *(outbuffer + offset + 0) = (u_lifter_cmd.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lifter_cmd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_grabber_cmd;
      u_grabber_cmd.base = 0;
      u_grabber_cmd.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->grabber_cmd = u_grabber_cmd.real;
      offset += sizeof(this->grabber_cmd);
      union {
        int8_t real;
        uint8_t base;
      } u_lifter_cmd;
      u_lifter_cmd.base = 0;
      u_lifter_cmd.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lifter_cmd = u_lifter_cmd.real;
      offset += sizeof(this->lifter_cmd);
     return offset;
    }

    const char * getType(){ return "samana_msgs/ArmCmd"; };
    const char * getMD5(){ return "4915ef1fb595c9707da4cb79c7caeeb8"; };

  };

}
#endif
