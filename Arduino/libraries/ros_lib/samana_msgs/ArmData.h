#ifndef _ROS_samana_msgs_ArmData_h
#define _ROS_samana_msgs_ArmData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace samana_msgs
{

  class ArmData : public ros::Msg
  {
    public:
      typedef int16_t _current_grabber_type;
      _current_grabber_type current_grabber;
      typedef int16_t _current_lifter_type;
      _current_lifter_type current_lifter;
      typedef uint8_t _limit_switches_type;
      _limit_switches_type limit_switches;

    ArmData():
      current_grabber(0),
      current_lifter(0),
      limit_switches(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_current_grabber;
      u_current_grabber.real = this->current_grabber;
      *(outbuffer + offset + 0) = (u_current_grabber.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_grabber.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_grabber);
      union {
        int16_t real;
        uint16_t base;
      } u_current_lifter;
      u_current_lifter.real = this->current_lifter;
      *(outbuffer + offset + 0) = (u_current_lifter.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_lifter.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_lifter);
      *(outbuffer + offset + 0) = (this->limit_switches >> (8 * 0)) & 0xFF;
      offset += sizeof(this->limit_switches);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_current_grabber;
      u_current_grabber.base = 0;
      u_current_grabber.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_grabber.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_grabber = u_current_grabber.real;
      offset += sizeof(this->current_grabber);
      union {
        int16_t real;
        uint16_t base;
      } u_current_lifter;
      u_current_lifter.base = 0;
      u_current_lifter.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_lifter.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_lifter = u_current_lifter.real;
      offset += sizeof(this->current_lifter);
      this->limit_switches =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->limit_switches);
     return offset;
    }

    const char * getType(){ return "samana_msgs/ArmData"; };
    const char * getMD5(){ return "3812456ac8efcba0c0c24a88991ba799"; };

  };

}
#endif
