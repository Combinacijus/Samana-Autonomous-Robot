#ifndef _ROS_samana_msgs_Teleop_h
#define _ROS_samana_msgs_Teleop_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace samana_msgs
{

  class Teleop : public ros::Msg
  {
    public:
      typedef int16_t _speed_type;
      _speed_type speed;
      typedef int16_t _steer_type;
      _steer_type steer;

    Teleop():
      speed(0),
      steer(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        int16_t real;
        uint16_t base;
      } u_steer;
      u_steer.real = this->steer;
      *(outbuffer + offset + 0) = (u_steer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steer.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steer);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        int16_t real;
        uint16_t base;
      } u_steer;
      u_steer.base = 0;
      u_steer.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steer.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steer = u_steer.real;
      offset += sizeof(this->steer);
     return offset;
    }

    const char * getType(){ return "samana_msgs/Teleop"; };
    const char * getMD5(){ return "d8b9ad615ef7ce8cbee2931ec476027b"; };

  };

}
#endif
