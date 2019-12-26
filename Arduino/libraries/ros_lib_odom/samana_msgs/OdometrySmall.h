#ifndef _ROS_samana_msgs_OdometrySmall_h
#define _ROS_samana_msgs_OdometrySmall_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace samana_msgs
{

  class OdometrySmall : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _ticks1_type;
      _ticks1_type ticks1;
      typedef uint16_t _ticks2_type;
      _ticks2_type ticks2;
      typedef float _speed1_type;
      _speed1_type speed1;
      typedef float _speed2_type;
      _speed2_type speed2;

    OdometrySmall():
      header(),
      ticks1(0),
      ticks2(0),
      speed1(0),
      speed2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ticks1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ticks1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticks1);
      *(outbuffer + offset + 0) = (this->ticks2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ticks2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticks2);
      union {
        float real;
        uint32_t base;
      } u_speed1;
      u_speed1.real = this->speed1;
      *(outbuffer + offset + 0) = (u_speed1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed1);
      union {
        float real;
        uint32_t base;
      } u_speed2;
      u_speed2.real = this->speed2;
      *(outbuffer + offset + 0) = (u_speed2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->ticks1 =  ((uint16_t) (*(inbuffer + offset)));
      this->ticks1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ticks1);
      this->ticks2 =  ((uint16_t) (*(inbuffer + offset)));
      this->ticks2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ticks2);
      union {
        float real;
        uint32_t base;
      } u_speed1;
      u_speed1.base = 0;
      u_speed1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed1 = u_speed1.real;
      offset += sizeof(this->speed1);
      union {
        float real;
        uint32_t base;
      } u_speed2;
      u_speed2.base = 0;
      u_speed2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed2 = u_speed2.real;
      offset += sizeof(this->speed2);
     return offset;
    }

    const char * getType(){ return "samana_msgs/OdometrySmall"; };
    const char * getMD5(){ return "66d5c83f5380a539385185906d186ae9"; };

  };

}
#endif
