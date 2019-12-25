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
      typedef int16_t _ticks1_type;
      _ticks1_type ticks1;
      typedef int16_t _ticks2_type;
      _ticks2_type ticks2;
      typedef float _rps1_type;
      _rps1_type rps1;
      typedef float _rps2_type;
      _rps2_type rps2;

    OdometrySmall():
      header(),
      ticks1(0),
      ticks2(0),
      rps1(0),
      rps2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_ticks1;
      u_ticks1.real = this->ticks1;
      *(outbuffer + offset + 0) = (u_ticks1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticks1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticks1);
      union {
        int16_t real;
        uint16_t base;
      } u_ticks2;
      u_ticks2.real = this->ticks2;
      *(outbuffer + offset + 0) = (u_ticks2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticks2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticks2);
      union {
        float real;
        uint32_t base;
      } u_rps1;
      u_rps1.real = this->rps1;
      *(outbuffer + offset + 0) = (u_rps1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rps1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rps1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rps1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rps1);
      union {
        float real;
        uint32_t base;
      } u_rps2;
      u_rps2.real = this->rps2;
      *(outbuffer + offset + 0) = (u_rps2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rps2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rps2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rps2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rps2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_ticks1;
      u_ticks1.base = 0;
      u_ticks1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticks1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ticks1 = u_ticks1.real;
      offset += sizeof(this->ticks1);
      union {
        int16_t real;
        uint16_t base;
      } u_ticks2;
      u_ticks2.base = 0;
      u_ticks2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticks2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ticks2 = u_ticks2.real;
      offset += sizeof(this->ticks2);
      union {
        float real;
        uint32_t base;
      } u_rps1;
      u_rps1.base = 0;
      u_rps1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rps1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rps1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rps1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rps1 = u_rps1.real;
      offset += sizeof(this->rps1);
      union {
        float real;
        uint32_t base;
      } u_rps2;
      u_rps2.base = 0;
      u_rps2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rps2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rps2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rps2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rps2 = u_rps2.real;
      offset += sizeof(this->rps2);
     return offset;
    }

    const char * getType(){ return "samana_msgs/OdometrySmall"; };
    const char * getMD5(){ return "ae1df5c5a8337951090ccf08be442d3d"; };

  };

}
#endif
