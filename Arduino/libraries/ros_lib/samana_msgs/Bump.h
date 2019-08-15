#ifndef _ROS_samana_msgs_Bump_h
#define _ROS_samana_msgs_Bump_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace samana_msgs
{

  class Bump : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _bump_bits_type;
      _bump_bits_type bump_bits;

    Bump():
      header(),
      bump_bits(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->bump_bits >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bump_bits >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bump_bits);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->bump_bits =  ((uint16_t) (*(inbuffer + offset)));
      this->bump_bits |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->bump_bits);
     return offset;
    }

    const char * getType(){ return "samana_msgs/Bump"; };
    const char * getMD5(){ return "be760e4b4932adb10f1e9fc66f2d1769"; };

  };

}
#endif
