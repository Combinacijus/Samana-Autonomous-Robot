#ifndef _ROS_samana_msgs_ImuCalib_h
#define _ROS_samana_msgs_ImuCalib_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace samana_msgs
{

  class ImuCalib : public ros::Msg
  {
    public:
      typedef uint8_t _sys_type;
      _sys_type sys;
      typedef uint8_t _gyr_type;
      _gyr_type gyr;
      typedef uint8_t _acc_type;
      _acc_type acc;
      typedef uint8_t _mag_type;
      _mag_type mag;
      typedef int8_t _temp_type;
      _temp_type temp;

    ImuCalib():
      sys(0),
      gyr(0),
      acc(0),
      mag(0),
      temp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sys >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sys);
      *(outbuffer + offset + 0) = (this->gyr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gyr);
      *(outbuffer + offset + 0) = (this->acc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->acc);
      *(outbuffer + offset + 0) = (this->mag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag);
      union {
        int8_t real;
        uint8_t base;
      } u_temp;
      u_temp.real = this->temp;
      *(outbuffer + offset + 0) = (u_temp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->temp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sys =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sys);
      this->gyr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gyr);
      this->acc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->acc);
      this->mag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mag);
      union {
        int8_t real;
        uint8_t base;
      } u_temp;
      u_temp.base = 0;
      u_temp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->temp = u_temp.real;
      offset += sizeof(this->temp);
     return offset;
    }

    const char * getType(){ return "samana_msgs/ImuCalib"; };
    const char * getMD5(){ return "7764c0234b4e443d9ef754fa0119997d"; };

  };

}
#endif
