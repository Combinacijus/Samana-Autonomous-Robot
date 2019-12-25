#ifndef _ROS_samana_msgs_ImuSmall_h
#define _ROS_samana_msgs_ImuSmall_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace samana_msgs
{

  class ImuSmall : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _quaternion_x_type;
      _quaternion_x_type quaternion_x;
      typedef float _quaternion_y_type;
      _quaternion_y_type quaternion_y;
      typedef float _quaternion_z_type;
      _quaternion_z_type quaternion_z;
      typedef float _quaternion_w_type;
      _quaternion_w_type quaternion_w;
      typedef float _linear_acceleration_x_type;
      _linear_acceleration_x_type linear_acceleration_x;
      typedef float _linear_acceleration_y_type;
      _linear_acceleration_y_type linear_acceleration_y;
      typedef float _linear_acceleration_z_type;
      _linear_acceleration_z_type linear_acceleration_z;
      typedef float _angular_velocity_x_type;
      _angular_velocity_x_type angular_velocity_x;
      typedef float _angular_velocity_y_type;
      _angular_velocity_y_type angular_velocity_y;
      typedef float _angular_velocity_z_type;
      _angular_velocity_z_type angular_velocity_z;

    ImuSmall():
      header(),
      quaternion_x(0),
      quaternion_y(0),
      quaternion_z(0),
      quaternion_w(0),
      linear_acceleration_x(0),
      linear_acceleration_y(0),
      linear_acceleration_z(0),
      angular_velocity_x(0),
      angular_velocity_y(0),
      angular_velocity_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_quaternion_x;
      u_quaternion_x.real = this->quaternion_x;
      *(outbuffer + offset + 0) = (u_quaternion_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quaternion_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quaternion_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quaternion_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->quaternion_x);
      union {
        float real;
        uint32_t base;
      } u_quaternion_y;
      u_quaternion_y.real = this->quaternion_y;
      *(outbuffer + offset + 0) = (u_quaternion_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quaternion_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quaternion_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quaternion_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->quaternion_y);
      union {
        float real;
        uint32_t base;
      } u_quaternion_z;
      u_quaternion_z.real = this->quaternion_z;
      *(outbuffer + offset + 0) = (u_quaternion_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quaternion_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quaternion_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quaternion_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->quaternion_z);
      union {
        float real;
        uint32_t base;
      } u_quaternion_w;
      u_quaternion_w.real = this->quaternion_w;
      *(outbuffer + offset + 0) = (u_quaternion_w.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_quaternion_w.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_quaternion_w.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_quaternion_w.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->quaternion_w);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_x;
      u_linear_acceleration_x.real = this->linear_acceleration_x;
      *(outbuffer + offset + 0) = (u_linear_acceleration_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acceleration_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acceleration_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acceleration_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acceleration_x);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_y;
      u_linear_acceleration_y.real = this->linear_acceleration_y;
      *(outbuffer + offset + 0) = (u_linear_acceleration_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acceleration_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acceleration_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acceleration_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acceleration_y);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_z;
      u_linear_acceleration_z.real = this->linear_acceleration_z;
      *(outbuffer + offset + 0) = (u_linear_acceleration_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acceleration_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acceleration_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acceleration_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acceleration_z);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_x;
      u_angular_velocity_x.real = this->angular_velocity_x;
      *(outbuffer + offset + 0) = (u_angular_velocity_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_x);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_y;
      u_angular_velocity_y.real = this->angular_velocity_y;
      *(outbuffer + offset + 0) = (u_angular_velocity_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_y);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_z;
      u_angular_velocity_z.real = this->angular_velocity_z;
      *(outbuffer + offset + 0) = (u_angular_velocity_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_quaternion_x;
      u_quaternion_x.base = 0;
      u_quaternion_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_quaternion_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_quaternion_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_quaternion_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->quaternion_x = u_quaternion_x.real;
      offset += sizeof(this->quaternion_x);
      union {
        float real;
        uint32_t base;
      } u_quaternion_y;
      u_quaternion_y.base = 0;
      u_quaternion_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_quaternion_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_quaternion_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_quaternion_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->quaternion_y = u_quaternion_y.real;
      offset += sizeof(this->quaternion_y);
      union {
        float real;
        uint32_t base;
      } u_quaternion_z;
      u_quaternion_z.base = 0;
      u_quaternion_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_quaternion_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_quaternion_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_quaternion_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->quaternion_z = u_quaternion_z.real;
      offset += sizeof(this->quaternion_z);
      union {
        float real;
        uint32_t base;
      } u_quaternion_w;
      u_quaternion_w.base = 0;
      u_quaternion_w.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_quaternion_w.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_quaternion_w.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_quaternion_w.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->quaternion_w = u_quaternion_w.real;
      offset += sizeof(this->quaternion_w);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_x;
      u_linear_acceleration_x.base = 0;
      u_linear_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acceleration_x = u_linear_acceleration_x.real;
      offset += sizeof(this->linear_acceleration_x);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_y;
      u_linear_acceleration_y.base = 0;
      u_linear_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acceleration_y = u_linear_acceleration_y.real;
      offset += sizeof(this->linear_acceleration_y);
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_z;
      u_linear_acceleration_z.base = 0;
      u_linear_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acceleration_z = u_linear_acceleration_z.real;
      offset += sizeof(this->linear_acceleration_z);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_x;
      u_angular_velocity_x.base = 0;
      u_angular_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_x = u_angular_velocity_x.real;
      offset += sizeof(this->angular_velocity_x);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_y;
      u_angular_velocity_y.base = 0;
      u_angular_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_y = u_angular_velocity_y.real;
      offset += sizeof(this->angular_velocity_y);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_z;
      u_angular_velocity_z.base = 0;
      u_angular_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_z = u_angular_velocity_z.real;
      offset += sizeof(this->angular_velocity_z);
     return offset;
    }

    const char * getType(){ return "samana_msgs/ImuSmall"; };
    const char * getMD5(){ return "c052128fb9568718800fa0ba7071e271"; };

  };

}
#endif
