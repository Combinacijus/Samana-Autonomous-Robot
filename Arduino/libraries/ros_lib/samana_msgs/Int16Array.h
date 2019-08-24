#ifndef _ROS_samana_msgs_Int16Array_h
#define _ROS_samana_msgs_Int16Array_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace samana_msgs
{

  class Int16Array : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t dist_length;
      typedef int16_t _dist_type;
      _dist_type st_dist;
      _dist_type * dist;

    Int16Array():
      header(),
      dist_length(0), dist(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->dist_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dist_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dist_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dist_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dist_length);
      for( uint32_t i = 0; i < dist_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_disti;
      u_disti.real = this->dist[i];
      *(outbuffer + offset + 0) = (u_disti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_disti.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dist[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t dist_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dist_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dist_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dist_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dist_length);
      if(dist_lengthT > dist_length)
        this->dist = (int16_t*)realloc(this->dist, dist_lengthT * sizeof(int16_t));
      dist_length = dist_lengthT;
      for( uint32_t i = 0; i < dist_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_dist;
      u_st_dist.base = 0;
      u_st_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_dist = u_st_dist.real;
      offset += sizeof(this->st_dist);
        memcpy( &(this->dist[i]), &(this->st_dist), sizeof(int16_t));
      }
     return offset;
    }

    const char * getType(){ return "samana_msgs/Int16Array"; };
    const char * getMD5(){ return "ff005e0bae1d9bc418c95faadd17f36d"; };

  };

}
#endif
