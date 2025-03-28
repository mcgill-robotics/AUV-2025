#ifndef _ROS_auv_msgs_PingerTimeDifference_h
#define _ROS_auv_msgs_PingerTimeDifference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class PingerTimeDifference : public ros::Msg
  {
    public:
      typedef int32_t _frequency_type;
      _frequency_type frequency;
      uint32_t times_length;
      typedef uint32_t _times_type;
      _times_type st_times;
      _times_type * times;

    PingerTimeDifference():
      frequency(0),
      times_length(0), st_times(), times(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_frequency;
      u_frequency.real = this->frequency;
      *(outbuffer + offset + 0) = (u_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset + 0) = (this->times_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_length);
      for( uint32_t i = 0; i < times_length; i++){
      *(outbuffer + offset + 0) = (this->times[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_frequency;
      u_frequency.base = 0;
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frequency = u_frequency.real;
      offset += sizeof(this->frequency);
      uint32_t times_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_length);
      if(times_lengthT > times_length)
        this->times = (uint32_t*)realloc(this->times, times_lengthT * sizeof(uint32_t));
      times_length = times_lengthT;
      for( uint32_t i = 0; i < times_length; i++){
      this->st_times =  ((uint32_t) (*(inbuffer + offset)));
      this->st_times |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_times |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_times |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_times);
        memcpy( &(this->times[i]), &(this->st_times), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/PingerTimeDifference"; };
    virtual const char * getMD5() override { return "190b09a555ef19ac4b340cefab903ff1"; };

  };

}
#endif
