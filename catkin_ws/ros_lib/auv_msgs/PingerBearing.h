#ifndef _ROS_auv_msgs_PingerBearing_h
#define _ROS_auv_msgs_PingerBearing_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace auv_msgs
{

  class PingerBearing : public ros::Msg
  {
    public:
      typedef int32_t _frequency_type;
      _frequency_type frequency;
      typedef geometry_msgs::Vector3 _pinger_bearing_type;
      _pinger_bearing_type pinger_bearing;
      typedef float _state_x_type;
      _state_x_type state_x;
      typedef float _state_y_type;
      _state_y_type state_y;
      typedef float _state_z_type;
      _state_z_type state_z;

    PingerBearing():
      frequency(0),
      pinger_bearing(),
      state_x(0),
      state_y(0),
      state_z(0)
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
      offset += this->pinger_bearing.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->state_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->state_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->state_z);
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
      offset += this->pinger_bearing.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->state_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->state_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->state_z));
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/PingerBearing"; };
    virtual const char * getMD5() override { return "3548f51c0c21fcf4ed54a18e3aa8c0f8"; };

  };

}
#endif
