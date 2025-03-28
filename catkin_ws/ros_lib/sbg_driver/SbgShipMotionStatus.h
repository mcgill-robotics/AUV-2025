#ifndef _ROS_sbg_driver_SbgShipMotionStatus_h
#define _ROS_sbg_driver_SbgShipMotionStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgShipMotionStatus : public ros::Msg
  {
    public:
      typedef bool _heave_valid_type;
      _heave_valid_type heave_valid;
      typedef bool _heave_vel_aided_type;
      _heave_vel_aided_type heave_vel_aided;
      typedef bool _period_available_type;
      _period_available_type period_available;
      typedef bool _period_valid_type;
      _period_valid_type period_valid;

    SbgShipMotionStatus():
      heave_valid(0),
      heave_vel_aided(0),
      period_available(0),
      period_valid(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_heave_valid;
      u_heave_valid.real = this->heave_valid;
      *(outbuffer + offset + 0) = (u_heave_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->heave_valid);
      union {
        bool real;
        uint8_t base;
      } u_heave_vel_aided;
      u_heave_vel_aided.real = this->heave_vel_aided;
      *(outbuffer + offset + 0) = (u_heave_vel_aided.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->heave_vel_aided);
      union {
        bool real;
        uint8_t base;
      } u_period_available;
      u_period_available.real = this->period_available;
      *(outbuffer + offset + 0) = (u_period_available.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->period_available);
      union {
        bool real;
        uint8_t base;
      } u_period_valid;
      u_period_valid.real = this->period_valid;
      *(outbuffer + offset + 0) = (u_period_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->period_valid);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_heave_valid;
      u_heave_valid.base = 0;
      u_heave_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->heave_valid = u_heave_valid.real;
      offset += sizeof(this->heave_valid);
      union {
        bool real;
        uint8_t base;
      } u_heave_vel_aided;
      u_heave_vel_aided.base = 0;
      u_heave_vel_aided.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->heave_vel_aided = u_heave_vel_aided.real;
      offset += sizeof(this->heave_vel_aided);
      union {
        bool real;
        uint8_t base;
      } u_period_available;
      u_period_available.base = 0;
      u_period_available.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->period_available = u_period_available.real;
      offset += sizeof(this->period_available);
      union {
        bool real;
        uint8_t base;
      } u_period_valid;
      u_period_valid.base = 0;
      u_period_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->period_valid = u_period_valid.real;
      offset += sizeof(this->period_valid);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgShipMotionStatus"; };
    virtual const char * getMD5() override { return "a834637d2b283cc5378d30ef92fcf13a"; };

  };

}
#endif
