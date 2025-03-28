#ifndef _ROS_sbg_driver_SbgUtcTimeStatus_h
#define _ROS_sbg_driver_SbgUtcTimeStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgUtcTimeStatus : public ros::Msg
  {
    public:
      typedef bool _clock_stable_type;
      _clock_stable_type clock_stable;
      typedef uint8_t _clock_status_type;
      _clock_status_type clock_status;
      typedef bool _clock_utc_sync_type;
      _clock_utc_sync_type clock_utc_sync;
      typedef uint8_t _clock_utc_status_type;
      _clock_utc_status_type clock_utc_status;

    SbgUtcTimeStatus():
      clock_stable(0),
      clock_status(0),
      clock_utc_sync(0),
      clock_utc_status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_clock_stable;
      u_clock_stable.real = this->clock_stable;
      *(outbuffer + offset + 0) = (u_clock_stable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clock_stable);
      *(outbuffer + offset + 0) = (this->clock_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clock_status);
      union {
        bool real;
        uint8_t base;
      } u_clock_utc_sync;
      u_clock_utc_sync.real = this->clock_utc_sync;
      *(outbuffer + offset + 0) = (u_clock_utc_sync.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clock_utc_sync);
      *(outbuffer + offset + 0) = (this->clock_utc_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clock_utc_status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_clock_stable;
      u_clock_stable.base = 0;
      u_clock_stable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clock_stable = u_clock_stable.real;
      offset += sizeof(this->clock_stable);
      this->clock_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->clock_status);
      union {
        bool real;
        uint8_t base;
      } u_clock_utc_sync;
      u_clock_utc_sync.base = 0;
      u_clock_utc_sync.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clock_utc_sync = u_clock_utc_sync.real;
      offset += sizeof(this->clock_utc_sync);
      this->clock_utc_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->clock_utc_status);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgUtcTimeStatus"; };
    virtual const char * getMD5() override { return "d140f95192866cb459fe7af2851c8eed"; };

  };

}
#endif
