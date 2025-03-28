#ifndef _ROS_sbg_driver_SbgGpsVelStatus_h
#define _ROS_sbg_driver_SbgGpsVelStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgGpsVelStatus : public ros::Msg
  {
    public:
      typedef uint8_t _vel_status_type;
      _vel_status_type vel_status;
      typedef uint8_t _vel_type_type;
      _vel_type_type vel_type;

    SbgGpsVelStatus():
      vel_status(0),
      vel_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->vel_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vel_status);
      *(outbuffer + offset + 0) = (this->vel_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vel_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->vel_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->vel_status);
      this->vel_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->vel_type);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgGpsVelStatus"; };
    virtual const char * getMD5() override { return "8c5fcc3c3ffd11ce820539049c166dde"; };

  };

}
#endif
