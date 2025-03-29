#ifndef _ROS_auv_msgs_ThrusterForces_h
#define _ROS_auv_msgs_ThrusterForces_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class ThrusterForces : public ros::Msg
  {
    public:
      typedef float _BACK_LEFT_type;
      _BACK_LEFT_type BACK_LEFT;
      typedef float _HEAVE_BACK_LEFT_type;
      _HEAVE_BACK_LEFT_type HEAVE_BACK_LEFT;
      typedef float _HEAVE_FRONT_LEFT_type;
      _HEAVE_FRONT_LEFT_type HEAVE_FRONT_LEFT;
      typedef float _FRONT_LEFT_type;
      _FRONT_LEFT_type FRONT_LEFT;
      typedef float _FRONT_RIGHT_type;
      _FRONT_RIGHT_type FRONT_RIGHT;
      typedef float _HEAVE_FRONT_RIGHT_type;
      _HEAVE_FRONT_RIGHT_type HEAVE_FRONT_RIGHT;
      typedef float _HEAVE_BACK_RIGHT_type;
      _HEAVE_BACK_RIGHT_type HEAVE_BACK_RIGHT;
      typedef float _BACK_RIGHT_type;
      _BACK_RIGHT_type BACK_RIGHT;

    ThrusterForces():
      BACK_LEFT(0),
      HEAVE_BACK_LEFT(0),
      HEAVE_FRONT_LEFT(0),
      FRONT_LEFT(0),
      FRONT_RIGHT(0),
      HEAVE_FRONT_RIGHT(0),
      HEAVE_BACK_RIGHT(0),
      BACK_RIGHT(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->BACK_LEFT);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_BACK_LEFT);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_FRONT_LEFT);
      offset += serializeAvrFloat64(outbuffer + offset, this->FRONT_LEFT);
      offset += serializeAvrFloat64(outbuffer + offset, this->FRONT_RIGHT);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_FRONT_RIGHT);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_BACK_RIGHT);
      offset += serializeAvrFloat64(outbuffer + offset, this->BACK_RIGHT);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->BACK_LEFT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_BACK_LEFT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_FRONT_LEFT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->FRONT_LEFT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->FRONT_RIGHT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_FRONT_RIGHT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_BACK_RIGHT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->BACK_RIGHT));
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/ThrusterForces"; };
    virtual const char * getMD5() override { return "3852caff2b3aec4312af1b5ef825391d"; };

  };

}
#endif
