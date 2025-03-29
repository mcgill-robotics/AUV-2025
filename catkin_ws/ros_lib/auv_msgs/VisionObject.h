#ifndef _ROS_auv_msgs_VisionObject_h
#define _ROS_auv_msgs_VisionObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class VisionObject : public ros::Msg
  {
    public:
      typedef const char* _label_type;
      _label_type label;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _theta_z_type;
      _theta_z_type theta_z;
      typedef float _extra_field_type;
      _extra_field_type extra_field;
      typedef float _confidence_type;
      _confidence_type confidence;

    VisionObject():
      label(""),
      x(0),
      y(0),
      z(0),
      theta_z(0),
      extra_field(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->extra_field);
      offset += serializeAvrFloat64(outbuffer + offset, this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->extra_field));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->confidence));
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/VisionObject"; };
    virtual const char * getMD5() override { return "0f89e22270b02eb8422664d888a2cde3"; };

  };

}
#endif
