#ifndef _ROS_auv_msgs_VisionObjectArray_h
#define _ROS_auv_msgs_VisionObjectArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "auv_msgs/VisionObject.h"

namespace auv_msgs
{

  class VisionObjectArray : public ros::Msg
  {
    public:
      uint32_t array_length;
      typedef auv_msgs::VisionObject _array_type;
      _array_type st_array;
      _array_type * array;

    VisionObjectArray():
      array_length(0), st_array(), array(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->array_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->array_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->array_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->array_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->array_length);
      for( uint32_t i = 0; i < array_length; i++){
      offset += this->array[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t array_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      array_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      array_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      array_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->array_length);
      if(array_lengthT > array_length)
        this->array = (auv_msgs::VisionObject*)realloc(this->array, array_lengthT * sizeof(auv_msgs::VisionObject));
      array_length = array_lengthT;
      for( uint32_t i = 0; i < array_length; i++){
      offset += this->st_array.deserialize(inbuffer + offset);
        memcpy( &(this->array[i]), &(this->st_array), sizeof(auv_msgs::VisionObject));
      }
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/VisionObjectArray"; };
    virtual const char * getMD5() override { return "24620b68da017aa3f094b90c805b8019"; };

  };

}
#endif
