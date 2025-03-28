#ifndef _ROS_sbg_driver_SbgEkfEuler_h
#define _ROS_sbg_driver_SbgEkfEuler_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sbg_driver/SbgEkfStatus.h"

namespace sbg_driver
{

  class SbgEkfEuler : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef geometry_msgs::Vector3 _angle_type;
      _angle_type angle;
      typedef geometry_msgs::Vector3 _accuracy_type;
      _accuracy_type accuracy;
      typedef sbg_driver::SbgEkfStatus _status_type;
      _status_type status;

    SbgEkfEuler():
      header(),
      time_stamp(0),
      angle(),
      accuracy(),
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->time_stamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_stamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_stamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_stamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_stamp);
      offset += this->angle.serialize(outbuffer + offset);
      offset += this->accuracy.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->time_stamp =  ((uint32_t) (*(inbuffer + offset)));
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_stamp);
      offset += this->angle.deserialize(inbuffer + offset);
      offset += this->accuracy.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgEkfEuler"; };
    virtual const char * getMD5() override { return "4862d08d71471abacf02c798eaa2e18f"; };

  };

}
#endif
