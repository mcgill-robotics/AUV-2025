#ifndef _ROS_sbg_driver_SbgMag_h
#define _ROS_sbg_driver_SbgMag_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sbg_driver/SbgMagStatus.h"

namespace sbg_driver
{

  class SbgMag : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef geometry_msgs::Vector3 _mag_type;
      _mag_type mag;
      typedef geometry_msgs::Vector3 _accel_type;
      _accel_type accel;
      typedef sbg_driver::SbgMagStatus _status_type;
      _status_type status;

    SbgMag():
      header(),
      time_stamp(0),
      mag(),
      accel(),
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
      offset += this->mag.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
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
      offset += this->mag.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgMag"; };
    virtual const char * getMD5() override { return "de7614c4cbb6cbd430c4a9b79bad88ca"; };

  };

}
#endif
