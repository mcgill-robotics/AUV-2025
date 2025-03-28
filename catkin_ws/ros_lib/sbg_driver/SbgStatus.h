#ifndef _ROS_sbg_driver_SbgStatus_h
#define _ROS_sbg_driver_SbgStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgStatusGeneral.h"
#include "sbg_driver/SbgStatusCom.h"
#include "sbg_driver/SbgStatusAiding.h"

namespace sbg_driver
{

  class SbgStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgStatusGeneral _status_general_type;
      _status_general_type status_general;
      typedef sbg_driver::SbgStatusCom _status_com_type;
      _status_com_type status_com;
      typedef sbg_driver::SbgStatusAiding _status_aiding_type;
      _status_aiding_type status_aiding;

    SbgStatus():
      header(),
      time_stamp(0),
      status_general(),
      status_com(),
      status_aiding()
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
      offset += this->status_general.serialize(outbuffer + offset);
      offset += this->status_com.serialize(outbuffer + offset);
      offset += this->status_aiding.serialize(outbuffer + offset);
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
      offset += this->status_general.deserialize(inbuffer + offset);
      offset += this->status_com.deserialize(inbuffer + offset);
      offset += this->status_aiding.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgStatus"; };
    virtual const char * getMD5() override { return "1b73c890bd111d40339f4be9a7495e96"; };

  };

}
#endif
