#ifndef _ROS_sbg_driver_SbgShipMotion_h
#define _ROS_sbg_driver_SbgShipMotion_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "sbg_driver/SbgShipMotionStatus.h"

namespace sbg_driver
{

  class SbgShipMotion : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef uint16_t _heave_period_type;
      _heave_period_type heave_period;
      typedef geometry_msgs::Vector3 _ship_motion_type;
      _ship_motion_type ship_motion;
      typedef geometry_msgs::Vector3 _acceleration_type;
      _acceleration_type acceleration;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef sbg_driver::SbgShipMotionStatus _status_type;
      _status_type status;

    SbgShipMotion():
      header(),
      time_stamp(0),
      heave_period(0),
      ship_motion(),
      acceleration(),
      velocity(),
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
      *(outbuffer + offset + 0) = (this->heave_period >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->heave_period >> (8 * 1)) & 0xFF;
      offset += sizeof(this->heave_period);
      offset += this->ship_motion.serialize(outbuffer + offset);
      offset += this->acceleration.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
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
      this->heave_period =  ((uint16_t) (*(inbuffer + offset)));
      this->heave_period |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->heave_period);
      offset += this->ship_motion.deserialize(inbuffer + offset);
      offset += this->acceleration.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgShipMotion"; };
    virtual const char * getMD5() override { return "f76d2a0b5a6d09d258ebd78ee0233cd0"; };

  };

}
#endif
