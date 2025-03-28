#ifndef _ROS_sbg_driver_SbgGpsVel_h
#define _ROS_sbg_driver_SbgGpsVel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgGpsVelStatus.h"
#include "geometry_msgs/Vector3.h"

namespace sbg_driver
{

  class SbgGpsVel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgGpsVelStatus _status_type;
      _status_type status;
      typedef uint32_t _gps_tow_type;
      _gps_tow_type gps_tow;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Vector3 _velocity_accuracy_type;
      _velocity_accuracy_type velocity_accuracy;
      typedef float _course_type;
      _course_type course;
      typedef float _course_acc_type;
      _course_acc_type course_acc;

    SbgGpsVel():
      header(),
      time_stamp(0),
      status(),
      gps_tow(0),
      velocity(),
      velocity_accuracy(),
      course(0),
      course_acc(0)
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
      offset += this->status.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->gps_tow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_tow >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_tow >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_tow >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_tow);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->velocity_accuracy.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_course;
      u_course.real = this->course;
      *(outbuffer + offset + 0) = (u_course.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_course.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_course.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_course.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->course);
      union {
        float real;
        uint32_t base;
      } u_course_acc;
      u_course_acc.real = this->course_acc;
      *(outbuffer + offset + 0) = (u_course_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_course_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_course_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_course_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->course_acc);
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
      offset += this->status.deserialize(inbuffer + offset);
      this->gps_tow =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_tow);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->velocity_accuracy.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_course;
      u_course.base = 0;
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->course = u_course.real;
      offset += sizeof(this->course);
      union {
        float real;
        uint32_t base;
      } u_course_acc;
      u_course_acc.base = 0;
      u_course_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_course_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_course_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_course_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->course_acc = u_course_acc.real;
      offset += sizeof(this->course_acc);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgGpsVel"; };
    virtual const char * getMD5() override { return "dc36a4705c96041ace5f0875af58a725"; };

  };

}
#endif
