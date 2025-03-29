#ifndef _ROS_sbg_driver_SbgGpsHdt_h
#define _ROS_sbg_driver_SbgGpsHdt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sbg_driver
{

  class SbgGpsHdt : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef uint16_t _status_type;
      _status_type status;
      typedef uint32_t _tow_type;
      _tow_type tow;
      typedef float _true_heading_type;
      _true_heading_type true_heading;
      typedef float _true_heading_acc_type;
      _true_heading_acc_type true_heading_acc;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _pitch_acc_type;
      _pitch_acc_type pitch_acc;
      typedef float _baseline_type;
      _baseline_type baseline;

    SbgGpsHdt():
      header(),
      time_stamp(0),
      status(0),
      tow(0),
      true_heading(0),
      true_heading_acc(0),
      pitch(0),
      pitch_acc(0),
      baseline(0)
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
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->tow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tow >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tow >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tow >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tow);
      union {
        float real;
        uint32_t base;
      } u_true_heading;
      u_true_heading.real = this->true_heading;
      *(outbuffer + offset + 0) = (u_true_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_true_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_true_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_true_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->true_heading);
      union {
        float real;
        uint32_t base;
      } u_true_heading_acc;
      u_true_heading_acc.real = this->true_heading_acc;
      *(outbuffer + offset + 0) = (u_true_heading_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_true_heading_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_true_heading_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_true_heading_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->true_heading_acc);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_pitch_acc;
      u_pitch_acc.real = this->pitch_acc;
      *(outbuffer + offset + 0) = (u_pitch_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_acc);
      union {
        float real;
        uint32_t base;
      } u_baseline;
      u_baseline.real = this->baseline;
      *(outbuffer + offset + 0) = (u_baseline.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_baseline.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_baseline.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_baseline.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->baseline);
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
      this->status =  ((uint16_t) (*(inbuffer + offset)));
      this->status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->status);
      this->tow =  ((uint32_t) (*(inbuffer + offset)));
      this->tow |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tow |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tow |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tow);
      union {
        float real;
        uint32_t base;
      } u_true_heading;
      u_true_heading.base = 0;
      u_true_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_true_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_true_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_true_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->true_heading = u_true_heading.real;
      offset += sizeof(this->true_heading);
      union {
        float real;
        uint32_t base;
      } u_true_heading_acc;
      u_true_heading_acc.base = 0;
      u_true_heading_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_true_heading_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_true_heading_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_true_heading_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->true_heading_acc = u_true_heading_acc.real;
      offset += sizeof(this->true_heading_acc);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_pitch_acc;
      u_pitch_acc.base = 0;
      u_pitch_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_acc = u_pitch_acc.real;
      offset += sizeof(this->pitch_acc);
      union {
        float real;
        uint32_t base;
      } u_baseline;
      u_baseline.base = 0;
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->baseline = u_baseline.real;
      offset += sizeof(this->baseline);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgGpsHdt"; };
    virtual const char * getMD5() override { return "8c984f2f80abb760d3b4701ef29aeceb"; };

  };

}
#endif
