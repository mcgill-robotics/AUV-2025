#ifndef _ROS_sbg_driver_SbgEvent_h
#define _ROS_sbg_driver_SbgEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sbg_driver
{

  class SbgEvent : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef bool _overflow_type;
      _overflow_type overflow;
      typedef bool _offset_0_valid_type;
      _offset_0_valid_type offset_0_valid;
      typedef bool _offset_1_valid_type;
      _offset_1_valid_type offset_1_valid;
      typedef bool _offset_2_valid_type;
      _offset_2_valid_type offset_2_valid;
      typedef bool _offset_3_valid_type;
      _offset_3_valid_type offset_3_valid;
      typedef uint16_t _time_offset_0_type;
      _time_offset_0_type time_offset_0;
      typedef uint16_t _time_offset_1_type;
      _time_offset_1_type time_offset_1;
      typedef uint16_t _time_offset_2_type;
      _time_offset_2_type time_offset_2;
      typedef uint16_t _time_offset_3_type;
      _time_offset_3_type time_offset_3;

    SbgEvent():
      header(),
      time_stamp(0),
      overflow(0),
      offset_0_valid(0),
      offset_1_valid(0),
      offset_2_valid(0),
      offset_3_valid(0),
      time_offset_0(0),
      time_offset_1(0),
      time_offset_2(0),
      time_offset_3(0)
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
      union {
        bool real;
        uint8_t base;
      } u_overflow;
      u_overflow.real = this->overflow;
      *(outbuffer + offset + 0) = (u_overflow.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->overflow);
      union {
        bool real;
        uint8_t base;
      } u_offset_0_valid;
      u_offset_0_valid.real = this->offset_0_valid;
      *(outbuffer + offset + 0) = (u_offset_0_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->offset_0_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_1_valid;
      u_offset_1_valid.real = this->offset_1_valid;
      *(outbuffer + offset + 0) = (u_offset_1_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->offset_1_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_2_valid;
      u_offset_2_valid.real = this->offset_2_valid;
      *(outbuffer + offset + 0) = (u_offset_2_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->offset_2_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_3_valid;
      u_offset_3_valid.real = this->offset_3_valid;
      *(outbuffer + offset + 0) = (u_offset_3_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->offset_3_valid);
      *(outbuffer + offset + 0) = (this->time_offset_0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_offset_0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->time_offset_0);
      *(outbuffer + offset + 0) = (this->time_offset_1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_offset_1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->time_offset_1);
      *(outbuffer + offset + 0) = (this->time_offset_2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_offset_2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->time_offset_2);
      *(outbuffer + offset + 0) = (this->time_offset_3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_offset_3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->time_offset_3);
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
      union {
        bool real;
        uint8_t base;
      } u_overflow;
      u_overflow.base = 0;
      u_overflow.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->overflow = u_overflow.real;
      offset += sizeof(this->overflow);
      union {
        bool real;
        uint8_t base;
      } u_offset_0_valid;
      u_offset_0_valid.base = 0;
      u_offset_0_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->offset_0_valid = u_offset_0_valid.real;
      offset += sizeof(this->offset_0_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_1_valid;
      u_offset_1_valid.base = 0;
      u_offset_1_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->offset_1_valid = u_offset_1_valid.real;
      offset += sizeof(this->offset_1_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_2_valid;
      u_offset_2_valid.base = 0;
      u_offset_2_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->offset_2_valid = u_offset_2_valid.real;
      offset += sizeof(this->offset_2_valid);
      union {
        bool real;
        uint8_t base;
      } u_offset_3_valid;
      u_offset_3_valid.base = 0;
      u_offset_3_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->offset_3_valid = u_offset_3_valid.real;
      offset += sizeof(this->offset_3_valid);
      this->time_offset_0 =  ((uint16_t) (*(inbuffer + offset)));
      this->time_offset_0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->time_offset_0);
      this->time_offset_1 =  ((uint16_t) (*(inbuffer + offset)));
      this->time_offset_1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->time_offset_1);
      this->time_offset_2 =  ((uint16_t) (*(inbuffer + offset)));
      this->time_offset_2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->time_offset_2);
      this->time_offset_3 =  ((uint16_t) (*(inbuffer + offset)));
      this->time_offset_3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->time_offset_3);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgEvent"; };
    virtual const char * getMD5() override { return "330fcb628fc06f7fd7e2c2276c83adfa"; };

  };

}
#endif
