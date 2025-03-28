#ifndef _ROS_sbg_driver_SbgUtcTime_h
#define _ROS_sbg_driver_SbgUtcTime_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgUtcTimeStatus.h"

namespace sbg_driver
{

  class SbgUtcTime : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgUtcTimeStatus _clock_status_type;
      _clock_status_type clock_status;
      typedef uint16_t _year_type;
      _year_type year;
      typedef uint8_t _month_type;
      _month_type month;
      typedef uint8_t _day_type;
      _day_type day;
      typedef uint8_t _hour_type;
      _hour_type hour;
      typedef uint8_t _min_type;
      _min_type min;
      typedef uint8_t _sec_type;
      _sec_type sec;
      typedef uint32_t _nanosec_type;
      _nanosec_type nanosec;
      typedef uint32_t _gps_tow_type;
      _gps_tow_type gps_tow;

    SbgUtcTime():
      header(),
      time_stamp(0),
      clock_status(),
      year(0),
      month(0),
      day(0),
      hour(0),
      min(0),
      sec(0),
      nanosec(0),
      gps_tow(0)
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
      offset += this->clock_status.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->year >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->year >> (8 * 1)) & 0xFF;
      offset += sizeof(this->year);
      *(outbuffer + offset + 0) = (this->month >> (8 * 0)) & 0xFF;
      offset += sizeof(this->month);
      *(outbuffer + offset + 0) = (this->day >> (8 * 0)) & 0xFF;
      offset += sizeof(this->day);
      *(outbuffer + offset + 0) = (this->hour >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hour);
      *(outbuffer + offset + 0) = (this->min >> (8 * 0)) & 0xFF;
      offset += sizeof(this->min);
      *(outbuffer + offset + 0) = (this->sec >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sec);
      *(outbuffer + offset + 0) = (this->nanosec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nanosec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nanosec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nanosec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nanosec);
      *(outbuffer + offset + 0) = (this->gps_tow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_tow >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_tow >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_tow >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_tow);
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
      offset += this->clock_status.deserialize(inbuffer + offset);
      this->year =  ((uint16_t) (*(inbuffer + offset)));
      this->year |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->year);
      this->month =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->month);
      this->day =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->day);
      this->hour =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->hour);
      this->min =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->min);
      this->sec =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sec);
      this->nanosec =  ((uint32_t) (*(inbuffer + offset)));
      this->nanosec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->nanosec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->nanosec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->nanosec);
      this->gps_tow =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_tow);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgUtcTime"; };
    virtual const char * getMD5() override { return "89495f07708fa38e487b6509c4edabaa"; };

  };

}
#endif
