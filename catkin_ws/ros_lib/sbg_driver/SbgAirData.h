#ifndef _ROS_sbg_driver_SbgAirData_h
#define _ROS_sbg_driver_SbgAirData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgAirDataStatus.h"

namespace sbg_driver
{

  class SbgAirData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgAirDataStatus _status_type;
      _status_type status;
      typedef float _pressure_abs_type;
      _pressure_abs_type pressure_abs;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _pressure_diff_type;
      _pressure_diff_type pressure_diff;
      typedef float _true_air_speed_type;
      _true_air_speed_type true_air_speed;
      typedef float _air_temperature_type;
      _air_temperature_type air_temperature;

    SbgAirData():
      header(),
      time_stamp(0),
      status(),
      pressure_abs(0),
      altitude(0),
      pressure_diff(0),
      true_air_speed(0),
      air_temperature(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_abs);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_diff);
      offset += serializeAvrFloat64(outbuffer + offset, this->true_air_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->air_temperature);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_abs));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_diff));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->true_air_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->air_temperature));
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgAirData"; };
    virtual const char * getMD5() override { return "f7982abc9b7165b89ea4d8dda93717f9"; };

  };

}
#endif
