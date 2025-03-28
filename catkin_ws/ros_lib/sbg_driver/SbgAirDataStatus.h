#ifndef _ROS_sbg_driver_SbgAirDataStatus_h
#define _ROS_sbg_driver_SbgAirDataStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgAirDataStatus : public ros::Msg
  {
    public:
      typedef bool _is_delay_time_type;
      _is_delay_time_type is_delay_time;
      typedef bool _pressure_valid_type;
      _pressure_valid_type pressure_valid;
      typedef bool _altitude_valid_type;
      _altitude_valid_type altitude_valid;
      typedef bool _pressure_diff_valid_type;
      _pressure_diff_valid_type pressure_diff_valid;
      typedef bool _air_speed_valid_type;
      _air_speed_valid_type air_speed_valid;
      typedef bool _air_temperature_valid_type;
      _air_temperature_valid_type air_temperature_valid;

    SbgAirDataStatus():
      is_delay_time(0),
      pressure_valid(0),
      altitude_valid(0),
      pressure_diff_valid(0),
      air_speed_valid(0),
      air_temperature_valid(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_delay_time;
      u_is_delay_time.real = this->is_delay_time;
      *(outbuffer + offset + 0) = (u_is_delay_time.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_delay_time);
      union {
        bool real;
        uint8_t base;
      } u_pressure_valid;
      u_pressure_valid.real = this->pressure_valid;
      *(outbuffer + offset + 0) = (u_pressure_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pressure_valid);
      union {
        bool real;
        uint8_t base;
      } u_altitude_valid;
      u_altitude_valid.real = this->altitude_valid;
      *(outbuffer + offset + 0) = (u_altitude_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->altitude_valid);
      union {
        bool real;
        uint8_t base;
      } u_pressure_diff_valid;
      u_pressure_diff_valid.real = this->pressure_diff_valid;
      *(outbuffer + offset + 0) = (u_pressure_diff_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pressure_diff_valid);
      union {
        bool real;
        uint8_t base;
      } u_air_speed_valid;
      u_air_speed_valid.real = this->air_speed_valid;
      *(outbuffer + offset + 0) = (u_air_speed_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->air_speed_valid);
      union {
        bool real;
        uint8_t base;
      } u_air_temperature_valid;
      u_air_temperature_valid.real = this->air_temperature_valid;
      *(outbuffer + offset + 0) = (u_air_temperature_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->air_temperature_valid);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_delay_time;
      u_is_delay_time.base = 0;
      u_is_delay_time.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_delay_time = u_is_delay_time.real;
      offset += sizeof(this->is_delay_time);
      union {
        bool real;
        uint8_t base;
      } u_pressure_valid;
      u_pressure_valid.base = 0;
      u_pressure_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pressure_valid = u_pressure_valid.real;
      offset += sizeof(this->pressure_valid);
      union {
        bool real;
        uint8_t base;
      } u_altitude_valid;
      u_altitude_valid.base = 0;
      u_altitude_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->altitude_valid = u_altitude_valid.real;
      offset += sizeof(this->altitude_valid);
      union {
        bool real;
        uint8_t base;
      } u_pressure_diff_valid;
      u_pressure_diff_valid.base = 0;
      u_pressure_diff_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pressure_diff_valid = u_pressure_diff_valid.real;
      offset += sizeof(this->pressure_diff_valid);
      union {
        bool real;
        uint8_t base;
      } u_air_speed_valid;
      u_air_speed_valid.base = 0;
      u_air_speed_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->air_speed_valid = u_air_speed_valid.real;
      offset += sizeof(this->air_speed_valid);
      union {
        bool real;
        uint8_t base;
      } u_air_temperature_valid;
      u_air_temperature_valid.base = 0;
      u_air_temperature_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->air_temperature_valid = u_air_temperature_valid.real;
      offset += sizeof(this->air_temperature_valid);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgAirDataStatus"; };
    virtual const char * getMD5() override { return "7a096a0984e0fe096308cfb42a254e4a"; };

  };

}
#endif
