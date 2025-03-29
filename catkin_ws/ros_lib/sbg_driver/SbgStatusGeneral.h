#ifndef _ROS_sbg_driver_SbgStatusGeneral_h
#define _ROS_sbg_driver_SbgStatusGeneral_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgStatusGeneral : public ros::Msg
  {
    public:
      typedef bool _main_power_type;
      _main_power_type main_power;
      typedef bool _imu_power_type;
      _imu_power_type imu_power;
      typedef bool _gps_power_type;
      _gps_power_type gps_power;
      typedef bool _settings_type;
      _settings_type settings;
      typedef bool _temperature_type;
      _temperature_type temperature;

    SbgStatusGeneral():
      main_power(0),
      imu_power(0),
      gps_power(0),
      settings(0),
      temperature(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_main_power;
      u_main_power.real = this->main_power;
      *(outbuffer + offset + 0) = (u_main_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->main_power);
      union {
        bool real;
        uint8_t base;
      } u_imu_power;
      u_imu_power.real = this->imu_power;
      *(outbuffer + offset + 0) = (u_imu_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_power);
      union {
        bool real;
        uint8_t base;
      } u_gps_power;
      u_gps_power.real = this->gps_power;
      *(outbuffer + offset + 0) = (u_gps_power.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_power);
      union {
        bool real;
        uint8_t base;
      } u_settings;
      u_settings.real = this->settings;
      *(outbuffer + offset + 0) = (u_settings.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->settings);
      union {
        bool real;
        uint8_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->temperature);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_main_power;
      u_main_power.base = 0;
      u_main_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->main_power = u_main_power.real;
      offset += sizeof(this->main_power);
      union {
        bool real;
        uint8_t base;
      } u_imu_power;
      u_imu_power.base = 0;
      u_imu_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_power = u_imu_power.real;
      offset += sizeof(this->imu_power);
      union {
        bool real;
        uint8_t base;
      } u_gps_power;
      u_gps_power.base = 0;
      u_gps_power.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps_power = u_gps_power.real;
      offset += sizeof(this->gps_power);
      union {
        bool real;
        uint8_t base;
      } u_settings;
      u_settings.base = 0;
      u_settings.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->settings = u_settings.real;
      offset += sizeof(this->settings);
      union {
        bool real;
        uint8_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgStatusGeneral"; };
    virtual const char * getMD5() override { return "693fdf7e799b5fc52833d1649c048053"; };

  };

}
#endif
