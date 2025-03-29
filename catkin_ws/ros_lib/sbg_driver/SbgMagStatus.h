#ifndef _ROS_sbg_driver_SbgMagStatus_h
#define _ROS_sbg_driver_SbgMagStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgMagStatus : public ros::Msg
  {
    public:
      typedef bool _mag_x_type;
      _mag_x_type mag_x;
      typedef bool _mag_y_type;
      _mag_y_type mag_y;
      typedef bool _mag_z_type;
      _mag_z_type mag_z;
      typedef bool _accel_x_type;
      _accel_x_type accel_x;
      typedef bool _accel_y_type;
      _accel_y_type accel_y;
      typedef bool _accel_z_type;
      _accel_z_type accel_z;
      typedef bool _mags_in_range_type;
      _mags_in_range_type mags_in_range;
      typedef bool _accels_in_range_type;
      _accels_in_range_type accels_in_range;
      typedef bool _calibration_type;
      _calibration_type calibration;

    SbgMagStatus():
      mag_x(0),
      mag_y(0),
      mag_z(0),
      accel_x(0),
      accel_y(0),
      accel_z(0),
      mags_in_range(0),
      accels_in_range(0),
      calibration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_mag_x;
      u_mag_x.real = this->mag_x;
      *(outbuffer + offset + 0) = (u_mag_x.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag_x);
      union {
        bool real;
        uint8_t base;
      } u_mag_y;
      u_mag_y.real = this->mag_y;
      *(outbuffer + offset + 0) = (u_mag_y.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag_y);
      union {
        bool real;
        uint8_t base;
      } u_mag_z;
      u_mag_z.real = this->mag_z;
      *(outbuffer + offset + 0) = (u_mag_z.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag_z);
      union {
        bool real;
        uint8_t base;
      } u_accel_x;
      u_accel_x.real = this->accel_x;
      *(outbuffer + offset + 0) = (u_accel_x.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accel_x);
      union {
        bool real;
        uint8_t base;
      } u_accel_y;
      u_accel_y.real = this->accel_y;
      *(outbuffer + offset + 0) = (u_accel_y.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accel_y);
      union {
        bool real;
        uint8_t base;
      } u_accel_z;
      u_accel_z.real = this->accel_z;
      *(outbuffer + offset + 0) = (u_accel_z.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accel_z);
      union {
        bool real;
        uint8_t base;
      } u_mags_in_range;
      u_mags_in_range.real = this->mags_in_range;
      *(outbuffer + offset + 0) = (u_mags_in_range.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mags_in_range);
      union {
        bool real;
        uint8_t base;
      } u_accels_in_range;
      u_accels_in_range.real = this->accels_in_range;
      *(outbuffer + offset + 0) = (u_accels_in_range.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->accels_in_range);
      union {
        bool real;
        uint8_t base;
      } u_calibration;
      u_calibration.real = this->calibration;
      *(outbuffer + offset + 0) = (u_calibration.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_mag_x;
      u_mag_x.base = 0;
      u_mag_x.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mag_x = u_mag_x.real;
      offset += sizeof(this->mag_x);
      union {
        bool real;
        uint8_t base;
      } u_mag_y;
      u_mag_y.base = 0;
      u_mag_y.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mag_y = u_mag_y.real;
      offset += sizeof(this->mag_y);
      union {
        bool real;
        uint8_t base;
      } u_mag_z;
      u_mag_z.base = 0;
      u_mag_z.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mag_z = u_mag_z.real;
      offset += sizeof(this->mag_z);
      union {
        bool real;
        uint8_t base;
      } u_accel_x;
      u_accel_x.base = 0;
      u_accel_x.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->accel_x = u_accel_x.real;
      offset += sizeof(this->accel_x);
      union {
        bool real;
        uint8_t base;
      } u_accel_y;
      u_accel_y.base = 0;
      u_accel_y.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->accel_y = u_accel_y.real;
      offset += sizeof(this->accel_y);
      union {
        bool real;
        uint8_t base;
      } u_accel_z;
      u_accel_z.base = 0;
      u_accel_z.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->accel_z = u_accel_z.real;
      offset += sizeof(this->accel_z);
      union {
        bool real;
        uint8_t base;
      } u_mags_in_range;
      u_mags_in_range.base = 0;
      u_mags_in_range.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mags_in_range = u_mags_in_range.real;
      offset += sizeof(this->mags_in_range);
      union {
        bool real;
        uint8_t base;
      } u_accels_in_range;
      u_accels_in_range.base = 0;
      u_accels_in_range.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->accels_in_range = u_accels_in_range.real;
      offset += sizeof(this->accels_in_range);
      union {
        bool real;
        uint8_t base;
      } u_calibration;
      u_calibration.base = 0;
      u_calibration.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calibration = u_calibration.real;
      offset += sizeof(this->calibration);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgMagStatus"; };
    virtual const char * getMD5() override { return "057cf294623d5a0b037fdcc47f99e3c4"; };

  };

}
#endif
