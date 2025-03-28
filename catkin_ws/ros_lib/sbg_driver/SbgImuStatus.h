#ifndef _ROS_sbg_driver_SbgImuStatus_h
#define _ROS_sbg_driver_SbgImuStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgImuStatus : public ros::Msg
  {
    public:
      typedef bool _imu_com_type;
      _imu_com_type imu_com;
      typedef bool _imu_status_type;
      _imu_status_type imu_status;
      typedef bool _imu_accel_x_type;
      _imu_accel_x_type imu_accel_x;
      typedef bool _imu_accel_y_type;
      _imu_accel_y_type imu_accel_y;
      typedef bool _imu_accel_z_type;
      _imu_accel_z_type imu_accel_z;
      typedef bool _imu_gyro_x_type;
      _imu_gyro_x_type imu_gyro_x;
      typedef bool _imu_gyro_y_type;
      _imu_gyro_y_type imu_gyro_y;
      typedef bool _imu_gyro_z_type;
      _imu_gyro_z_type imu_gyro_z;
      typedef bool _imu_accels_in_range_type;
      _imu_accels_in_range_type imu_accels_in_range;
      typedef bool _imu_gyros_in_range_type;
      _imu_gyros_in_range_type imu_gyros_in_range;

    SbgImuStatus():
      imu_com(0),
      imu_status(0),
      imu_accel_x(0),
      imu_accel_y(0),
      imu_accel_z(0),
      imu_gyro_x(0),
      imu_gyro_y(0),
      imu_gyro_z(0),
      imu_accels_in_range(0),
      imu_gyros_in_range(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_imu_com;
      u_imu_com.real = this->imu_com;
      *(outbuffer + offset + 0) = (u_imu_com.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_com);
      union {
        bool real;
        uint8_t base;
      } u_imu_status;
      u_imu_status.real = this->imu_status;
      *(outbuffer + offset + 0) = (u_imu_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_status);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_x;
      u_imu_accel_x.real = this->imu_accel_x;
      *(outbuffer + offset + 0) = (u_imu_accel_x.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_accel_x);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_y;
      u_imu_accel_y.real = this->imu_accel_y;
      *(outbuffer + offset + 0) = (u_imu_accel_y.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_accel_y);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_z;
      u_imu_accel_z.real = this->imu_accel_z;
      *(outbuffer + offset + 0) = (u_imu_accel_z.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_accel_z);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_x;
      u_imu_gyro_x.real = this->imu_gyro_x;
      *(outbuffer + offset + 0) = (u_imu_gyro_x.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_gyro_x);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_y;
      u_imu_gyro_y.real = this->imu_gyro_y;
      *(outbuffer + offset + 0) = (u_imu_gyro_y.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_gyro_y);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_z;
      u_imu_gyro_z.real = this->imu_gyro_z;
      *(outbuffer + offset + 0) = (u_imu_gyro_z.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_gyro_z);
      union {
        bool real;
        uint8_t base;
      } u_imu_accels_in_range;
      u_imu_accels_in_range.real = this->imu_accels_in_range;
      *(outbuffer + offset + 0) = (u_imu_accels_in_range.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_accels_in_range);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyros_in_range;
      u_imu_gyros_in_range.real = this->imu_gyros_in_range;
      *(outbuffer + offset + 0) = (u_imu_gyros_in_range.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_gyros_in_range);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_imu_com;
      u_imu_com.base = 0;
      u_imu_com.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_com = u_imu_com.real;
      offset += sizeof(this->imu_com);
      union {
        bool real;
        uint8_t base;
      } u_imu_status;
      u_imu_status.base = 0;
      u_imu_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_status = u_imu_status.real;
      offset += sizeof(this->imu_status);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_x;
      u_imu_accel_x.base = 0;
      u_imu_accel_x.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_accel_x = u_imu_accel_x.real;
      offset += sizeof(this->imu_accel_x);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_y;
      u_imu_accel_y.base = 0;
      u_imu_accel_y.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_accel_y = u_imu_accel_y.real;
      offset += sizeof(this->imu_accel_y);
      union {
        bool real;
        uint8_t base;
      } u_imu_accel_z;
      u_imu_accel_z.base = 0;
      u_imu_accel_z.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_accel_z = u_imu_accel_z.real;
      offset += sizeof(this->imu_accel_z);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_x;
      u_imu_gyro_x.base = 0;
      u_imu_gyro_x.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_gyro_x = u_imu_gyro_x.real;
      offset += sizeof(this->imu_gyro_x);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_y;
      u_imu_gyro_y.base = 0;
      u_imu_gyro_y.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_gyro_y = u_imu_gyro_y.real;
      offset += sizeof(this->imu_gyro_y);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyro_z;
      u_imu_gyro_z.base = 0;
      u_imu_gyro_z.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_gyro_z = u_imu_gyro_z.real;
      offset += sizeof(this->imu_gyro_z);
      union {
        bool real;
        uint8_t base;
      } u_imu_accels_in_range;
      u_imu_accels_in_range.base = 0;
      u_imu_accels_in_range.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_accels_in_range = u_imu_accels_in_range.real;
      offset += sizeof(this->imu_accels_in_range);
      union {
        bool real;
        uint8_t base;
      } u_imu_gyros_in_range;
      u_imu_gyros_in_range.base = 0;
      u_imu_gyros_in_range.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_gyros_in_range = u_imu_gyros_in_range.real;
      offset += sizeof(this->imu_gyros_in_range);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgImuStatus"; };
    virtual const char * getMD5() override { return "e9a1bd33215fe69b9523e359cacbe127"; };

  };

}
#endif
