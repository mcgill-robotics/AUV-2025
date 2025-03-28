#ifndef _ROS_sbg_driver_SbgImuData_h
#define _ROS_sbg_driver_SbgImuData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgImuStatus.h"
#include "geometry_msgs/Vector3.h"

namespace sbg_driver
{

  class SbgImuData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgImuStatus _imu_status_type;
      _imu_status_type imu_status;
      typedef geometry_msgs::Vector3 _accel_type;
      _accel_type accel;
      typedef geometry_msgs::Vector3 _gyro_type;
      _gyro_type gyro;
      typedef float _temp_type;
      _temp_type temp;
      typedef geometry_msgs::Vector3 _delta_vel_type;
      _delta_vel_type delta_vel;
      typedef geometry_msgs::Vector3 _delta_angle_type;
      _delta_angle_type delta_angle;

    SbgImuData():
      header(),
      time_stamp(0),
      imu_status(),
      accel(),
      gyro(),
      temp(0),
      delta_vel(),
      delta_angle()
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
      offset += this->imu_status.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      offset += this->gyro.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.real = this->temp;
      *(outbuffer + offset + 0) = (u_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp);
      offset += this->delta_vel.serialize(outbuffer + offset);
      offset += this->delta_angle.serialize(outbuffer + offset);
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
      offset += this->imu_status.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
      offset += this->gyro.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.base = 0;
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp = u_temp.real;
      offset += sizeof(this->temp);
      offset += this->delta_vel.deserialize(inbuffer + offset);
      offset += this->delta_angle.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgImuData"; };
    virtual const char * getMD5() override { return "59cc541d794c367e71030fa700720826"; };

  };

}
#endif
