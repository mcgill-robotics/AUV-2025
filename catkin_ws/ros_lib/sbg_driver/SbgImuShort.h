#ifndef _ROS_sbg_driver_SbgImuShort_h
#define _ROS_sbg_driver_SbgImuShort_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgImuStatus.h"
#include "geometry_msgs/Vector3.h"

namespace sbg_driver
{

  class SbgImuShort : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgImuStatus _imu_status_type;
      _imu_status_type imu_status;
      typedef geometry_msgs::Vector3 _delta_velocity_type;
      _delta_velocity_type delta_velocity;
      typedef geometry_msgs::Vector3 _delta_angle_type;
      _delta_angle_type delta_angle;
      typedef int16_t _temperature_type;
      _temperature_type temperature;

    SbgImuShort():
      header(),
      time_stamp(0),
      imu_status(),
      delta_velocity(),
      delta_angle(),
      temperature(0)
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
      offset += this->delta_velocity.serialize(outbuffer + offset);
      offset += this->delta_angle.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temperature);
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
      offset += this->delta_velocity.deserialize(inbuffer + offset);
      offset += this->delta_angle.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgImuShort"; };
    virtual const char * getMD5() override { return "3be77ff9ef2640e787db71f0246e7518"; };

  };

}
#endif
