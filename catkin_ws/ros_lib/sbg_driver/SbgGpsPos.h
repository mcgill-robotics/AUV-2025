#ifndef _ROS_sbg_driver_SbgGpsPos_h
#define _ROS_sbg_driver_SbgGpsPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgGpsPosStatus.h"
#include "geometry_msgs/Vector3.h"

namespace sbg_driver
{

  class SbgGpsPos : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgGpsPosStatus _status_type;
      _status_type status;
      typedef uint32_t _gps_tow_type;
      _gps_tow_type gps_tow;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _undulation_type;
      _undulation_type undulation;
      typedef geometry_msgs::Vector3 _position_accuracy_type;
      _position_accuracy_type position_accuracy;
      typedef uint8_t _num_sv_used_type;
      _num_sv_used_type num_sv_used;
      typedef uint16_t _base_station_id_type;
      _base_station_id_type base_station_id;
      typedef uint16_t _diff_age_type;
      _diff_age_type diff_age;

    SbgGpsPos():
      header(),
      time_stamp(0),
      status(),
      gps_tow(0),
      latitude(0),
      longitude(0),
      altitude(0),
      undulation(0),
      position_accuracy(),
      num_sv_used(0),
      base_station_id(0),
      diff_age(0)
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
      *(outbuffer + offset + 0) = (this->gps_tow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gps_tow >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gps_tow >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gps_tow >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_tow);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      union {
        float real;
        uint32_t base;
      } u_undulation;
      u_undulation.real = this->undulation;
      *(outbuffer + offset + 0) = (u_undulation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_undulation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_undulation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_undulation.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->undulation);
      offset += this->position_accuracy.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->num_sv_used >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_sv_used);
      *(outbuffer + offset + 0) = (this->base_station_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_station_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->base_station_id);
      *(outbuffer + offset + 0) = (this->diff_age >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->diff_age >> (8 * 1)) & 0xFF;
      offset += sizeof(this->diff_age);
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
      this->gps_tow =  ((uint32_t) (*(inbuffer + offset)));
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gps_tow |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gps_tow);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      union {
        float real;
        uint32_t base;
      } u_undulation;
      u_undulation.base = 0;
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_undulation.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->undulation = u_undulation.real;
      offset += sizeof(this->undulation);
      offset += this->position_accuracy.deserialize(inbuffer + offset);
      this->num_sv_used =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->num_sv_used);
      this->base_station_id =  ((uint16_t) (*(inbuffer + offset)));
      this->base_station_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->base_station_id);
      this->diff_age =  ((uint16_t) (*(inbuffer + offset)));
      this->diff_age |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->diff_age);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgGpsPos"; };
    virtual const char * getMD5() override { return "6b214c87825603003c01f4e03d945a32"; };

  };

}
#endif
