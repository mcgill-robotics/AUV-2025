#ifndef _ROS_auv_msgs_VelocityReport_h
#define _ROS_auv_msgs_VelocityReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class VelocityReport : public ros::Msg
  {
    public:
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _vz_type;
      _vz_type vz;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef bool _valid_type;
      _valid_type valid;
      typedef float _fom_type;
      _fom_type fom;
      uint32_t covariance_length;
      typedef float _covariance_type;
      _covariance_type st_covariance;
      _covariance_type * covariance;
      typedef float _time_of_validity_type;
      _time_of_validity_type time_of_validity;
      typedef float _time_of_transmission_type;
      _time_of_transmission_type time_of_transmission;
      typedef float _time_type;
      _time_type time;
      typedef bool _status_type;
      _status_type status;

    VelocityReport():
      vx(0),
      vy(0),
      vz(0),
      altitude(0),
      valid(0),
      fom(0),
      covariance_length(0), st_covariance(), covariance(nullptr),
      time_of_validity(0),
      time_of_transmission(0),
      time(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->vz);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.real = this->valid;
      *(outbuffer + offset + 0) = (u_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      offset += serializeAvrFloat64(outbuffer + offset, this->fom);
      *(outbuffer + offset + 0) = (this->covariance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->covariance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->covariance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->covariance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->covariance_length);
      for( uint32_t i = 0; i < covariance_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->time_of_validity);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_of_transmission);
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.base = 0;
      u_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->valid = u_valid.real;
      offset += sizeof(this->valid);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fom));
      uint32_t covariance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->covariance_length);
      if(covariance_lengthT > covariance_length)
        this->covariance = (float*)realloc(this->covariance, covariance_lengthT * sizeof(float));
      covariance_length = covariance_lengthT;
      for( uint32_t i = 0; i < covariance_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_covariance));
        memcpy( &(this->covariance[i]), &(this->st_covariance), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_of_validity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_of_transmission));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/VelocityReport"; };
    virtual const char * getMD5() override { return "673a7d5c3a5d7647c78b173ad54c86b6"; };

  };

}
#endif
