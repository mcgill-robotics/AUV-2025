#ifndef _ROS_sbg_driver_SbgGpsPosStatus_h
#define _ROS_sbg_driver_SbgGpsPosStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgGpsPosStatus : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      typedef uint8_t _type_type;
      _type_type type;
      typedef bool _gps_l1_used_type;
      _gps_l1_used_type gps_l1_used;
      typedef bool _gps_l2_used_type;
      _gps_l2_used_type gps_l2_used;
      typedef bool _gps_l5_used_type;
      _gps_l5_used_type gps_l5_used;
      typedef bool _glo_l1_used_type;
      _glo_l1_used_type glo_l1_used;
      typedef bool _glo_l2_used_type;
      _glo_l2_used_type glo_l2_used;

    SbgGpsPosStatus():
      status(0),
      type(0),
      gps_l1_used(0),
      gps_l2_used(0),
      gps_l5_used(0),
      glo_l1_used(0),
      glo_l2_used(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_gps_l1_used;
      u_gps_l1_used.real = this->gps_l1_used;
      *(outbuffer + offset + 0) = (u_gps_l1_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_l1_used);
      union {
        bool real;
        uint8_t base;
      } u_gps_l2_used;
      u_gps_l2_used.real = this->gps_l2_used;
      *(outbuffer + offset + 0) = (u_gps_l2_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_l2_used);
      union {
        bool real;
        uint8_t base;
      } u_gps_l5_used;
      u_gps_l5_used.real = this->gps_l5_used;
      *(outbuffer + offset + 0) = (u_gps_l5_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_l5_used);
      union {
        bool real;
        uint8_t base;
      } u_glo_l1_used;
      u_glo_l1_used.real = this->glo_l1_used;
      *(outbuffer + offset + 0) = (u_glo_l1_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->glo_l1_used);
      union {
        bool real;
        uint8_t base;
      } u_glo_l2_used;
      u_glo_l2_used.real = this->glo_l2_used;
      *(outbuffer + offset + 0) = (u_glo_l2_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->glo_l2_used);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_gps_l1_used;
      u_gps_l1_used.base = 0;
      u_gps_l1_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps_l1_used = u_gps_l1_used.real;
      offset += sizeof(this->gps_l1_used);
      union {
        bool real;
        uint8_t base;
      } u_gps_l2_used;
      u_gps_l2_used.base = 0;
      u_gps_l2_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps_l2_used = u_gps_l2_used.real;
      offset += sizeof(this->gps_l2_used);
      union {
        bool real;
        uint8_t base;
      } u_gps_l5_used;
      u_gps_l5_used.base = 0;
      u_gps_l5_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps_l5_used = u_gps_l5_used.real;
      offset += sizeof(this->gps_l5_used);
      union {
        bool real;
        uint8_t base;
      } u_glo_l1_used;
      u_glo_l1_used.base = 0;
      u_glo_l1_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->glo_l1_used = u_glo_l1_used.real;
      offset += sizeof(this->glo_l1_used);
      union {
        bool real;
        uint8_t base;
      } u_glo_l2_used;
      u_glo_l2_used.base = 0;
      u_glo_l2_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->glo_l2_used = u_glo_l2_used.real;
      offset += sizeof(this->glo_l2_used);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgGpsPosStatus"; };
    virtual const char * getMD5() override { return "85506deb3699c6f0e87097da56884a7e"; };

  };

}
#endif
