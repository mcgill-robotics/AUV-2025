#ifndef _ROS_sbg_driver_SbgStatusAiding_h
#define _ROS_sbg_driver_SbgStatusAiding_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgStatusAiding : public ros::Msg
  {
    public:
      typedef bool _gps1_pos_recv_type;
      _gps1_pos_recv_type gps1_pos_recv;
      typedef bool _gps1_vel_recv_type;
      _gps1_vel_recv_type gps1_vel_recv;
      typedef bool _gps1_hdt_recv_type;
      _gps1_hdt_recv_type gps1_hdt_recv;
      typedef bool _gps1_utc_recv_type;
      _gps1_utc_recv_type gps1_utc_recv;
      typedef bool _mag_recv_type;
      _mag_recv_type mag_recv;
      typedef bool _odo_recv_type;
      _odo_recv_type odo_recv;
      typedef bool _dvl_recv_type;
      _dvl_recv_type dvl_recv;

    SbgStatusAiding():
      gps1_pos_recv(0),
      gps1_vel_recv(0),
      gps1_hdt_recv(0),
      gps1_utc_recv(0),
      mag_recv(0),
      odo_recv(0),
      dvl_recv(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gps1_pos_recv;
      u_gps1_pos_recv.real = this->gps1_pos_recv;
      *(outbuffer + offset + 0) = (u_gps1_pos_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_pos_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_vel_recv;
      u_gps1_vel_recv.real = this->gps1_vel_recv;
      *(outbuffer + offset + 0) = (u_gps1_vel_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_vel_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_hdt_recv;
      u_gps1_hdt_recv.real = this->gps1_hdt_recv;
      *(outbuffer + offset + 0) = (u_gps1_hdt_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_hdt_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_utc_recv;
      u_gps1_utc_recv.real = this->gps1_utc_recv;
      *(outbuffer + offset + 0) = (u_gps1_utc_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_utc_recv);
      union {
        bool real;
        uint8_t base;
      } u_mag_recv;
      u_mag_recv.real = this->mag_recv;
      *(outbuffer + offset + 0) = (u_mag_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag_recv);
      union {
        bool real;
        uint8_t base;
      } u_odo_recv;
      u_odo_recv.real = this->odo_recv;
      *(outbuffer + offset + 0) = (u_odo_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->odo_recv);
      union {
        bool real;
        uint8_t base;
      } u_dvl_recv;
      u_dvl_recv.real = this->dvl_recv;
      *(outbuffer + offset + 0) = (u_dvl_recv.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dvl_recv);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gps1_pos_recv;
      u_gps1_pos_recv.base = 0;
      u_gps1_pos_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_pos_recv = u_gps1_pos_recv.real;
      offset += sizeof(this->gps1_pos_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_vel_recv;
      u_gps1_vel_recv.base = 0;
      u_gps1_vel_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_vel_recv = u_gps1_vel_recv.real;
      offset += sizeof(this->gps1_vel_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_hdt_recv;
      u_gps1_hdt_recv.base = 0;
      u_gps1_hdt_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_hdt_recv = u_gps1_hdt_recv.real;
      offset += sizeof(this->gps1_hdt_recv);
      union {
        bool real;
        uint8_t base;
      } u_gps1_utc_recv;
      u_gps1_utc_recv.base = 0;
      u_gps1_utc_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_utc_recv = u_gps1_utc_recv.real;
      offset += sizeof(this->gps1_utc_recv);
      union {
        bool real;
        uint8_t base;
      } u_mag_recv;
      u_mag_recv.base = 0;
      u_mag_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mag_recv = u_mag_recv.real;
      offset += sizeof(this->mag_recv);
      union {
        bool real;
        uint8_t base;
      } u_odo_recv;
      u_odo_recv.base = 0;
      u_odo_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->odo_recv = u_odo_recv.real;
      offset += sizeof(this->odo_recv);
      union {
        bool real;
        uint8_t base;
      } u_dvl_recv;
      u_dvl_recv.base = 0;
      u_dvl_recv.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dvl_recv = u_dvl_recv.real;
      offset += sizeof(this->dvl_recv);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgStatusAiding"; };
    virtual const char * getMD5() override { return "873fbacbcfa106e3f518a7cd0cce3cfb"; };

  };

}
#endif
