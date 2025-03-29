#ifndef _ROS_sbg_driver_SbgEkfStatus_h
#define _ROS_sbg_driver_SbgEkfStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgEkfStatus : public ros::Msg
  {
    public:
      typedef uint8_t _solution_mode_type;
      _solution_mode_type solution_mode;
      typedef bool _attitude_valid_type;
      _attitude_valid_type attitude_valid;
      typedef bool _heading_valid_type;
      _heading_valid_type heading_valid;
      typedef bool _velocity_valid_type;
      _velocity_valid_type velocity_valid;
      typedef bool _position_valid_type;
      _position_valid_type position_valid;
      typedef bool _vert_ref_used_type;
      _vert_ref_used_type vert_ref_used;
      typedef bool _mag_ref_used_type;
      _mag_ref_used_type mag_ref_used;
      typedef bool _gps1_vel_used_type;
      _gps1_vel_used_type gps1_vel_used;
      typedef bool _gps1_pos_used_type;
      _gps1_pos_used_type gps1_pos_used;
      typedef bool _gps1_course_used_type;
      _gps1_course_used_type gps1_course_used;
      typedef bool _gps1_hdt_used_type;
      _gps1_hdt_used_type gps1_hdt_used;
      typedef bool _gps2_vel_used_type;
      _gps2_vel_used_type gps2_vel_used;
      typedef bool _gps2_pos_used_type;
      _gps2_pos_used_type gps2_pos_used;
      typedef bool _gps2_course_used_type;
      _gps2_course_used_type gps2_course_used;
      typedef bool _gps2_hdt_used_type;
      _gps2_hdt_used_type gps2_hdt_used;
      typedef bool _odo_used_type;
      _odo_used_type odo_used;

    SbgEkfStatus():
      solution_mode(0),
      attitude_valid(0),
      heading_valid(0),
      velocity_valid(0),
      position_valid(0),
      vert_ref_used(0),
      mag_ref_used(0),
      gps1_vel_used(0),
      gps1_pos_used(0),
      gps1_course_used(0),
      gps1_hdt_used(0),
      gps2_vel_used(0),
      gps2_pos_used(0),
      gps2_course_used(0),
      gps2_hdt_used(0),
      odo_used(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->solution_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solution_mode);
      union {
        bool real;
        uint8_t base;
      } u_attitude_valid;
      u_attitude_valid.real = this->attitude_valid;
      *(outbuffer + offset + 0) = (u_attitude_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->attitude_valid);
      union {
        bool real;
        uint8_t base;
      } u_heading_valid;
      u_heading_valid.real = this->heading_valid;
      *(outbuffer + offset + 0) = (u_heading_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->heading_valid);
      union {
        bool real;
        uint8_t base;
      } u_velocity_valid;
      u_velocity_valid.real = this->velocity_valid;
      *(outbuffer + offset + 0) = (u_velocity_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->velocity_valid);
      union {
        bool real;
        uint8_t base;
      } u_position_valid;
      u_position_valid.real = this->position_valid;
      *(outbuffer + offset + 0) = (u_position_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_valid);
      union {
        bool real;
        uint8_t base;
      } u_vert_ref_used;
      u_vert_ref_used.real = this->vert_ref_used;
      *(outbuffer + offset + 0) = (u_vert_ref_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vert_ref_used);
      union {
        bool real;
        uint8_t base;
      } u_mag_ref_used;
      u_mag_ref_used.real = this->mag_ref_used;
      *(outbuffer + offset + 0) = (u_mag_ref_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mag_ref_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_vel_used;
      u_gps1_vel_used.real = this->gps1_vel_used;
      *(outbuffer + offset + 0) = (u_gps1_vel_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_vel_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_pos_used;
      u_gps1_pos_used.real = this->gps1_pos_used;
      *(outbuffer + offset + 0) = (u_gps1_pos_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_pos_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_course_used;
      u_gps1_course_used.real = this->gps1_course_used;
      *(outbuffer + offset + 0) = (u_gps1_course_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_course_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_hdt_used;
      u_gps1_hdt_used.real = this->gps1_hdt_used;
      *(outbuffer + offset + 0) = (u_gps1_hdt_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps1_hdt_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_vel_used;
      u_gps2_vel_used.real = this->gps2_vel_used;
      *(outbuffer + offset + 0) = (u_gps2_vel_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps2_vel_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_pos_used;
      u_gps2_pos_used.real = this->gps2_pos_used;
      *(outbuffer + offset + 0) = (u_gps2_pos_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps2_pos_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_course_used;
      u_gps2_course_used.real = this->gps2_course_used;
      *(outbuffer + offset + 0) = (u_gps2_course_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps2_course_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_hdt_used;
      u_gps2_hdt_used.real = this->gps2_hdt_used;
      *(outbuffer + offset + 0) = (u_gps2_hdt_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps2_hdt_used);
      union {
        bool real;
        uint8_t base;
      } u_odo_used;
      u_odo_used.real = this->odo_used;
      *(outbuffer + offset + 0) = (u_odo_used.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->odo_used);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->solution_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->solution_mode);
      union {
        bool real;
        uint8_t base;
      } u_attitude_valid;
      u_attitude_valid.base = 0;
      u_attitude_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->attitude_valid = u_attitude_valid.real;
      offset += sizeof(this->attitude_valid);
      union {
        bool real;
        uint8_t base;
      } u_heading_valid;
      u_heading_valid.base = 0;
      u_heading_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->heading_valid = u_heading_valid.real;
      offset += sizeof(this->heading_valid);
      union {
        bool real;
        uint8_t base;
      } u_velocity_valid;
      u_velocity_valid.base = 0;
      u_velocity_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->velocity_valid = u_velocity_valid.real;
      offset += sizeof(this->velocity_valid);
      union {
        bool real;
        uint8_t base;
      } u_position_valid;
      u_position_valid.base = 0;
      u_position_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->position_valid = u_position_valid.real;
      offset += sizeof(this->position_valid);
      union {
        bool real;
        uint8_t base;
      } u_vert_ref_used;
      u_vert_ref_used.base = 0;
      u_vert_ref_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vert_ref_used = u_vert_ref_used.real;
      offset += sizeof(this->vert_ref_used);
      union {
        bool real;
        uint8_t base;
      } u_mag_ref_used;
      u_mag_ref_used.base = 0;
      u_mag_ref_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mag_ref_used = u_mag_ref_used.real;
      offset += sizeof(this->mag_ref_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_vel_used;
      u_gps1_vel_used.base = 0;
      u_gps1_vel_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_vel_used = u_gps1_vel_used.real;
      offset += sizeof(this->gps1_vel_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_pos_used;
      u_gps1_pos_used.base = 0;
      u_gps1_pos_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_pos_used = u_gps1_pos_used.real;
      offset += sizeof(this->gps1_pos_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_course_used;
      u_gps1_course_used.base = 0;
      u_gps1_course_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_course_used = u_gps1_course_used.real;
      offset += sizeof(this->gps1_course_used);
      union {
        bool real;
        uint8_t base;
      } u_gps1_hdt_used;
      u_gps1_hdt_used.base = 0;
      u_gps1_hdt_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps1_hdt_used = u_gps1_hdt_used.real;
      offset += sizeof(this->gps1_hdt_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_vel_used;
      u_gps2_vel_used.base = 0;
      u_gps2_vel_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps2_vel_used = u_gps2_vel_used.real;
      offset += sizeof(this->gps2_vel_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_pos_used;
      u_gps2_pos_used.base = 0;
      u_gps2_pos_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps2_pos_used = u_gps2_pos_used.real;
      offset += sizeof(this->gps2_pos_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_course_used;
      u_gps2_course_used.base = 0;
      u_gps2_course_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps2_course_used = u_gps2_course_used.real;
      offset += sizeof(this->gps2_course_used);
      union {
        bool real;
        uint8_t base;
      } u_gps2_hdt_used;
      u_gps2_hdt_used.base = 0;
      u_gps2_hdt_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gps2_hdt_used = u_gps2_hdt_used.real;
      offset += sizeof(this->gps2_hdt_used);
      union {
        bool real;
        uint8_t base;
      } u_odo_used;
      u_odo_used.base = 0;
      u_odo_used.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->odo_used = u_odo_used.real;
      offset += sizeof(this->odo_used);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgEkfStatus"; };
    virtual const char * getMD5() override { return "779a904443daf8f46a24dc18bd3bbedb"; };

  };

}
#endif
