#ifndef _ROS_auv_msgs_UnityState_h
#define _ROS_auv_msgs_UnityState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace auv_msgs
{

  class UnityState : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      uint32_t frequencies_length;
      typedef int32_t _frequencies_type;
      _frequencies_type st_frequencies;
      _frequencies_type * frequencies;
      uint32_t times_pinger_1_length;
      typedef uint32_t _times_pinger_1_type;
      _times_pinger_1_type st_times_pinger_1;
      _times_pinger_1_type * times_pinger_1;
      uint32_t times_pinger_2_length;
      typedef uint32_t _times_pinger_2_type;
      _times_pinger_2_type st_times_pinger_2;
      _times_pinger_2_type * times_pinger_2;
      uint32_t times_pinger_3_length;
      typedef uint32_t _times_pinger_3_type;
      _times_pinger_3_type st_times_pinger_3;
      _times_pinger_3_type * times_pinger_3;
      uint32_t times_pinger_4_length;
      typedef uint32_t _times_pinger_4_type;
      _times_pinger_4_type st_times_pinger_4;
      _times_pinger_4_type * times_pinger_4;
      typedef int32_t _isDVLActive_type;
      _isDVLActive_type isDVLActive;
      typedef int32_t _isDepthSensorActive_type;
      _isDepthSensorActive_type isDepthSensorActive;
      typedef int32_t _isIMUActive_type;
      _isIMUActive_type isIMUActive;
      typedef int32_t _isHydrophonesActive_type;
      _isHydrophonesActive_type isHydrophonesActive;

    UnityState():
      position(),
      orientation(),
      velocity(),
      angular_velocity(),
      linear_acceleration(),
      frequencies_length(0), st_frequencies(), frequencies(nullptr),
      times_pinger_1_length(0), st_times_pinger_1(), times_pinger_1(nullptr),
      times_pinger_2_length(0), st_times_pinger_2(), times_pinger_2(nullptr),
      times_pinger_3_length(0), st_times_pinger_3(), times_pinger_3(nullptr),
      times_pinger_4_length(0), st_times_pinger_4(), times_pinger_4(nullptr),
      isDVLActive(0),
      isDepthSensorActive(0),
      isIMUActive(0),
      isHydrophonesActive(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->frequencies_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frequencies_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->frequencies_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->frequencies_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequencies_length);
      for( uint32_t i = 0; i < frequencies_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_frequenciesi;
      u_frequenciesi.real = this->frequencies[i];
      *(outbuffer + offset + 0) = (u_frequenciesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequenciesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frequenciesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frequenciesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequencies[i]);
      }
      *(outbuffer + offset + 0) = (this->times_pinger_1_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_1_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_1_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_1_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_1_length);
      for( uint32_t i = 0; i < times_pinger_1_length; i++){
      *(outbuffer + offset + 0) = (this->times_pinger_1[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_1[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_1[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_1[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_1[i]);
      }
      *(outbuffer + offset + 0) = (this->times_pinger_2_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_2_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_2_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_2_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_2_length);
      for( uint32_t i = 0; i < times_pinger_2_length; i++){
      *(outbuffer + offset + 0) = (this->times_pinger_2[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_2[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_2[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_2[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_2[i]);
      }
      *(outbuffer + offset + 0) = (this->times_pinger_3_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_3_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_3_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_3_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_3_length);
      for( uint32_t i = 0; i < times_pinger_3_length; i++){
      *(outbuffer + offset + 0) = (this->times_pinger_3[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_3[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_3[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_3[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_3[i]);
      }
      *(outbuffer + offset + 0) = (this->times_pinger_4_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_4_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_4_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_4_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_4_length);
      for( uint32_t i = 0; i < times_pinger_4_length; i++){
      *(outbuffer + offset + 0) = (this->times_pinger_4[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times_pinger_4[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->times_pinger_4[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->times_pinger_4[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->times_pinger_4[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_isDVLActive;
      u_isDVLActive.real = this->isDVLActive;
      *(outbuffer + offset + 0) = (u_isDVLActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isDVLActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isDVLActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isDVLActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isDVLActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isDepthSensorActive;
      u_isDepthSensorActive.real = this->isDepthSensorActive;
      *(outbuffer + offset + 0) = (u_isDepthSensorActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isDepthSensorActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isDepthSensorActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isDepthSensorActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isDepthSensorActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isIMUActive;
      u_isIMUActive.real = this->isIMUActive;
      *(outbuffer + offset + 0) = (u_isIMUActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isIMUActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isIMUActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isIMUActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isIMUActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isHydrophonesActive;
      u_isHydrophonesActive.real = this->isHydrophonesActive;
      *(outbuffer + offset + 0) = (u_isHydrophonesActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isHydrophonesActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isHydrophonesActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isHydrophonesActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isHydrophonesActive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      uint32_t frequencies_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      frequencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->frequencies_length);
      if(frequencies_lengthT > frequencies_length)
        this->frequencies = (int32_t*)realloc(this->frequencies, frequencies_lengthT * sizeof(int32_t));
      frequencies_length = frequencies_lengthT;
      for( uint32_t i = 0; i < frequencies_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_frequencies;
      u_st_frequencies.base = 0;
      u_st_frequencies.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_frequencies.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_frequencies.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_frequencies.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_frequencies = u_st_frequencies.real;
      offset += sizeof(this->st_frequencies);
        memcpy( &(this->frequencies[i]), &(this->st_frequencies), sizeof(int32_t));
      }
      uint32_t times_pinger_1_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_pinger_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_pinger_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_pinger_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_pinger_1_length);
      if(times_pinger_1_lengthT > times_pinger_1_length)
        this->times_pinger_1 = (uint32_t*)realloc(this->times_pinger_1, times_pinger_1_lengthT * sizeof(uint32_t));
      times_pinger_1_length = times_pinger_1_lengthT;
      for( uint32_t i = 0; i < times_pinger_1_length; i++){
      this->st_times_pinger_1 =  ((uint32_t) (*(inbuffer + offset)));
      this->st_times_pinger_1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_times_pinger_1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_times_pinger_1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_times_pinger_1);
        memcpy( &(this->times_pinger_1[i]), &(this->st_times_pinger_1), sizeof(uint32_t));
      }
      uint32_t times_pinger_2_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_pinger_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_pinger_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_pinger_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_pinger_2_length);
      if(times_pinger_2_lengthT > times_pinger_2_length)
        this->times_pinger_2 = (uint32_t*)realloc(this->times_pinger_2, times_pinger_2_lengthT * sizeof(uint32_t));
      times_pinger_2_length = times_pinger_2_lengthT;
      for( uint32_t i = 0; i < times_pinger_2_length; i++){
      this->st_times_pinger_2 =  ((uint32_t) (*(inbuffer + offset)));
      this->st_times_pinger_2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_times_pinger_2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_times_pinger_2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_times_pinger_2);
        memcpy( &(this->times_pinger_2[i]), &(this->st_times_pinger_2), sizeof(uint32_t));
      }
      uint32_t times_pinger_3_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_pinger_3_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_pinger_3_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_pinger_3_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_pinger_3_length);
      if(times_pinger_3_lengthT > times_pinger_3_length)
        this->times_pinger_3 = (uint32_t*)realloc(this->times_pinger_3, times_pinger_3_lengthT * sizeof(uint32_t));
      times_pinger_3_length = times_pinger_3_lengthT;
      for( uint32_t i = 0; i < times_pinger_3_length; i++){
      this->st_times_pinger_3 =  ((uint32_t) (*(inbuffer + offset)));
      this->st_times_pinger_3 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_times_pinger_3 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_times_pinger_3 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_times_pinger_3);
        memcpy( &(this->times_pinger_3[i]), &(this->st_times_pinger_3), sizeof(uint32_t));
      }
      uint32_t times_pinger_4_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      times_pinger_4_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      times_pinger_4_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      times_pinger_4_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->times_pinger_4_length);
      if(times_pinger_4_lengthT > times_pinger_4_length)
        this->times_pinger_4 = (uint32_t*)realloc(this->times_pinger_4, times_pinger_4_lengthT * sizeof(uint32_t));
      times_pinger_4_length = times_pinger_4_lengthT;
      for( uint32_t i = 0; i < times_pinger_4_length; i++){
      this->st_times_pinger_4 =  ((uint32_t) (*(inbuffer + offset)));
      this->st_times_pinger_4 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_times_pinger_4 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_times_pinger_4 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_times_pinger_4);
        memcpy( &(this->times_pinger_4[i]), &(this->st_times_pinger_4), sizeof(uint32_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_isDVLActive;
      u_isDVLActive.base = 0;
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isDVLActive = u_isDVLActive.real;
      offset += sizeof(this->isDVLActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isDepthSensorActive;
      u_isDepthSensorActive.base = 0;
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isDepthSensorActive = u_isDepthSensorActive.real;
      offset += sizeof(this->isDepthSensorActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isIMUActive;
      u_isIMUActive.base = 0;
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isIMUActive = u_isIMUActive.real;
      offset += sizeof(this->isIMUActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isHydrophonesActive;
      u_isHydrophonesActive.base = 0;
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isHydrophonesActive = u_isHydrophonesActive.real;
      offset += sizeof(this->isHydrophonesActive);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/UnityState"; };
    virtual const char * getMD5() override { return "eafbb66a66e394412e0b607bd4a7ddd3"; };

  };

}
#endif
