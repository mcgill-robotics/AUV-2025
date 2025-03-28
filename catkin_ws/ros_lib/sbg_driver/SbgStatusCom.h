#ifndef _ROS_sbg_driver_SbgStatusCom_h
#define _ROS_sbg_driver_SbgStatusCom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sbg_driver
{

  class SbgStatusCom : public ros::Msg
  {
    public:
      typedef bool _port_a_type;
      _port_a_type port_a;
      typedef bool _port_b_type;
      _port_b_type port_b;
      typedef bool _port_c_type;
      _port_c_type port_c;
      typedef bool _port_d_type;
      _port_d_type port_d;
      typedef bool _port_e_type;
      _port_e_type port_e;
      typedef bool _port_a_rx_type;
      _port_a_rx_type port_a_rx;
      typedef bool _port_a_tx_type;
      _port_a_tx_type port_a_tx;
      typedef bool _port_b_rx_type;
      _port_b_rx_type port_b_rx;
      typedef bool _port_b_tx_type;
      _port_b_tx_type port_b_tx;
      typedef bool _port_c_rx_type;
      _port_c_rx_type port_c_rx;
      typedef bool _port_c_tx_type;
      _port_c_tx_type port_c_tx;
      typedef bool _port_d_rx_type;
      _port_d_rx_type port_d_rx;
      typedef bool _port_d_tx_type;
      _port_d_tx_type port_d_tx;
      typedef bool _port_e_rx_type;
      _port_e_rx_type port_e_rx;
      typedef bool _port_e_tx_type;
      _port_e_tx_type port_e_tx;
      typedef bool _can_rx_type;
      _can_rx_type can_rx;
      typedef bool _can_tx_type;
      _can_tx_type can_tx;
      typedef uint8_t _can_status_type;
      _can_status_type can_status;

    SbgStatusCom():
      port_a(0),
      port_b(0),
      port_c(0),
      port_d(0),
      port_e(0),
      port_a_rx(0),
      port_a_tx(0),
      port_b_rx(0),
      port_b_tx(0),
      port_c_rx(0),
      port_c_tx(0),
      port_d_rx(0),
      port_d_tx(0),
      port_e_rx(0),
      port_e_tx(0),
      can_rx(0),
      can_tx(0),
      can_status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_port_a;
      u_port_a.real = this->port_a;
      *(outbuffer + offset + 0) = (u_port_a.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_a);
      union {
        bool real;
        uint8_t base;
      } u_port_b;
      u_port_b.real = this->port_b;
      *(outbuffer + offset + 0) = (u_port_b.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_b);
      union {
        bool real;
        uint8_t base;
      } u_port_c;
      u_port_c.real = this->port_c;
      *(outbuffer + offset + 0) = (u_port_c.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_c);
      union {
        bool real;
        uint8_t base;
      } u_port_d;
      u_port_d.real = this->port_d;
      *(outbuffer + offset + 0) = (u_port_d.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_d);
      union {
        bool real;
        uint8_t base;
      } u_port_e;
      u_port_e.real = this->port_e;
      *(outbuffer + offset + 0) = (u_port_e.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_e);
      union {
        bool real;
        uint8_t base;
      } u_port_a_rx;
      u_port_a_rx.real = this->port_a_rx;
      *(outbuffer + offset + 0) = (u_port_a_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_a_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_a_tx;
      u_port_a_tx.real = this->port_a_tx;
      *(outbuffer + offset + 0) = (u_port_a_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_a_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_b_rx;
      u_port_b_rx.real = this->port_b_rx;
      *(outbuffer + offset + 0) = (u_port_b_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_b_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_b_tx;
      u_port_b_tx.real = this->port_b_tx;
      *(outbuffer + offset + 0) = (u_port_b_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_b_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_c_rx;
      u_port_c_rx.real = this->port_c_rx;
      *(outbuffer + offset + 0) = (u_port_c_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_c_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_c_tx;
      u_port_c_tx.real = this->port_c_tx;
      *(outbuffer + offset + 0) = (u_port_c_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_c_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_d_rx;
      u_port_d_rx.real = this->port_d_rx;
      *(outbuffer + offset + 0) = (u_port_d_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_d_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_d_tx;
      u_port_d_tx.real = this->port_d_tx;
      *(outbuffer + offset + 0) = (u_port_d_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_d_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_e_rx;
      u_port_e_rx.real = this->port_e_rx;
      *(outbuffer + offset + 0) = (u_port_e_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_e_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_e_tx;
      u_port_e_tx.real = this->port_e_tx;
      *(outbuffer + offset + 0) = (u_port_e_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->port_e_tx);
      union {
        bool real;
        uint8_t base;
      } u_can_rx;
      u_can_rx.real = this->can_rx;
      *(outbuffer + offset + 0) = (u_can_rx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->can_rx);
      union {
        bool real;
        uint8_t base;
      } u_can_tx;
      u_can_tx.real = this->can_tx;
      *(outbuffer + offset + 0) = (u_can_tx.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->can_tx);
      *(outbuffer + offset + 0) = (this->can_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->can_status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_port_a;
      u_port_a.base = 0;
      u_port_a.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_a = u_port_a.real;
      offset += sizeof(this->port_a);
      union {
        bool real;
        uint8_t base;
      } u_port_b;
      u_port_b.base = 0;
      u_port_b.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_b = u_port_b.real;
      offset += sizeof(this->port_b);
      union {
        bool real;
        uint8_t base;
      } u_port_c;
      u_port_c.base = 0;
      u_port_c.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_c = u_port_c.real;
      offset += sizeof(this->port_c);
      union {
        bool real;
        uint8_t base;
      } u_port_d;
      u_port_d.base = 0;
      u_port_d.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_d = u_port_d.real;
      offset += sizeof(this->port_d);
      union {
        bool real;
        uint8_t base;
      } u_port_e;
      u_port_e.base = 0;
      u_port_e.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_e = u_port_e.real;
      offset += sizeof(this->port_e);
      union {
        bool real;
        uint8_t base;
      } u_port_a_rx;
      u_port_a_rx.base = 0;
      u_port_a_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_a_rx = u_port_a_rx.real;
      offset += sizeof(this->port_a_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_a_tx;
      u_port_a_tx.base = 0;
      u_port_a_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_a_tx = u_port_a_tx.real;
      offset += sizeof(this->port_a_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_b_rx;
      u_port_b_rx.base = 0;
      u_port_b_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_b_rx = u_port_b_rx.real;
      offset += sizeof(this->port_b_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_b_tx;
      u_port_b_tx.base = 0;
      u_port_b_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_b_tx = u_port_b_tx.real;
      offset += sizeof(this->port_b_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_c_rx;
      u_port_c_rx.base = 0;
      u_port_c_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_c_rx = u_port_c_rx.real;
      offset += sizeof(this->port_c_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_c_tx;
      u_port_c_tx.base = 0;
      u_port_c_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_c_tx = u_port_c_tx.real;
      offset += sizeof(this->port_c_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_d_rx;
      u_port_d_rx.base = 0;
      u_port_d_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_d_rx = u_port_d_rx.real;
      offset += sizeof(this->port_d_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_d_tx;
      u_port_d_tx.base = 0;
      u_port_d_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_d_tx = u_port_d_tx.real;
      offset += sizeof(this->port_d_tx);
      union {
        bool real;
        uint8_t base;
      } u_port_e_rx;
      u_port_e_rx.base = 0;
      u_port_e_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_e_rx = u_port_e_rx.real;
      offset += sizeof(this->port_e_rx);
      union {
        bool real;
        uint8_t base;
      } u_port_e_tx;
      u_port_e_tx.base = 0;
      u_port_e_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->port_e_tx = u_port_e_tx.real;
      offset += sizeof(this->port_e_tx);
      union {
        bool real;
        uint8_t base;
      } u_can_rx;
      u_can_rx.base = 0;
      u_can_rx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->can_rx = u_can_rx.real;
      offset += sizeof(this->can_rx);
      union {
        bool real;
        uint8_t base;
      } u_can_tx;
      u_can_tx.base = 0;
      u_can_tx.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->can_tx = u_can_tx.real;
      offset += sizeof(this->can_tx);
      this->can_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->can_status);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgStatusCom"; };
    virtual const char * getMD5() override { return "0586194daf83121bc54eda7bece4880f"; };

  };

}
#endif
