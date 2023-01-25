#ifndef _ROS_serp_Velocity_h
#define _ROS_serp_Velocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace serp
{

  class Velocity : public ros::Msg
  {
    public:
      typedef int8_t _vel_motor_left_type;
      _vel_motor_left_type vel_motor_left;
      typedef int8_t _vel_motor_right_type;
      _vel_motor_right_type vel_motor_right;

    Velocity():
      vel_motor_left(0),
      vel_motor_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_vel_motor_left;
      u_vel_motor_left.real = this->vel_motor_left;
      *(outbuffer + offset + 0) = (u_vel_motor_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vel_motor_left);
      union {
        int8_t real;
        uint8_t base;
      } u_vel_motor_right;
      u_vel_motor_right.real = this->vel_motor_right;
      *(outbuffer + offset + 0) = (u_vel_motor_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vel_motor_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_vel_motor_left;
      u_vel_motor_left.base = 0;
      u_vel_motor_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vel_motor_left = u_vel_motor_left.real;
      offset += sizeof(this->vel_motor_left);
      union {
        int8_t real;
        uint8_t base;
      } u_vel_motor_right;
      u_vel_motor_right.base = 0;
      u_vel_motor_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vel_motor_right = u_vel_motor_right.real;
      offset += sizeof(this->vel_motor_right);
     return offset;
    }

    virtual const char * getType() override { return "serp/Velocity"; };
    virtual const char * getMD5() override { return "7d1b2453ec28e67aef9873abf0e0dd81"; };

  };

}
#endif
