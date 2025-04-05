#ifndef _ROS_self_balancing_robot_msgs_RobotState_h
#define _ROS_self_balancing_robot_msgs_RobotState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace self_balancing_robot_msgs
{

  class RobotState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _orientation_type;
      _orientation_type orientation;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef int16_t _left_encoder_type;
      _left_encoder_type left_encoder;
      typedef int16_t _right_encoder_type;
      _right_encoder_type right_encoder;

    RobotState():
      header(),
      orientation(),
      angular_velocity(),
      linear_acceleration(),
      left_encoder(0),
      right_encoder(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_left_encoder;
      u_left_encoder.real = this->left_encoder;
      *(outbuffer + offset + 0) = (u_left_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_right_encoder;
      u_right_encoder.real = this->right_encoder;
      *(outbuffer + offset + 0) = (u_right_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_encoder);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_left_encoder;
      u_left_encoder.base = 0;
      u_left_encoder.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_encoder = u_left_encoder.real;
      offset += sizeof(this->left_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_right_encoder;
      u_right_encoder.base = 0;
      u_right_encoder.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_encoder = u_right_encoder.real;
      offset += sizeof(this->right_encoder);
     return offset;
    }

    virtual const char * getType() override { return "self_balancing_robot_msgs/RobotState"; };
    virtual const char * getMD5() override { return "fbf2c2864ad0d1cd83f06b6bf205793c"; };

  };

}
#endif
