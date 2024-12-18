#ifndef _ROS_self_balancing_robot_msgs_PIDParams_h
#define _ROS_self_balancing_robot_msgs_PIDParams_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace self_balancing_robot_msgs
{

  class PIDParams : public ros::Msg
  {
    public:
      typedef float _kp_type;
      _kp_type kp;
      typedef float _ki_type;
      _ki_type ki;
      typedef float _kd_type;
      _kd_type kd;
      typedef float _kp_pos_type;
      _kp_pos_type kp_pos;
      typedef float _ki_pos_type;
      _ki_pos_type ki_pos;
      typedef float _kd_pos_type;
      _kd_pos_type kd_pos;

    PIDParams():
      kp(0),
      ki(0),
      kd(0),
      kp_pos(0),
      ki_pos(0),
      kd_pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd);
      union {
        float real;
        uint32_t base;
      } u_kp_pos;
      u_kp_pos.real = this->kp_pos;
      *(outbuffer + offset + 0) = (u_kp_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp_pos);
      union {
        float real;
        uint32_t base;
      } u_ki_pos;
      u_ki_pos.real = this->ki_pos;
      *(outbuffer + offset + 0) = (u_ki_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki_pos);
      union {
        float real;
        uint32_t base;
      } u_kd_pos;
      u_kd_pos.real = this->kd_pos;
      *(outbuffer + offset + 0) = (u_kd_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd_pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
      union {
        float real;
        uint32_t base;
      } u_kp_pos;
      u_kp_pos.base = 0;
      u_kp_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp_pos = u_kp_pos.real;
      offset += sizeof(this->kp_pos);
      union {
        float real;
        uint32_t base;
      } u_ki_pos;
      u_ki_pos.base = 0;
      u_ki_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki_pos = u_ki_pos.real;
      offset += sizeof(this->ki_pos);
      union {
        float real;
        uint32_t base;
      } u_kd_pos;
      u_kd_pos.base = 0;
      u_kd_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd_pos = u_kd_pos.real;
      offset += sizeof(this->kd_pos);
     return offset;
    }

    virtual const char * getType() override { return "self_balancing_robot_msgs/PIDParams"; };
    virtual const char * getMD5() override { return "a301d29402c6428d86db47afe6dfe52a"; };

  };

}
#endif
