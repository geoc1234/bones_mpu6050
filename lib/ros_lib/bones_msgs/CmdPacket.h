#ifndef _ROS_bones_msgs_CmdPacket_h
#define _ROS_bones_msgs_CmdPacket_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace bones_msgs
{

  class CmdPacket : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _chksm_type;
      _chksm_type chksm;
      typedef float _Mode_type;
      _Mode_type Mode;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _angularSpeed_type;
      _angularSpeed_type angularSpeed;
      typedef float _distance_type;
      _distance_type distance;
      typedef float _heading_type;
      _heading_type heading;
      typedef float _pxTarget_type;
      _pxTarget_type pxTarget;
      typedef float _pyTarget_type;
      _pyTarget_type pyTarget;

    CmdPacket():
      header(),
      chksm(0),
      Mode(0),
      speed(0),
      angularSpeed(0),
      distance(0),
      heading(0),
      pxTarget(0),
      pyTarget(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_chksm;
      u_chksm.real = this->chksm;
      *(outbuffer + offset + 0) = (u_chksm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_chksm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_chksm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_chksm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->chksm);
      union {
        float real;
        uint32_t base;
      } u_Mode;
      u_Mode.real = this->Mode;
      *(outbuffer + offset + 0) = (u_Mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Mode);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_angularSpeed;
      u_angularSpeed.real = this->angularSpeed;
      *(outbuffer + offset + 0) = (u_angularSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angularSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angularSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angularSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angularSpeed);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_pxTarget;
      u_pxTarget.real = this->pxTarget;
      *(outbuffer + offset + 0) = (u_pxTarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pxTarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pxTarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pxTarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pxTarget);
      union {
        float real;
        uint32_t base;
      } u_pyTarget;
      u_pyTarget.real = this->pyTarget;
      *(outbuffer + offset + 0) = (u_pyTarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pyTarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pyTarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pyTarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pyTarget);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_chksm;
      u_chksm.base = 0;
      u_chksm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_chksm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_chksm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_chksm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->chksm = u_chksm.real;
      offset += sizeof(this->chksm);
      union {
        float real;
        uint32_t base;
      } u_Mode;
      u_Mode.base = 0;
      u_Mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Mode = u_Mode.real;
      offset += sizeof(this->Mode);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_angularSpeed;
      u_angularSpeed.base = 0;
      u_angularSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angularSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angularSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angularSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angularSpeed = u_angularSpeed.real;
      offset += sizeof(this->angularSpeed);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_pxTarget;
      u_pxTarget.base = 0;
      u_pxTarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pxTarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pxTarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pxTarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pxTarget = u_pxTarget.real;
      offset += sizeof(this->pxTarget);
      union {
        float real;
        uint32_t base;
      } u_pyTarget;
      u_pyTarget.base = 0;
      u_pyTarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pyTarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pyTarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pyTarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pyTarget = u_pyTarget.real;
      offset += sizeof(this->pyTarget);
     return offset;
    }

    virtual const char * getType() override { return "bones_msgs/CmdPacket"; };
    virtual const char * getMD5() override { return "76381d74c679f994742b242027f1fb99"; };

  };

}
#endif
