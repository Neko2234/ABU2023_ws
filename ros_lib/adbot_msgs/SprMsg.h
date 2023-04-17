#ifndef _ROS_adbot_msgs_SprMsg_h
#define _ROS_adbot_msgs_SprMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace adbot_msgs
{

  class SprMsg : public ros::Msg
  {
    public:
      typedef bool _isOn_type;
      _isOn_type isOn;
      typedef int16_t _duty_type;
      _duty_type duty;
      typedef int16_t _manual_type;
      _manual_type manual;

    SprMsg():
      isOn(0),
      duty(0),
      manual(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isOn;
      u_isOn.real = this->isOn;
      *(outbuffer + offset + 0) = (u_isOn.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isOn);
      union {
        int16_t real;
        uint16_t base;
      } u_duty;
      u_duty.real = this->duty;
      *(outbuffer + offset + 0) = (u_duty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duty.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->duty);
      union {
        int16_t real;
        uint16_t base;
      } u_manual;
      u_manual.real = this->manual;
      *(outbuffer + offset + 0) = (u_manual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_manual.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->manual);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isOn;
      u_isOn.base = 0;
      u_isOn.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isOn = u_isOn.real;
      offset += sizeof(this->isOn);
      union {
        int16_t real;
        uint16_t base;
      } u_duty;
      u_duty.base = 0;
      u_duty.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duty.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duty = u_duty.real;
      offset += sizeof(this->duty);
      union {
        int16_t real;
        uint16_t base;
      } u_manual;
      u_manual.base = 0;
      u_manual.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_manual.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->manual = u_manual.real;
      offset += sizeof(this->manual);
     return offset;
    }

    virtual const char * getType() override { return "adbot_msgs/SprMsg"; };
    virtual const char * getMD5() override { return "ca3c8405dd8dce3448bc9c869b874ea8"; };

  };

}
#endif
