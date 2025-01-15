#ifndef _ROS_flight_stack_DroneInfo_h
#define _ROS_flight_stack_DroneInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"

namespace flight_stack
{

  class DroneInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef mavros_msgs::State _state_type;
      _state_type state;
      typedef mavros_msgs::ExtendedState _extended_state_type;
      _extended_state_type extended_state;
      typedef sensor_msgs::NavSatFix _abs_pose_type;
      _abs_pose_type abs_pose;
      typedef geographic_msgs::GeoPoseStamped _current_pose_type;
      _current_pose_type current_pose;
      typedef std_msgs::Float64 _compass_type;
      _compass_type compass;
      typedef sensor_msgs::Imu _imu_type;
      _imu_type imu;

    DroneInfo():
      header(),
      state(),
      extended_state(),
      abs_pose(),
      current_pose(),
      compass(),
      imu()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->state.serialize(outbuffer + offset);
      offset += this->extended_state.serialize(outbuffer + offset);
      offset += this->abs_pose.serialize(outbuffer + offset);
      offset += this->current_pose.serialize(outbuffer + offset);
      offset += this->compass.serialize(outbuffer + offset);
      offset += this->imu.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->state.deserialize(inbuffer + offset);
      offset += this->extended_state.deserialize(inbuffer + offset);
      offset += this->abs_pose.deserialize(inbuffer + offset);
      offset += this->current_pose.deserialize(inbuffer + offset);
      offset += this->compass.deserialize(inbuffer + offset);
      offset += this->imu.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "flight_stack/DroneInfo"; };
    virtual const char * getMD5() override { return "d5a1c029b748c563b85f0760da378aca"; };

  };

}
#endif
