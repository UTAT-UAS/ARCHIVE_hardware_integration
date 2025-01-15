#ifndef _ROS_flight_stack_ControllerInfo_h
#define _ROS_flight_stack_ControllerInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "geometry_msgs/Twist.h"

namespace flight_stack
{

  class ControllerInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _temporary_type;
      _temporary_type temporary;
      typedef geographic_msgs::GeoPoseStamped _start_pose_type;
      _start_pose_type start_pose;
      typedef geographic_msgs::GeoPoseStamped _target_pose_type;
      _target_pose_type target_pose;
      typedef geometry_msgs::Twist _target_velocity_type;
      _target_velocity_type target_velocity;
      typedef float _camera_angle_type;
      _camera_angle_type camera_angle;
      typedef float _cruise_velocity_type;
      _cruise_velocity_type cruise_velocity;
      typedef float _yaw_rate_type;
      _yaw_rate_type yaw_rate;
      typedef float _coord_tolerance_raw_type;
      _coord_tolerance_raw_type coord_tolerance_raw;
      typedef float _alt_tolerance_type;
      _alt_tolerance_type alt_tolerance;
      typedef float _angle_tolerance_type;
      _angle_tolerance_type angle_tolerance;
      typedef float _lat_tolerance_type;
      _lat_tolerance_type lat_tolerance;
      typedef float _long_tolerance_type;
      _long_tolerance_type long_tolerance;
      typedef float _lat_factor_type;
      _lat_factor_type lat_factor;
      typedef float _long_factor_type;
      _long_factor_type long_factor;
      typedef const char* _drone_mode_type;
      _drone_mode_type drone_mode;
      typedef const char* _flight_mode_type;
      _flight_mode_type flight_mode;

    ControllerInfo():
      header(),
      temporary(""),
      start_pose(),
      target_pose(),
      target_velocity(),
      camera_angle(0),
      cruise_velocity(0),
      yaw_rate(0),
      coord_tolerance_raw(0),
      alt_tolerance(0),
      angle_tolerance(0),
      lat_tolerance(0),
      long_tolerance(0),
      lat_factor(0),
      long_factor(0),
      drone_mode(""),
      flight_mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_temporary = strlen(this->temporary);
      varToArr(outbuffer + offset, length_temporary);
      offset += 4;
      memcpy(outbuffer + offset, this->temporary, length_temporary);
      offset += length_temporary;
      offset += this->start_pose.serialize(outbuffer + offset);
      offset += this->target_pose.serialize(outbuffer + offset);
      offset += this->target_velocity.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->camera_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->cruise_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->coord_tolerance_raw);
      offset += serializeAvrFloat64(outbuffer + offset, this->alt_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->lat_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->long_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->lat_factor);
      offset += serializeAvrFloat64(outbuffer + offset, this->long_factor);
      uint32_t length_drone_mode = strlen(this->drone_mode);
      varToArr(outbuffer + offset, length_drone_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->drone_mode, length_drone_mode);
      offset += length_drone_mode;
      uint32_t length_flight_mode = strlen(this->flight_mode);
      varToArr(outbuffer + offset, length_flight_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->flight_mode, length_flight_mode);
      offset += length_flight_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_temporary;
      arrToVar(length_temporary, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_temporary; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_temporary-1]=0;
      this->temporary = (char *)(inbuffer + offset-1);
      offset += length_temporary;
      offset += this->start_pose.deserialize(inbuffer + offset);
      offset += this->target_pose.deserialize(inbuffer + offset);
      offset += this->target_velocity.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->camera_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cruise_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->coord_tolerance_raw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alt_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lat_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->long_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lat_factor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->long_factor));
      uint32_t length_drone_mode;
      arrToVar(length_drone_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_drone_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_drone_mode-1]=0;
      this->drone_mode = (char *)(inbuffer + offset-1);
      offset += length_drone_mode;
      uint32_t length_flight_mode;
      arrToVar(length_flight_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_flight_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_flight_mode-1]=0;
      this->flight_mode = (char *)(inbuffer + offset-1);
      offset += length_flight_mode;
     return offset;
    }

    virtual const char * getType() override { return "flight_stack/ControllerInfo"; };
    virtual const char * getMD5() override { return "3ccab51a93015c842f2248df9fa3b9cb"; };

  };

}
#endif
