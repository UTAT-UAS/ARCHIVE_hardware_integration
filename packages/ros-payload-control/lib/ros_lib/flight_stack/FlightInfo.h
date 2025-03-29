#ifndef _ROS_flight_stack_FlightInfo_h
#define _ROS_flight_stack_FlightInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace flight_stack
{

  class FlightInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _stack_status_type;
      _stack_status_type stack_status;
      typedef const char* _current_state_type;
      _current_state_type current_state;
      typedef const char* _current_state_id_type;
      _current_state_id_type current_state_id;
      typedef const char* _previous_state_type;
      _previous_state_type previous_state;
      typedef const char* _pervious_state_id_type;
      _pervious_state_id_type pervious_state_id;
      enum { INITIAL = 0 };
      enum { START = 1 };
      enum { RUNNING = 2 };
      enum { EXIT = 3 };

    FlightInfo():
      header(),
      stack_status(0),
      current_state(""),
      current_state_id(""),
      previous_state(""),
      pervious_state_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->stack_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stack_status);
      uint32_t length_current_state = strlen(this->current_state);
      varToArr(outbuffer + offset, length_current_state);
      offset += 4;
      memcpy(outbuffer + offset, this->current_state, length_current_state);
      offset += length_current_state;
      uint32_t length_current_state_id = strlen(this->current_state_id);
      varToArr(outbuffer + offset, length_current_state_id);
      offset += 4;
      memcpy(outbuffer + offset, this->current_state_id, length_current_state_id);
      offset += length_current_state_id;
      uint32_t length_previous_state = strlen(this->previous_state);
      varToArr(outbuffer + offset, length_previous_state);
      offset += 4;
      memcpy(outbuffer + offset, this->previous_state, length_previous_state);
      offset += length_previous_state;
      uint32_t length_pervious_state_id = strlen(this->pervious_state_id);
      varToArr(outbuffer + offset, length_pervious_state_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pervious_state_id, length_pervious_state_id);
      offset += length_pervious_state_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->stack_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stack_status);
      uint32_t length_current_state;
      arrToVar(length_current_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_state-1]=0;
      this->current_state = (char *)(inbuffer + offset-1);
      offset += length_current_state;
      uint32_t length_current_state_id;
      arrToVar(length_current_state_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_state_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_state_id-1]=0;
      this->current_state_id = (char *)(inbuffer + offset-1);
      offset += length_current_state_id;
      uint32_t length_previous_state;
      arrToVar(length_previous_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_previous_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_previous_state-1]=0;
      this->previous_state = (char *)(inbuffer + offset-1);
      offset += length_previous_state;
      uint32_t length_pervious_state_id;
      arrToVar(length_pervious_state_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pervious_state_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pervious_state_id-1]=0;
      this->pervious_state_id = (char *)(inbuffer + offset-1);
      offset += length_pervious_state_id;
     return offset;
    }

    virtual const char * getType() override { return "flight_stack/FlightInfo"; };
    virtual const char * getMD5() override { return "1a9ba9b2884ba8b873b3fbf570ec5ac4"; };

  };

}
#endif
