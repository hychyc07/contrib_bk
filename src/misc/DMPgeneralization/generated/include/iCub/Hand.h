// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_Hand
#define YARP_THRIFT_GENERATOR_ENUM_Hand

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  enum Hand {
    RIGHT = 0,
    LEFT = 1,
    INDIFF = 2
  };

  class HandVocab;
}

class iCub::HandVocab : public yarp::os::idl::WireVocab {
public:
  virtual int fromString(const std::string& input);
  virtual std::string toString(int input);
};


#endif
