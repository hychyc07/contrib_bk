// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_dmpManagerInterface
#define YARP_THRIFT_GENERATOR_dmpManagerInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  class dmpManagerInterface;
}


class iCub::dmpManagerInterface : public yarp::os::Wire {
public:
  dmpManagerInterface() { yarp().setOwner(*this); }
  virtual bool train(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand);
  virtual bool stop_training();
  virtual bool s();
  virtual bool test(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand);
  virtual std::string observe_state();
  virtual void go_home();
  virtual void quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

