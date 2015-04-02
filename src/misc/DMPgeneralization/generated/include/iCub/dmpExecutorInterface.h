// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_dmpExecutorInterface
#define YARP_THRIFT_GENERATOR_dmpExecutorInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iCub/Hand.h>

namespace iCub {
  class dmpExecutorInterface;
}


class iCub::dmpExecutorInterface : public yarp::os::Wire {
public:
  dmpExecutorInterface() { yarp().setOwner(*this); }
  virtual bool run();
  virtual bool is_running();
  virtual bool stop();
  virtual bool s();
  virtual bool execute_OPC(const int32_t id);
  virtual bool waitMotionDone(const double period = 0.5, const double timeout = 0);
  virtual bool set_hand(const Hand newHand);
  virtual Hand get_hand();
  virtual bool teach_start(const std::string& actionName, const Hand handToUse = RIGHT);
  virtual void quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

