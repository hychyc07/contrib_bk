// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_GurlsClassificationInterface
#define YARP_THRIFT_GENERATOR_GurlsClassificationInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <ClassScore.h>
#include <yarp/sig/Vector.h>

class GurlsClassificationInterface;


class GurlsClassificationInterface : public yarp::os::Wire {
public:
  GurlsClassificationInterface() { yarp().setOwner(*this); }
  virtual bool add_sample(const std::string& className, const yarp::sig::Vector& sample);
  virtual bool save(const std::string& className);
  virtual bool train();
  virtual bool forget(const std::string& className);
  virtual std::vector<std::string>  classList();
  virtual bool stop();
  virtual bool recognize();
  virtual std::string classify_sample(const yarp::sig::Vector& sample);
  virtual std::vector<ClassScore>  get_scores_for_sample(const yarp::sig::Vector& sample);
  virtual std::string get_parameter(const std::string& parameterName);
  virtual bool set_parameter(const std::string& parameterName, const std::string& parameterValue);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

