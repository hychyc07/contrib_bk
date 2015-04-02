// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_ClassScore
#define YARP_THRIFT_GENERATOR_STRUCT_ClassScore

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class ClassScore;


class ClassScore : public yarp::os::idl::WirePortable {
public:
  std::string className;
  double score;
  ClassScore() : className(""), score(0) {
  }
  ClassScore(const std::string& className,const double score) : className(className), score(score) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

