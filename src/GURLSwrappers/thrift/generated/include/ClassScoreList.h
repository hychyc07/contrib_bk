// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_ClassScoreList
#define YARP_THRIFT_GENERATOR_STRUCT_ClassScoreList

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <ClassScore.h>

class ClassScoreList;


class ClassScoreList : public yarp::os::idl::WirePortable {
public:
  std::vector<ClassScore>  elements;
  ClassScoreList() {
  }
  ClassScoreList(const std::vector<ClassScore> & elements) : elements(elements) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

