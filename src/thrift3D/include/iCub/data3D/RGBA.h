// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_RGBA
#define YARP_THRIFT_GENERATOR_STRUCT_RGBA

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  namespace data3D {
    class RGBA;
  }
}


class iCub::data3D::RGBA : public yarp::os::idl::WirePortable {
public:
  int32_t rgba;
  RGBA() : rgba(0) {
  }
  RGBA(const int32_t rgba) : rgba(rgba) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

