// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_Polygon
#define YARP_THRIFT_GENERATOR_STRUCT_Polygon

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  namespace data3D {
    class Polygon;
  }
}


class iCub::data3D::Polygon : public yarp::os::idl::WirePortable {
public:
  std::vector<int32_t>  vertices;
  Polygon() {
  }
  Polygon(const std::vector<int32_t> & vertices) : vertices(vertices) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

