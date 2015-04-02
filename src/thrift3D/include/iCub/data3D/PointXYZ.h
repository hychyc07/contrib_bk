// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_PointXYZ
#define YARP_THRIFT_GENERATOR_STRUCT_PointXYZ

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  namespace data3D {
    class PointXYZ;
  }
}


class iCub::data3D::PointXYZ : public yarp::os::idl::WirePortable {
public:
  double x;
  double y;
  double z;
  PointXYZ() : x(0), y(0), z(0) {
  }
  PointXYZ(const double x,const double y,const double z) : x(x), y(y), z(z) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

