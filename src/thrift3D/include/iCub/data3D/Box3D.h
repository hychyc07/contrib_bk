// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_Box3D
#define YARP_THRIFT_GENERATOR_STRUCT_Box3D

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iCub/data3D/PointXYZ.h>
#include <yarp/sig/Matrix.h>

namespace iCub {
  namespace data3D {
    class Box3D;
  }
}


class iCub::data3D::Box3D : public yarp::os::idl::WirePortable {
public:
  std::vector<PointXYZ>  corners;
  yarp::sig::Matrix orientation;
  Box3D() {
  }
  Box3D(const std::vector<PointXYZ> & corners,const yarp::sig::Matrix& orientation) : corners(corners), orientation(orientation) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

