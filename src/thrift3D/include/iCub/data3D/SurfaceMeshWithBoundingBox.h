// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_SurfaceMeshWithBoundingBox
#define YARP_THRIFT_GENERATOR_STRUCT_SurfaceMeshWithBoundingBox

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iCub/data3D/Box3D.h>
#include <iCub/data3D/SurfaceMesh.h>

namespace iCub {
  namespace data3D {
    class SurfaceMeshWithBoundingBox;
  }
}


class iCub::data3D::SurfaceMeshWithBoundingBox : public yarp::os::idl::WirePortable {
public:
  SurfaceMesh mesh;
  Box3D boundingBox;
  SurfaceMeshWithBoundingBox() {
  }
  SurfaceMeshWithBoundingBox(const SurfaceMesh& mesh,const Box3D& boundingBox) : mesh(mesh), boundingBox(boundingBox) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

