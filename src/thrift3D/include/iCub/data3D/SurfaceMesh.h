// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_SurfaceMesh
#define YARP_THRIFT_GENERATOR_STRUCT_SurfaceMesh

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iCub/data3D/PointXYZ.h>
#include <iCub/data3D/Polygon.h>
#include <iCub/data3D/RGBA.h>

namespace iCub {
  namespace data3D {
    class SurfaceMesh;
  }
}


class iCub::data3D::SurfaceMesh : public yarp::os::idl::WirePortable {
public:
  std::string meshName;
  std::vector<PointXYZ>  points;
  std::vector<RGBA>  rgbColour;
  std::vector<Polygon>  polygons;
  SurfaceMesh() : meshName("") {
  }
  SurfaceMesh(const std::string& meshName,const std::vector<PointXYZ> & points,const std::vector<RGBA> & rgbColour,const std::vector<Polygon> & polygons) : meshName(meshName), points(points), rgbColour(rgbColour), polygons(polygons) {
  }
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);
};

#endif

