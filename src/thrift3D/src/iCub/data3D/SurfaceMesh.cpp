// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/SurfaceMesh.h>

namespace iCub { namespace data3D {
bool SurfaceMesh::read(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(meshName)) {
    reader.fail();
    return false;
  }
  {
    points.clear();
    uint32_t _size6;
    yarp::os::idl::WireState _etype9;
    reader.readListBegin(_etype9, _size6);
    points.resize(_size6);
    uint32_t _i10;
    for (_i10 = 0; _i10 < _size6; ++_i10)
    {
      if (!reader.readNested(points[_i10])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  {
    rgbColour.clear();
    uint32_t _size11;
    yarp::os::idl::WireState _etype14;
    reader.readListBegin(_etype14, _size11);
    rgbColour.resize(_size11);
    uint32_t _i15;
    for (_i15 = 0; _i15 < _size11; ++_i15)
    {
      if (!reader.readNested(rgbColour[_i15])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  {
    polygons.clear();
    uint32_t _size16;
    yarp::os::idl::WireState _etype19;
    reader.readListBegin(_etype19, _size16);
    polygons.resize(_size16);
    uint32_t _i20;
    for (_i20 = 0; _i20 < _size16; ++_i20)
    {
      if (!reader.readNested(polygons[_i20])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return !reader.isError();
}

bool SurfaceMesh::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(4)) return false;
  return read(reader);
}

bool SurfaceMesh::write(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(meshName)) return false;
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(points.size()))) return false;
    std::vector<PointXYZ> ::iterator _iter21;
    for (_iter21 = points.begin(); _iter21 != points.end(); ++_iter21)
    {
      if (!writer.writeNested((*_iter21))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(rgbColour.size()))) return false;
    std::vector<RGBA> ::iterator _iter22;
    for (_iter22 = rgbColour.begin(); _iter22 != rgbColour.end(); ++_iter22)
    {
      if (!writer.writeNested((*_iter22))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(polygons.size()))) return false;
    std::vector<Polygon> ::iterator _iter23;
    for (_iter23 = polygons.begin(); _iter23 != polygons.end(); ++_iter23)
    {
      if (!writer.writeNested((*_iter23))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return !writer.isError();
}

bool SurfaceMesh::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  return write(writer);
}
}} // namespace
