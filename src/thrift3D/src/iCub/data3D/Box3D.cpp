// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/Box3D.h>

namespace iCub { namespace data3D {
bool Box3D::read(yarp::os::idl::WireReader& reader) {
  {
    corners.clear();
    uint32_t _size24;
    yarp::os::idl::WireState _etype27;
    reader.readListBegin(_etype27, _size24);
    corners.resize(_size24);
    uint32_t _i28;
    for (_i28 = 0; _i28 < _size24; ++_i28)
    {
      if (!reader.readNested(corners[_i28])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  if (!reader.read(orientation)) {
    reader.fail();
    return false;
  }
  return !reader.isError();
}

bool Box3D::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(2)) return false;
  return read(reader);
}

bool Box3D::write(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(corners.size()))) return false;
    std::vector<PointXYZ> ::iterator _iter29;
    for (_iter29 = corners.begin(); _iter29 != corners.end(); ++_iter29)
    {
      if (!writer.writeNested((*_iter29))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  if (!writer.write(orientation)) return false;
  return !writer.isError();
}

bool Box3D::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  return write(writer);
}
}} // namespace
