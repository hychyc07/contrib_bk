// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/Polygon.h>

namespace iCub { namespace data3D {
bool Polygon::read(yarp::os::idl::WireReader& reader) {
  {
    vertices.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    vertices.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readI32(vertices[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return !reader.isError();
}

bool Polygon::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(1)) return false;
  return read(reader);
}

bool Polygon::write(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(vertices.size()))) return false;
    std::vector<int32_t> ::iterator _iter5;
    for (_iter5 = vertices.begin(); _iter5 != vertices.end(); ++_iter5)
    {
      if (!writer.writeI32((*_iter5))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return !writer.isError();
}

bool Polygon::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  return write(writer);
}
}} // namespace
