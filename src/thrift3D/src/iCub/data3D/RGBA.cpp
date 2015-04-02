// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/RGBA.h>

namespace iCub { namespace data3D {
bool RGBA::read(yarp::os::idl::WireReader& reader) {
  if (!reader.readI32(rgba)) {
    reader.fail();
    return false;
  }
  return !reader.isError();
}

bool RGBA::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(1)) return false;
  return read(reader);
}

bool RGBA::write(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeI32(rgba)) return false;
  return !writer.isError();
}

bool RGBA::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  return write(writer);
}
}} // namespace
