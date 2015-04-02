// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/PointXYZ.h>

namespace iCub { namespace data3D {
bool PointXYZ::read(yarp::os::idl::WireReader& reader) {
  if (!reader.readDouble(x)) {
    reader.fail();
    return false;
  }
  if (!reader.readDouble(y)) {
    reader.fail();
    return false;
  }
  if (!reader.readDouble(z)) {
    reader.fail();
    return false;
  }
  return !reader.isError();
}

bool PointXYZ::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(3)) return false;
  return read(reader);
}

bool PointXYZ::write(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeDouble(x)) return false;
  if (!writer.writeDouble(y)) return false;
  if (!writer.writeDouble(z)) return false;
  return !writer.isError();
}

bool PointXYZ::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  return write(writer);
}
}} // namespace
