// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>

namespace iCub { namespace data3D {
bool SurfaceMeshWithBoundingBox::read(yarp::os::idl::WireReader& reader) {
  if (!reader.read(mesh)) {
    reader.fail();
    return false;
  }
  if (!reader.read(boundingBox)) {
    reader.fail();
    return false;
  }
  return !reader.isError();
}

bool SurfaceMeshWithBoundingBox::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(6)) return false;
  return read(reader);
}

bool SurfaceMeshWithBoundingBox::write(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(mesh)) return false;
  if (!writer.write(boundingBox)) return false;
  return !writer.isError();
}

bool SurfaceMeshWithBoundingBox::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  return write(writer);
}
}} // namespace
