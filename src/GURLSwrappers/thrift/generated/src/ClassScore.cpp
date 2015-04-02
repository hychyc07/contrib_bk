// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ClassScore.h>

bool ClassScore::read(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(className)) {
    reader.fail();
    return false;
  }
  if (!reader.readDouble(score)) {
    reader.fail();
    return false;
  }
  return !reader.isError();
}

bool ClassScore::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(2)) return false;
  return read(reader);
}

bool ClassScore::write(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(className)) return false;
  if (!writer.writeDouble(score)) return false;
  return !writer.isError();
}

bool ClassScore::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  return write(writer);
}
