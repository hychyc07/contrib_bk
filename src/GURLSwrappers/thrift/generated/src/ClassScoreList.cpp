// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ClassScoreList.h>

bool ClassScoreList::read(yarp::os::idl::WireReader& reader) {
  {
    elements.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    elements.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readNested(elements[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return !reader.isError();
}

bool ClassScoreList::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(1)) return false;
  return read(reader);
}

bool ClassScoreList::write(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(elements.size()))) return false;
    std::vector<ClassScore> ::iterator _iter5;
    for (_iter5 = elements.begin(); _iter5 != elements.end(); ++_iter5)
    {
      if (!writer.writeNested((*_iter5))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return !writer.isError();
}

bool ClassScoreList::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  return write(writer);
}
