// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/dmpManagerInterface.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {


class dmpManagerInterface_train : public yarp::os::Portable {
public:
  std::string action;
  std::string target;
  std::string tool;
  std::string hand;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5)) return false;
    if (!writer.writeTag("train",1,1)) return false;
    if (!writer.writeString(action)) return false;
    if (!writer.writeString(target)) return false;
    if (!writer.writeString(tool)) return false;
    if (!writer.writeString(hand)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class dmpManagerInterface_stop_training : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("stop_training",1,2)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class dmpManagerInterface_s : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("s",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class dmpManagerInterface_test : public yarp::os::Portable {
public:
  std::string action;
  std::string target;
  std::string tool;
  std::string hand;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5)) return false;
    if (!writer.writeTag("test",1,1)) return false;
    if (!writer.writeString(action)) return false;
    if (!writer.writeString(target)) return false;
    if (!writer.writeString(tool)) return false;
    if (!writer.writeString(hand)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class dmpManagerInterface_observe_state : public yarp::os::Portable {
public:
  std::string _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("observe_state",1,2)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readString(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class dmpManagerInterface_go_home : public yarp::os::Portable {
public:
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("go_home",1,2)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    return true;
  }
};

class dmpManagerInterface_quit : public yarp::os::Portable {
public:
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("quit",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    return true;
  }
};

bool dmpManagerInterface::train(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand) {
  bool _return = false;
  dmpManagerInterface_train helper;
  helper.action = action;
  helper.target = target;
  helper.tool = tool;
  helper.hand = hand;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpManagerInterface::stop_training() {
  bool _return = false;
  dmpManagerInterface_stop_training helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpManagerInterface::s() {
  bool _return = false;
  dmpManagerInterface_s helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpManagerInterface::test(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand) {
  bool _return = false;
  dmpManagerInterface_test helper;
  helper.action = action;
  helper.target = target;
  helper.tool = tool;
  helper.hand = hand;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string dmpManagerInterface::observe_state() {
  std::string _return = "";
  dmpManagerInterface_observe_state helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void dmpManagerInterface::go_home() {
  dmpManagerInterface_go_home helper;
  yarp().write(helper);
}
void dmpManagerInterface::quit() {
  dmpManagerInterface_quit helper;
  yarp().write(helper);
}

bool dmpManagerInterface::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "train") {
      std::string action;
      std::string target;
      std::string tool;
      std::string hand;
      if (!reader.readString(action)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(target)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(tool)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(hand)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = train(action,target,tool,hand);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stop_training") {
      bool _return;
      _return = stop_training();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "s") {
      bool _return;
      _return = s();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "test") {
      std::string action;
      std::string target;
      std::string tool;
      std::string hand;
      if (!reader.readString(action)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(target)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(tool)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(hand)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = test(action,target,tool,hand);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "observe_state") {
      std::string _return;
      _return = observe_state();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "go_home") {
      go_home();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}
} // namespace


