// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/dmpLearnerInterface.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {


class dmpLearnerInterface_sync_opc : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("sync_opc",1,2)) return false;
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

class dmpLearnerInterface_estimate_DMP : public yarp::os::Portable {
public:
  int32_t id;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("estimate_DMP",1,2)) return false;
    if (!writer.writeI32(id)) return false;
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

class dmpLearnerInterface_train_ids : public yarp::os::Portable {
public:
  std::vector<int32_t>  trainInputIds;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("train_ids",1,2)) return false;
    {
      if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(trainInputIds.size()))) return false;
      std::vector<int32_t> ::iterator _iter0;
      for (_iter0 = trainInputIds.begin(); _iter0 != trainInputIds.end(); ++_iter0)
      {
        if (!writer.writeI32((*_iter0))) return false;
      }
      if (!writer.writeListEnd()) return false;
    }
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

class dmpLearnerInterface_train_action : public yarp::os::Portable {
public:
  std::string actionName;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("train_action",1,2)) return false;
    if (!writer.writeString(actionName)) return false;
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

class dmpLearnerInterface_generalize_DMP : public yarp::os::Portable {
public:
  int32_t id;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("generalize_DMP",1,2)) return false;
    if (!writer.writeI32(id)) return false;
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

class dmpLearnerInterface_set_num_basis_functions : public yarp::os::Portable {
public:
  int32_t N;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5)) return false;
    if (!writer.writeTag("set_num_basis_functions",1,4)) return false;
    if (!writer.writeI32(N)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class dmpLearnerInterface_set_alphax : public yarp::os::Portable {
public:
  double alphax;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("set_alphax",1,2)) return false;
    if (!writer.writeDouble(alphax)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class dmpLearnerInterface_set_alphaz : public yarp::os::Portable {
public:
  double alphaz;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("set_alphaz",1,2)) return false;
    if (!writer.writeDouble(alphaz)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class dmpLearnerInterface_set_betaz : public yarp::os::Portable {
public:
  double betaz;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("set_betaz",1,2)) return false;
    if (!writer.writeDouble(betaz)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class dmpLearnerInterface_quit : public yarp::os::Portable {
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

bool dmpLearnerInterface::sync_opc() {
  bool _return = false;
  dmpLearnerInterface_sync_opc helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpLearnerInterface::estimate_DMP(const int32_t id) {
  bool _return = false;
  dmpLearnerInterface_estimate_DMP helper;
  helper.id = id;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpLearnerInterface::train_ids(const std::vector<int32_t> & trainInputIds) {
  bool _return = false;
  dmpLearnerInterface_train_ids helper;
  helper.trainInputIds = trainInputIds;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpLearnerInterface::train_action(const std::string& actionName) {
  bool _return = false;
  dmpLearnerInterface_train_action helper;
  helper.actionName = actionName;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpLearnerInterface::generalize_DMP(const int32_t id) {
  bool _return = false;
  dmpLearnerInterface_generalize_DMP helper;
  helper.id = id;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void dmpLearnerInterface::set_num_basis_functions(const int32_t N) {
  dmpLearnerInterface_set_num_basis_functions helper;
  helper.N = N;
  yarp().write(helper,helper);
}
void dmpLearnerInterface::set_alphax(const double alphax) {
  dmpLearnerInterface_set_alphax helper;
  helper.alphax = alphax;
  yarp().write(helper,helper);
}
void dmpLearnerInterface::set_alphaz(const double alphaz) {
  dmpLearnerInterface_set_alphaz helper;
  helper.alphaz = alphaz;
  yarp().write(helper,helper);
}
void dmpLearnerInterface::set_betaz(const double betaz) {
  dmpLearnerInterface_set_betaz helper;
  helper.betaz = betaz;
  yarp().write(helper,helper);
}
void dmpLearnerInterface::quit() {
  dmpLearnerInterface_quit helper;
  yarp().write(helper);
}

bool dmpLearnerInterface::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "sync_opc") {
      bool _return;
      _return = sync_opc();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "estimate_DMP") {
      int32_t id;
      if (!reader.readI32(id)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = estimate_DMP(id);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "train_ids") {
      std::vector<int32_t>  trainInputIds;
      {
        trainInputIds.clear();
        uint32_t _size1;
        yarp::os::idl::WireState _etype4;
        reader.readListBegin(_etype4, _size1);
        trainInputIds.resize(_size1);
        uint32_t _i5;
        for (_i5 = 0; _i5 < _size1; ++_i5)
        {
          if (!reader.readI32(trainInputIds[_i5])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
      bool _return;
      _return = train_ids(trainInputIds);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "train_action") {
      std::string actionName;
      if (!reader.readString(actionName)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = train_action(actionName);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "generalize_DMP") {
      int32_t id;
      if (!reader.readI32(id)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = generalize_DMP(id);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_num_basis_functions") {
      int32_t N;
      if (!reader.readI32(N)) {
        reader.fail();
        return false;
      }
      set_num_basis_functions(N);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_alphax") {
      double alphax;
      if (!reader.readDouble(alphax)) {
        reader.fail();
        return false;
      }
      set_alphax(alphax);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_alphaz") {
      double alphaz;
      if (!reader.readDouble(alphaz)) {
        reader.fail();
        return false;
      }
      set_alphaz(alphaz);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_betaz") {
      double betaz;
      if (!reader.readDouble(betaz)) {
        reader.fail();
        return false;
      }
      set_betaz(betaz);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
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


