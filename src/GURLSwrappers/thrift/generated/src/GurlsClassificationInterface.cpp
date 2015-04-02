// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <GurlsClassificationInterface.h>
#include <yarp/os/idl/WireTypes.h>



class GurlsClassificationInterface_add_sample : public yarp::os::Portable {
public:
  std::string className;
  yarp::sig::Vector sample;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(4)) return false;
    if (!writer.writeTag("add_sample",1,2)) return false;
    if (!writer.writeString(className)) return false;
    if (!writer.write(sample)) return false;
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

class GurlsClassificationInterface_save : public yarp::os::Portable {
public:
  std::string className;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("save",1,1)) return false;
    if (!writer.writeString(className)) return false;
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

class GurlsClassificationInterface_train : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("train",1,1)) return false;
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

class GurlsClassificationInterface_forget : public yarp::os::Portable {
public:
  std::string className;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("forget",1,1)) return false;
    if (!writer.writeString(className)) return false;
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

class GurlsClassificationInterface_classList : public yarp::os::Portable {
public:
  std::vector<std::string>  _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("classList",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    {
      _return.clear();
      uint32_t _size6;
      yarp::os::idl::WireState _etype9;
      reader.readListBegin(_etype9, _size6);
      _return.resize(_size6);
      uint32_t _i10;
      for (_i10 = 0; _i10 < _size6; ++_i10)
      {
        if (!reader.readString(_return[_i10])) {
          reader.fail();
          return false;
        }
      }
      reader.readListEnd();
    }
    return true;
  }
};

class GurlsClassificationInterface_stop : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("stop",1,1)) return false;
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

class GurlsClassificationInterface_recognize : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("recognize",1,1)) return false;
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

class GurlsClassificationInterface_classify_sample : public yarp::os::Portable {
public:
  yarp::sig::Vector sample;
  std::string _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("classify_sample",1,2)) return false;
    if (!writer.write(sample)) return false;
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

class GurlsClassificationInterface_get_scores_for_sample : public yarp::os::Portable {
public:
  yarp::sig::Vector sample;
  std::vector<ClassScore>  _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5)) return false;
    if (!writer.writeTag("get_scores_for_sample",1,4)) return false;
    if (!writer.write(sample)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    {
      _return.clear();
      uint32_t _size11;
      yarp::os::idl::WireState _etype14;
      reader.readListBegin(_etype14, _size11);
      _return.resize(_size11);
      uint32_t _i15;
      for (_i15 = 0; _i15 < _size11; ++_i15)
      {
        if (!reader.readNested(_return[_i15])) {
          reader.fail();
          return false;
        }
      }
      reader.readListEnd();
    }
    return true;
  }
};

class GurlsClassificationInterface_get_parameter : public yarp::os::Portable {
public:
  std::string parameterName;
  std::string _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("get_parameter",1,2)) return false;
    if (!writer.writeString(parameterName)) return false;
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

class GurlsClassificationInterface_set_parameter : public yarp::os::Portable {
public:
  std::string parameterName;
  std::string parameterValue;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(4)) return false;
    if (!writer.writeTag("set_parameter",1,2)) return false;
    if (!writer.writeString(parameterName)) return false;
    if (!writer.writeString(parameterValue)) return false;
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

bool GurlsClassificationInterface::add_sample(const std::string& className, const yarp::sig::Vector& sample) {
  bool _return = false;
  GurlsClassificationInterface_add_sample helper;
  helper.className = className;
  helper.sample = sample;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::save(const std::string& className) {
  bool _return = false;
  GurlsClassificationInterface_save helper;
  helper.className = className;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::train() {
  bool _return = false;
  GurlsClassificationInterface_train helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::forget(const std::string& className) {
  bool _return = false;
  GurlsClassificationInterface_forget helper;
  helper.className = className;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::vector<std::string>  GurlsClassificationInterface::classList() {
  std::vector<std::string>  _return;
  GurlsClassificationInterface_classList helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::stop() {
  bool _return = false;
  GurlsClassificationInterface_stop helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::recognize() {
  bool _return = false;
  GurlsClassificationInterface_recognize helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string GurlsClassificationInterface::classify_sample(const yarp::sig::Vector& sample) {
  std::string _return = "";
  GurlsClassificationInterface_classify_sample helper;
  helper.sample = sample;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::vector<ClassScore>  GurlsClassificationInterface::get_scores_for_sample(const yarp::sig::Vector& sample) {
  std::vector<ClassScore>  _return;
  GurlsClassificationInterface_get_scores_for_sample helper;
  helper.sample = sample;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string GurlsClassificationInterface::get_parameter(const std::string& parameterName) {
  std::string _return = "";
  GurlsClassificationInterface_get_parameter helper;
  helper.parameterName = parameterName;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool GurlsClassificationInterface::set_parameter(const std::string& parameterName, const std::string& parameterValue) {
  bool _return = false;
  GurlsClassificationInterface_set_parameter helper;
  helper.parameterName = parameterName;
  helper.parameterValue = parameterValue;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool GurlsClassificationInterface::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "add_sample") {
      std::string className;
      yarp::sig::Vector sample;
      if (!reader.readString(className)) {
        reader.fail();
        return false;
      }
      if (!reader.read(sample)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = add_sample(className,sample);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "save") {
      std::string className;
      if (!reader.readString(className)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = save(className);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "train") {
      bool _return;
      _return = train();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "forget") {
      std::string className;
      if (!reader.readString(className)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = forget(className);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "classList") {
      std::vector<std::string>  _return;
      _return = classList();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_STRING, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iter16;
          for (_iter16 = _return.begin(); _iter16 != _return.end(); ++_iter16)
          {
            if (!writer.writeString((*_iter16))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "stop") {
      bool _return;
      _return = stop();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "recognize") {
      bool _return;
      _return = recognize();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "classify_sample") {
      yarp::sig::Vector sample;
      if (!reader.read(sample)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = classify_sample(sample);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_scores_for_sample") {
      yarp::sig::Vector sample;
      if (!reader.read(sample)) {
        reader.fail();
        return false;
      }
      std::vector<ClassScore>  _return;
      _return = get_scores_for_sample(sample);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<ClassScore> ::iterator _iter17;
          for (_iter17 = _return.begin(); _iter17 != _return.end(); ++_iter17)
          {
            if (!writer.writeNested((*_iter17))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "get_parameter") {
      std::string parameterName;
      if (!reader.readString(parameterName)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = get_parameter(parameterName);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_parameter") {
      std::string parameterName;
      std::string parameterValue;
      if (!reader.readString(parameterName)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(parameterValue)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_parameter(parameterName,parameterValue);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
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


