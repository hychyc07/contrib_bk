// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_dmpLearnerInterface
#define YARP_THRIFT_GENERATOR_dmpLearnerInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {
  class dmpLearnerInterface;
}


class iCub::dmpLearnerInterface : public yarp::os::Wire {
public:
  dmpLearnerInterface() { yarp().setOwner(*this); }
/**
 * Ask objectsPropertiesCollector to provide all available "action" entities; if they have "trajectory" but not DMPs, update them with DMP parameters.
 * Keep all actions and relative "targetPosition"s in memory.
 * @return true if connection with OPC succeded.
 */
  virtual bool sync_opc();
/**
 * Estimate DMP parameters from "trajectory" property of an OPC "action" entity. Update the OPC entry with DMP parameters.
 * @param id the OPC id number for "action" entity that needs DMP estimation.
 * @return true if connection with OPC succeded.
 */
  virtual bool estimate_DMP(const int32_t id);
/**
 * Train DMP generalizer from a database of DMPs.
 * @param trainInputIds the OPC id numbers for "action" entities that are used as training inputs for generalization.
 * @return true if training process was successfully performed
 */
  virtual bool train_ids(const std::vector<int32_t> & trainInputIds);
/**
 * Train DMP generalizer from a database of DMPs.
 * @param actionName name of the action the examples of which (action entities with that "name" property in OPC )are used for training.
 * @return true if training process was successfully performed
 */
  virtual bool train_action(const std::string& actionName);
/**
 * Perform DMP generalization for a target provided by OPC action entity as "targetPosition" property.
 * @param id the OPC id number for "action" entity that has a "targetPosition" on which to perform DMP generalization
 * @return true if generalization was successfully performed
 */
  virtual bool generalize_DMP(const int32_t id);
  virtual void set_num_basis_functions(const int32_t N);
  virtual void set_alphax(const double alphax);
  virtual void set_alphaz(const double alphaz);
  virtual void set_betaz(const double betaz);
  virtual void quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

