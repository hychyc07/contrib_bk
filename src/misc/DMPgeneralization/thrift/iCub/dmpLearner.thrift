//Elena Ceseracciu - RBCS, Istituto Italiano di Tecnologia, 2012

/**
*   dmpLearnerInterface
*
* Interface for a module that performs DMP estimation and generalization from a DMP+target database
*/

/// @cond


namespace yarp iCub

service dmpLearnerInterface
{
  /**
  * Ask objectsPropertiesCollector to provide all available "action" entities; if they have "trajectory" but not DMPs, update them with DMP parameters. 
  * Keep all actions and relative "targetPosition"s in memory.
  * @return true if connection with OPC succeded.
  */
  bool sync_opc();

  /**
  * Estimate DMP parameters from "trajectory" property of an OPC "action" entity. Update the OPC entry with DMP parameters. 
  * @param id the OPC id number for "action" entity that needs DMP estimation. 
  * @return true if connection with OPC succeded.
  */
  bool estimate_DMP(1:i32 id);
  
  /**
  * Train DMP generalizer from a database of DMPs.
  * @param trainInputIds the OPC id numbers for "action" entities that are used as training inputs for generalization. 
  * @return true if training process was successfully performed
  */
  bool train_ids(1:list<i32> trainInputIds);

  /**
  * Train DMP generalizer from a database of DMPs.
  * @param actionName name of the action the examples of which (action entities with that "name" property in OPC )are used for training.
  * @return true if training process was successfully performed
  */
  bool train_action(1:string actionName);

  /**
  * Perform DMP generalization for a target provided by OPC action entity as "targetPosition" property.
  * @param id the OPC id number for "action" entity that has a "targetPosition" on which to perform DMP generalization
  * @return true if generalization was successfully performed
  */
  bool generalize_DMP(1:i32 id);

  void set_num_basis_functions(1:i32 N);
  void set_alphax(1:double alphax);
  void set_alphaz(1:double alphaz);
  void set_betaz(1:double betaz);
  oneway void quit();

}

/// @endcond
