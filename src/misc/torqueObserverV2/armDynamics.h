#ifndef __ARM_DYNAMICS_V2_H_
#define __ARM_DYNAMICS_V2_H_

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>
#include <string>


#define HAS_EXT_FTSENSOR 1
#define TORQUE_PROJECTION 1

class iCubArmDynV2 : public iCub::iDyn::iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArmDynV2();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and "right" arm
    */
	iCubArmDynV2(const std::string &_type, const iCub::iDyn::ChainComputationMode _mode=iCub::iDyn::KINFWD_WREBWD);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArmDynV2(const iCubArmDynV2 &arm);

	/**
    * Alignes the Arm joints bounds with current values set aboard 
    * the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Torso and the Arm limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);

};


class iDynSensorArmV2 : public iCub::iDyn::iDynSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubArmDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iDynSensorArmV2(iCubArmDynV2 *_c, const iCub::iDyn::NewEulMode _mode = iCub::iDyn::DYNAMIC, unsigned int verb = iCub::iDyn::NO_VERBOSE);

	/**
	* @return type the arm sensor type: left/arm
	*/
	std::string getType() const;


};


/**
* \ingroup iDynInv
*
* A class for computing force/moment of the FT sensor placed
* in the middle of the iCub's left or right arm. The sensor
* parameters are automatically set by chosing left or right
* during initialization of the iCubArmNoTorsoDyn.
* 
*/
class iDynInvSensorArmV2 : public iCub::iDyn::iDynInvSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubArmNoTorsoDyn where the sensor is placed on
	* @param _mode the analysis mode (STATIC/DYNAMIC)
	* @param verb flag for verbosity
    */
	iDynInvSensorArmV2(iCubArmDynV2 *_c, const iCub::iDyn::NewEulMode _mode = iCub::iDyn::DYNAMIC, unsigned int verb = iCub::iDyn::NO_VERBOSE);

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice; note that in this case 
	* there is not a specification of the iCubArmNoTorsoDyn, but the part must be specified
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _type a string setting the arm type
	* @param _mode the analysis mode (STATIC/DYNAMIC)
	* @param verb flag for verbosity
    */
	iDynInvSensorArmV2(iCub::iDyn::iDynChain *_c, const std::string _type, const iCub::iDyn::NewEulMode _mode = iCub::iDyn::DYNAMIC, unsigned int verb = iCub::iDyn::NO_VERBOSE);

	/**
	* @return type the arm type: left/arm
	*/
	std::string getType() const;


};

#endif