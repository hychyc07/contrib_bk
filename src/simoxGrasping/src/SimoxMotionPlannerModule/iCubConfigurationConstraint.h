
#ifndef _iCubConfigConstraint_h
#define _iCubConfigConstraint_h

#include <MotionPlanning/Saba.h>
#include <MotionPlanning/CSpace/CSpace.h>
#include <MotionPlanning/CSpace/ConfigurationConstraint.h>
#include <vector>

/*!
*
* \brief This constraint class forbids joint configurations which will break iCub's tendons.
*
*/
class iCubConfigurationConstraint : public Saba::ConfigurationConstraint
{
public:
	enum ConstraintConfig
	{
		eArm,		// dim = 7
		eTorsoArm	// dim = 10
	};
	iCubConfigurationConstraint(ConstraintConfig setup, unsigned int dimension);
	virtual ~iCubConfigurationConstraint();

	/*!
		An derived class has to implement this method in order to check 
		if a configuration c satisfies the constraint or not.
		\param c The config to be tested.
		\return True if c satisfies the constraint.
	*/
	virtual bool isValid(const Eigen::VectorXf &c);


protected:
	ConstraintConfig mode;
	
};


#endif // _iCubConfigConstraint_h
