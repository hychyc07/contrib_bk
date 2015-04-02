#include "iCubConfigurationConstraint.h"
#include <MotionPlanning/CSpace/CSpace.h>


iCubConfigurationConstraint::iCubConfigurationConstraint(ConstraintConfig setup, unsigned int dimension)
	:Saba::ConfigurationConstraint(dimension)
{
	switch (setup)
	{
	case eArm:
		THROW_VR_EXCEPTION_IF(dimension!=7,"Dimension does not fit, expecting 7");
		break;
	case eTorsoArm:
		THROW_VR_EXCEPTION_IF(dimension!=10,"Dimension does not fit, expecting 10");
		break;
	default:
		THROW_VR_EXCEPTION ("mode not yet implemented...");
	}
	mode = setup;
}

iCubConfigurationConstraint::~iCubConfigurationConstraint()
{
}

bool iCubConfigurationConstraint::isValid( const Eigen::VectorXf &c )
{
	static const float DEG_RAD = ((float)M_PI/180.0f);
	VR_ASSERT(c.rows()==dimension);

	float theta0,theta1,theta2;
	if (mode==eArm)
	{
		theta0 = c[0];
		theta1 = c[1];
		theta2 = c[2];
	} else
	{
		theta0 = c[3];
		theta1 = c[4];
		theta2 = c[5];
	}
	if ( 1.71f * (theta0 - theta1) <= -347.0f * DEG_RAD )
		return false;

	if (-366.57f*DEG_RAD >= 1.71f * (theta0 - theta1 - theta2))
		return false;

	if (1.71f * (theta0 - theta1 - theta2) >= 112.42f*DEG_RAD)
		return false;

	if (-66.6f*DEG_RAD >= (theta1 + theta2))
		return false;

	if ((theta1 + theta2) >= 213.3f*DEG_RAD)
		return false;

	return true;
}

