#include "armDynamics.h"

using namespace std;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::dev;

//////////
//======================================
//
//	      ICUB ARM NO TORSO DYN
//
//======================================



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDynV2::iCubArmDynV2()
{
    allocate("right");
	setIterMode(iCub::iDyn::KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDynV2::iCubArmDynV2(const string &_type, const iCub::iDyn::ChainComputationMode _mode)
{
    allocate(_type);
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iCubArmDynV2::iCubArmDynV2(const iCubArmDynV2 &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCubArmDynV2::allocate(const string &_type)
{
    iDynLimb::allocate(_type);

    H0.zero();
	H0.eye();
#ifdef HAS_EXT_FTSENSOR
    linkList.resize(5);
#else
    linkList.resize(4);
#endif

    if (type=="right")
    {

        //note: the D value in joint 0 is different from the corresponding one in iCubArmDyn (and iKin::iCubArm)
        // in ikin: -0.10774   here: 0.0
        // see the RBT matrix of the right arm connected to the UpperTorso or UpperBody nodes
		linkList[0]=new iDynLink(0.189,		 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0,     -0.0,		M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD); 
		linkList[1]=new iDynLink(0.179,		-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0,		-M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(0.884,		  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228,		-M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(0.610,		 -1.66e-3, -0.45e-3,   -6.1e-2,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,		M_PI/2.0,                 0.0,   0.0*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD);
        linkList[4]=new iDynLink(0.123,		 0.0,0.0,0.0,		  0.0,0.0,0.0,0.0,0.0,0.0,																	    0.0,      -0.1373,		M_PI,                 M_PI,   0.0*CTRL_DEG2RAD, 360.0*CTRL_DEG2RAD);
 //       linkList[4]=new iDynLink(0.525,		-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
 //       linkList[5]=new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD);
 //       linkList[6]=new iDynLink(0.213,		  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD);
 	}
    else
    {
        //note: the D value in joint 0 is different from the corresponding one in iCubArmDyn (and iKin::iCubArm)
        // in ikin:  +0.10774   here: 0.0
        // see the RBT matrix of the right arm connected to the UpperTorso or UpperBody nodes
        linkList[0]=new iDynLink(0.13,	-0.004e-3, 14.915e-3, -0.019e-3,		54.421e-6,   0.009e-6,     0.0e-6,   9.331e-6,  -0.017e-6,  54.862e-6,			0.0,	 0.0,	   -M_PI/2.0,            M_PI/2.0,  -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD);
        linkList[1]=new iDynLink(0.178,  0.097e-3,  -6.271e-3, 16.622e-3,		 137.2e-6,   0.466e-6,   0.365e-6,  82.927e-6, -20.524e-6,  99.274e-6,			0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,				   0.0,	160.8*CTRL_DEG2RAD);
        linkList[2]=new iDynLink(0.894, -1.769e-3, 63.302e-3, -0.084e-3,	   748.531e-6,  63.340e-6,  -0.903e-6, 338.109e-6,  -4.031e-6, 741.022e-6,		  0.015, 0.15228,	   -M_PI/2.0,   75.0*CTRL_DEG2RAD,  -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
        linkList[3]=new iDynLink(0.610,		 -1.66e-3, -0.45e-3,   -6.1e-2,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD);
 //       linkList[4]=new iDynLink(0.525, 0.264e-3, -71.327e-3,  4.672e-3,	   765.393e-6,   4.337e-6,   0.239e-6, 164.578e-6,  19.381e-6, 698.060e-6,			0.0,  0.1373,		M_PI/2.0,           -M_PI/2.0,	-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD);
 //       linkList[5]=new iDynLink(	 0,		   0,		   0,		  0,				0,		    0,	        0,		    0,		    0,		    0,			0.0,	 0.0,		M_PI/2.0,            M_PI/2.0,	-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD);
 //       linkList[6]=new iDynLink(0.214, 7.851e-3, -8.319e-3, 9.284e-3,		   157.143e-6,  12.780e-6,   4.823e-6, 247.995e-6, -18.188e-6, 380.535e-6,		 0.0625,  -0.016,			 0.0,                 0.0,	-20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD);
    }
	
#ifdef HAS_EXT_FTSENSOR
	if (type=="right")
    {        
		linkList[4]=new iDynLink(0.123,		 0.0,0.0,0.0,		  0.0,0.0,0.0,0.0,0.0,0.0,																	    0.0,      -0.1373,		M_PI,                 M_PI,   0.0*CTRL_DEG2RAD, 360.0*CTRL_DEG2RAD);
 	}
#endif

    for(unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

#ifdef HAS_EXT_FTSENSOR
	blockLink(4,0.0);
#endif
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCubArmDynV2::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limArm  =*lim[0];

    unsigned int iArm;
    double min, max;

    for (iArm=0; iArm<getN(); iArm++)
    {   
        if (!limArm.getLimits(iArm,&min,&max))
            return false;

        (*this)[iArm].setMin(CTRL_DEG2RAD*min);
        (*this)[iArm].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}



iDynSensorArmV2::iDynSensorArmV2(iCubArmDynV2 *_c, const iCub::iDyn::NewEulMode _mode, unsigned int verb)
:iDynSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note position 5 if with torso, 2 without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynSensorArm: assuming right arm.\n");
        }
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_c->getType(),mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynSensorArmV2::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//=========================================
//
//		 iDYN INV SENSOR ARM NO TORSO
//
//=========================================

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensorArmV2::iDynInvSensorArmV2(iCubArmDynV2 *_c, const iCub::iDyn::NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c->asChain(),_c->getType(),_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note: it's 5 if arm with torso, 2 if arm without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_c->getType()=="left")||(_c->getType()=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm. \n");
        }
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_c->getType(),mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
iDynInvSensorArmV2::iDynInvSensorArmV2(iDynChain *_c, const string _type, const iCub::iDyn::NewEulMode _mode, unsigned int verb)
:iDynInvSensor(_c,_type,_mode,verb)
{
	// FT sensor is in position 2 in the kinematic chain in both arms
	// note: it's 5 if arm with torso, 2 if arm without torso
	lSens = 2;
	// the arm type determines the sensor properties
	if( !((_type=="left")||(_type=="right"))  )
	{
		if(verbose)
        {
            fprintf(stderr,"iDynInvSensorArm error: type is not left/right. iCub only has a left and a right arm, it is not an octopus :) \n");
			fprintf(stderr,"iDynInvSensorArm: assuming right arm. \n");
        }
		// set the sensor with the default value
		sens = new iCubArmSensorLink("right",mode,verbose);
	}
	else
	{
		// set the sensor properly
		sens = new iCubArmSensorLink(_type,mode,verbose);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string iDynInvSensorArmV2::getType() const
{ 
	return sens->getType();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
