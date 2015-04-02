/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea DelPrete
  * email: andrea.delprete@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

#include "iCub/skinForceControl/robotArm.h"
#include <string>
#include <map>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>

using namespace iCub::skinForceControl;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RobotArm::RobotArm(RobotInterfaces* ri): ri(ri)
{
    allocate(RIGHT_ARM);
	setIterMode(KINFWD_WREBWD);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RobotArm::RobotArm(RobotInterfaces* ri, const BodyPart &_type, const ChainComputationMode _mode): ri(ri)
{
    allocate(_type);
	setIterMode(_mode);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RobotArm::RobotArm(const RobotArm &arm)
{
    clone(arm);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void RobotArm::allocate(const BodyPart &_type)
{    
    if(_type==LEFT_ARM)
    {
        iDynLimb::allocate("left");
        robotSensor = new iCubArmSensorLink("left");
    }
    else
    {
        iDynLimb::allocate("right");
        robotSensor = new iCubArmSensorLink("right");
    }

	Matrix H0(4,4);
    H0.zero();
    H0(0,1)= -1.0;
    H0(1,2)= -1.0;
    H0(2,0)=  1.0;
    H0(3,3)=  1.0;
	setH0(H0);

    // add the first two links of the torso (which are the same for both arms)
    pushLink(new iDynLink(0,	      3.120e-2,	 6.300e-4,  -9.758e-7,		4.544e-4,  -4.263e-5,  -3.889e-8,   1.141e-3,   0.000e-0,   1.236e-3,		32.0e-3,         0,     M_PI/2.0,        0.0,	-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
	pushLink(new iDynLink(0,	      4.265e-2,	+4.296e-5,	-1.360e-3,		5.308e-4,  -1.923e-6,   5.095e-5,   2.031e-3,  -3.849e-7,   1.803e-3,		      0,   -5.5e-3,     M_PI/2.0,  -M_PI/2.0,	-39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
    if (_type==RIGHT_ARM)
    {
		//           iDynLink(mass,       rC (3x1),                             I(6x1),                                                                          A,        D,      alfa,              offset,         min,               max);        
	    pushLink(new iDynLink(4.81e+0,	 -8.102e-5, -1.183e-1,   7.905e-3,		7.472e-2,  -3.600e-6,  -4.705e-5,   8.145e-2,   4.567e-3,   1.306e-2,	 -0.0233647,   -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD,	-59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.189,	 0.005e-3,  18.7e-3,   1.19e-3,		 123.0e-6,   0.021e-6,  -0.001e-6,    24.4e-6,    4.22e-6,   113.0e-6,			0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD)); 
		pushLink(new iDynLink(0.179,	-0.094e-3, -6.27e-3,  -16.6e-3,		 137.0e-6, -0.453e-06,  0.203e-06,    83.0e-6,    20.7e-6,    99.3e-6,			0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.884,	  1.79e-3, -62.9e-3, 0.064e-03,		 743.0e-6,    63.9e-6,  0.851e-06,   336.0e-6,   -3.61e-6,   735.0e-6, 		 -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  80.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074,	 -13.7e-3, -3.71e-3,   1.05e-3,		  28.4e-6,  -0.502e-6,  -0.399e-6,    9.24e-6,  -0.371e-6,    29.9e-6,		  0.015,      0.0,  M_PI/2.0,                 0.0,  15.0*CTRL_DEG2RAD, 105.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525,	-0.347e-3,  71.3e-3,  -4.76e-3,		 766.0e-6,    5.66e-6,    1.40e-6,   164.0e-6,    18.2e-6,   699.0e-6,	        0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,			    0,        0,         0,		 	    0,		    0,		    0,			0,			0,		    0,	        0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,       0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.213,	  7.73e-3, -8.05e-3,  -9.00e-3,		 154.0e-6,	  12.6e-6,   -6.08e-6,   250.0e-6,    17.6e-6,   378.0e-6,	     0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));                                                                                                                            
    }
    else
    {
        pushLink(new iDynLink(4.81e+0,	 -8.102e-5, -1.183e-1,   7.905e-3,		7.472e-2,  -3.600e-6,  -4.705e-5,   8.145e-2,   4.567e-3,   1.306e-2,	  0.0233647,   -0.1433,  -M_PI/2.0, 105.0*CTRL_DEG2RAD,	-59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.189,	-0.005e-3,    18.7e-3,  -1.19e-3,		54.421e-6,   0.009e-6,     0.0e-6,   9.331e-6,  -0.017e-6,  54.862e-6,			0.0, 0.10774,	   -M_PI/2.0,            M_PI/2.0,  -95.5*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.179,     0.094e-3,   -6.27e-3,   16.6e-3,		 137.2e-6,   0.466e-6,   0.365e-6,  82.927e-6, -20.524e-6,  99.274e-6,			0.0,	 0.0,		M_PI/2.0,           -M_PI/2.0,				   0.0,	160.8*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.884,     -1.79e-3,    62.9e-3, -0.064e-3,	   748.531e-6,  63.340e-6,  -0.903e-6, 338.109e-6,  -4.031e-6, 741.022e-6,		  0.015, 0.15228,	   -M_PI/2.0,   75.0*CTRL_DEG2RAD,  -37.0*CTRL_DEG2RAD,  80.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.074,      13.7e-3,    3.71e-3,  -1.05e-3,		28.389e-6,  -0.515e-6,  -0.408e-6,   9.244e-6,  -0.371e-6,  29.968e-6,		 -0.015,     0.0,		M_PI/2.0,                 0.0,   15.0*CTRL_DEG2RAD, 105.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.525,     0.347e-3,   -71.3e-3,   4.76e-3,	   765.393e-6,   4.337e-6,   0.239e-6, 164.578e-6,  19.381e-6, 698.060e-6,			0.0,  0.1373,		M_PI/2.0,           -M_PI/2.0,	-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(	 0,	   	        0, 		    0,		   0,				0,		    0,	        0,		    0,		    0,		    0,			0.0,	 0.0,		M_PI/2.0,            M_PI/2.0,	-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iDynLink(0.213,      7.73e-3,   -8.05e-3,   9.00e-3,		   157.143e-6,  12.780e-6,   4.823e-6, 247.995e-6, -18.188e-6, 380.535e-6,		 0.0625,  -0.016,			 0.0,                 0.0,	-20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    }

    bodyParts.resize(2);
    bodyParts[0] = _type;
    bodyParts[1] = TORSO;

    intIndexes.resize(N);
    // torso's links are reversed w.r.t. the interface indexing
    intIndexes[0] = InterfaceIndex(2, TORSO);
    intIndexes[1] = InterfaceIndex(1, TORSO);
    intIndexes[2] = InterfaceIndex(0, TORSO);
    for(int i=0; i<ARM_DOF; i++)
        intIndexes[TORSO_DOF+i] = InterfaceIndex(i, _type);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::init(){
    int partNum = bodyParts.size();
    bodyPartSizes.resize(partNum, 0);
    int ax=0, i=0;
    for(vector<BodyPart>::const_iterator it=bodyParts.begin(); it!=bodyParts.end(); it++){
        if(ri->ienc[*it]->getAxes(&ax))
            bodyPartSizes[i] = ax;
        else
            armStatus.addErrMsg("Error while reading number of axes of body part "+BodyPart_s[*it]);
        i++;
    }
    return armStatus;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::readAng(Vector &q, bool mustWait){
    map<BodyPart, Vector> qTemp;
    q = getAng();
    Status res;
    int i=0;
    for(vector<BodyPart>::const_iterator it=bodyParts.begin(); it!=bodyParts.end(); it++){
        qTemp[*it].resize(bodyPartSizes[i], 0);
        if(mustWait){
            while(ri->ienc[*it]->getEncoders(qTemp[*it].data()) == false)
                Time::delay(0.1);
        }else{
            if(ri->ienc[*it]->getEncoders(qTemp[*it].data()) == false)
                res.addErrMsg(BodyPart_s[*it]+" joint angles not updated");
        }
        i++;
    }
    i=0;
    for(vector<InterfaceIndex>::const_iterator it=intIndexes.begin(); it!=intIndexes.end(); it++){
        q[i] = qTemp[it->part][it->i];
        i++;
    }
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::readTorques(Vector &torques){
    map<BodyPart, Vector> taoTemp;
    torques.resize(N);
    Status res;
    int i=0;
    for(vector<BodyPart>::const_iterator it=bodyParts.begin(); it!=bodyParts.end(); it++){
        taoTemp[*it].resize(bodyPartSizes[i], 0);
        if(ri->itrq[*it]->getTorques(taoTemp[*it].data()) == false)
            res.addErrMsg(BodyPart_s[*it]+" joint torques not updated");
        i++;
    }
    i=0;
    for(vector<InterfaceIndex>::const_iterator it=intIndexes.begin(); it!=intIndexes.end(); it++){
        torques[i] = taoTemp[it->part][it->i];
        i++;
    }
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setCtrlMode(ControlMode cm, int j){
    switch(cm){
        case POS_CTRL_MODE:     return setPositionMode(j);
        case TORQUE_CTRL_MODE:  return setTorqueMode(j);
        case VEL_CTRL_MODE:     return setVelocityMode(j);
    }
    return Status("setControlMode failed because specified control mode is unknown");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setPositionMode(int j){
    Status res;
    if(j<0){
        for(unsigned int i=0; i<N; i++)
            if(!ri->icmd[intIndexes[i].part]->setPositionMode(intIndexes[i].i))
                res.addErrMsg("Error while setting position mode "+intIndexes[i].toString());
        return res;
    }
    if(!ri->icmd[intIndexes[j].part]->setPositionMode(intIndexes[j].i))
        res.addErrMsg("Error while setting position mode "+intIndexes[j].toString());
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setVelocityMode(int j){
    Status res;
    if(j<0){
        for(unsigned int i=0; i<N; i++)
            if(!ri->icmd[intIndexes[i].part]->setVelocityMode(intIndexes[i].i))
                res.addErrMsg("Error while setting velocity mode "+intIndexes[i].toString());
        return res;
    }
    if(!ri->icmd[intIndexes[j].part]->setVelocityMode(intIndexes[j].i))
        res.addErrMsg("Error while setting velocity mode "+intIndexes[j].toString());
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setTorqueMode(int j){
    Status res;
    if(j<0){
        for(unsigned int i=0; i<N; i++)
            if(!ri->icmd[intIndexes[i].part]->setTorqueMode(intIndexes[i].i))
                res.addErrMsg("Error while setting torque mode "+intIndexes[i].toString());
        return res;
    }
    if(!ri->icmd[intIndexes[j].part]->setTorqueMode(intIndexes[j].i))
        res.addErrMsg("Error while setting torque mode "+intIndexes[j].toString());
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefTorque(unsigned int j, double torque){
    if(j>=N)
        return Status("Error while setting ref torque: joint index out of range");
    if(!ri->itrq[intIndexes[j].part]->setRefTorque(intIndexes[j].i, torque))
        return Status("Error while setting ref torque "+intIndexes[j].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefTorques(Vector torques){
    if(torques.size()!=N)
        return Status("Error while setting ref torques: wrong torque vector size");
    Status res;
    for(unsigned int i=0;i<N;i++)
        if(!ri->itrq[intIndexes[i].part]->setRefTorque(intIndexes[i].i, torques[i]))
            res.addErrMsg("Error while setting ref torque "+intIndexes[i].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefVelocity(unsigned int j, double vel){
    if(j>=N)
        return Status("Error while setting ref velocity: joint index out of range");
    if(fabs(vel)<1e-2) vel = 0.0;
    if(!ri->ivel[intIndexes[j].part]->velocityMove(intIndexes[j].i, vel))
        return Status("Error while setting ref velocity "+intIndexes[j].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefVelocities(Vector velocities){
    if(velocities.size()!=N)
        return Status("Error while setting ref velocities: wrong torque vector size");
    Status res;
    for(unsigned int i=0;i<N;i++)
        if(!ri->ivel[intIndexes[i].part]->velocityMove(intIndexes[i].i, velocities[i]))
            res.addErrMsg("Error while setting ref velocities "+intIndexes[i].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefPosition(unsigned int j, double pos){
    if(j>=N)
        return Status("Error while setting ref position: joint index out of range");
    if(!ri->ipos[intIndexes[j].part]->positionMove(intIndexes[j].i, pos))
        return Status("Error while setting ref position "+intIndexes[j].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRefPositions(Vector positions){
    if(positions.size()!=N)
        return Status("Error while setting ref positions: wrong torque vector size");
    Status res;
    for(unsigned int i=0;i<N;i++)
        if(!ri->ipos[intIndexes[i].part]->positionMove(intIndexes[i].i, positions[i]))
            res.addErrMsg("Error while setting ref positions "+intIndexes[i].toString());
    return Status();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRef(ControlMode cm, unsigned int j, double ref){
    switch(cm){
        case POS_CTRL_MODE:     return setRefPosition(j, ref);
        case TORQUE_CTRL_MODE:  return setRefTorque(j, ref);
        case VEL_CTRL_MODE:     return setRefVelocity(j, ref);
    }
    return Status("setRef failed because specified control mode is unknown");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status RobotArm::setRef(ControlMode cm, yarp::sig::Vector ref){
    switch(cm){
        case POS_CTRL_MODE:     return setRefPositions(ref);
        case TORQUE_CTRL_MODE:  return setRefTorques(ref);
        case VEL_CTRL_MODE:     return setRefVelocities(ref);
    }
    return Status("setRef failed because specified control mode is unknown");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Matrix& RobotArm::getHSens()
{
    return robotSensor->getH();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Matrix& RobotArm::getRSens()
{
    return robotSensor->getR();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& RobotArm::getrSens()
{
    return robotSensor->getr();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix RobotArm::getH_i_j(unsigned int i, unsigned int j)
{
    if(i==j)
        return eye(4,4);

    if(i>j)
    {
        Matrix Hinv = getH_i_j(j, i);
        return SE3inv(Hinv);
    }
    
    Matrix H = refLink(i+1)->getH();     // initialize H with the rototranslation from <i> to <i+1>
    for(unsigned int k=i+2; k<=j; k++)
        H *= refLink(k)->getH();
    return H;
}
