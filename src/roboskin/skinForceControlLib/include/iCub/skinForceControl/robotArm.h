/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
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

#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <vector>
#include <sstream>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/common.h"

#include <iCub/skinForceControl/skinForceControlLib.h>
#include "iCub/skinForceControl/robot_interfaces.h"
#include "iCub/skinForceControl/util.h"

namespace iCub
{

namespace skinForceControl
{

    class InterfaceIndex{
    public:
        inline InterfaceIndex(): i(0), part(iCub::skinDynLib::BODY_PART_UNKNOWN){}
        inline InterfaceIndex(unsigned int i, iCub::skinDynLib::BodyPart part): i(i), part(part){}
        
        inline std::string toString(){
            std::stringstream ss;
            ss<<"joint: "<< i<< "; body part: "<< iCub::skinDynLib::BodyPart_s[part];
            return ss.str();
        }

        unsigned int i;
        iCub::skinDynLib::BodyPart part;
    };

    class RobotArm : public iCub::iDyn::iDynLimb
    {
    protected:
        Status                                      armStatus;      // status of the object
        RobotInterfaces*                            ri;             // interfaces to communicate with the robot
        iCub::iDyn::iCubArmSensorLink*              robotSensor;    // kinematics/dynamics representation of robot F/T sensor
        std::vector<iCub::skinDynLib::BodyPart>     bodyParts;      // list of body parts
        std::vector<int>                            bodyPartSizes;  // dof of each body part
        std::vector<InterfaceIndex>                 intIndexes;     // indexes associating each link with a body part and an index relative to that body part

        virtual void allocate(const skinDynLib::BodyPart &_type);

    public:
        /**
        * Default constructor. 
        */
        RobotArm(RobotInterfaces* ri);

        /**
        * Constructor. 
        * @param _type either left arm or right arm
        */
        RobotArm(RobotInterfaces* ri, const skinDynLib::BodyPart &_type, const iCub::iDyn::ChainComputationMode _mode=iCub::iDyn::KINFWD_WREBWD);
        
        /**
        * Copy constructor.
        * @param arm is the Arm to be copied.
        */
        RobotArm(const RobotArm &arm);

        /**
         * Initialize the robot arm.
         */
        Status init();

        /**
         * Read the joint angles from the robot.
         * @param q vector to fill with the read joint angles (in deg)
         * @param mustWait true if the read has to be blocking, false otherwise
         * @return status of the operation
         */
        Status readAng(yarp::sig::Vector &q, bool mustWait=false);

        /**
         * Read the joint torques from the robot.
         * @param torques vector to fill with the torques
         * @return status of the operation
         */
        Status readTorques(yarp::sig::Vector &torques);

        /**
        * Set the control mode of the specified joint.
        * If j is negative then all joints are set.
        * @param j joint number
        * @return status of the operation
        */
        Status setCtrlMode(ControlMode cm, int j=-1);

        /**
         * Set the control mode of the specified joint to position control.
         * If j is negative then all the joints are set.
         * @param j joint number
         * @return status of the operation
         */
        Status setPositionMode(int j=-1);

        /**
         * Set the control mode of the specified joint to velocity control.
         * If j is negative then all the joints are set.
         * @param j joint number
         * @return status of the operation
         */
        Status setVelocityMode(int j=-1);

        /**
         * Set the control mode of the specified joint to torque control.
         * If j is negative then all the joints are set.
         * @param j joint number
         * @return status of the operation
         */
        Status setTorqueMode(int j=-1);

        /**
         * Set the reference torque of the specified joint.
         * @param j joint number
         * @param torque torque to set
         * @return status of the operation
         */
        Status setRefTorque(unsigned int j, double torque);

        /**
         * Set the reference torques of all the joints in the chain.
         * @param torques torques to set
         * @return status of the operation
         */
        Status setRefTorques(yarp::sig::Vector torques);

        /**
         * Set the reference torque of the specified joint.
         * @param j joint number
         * @param torque torque to set
         * @return status of the operation
         */
        Status setRefVelocity(unsigned int j, double velocity);

        /**
         * Set the reference torques of all the joints in the chain.
         * @param torques torques to set
         * @return status of the operation
         */
        Status setRefVelocities(yarp::sig::Vector velocities);

        /**
         * Set the reference torque of the specified joint.
         * @param j joint number
         * @param torque torque to set
         * @return status of the operation
         */
        Status setRefPosition(unsigned int j, double position);

        /**
         * Set the reference torques of all the joints in the chain.
         * @param torques torques to set
         * @return status of the operation
         */
        Status setRefPositions(yarp::sig::Vector positions);

        /**
         * Set the reference vector of the specified joint.
         * @param cm control mode
         * @param j joint number
         * @param ref command reference
         * @return status of the operation
         */
        Status setRef(ControlMode cm, unsigned int j, double ref);

        /**
         * Set the reference vector of all the joints in the chain.
         * @param cm control mode
         * @param ref command reference vector
         * @return status of the operation
         */
        Status setRef(ControlMode cm, yarp::sig::Vector ref);

        unsigned int getSensorLinkIndex(){ return iCub::skinDynLib::ARM_FT_SENSOR_LINK_INDEX + iCub::skinDynLib::TORSO_DOF; }

        const yarp::sig::Matrix& getHSens();

        const yarp::sig::Matrix& getRSens();

        const yarp::sig::Vector& getrSens();

        yarp::sig::Matrix getH_i_j(unsigned int i, unsigned int j);
    };
}

} // end namespace

#endif

