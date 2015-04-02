/**
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Dalia De Santis
 * email:  dalia.desantis@iit.it
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

/**
 * Abstract interface class for dealing with the Passive Motion Paradigm 
 * (PMP) control. 
 * \author Dalia De Santis  
 */ 

#ifndef __PMP_INTERFACE_H__
#define __PMP_INTERFACE_H__

#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Property.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

namespace iCub{
	namespace pmplib{

#define PMPNET_CMD_ACK				VOCAB3('a','c','k')
#define PMPNET_CMD_NACK				VOCAB4('n','a','c','k')

#define PMPNET_CMD_OPEN				VOCAB3('o','p','n')
#define PMPNET_CMD_CLOSE			VOCAB3('c','l','s')

// network manipulation:
#define PMPNET_CMD_CREATE_NODE		VOCAB3('c','r','n')
#define PMPNET_CMD_DEL_NODE			VOCAB3('d','l','n')
#define PMPNET_CMD_ENABLE_NODE		VOCAB3('e','n','n')
#define PMPNET_CMD_DISABLE_NODE		VOCAB4('d','i','s','n')
#define PMPNET_CMD_CREATE_CONN		VOCAB3('c','r','c')
#define PMPNET_CMD_DSTRY_CONN		VOCAB3('d','y','c')
#define PMPNET_CMD_ENABLE_CONN		VOCAB3('e','n','c')
#define PMPNET_CMD_DISABLE_CONN		VOCAB4('d','i','s','c')

// state manipulation:
#define PMPNET_CMD_GET_POSITION		VOCAB4('g','p','o','n')
#define PMPNET_CMD_GET_POSE			VOCAB4('g','p','s','e')
#define PMPNET_CMD_GET_ANG			VOCAB4('s','a','n','g')
#define PMPNET_CMD_SET_ANG			VOCAB4('g','a','n','g')
#define PMPNET_CMD_SET_TGPOSITION	VOCAB4('s','p','o','n')
#define PMPNET_CMD_SET_TGPOSE		VOCAB4('s','p','s','e')
#define PMPNET_CMD_SET_TG			VOCAB3('s','t','g')
#define PMPNET_CMD_SET_NODEPAR		VOCAB3('s','n','p')
#define PMPNET_CMD_GET_NODEPAR		VOCAB3('g','n','p')
#define PMPNET_CMD_SET_CONNPAR		VOCAB3('s','c','p')
#define PMPNET_CMD_GET_CONNPAR		VOCAB3('g','c','p')

// execution control:
#define PMPNET_CMD_GET_SUCCESS		VOCAB3('g','s','x')
#define PMPNET_CMD_START			VOCAB4('s','t','r','t')
#define PMPNET_CMD_READY			VOCAB3('r','d','y')

/**
* PMP interface class definition
*/
class PMP_interface
{
public:
	virtual bool openInterface(yarp::os::Property &options) = 0;
	virtual void closeInterface() = 0;

	/**
	* Create a new pmp node from a list of properties.
	* @param options contains the set of options in form of a Property object.
	*
	* All the following fields must be specified:
	* \b chain: The name of the kinematic chain to be associated with the block
	* \b q_initial: Initial joint positions (in degrees)
	* \b q_ref: Joint angles reference value for computing internal joint field (in degrees)
	* \b Admittance: Vector of joint admittances
	* \b VirtualStiffness: 3D vector of virtual stiffness values
	* \b JointStiffness: Vector of virtual joint stiffnesses
	*
	* Only some of the following parameters can be specified. All the non-specified
	* parameters will be initialized using default values.
	* \b root: if root field is specified, the node will be set as the root node of the network
	* \b T_init: initial time for internal tbg to start the computation (in sec)
	* \b T_dur: Duration of the internal attractor dynamics (in sec)
	* \b SlopeRamp: Time incremental step (in sec)
	* \b alpha: parameter regulating the steepness of the gamma function
	*/
	virtual bool createNode(yarp::os::Property &options) = 0;

	/**
	* Create a new connection between two pmp nodes
	* @param options contains the set of options in form of a Property object.
	*
	* All the following fields must be specified:
	* \b source: The name of the pmp source node
	* \b target: The name of the pmp target node
	*
	* Only some of the following parameters can be specified. All the non-specified
	* parameters will be initialized using parameters of the source node.
	* \b chain: The name of the kinematic chain to be associated with the block
	* \b Admittance: Vector of joint admittances
	*/
	virtual bool createConnection(yarp::os::Property &options) = 0;

	/** 
	* Delete an existing node.
	* @param name is the node name
	* Returns false if the node is non existing.
	*/
	virtual bool destroyNode(const std::string &name) = 0;
	/** 
	* Delete an existing connection between two nodes.
	* @param source is the source node name
	* @param target is the target node name
	* Returns false if the node is non existing.
	*/
	virtual bool destroyConnection(const std::string &source, const std::string &target) = 0;

	/** 
	* Enable an existing node making it visible.
	* @param name is the node name
	* Returns false if the node is non existing.
	*/
	virtual bool enableNode(const std::string &name, yarp::os::Property *options = NULL) = 0;
	/** 
	* Enable an existing connection between two nodes making it visible.
	* If the target node is disabled, it will be automatically enabled.
	* @param source is the source node name
	* @param target is the target node name
	* Returns false if the node is non existing.
	*/
	virtual bool enableConnection(const std::string &src, const std::string &tg, yarp::os::Property *options = NULL) = 0;
	/** 
	* Disable an existing node making it hidden.
	* All the ingoing and outgoing connections are consequently disabled.
	* If an outgoing connection has a target node that has no other input connections,
	* its target node will be disabled. This process continues recursively untill a node
	* with no output connection is met.
	* @param name is the node name
	* Returns false if the node is non existing.
	*/
	virtual bool disableNode(const std::string &name) = 0;
	/** 
	* Enable an existing connection between two nodes making it hidden.
	* If a connection has a target node that has no other input connections,
	* its target node will be disabled. This process continues recursively untill a node
	* with no output connection is met.
	* @param source is the source node name
	* @param target is the target node name
	* Returns false if the node is non existing.
	*/
	virtual bool disableConnection(const std::string &src, const std::string &tg) = 0;

	/**
	* Get EE or joint position of a specific node in the current pmp network.
	* Returns false if the node is non existing.
	*/
	virtual bool getPosition(const std::string &nodeName, yarp::sig::Vector & x) = 0;
	virtual bool getPosition(const std::string &nodeName, const int & joint, yarp::sig::Vector & x) = 0;
	/**
	* Get EE or joint pose of a specific node in the current pmp network in the form of a
	* 3x3 rotation matrix. Returns false if the node is non existing.
	*/
	virtual bool getPose(const std::string &nodeName, yarp::sig::Matrix & x) = 0;
	virtual bool getPose(const std::string &nodeName, const int & joint, yarp::sig::Matrix & x) = 0;
	
	/**
	* Get joint angles of a pmp node in the network. Returns false if the node is non existing
	*/
	virtual bool getAng(const std::string &nodeName, yarp::sig::Vector &x) = 0;
	
	/**
	* Set joint angles of a pmp node in the network. Returns false if the node is non existing
	* or the size of the vector is different from the node's DOF
	*/
	virtual bool setAng(const std::string &nodeName, const yarp::sig::Vector &x) = 0;

	/**
	* Assign a target to a PMP node.
	* @param name is the target node name
	* @param options contains the set of options in form of a Property object.
	*
	* Available parameters are:
	* \b position: a yarp::sig::Vector of size three. (TODO: a Matrix of target points)
	* \b pose: a yarp::sig::Vector of size three representing Roll-Pitch-Yaw angles
	* \b compensate: 1 if compensation of anisotropies is desired, 0 otherwise (default value is 1)
	* \b vtgsMode: 'default' to generate straight-to-target trajectories (default value is 'default')
	* TO BE DEALT WITH IN FUTURE:
	* \b T_init1: initial time for internal tbg to start the computation (in sec) of the first tbg
	* \b T_init2: initial time for internal tbg to start the computation (in sec) of the second tbg
	* \b T_dur1: Duration of the internal attractor dynamics (in sec) of the first tbg
	* \b T_dur2: Duration of the internal attractor dynamics (in sec) of the second tbg
	* \b SlopeRamp1: Time incremental step (in sec) of the first tbg
	* \b SlopeRamp2: Time incremental step (in sec) of the second tbg
	* \b alpha1: parameter regulating the steepness of the gamma function of the first target tbg
	* \b alpha2: parameter regulating the steepness of the gamma function of the second target tbg
	*
	* RPY angles are to be given wrt hand position with palm parallel to ground and directed forward
	* x is along fingers ( xref = -x_root)
	* z is pointing towards the floor (zref = -z_root)
	* y is choosen accordingly (yref = y_root)
	*
	* if no vtgs parameters are specified, the module will use the default ones (set in config file)
	*/
	virtual bool setTarget(const std::string &name, yarp::os::Property &options) = 0; // deve ctrl se nodo è abilitato!
	virtual bool setTargetPosition(const std::string &name, const yarp::sig::Vector &v) = 0;
	virtual bool setTargetPose(const std::string &name, const yarp::sig::Vector &rpy) = 0;

	/**
	* Set pmp node parameters. 
	* @param options contains the set of options in form of a Property object.
	*
	* Available parameters are:
	*
	* \b VirtualStiffness
	* \b Admittance
	* \b JointStiffness
	* \b q_initial
	* \b q_ref
	* \b blockedLinks
	* \b T_init
	* \b T_dur
	* \b SlopeRamp
	* \b alpha
	*/
	virtual bool setNodeParameters(const std::string &name, yarp::os::Property &params) = 0;
	virtual bool getNodeParameters(const std::string &name, yarp::os::Property &params) = 0;
	/**
	* Set pmp connection parameters. 
	* @param options contains the set of options in form of a Property object.
	*
	* Available parameters are:
	*
	* \b Admittance
	* \b blockedLinks
	*/
	virtual bool setConnectionParameters(const std::string &source, const std::string &target, yarp::os::Property &params) = 0;
	virtual bool getConnectionParameters(const std::string &source, const std::string &target, yarp::os::Property &params) = 0;
	virtual bool enableSimulation() = 0;
	virtual bool enableExecution() = 0;

	/**
	* Start pmp network trajectory computation in the execution mode specified by
	* enableSimulation() or disableSimulation()
	*/
	virtual bool start() = 0;

	/**
	* Get Simulation result
	*/
	virtual bool getSuccess() = 0;
};
}
}

#endif