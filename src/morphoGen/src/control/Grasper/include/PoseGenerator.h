//to implement:
//--generation of probabilities
//--save probabilities on file
//--fetch vision system
//fetch affordances     (TODO: return a matrix of double for selecting the dominant hand)
//--extract bounding box position
//--extract object width, height and orientation
//--decide object class
//account the delta
//--generate wrist orientation
//-switch dominant wrist orientation and select wrist angle and contact points (distance from the center of the bounding box)
//-select dominant hand (left/right) according to affordance output (TODO)
//put the robot to rest
//approach the object
//redefine contact point closer to the object (for grasping)
//grasp
//set next position (TODO)
//move hand
//open hand
//human vote dominant wrist orientation,non dominant wrist orientation, finger configuration
//--update weights

//TODO code in PoseGenerator the finger stretch angle (at the moment it is coded in PMP)
//TODO - implement a Goal class for stacking, it sets the probabilities in this way: a probability for grasping from the side, grasping from the top and grsping from the bottom (a very low value) is generated from a formula that takes into account the stack's height, then this is multiplied with the dominant hand's prob matrix
//     - Goal is part of Context

//TODO PoseGenerator have to became a singleton!

#ifndef POSE_GENERATOR
#define POSE_GENERATOR

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "probabilities.h"

//one day probabilities.h will be all coded here

/**
	Class encoding the Posture Tree and wrapping class Probab. This class implements the Singleton pattern. Its main purpose is to generate a valid wrist orientation for both hands and a finger configuration for the dominant hand

*/
class PoseGenerator{

    // 0 : object class \ dominant orientation
    // 1 : dominant orientation \ non-dominant orientation
    // 2 : dominant orientation \ finger configuration
    int Nrow[3],Ncol[3];

    int last_dominant, last_nonDominant;  //here for compatibility reasons
    int last_ObjectClass;                 //as above
    
    yarp::sig::Matrix VoteWr;//The total vote for grasping with dominant hand in three different wrist orientations
    
    yarp::sig::Matrix VoteWrOtherHand;//The total vote for other hand wrist orientations for each class of dominant hand wrist orientation

    yarp::sig::Matrix VoteFinger;//The total vote for wrist yaw and finger primitive

    static PoseGenerator* _instance;

    PoseGenerator();
    ~PoseGenerator();

public:

	/**
		Enum encoding the hands available on the robot
	*/
    enum Hands{
        Left,
        Right,
        HandsNum
    };

	/**
		Enum describing the content in a configuration Vector
	*/
    enum Config{
        LeftHand,
        RightHand,
        FingerPrim,
        ConfigNum
    };

	/**
		Enum encodign the available object classes, for now just Flat and Tall
	*/
	enum ObjectClass{
        Flat,
        Tall,
        ObjectClassNum
    };

	/**
		The wrist can be set at the value, in radiants, indicated by this constant (either 1.27 or -1.27
	*/
	static const double ANGLE;


	/**
		The function providing the only available instance of this singleton class. If not instance is present, it will be triggered the constructor
		@return the class's instance
	*/
    static PoseGenerator* getInstance();

	/**
		This function destroys the instance if present
	*/
    static void destroyInstance();

	/**
		Function to set the branching probabilities to an arbitrary value. Each branching is a matrix
		@param newVoteWr new branching probability for the object class/dominant hand pair (2x3 matrix)
		@param newVoteWrOtherHand new branching probability for the dominant hand/non-dominant hand pair (3x4 matrix)
		@param newVoteFinger new branching probability for the dominant hand/finger configuration pair (3x3 matrix)
	*/
	void setProbabilities(const yarp::sig::Matrix& newVoteWr, const yarp::sig::Matrix& newVoteWrOtherHand, const yarp::sig::Matrix& newVoteFinger);


	/**
		Function to get a posture from the tree given an object class and a dominant hand. It is also a wrapper for Probab::GenerateWrOrientation
		@param objClass an object class selected from the ObjectClass enum
		@param dominant the dominant hand, selected from the Hands enum
		@return a Vector containing the angle values for the left hand and the right hand, plus an indefitier for the finger stretching (the stretching can be either 10 degrees or 0, and is set in class PassiveMotionParadigm)
	*/

    // 0 : left hand, 1 : right hand, 2 : finger primitive
    yarp::sig::Vector getConfiguration(Probab::ObjectClass objClass,Hands dominant);

	/**
		Wrapper for Probab::UpdateWeights
		@param succDom a boolean value stating if the dominant hand had a correct orientation (true) or not (false)
		@param succNonDom a boolean value stating if the non-dominant hand had a correct orientation (true) or not (false)
		@param succFing a boolean value stating if the finger configuration was correct (true) or not (false)
	*/

    void updateWeights(bool succDom,bool succNonDom,bool succFing);
};

#endif