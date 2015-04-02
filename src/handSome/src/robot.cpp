//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#include "robot.hpp"

using namespace HandSome;

Robot::Robot(bool torso)
{
	this->torso = torso;

	left_arm = new body_part;
	left_arm->driver = NULL;
	left_arm->limb = NULL;
	
	right_arm = new body_part;
	right_arm->driver = NULL;
	right_arm->limb = NULL;
	
	head_neck = new body_part;
	head_neck->driver = NULL;
	head_neck->limb = NULL;
	
}

bool Robot::init(string robot)
{
	Property option;
	string arm_string, arm_specific_string;
	
	arm_string = "/";
	arm_string += robot;
	
	option.put("device","remote_controlboard");
		
	/********************* Left Arm ************************/
	arm_specific_string = arm_string + "/left_arm";
	option.put("remote",arm_specific_string.c_str());
	option.put("local","/handsome/left_arm");
	left_arm->driver = new PolyDriver;
	left_arm->driver->open(option);
	if (!left_arm->driver->isValid()) {
		printf(" Left arm device not available.\n");
		return false;
	}
	
	if(torso){
		left_arm->limb = new iCubArmDyn("left");
		left_arm->limb->releaseLink(0);	
		left_arm->limb->releaseLink(1);
		left_arm->limb->releaseLink(2);	
	}
	else
		left_arm->limb = new iCubArmNoTorsoDyn("left");


	/********************* Right Arm ************************/
	arm_specific_string = arm_string + "/right_arm";
	option.put("remote",arm_specific_string.c_str());
	option.put("local","/handsome/right_arm");
    right_arm->driver = new PolyDriver(option);
    if (!right_arm->driver->isValid()) {
        printf(" Right arm device not available.\n");
        return false;
    }
	if(torso){
		right_arm->limb = new iCubArmDyn("right");
		right_arm->limb->releaseLink(0);	
		right_arm->limb->releaseLink(1);
		right_arm->limb->releaseLink(2);
	}
	else
		right_arm->limb = new iCubArmNoTorsoDyn("right");
    
    
    /********************* Head/Neck ************************/
    /*
    //To check...for robot and simulator
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
	option.put("local","/handsome/gaze");
    head_neck->driver = new PolyDriver(option);
    if (!head_neck->driver->isValid()) {
        printf(" Hand/Neck device not available.\n");
        return false;
    }
    head_neck->limb = new iCubEyeNeckRefDyn ();	
    */
    cout<<"Robot Initializazion...Done!"<<endl;
}

body_part* Robot::get_part(int part)
{
	switch(part)
	{
		case LEFT_ARM:	return this->left_arm;
		case RIGHT_ARM:	return this->right_arm;
		case NECK:		return this->head_neck;
	}
	return NULL;
}

PolyDriver* Robot::get_driver(int part)
{
	switch(part)
	{
		case LEFT_ARM:	return this->left_arm->driver;
		case RIGHT_ARM:	return this->right_arm->driver;
		case NECK:		return this->head_neck->driver;
	}
	return NULL;
}

iDynLimb* Robot::get_limb(int part)
{
	switch(part)
	{
		case LEFT_ARM:	return this->left_arm->limb;
		case RIGHT_ARM:	return this->right_arm->limb;
		case NECK:		return this->head_neck->limb;
	}
	return NULL;
}

int Robot::get_dof(int part)
{
	switch(part)
	{
		case LEFT_ARM:	return this->left_arm->limb->getDOF();
		case RIGHT_ARM:	return this->right_arm->limb->getDOF();
		case NECK:		return this->head_neck->limb->getDOF();
	}
	return -1;
}

bool Robot::get_torso()
{
	return this->torso;
}

Robot::~Robot()
{
	delete left_arm;
	delete right_arm;
	delete head_neck;
}
