//Robot class
//It sets and contains all the drivers and dynamic chains of the Robot
//left_arm, right_arm and neck

//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#include <iostream>
#include <iCub/iDyn/iDyn.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#define LEFT_ARM 	0
#define RIGHT_ARM 	1
#define NECK		2

using namespace yarp;
using namespace iCub::iDyn;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

namespace HandSome
{
	struct body_part{
		PolyDriver	*driver;	//Polydriver of the device
		iDynLimb	*limb;		//Dynamic properties
	};
	
	class Robot
	{
	 
	 private:
	 	body_part *left_arm;
	 	body_part *right_arm;
	 	body_part *head_neck;
	 	bool torso;
	 	
	 public:
	 	Robot(bool);
	 	~Robot();
	 	
	 	int get_dof(int);
	 	bool get_torso();
	 	bool init(string);
	 	body_part* get_part(int);
	 	PolyDriver* get_driver(int);
	 	iDynLimb* get_limb(int);
	};
}
