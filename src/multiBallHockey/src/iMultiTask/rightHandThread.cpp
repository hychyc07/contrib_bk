#include "rightHandThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

using namespace std;

YARP_DECLARE_DEVICES(icubmod)
#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     45.0    // [deg]

// constructor
RightHandThread::RightHandThread(int period)
        : TaskThread(period) 
{	

movedRight = false;
movedLeft = false;
resettedTorso = false;

//igaze = ig;
homeRight.resize(3);
homeRight[0]= -0.2;
homeRight[1]= 0.3;
homeRight[2]= 0.05;


homeLeft.resize(3);
homeLeft[0]= -0.2;
homeLeft[1]= -0.3;
homeLeft[2]= 0.05;

oRight.resize(3,3);
oRight.zero();
//oRight(0,0) = -1; 
//oRight(1,2) = -1;
//oRight(2,1) = -1;
oRightHome.resize(3,3);
oRight(0,2) = -1;
oRight(1,0) = 1;
oRight(2,1) = -1;

oRightHome.zero();
oRightHome(0,0) = -1;
oRightHome(1,2) = -1;
oRightHome(2,1) = -1;


oLeft.resize(3,3);
oLeft.zero();
oLeft(0,2) = -1;
oLeft(1,0) =  1;
oLeft(2,1) = -1;

oLeftHome.resize(3,3);
oLeftHome.zero();
oLeftHome(0,1) =  1;
oLeftHome(1,0) =  1;
oLeftHome(2,2) = -1;

prevHitPos.resize(3);
prevHitPos[0] = -1;
prevHitPos[1] = -1;
prevHitPos[2] = -1;
nextPos.resize(3);
nextPos[0] = -1;
nextPos[1] = -1;
nextPos[2] = -1;	
}


 void RightHandThread::limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;
	resetCounter = 0;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        
	arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min, MAX_TORSO_PITCH);


	int laxis=0; // pitch joint
        double lmin, lmax;
	leftarm->getLimits(laxis,&lmin,&lmax);
        leftarm->setLimits(laxis,lmin, MAX_TORSO_PITCH);

    }


void  RightHandThread::initTorsoPosControl(){
    Property options;
    options.put("device","remote_controlboard");
    options.put("local", "/positionControl/torso");   //local port names
    options.put("remote", "/icub/torso"); // remote port where we connect to

    if (!torsoPosDevice.open(options)){
        printf("Torso device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
	exit(0);
    }
    bool ok;
    ok = torsoPosDevice.view(posTorso);
    ok = ok && torsoPosDevice.view(encsTorso);

     if (!ok) {
        printf("Problems acquiring torso interfaces\n");
        exit(0);
    }

      
    posTorso->getAxes(&njTorso);
    Vector tmp;
    encodersTorso.resize(njTorso);
    tmp.resize(njTorso);
    commandTorso.resize(njTorso);	

    int i;
    for (i = 0; i < njTorso; i++) {
         tmp[i] = 50.0;
    }
    posTorso->setRefAccelerations(tmp.data());

    for (i = 0; i < njTorso; i++) {
        tmp[i] = 20.0;
        posTorso->setRefSpeed(i, tmp[i]);
    }
	

}



void  RightHandThread::initRightArmPosControl(){
    Property options;
    options.put("device","remote_controlboard");
    options.put("local", "/positionControl/right_arm");   //local port names
    options.put("remote", "/icub/right_arm"); // remote port where we connect to

    if (!rightArmPosDevice.open(options)){
        printf("Right arm device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
	exit(0);
    }
    bool ok;
    ok = rightArmPosDevice.view(posRight);
    ok = ok && rightArmPosDevice.view(encsRight);

     if (!ok) {
        printf("Problems acquiring right arm interfaces\n");
        exit(0);
    }

      
    posRight->getAxes(&njRight);
    Vector tmp;
    encodersRight.resize(njRight);
    tmp.resize(njRight);
    commandRight.resize(njRight);	

    int i;
    for (i = 0; i < njRight; i++) {
         tmp[i] = 50.0;
    }
    posRight->setRefAccelerations(tmp.data());

    for (i = 0; i < njRight; i++) {
        tmp[i] = 15.0;
        posRight->setRefSpeed(i, tmp[i]);
    }
	


}



void  RightHandThread::initLeftArmPosControl(){
    Property options;
    options.put("device","remote_controlboard");
    options.put("local", "/positionControl/left_arm");   //local port names
    options.put("remote", "/icub/left_arm"); // remote port where we connect to

    if (!leftArmPosDevice.open(options)){
        printf("Left arm device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
	exit(0);
    }
    bool ok;
    ok = leftArmPosDevice.view(posLeft);
    ok = ok && leftArmPosDevice.view(encsLeft);

     if (!ok) {
        printf("Problems acquiring left arm interfaces\n");
        exit(0);
    }

      
    posLeft->getAxes(&njLeft);
    Vector tmp;
    encodersLeft.resize(njLeft);
    tmp.resize(njLeft);
    commandLeft.resize(njLeft);	

    int i;
    for (i = 0; i < njLeft; i++) {
         tmp[i] = 50.0;
    }
    posLeft->setRefAccelerations(tmp.data());

    for (i = 0; i < njLeft; i++) {
        tmp[i] = 15.0;
        posLeft->setRefSpeed(i, tmp[i]);
    }
	


}

bool RightHandThread::threadInit(){
  

	initLeftArmPosControl();
	initRightArmPosControl();
	initTorsoPosControl();
	
	// right arm
	Property optionLeft("(device cartesiancontrollerclient)");
        optionLeft.put("remote","/icub/cartesianController/left_arm");
        optionLeft.put("local","/cartesian_client/left_arm");

       if (!leftclient.open(optionLeft))
            return false;

        // open the view
        leftclient.view(leftarm);
	leftarm->storeContext(&startup_context_left_init);

        // set trajectory time
        leftarm->setTrajTime(1.2);

	


        // get the torso dofs
        Vector newDofl, curDofl;
        leftarm->getDOF(curDofl);
        newDofl=curDofl;

	// from here
	// enable the torso yaw and pitch
        // disable the torso roll
        newDofl[0]=1;
        newDofl[1]=1; // roll
        newDofl[2]=1;
	// impose some restriction on the torso pitch
        //limitTorsoPitch();

        // send the request for dofs reconfiguration
        leftarm->setDOF(newDofl,curDofl);


        leftxd.resize(3);
        //od.resize(4);

	// full roll context
	leftarm->storeContext(&startup_context_left_dof10);
	leftarm->setTrackingMode(false);
	


	//-------------------------- right arm
	 Property option("(device cartesiancontrollerclient)");
        option.put("remote","/icub/cartesianController/right_arm");
        option.put("local","/cartesian_client/right_arm");

        if (!client.open(option))
            return false;

        // open the view
        client.view(arm);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        

	arm->storeContext(&startup_context_right_init);

        // set trajectory time
        arm->setTrajTime(1.2);

        // get the torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;
        newDof[1]=1; // roll
        newDof[2]=1;
	// impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        arm->setDOF(newDof,curDof);

        xd.resize(3);
        //od.resize(4);

	// full roll context
	arm->storeContext(&startup_context_right_dof10);

	// no roll context
	 newDof[1]=0; // roll
	 arm->setDOF(newDof,curDof);
	arm->storeContext(&startup_context_right_dof9);
	arm->setTrackingMode(false);

	firstPosSet = false;

	//setRightHighWeight();


	resetLeftJoints();
	resetRightJoints();
	reset();

   return true;		  
}

void RightHandThread::threadRelease(){
  
  reset();
       arm->stopControl();
	leftarm->stopControl();
        // it's a good rule to restore the controller
        // context as it was before opening the module
        arm->restoreContext(startup_context_right_init);
	leftarm->restoreContext(startup_context_left_init);
        client.close();	
	leftclient.close();	

	leftArmPosDevice.close();
	rightArmPosDevice.close();
	torsoPosDevice.close();
	
}

void RightHandThread::stop( )
{
	arm->restoreContext(startup_context_right_init);
	leftarm->restoreContext(startup_context_left_init);
}

void RightHandThread::run( )
{
	//std::cout << "arm running......" << std::endl;
	runNextTask();
}
// 
void RightHandThread::setNextTaskPosition(Vector pos){
        pos[0] = -0.4;
	pos[2] = 0.0; // height (z-axis) at which the robot will tap (table = -0.12)
	//if (pos[1] > 0.4){pos[1] = 0.4;}
	//if  (pos[1] < -0.4){pos[1] = -0.4;}
	if ((pos[1] <= 0.4) && (pos[1] >= -0.4)){
	
	
		firstPosSet = true;
		nextPos = pos;
		taskDone = false;
		std::cout << "setting next pos to" <<  pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
	
	}
}


bool RightHandThread::runNextTaskRight(){
	
	return true;
}
bool RightHandThread::runNextTaskLeft(){
	
	return true;
}

void RightHandThread::setRightHighWeight(){
	Vector restWl;
	leftarm->getRestWeights(restWl);
	//restWl[0] = 10.0; 
	restWl[1] = 10.0; // roll
	//restWl[2] = 10.0;
	leftarm->setRestWeights(restWl, restWl);

	Vector restWr;
	arm->getRestWeights(restWr);
	//restWr[0] = 1.0;
	restWr[1] = 1.0; // roll
	//restWr[2] = 1.0;
	arm->setRestWeights(restWr, restWr);
}

void RightHandThread::setLeftHighWeight(){
	Vector restWr;
	arm->getRestWeights(restWr);
	restWr[1] = 10.0;
	//restWr[0] = 10.0;
	//restWr[2] = 10.0;
	arm->setRestWeights(restWr, restWr);


	Vector restWl;
	leftarm->getRestWeights(restWl);
	restWl[1] = 1.0;
	//restWl[0] = 1.0;	
	//restWl[2] = 1.0;
	leftarm->setRestWeights(restWl, restWl);

}


bool RightHandThread::runNextTask( ){
  bool ret = true;

  float diff = sqrt((nextPos[0] - prevHitPos[0])*(nextPos[0] - prevHitPos[0]) + (nextPos[1] - prevHitPos[1]) * (nextPos[1] - prevHitPos[1]) + (nextPos[2] - prevHitPos[2]) * (nextPos[2] - prevHitPos[2]));
	
  if (diff < 0.1) return true;
	
  prevHitPos = nextPos;
	
  if (firstPosSet){
   	firstPosSet = false;	
   	// right hand
	if (nextPos[1] > 0.05){
		movedRight = true;
		resettedTorso = false;
		setRightHighWeight();
		std::cout << "right hand running task to" <<  nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
		resetLeftJoints();
		arm->goToPose(nextPos, dcm2axis(oRight));		
	}

	// left hand
	else if (nextPos[1] < -0.05){
		movedLeft = true;
		resettedTorso = false;
		setLeftHighWeight();
		std::cout << "left hand running task to" <<  nextPos[0] << ", " << nextPos[1] << ", " << nextPos[2] << std::endl;
		resetRightJoints();
		Vector od;
	 	od.resize(4);
	 	od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;	
		leftarm->goToPose(nextPos, od);
	}
	
   }
   
  return ret;
}

bool RightHandThread::resetRightCartesian(){
	std::cout << "right arm cartesian resetting...." << std::endl;
	arm->goToPoseSync(homeRight, dcm2axis(oRightHome));
	return true;
}
bool RightHandThread::resetLeftCartesian(){
	std::cout << "left arm cartesian resetting...." << std::endl;
	Vector od;
	 od.resize(4);
	 od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
	leftarm->goToPoseSync(homeLeft, od);
	return true;
}

bool RightHandThread::resetTorso(){

 	bool ret = true;
	   resettedTorso = true;
    std::cout << "resetting torso............." << std::endl;

    commandTorso=0;
    
    commandTorso[0]=0;
    commandTorso[1]=0;
    commandTorso[2]=-1;

    posTorso->positionMove(commandTorso.data());
	bool done = false;
	while(!done)
    	{
        	posTorso->checkMotionDone(&done);
        	Time::delay(0.1);
    	}

  	
   return ret;

}



bool RightHandThread::resetLeftJoints(){
	
	
	movedLeft = false;
 	bool ret = true;
	 
    std::cout << "joint resetting left arm............." << std::endl;

	commandLeft=0;
    //now set the shoulder to some value
    commandLeft[0]=-60;
    commandLeft[1]=70;
    commandLeft[2]=0;
    commandLeft[3]=50;
    posLeft->positionMove(commandLeft.data());

  	
   return ret;

}


bool RightHandThread::resetRightJoints(){
   bool ret = true;
   movedRight = false;
   std::cout << "joint resetting right arm............." << std::endl;
  
	 commandRight=0;
    //now set the shoulder to some value
    commandRight[0]=-60;
    commandRight[1]=70;
    commandRight[2]=0;
    commandRight[3]=50;
    posRight->positionMove(commandRight.data());

   return ret;
}


bool RightHandThread::reset(){
	firstPosSet = false;
	
	if (movedRight){
		resetRightCartesian(); // cartesian control
	}
	if (movedLeft){
		resetLeftCartesian(); // cartesian control
	}
	if (!resettedTorso){
		//resetTorso(); // position control
	}
	
	return true;
}


bool RightHandThread::terminateCurrentTask( ){
  bool ret = false;
  return ret;
}

