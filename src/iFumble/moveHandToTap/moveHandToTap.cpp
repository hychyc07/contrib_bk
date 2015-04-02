
#include <iostream>
//#include <iomanip>


#ifdef WIN32
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif



// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/Network.h>

#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

using namespace std;


int main()
{

  YARP_REGISTER_DEVICES(icubmod)

    string ARM_USED = "left";

 Network yarp; // set up yarp
 BufferedPort<Bottle> inport;  
 inport.open("/iFumble/moveHand2Tap/in");


 // right arm home position
 Vector homeRight; homeRight.resize(3);
 homeRight[0]= -0.2;
 homeRight[1]= 0.3;
 homeRight[2]= 0.1;

 
 // left arm home position
 Vector homeLeft; homeLeft.resize(3);
 homeLeft[0]= -0.2;
 homeLeft[1]= -0.3;
 homeLeft[2]= 0.1;

 // right arm
 Property option;
 option.put("device","cartesiancontrollerclient");
 option.put("remote","/icub/cartesianController/right_arm");
 option.put("local","/moveHandToTap/right_arm");
 PolyDriver clientCartCtrlRight(option);
 ICartesianControl *armCartRight=NULL;
 if (clientCartCtrlRight.isValid()) {
   clientCartCtrlRight.view(armCartRight);

 } else {
   return 0;
 }


  Matrix oRight;
  oRight.resize(3,3);
  oRight.zero();
  oRight(0,0) = -1;
  oRight(1,2) = -1;
  oRight(2,1) = -1;
 
 if (ARM_USED=="left"|ARM_USED=="both")
   {
     armCartRight->goToPose(homeRight, dcm2axis(oRight));
     armCartRight->setTrackingMode(true);
   }

 // left arm
 Property option2;
 option2.put("device","cartesiancontrollerclient");
 option2.put("remote","/icub/cartesianController/left_arm");
 option2.put("local","/moveHandToTap/left_arm");
 PolyDriver clientCartCtrlLeft(option2);
 ICartesianControl *armCartLeft=NULL;
 if (clientCartCtrlLeft.isValid()) {
   clientCartCtrlLeft.view(armCartLeft);
   
 } else {
   return 0;
 }


  Matrix oLeft;
  oLeft.resize(3,3);
  oLeft.zero();
  oLeft(0,0) = -1;
  oLeft(1,2) = -1;
  oLeft(2,1) = -1;

  //armCartLeft->getPose(x0,oLeft);
  if (ARM_USED=="left"|ARM_USED=="both")
    {
      armCartLeft->goToPose(homeLeft, dcm2axis(oLeft));
      armCartLeft->setTrackingMode(true);
    }
 
 // head
 Property option3;
 option3.put("device","gazecontrollerclient");
 option3.put("remote","/iKinGazeCtrl");
 option3.put("local","/moveHandToTap/gaze");
 
 PolyDriver clientGazeCtrl(option3);

 IGazeControl *igaze=NULL;
 if (clientGazeCtrl.isValid()) {
   clientGazeCtrl.view(igaze);
 } else {
   return 0;
 }



 //   BufferedPort<Bottle> outport;  
 //outport.open("/moveHand2Tap/out");

  Vector newPos; newPos.resize(3);


 double sendx, sendy, sendz;
 sendz = 0.05;
  
  while (true)
  {

  
    // read in data from traza
    Bottle *input = inport.read();
    if ((input!=NULL) && (input->size()==4)){
      //cout << "got " << input->toString().c_str() << endl;
              
           sendx= input->get(1).asDouble();
	   sendy= input->get(2).asDouble();
	   //sendz= input->get(2).asDouble();
	   cout << "received: " << sendx << ", " << sendy << endl;

	   newPos[0] = sendx;
	   newPos[1] = sendy;
	   newPos[2] = sendz;
	   // always send gaze
	   igaze->lookAtFixationPoint(newPos);
	   // check if the values are not dangerous to robot
	   if ((sendx < -0.02) && (sendx > -1.0) && (sendy < 1.0) && (sendy > -1.0)){
	     // use right hand
	     if (sendy >= 0.0){
	       // reset left hand
	       if (ARM_USED=="left"|ARM_USED=="both")
		 armCartLeft->goToPose(homeLeft, dcm2axis(oLeft));
	       // move right hand
	       if (ARM_USED=="right"|ARM_USED=="both")
		 armCartRight->goToPose(newPos, dcm2axis(oRight));		      

	       
	     } else{// use left hand
	        // reset right hand
	        if (ARM_USED=="right"|ARM_USED=="both")
		  armCartRight->goToPose(homeRight, dcm2axis(oRight));
	       // move left hand
		 if (ARM_USED=="left"|ARM_USED=="both")
		   armCartLeft->goToPose(newPos, dcm2axis(oLeft));
	     }
	    
	   }
	   else{
	     cout << "ignoring out of range requests " << endl;
	   }
     }
    else{ // if no input..do something??

    }

      
      
    
    usleep(10e2);
  }

  clientCartCtrlLeft.close();
  clientCartCtrlRight.close();
  clientGazeCtrl.close();
 
  inport.close();
  
  return 0;
}
