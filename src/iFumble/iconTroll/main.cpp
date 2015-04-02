// iconTroll (an iFumble module)
// main.cpp : v0.1 : Defines the entry point for the application.
//

// Includes

#include <stdio.h>
#include "yarp/os/all.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include "VisionReader.h"

using namespace yarp::os;

// Definitions

#define CMD_CALI					VOCAB4('c','a','l','i')
#define CMD_VISI					VOCAB4('v','i','s','i')
#define CMD_EXPL					VOCAB4('e','x','p','l')
#define CMD_GOLF					VOCAB4('g','o','l','f')
#define CMD_QUIT					VOCAB4('q','u','i','t')
#define CMD_FIDL					VOCAB4('f','i','d','l')

#define TIME_STEP					0.1 // [s]

// Procedures

int main(int argc, char *argv[])
{
	// Object construction
	Network yarp;
	ConstString bottlestring;
	Port trollCmdPort;
	Port fumblyRpcPort;
	Port learnRpcPort;
	BufferedPort<Bottle> cstuffPort;
	BufferedPort<Bottle> learnPort;
	Bottle trollCmdBottle, fumblyRpcBottleO, fumblyRpcBottleI, learnRpcBottleI, learnRpcBottleO;
    Bottle& learnerBottle = learnPort.prepare();
	int cmd, objectIndex;	
	bool connected;
	VisionReader vs;
	double objX, objY, objZ, objDX, objDY, objDZ;
	
	// Object initialization
	trollCmdPort.open("/iFumble/iconTroll/rpc");
	fumblyRpcPort.open("/iFumble/iconTroll/toFumblyRpc");
	cstuffPort.open("/iFumble/iconTroll/toCStuff");
	learnPort.open("/iFumble/iconTroll/toLearnStuff");
	learnRpcPort.open("/iFumble/iconTroll/toLearnRpc");
	vs.open("/iFumble/iconTroll/fromVS");
	connected = false;
	objDX = objDY = objDZ = objectIndex = 0;
	objX = -0.25;
	objY = 0;
	objZ = 0.20;
		
	// Main RPC loop
	std::cout<<"Module iconTroll ARE GO!"<<std::endl;	
	std::cout<<"Waiting for a command (calibrate, explore, golf, quit)..."<<std::endl;	
	do
	{
		// Get a command (from "/iFumble/iconTroll/rpc")
		trollCmdBottle.clear();
		trollCmdPort.read(trollCmdBottle);
		cmd = trollCmdBottle.get(0).asVocab();
        
		switch (cmd)
		{
			case CMD_CALI:
			{
				std::cout<<"Calibrating..."<<std::endl;

				fumblyRpcBottleO.clear();
				fumblyRpcBottleI.clear();
				fumblyRpcBottleO.addVocab(CMD_CALI);				
				fumblyRpcPort.write(fumblyRpcBottleO,fumblyRpcBottleI);
				std::cout<<"REC: "<< fumblyRpcBottleI.toString() << std::endl;

				break;
			}
			case CMD_VISI:
			{
			  std::vector <int> currentObjects = vs.getCurrObjectIndices();
			  for(int i=0;i<currentObjects.size();i++){
			    double xx,yy,zz;
			    vs.getCoords(i, xx, yy, zz); 
			    std::cout<<"VISION PROVIDES LABEL "<<vs.getObjectLabel(i).c_str()<<" x="<<xx<<" y="<<yy<<" z="<<zz<<std::endl;
			  }
			  
			}
			case CMD_EXPL:
			{
				std::cout<<"Exploring..."<<std::endl;

				// Get number of trials
				int trials = 1;				
				if (trollCmdBottle.size() > 1)
					trials = trollCmdBottle.get(1).asInt();
				
				// For each trial
				for (int i = 0; i < trials; i++)
				{
				  
				  double thetai,thetaf;
				  
				  do{
				    thetai=2*M_PI*((double)rand())/RAND_MAX;
				  }while(   (thetai) >=5*M_PI/4   &&   (thetai)<=7*M_PI/4   );
				  
				  do{
				    thetaf=thetai+(0.2)*M_PI*((double)rand())/RAND_MAX-0.1;
				  }while(   (thetaf) >=5*M_PI/4   &&   (thetaf)<=7*M_PI/4   );
				  				  
				  
				  double execTime=0.1+2*((double)rand())/RAND_MAX;
				  
					std::cout<<" - trial " << i+1 << std::endl;
					
					// Read objects in scene
					std::vector <int> currentObjects = vs.getCurrObjectIndices();
					
					// Choose an object
					objectIndex = currentObjects[rand()%currentObjects.size()];
					
					// Get object coordinates from vision	
					vs.getCoords(objectIndex, objX, objY, objZ);
					std::cout<<"REC(V): " << objX << " " << objY << " " << objZ << std::endl;
					
					// Select an action (at random or based on learner)
					fumblyRpcBottleO.clear();
					fumblyRpcBottleO.addVocab(CMD_FIDL);				
					
					fumblyRpcBottleO.addDouble(objX); // x (double) from vision [m]
					fumblyRpcBottleO.addDouble(objY); // y (double) from vision [m]
					fumblyRpcBottleO.addDouble(objZ); // z (double) from vision (given by table height) [m]
					
					
					fumblyRpcBottleO.addDouble(thetai); // theta (double): the hand angle used initially (orientation given by robot) [radians] 
					fumblyRpcBottleO.addDouble(0.05); // rstart (double): the initial hand distance to object [m]
					fumblyRpcBottleO.addDouble(thetaf); // ftheta (double): the final hand angle (orientation given by robot) [radians]
					fumblyRpcBottleO.addDouble(0.05); // rstop (double): the final hand distance relative to the initial object position [m]
					fumblyRpcBottleO.addDouble(execTime); // execTime (double): the reference velocity [s]
					
					// ...
					//5*pi/4 to 7*pi/4. 
					
					/*
						* fidl (vocab): VOCAB4('f','i','d','l')
						* x (double): target good ol' x [m] (provided by eye2world)
						* y (double): target good ol' y [m] (provided by eye2world)
						* z (double): target good ol' z [m] (provided by the table height in fact)
						* theta (double): the hand angle used initially (orientation given by robot) [radians]
						* rstart (double): the initial hand distance to object [m]
						* ftheta (double): the final hand angle (orientation given by robot) [radians]
						* rstop (double): the final hand distance relative to the initial object position [m]
						* execTime (double): the reference velocity [s]
					*/
				
					// Execute action
					fumblyRpcBottleI.clear();
					fumblyRpcPort.write(fumblyRpcBottleO,fumblyRpcBottleI); // Wait here until action is finished?
					std::cout<<"REC(F): "<< fumblyRpcBottleI.toString() << std::endl;								
								
					// Get (or wait for?) feedback (consequences) from vision
					
					Time::delay(2.0);
					
					objDX = objX;
					objDY = objY;
					objDZ = objZ;
					
					vs.getCoords(objectIndex, objX, objY, objZ);
					
					objDX = objX - objDX;
					objDY = objY - objDY;
					objDZ = objZ - objDZ;
					
					// Send data (action and consequences) to learner
					
					learnerBottle.clear();
					learnerBottle.addString("DATAPOINT");
					//learnerBottle.addString(vs.getObjectLabel(objectIndex).c_str()); // object_label
					learnerBottle.addString("0"); // object_label
					learnerBottle.addDouble(thetai); // theta
					learnerBottle.addDouble(thetaf); // final theta
					learnerBottle.addDouble(execTime); // execution time (velocity)
					learnerBottle.addDouble(objDX); // x_delta
					learnerBottle.addDouble(objDY); // y_delta
					learnPort.write();
					//others --> category acion--> initial_theta, final_theta, execution_time/velocity,  from vision--> x_delta, y_delta
				
				}
				
				break;
			}
			case CMD_GOLF:
			{
				std::cout<<"Golfing..."<<std::endl;	
				
				// Get number of trials
				int trials = 1;				
				if (trollCmdBottle.size() > 1)
					trials = trollCmdBottle.get(1).asInt();					

				// For each trial
				for (int i = 0; i < trials; i++)
				{
					std::cout<<" - trial " << i+1 << std::endl;				
					
					// Locate goal location (using vision?)
				    const double hardx = -0.2;
				    const double hardy = 0.0;
                    double objx,objy,objz;
					// Select best action to achieve goal (using Learner)
				    //vs.getCoords(int index,double &x,double &y,double &z);
                    vs.getCoords(0,objx,objy,objz);
                    objx=hardx-objx;
                    objy=hardy-objy;
					learnRpcBottleI.clear();
					learnRpcBottleO.clear();
                    learnRpcBottleO.addString("INFER");
                    learnRpcBottleO.addString("0");
                    learnRpcBottleO.addInt(2);
                    learnRpcBottleO.addInt(3);
                    learnRpcBottleO.addInt(4);
                    learnRpcBottleO.addDouble(objx);
                    learnRpcBottleO.addDouble(objy);
					learnRpcPort.write(learnRpcBottleO,learnRpcBottleI); // Wait here until action is finished?

                    // Execute action
					fumblyRpcPort.write(fumblyRpcBottleO,fumblyRpcBottleI); // Wait here until action is finished?
				
					// Repeat...?
				}	
				break;
			}		
		}							
	} while (cmd != CMD_QUIT);
	
	std::cout<<"Terminating iconTroll uplink..."<<std::endl;
	
	trollCmdPort.close();
	fumblyRpcPort.close();
	cstuffPort.close();
	learnPort.close();
				
	return 0;
}


/*
	Bottle& fumblyCmdBottleO = fumblyCmdPort.prepare(); // ?????
	
	Bottle fumblyRpcBottleI;
	Bottle fumblyRpcBottleO;
	fumblyRpcBottleI.clear();
	fumblyCmdBottleO.clear();
	
fumblyCmdBottleO.addVocab(VOCAB4('c','a','l','i'));
fumblyCmdBottleO.addVocab(VOCAB4('t','a','b','l'));
fumblyCmdPort.write();
//	fumblyPort.read();
// Welcome message
printf("Aloha!\n");
////////////////////////////////////// MAIN LOOP ///////////////////////////////////
bool theBooleanWarrior=false;
do{
	Time::delay(1);
	std::cout<<"Done? "<<std::endl;
	fumblyRpcBottleO.clear();
	fumblyRpcBottleO.addVocab(VOCAB4('s','t','a','t'));
	fumblyRpcPort.write(fumblyRpcBottleO,fumblyRpcBottleI);
	theBooleanWarrior=true;
	theBooleanWarrior=theBooleanWarrior&&(fumblyRpcBottleI.get(0).asInt()==0);
}while(!theBooleanWarrior);
 // while (true) {
//Time::delay(TIME_STEP);
 // }
*/
