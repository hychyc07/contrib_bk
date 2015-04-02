
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete, Alexander Schmitz
 * email:   andrea.delprete@iit.it, alexander.schmitz@iit.it
 * website: www.robotcub.org 
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


#include <sstream>			// string stream
#include <yarp/os/Vocab.h>

#include <iCub/skinDynLib/rpcSkinManager.h>
#include "iCub/tactileGrasp/TactileGrasp.h"

using namespace iCub::tactileGrasp;
using namespace iCub::skinManager;
using namespace yarp::os;


// the order of the command in this list MUST correspond to the order of the enum TactileGrasp::TactileGraspCommand
const string TactileGrasp::COMMAND_LIST[]  = {
	"control", "open hand", "calibrate", 
	"grasp tough", "grasp soft", "grasp compliant", 
	"stop", "set touch threshold", "set desired touch", 
	"set kp", "set ki", "set kd", 
	"help", "quit"};

// the order in COMMAND_DESC must correspond to the order in COMMAND_LIST
const string TactileGrasp::COMMAND_DESC[]  = {
	"start the pid controller for controlling the finger positions so as to mantain the desired touch value",
	"set the arm joints to the initial position", 
	"calibrate the tactile sensors", 
	"grasp without using the tactile feedback", 
	"grasp until touch is detected", 
	"grasp when no touch is detected, otherwise open the hand", 
	"stop the grasping and reset the arm joint positions", 
	"set the touch threshold to the specified value (int in [0,255])", 
	"set the desired touch that the pid controller must try to mantain (float in [0, 255])",
	"set the proportional gain for the specified finger or all the finger if no index is specified (float >= 0)", 
	"set the integrative gain for the specified finger or all the finger if no index is specified (float >= 0)", 
	"set the derivative gain for the specified finger or all the finger if no index is specified (float >= 0)",
	"get this list", 
	"quit the module"};


// message to send to the skinDrifCompensation module in order to force the sensors calibration
//const string TactileGrasp::CALIBRATION_COMMAND[] = {"force", "calibration"};
//const unsigned int TactileGrasp::CALIBRATION_COMMAND_LENGTH = 2;	// size of CALIBRATION_COMMAND array
//
//// message to send to the skinDrifCompensation module in order to get the percentile
//const string TactileGrasp::GET_PERCENTILE_COMMAND[] = {"get", "percentile"};
//const unsigned int TactileGrasp::GET_PERCENTILE_COMMAND_LENGTH = 2;	// size of GET_PERCENTILE_COMMAND array


bool TactileGrasp::configure(yarp::os::ResourceFinder &rf)
{    
	/* Process all parameters from both command-line and .ini file */

	/* get the module name which will form the stem of all module port names */
	moduleName		= rf.check("name", Value("tactileGrasp"), "module name (string)").asString();
	robotName		= rf.check("robot", Value("icub"), "name of the robot (string)").asString();
	/* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name*/
	setName(moduleName.c_str());
	
	// read the "hand" parameter (mandatory)
	if(rf.check("left_hand")){
	    rightHand = false;
	    printf("Using left hand\n");
    }
    else{
        rightHand = true;
	    printf("Using right hand\n");
    }
	/*string hand		= rf.check("hand", Value("right"), "Hand to take as reference (string)").asString().c_str();
	if(hand.compare("right")==0){
		rightHand = true;		
	}else if(hand.compare("left")==0){
		rightHand = false;
	}else{
		fprintf(stderr, "The hand specified is not valid: %s\n", hand.c_str());
		return false;
	}*/	


	// read the port names, if specified
	string skinManagerRpcPortName   = "/";
	string handlerPortName			= "/";
	skinManagerRpcPortName  		+= rf.check("skinManagerName", Value("skinManager")).asString() + "/rpc";
	handlerPortName					+= getName(rf.check("handlerPort", Value("/rpc")).asString());


	// open the ports
	if (!skinDriftCompRpcPort.open(("/"+moduleName+"/skinManagerRpc").c_str())) {
		cout << getName() << ": unable to open the skinManager rpc output port " << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}
	if (!handlerPort.open(handlerPortName.c_str())) {
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}
    // connect to skinManager rpc port
    if( !Network::connect(skinDriftCompRpcPort.getName().c_str(), skinManagerRpcPortName.c_str()) ){
        cout << getName() << ": Unable to connect to skinManager rpc port " << skinManagerRpcPortName.c_str() << endl;  
		return false;
    }

	attach(handlerPort);                  // attach to rpc port

	// create and start the thread to do the work
	graspThread = new GraspThread(&rf, robotName, moduleName, rightHand);
	graspThread->start(); // this calls threadInit() and it if returns true, it then calls run()	

	percentileHasBeenSet = false;	// so that the percentile is set when the first command is received on the rpc port

	return true ;      // let the RFModule know everything went well, so that it will then run the module
}


//bool TactileGrasp::moveHead()
//{
//	Property params;
//	params.put("robot", robotName.c_str());
//	params.put("part", "head");
//	params.put("device", "remote_controlboard");
//	params.put("local", ("/"+moduleName+"/head_client").c_str());
//	params.put("remote", ("/"+robotName+"/head").c_str());
//
//
//	// create a device
//	PolyDriver robotDevice(params);
//	if (!robotDevice.isValid()) {
//		printf("Device not available.  Here are the known devices:\n");
//		printf("%s", Drivers::factory().toString().c_str());
//		return false;
//	}
//
//	IPositionControl *pos;
//	bool ok = robotDevice.view(pos);
//	if (!ok) {
//		printf("Problems acquiring interfaces\n");
//		return false;
//	}	
//
//	// set speed
//	int nj=0;
//	pos->getAxes(&nj); //the number of axes
//	for (int i = 0; i < nj; i++) {
//		pos->setRefSpeed(i, REF_SPEED_ACC);
//	}
//
//	pos->positionMove(0 ,-30);
//	pos->positionMove(1 ,  0);
//	pos->positionMove(3 ,  0);
//	pos->positionMove(4 ,  0);
//	pos->positionMove(5 ,  0);
//	if (rightHand) { 		
//		pos->positionMove(2 ,-20);		
//	}
//	else { 
//		pos->positionMove(2 , 20);		
//	}
//	return true;
//}

bool TactileGrasp::interruptModule(){
	fprintf(stderr, "\nGoing to interrupt the tactile grasp module.\n");
	skinDriftCompRpcPort.interrupt();
	handlerPort.interrupt();

	return true;
}


bool TactileGrasp::close(){
	/* stop the thread */
	if(graspThread && graspThread->isRunning()){
		fprintf(stderr, "\nGoing to stop the grasping thread.\n");
		graspThread->stop();
	}

	skinDriftCompRpcPort.close();
	handlerPort.close();	

	return true;
}


/* 
 * Send the calibration command to the skinDriftCompensation rpc port.
*/
void TactileGrasp::sendCalibrationMess(){
	Bottle calibBottle;
    calibBottle.addInt(iCub::skinManager::calibrate);
	skinDriftCompRpcPort.write(calibBottle);
	Time::delay(1.0);
}

/*
 * Get the percentile values from the skinDriftCompensation module and set the read values
 * in the grasping thread.
 * Return true if the operation succeeds, false otherwise.
 */
bool TactileGrasp::updatePercentile(){
	Bottle commandBot, percentileBot;
    commandBot.addInt(skinManager::get_touch_thr);
	skinDriftCompRpcPort.write(commandBot, percentileBot);			// send command, wait for reply

	vector<float> percentile;
	for(int i=0; i<percentileBot.size(); i++)
		percentile.push_back((float)percentileBot.get(i).asDouble());
	
	if(graspThread->setPercentile(percentile)){
		fprintf(stderr, "Set new percentile: %s\n", percentileBot.toString().c_str());
		return true;
	}	
	fprintf(stderr, "ERROR while setting the new percentile: %s\n", percentileBot.toString().c_str());
	return false;
}

/**
  * Respond to the command received by the rpc port.
  */
bool TactileGrasp::respond(const Bottle& command, Bottle& reply) {	
	string helpMessage =  string(getName().c_str()) + " commands are: ";
	reply.clear();

	TactileGraspCommand com;
	if(!identifyCommand(command, com)){
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	// the first time a command is received, update the percentile
	if(!percentileHasBeenSet){		
		percentileHasBeenSet = updatePercentile();
	}

	switch( com ){
		case quit:
			reply.addString("quitting");
			return false;

		case help:
			reply.addVocab(Vocab::encode("many"));				// print every string added to the bottle on a new line
			reply.addString(helpMessage.c_str());
			for(unsigned int i=0; i< COMMANDS_COUNT; i++){
				reply.addString( ("- "+COMMAND_LIST[i]+": "+COMMAND_DESC[i]).c_str() );
			}
			return true;

		case calibrate: 
			sendCalibrationMess();
			if(updatePercentile())
				reply.addString("Calibration finished. Percentile updated.");
			else
				reply.addString("Error in calibration or percentile update.");
			break;

		case set_threshold:
		{
			if(command.size()<4 || !command.get(3).isInt()){
				reply.addString("New touch threshold value missing or not an int! Touch threshold not updated.");
				return true;
			}
						
			stringstream temp;
			if(graspThread->setTouchThreshold(command.get(3).asInt())){				
				temp<< "New touch threshold set: "<< command.get(3).asInt();				
			}
			else{
				temp<< "ERROR in setting new touch threshold: "<< command.get(3).asInt();
			}
			reply.addString( temp.str().c_str());
			return true;
		}

		case set_desired_touch:
		{
			if(command.size()<4 || (!command.get(3).isDouble() && !command.get(3).isInt())){
				reply.addString("New desired touch value missing or not a number! Desired touch not updated.");
				return true;
			}

			stringstream temp;
			if(graspThread->setTouchDesired((float)command.get(3).asDouble())){				
				temp<< "New desired touch set: "<< command.get(3).asDouble();				
			}
			else{
				temp<< "ERROR in setting new desired touch: "<< command.get(3).asDouble();
			}
			reply.addString( temp.str().c_str());
			return true;
		}

		case set_KP:
		{
			stringstream temp;
			if(command.size()==3){
				// the third parameter has to be a number
				if(!command.get(2).isDouble() && !command.get(2).isInt()){
					reply.addString("New KP value is not a number! KP not updated.");
					return true;
				}
				if(graspThread->setKP((float)command.get(2).asDouble()))
					temp<< "New KPs set: "<< command.get(2).asDouble();
				else
					temp<< "ERROR in setting new KP: "<< command.get(2).asDouble();
			}
			else if(command.size()>3){
				// the third parameter has to be a int and the fourth has to be a number
				if(!command.get(2).isInt() || (!command.get(3).isDouble() && !command.get(3).isInt())){
					reply.addString("Index is not as int or new KP value is not a number! KP not updated.");
					return true;
				}
				if( graspThread->setKP(command.get(2).asInt(), (float)command.get(3).asDouble()) )
					temp<< "KP of finger "<< command.get(2).asInt()<< " set: "<< command.get(3).asDouble();
				else
					temp<< "ERROR in setting new KP: "<< command.get(2).asDouble();
			}
			else
				temp<< "New KP value missing! KP not updated.";
						
			reply.addString( temp.str().c_str());
			return true;
		}

		case set_KI:
		{
			stringstream temp;
			if(command.size()==3){
				// the third parameter has to be a number
				if(!command.get(2).isDouble() && !command.get(2).isInt()){
					reply.addString("New KI value is not a number! KI not updated.");
					return true;
				}
				if(graspThread->setKI((float)command.get(2).asDouble()))
					temp<< "New KIs set: "<< command.get(2).asDouble();
				else
					temp<< "ERROR in setting new KI: "<< command.get(2).asDouble();
			}
			else if(command.size()>3){
				// the third parameter has to be a int and the fourth has to be a number
				if(!command.get(2).isInt() || (!command.get(3).isDouble() && !command.get(3).isInt())){
					reply.addString("Index is not as int or new KI value is not a number! KI not updated.");
					return true;
				}
				if( graspThread->setKI(command.get(2).asInt(), (float)command.get(3).asDouble()) )
					temp<< "KI of finger "<< command.get(2).asInt()<< " set: "<< command.get(3).asDouble();
				else
					temp<< "ERROR in setting new KI: "<< command.get(2).asDouble();
			}
			else
				temp<< "New KI value missing! KI not updated.";
						
			reply.addString( temp.str().c_str());
			return true;
		}

		case set_KD:
		{
			stringstream temp;
			if(command.size()==3){
				// the third parameter has to be a number
				if(!command.get(2).isDouble() && !command.get(2).isInt()){
					reply.addString("New KD value is not a number! KD not updated.");
					return true;
				}
				if(graspThread->setKD((float)command.get(2).asDouble()))
					temp<< "New KDs set: "<< command.get(2).asDouble();
				else
					temp<< "ERROR in setting new KD: "<< command.get(2).asDouble();
			}
			else if(command.size()>3){
				// the third parameter has to be a int and the fourth has to be a number
				if(!command.get(2).isInt() || (!command.get(3).isDouble() && !command.get(3).isInt())){
					reply.addString("Index is not as int or new KD value is not a number! KD not updated.");
					return true;
				}
				if( graspThread->setKD(command.get(2).asInt(), (float)command.get(3).asDouble()) )
					temp<< "KD of finger "<< command.get(2).asInt()<< " set: "<< command.get(3).asDouble();
				else
					temp<< "ERROR in setting new KD: "<< command.get(2).asDouble();
			}
			else
				temp<< "New KD value missing! KD not updated.";
						
			reply.addString( temp.str().c_str());
			return true;
		}

		case open_hand:
			graspThread->setGraspCommand(graspThread->openHand);
			break;

		case stop:
			graspThread->setGraspCommand(graspThread->openHand);	// the stop command and the open hand command do the same thing
			break;

		case control:
			graspThread->setGraspCommand(graspThread->control);
			break;

		case tough_grasp:
			graspThread->setGraspCommand(graspThread->tough);
			break;

		case soft_grasp:
			graspThread->setGraspCommand(graspThread->soft);
			break;

		case compliant_grasp:
			graspThread->setGraspCommand(graspThread->compliant);
			break;		

		default:
			reply.addString("ERROR: This command is known but it is not managed in the code.");
			return true;
	}

	reply.addString( (COMMAND_LIST[com]+" command received.").c_str());

	return true;
}

/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool TactileGrasp::identifyCommand(Bottle commandBot, TactileGrasp::TactileGraspCommand &com){
	for(unsigned int i=0; i<COMMANDS_COUNT; i++){
		stringstream stream(COMMAND_LIST[i]);
		string word;
		int j=0;
		bool found = true;

		while(stream>>word){
			if (commandBot.get(j).asString() != word.c_str()){
				found=false;
				break;
			}
			j++;
		}
		if(found){
			com = (TactileGraspCommand)i;
			return true;
		}
	}

	return false;
}

bool TactileGrasp::updateModule(){ return true;}

double TactileGrasp::getPeriod(){ return 0.1;}
