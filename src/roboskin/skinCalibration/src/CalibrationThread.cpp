#include "iCub/skinCalibration/CalibrationThread.h"

using namespace iCub::skinCalibration;
#include <sstream>

#define smooth 0.995

CalibrationThread::CalibrationThread(int period,
									string name, 
									string robotName, 
									BodyPart robotBodyPart, 
									SkinPart skinPart, 
									int skinLink, 
									int FTLink, 
									Matrix FTLink2FT, 
									double contact_threshold,
									double force_thrs, 
									int max_txs_in_contact,
									CalibrationSamplesCollector *calsempcoll):RateThread(period){
	csc = calsempcoll;
	//contacts.open("/contacts:o");
	this->name = name;
    this->robotName = robotName;
	this->robotBodyPart = robotBodyPart;
	this->skinPart = skinPart;
	this->FTLink = FTLink;
	this->skinLink = skinLink;
	this->FTLink2FT = FTLink2FT;
	this->contact_threshold = contact_threshold;
	this->force_threshold = force_thrs;
	this->max_taxels_in_contact = max_txs_in_contact;

	localFTPortName = "/"+name+"/FT:i";
	#ifdef SKINCALIBRATION_SIMULATION	
		FTPortName ="/"+robotName+"/"+BodyPart_s[robotBodyPart]+"/analog:o";
	#else
		FTPortName ="/wholeBodyDynamics/"+BodyPart_s[robotBodyPart]+"/ext_ft_sens:o";
	#endif

	localSkinPortName = "/"+name+"/Skin:i";
#ifdef SKINCALIBRATION_SIMULATION
//	if(robotBodyPart % 2) //workaround...
	SkinPortName = "/"+robotName+"/skin/"+SkinPart_s[skinPart].substr(5);
//	else
//		SkinPortName = "/"+robotName+"/skin/right_"+SkinPart_s[skinPart];
#else
	//if(robotBodyPart % 2) //workaround...
		SkinPortName = "/"+robotName+"/skin/"+SkinPart_s[skinPart].substr(5)+"_comp";
	//else
	//	SkinPortName = "/"+robotName+"/skin/right_"+SkinPart_s[skinPart]+"_comp";
#endif 
	localJointsPositionsPortName = "/"+name+"/JointsPositions:i";
#ifdef SKINCALIBRATION_SIMULATION
	JointsPositionsPortName = "/"+robotName+"/"+BodyPart_s[robotBodyPart]+"/state:o";
#else
    JointsPositionsPortName = "/"+robotName+"/"+BodyPart_s[robotBodyPart];
#endif
	infoPortName = "/"+name+"/info:o";         // output occasional data
}

bool CalibrationThread::threadInit(){
	SkinCalibrationLogger::writeMsg("[CalibrationThread] Init...\n");
	cycle_counter =0;
	// open the output ports for communicating with the gui
    if(!infoPort.open(infoPortName.c_str())){
		string msg = "[CalibrationThread] Unable to open port " + infoPortName +"\n";
        SkinCalibrationLogger::writeMsg(msg.c_str());
		return false;
	}
	skinvalue = 0;
	Property options;
#ifndef SKINCALIBRATION_SIMULATION
	FTDevice = new BufferedPort<Vector>();
	if(!FTDevice->open(localFTPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot open the local F\\T sensor port...aborting!\n");
	    return false;
	}
	if(!NetworkBase::connect(FTPortName.c_str(),localFTPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot connect to the F\\T sensor port...aborting!\n");
	    return false;
	}
#else
	startup = true;
	startup_counter = 0;
	skinDataComp.clear();
	FTDataComp.resize(6);
	FTDataComp.zero();
	baseline.resize(6);
	baseline.zero();
	FTDevice = new BufferedPort<Vector>();
	if(!FTDevice->open(localFTPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot open the local F\\T sensor port...aborting!\n");
	    return false;
	}
	if(!NetworkBase::connect(FTPortName.c_str(),localFTPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot connect to the F\\T sensor port...aborting!\n");
	    return false;
	}
#endif
    FTData.resize(6);
	skinDevice = new BufferedPort<Vector>();
	if(!skinDevice->open(localSkinPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot open the local Skin sensor port...aborting!\n");
	    return false;
	}
	
	SkinCalibrationLogger::writeMsg("[CalibrationThread] attempting to connect %s %s.\n",SkinPortName.c_str(),localSkinPortName.c_str());
	if(!NetworkBase::connect(SkinPortName.c_str(),localSkinPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot connect to the Skin sensor port %s...aborting!\n",SkinPortName.c_str());
	    return false;
	}

//#endif

#ifndef SKINCALIBRATION_SIMULATION
	options.put("device","remote_controlboard");
	options.put("local", localJointsPositionsPortName.c_str());
	options.put("remote",JointsPositionsPortName.c_str());
	jointsPositionsDevice = new PolyDriver(options);
	if (!jointsPositionsDevice->isValid()){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Joints Positions device not available\n");
		return false;
	}
	if (!jointsPositionsDevice->view(encoders)){
	    SkinCalibrationLogger::writeMsg("[CalibrationThread] Problems assigning an interface to the Joints Positions device\n");
	    return false;
    }
	int jnts = 0;
    encoders->getAxes(&jnts);
	if(jnts==0){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] The encoder has not any axis...aborting!!\n");
	    return false;
		//jnts = 8;
	}

	jointsPositionsData.resize(jnts);
#else
	jointsPositionsDevice = new BufferedPort<Bottle>();
	if(!jointsPositionsDevice->open(localJointsPositionsPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot open the local encoder port...aborting!\n");
	    return false;
	}
	if(!NetworkBase::connect(JointsPositionsPortName.c_str(),localJointsPositionsPortName.c_str())){
		SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot connect to the encoder port...aborting!\n");
	    return false;
	}

	tempFT = tempSkin = '\0';
    tempJoints = '\0';
#endif

	if(robotBodyPart == LEFT_ARM)
		limb = new iCubArm("left");
	else if(robotBodyPart == RIGHT_ARM)
		limb = new iCubArm("right");
	else if(robotBodyPart == LEFT_LEG)
		limb = new iCubLeg("left");
	else if(robotBodyPart == RIGHT_LEG)
		limb = new iCubLeg("right");
	else{
		SkinCalibrationLogger::writeMsg("[CalibrationThread] The skin on the specified robot body part cannot be calibrated with the current implementation\n");
	    return false;
	}
	limb->releaseLink(0);
	limb->releaseLink(1);
	limb->releaseLink(2);
	SkinCalibrationLogger::writeMsg("[CalibrationThread] ...Done.\n");
	return true;
}


void CalibrationThread::threadRelease(){

	SkinCalibrationLogger::writeMsg("[CalibrationThread] Terminating.\n");
	
	if(limb){
		delete limb;
		limb = '\0';
	}

	SkinCalibrationLogger::writeMsg("[CalibrationThread] Closing FTDevice...\n");
	if(FTDevice){
		FTDevice->interrupt();
		FTDevice->close();
		delete FTDevice;
		FTDevice = '\0';
	}
	SkinCalibrationLogger::writeMsg("[CalibrationThread] ...Done.\n");

	SkinCalibrationLogger::writeMsg("[CalibrationThread] Closing skinDevice...\n");
	if(skinDevice){
		skinDevice->interrupt();
		skinDevice->close();
		delete skinDevice;
		skinDevice = '\0';
	}
	SkinCalibrationLogger::writeMsg("[CalibrationThread] ...Done.\n");

	SkinCalibrationLogger::writeMsg("[CalibrationThread] Closing jointsPositionsDevice...\n");
	if(jointsPositionsDevice){
		#ifdef SKINCALIBRATION_SIMULATION
		jointsPositionsDevice->interrupt();
		#endif
		jointsPositionsDevice->close();
		delete jointsPositionsDevice;
		jointsPositionsDevice = '\0';
	}
	if(encoders){
		encoders = '\0';
	}
	SkinCalibrationLogger::writeMsg("[CalibrationThread] ...Done!\n");
	
	SkinCalibrationLogger::writeMsg("[CalibrationThread] cycles %li\n",cycle_counter);
	infoPort.interrupt();
	infoPort.close();
}

void CalibrationThread::run(){
	stringstream str;
	Matrix Ts_i;
	Matrix R_ws,R_w;
	Vector Force,Moment,o_ws,o_w;
	Force.resize(3);
	Moment.resize(3);
	map<long int,double> taxels_measures;
	#ifndef SKINCALIBRATION_SIMULATION

	    if(!tempFT || FTDevice->getPendingReads())
	        tempFT = FTDevice->read(false);
    	if(!tempSkin || skinDevice->getPendingReads())
	    	tempSkin = skinDevice->read(false);
		if(!tempSkin || !tempFT)
		    return;
	    encoders->getEncoders(jointsPositionsData.data());
        cycle_counter++;
        for(unsigned int i=0; i<tempFT->size(); i++)
			FTData[i] = -(*tempFT)(i);
        
	#else
		if(!tempFT || FTDevice->getPendingReads())
			tempFT = FTDevice->read(false);
		if(!tempSkin || skinDevice->getPendingReads())
			tempSkin = skinDevice->read(false);
		if(!tempJoints || jointsPositionsDevice->getPendingReads())
			tempJoints =  jointsPositionsDevice->read(false);
		
		
		if(!tempSkin || !tempFT || !tempJoints)
			return;
		cycle_counter++;

		if(skinDataComp.size() == 0){
			skinDataComp.resize(tempSkin->size());
			skinDataComp.zero();
		}

		if(startup)
			for(unsigned int i=0; i<tempSkin->size(); i++)
				skinDataComp[i] +=(*tempSkin)(i); 
			
		for(unsigned int i=0; i<tempFT->size(); i++){
			FTData[i] = (*tempFT)(i);
			if(startup)
				FTDataComp[i] += (*tempFT)(i);
		}
	
		if(startup)
			startup_counter++;
		if(startup && startup_counter>=100){
			SkinCalibrationLogger::writeMsg("[CalibrationThread] Startup phase finished!\n");
			startup = false;
			FTDataComp = FTDataComp / (float)(startup_counter);
			skinDataComp = skinDataComp / (float)(startup_counter);
			SkinCalibrationLogger::writeMsg("[CalibrationThread] Force comp: %f %f %f %f %f %f\n",FTDataComp(0),FTDataComp(1),FTDataComp(2),FTDataComp(3),FTDataComp(4),FTDataComp(5));
		}
		
		if(!startup){
	#endif
			#ifdef SKINCALIBRATION_SIMULATION
				for(int i=0;i< tempJoints->size();i++)
					limb->setAng(i,CTRL_DEG2RAD*tempJoints->get(i).asDouble());
			#else
				for(unsigned int i=0;i< jointsPositionsData.size();i++)
					limb->setAng(i+TORSO_DOF,CTRL_DEG2RAD*jointsPositionsData[i]);
			#endif

			Ts_i = SE3inv(limb->getH(skinLink,true))*limb->getH(FTLink,true)*FTLink2FT; 
			Ts_i = SE3inv(Ts_i);
			R_ws = Ts_i.submatrix(0,2,0,2); //Rotation Matrix from wrist to sensor reference frame
			R_w  = limb->getH(skinLink,true).submatrix(0,2,0,2);
			o_ws = Ts_i.getCol(3).subVector(0,2); //Translation from wrist to sensor reference frame
			o_w  = limb->getH(skinLink,true).getCol(3).subVector(0,2);


			Force = FTData.subVector(0,2);			
			#ifdef SKINCALIBRATION_SIMULATION
				Force = Force - FTDataComp.subVector(0,2);
			#endif
			Moment = FTData.subVector(3,5);
			#ifdef SKINCALIBRATION_SIMULATION
				Moment = Moment - FTDataComp.subVector(3,5);
			#endif
				
			#ifndef SKINCALIBRATION_SIMULATION
			for(unsigned int i=0; i<tempSkin->size(); i++){
				skinvalue = (*tempSkin)(i);
			#else
			for(unsigned int i=0; i<tempSkin->size(); i++){
				skinvalue = (int)(skinDataComp(i)- (*tempSkin)(i));
			#endif
				if(skinvalue>=contact_threshold){
					taxels_measures[i] = skinvalue;
				}
			}
			if(taxels_measures.size() != 0 && taxels_measures.size() <= (unsigned int)max_taxels_in_contact  && norm(Force)>=force_threshold){
				if(!csc->push(Force,Moment,taxels_measures,R_ws,o_ws,R_w,o_w))
					SkinCalibrationLogger::writeMsg("[CalibrationThread] Cannot submit a sample!\n");
			}
			tempFT = tempSkin = '\0';
	#ifdef SKINCALIBRATION_SIMULATION
			tempFT = tempSkin = '\0';
			tempJoints = '\0';
		}
	#endif
}


void CalibrationThread::stop(){
	#ifdef SKINCALIBRATION_SIMULATION
		if(skinDevice)
			skinDevice->interrupt();
		if(FTDevice)
			FTDevice->interrupt();
		if(jointsPositionsDevice)
			jointsPositionsDevice->interrupt();
	#endif
	RateThread::stop();
}
