#include "iCub/skinCalibration/CalibrationModule.h"

using namespace iCub::skinCalibration;

bool CalibrationModule::configure(ResourceFinder &rf){

		string fwdSlash = "/";                           
		double contact_threshold;
		double force_threshold;
		int max_taxels_in_contact;
		SkinPart skinPart;                          // id of the part of the skin (hand, forearm_lower, arm_internal, ...)
		BodyPart robotBodyPart;
		int skinLink;
		int FTLink;
		int buffsize;
		int buffnum;
		int algorithm;
		Matrix FTLink2FT;
		ct = '\0';
		csc = '\0';
		ce = '\0';

        //-----------------GET THE MODULE NAME-------------------//
        string name = "skinCalibration";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());

		SkinCalibrationLogger::setContextLabel("["+name+"] ");
		SkinCalibrationLogger::setStreamToStdOut();
        
        //-----------------GET THE PERIOD-------------------//
        int period = 10;
        if (rf.check("period"))
            period = rf.find("period").asInt();

		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

		//-----------------GET CONTACT THRESHOLD-------------------//
		contact_threshold = 10;
		if (rf.check("contactThreshold"))
			contact_threshold = rf.find("contactThreshold").asInt();

		//-----------------GET FORCE THRESHOLD-------------------//
		force_threshold = 3;
		if (rf.check("forceThreshold"))
			force_threshold = rf.find("forceThreshold").asInt();

		//-----------------GET MAX TAXELS IN CONTACT--------//
		max_taxels_in_contact = 6;
		if (rf.check("maxTaxelsInContact"))
			max_taxels_in_contact = rf.find("maxTaxelsInContact").asInt();
		
		//-----------------GET SKIN PART-------------------//
		skinPart = (SkinPart)0;
		if (rf.check("skinPart"))
			skinPart = (SkinPart) rf.find("skinPart").asInt();

		//-----------------GET BODY PART-------------------//
		robotBodyPart = (BodyPart)0;
		if (rf.check("bodyPart"))
			robotBodyPart = (BodyPart)rf.find("bodyPart").asInt();

		//-----------------GET SKIN LINK-------------------//
		skinLink = 0;
		if (rf.check("skinLink"))
			skinLink = rf.find("skinLink").asInt();

		//-----------------GET FT SENSOR LINK-------------------//
		FTLink = 0;
		if (rf.check("FTLink"))
			FTLink = rf.find("FTLink").asInt();

		//-----------------GET FT LINK TO FT SENSOR FRAME TRANSFORMATION-------------------//
		FTLink2FT.zero();
		if (rf.check("FTLink2FT")){
			Vector angaxis(4);
			angaxis.zero();
			Bottle *temp;
			temp = rf.find("FTLink2FT").asList();
			if(temp && temp->size() == 7){
				for(int i=3; i<7; i++){
					angaxis[i-3] = (temp->get(i).asDouble());
					SkinCalibrationLogger::writeMsg("FTLink2FT angaxis %i %f\n",i-3,temp->get(i).asDouble());
				}
				FTLink2FT = axis2dcm(angaxis,1);
				for(int i=0; i<3; i++)
					FTLink2FT(i,3) = temp->get(i).asDouble();
			}
			else{
				SkinCalibrationLogger::writeMsg("FTLink2FT wrong format...\n");
				return false;
			}
			SkinCalibrationLogger::writeMsg("FTLink2FT matrix %f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f;\n",FTLink2FT(0,0),FTLink2FT(0,1),FTLink2FT(0,2),FTLink2FT(0,3),FTLink2FT(1,0),FTLink2FT(1,1),FTLink2FT(1,2),FTLink2FT(1,3),FTLink2FT(2,0),FTLink2FT(2,1),FTLink2FT(2,2),FTLink2FT(2,3),FTLink2FT(3,0),FTLink2FT(3,1),FTLink2FT(3,2),FTLink2FT(3,3));
		}

		//-----------------GET BUFFER SIZE-------------------//
		buffsize = 100;
		if (rf.check("bufferSize"))
			buffsize =  rf.find("bufferSize").asInt();

		//-----------------GET BUFFER NUMBER-------------------//
		buffnum = 0;
		if (rf.check("bufferNumber"))
			buffnum = rf.find("bufferNumber").asInt();
		
		//-----------------GET ALGORITHM-------------------//
		algorithm = 0;
		if (rf.check("algorithm"))
			algorithm = rf.find("algorithm").asInt();

        //---------------------RPC PORT--------------------------//
        rpcPort.open("/skinCalibration/rpc");
        attach(rpcPort);


        //--------------------------THREAD--------------------------//

		ce = new CalibrationEngine(&rf,robotBodyPart,skinPart);
		SkinCalibrationLogger::writeMsg("Calibration engine thread istantiated.\n");
		SkinCalibrationLogger::writeMsg("Setting calibration algorithm number %i.\n",algorithm);
		if(!ce->changeCalibrationAlgorithm(algorithm)){
			SkinCalibrationLogger::writeMsg("Init calibration algorithm number %i failed, aborting!\n",algorithm);
			return false;
		}
		csc = new CalibrationSamplesCollector(ce,buffsize,buffnum);
        ct = new CalibrationThread(period,
								   name,
								   robot_name,
								   robotBodyPart,
								   skinPart,
								   skinLink,
								   FTLink,
								   FTLink2FT,
								   contact_threshold,
								   force_threshold,
								   max_taxels_in_contact,
								   csc);
		SkinCalibrationLogger::writeMsg("Control thread istantiated.\n");
        if(ce->start())
			SkinCalibrationLogger::writeMsg("Calibration engine started.\n");
		else{
			SkinCalibrationLogger::writeMsg("Calibration engine did not start, aborting!\n");
			return false;
		}
        if(ct->start())
			SkinCalibrationLogger::writeMsg("Control thread started.\n");
		else{
			SkinCalibrationLogger::writeMsg("Control thread did not start, aborting!\n");
			return false;
		}

        return true;
}

bool CalibrationModule::respond(const Bottle& command, Bottle& reply) 
{

	string temp;
	string helpMessage =  string(getName().c_str()) + " commands are: ";
	reply.clear();

	SkinCalibrationCtrlCommand com;
    Bottle param;
    if(command.get(0).isInt()){
        // if first value is int then it is the id of the command
        com = (SkinCalibrationCtrlCommand)command.get(0).asInt();
        param = command.tail();
    }
	else if(!identifyCommand(command, com, param)){
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	switch( com ){
		 case quit:          reply.addString("quitting");    return false;
		 case help:
			 {
				reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
				reply.addString(helpMessage.c_str());
				for(unsigned int i=0; i< SCC_COMMAND_COUNT; i++){
					reply.addString( ("- "+SkinCalibrationCtrlCommand_s[i]+": "+SkinCalibrationCtrlCommand_desc[i]).c_str() );
				}
				return true;
			 }
		 case get_algorithm_list:
			 {
				reply.addVocab(Vocab::encode("many"));
				for(int i=0; i<ce->getAlgorithmsNumber(); i++){
					stringstream algorithm;
					algorithm<<i<<". ";
					algorithm<<ce->algorithms[i]->getName().c_str();
					reply.addString(algorithm.str().c_str());
				}
				break;
			 }
		 case set_algorithm:
			 {
				 ct->suspend();
				 if(ce->changeCalibrationAlgorithm(param.get(0).asInt()))
					SkinCalibrationLogger::writeMsg("Calibration algorithm %i set.\n",param.get(0).asInt());
				 else
					SkinCalibrationLogger::writeMsg("Calibration algorithm %i not set.\n",param.get(0).asInt());
				 ct->resume();
				break;
			 }
		 case pause_calibration:
			 {
				SkinCalibrationLogger::writeMsg("Pausing the calibration process.\n");
				ct->suspend();
				break;
			 }
		 case start_calibration:
			 {
				SkinCalibrationLogger::writeMsg("Resuming the calibration process.\n");
				ct->resume();
				break;
			 }
		default:
			SkinCalibrationLogger::writeMsg("Command not recognized.\n");
	}
	reply.addString( (SkinCalibrationCtrlCommand_s[com]+" command received.").c_str());
	return true;	
}

bool CalibrationModule::close(){
	//stop thread 
	SkinCalibrationLogger::writeMsg("Stopping Calibration Thread...\n");
	if(ct){
		if(ct->isRunning())
			ct->stop();
        delete ct;
        ct = '\0';
    }
	SkinCalibrationLogger::writeMsg("...Done\n");
	SkinCalibrationLogger::writeMsg("Deleting CalibrationSamplesCollector...\n");	
	if(csc){
		delete csc;
		csc = '\0';
	}
	SkinCalibrationLogger::writeMsg("...Done\n");
	SkinCalibrationLogger::writeMsg("Stopping Calibration Engine...\n");
	if(ce){
		if(ce->isRunning())
			ce->stop();
		delete ce;
		ce = '\0';
	}
	SkinCalibrationLogger::writeMsg("...Done\n");
		

	SkinCalibrationLogger::writeMsg("FlyWeight Samples not correctly deallocated %li\n",CalibrationSample::getAllocated());
	SkinCalibrationLogger::writeMsg("HeavyWeight Samples not correctly deallocated %li\n",CalibrationSample::getHeavyWeight());
	//closing ports
    rpcPort.interrupt();
	rpcPort.close();
	SkinCalibrationLogger::writeMsg("Calibration Module terminated\n");
    return true;
 }

 bool CalibrationModule::identifyCommand(const Bottle &commandBot, SkinCalibrationCtrlCommand &com, Bottle &param){
	for(unsigned int i=0; i<SCC_COMMAND_COUNT; i++){
		stringstream stream(SkinCalibrationCtrlCommand_s[i]);
		string word;
		int wordCounter=0;
		bool found = true;

		while(stream>>word){
			if (commandBot.get(wordCounter).asString() != word.c_str()){
				found=false;
				break;
			}
			wordCounter++;
		}
		if(found){
			com = (SkinCalibrationCtrlCommand)i;
            for(int k=wordCounter; k<commandBot.size(); k++)
                param.add(commandBot.get(k));
			return true;
		}
	}

	return false;
}

bool CalibrationModule::updateModule(){ return true;}
