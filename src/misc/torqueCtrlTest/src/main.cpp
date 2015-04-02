/**
@ingroup icub_module
\defgroup torqueCtrlTest torqueCtrlTest
Test different kind of low level torque control using the open loop control interface to set the motor PWM.
Copyright (C) 2008 RobotCub Consortium
Author: Andrea Del Prete
Date: first release 08/2011
CopyPolicy: Released under the terms of the GNU GPL v2.0.

\author Andrea Del Prete
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/common.h>

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "iCub/torqueCtrlTest/controlConstants.h"
#include "iCub/torqueCtrlTest/controlThread.h"

YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{

namespace torqueCtrlTest
{

class controlModule: public RFModule
{
private:
    
	int period;
	controlThread *contrThread;	
    BodyPart bodyPart;    
    Port rpcPort;        	

public:
    controlModule()
    {
		bodyPart            = BODY_PART_UNKNOWN;
        contrThread         = 0;
        period              = 10;
    }
   
    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        //-----------------GET THE MODULE NAME-------------------//
        string name = "torqueCtrlTest";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());
        
        //-----------------GET THE PERIOD-------------------//
        int period = 10;
        if (rf.check("period"))
            period = rf.find("period").asInt();

		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

		//------------------CHECK WHICH ARM IS ENABLED-----------//
		if (rf.check("left_arm")){
			bodyPart = LEFT_ARM;
			fprintf(stderr,"'left_arm' option found. Left arm will be enabled.\n");
		}
        else if (rf.check("right_arm")){
			bodyPart = RIGHT_ARM;
			fprintf(stderr,"'right_arm' option found. Right arm will be enabled.\n");
		}
        else{
            bodyPart = LEFT_ARM;
            fprintf(stderr, "No arm specified. Left arm will be used by default.\n");
        }

        //---------------------RPC PORT--------------------------//
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);


        //--------------------------THREAD--------------------------
        contrThread = new controlThread(name, robot_name, period, bodyPart, VERBOSE);
        fprintf(stderr,"control thread istantiated...\n");
        contrThread->start();
        fprintf(stderr,"control thread started\n");
        return true;
    }


    bool respond(const Bottle& command, Bottle& reply) 
    {
	    stringstream temp;
	    string helpMessage =  string(getName().c_str()) + " commands are: ";
	    reply.clear();

	    TorqueCtrlTestCommand com;
        Bottle param;
	    if(!identifyCommand(command, com, param)){
		    reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		    return true;
	    }

	    switch( com ){
		    case quit:          reply.addString("quitting");    return false;
		    case help:
                reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
                reply.addString(helpMessage.c_str());
			    for(unsigned int i=0; i< SFC_COMMAND_COUNT; i++){
				    reply.addString( ("- "+TorqueCtrlTestCommand_s[i]+": "+TorqueCtrlTestCommand_desc[i]).c_str() );
			    }
			    return true;

            case no_ctrl:       contrThread->setCtrlMode(NO_CONTROL);           break;
            case set_ctrl:
                {
                    if(param.size()<1 || !(param.get(0).isInt() || param.get(0).isDouble())){
                        reply.addString("Error: control law number missing or not an int.");
                        return true;
                    }
                    int cl = param.get(0).asInt();
                    if(cl<0 || cl>=CONTROL_MODE_SIZE){
                        reply.addString("Error: control law number out of range.");
                        return true;
                    }
                    contrThread->setCtrlMode((ControlMode)cl);
                    reply.addString(("Control law "+ControlMode_desc[cl]+"set.").c_str());
                    return true;
                }
            case get_ctrl:
                {
                    int cl = (int)contrThread->getCtrlMode();
                    reply.addInt(cl);
                    reply.addString(ControlMode_desc[cl].c_str());
                    return true;
                }

            case get_kp:
                    reply.addDouble(contrThread->getKp());
                    return true;
            case get_kd:
                    reply.addDouble(contrThread->getKd());
                    return true;
            case get_ki:
                    reply.addDouble(contrThread->getKi());
                    return true;
            case get_ktao:
                    reply.addDouble(contrThread->getKtao());
                    return true;
            case get_kbemf:
                    reply.addDouble(contrThread->getKbemf());
                    return true;
            case get_kstic:
                    reply.addDouble(contrThread->getKstic());
                    reply.addDouble(contrThread->getKcoulomb());
                    return true;
            case get_kcp:
                    reply.addDouble(contrThread->getKcp());
                    return true;
            case get_kcn:
                    reply.addDouble(contrThread->getKcn());
                    return true;
            case get_kdither:
                    reply.addDouble(contrThread->getKdither());
                    return true;
            case get_wdither:
                    reply.addDouble(contrThread->getWdither());
                    return true;
            case get_est_thr:
                    reply.addDouble(contrThread->getEstThr());
                    return true;
            case get_est_wind:
                    reply.addDouble(contrThread->getEstWind());
                    return true;
            case get_taod:
                {
                    Vector taod = contrThread->getTaod();
                    reply.addString(taod.toString(3).c_str());
                    return true;
                }
            case get_tao:
                {
                    Vector tao = contrThread->getTao();
                    reply.addString(tao.toString(3).c_str());
                    return true;
                }           
            case get_alpha:
                reply.addDouble(contrThread->getAlpha());
                reply.addDouble(contrThread->getAlphaStic());
                return true;
            case get_joint:
                reply.addInt(contrThread->getJoint());
                return true;
            case get_pwm:
                reply.addInt((int)contrThread->getPwmD());
                return true;
            case get_qd:
                reply.addDouble(contrThread->getQd());
                return true;
            case get_traj_time:
                reply.addDouble(contrThread->getTrajTime());
                return true;
            case get_body_part:
                reply.addInt(contrThread->getBodyPart());
                reply.addString(BodyPart_s[contrThread->getBodyPart()].c_str());
                return true;
            case get_impedance:
                reply.addString("Desired inertia");
                reply.addDouble(contrThread->getMd());
                reply.addString("Real inertia");
                reply.addDouble(contrThread->getM());
                reply.addString("Damping");
                reply.addDouble(contrThread->getB());
                reply.addString("Stiffness");
                reply.addDouble(contrThread->getK());
                return true;

            case set_kp:
                {
                    try{
                        contrThread->setKp(param.get(0).asDouble());
                        reply.addString("kp set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kp failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_kd:
                {
                    try{
                        contrThread->setKd(param.get(0).asDouble());
                        reply.addString("kd set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kd failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_ki:
                {
                    try{
                        contrThread->setKi(param.get(0).asDouble());
                        reply.addString("ki set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set ki failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_ktao:
                {
                    try{
                        contrThread->setKtao(param.get(0).asDouble());
                        reply.addString("ktao set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set ktao failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }  
            case set_kbemf:
                {
                    try{
                        contrThread->setKbemf(param.get(0).asDouble());
                        reply.addString("kbemf set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kbemf failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                } 
            case set_kstic:
                {
                    try{
                        contrThread->setKstic(param.get(0).asDouble());
                        if(param.size()>1)
                            contrThread->setKcoulomb(param.get(1).asDouble());
                        reply.addString("kstic set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kstic failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                } 
            case set_kcp:
                {
                    try{
                        contrThread->setKcp(param.get(0).asDouble());
                        reply.addString("kcp set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kcp failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_kcn:
                {
                    try{
                        contrThread->setKcn(param.get(0).asDouble());
                        reply.addString("kcn set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kcn failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_kdither:
                {
                    try{
                        contrThread->setKdither(param.get(0).asDouble());
                        reply.addString("kdither set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kdither failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_wdither:
                {
                    try{
                        contrThread->setWdither(param.get(0).asDouble());
                        reply.addString("wdith set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set wdith failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_est_wind:
                {
                    try{
                        contrThread->setEstWind(param.get(0).asInt());
                        reply.addString("est wind set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set est wind failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                } 
            case set_est_thr:
                {
                    try{
                        contrThread->setEstThr(param.get(0).asDouble());
                        reply.addString("est thr set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set est thr failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                } 
            case set_taod:
                {
                    try{
                        if(param.size()>1){
                            contrThread->setTaod(bottle2vector(param));
                        }else{
                            contrThread->setTaod(param.get(0).asDouble());
                        }
                        reply.addString("taod set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set taod failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }           
            case set_alpha:
                {
                    try{
                        contrThread->setAlpha(param.get(0).asDouble());
                        if(param.size()>1)
                            contrThread->setAlphaStic(param.get(1).asDouble());
                        reply.addString("alpha set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set alpha failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }    
            case set_joint:
                try{
                    contrThread->setJoint(param.get(0).asInt());
                    reply.addString("joint set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set joint failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_pwm:
                try{
                    contrThread->setPwmD(param.get(0).asDouble());
                    reply.addString("pwm set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set pwm failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_qd:
                try{
                    contrThread->setQd(param.get(0).asDouble());
                    reply.addString("qd set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set qd failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_traj_time:
                try{
                    contrThread->setTrajTime(param.get(0).asDouble());
                    reply.addString("traj time set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set traj time failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_body_part:
                try{
                    contrThread->setBodyPart((BodyPart)param.get(0).asInt());
                    reply.addString("body part set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set body part failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_impedance:
                try{
                    if(param.size()==2)
                    {
                        if(param.get(0).asInt()==0) contrThread->setMd(param.get(1).asDouble());
                        else if(param.get(0).asInt()==1) contrThread->setM(param.get(1).asDouble());
                        else if(param.get(0).asInt()==2) contrThread->setB(param.get(1).asDouble());
                        else if(param.get(0).asInt()==3) contrThread->setK(param.get(1).asDouble());
                        if(param.get(0).asInt()>3 || param.get(0).asInt()<0)
                            reply.addString("Error: first parameter has to be an int in [0, 3].");
                        else
                            reply.addString("Impedance value set.");
                    }
                    else if(param.size()==4)
                    {
                        contrThread->setMd(param.get(0).asDouble());
                        contrThread->setM(param.get(1).asDouble());
                        contrThread->setB(param.get(2).asDouble());
                        contrThread->setK(param.get(3).asDouble());
                        reply.addString("Impedance parameters set.");
                    }
                    else
                        reply.addString("Error: this method accepts either 2 or 4 parameters.");
                    return true;
                }catch(runtime_error &e){
                    reply.addString("set impedance failed: ");
                    reply.addString(e.what());
                }

            case reset_pid:
                {
                    contrThread->resetTorquePid();
                    break;
                }

            case sim_on:    contrThread->setSimMode(true);  break;
            case sim_off:   contrThread->setSimMode(false); break;        
		    default: reply.addString("ERROR: This command is known but it is not managed in the code."); return true;
	    }

	    reply.addString( (TorqueCtrlTestCommand_s[com]+" command received.").c_str());
	    return true;	
    }


    /**
      * Identify the command in the bottle and return the correspondent enum value.
      * All the elements of the Bottle that are after the identified command are inserted 
      * in the param bottle.
      */
    bool identifyCommand(const Bottle &commandBot, TorqueCtrlTestCommand &com, Bottle &param){
	    for(unsigned int i=0; i<SFC_COMMAND_COUNT; i++){
		    stringstream stream(TorqueCtrlTestCommand_s[i]);
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
			    com = (TorqueCtrlTestCommand)i;
                for(int k=wordCounter; k<commandBot.size(); k++)
                    param.add(commandBot.get(k));
			    return true;
		    }
	    }

	    return false;
    }


    bool close(){
		//stop thread 
		if(contrThread){
            contrThread->stop();
            delete contrThread;
            contrThread = 0;
        }
				
		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        return true;
    }

    double getPeriod()  { return 0.3;  }
    bool updateModule()
	{
        if (contrThread==0) 
            return false;

        double avgTime, stdDev, period;
        period = contrThread->getRate();
        //contrThread->getEstUsed(avgTime, stdDev);     // real duration of run()
        contrThread->getEstPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
            printf("WARNING: Control loop is too slow (real period: %3.3f+/-%3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }
        
        ControlThreadStatus thread_status = contrThread->getThreadStatus();

		if (thread_status==STATUS_OK)
            return true;
		else if (thread_status==STATUS_DISCONNECTED){
			printf ("torqueCtrlTest module lost connection with iCubInterface or wholeBodyDynamics, now closing...\n");
			return false;
		}else{
			fprintf(stderr,"torqueCtrlTest module was closed successfully! \n");
			return true;
		}        
	}   
    Vector bottle2vector(const Bottle &b) {
        Vector v(b.size());
        for(int i=0; i<b.size();i++){
            if(!b.get(i).isInt() && !b.get(i).isDouble()){
                throw runtime_error("One of the values is not a number");
            }
            v(i) = b.get(i).asDouble();
        }
        return v;
    }
};

}
} // end namespace

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                        << endl;
        cout << "\t--name       name: the name of the module used for the port names. default torqueCtrlTest"	                  << endl;
        cout << "\t--robot      robot: the name of the robot. default icub"	                                					  << endl;
        cout << "\t--period     period: the period used by the module. default 10ms (not less than 10ms)"						  << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    iCub::torqueCtrlTest::controlModule module;
    module.runModule(rf);

    //cin.get();
    return 0;
}

