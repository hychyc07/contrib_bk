// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

#include "HandSkinControllerModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>
#include <string.h>

#include "MathLib/MathLib.h"
#include "MathLib/GMR2.h"

#define FINGERS_COUNT 3
int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  HandSkinControllerModule module;

  return module.runModule(argc,argv);
}


HandSkinControllerModule::HandSkinControllerModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
HandSkinControllerModule::~HandSkinControllerModule(){
    close();
}

bool HandSkinControllerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    // parsing of the papameters

    if(!mParams.check("name")){
        mParams.put("name","GraspAdaptation");
        fprintf(stderr, "No module base name specifed, using <GraspAdaptation> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.02);
        fprintf(stderr, "No module period specifed, using <0.02> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.02)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.02> ms as default\n",mPeriod);
        mPeriod = 0.02;
    }

    if(!mParams.check("robot")){
        mParams.put("robot","icubSim");
        fprintf(stderr, "No robot name specifed, using <icubSim> as default\n");
        fprintf(stderr, "  usage: --robot name (e.g. icub)\n");
    }
    strcpy(mRobotName,mParams.find("robot").asString().c_str());


    char basePath[256];
    if(!mParams.check("path")){
        fprintf(stderr, "Warning: no base path provided (parameter --path <string>): using <./data/GraspAdaptation> as default\n");
        strcpy(basePath, "./data/GraspAdaptation");
    }else{
        strcpy(basePath,mParams.find("path").toString().c_str());
        fprintf(stderr, "Using: base path <%s>\n",basePath);
    }



    mDriver = NULL;
    

    // loading the part driver
    if(!mParams.check("part")){
        fprintf(stderr, "No parts specifed");
        fprintf(stderr, "  Please choose and check at least one:\n");
        fprintf(stderr, "    e.g. --part (right or left) \n");
        return false;    
    }else{
        char part[512];
        strcpy(part,mParams.find("part").asString().c_str());
        if((strcmp(part,"left")!=0)&&(strcmp(part,"right")!=0)){
            fprintf(stderr, "Bad parts specifed (%s)\n",part);
            fprintf(stderr, "  Please choose and check at least one:\n");
            fprintf(stderr, "    e.g. --part (right or left) \n");
            return false;    
        }else{
            char partName[512];
            char txtBuffer[512];
            sprintf(partName,"%s_arm",part);
            Property options("");
            options.put("device","remote_controlboard");
            sprintf(txtBuffer,"/%s/%s",mRobotName,partName);
            options.put("remote",txtBuffer);
            sprintf(txtBuffer,"/%s/%s",getName().c_str(),partName);
            options.put("local",txtBuffer);    
            PolyDriver *driver = new PolyDriver(options);
            if(!driver->isValid()){
                driver->close();
                delete driver;
                driver = NULL;                
            }
            mDriver = driver;
            if(mDriver==NULL){
                fprintf(stderr, "Unable to open part %s\n",partName);
                fprintf(stderr, "  Please check it's up and running\n");
                return false;
            }

            // Setting up a port
            char portName[255];
            snprintf(portName,255,"/HandSkinController/%s/rpc",part);
            mControlPort.open(portName);
            attach(mControlPort,true);
            
            // Creating the thread
            snprintf(portName,255,"HandSkinController/%s",part);
            mThread = new HandSkinControllerThread(int(floor(mPeriod*1000)),portName,part);
            mThread->SetDriver(mDriver);
            mThread->SetBasePath(basePath);
            mThread->start();

            cerr << endl;
            cerr << "Module running:"<<endl;
            cerr << "  Type 'help' to get a command list"<<endl;

            // and BAM!

            bIsReady = true;
            return bIsReady;
        }
    }
}

bool HandSkinControllerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    if(mDriver)
        delete mDriver;
    
    bIsReady = false;
    return true;
}

bool HandSkinControllerModule::respond(const Bottle& command, Bottle& reply) {
    int  index      = 0;
    int  cmdSize    = command.size();
    bool retVal     = true;
    
    // system level commands: see help section below    
    if(cmdSize<=0){
        mThread->SwitchSendOutputData();
    }else{        
        switch(command.get(0).asVocab()) {
        case VOCAB3('r','u','n'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_RUN);
            break;        
        case VOCAB3('r','e','c'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_REC);
            break;        
        case VOCAB4('r','e','c','r'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_RECRUN);
            break;        
        case VOCAB3('r','e','p'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_REPLAY);
            break;        
        case VOCAB4('r','e','p','s'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_REPLAYSIM);
            break;                    
        case VOCAB4('i','d','l','e'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_IDLE);
            break;        
        case VOCAB4('r','e','s','t'):
            mThread->SetCtrlMode(HandSkinControllerThread::CM_REST);
            break;        
        case VOCAB3('d','o','n'):
            mThread->bDebugMode = true;
            break;        
        case VOCAB4('d','o','f','f'):
            mThread->bDebugMode = false;
            break;        
        case VOCAB4('a','s','a','v'):
            mThread->bAutoSaveTrainingData = !mThread->bAutoSaveTrainingData;
            if(mThread->bAutoSaveTrainingData){
                cout << "Autosave mode on"<<endl;
            }else{
                cout << "Autosave mode off"<<endl;
            }
            break;        
        case VOCAB4('s','c','a','l'):
            mThread->ResetSkin();
            break;
        case VOCAB4('l','g','m','m'):
            if(command.size()>1)
                mThread->LoadGMM(command.get(1).asString().c_str());
            break;
        case VOCAB4('l','r','e','p'):
            if(command.size()>1)
                mThread->LoadReplay(command.get(1).asString().c_str());
            break;
        case  VOCAB4('d','n','a','m'):
            if(command.size()>1)
                strcpy(mThread->mTrainingDataName,command.get(1).asString().c_str());
            break;
        case VOCAB3('g','o','n'):
            mThread->bGradientAscent = true;
            break;        
        case VOCAB4('g','o','f','f'):
            mThread->bGradientAscent = false;
            break;                               
        case VOCAB4('t','e','s','t'):
            mThread->bTestingOutput = !mThread->bTestingOutput;
            if(mThread->bTestingOutput) cout << "Testing mode on"<<endl;
            else cout << "Testing mode off"<<endl;
            break;                               
        case VOCAB4('f','b','a','d'):
            if(command.size()>1){
                mThread->bBadSkin = true;
                int id = command.get(1).asInt();
                mThread->mBadFingerId = TRUNC(id,0,2);
                cout << "Bad skin mode on: using finger "<<mThread->mBadFingerId<<endl;
            }else{
                mThread->bBadSkin = !mThread->bBadSkin;
                if(mThread->bBadSkin) cout << "Bad skin mode on"<<endl;
                else cout << "Bad skin off"<<endl;
            }
            break;                               
        case VOCAB4('m','i','s','m'):
            mThread->bNoMissingCheck = !mThread->bNoMissingCheck;
            if(mThread->bNoMissingCheck) cout << "Missing check off"<<endl;
            else cout << "Missing check on"<<endl;
            break;   
        case VOCAB4('g','a','i','n'):
            if(command.size()>1){
                double id = command.get(1).asDouble();
                mThread->mJointsPidGainGlobal = TRUNC(id,0.0,10.0);
                cout << "Setting global gain to: "<<mThread->mJointsPidGainGlobal<<endl;
            }else{                                        
                cout << "Error: Need a gain paramater"<<endl;
            }
            break;
        case VOCAB2('d','0'):
            mThread->LoadGMM("demo_init_rep");
            mThread->SetCtrlMode(HandSkinControllerThread::CM_RUN);
            break;                               
        case VOCAB2('d','1'):
            strcpy(mThread->mTrainingDataName,"rsdemo0");
            mThread->SetCtrlMode(HandSkinControllerThread::CM_RECRUN);
            break;                               
        case VOCAB2('d','2'):
            strcpy(mThread->mTrainingDataName,"rsdemo0_rep");
            mThread->LoadReplay("rsdemo0");
            mThread->SetCtrlMode(HandSkinControllerThread::CM_REPLAY);
            break;                               
        case VOCAB2('d','3'):
            mThread->LoadGMM("rsdemo0_rep");
            mThread->SetCtrlMode(HandSkinControllerThread::CM_RUN);
            break;                               
            
        case VOCAB4('h','e','l','p'):
            cerr << "**************************************"<<endl;
            cerr << "Command list:"<<endl;
            cerr << "**************************************"<<endl;
            cerr << "  idle   : Idle mode"<<endl;
            cerr << "           - No commands are sent"<<endl;
            cerr << "           - robotMotorGui can be used to set a pose"<<endl;            
            cerr << "  rest   : Go to rest position mode"<<endl;
            cerr << "  rec    : Record data mode "<<endl;
            cerr << "           - Keep the last position fron Idle mode"<<endl;
            cerr << "           - Use low gains"<<endl;            
            cerr << "  run    : Run the model"<<endl;
            cerr << "  recr   : Run and record data mode "<<endl;
            cerr << "           - Use the model to help manipulating"<<endl;
            cerr << "           - Use low gains"<<endl;            
            cerr << "**************************************"<<endl;
            cerr << "  <SPC>  : Switch data output on port"<<endl;
            cerr << "  rep    : Replay data"<<endl;
            cerr << "  reps   : Replay data and simulate skin input"<<endl;
            cerr << "  lrep <name>  : Load replay data"<<endl;
            cerr << "  lgmm <name>  : Load GMM data"<<endl;
            cerr << "  dnam <name>  : Use <name> for auto saving data when output streaming is on"<<endl;
            cerr << "  asav   : Toggle auto save during output streaming"<<endl;
            cerr << "**************************************"<<endl;
            cerr << "  test   : toggle test data output mode"<<endl;
            cerr << "  mism   : toggle reliability measure check"<<endl;
            cerr << "  gon    : turn on graient ascent mode"<<endl;
            cerr << "  goff   : turn off graient ascent mode"<<endl;
            cerr << "**************************************"<<endl;
            cerr << "  scal   : Reset skin calibration"<<endl;
            cerr << "**************************************"<<endl;
            cerr << "  don    : Debug message On"<<endl;
            cerr << "  doff   : Debug message Off"<<endl;
            cerr << "**************************************"<<endl;
            
            
            break;            
        default:
            return Module::respond(command,reply);
            break;
        }            
    }

    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }
    return retVal;
}


double HandSkinControllerModule::getPeriod(){
    return 2.0;
}

int HandSkinControllerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   HandSkinControllerModule::updateModule() {
    return true;
}



