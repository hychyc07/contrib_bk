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

#include "YarpVectorBagModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  YarpVectorBagModule module;

  return module.runModule(argc,argv);
}


YarpVectorBagModule::YarpVectorBagModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
YarpVectorBagModule::~YarpVectorBagModule(){
    close();
}

bool YarpVectorBagModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","YarpVectorBag000");
        fprintf(stderr, "No module base name specifed, using <YarpVectorBag000> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.01);
        fprintf(stderr, "No module period specifed, using <0.01> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.01)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.01> ms as default\n",mPeriod);
        mPeriod = 0.01;
    }

    bool bErr = false;

    int mode=0;
    if((mParams.check("rec"))||(mParams.check("play"))||(mParams.check("loop"))){
        if(mParams.check("rec"))
            mode = 0;
        else if(mParams.check("play"))
            mode = 1;    
        else
            mode = 2;
    }else{
        fprintf(stderr, "Error: Rec, play or loop mode must be specified (e.g. --rec or --play --loop)\n");
        bErr = true;
    }
    if(!mParams.check("bag")){
        fprintf(stderr, "Error: No bag file specified (e.g. --bag name.bag)\n");
        bErr = true;
    }
    char bagPath[256];
    snprintf(bagPath,255,"%s",mParams.find("bag").asString().c_str());
    if(strlen(bagPath)==0){
        fprintf(stderr, "Error: bag file is NULL\n");
        bErr = true;
    }
    bool bPortParamVerb = true;
    bool bNewPortParamVerb = false;
    if(mode==0){
        if(!mParams.check("ports")){
            fprintf(stderr, "Error: No ports specified (e.g. --ports \"/portA /portB\")\n");
            bErr = true;
        }
    }else{
        if(!mParams.check("ports")){
            fprintf(stderr, "Info: No ports specified: will replay all found in the bag\n");
            bPortParamVerb = false;
        }
        if(!mParams.check("renames")){
            fprintf(stderr, "Info: No renames ports specified: but that's optional\n");
        }else{
            bNewPortParamVerb = true;
        }
    }
    std::vector<std::string> bagPorts;
    std::vector<std::string> newBagPorts;
    std::string portList = mParams.find("ports").asString().c_str();
    if(portList.length()==0){
        if(bPortParamVerb){
            fprintf(stderr, "Error: Port list specified is empty\n");
            bErr = true;
        }
    }
    bool pausing = false;
    if(mParams.check("pause")){
        pausing = true;
    }else{
        fprintf(stderr, "Info: To pause the bag before start, use param --pause\n");            
        fprintf(stderr, "      Then use command cont/run/stop\n");            
        fprintf(stderr, "      Commands cont/stop only in play mode\n");            
    }    

    if(bErr)
        return false;

    if(bPortParamVerb){
        size_t endPos   = 0;
        size_t startPos = 0;
        while(endPos!=string::npos){
            endPos = portList.find(" ",startPos);
            if(endPos!=string::npos){
                if(endPos-startPos>=1)
                    bagPorts.push_back(portList.substr(startPos,endPos-startPos));
                startPos = endPos+1;
            }
        }
        if(startPos<(int)portList.size())
            bagPorts.push_back(portList.substr(startPos));

        if(bagPorts.size()==0){
            fprintf(stderr, "Error: Cannot get list of ports from the list\n");
            bErr = true;
        }
    }

    if(bErr)
        return false;

    std::string newPortList = mParams.find("renames").asString().c_str();
    if(newPortList.length()==0){
        if(bNewPortParamVerb){
            fprintf(stderr, "Error: Remanes Port list specified is empty\n");
            bErr = true;
        }
    }
    if(bNewPortParamVerb){
        size_t endPos   = 0;
        size_t startPos = 0;
        while(endPos!=string::npos){
            endPos = newPortList.find(" ",startPos);
            if(endPos!=string::npos){
                if(endPos-startPos>=1)
                    newBagPorts.push_back(newPortList.substr(startPos,endPos-startPos));
                startPos = endPos+1;
            }
        }
        if(startPos<(int)newPortList.size())
            newBagPorts.push_back(newPortList.substr(startPos));

        if(newBagPorts.size()==0){
            fprintf(stderr, "Error: Cannot get list of ports from the list\n");
            bErr = true;
        }
    }

    if(bErr)
        return false;



    char portName[255];
    snprintf(portName,255,"/YarpVectorBag/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);

    snprintf(portName,255,"YarpVectorBag/%s",getName().c_str());
    
    mThread = new YarpVectorBagThread(int(floor(mPeriod*1000)),portName,bagPorts,newBagPorts,bagPath,mode);

    if(mThread->start()){
        if(!pausing){
            mThread->StartBag();
        }else{
            if(mode!=0){
                mThread->StartBag();
                mThread->PauseBag();
            }
        }
        bIsReady = true;
    }else{
        delete mThread;
    }
    return bIsReady;
}

bool YarpVectorBagModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool YarpVectorBagModule::respond(const Bottle& command, Bottle& reply) {
    int  cmdSize    = command.size();
    bool retVal     = true;
        
    if(cmdSize<=0){
        retVal      = Module::respond(command,reply);
    }else{
        switch(command.get(0).asVocab()) {
        case VOCAB3('r','u','n'):
            mThread->StartBag();
            reply.addVocab(Vocab::encode("ack"));
            retVal = true;
            break;
        case VOCAB4('s','t','o','p'):
            mThread->PauseBag();
            reply.addVocab(Vocab::encode("ack"));
            retVal = true;
            break;
        case VOCAB4('c','o','n','t'):
            mThread->ResumeBag();
            reply.addVocab(Vocab::encode("ack"));
            retVal = true;
            break;
        default:
            retVal      = Module::respond(command,reply);
            break;
        }
    }
    return retVal;
}


double YarpVectorBagModule::getPeriod(){
    return 2.0;
}

int YarpVectorBagModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   YarpVectorBagModule::updateModule() {
    return true;
}



