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

#include "YarpVectorMixerModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  YarpVectorMixerModule module;

  return module.runModule(argc,argv);
}


YarpVectorMixerModule::YarpVectorMixerModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
YarpVectorMixerModule::~YarpVectorMixerModule(){
    close();
}

bool YarpVectorMixerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","YarpVectorMixer000");
        fprintf(stderr, "No module base name specifed, using <YarpVectorMixer000> as default\n");
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

    
    
    char portName[255];

    snprintf(portName,255,"YarpVectorMixer/%s",getName().c_str());
    mThread = new YarpVectorMixerThread(int(floor(mPeriod*1000)),portName);
    if(mParams.check("time")){
        mThread->ShowTime(true);
    }else{
        fprintf(stderr, "Info: to display time use param --time \n");
        fprintf(stderr, " usage: --time\n");
    }
    
    if(mParams.check("auto")){
        mThread->AutoMode(true);
    }else{
        fprintf(stderr, "Info: to start streaming automatically, use param --auto\n");
        fprintf(stderr, " usage: --auto\n");
    }
    if(mParams.check("portNum")){
        int num = mParams.find("portNum").asInt();
        if((num>0)&&(num<=MAX_INPUT_PORTS)){
            mThread->SetPortNumber(num);
        }else{
            fprintf(stderr, "Error: the number of input port should be between 1 and %d\n",MAX_INPUT_PORTS);
            fprintf(stderr, " usage: --portNum 2\n");        
            return false;
        }
    }else{
        fprintf(stderr, "Info: to fix the number of input port in advance, use param --portNum [int]\n");
        fprintf(stderr, " usage: --portNum 2\n");
    }

    

    snprintf(portName,255,"/YarpVectorMixer/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);
    
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool YarpVectorMixerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool YarpVectorMixerModule::respond(const Bottle& command, Bottle& reply) {
    int  cmdSize    = command.size();
    bool retVal     = true;
    
    
    if(cmdSize<=0){
        retVal = true;
        mThread->StartStop();
        reply.addVocab(Vocab::encode("ack"));
    }else{
        retVal      = Module::respond(command,reply);
    }
    return retVal;
}


double YarpVectorMixerModule::getPeriod(){
    return 2.0;
}

int YarpVectorMixerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   YarpVectorMixerModule::updateModule() {
    return true;
}



