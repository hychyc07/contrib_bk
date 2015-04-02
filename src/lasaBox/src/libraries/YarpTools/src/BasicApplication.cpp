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

#include "BasicApplication.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>
#include <string.h>



BasicApplication::BasicApplication(const char* name, BasicApplicationThread *basicAppplicationThreadPtr){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = basicAppplicationThreadPtr;
    if((name!=NULL)&&(name[0]!=0))
        strncpy(mAppName,name,256);
    else
        strncpy(mAppName,"DefaultApplication",256);
}
BasicApplication::~BasicApplication(){
    close();
}

bool BasicApplication::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","");
        fprintf(stderr, "No module base name specifed, using <> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.1);
        fprintf(stderr, "No module period specifed, using <0.1> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.1)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <1.0> ms as default\n",mPeriod);
        mPeriod = 1.0;
    }

    if(mThread == NULL){
        fprintf(stderr, "Error: Provided thread pointer is a null pointer\n",mPeriod);
        return false;
    }
        

    char portName[255];
    if(strlen(getName().c_str())==0)
        snprintf(portName,255,"/%s/rpc",mAppName);
    else
        snprintf(portName,255,"/%s/%s/rpc",mAppName,getName().c_str());
    mControlPort.open(portName);
    mControlPort.setStrict();
    attach(mControlPort,true);
    
    if(strlen(getName().c_str())==0)
        snprintf(portName,255,"%s",mAppName);
    else
        snprintf(portName,255,"%s/%s",mAppName,getName().c_str());
        
    mThread->setRate(int(floor(mPeriod*1000)));
    mThread->setBaseName(portName);
    //mThread = new BasicRobotControllerThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool BasicApplication::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}
//#include <iostream>
//using namespace std;
bool BasicApplication::respond(const Bottle& command, Bottle& reply) {
    //cout << "CMD: "<<command.toString()<<endl;
    bool retVal = true;
    int res = mThread->respond(command,reply);
    if(res<0){
        retVal = Module::respond(command,reply);
        if(retVal){
            reply.addVocab(Vocab::encode("ack"));
        }
    }else{
        retVal = (res>=0);
    }
    return retVal;
}


double BasicApplication::getPeriod(){
    return 2.0;
}

int BasicApplication::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   BasicApplication::updateModule() {
    return true;
}



