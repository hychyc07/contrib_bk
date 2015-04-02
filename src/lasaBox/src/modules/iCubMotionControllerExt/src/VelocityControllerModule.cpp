// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

#include "VelocityControllerModule.h"

int main(int argc, char *argv[])
{
  Network yarp;
  if (!yarp.checkNetwork())
    return 0;
    
  VelocityControllerModule module;

  return module.runModule(argc,argv);
}


VelocityControllerModule::VelocityControllerModule(){
    mDrivers.clear();
    mControllers.clear();
    mThread         = NULL;
    mRobotName[0]   = 0;
    mModuleName[0]  = 0;
    mPeriod         = 0.02;
    bIsReady        = false;
    bFakeDrivers    = false;
}

bool VelocityControllerModule::open(Searchable &s){
    if(bIsReady)
        return true;


    if(!mParams.check("robot")){
        mParams.put("robot","icubSim");
        fprintf(stderr, "No robot name specifed, using <icubSim> as default\n");
        fprintf(stderr, "  usage: --robot name (e.g. icub)\n");
    }
    strcpy(mRobotName,mParams.find("robot").asString().c_str());
    

    if(!mParams.check("name")){
        mParams.put("name","MotionControllerExt");
        fprintf(stderr, "No module base name specifed, using <MotionControllerExt> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    strcpy(mModuleName,mParams.find("name").asString().c_str());


    if(!mParams.check("period")){
        mParams.put("period",0.01);
        fprintf(stderr, "No module period specifed, using <0.01> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.01)\n");
    }    
    double period = mParams.find("period").asDouble();
    if(period<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.01> ms as default\n",period);
        period = 0.01;
    }
    mPeriod = period;

    bFakeDrivers = mParams.check("fake");
    if(bFakeDrivers){
       fprintf(stderr, "Using fake drivers if necessay\n"); 
    }


    int  nbParts = 12;
    char partsName[nbParts][256];
    strcpy(partsName[ 0],"right_arm");
    strcpy(partsName[ 1],"left_arm");
    strcpy(partsName[ 2],"head");
    strcpy(partsName[ 3],"torso");
    strcpy(partsName[ 4],"right_leg");
    strcpy(partsName[ 5],"left_leg");
    strcpy(partsName[ 6],"fake_right_arm");
    strcpy(partsName[ 7],"fake_left_arm");
    strcpy(partsName[ 8],"fake_head");
    strcpy(partsName[ 9],"fake_torso");
    strcpy(partsName[10],"fake_right_leg");
    strcpy(partsName[11],"fake_left_leg");
    
    char txtBuffer[256];         

    bool hasValidDriver = false;
    
    if(!mParams.check("part")){
        fprintf(stderr, "No parts specifed or they where unable to open");
        fprintf(stderr, "  Please choose and check at least one:\n");
        fprintf(stderr, "    e.g. --part (right_arm+left_arm+right_leg+left_leg+head+torso \n");
        return false;    
    }else{
        char partList[512];
        strcpy(partList,mParams.find("part").asString().c_str());
        int partPos = 0;
        int partPosEnd = 0;
        while(partList[partPos]!=0){
            partPosEnd = partPos;
            while((partList[partPosEnd]!=0)&&(partList[partPosEnd]!='+')){
                partPosEnd++;
            }
            if(partPosEnd!=partPos){
                int partId = -1;
                for(int i=0;i<nbParts;i++){
                    if(strncmp(partsName[i],partList+partPos,partPosEnd-partPos)==0){
                        partId = i;
                        break;
                    }
                }
                if(partId>=0){
                    Property options("");
                    options.put("device","remote_controlboard");
                    sprintf(txtBuffer,"/%s/%s",mRobotName,partsName[partId]);
                    options.put("remote",txtBuffer);
                    sprintf(txtBuffer,"/%s/%s",mModuleName,partsName[partId]);
                    options.put("local",txtBuffer);    
                    PolyDriver *driver = new PolyDriver(options);
                    if(!driver->isValid()){
                        driver->close();
                        delete driver;
                        driver = NULL;                
                    }
                    if((driver!=NULL)||(bFakeDrivers)||(partId>=6)){
                        hasValidDriver = true;
                        VelocityController* vc = new VelocityController;
                        vc->Init(driver,partsName[partId],mModuleName);
                        mControllers.push_back(vc);    
                    }
                    mDrivers.push_back(driver);
                }
            }
            if(partList[partPosEnd]==0)
                break;
            partPos = partPosEnd+1;
        }
        if(hasValidDriver){    
            sprintf(txtBuffer,"/%s/rpc",mModuleName);
            mRpcPort.open(txtBuffer);
            mRpcPort.setStrict();
            attach(mRpcPort,true);

            mThread = new VelocityControllerThread(int(floor(mPeriod*1000.0)),mModuleName,&mControllers);
            mThread->start();

            return true;        
        }else{
            fprintf(stderr, "No parts specifed or they where unable to open");
            fprintf(stderr, "  Please choose and check at least one:\n");
            fprintf(stderr, "    --part \"right_arm | left_arm | right_leg | left_leg | head | torso\" \n");
            return false;
        }

    }
}

bool VelocityControllerModule::close(){
    if(!bIsReady)
        return true;

    mRpcPort.close();
    mThread->stop();

    for(int i=0;i<int(mControllers.size());i++){
        delete mControllers[i];
    }
    mControllers.clear();
    for(int i=0;i<int(mDrivers.size());i++){
        if(mDrivers[i]) delete mDrivers[i];
    }
    mDrivers.clear();
    

    return true;
}

bool VelocityControllerModule::respond(const Bottle& command, Bottle& reply) {
    int index = 0;
    int cmdSize = command.size();

    bool retVal = true;
    
    if(cmdSize>0){
        cerr << "Cmd: ";
        for(int i=0;i<cmdSize;i++)
            cerr <<command.get(i).asString().c_str()<<" ";
        cerr <<endl;
        switch(command.get(0).asVocab())  {
        /*
        case VOCAB4('m','a','s','k'):
            if(cmdSize==2){
                if(command.get(1).asString() == "all"){
                    for(int i=0;i<int(mControllers.size());i++)
                        mControllers[i]->SetMaskAll();
                }else if(command.get(index+1).asString() == "none"){
                    for(int i=0;i<int(mControllers.size());i++)
                        mControllers[i]->SetMaskNone();                     
                }
            }else if(cmdSize>2){
                Vector mask(cmdSize-1);
                Vector submask;
                for(int j=0;j<cmdSize-1;j++){
                    mask[j] = command.get(j+1).asDouble();
                    for(int i=0;i<int(mControllers.size());i++)
                        mControllers[i]->SetMask(mThread->DispatchVector(mask,i,submask));
                }
            }else{
                retVal = false;    
            }
            break;
        */
        case VOCAB3('r','u','n'):
        case VOCAB4('r','u','n','p'):
            for(int i=0;i<int(mControllers.size());i++){
                mControllers[i]->SetControlMode(VelocityController::VC_POSITION);
            }
            break;
        case VOCAB4('r','u','n','v'):
            for(int i=0;i<int(mControllers.size());i++){
                mControllers[i]->SetControlMode(VelocityController::VC_VELOCITY);
            }            
            break;
        case VOCAB4('r','u','n','c'):
            for(int i=0;i<int(mControllers.size());i++){
                mControllers[i]->SetControlMode(VelocityController::VC_COMBINED);
            }
            break;
        case VOCAB4('i','d','l','e'):
        case VOCAB4('s','u','s','p'):
        case VOCAB4('s','t','o','p'):
            for(int i=0;i<int(mControllers.size());i++){
                mControllers[i]->SetControlMode(VelocityController::VC_IDLE);
            }            
            break;
        case VOCAB4('r','e','s','t'):
            for(int i=0;i<int(mControllers.size());i++){
                mControllers[i]->SetControlMode(VelocityController::VC_REST);
            }
            break;
        /*
        case VOCAB2('k','p'):
            if(cmdSize==2){
                for(int i=0;i<int(mControllers.size());i++)
                    mControllers[i]->SetKp(command.get(1).asDouble());
            }            
            break;
        case VOCAB2('k','d'):
            if(cmdSize==2){
                for(int i=0;i<int(mControllers.size());i++)
                    mControllers[i]->SetKd(command.get(1).asDouble());
            }            
            break;
        */
        default:
            retVal = false;
            break;            
        }         
        
    }else{
        retVal = false;
    }

    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }else{
        return Module::respond(command,reply);
    }
    
    return true;
}


double VelocityControllerModule::getPeriod(){
    return 2.0;
}
int VelocityControllerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);                                    
}

bool   VelocityControllerModule::updateModule() {
    return true;
}


VelocityControllerThread::VelocityControllerThread(int period,
                                                   const char* moduleName,
                                                   vector<VelocityController*> *controllers)
:RateThread(period)
{
    mPeriod         = period;
    mControllersPtr = controllers;
    strcpy(mModuleName, moduleName);
    mTime               = 0.0;
    mPrevTime           =-1.0;    
}

VelocityControllerThread::~VelocityControllerThread(){}

void VelocityControllerThread::run(){
    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;    
    
    vector<VelocityController*> &mControllers = *mControllersPtr;
    
    int totalJoints = 0;
    for(int i=0;i<int(mControllers.size());i++)
        totalJoints += mControllers[i]->GetJointsSize();
    
    Vector target;
    Vector *remoteTarget = NULL;
    remoteTarget = mTargetPosPort.read(false);    
    if(remoteTarget!=NULL){
        for(int i=0;i<int(mControllers.size());i++){
            mControllers[i]->SetPositionTarget(DispatchVector(*remoteTarget,i,target));
        }        
    }
    remoteTarget = mTargetVelPort.read(false);    
    if(remoteTarget!=NULL){
        for(int i=0;i<int(mControllers.size());i++){
            mControllers[i]->SetVelocityTarget(DispatchVector(*remoteTarget,i,target));
        }        
    }

    
    for(int i=0;i<int(mControllers.size());i++){
        mControllers[i]->Update(dt);
    }
    
    Vector &outputPosVec = mPosPort.prepare();
    Vector &outputVelVec = mVelPort.prepare();
    outputPosVec.resize(totalJoints);
    outputVelVec.resize(totalJoints);

    Vector cpos,cvel;
    int cid = 0;
    for(int i=0;i<int(mControllers.size());i++){
        int size = mControllers[i]->GetJointsSize();
        mControllers[i]->GetPosition(cpos);
        mControllers[i]->GetVelocity(cvel);
        for(int j=0;j<size;j++){
            outputPosVec[cid] = cpos[j];
            outputVelVec[cid] = cvel[j];
            cid++;
        }
    }
    mPosPort.write();    
    mVelPort.write();    
}

bool VelocityControllerThread::threadInit(){
    char txtBuffer[256];
    
    sprintf(txtBuffer,"/%s/targetPosition",mModuleName);
    mTargetPosPort.open(txtBuffer);

    sprintf(txtBuffer,"/%s/targetVelocity",mModuleName);
    mTargetVelPort.open(txtBuffer);

    sprintf(txtBuffer,"/%s/position",mModuleName);
    mPosPort.open(txtBuffer);

    sprintf(txtBuffer,"/%s/velocity",mModuleName);
    mVelPort.open(txtBuffer);
}

void VelocityControllerThread::threadRelease(){
    mTargetPosPort.close();
    mTargetVelPort.close();
    mPosPort.close();
    mVelPort.close();
}

Vector& VelocityControllerThread::DispatchVector(Vector& src, int cId, Vector &res){
    vector<VelocityController*> &mControllers = *mControllersPtr;

    if((cId<0)||(cId>=int(mControllers.size()))){
        res.resize(0);
        return res;
    }
        
    int offset = 0;
    for(int i=0;i<cId;i++)
        offset += mControllers[i]->GetJointsSize();

    int size = mControllers[cId]->GetJointsSize();            

    int srcSize = src.size();

    int mx = (offset+size<=srcSize?size:srcSize-offset);
    
    res.resize(mx);
    for(int i=0;i<mx;i++)
        res[i] = src[i+offset];
    return res;
}


