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

#include "iCubRecordAndReplay.h"

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return 0;

    /*
    char partName[256];
    if(argc==2){
        strcpy(partName, argv[1]);
    }else{
        fprintf(stderr, "Error: part name required\n");
        fprintf(stderr, "  Example: ./iCubRecordAndReplay right_leg\n");
        return 0;
    }
    */

    iCubRecordAndReplayModule *module = new iCubRecordAndReplayModule();

    module->runModule(argc,argv);
    
    delete module;

    return 0;
}


iCubRecordAndReplayModule::iCubRecordAndReplayModule(){
    mDriver         = NULL;
    mThread         = NULL;
    mRobotName[0]   = 0;
    mModuleName[0]  = 0;
    mPeriod         = 0.02;
    bIsReady        = false;
    mPartName[0]    = 0;
}

bool iCubRecordAndReplayModule::open(Searchable &s){
    if(bIsReady)
        return true;

    mPeriod = 0.01;

    char txtBuffer[256];         

    char robotName[256];
    if(s.check("robot")){
        strcpy(robotName,s.find("robot").toString().c_str());
    }else{
        fprintf(stderr, "Error: no --robot specified\n");
        return false;
    }

    if(s.check("part")){
        strcpy(mPartName,s.find("part").toString().c_str());
    }else{
        fprintf(stderr, "Error: no --part specified\n");
        return false;
    }

    

    Property options("");
    options.put("device","remote_controlboard");
    sprintf(txtBuffer,"/%s/%s",robotName,mPartName);
    options.put("remote",txtBuffer);
    if(s.check("name")){
        sprintf(txtBuffer,"/%s/%s",s.find("name").toString().c_str(),mPartName);
    }else{
        sprintf(txtBuffer,"/%s/%s","iCubRecordAndReplay",mPartName);
    }
    options.put("local",txtBuffer);    
    PolyDriver *driver = new PolyDriver(options);
    if(!driver->isValid()){
        driver->close();
        delete driver;
        driver = NULL;                
        fprintf(stderr, "Error: failed to open driver for the part: %s\n",mPartName);
        return false;
    }

    mDriver = driver;

    if(s.check("name")){
        sprintf(txtBuffer,"/%s/%s/rpc",s.find("name").toString().c_str(),mPartName);
    }else{
        sprintf(txtBuffer,"/%s/%s/rpc","iCubRecordAndReplay",mPartName);
    }
    mRpcPort.open(txtBuffer);
    mRpcPort.setStrict();
    attach(mRpcPort,true);

    if(s.check("name")){
        sprintf(txtBuffer,"/%s/%s",s.find("name").toString().c_str(),mPartName);
    }else{
        sprintf(txtBuffer,"/%s/%s","iCubRecordAndReplay",mPartName);
    }
    mThread = new iCubRecordAndReplayThread(int(floor(mPeriod*1000.0)),mDriver,txtBuffer);
    bIsReady = mThread->start();

    return bIsReady;        
}

bool iCubRecordAndReplayModule::close(){

    mThread->stop();

    mRpcPort.close();

    if(mDriver!=NULL){
        mDriver->close();
        delete mDriver;
    }
    mDriver = NULL;

    return true;
}

bool iCubRecordAndReplayModule::respond(const Bottle& command, Bottle& reply) {
    int index = 0;
    int cmdSize = command.size();

    bool retVal = true;
    
    if(cmdSize>0){
        switch(command.get(0).asVocab())  {
        case VOCAB4('l','o','a','d'):
            if(cmdSize==2){
                mThread->load(command.get(1).asString().c_str());
            }else{
                retVal = false;    
            }
            break;
        case VOCAB4('s','a','v','e'):
            if(cmdSize==2){
                mThread->save(command.get(1).asString().c_str());
            }else{
                retVal = false;    
            }
            break;

        case VOCAB4('g','c','m','p'):
            if(cmdSize==2){
                if(command.get(1).asString()=="arm"){
                    mThread->gravityComp(1);
                }else if(command.get(1).asString()=="hand"){
                    mThread->gravityComp(2);
                }
            }else{
                mThread->gravityComp();
            }
            break;
        case VOCAB3('p','o','s'):
            mThread->positionMode();
            break;
        case VOCAB3('r','e','c'):
            mThread->record();
            break;
        case VOCAB4('s','t','o','p'):
            mThread->stop();
            break;
        case VOCAB4('p','r','e','p'):
            mThread->move();
            break;
        case VOCAB2('g','o'):
            if(cmdSize==1){
                mThread->go();
            }else if(cmdSize==2){
                if(strcmp("force",command.get(1).asString().c_str())==0){
                    mThread->go(true);
                }else{
                    cerr << "Error"<<endl;
                    retVal = false;    
                }
            }else{
                cerr << "Error"<<endl;
                retVal = false;    
            }
            break;
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


double iCubRecordAndReplayModule::getPeriod(){
    return 2.0;
}
int iCubRecordAndReplayModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);                                    
}

bool   iCubRecordAndReplayModule::updateModule() {
    return true;
}


iCubRecordAndReplayThread::iCubRecordAndReplayThread(int period, PolyDriver* driver, const char * name)
:RateThread(period)
{
    bIsReady        = false;

    strcpy(mModuleName, name);

    mPeriod         = period;
    mTime           = 0.0;
    mPrevTime       =-1.0;    
    mDriver         = driver;

    bool bOk = true;
    if(mDriver){
        bOk &= mDriver->view(mPositionController);
        bOk &= mDriver->view(mPIDController);
        bOk &= mDriver->view(mEncoders);  
        bOk &= mDriver->view(mControlModeController);
        bOk &= mDriver->view(mLimitsController);
        bOk &= mDriver->view(mAmplifierController);

        if(bOk){
            mPositionController->getAxes(&mJointsSize);
            cout << "JointSize: " <<mJointsSize<<endl;

            mJointsPos.resize(mJointsSize);
            mJointsLim[0].resize(mJointsSize);
            mJointsLim[1].resize(mJointsSize);
            
            for(int i=0;i<mJointsSize;i++){
                mLimitsController->getLimits(i,mJointsLim[0].data()+i,mJointsLim[1].data()+i);
            }
            mPos = -1;

            bPlaying    = false;
            bReady      = false;

            char txt[256];
            sprintf(txt,"%s/joints:o",mModuleName);
            mJointPort.open(txt);

            bIsReady    = true;
        }else{
            cout << "Error while retrieving drivers interfaces"<<endl;
        }
    }
}

iCubRecordAndReplayThread::~iCubRecordAndReplayThread(){
}

bool iCubRecordAndReplayThread::threadInit(){
    if(bIsReady){
        bPlaying        = false;
        bReady          = false;
        bPositionMode   = false;
        bHasRecord      = 0;
        positionMode();
        return true;
    }else{
        return false;
    }
}

void iCubRecordAndReplayThread::threadRelease(){
    mJointPort.close();
}

void iCubRecordAndReplayThread::run(){
    mMutex.wait();

    if(mPrevTime<0.0){
        mPrevTime = Time::now();
        mMutex.post();
        return;
    }else{
        mPrevTime = mTime;    
    }    
    mTime       = Time::now();
    double dt   = mTime - mPrevTime;    

    mEncoders->getEncoders              (mJointsPos.data());

    {
        Vector & vec = mJointPort.prepare();
        vec.resize(mJointsSize);
        vec = mJointsPos;
        mJointPort.write();
    }

    if(bPlaying){    
        // Started and not done?
        if(mPos>=0){

            // Prepare position data
            Vector pos;
            pos.resize(mJointsSize);
            for(int i=0;i<mJointsSize;i++){
                pos[i] = TRUNC(mData(mPos,i),mJointsLim[0][i],mJointsLim[1][i]);
            }

            // Execute them
            mPIDController->setReferences(pos.data());

            // Increase counter until done
            mPos++;
            if(mPos>=mData.RowSize()){
                mPos = -1;
                cout << "Playing done"<<endl;
                bPlaying = false;
            }
        }
        bHasRecord = 0;
    }else{
        // Recording started or not done?
        if(mPos>=0){
            // Get data
            for(int i=0;i<mJointsSize;i++){
                mData(mPos,i) = TRUNC(mJointsPos[i],mJointsLim[0][i],mJointsLim[1][i]);
            }
            // Increase counter
            mPos++;
            bHasRecord = mPos;
            if(mPos>=mData.RowSize()){
                mPos = -1;
                cout << "Record buffer overflow: done with recording"<<endl;
            }
            if(bHasRecord>mData.RowSize()){
                bHasRecord = mData.RowSize();
            }
        }
    }
    mMutex.post();
}

void iCubRecordAndReplayThread::stop(){
    mMutex.wait();

    if(mPos>=0){
        if(bPlaying){
            cerr << "Stop playing"<<endl;
            bPlaying = false;
        }
        else
            cerr << "Stop recording"<<endl;
    }

    mPos = -1;

    mMutex.post();
}


void iCubRecordAndReplayThread::load(const char *name){
    mMutex.wait();
    bReady = false;

    bHasRecord = 0;

    if(mPos>=0){
        if(bPlaying)
            cerr << "Stop playing"<<endl;
        else
            cerr << "Stop recording"<<endl;
    }

    mPos = -1;
    if(mData.Load(name)){
        cerr << "File "<<name<<" loaded"<<endl;
        cerr << "  Data size: "<<mData.RowSize()<<" "<<mData.ColumnSize()<<endl;
        if(mData.ColumnSize()==mJointsSize){
            cerr << "  Ready"<<endl;
            bReady = true;            
        }else{
            cerr << "  Error: number of column (nb of joints in the data) doesn't match. Should be "<< mJointsSize<<endl;
        }
    }else{
        cerr << "Error while loading file: "<<name<<endl;
    }
    mMutex.post();
}

void iCubRecordAndReplayThread::save(const char *name){
    mMutex.wait();
    if(!bPlaying){
        if(bHasRecord>0){
            if(mData.Save(name,4,bHasRecord)){
                cerr << "File "<<name<<" saved"<<endl;
                cerr << "  Data size: "<<bHasRecord<<" "<<mData.ColumnSize()<<endl;            
            }else{
                cerr << "Error while saving file "<<name<<endl;
            }
        }else{
            cerr << "Nothing to be saved"<<endl;
        }
        mPos = -1;
    }else{
        cerr << "Nothing to be saved: nothing was recorded recently"<<endl;
    }
    mMutex.post();
}


void iCubRecordAndReplayThread::go(bool force){
    mMutex.wait();

    if(mPos<0){
        if(bReady){
            mMutex.post();
            positionMode();
            mMutex.wait();

            if(!force){
                // Checking good start conditions
                double dist    = 0;
                double maxDist = 0;
                int js = MIN(mJointsSize,7);
                for(int i=0;i<js;i++){
                    double v = mJointsPos[i]- mData(0,i);
                    dist = v*v;
                    if(dist>maxDist) maxDist = dist;
                }
                //dist = sqrt(dist);

                Vector pos; pos.resize(mJointsSize);
                for(int i=0;i<mJointsSize;i++){
                    pos[i] = mData(0,i);
                }        
                if(maxDist<2.0){
                    mPos = 0;
                    bPlaying    = true;

                    cout << "Started"<<endl;
                }else{
                    cout << "Too far: ("<<maxDist<<") Move first"<<endl;
                    cout << "Is there:        "<<mJointsPos.toString()<<endl;
                    cout << "Should be there: "<<pos.toString()<<endl;
                }
            }else{
                mPos = 0;
                bPlaying    = true;

                cout << "Started (forcing mode!!!)"<<endl;
            }
        }else{
            cerr<<"Error: not ready"<<endl;
        }
    }else{
        cerr<<"Error: not done with current task"<<endl;
    }
    mMutex.post();
}

void iCubRecordAndReplayThread::move(){
    mMutex.wait();

    if(mPos<0){
        if(bReady){

            mMutex.post();
            positionMode();
            mMutex.wait();

            Vector pos; pos.resize(mJointsSize);
            for(int i=0;i<mJointsSize;i++){
                pos[i] = mData(0,i);
            }
            cout << "Going there: "<<pos.toString()<<endl;
            mPositionController->positionMove(pos.data());

            bPlaying    = false;

        }else{
            cerr<<"Error: not ready"<<endl;
        }
    }else{
        cerr<<"Error: not done with current task"<<endl;
    }
    mMutex.post();
}

void iCubRecordAndReplayThread::gravityComp(int part){
    mMutex.wait();

    if(part!=0){
        if(mJointsSize==16){
            if(part==1){
                for(int i=0;i<8;i++){
                    mControlModeController->setTorqueMode(i);
                }    
                bPositionMode   = false;
            }else if(part==2){
                for(int i=8;i<mJointsSize;i++){
                    mControlModeController->setTorqueMode(i);
                }    
                bPositionMode   = false;
            }
        }
    }else{
        if(bPositionMode){    
            for(int i=0;i<mJointsSize;i++){
                mControlModeController->setTorqueMode(i);
            }
            bPositionMode   = false;
        }
    }
    mMutex.post();
}

void iCubRecordAndReplayThread::positionMode(){
    mMutex.wait();

    if(!bPositionMode){
        if(mEncoders->getEncoders(mJointsPos.data())){

            for(int i=0;i<mJointsSize;i++){
                mAmplifierController->enableAmp(i);
                mPIDController->enablePid(i);
                mControlModeController->setPositionMode(i);
            }
            Vector vels; vels.resize(mJointsSize);
            vels = 10;
            mPositionController->setRefSpeeds(vels.data());

            bPositionMode   = true;

        }else{
            cerr<<"Error: unable to get position response from motor: please retry."<<endl;
        }
    }

    mMutex.post();
}


void iCubRecordAndReplayThread::record(){
    mMutex.wait();
    if(mPos<0){
        if(!bPlaying){
            mData.Resize(10000,mJointsSize);
            mPos = 0;
            bHasRecord = 0;
            cerr << "Starting recording (max 10'000 datapoints)"<<endl;
        }else{
            cerr << "Cannot record: bad mode"<<endl;
        }    
    }else{
        cerr<<"Error: not done with current task"<<endl;
    }
    mMutex.post();
}

