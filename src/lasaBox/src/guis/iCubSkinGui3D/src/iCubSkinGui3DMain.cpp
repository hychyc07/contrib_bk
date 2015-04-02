// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "iCubSkinGui3DMain.h"

#include <QApplication>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>


int main(int argc, char ** argv){

    QApplication myApp( argc, argv );
    iCubSkinGui3DWidget mainWindow;
    char basename[256];
    char path[256];

    strcpy(basename,"iCubSkinGUI3D");
    strcpy(path,".");

    /*
    if(argc>1){
        strcpy(basename,argv[1]);
    }
    if(argc>2){
      strcpy(path,argv[2]);
    }
    */


    //check name parameter
    //check pathTo skinMeshes

    yarp::os::Network yarp;
    if(!yarp.checkNetwork())
        return 0;

    
    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(false);
    //rf.setDefaultConfigFile("rightforearm.ini");			//overridden by --from parameter
    //rf.setDefaultContext("iCubSkinGUI3D/conf");		//overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
    if(rf.check("name")){
        strcpy(basename,rf.find("name").toString().c_str());
    }else{
        cerr << "Info: --name parameter not provided: using default value"<<endl;        
    }
    cerr << "Info: using <"<<basename<< "> as module's basename"<<endl;

    if(rf.check("meshPath")){
        strcpy(path,rf.find("meshPath").toString().c_str());
    }else{
        cerr << "Info: --meshPath parameter not provided: using default value"<<endl;        
    }
    cerr << "Info: using <"<<path<< "> as path for skin meshes"<<endl;
    
    cerr << "Info: opening meshes..."<<endl;
    bool bOk = true;
    bOk &= mainWindow.Load(path);
    if(!bOk) 
        return 0;

    if(rf.check("showNormal")){
        mainWindow.ShowNormal(true);
    }


    iCubSkinGui3DThread *mThread;
    mThread = new iCubSkinGui3DThread(20,basename);
    mThread->mWidget = &mainWindow;
    bOk &= mThread->start();
    if(!bOk) 
        return 0;

    mainWindow.show();


    myApp.exec();

    mThread->stop();
    delete mThread;

    return 0;
}

 


iCubSkinGui3DThread::iCubSkinGui3DThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

iCubSkinGui3DThread::~iCubSkinGui3DThread()
{}

bool iCubSkinGui3DThread::threadInit()
{
    char portName[256];
    snprintf(portName,256,"/%s/skin:i",mBaseName);
    mInputPort.open(portName);

    snprintf(portName,256,"/%s/obstacles:i",mBaseName);
    mObsInputPort.open(portName);

    snprintf(portName,256,"/%s/clusters:o",mBaseName);
    mOutputPort.open(portName);

    mTime               = 0.0;
    mPrevTime           =-1.0;

    return true;
}

void iCubSkinGui3DThread::threadRelease()
{
    mInputPort.close();
    mObsInputPort.close();
    mOutputPort.close();
}

void iCubSkinGui3DThread::run()
{
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


    // Read data from input port
    yarp::sig::Matrix *inputMat = mObsInputPort.read(false);
    if(inputMat!=NULL){
        Matrix val;
        if(inputMat->rows()>1){
            val.Set(inputMat->data(),inputMat->rows()-1,inputMat->cols());
        }
        mWidget->SetObstaclePos(val);
    }

    yarp::sig::Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){
        if(inputVec->size()>0){
            Vector val(inputVec->size());
            for(int i=0;i<inputVec->size();i++){
                val.RefNoCheck(i) = 255.0 - (*inputVec)[i];
            }
            val.Trunc(0,255);
            val /=255.0;
            mWidget->SetSkinValues(val);
        }
    }



    // Write data to output port
    Vector outData;
    mWidget->GetClusterData(outData);
    yarp::sig::Vector &outputVec = mOutputPort.prepare();
    if(outData.Size()>0){
        outputVec.resize(outData.Size());
        for(int i=0;i<int(outData.Size());i++)
            outputVec[i] = outData[i];
    }else{
        outputVec.resize(1);
    }
    mOutputPort.write();
    mMutex.post();
}

