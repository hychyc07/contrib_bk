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

 
#include "BasicApplicationThread.h"

#include <string.h>
#include <iostream>
using namespace std;

#include <yarp/os/Network.h>
using namespace yarp::os;

#include "StdTools/Various.h"



BasicApplicationThread::BasicApplicationThread(int period, const char* baseName)
:RateThread(period)
{
    if((baseName!=NULL)&&(baseName[0]!=0))  strncpy(mBaseName,baseName,256);
    else                                    strncpy(mBaseName,"DefaultApplicationThread",256);
    
    mCtrlPortsCount = 0;
    mSrcPortsCount  = 0;
    mDstPortsCount  = 0;

}
BasicApplicationThread::~BasicApplicationThread()
{}

void    BasicApplicationThread::setBaseName(const char* baseName){
    if((baseName!=NULL)&&(baseName[0]!=0))  strncpy(mBaseName,baseName,256);
    else                                    strncpy(mBaseName,"DefaultApplicationThread",256);
}


int     BasicApplicationThread::AddControlPort(const char* portName){
    if(mCtrlPortsCount>=MAX_CTRL_PORTS_COUNT){
        cerr << "Error: too many ctrl port"<<endl;
        return -1;
    }
    snprintf(mSrcCtrlPortName[mCtrlPortsCount],        256,"/%s/%s",mBaseName,portName);
    snprintf(mDstCtrlPortName[mCtrlPortsCount],        256,"/%s/rpc",portName);
    mCtrlPortsCount++;
    return mCtrlPortsCount-1;
}
int     BasicApplicationThread::AddSrcPort(const char* portName){
    if(mSrcPortsCount>=MAX_SRC_PORTS_COUNT){
        cerr << "Error: too many src port"<<endl;
        return -1;
    }
    snprintf(mSrcPortName[mSrcPortsCount],        256,"%s",portName);
    mSrcPortsCount++;
    return mSrcPortsCount-1;
}
int     BasicApplicationThread::AddDstPort(const char* portName){
    if(mDstPortsCount>=MAX_DST_PORTS_COUNT){
        cerr << "Error: too many dst port"<<endl;
        return -1;
    }
    snprintf(mDstPortName[mDstPortsCount],        256,"%s",portName);
    mDstPortsCount++;
    return mDstPortsCount-1;
}

void  BasicApplicationThread::SetupPorts(){
}

bool BasicApplicationThread::threadInit()
{    
    cout << "***********************************"<<endl;
    cout << "Setting up ports:"<<endl;
    SetupPorts();
    cout << "***********************************"<<endl;
    
    cout << "***********************************"<<endl;
    cout << "Opening control ports:"<<endl;
    for(int i=0;i<mCtrlPortsCount;i++){
        mCommandPorts[i].open(mSrcCtrlPortName[i]);
        mCommandPorts[i].setStrict();
    }
    cout << "***********************************"<<endl;

    for(int i=0;i<MAX_SRC_PORTS_COUNT;i++)
        mSrcPortFound[i] = false;
    for(int i=0;i<MAX_DST_PORTS_COUNT;i++)
        mDstPortFound[i] = false;

    mBasicCommand = 0;
    
    return true;
}

void BasicApplicationThread::threadRelease()
{
    ClearCommands();
    RemAllConnexions();

    ProcessBasicCommand();
    
    SendCommands();
    ConnectToNetwork(false);

    for(int i=0;i<mCtrlPortsCount;i++){
        mCommandPorts[i].close();
    }

}

void BasicApplicationThread::run()
{
    mMutex.wait();
    ClearCommands();

    ProcessBasicCommand();
    
    SendCommands();
    mMutex.post();
    
}


void BasicApplicationThread::ProcessBasicCommand(){
    mBasicCommand       = 0;
    mBasicCommandParams = "";
}



void BasicApplicationThread::ConnectToNetwork(bool bConnect){
    cout <<"*******************************************************"<<endl;
    cout << (bConnect?"Connecting to network":"Disonnecting from network")<<endl;
    cout <<"*******************************************************"<<endl;
    bool bPConnected[MAX_CTRL_PORTS_COUNT];
    
    for(int i=0;i<mCtrlPortsCount;i++){
        bPConnected[i] = false;
        if(!bFakeNetwork){
            if(bConnect){
                if(!Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                    if(Network::connect(mSrcCtrlPortName[i],mDstCtrlPortName[i])){
                        bPConnected[i] = true;
                    }
            }else{
                if( Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                    Network::disconnect(mSrcCtrlPortName[i],mDstCtrlPortName[i]);
            }
        }
    }

    if(bConnect){
        char portname[256];    
        snprintf(portname,256,"/%s/dummy",mBaseName);

        Port dummy;
        dummy.open(portname);
        for(int i=0;i<mSrcPortsCount;i++){
            mSrcPortFound[i] = false;
            Network::connect(mSrcPortName[i],portname);
            if(Network::isConnected(mSrcPortName[i],portname)){
                Network::disconnect(mSrcPortName[i],portname);
                mSrcPortFound[i] = true;
            }
        }
        for(int i=0;i<mDstPortsCount;i++){
            mDstPortFound[i] = false;
            Network::connect(portname,mDstPortName[i]);
            if(Network::isConnected(portname,mDstPortName[i])){
                Network::disconnect(portname,mDstPortName[i]);
                mDstPortFound[i] = true;
            }
        }
        dummy.close();
    }
    if(bConnect){
        cout <<"*******************************************************"<<endl;
        cout <<"Connexion summary:"<<endl;
        cout <<"*******************************************************"<<endl;
        for(int i=0;i<mCtrlPortsCount;i++){
            if(!bPConnected[i])
                cerr << "Warning: Unable to connect             "<<mSrcCtrlPortName[i] <<" to "<<mDstCtrlPortName[i]<<endl;
        }
        for(int i=0;i<mSrcPortsCount;i++){
            if(!mSrcPortFound[i])
                cerr << "Warning: Unreachable Source port:      "<<mSrcPortName[i] <<endl;
        }
        for(int i=0;i<mDstPortsCount;i++){
            if(!mDstPortFound[i])
                cerr << "Warning: Unreachable Destination port: "<<mDstPortName[i] <<endl;
        }
        cout <<"*******************************************************"<<endl;
    }
}

void BasicApplicationThread::ClearCommands(){
    mCommandsType.clear();
    mCommands.clear();
    mCommandsPort.clear();
    mConnexionsSrcPort.clear();
    mConnexionsDstPort.clear();
}

void BasicApplicationThread::AddCommand(int port, const char *cmd, const char *params){
    if((port<0)||(port>=mCtrlPortsCount)){
        cerr << "Error: Bad port number <"<< port <<"> while adding command."<<endl;
        return;
    }
    if(cmd!=NULL){
        if(cmd[0]!=0){
            mCommandsType.push_back(0);
            if((params!=NULL)&&(params[0]!=0)){
                char cmd2[512];
                snprintf(cmd2,512,"%s %s",cmd,params);
                mCommands.push_back(cmd2);
            }else{
                mCommands.push_back(cmd);
            }
            mCommandsPort.push_back(port);
        }
    }
}

void    BasicApplicationThread::AddConnexion(int src, int dst, bool bUnique){
    if((src<0)||(src>=mSrcPortsCount)){
        cerr << "Error: Bad src port number <"<< src <<"> while adding connexion."<<endl;
        return;
    }
    if((dst<0)||(dst>=mDstPortsCount)){
        cerr << "Error: Bad dst port number <"<< dst <<"> while adding connexion."<<endl;
        return;
    }

    if(bUnique)
        RemAllSrcConnexions(dst);
    
    mCommandsType.push_back(1);
    mConnexionsSrcPort.push_back(src);
    mConnexionsDstPort.push_back(dst);
}
void    BasicApplicationThread::RemConnexion(int src, int dst){
    if((src<0)||(src>=mSrcPortsCount)){
        cerr << "Error: Bad src port number <"<< src <<"> while removing connexion."<<endl;
        return;
    }
    if((dst<0)||(dst>=mDstPortsCount)){
        cerr << "Error: Bad dst port number <"<< dst <<"> while removing connexion."<<endl;
        return;
    }

    if((mSrcPortFound[src])&&(mDstPortFound[dst])){
        mCommandsType.push_back(2);
        mConnexionsSrcPort.push_back(src);
        mConnexionsDstPort.push_back(dst);
    }
}
void    BasicApplicationThread::RemAllSrcConnexions(int dst){
    if((dst<0)||(dst>=mDstPortsCount)){
        cerr << "Error: Bad dst port number <"<< dst <<"> while removing all src connexion."<<endl;
        return;
    }
    for(int i=0;i<mSrcPortsCount;i++)
        RemConnexion(i,dst);
}
void    BasicApplicationThread::RemAllDstConnexions(int src){
    if((src<0)||(src>=mSrcPortsCount)){
        cerr << "Error: Bad src port number <"<< src <<"> while removing all dst connexion."<<endl;
        return;
    }
    for(int i=0;i<mDstPortsCount;i++)
        RemConnexion(src,i);
}
void    BasicApplicationThread::RemAllConnexions(){
    for(int i=0;i<mSrcPortsCount;i++)
        for(int j=0;j<mDstPortsCount;j++)
            RemConnexion(i,j);
}


void BasicApplicationThread::SendCommands(){
    int cmdCnt = 0;
    int conCnt = 0;
    if(mCommandsType.size()>0){
        cout <<"*******************************************************"<<endl;
        cout << "Sending commands..."<<endl;
    }
    for(size_t i=0;i<mCommandsType.size();i++){
        switch(mCommandsType[i]){
        case 0:
            {
                if(!bFakeNetwork){
                    Bottle &cmd = mCommandPorts[mCommandsPort[cmdCnt]].prepare();
                    cmd.fromString(mCommands[cmdCnt].c_str());
                    mCommandPorts[mCommandsPort[cmdCnt]].writeStrict();
                }
                cout << "  Sending: <"<<mCommands[cmdCnt]<<"> to: "<< mDstCtrlPortName[mCommandsPort[cmdCnt]] <<endl;
                cmdCnt++;
                break;
            }
        case 1:
            {
                cout << "  Connecting: "<<mSrcPortName[mConnexionsSrcPort[conCnt]]<<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;
                if(!bFakeNetwork){
                    if(!Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                        if(!Network::connect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                            cerr << "Error: Unable to connect "<<mSrcPortName[mConnexionsSrcPort[conCnt]] <<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;                
                }
                conCnt++;
            }
            break;
        case 2:
            {
                if(!bFakeNetwork){
                    if(Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]])){
                        cout << "  Disconnecting: "<<mSrcPortName[mConnexionsSrcPort[conCnt]]<<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;
                        Network::disconnect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]);
                    }
                }
                conCnt++;
            }
            break;
        }        
    }
    if(mCommandsType.size()>0){
        cout <<"*******************************************************"<<endl;
    }

}

int BasicApplicationThread::respond(const Bottle& command, Bottle& reply){
    mMutex.wait();
    int res = 0;
    ClearCommands();

    res = respondToBasicCommand(command, reply);
    ProcessBasicCommand();
    
    SendCommands();
    mMutex.post();
    return res;
}

int BasicApplicationThread::respondToBasicCommand(const Bottle& command, Bottle& reply){

    int  cmdSize    = command.size();
    int  retVal     = 1;
    return retVal;
}
