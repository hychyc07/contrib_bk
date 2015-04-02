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

#ifndef BasicApplicationThread_H_
#define BasicApplicationThread_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include <vector>
#include <string>

using namespace std;

#define TEST_AND_ADD_CTRL_PORT(id,port) {int res = AddControlPort(port); if(res!=id) cerr << "**** AddControlPort Error: bad returned id: "<<res<< " != " << id << " ****"<<endl;}
#define TEST_AND_ADD_SRC_PORT(id,port)  {int res = AddSrcPort(port);     if(res!=id) cerr << "**** AddSrcPort Error: bad returned id: "<<res<< " != " << id << " ****"<<endl;}
#define TEST_AND_ADD_DST_PORT(id,port)  {int res = AddDstPort(port);     if(res!=id) cerr << "**** AddDstPort Error: bad returned id: "<<res<< " != " << id << " ****"<<endl;}

#define MAX_CTRL_PORTS_COUNT             32
#define MAX_SRC_PORTS_COUNT             128
#define MAX_DST_PORTS_COUNT             128

class BasicApplicationThread: public RateThread
{
protected:    
    Semaphore               mMutex;
    char                    mBaseName[256];
    bool                    bFakeNetwork;
        
    int                     mSrcPortsCount;
    int                     mDstPortsCount;
    int                     mCtrlPortsCount;
    
    char                    mSrcPortName[MAX_SRC_PORTS_COUNT][256];
    char                    mDstPortName[MAX_DST_PORTS_COUNT][256];
    char                    mSrcCtrlPortName[MAX_CTRL_PORTS_COUNT][256];
    char                    mDstCtrlPortName[MAX_CTRL_PORTS_COUNT][256];

    bool                    mSrcPortFound[MAX_SRC_PORTS_COUNT];
    bool                    mDstPortFound[MAX_DST_PORTS_COUNT];
    
    
    vector<int>             mCommandsType;
    vector<int>             mConnexionsSrcPort;
    vector<int>             mConnexionsDstPort;
    vector<string>          mCommands;
    vector<int>             mCommandsPort;
    
    BufferedPort<Bottle>    mCommandPorts[MAX_CTRL_PORTS_COUNT];

    
    int                     mBasicCommand;
    string                  mBasicCommandParams;

    
public:
    BasicApplicationThread(int period = 1000, const char* baseName = NULL);
    virtual ~BasicApplicationThread();

    virtual void    SetupPorts();
    
            int     AddControlPort      (const char* portName);
            int     AddSrcPort          (const char* portName);
            int     AddDstPort          (const char* portName);
    
            void    ConnectToNetwork    (bool bConnect);
            void    ClearCommands       ();
            void    SendCommands        ();
            void    AddCommand          (int port, const char *cmd, const char *params=NULL);
            void    AddConnexion        (int src,  int dst, bool bUnique = true);
            void    RemConnexion        (int src,  int dst);
            void    RemAllSrcConnexions (int dst);
            void    RemAllDstConnexions (int src);
            void    RemAllConnexions    ();

    virtual void    ProcessBasicCommand();

            int     respond(const Bottle& command, Bottle& reply);
    virtual int     respondToBasicCommand(const Bottle& command, Bottle& reply);

    virtual void    run();
    virtual bool    threadInit();
    virtual void    threadRelease();
            void    setBaseName(const char* baseName);
};

#endif

