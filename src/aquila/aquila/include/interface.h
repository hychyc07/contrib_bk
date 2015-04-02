//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, <Martin Peniak - www.martinpeniak.com>																																					//
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <yarp/os/all.h>
#include "gui.h"

/*! Base class for interfacing GUIs and modules */
class Interface : public QThread
{
    Q_OBJECT

public:
    Interface(GUI *pGUI);

    void sendGpuListRequest();
    void sendCpuRequest();
    void sendParametersRequest();   
    void sendQuitRequest();
    void close();
    void sendGpuID(int gpuID);
    void sendGpuIDs(QVector<int> gpuIDs);

protected:
    GUI *module;

    yarp::os::Port inputPort;
    yarp::os::Port outputPort;
    yarp::os::Bottle receivedBottle;

    virtual void processPortData();
    virtual void processGpuList();
    virtual void printMessage();

private:
    bool running;
    QString inputPortName;
    QString outputPortName;

    void run();

public slots:
    void sendAbortRequest();
    void sendStartRequest();
    void sendStopRequest();
    void sendSimulationMode(int simulationMode);

signals:
    void gpuListReceived(QVector<QStringList> gpuList);
    void progressReceived(int progress);
    void statusReceived(int statusID);
};

#endif//INTERFACE_H
