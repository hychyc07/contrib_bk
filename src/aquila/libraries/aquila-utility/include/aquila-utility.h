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

#ifndef AQUILA_UTILITY_LIB_H
#define AQUILA_UTILITY_LIB_H

#include <QVector>
#include <cuda.h>
#include <iostream>
#include <cuda_runtime.h>
#include <yarp/os/all.h>
#include <QObject>
#include <QStringList>

using namespace yarp::os;
using namespace std;

namespace aquila
{
class GPU : public QObject
{
        Q_OBJECT
public:
    GPU();
    ~GPU();

    int numDevices;
    int numRequestedDevices;
    int current;
    bool active;
    bool peerToPeerActive;
    QVector<int> device;
    QVector<cudaDeviceProp> property;

    void setGPUMode(bool gpuActive);
    void setDevices(QVector<int> deviceID);
    bool setDevice(int id);
    int getDevice();

    QVector<QStringList> getDeviceList();

signals:
    void gpuModeSet(bool gpuActive);
};

class Messenger : public QObject
{
        Q_OBJECT
public:
    Messenger(Port &targetPort);
    ~Messenger();

    Port port;

public slots:
    void sendMessage(QString message);
    void sendProgress(int progress);
    void sendGpuDeviceList(QVector<QStringList> deviceList);
    void sendStatus(int status);
};

class Math : public QObject
{
        Q_OBJECT
public:
    Math();
    ~Math();

private:
    int seed;

public:
    void setSeed(int value=0);
    int getRandomIntegerUpTo(int limit);
    int nextPow2(int x);
    float getRandomFloatInRange(float min, float max);
    float scaleRange(float in, float oldMin, float oldMax, float newMin, float newMax);

    int getSeed();
};
}

#endif//AQUILA_UTILITY_LIB_H
