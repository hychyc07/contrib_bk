//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse																																											//
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

#ifndef ERA_H
#define ERA_H

#define MAX_ENGINE_INSTANCES 20 //!< maximum number of module instances
#define MAX_TIMEOUT_ATTEMPTS 100 //!< maximum number of attempts to ping an module

#include <QtNetwork/QHostInfo>
#include <QString>
#include <QVector>
#include <QProcess>
#include <QThread>
#include <omp.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "aquila-utility.h"

using namespace aquila;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class PortListener : public QThread
{
    Q_OBJECT

public:
    PortListener(Port *pPort, int portID);

private:
    Port *port;
    Bottle command;
    int id;

    bool running;
    void run();

public slots:
    void stop();

signals:
    void messageSet(QString message);
    void poolUnitActivationRequested(QString inputFrom, int unitIdx);
};

struct WeightsType
{
    int i;
    int j;
    float weight;
};
Q_DECLARE_TYPEINFO( WeightsType, Q_MOVABLE_TYPE );

struct ConnectionMatrixType
{
    int connectMaps[2];
    QVector<WeightsType> weights;
};
Q_DECLARE_TYPEINFO( ConnectionMatrixType, Q_MOVABLE_TYPE );

struct PoolActivityType
{
    float activity;
    float netInput;
    float extInput;
    QString label;
    int X;
    int Y;
};
Q_DECLARE_TYPEINFO( PoolActivityType, Q_MOVABLE_TYPE );

struct PoolType
{
//    int size;
    QString kind;
    QString input;
    QVector<PoolActivityType> state;
};
Q_DECLARE_TYPEINFO( PoolType, Q_MOVABLE_TYPE );

struct ObjectType
{
    int X;
    int Y;
};

class ERA : public QThread
{
    Q_OBJECT

public:
    ERA();
    GPU *gpu;

    int numSOMs;
    bool terminalMode;

    void clean();
    bool openPorts(QString portPrefix);
    bool spawnSOMs(QString portPrefix);

    void connectPorts();
    void disconnectPorts();

    void setSimulationMode(bool simulation);
    void setSpeechInput(const Bottle& command);
    void setFieldInput(const Bottle& command);
    void setWinner(int somID, int winner);

    bool getSimulationMode();
    int getIacWinner(QString inputFrom);

private:
    QVector<PortListener*> somPortListener;
    QString hostName;
    QString hostServerName;
    QVector<QString> server;
    QProcess *process;
    QVector<Port*> somInputPort;
    QVector<Port*> somOutputPort;
    QVector<QString> somInputPortName;
    QVector<QString> somOutputPortName;
    QVector<int> moduleID;

    int simulationMode;

    //IAC parameters
    float internalBias;
    float externalBias;
    float iacMax;
    float iacMin;
    float decay;
    float rest;
    float landa;
    float inhibition;
    float learningScalar;
    float colourSpectrum[36]; //NOTE: this could be improved to avoid border issues causing big changes
    bool initialised;
    bool running;

    QVector<QString> dictionary;
    QVector<ConnectionMatrixType> connectionMatrix;
    QVector<PoolType> pool;
    QVector<ObjectType> objects;

    struct Image
    {
        BufferedPort<ImageOf<PixelRgb> > inputPort;
        BufferedPort<ImageOf<PixelRgb> > outputPort;
        QString inputPortName;
        QString outputPortName;
    }; Image leftCam;

    void run();
    void init();
    void iacStep();
    void setupPools();
    void sendData();
    void speechOutput();
    void connectSOM(int somInstanceID);
    void disconnectSOM(int somInstanceID);
    void findFreeModuleIDs(QString portPrefix);
    void setupConnectionMatrix(int hub, int maxIdx);
    void setSomParameters(int somId, int numInputs, int numOutputs, bool gpu);
    void sendToSom(int somId, float *dataToSend, int size, int learningOn);
    void getFovia(ImageOf<PixelRgb> *image, int xCenter, int yCenter, int foviaSize);
    void RGBtoHSV(float r, float g, float b, float *h, float *s, float *v);
    void partiallyPreTrainSom(int somId, int inputSize, int itterations);
    void markImage(ImageOf<PixelRgb> *image, int target_x, int target_y, int r, int g, int b);

public slots:
    void stop();
    void activatePoolUnit(QString inputFrom, int unitIdx);

signals:
    void quitting();
    void updateGraphRequested(int size, double *activity, double *extInput);
    void messageSet(QString message);
    void started(int id);
    void stopped(int id);
};

#endif//ERA_H
