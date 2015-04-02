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

#ifndef SOM_INTERFACE_H
#define SOM_INTERFACE_H

#include "interface.h"

/*! Interface between SOM GUI and SOM module */
class SOMInterface : public Interface
{
    Q_OBJECT

public:
    SOMInterface(GUI *pGUI);

    void sendSubIterations(int numSubIterations);
    void sendNumOutputs(int numOutputs);
    void sendIterationsPause(int iterationPause);
    void sendLearningRate(double learningRate);
    void sendTrainingRequest(QString fileName);
    void sendTrainingRequest(int numSamples, int numInputs, QVector<double> trainingData);
    void sendSaveRequest(QString fileName);

protected:
    void processPortData();

private:
    void processMap();

public slots:
    void sendParameters(float learningRate, int numSubIterations, int numOutputs, int iterationPause);
    void sendVisualiseLearning(int visualiseLearning);

signals:
    void mapReceived(QVector<float> map);
    void numInputsReceived(int numInputs);
    void trainingTimeReceived(float time);
    void dataPointLimitsReceived(float low, float hi);
    void parametersReceived(float learningRate, int numSubIterations, int numOutputs, int iterationPause);
};

#endif//SOM_INTERFACE_H
