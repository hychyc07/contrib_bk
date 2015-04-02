//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse																																			//
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

#ifndef LIB_AQUILA_NNET_ESN_H
#define LIB_AQUILA_NNET_ESN_H

//Tony: I would make these variables and initialse them from the configuration file, then you can dynamically change your ESNs, Martin
#define ESN_INPUT_SIZE 14	//the number of inputs to the ESN - 14 sensors for the emotive epoch headset
#define ESN_OUTPUT_SIZE 2	//the number of outputs to the ESN - ?
#define ESN_SPARCE 20		//the probability (%) of each connection
#define ESN_STRENGTH 0.2	//the range +- of each connection (actual strength is random within this range)
#define FEEDBACK 0          //switch feedback on (1) or off (0)
#define LEARNING_RATE 0.01	//the learning rate for percetron training

#include <QThread>
#include <cuda.h>
#include <cuda_runtime.h>
#include "aquila-utility.h"

using namespace std;
namespace aquila
{
class ESN : public QThread
{
    Q_OBJECT

public:
    ESN();
    ~ESN();

    GPU *gpu;
    Math *math;

    bool terminalMode;

private:
    size_t inputActivitySize;
    size_t activitySize;
    size_t outputActivitySize;
    size_t inputWeightsSize;
    size_t weightsSize;
    size_t outputWeightsSize;

    int esnSize;
    int inputSource;
    int stepCounter;
    bool running;

    float *weights;
    float *activity;
    float *newActivity;
    float *inputWeights;
    float *inputActivity;
    float *newInputActivity;
    float *outputWeights;
    float *outputActivity;
    float *newOutputActivity;
    float *PERCoutputWeights;
    float *PERCinputWeights;
    float *PERCoutputTarget;
    float *PERCinputTarget;

    void run();
    void initialise();
    void calculateActivity(float *activity, int N, float *weights, float *newActivity, int noElements);
    void PERCweightsUpdate(float *activity, float *PERCactivity, float *PERCweights, float *target, float learningRate, int noPercUnits, int size);
    void finishStep (float *newActivity, float *activity, int noElements, float *newOutActivity, float *outActivity, int noOutElements, float *newInputActivity, float *inputActivity, int noInputElements, 	int inputClamp, int outputClamp);

public:
    void stop();

    void setSize(int size);
    void setInputSource(int source);

    int getSize();
    int getInputSource();

signals:
    void messageSet(QString message);
    void activitySet(QVector<float> activity);
    void started(int id);
    void stopped(int id);
};
}

#endif//LIB_AQUILA_NNET_ESN_H
