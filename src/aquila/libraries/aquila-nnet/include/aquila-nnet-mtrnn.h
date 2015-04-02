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

#ifndef LIB_AQUILA_NNET_MTRNN_H
#define LIB_AQUILA_NNET_MTRNN_H

#define MAX_GPU_DEVICES 4
#define MAX_SEQUENCES 100

#include <QThread>
#include <cuda.h>
#include <cuda_runtime.h>
#include "aquila-utility.h"

using namespace std;
namespace aquila
{
class MTRNN : public QThread
{
    Q_OBJECT

public:
    MTRNN();
    ~MTRNN();

    GPU *gpu;
    Math *math;

    bool terminalMode;

private:
    QString trainingFileName;
    QString networkFileName;
    Bottle trainingData;

    bool running;

    //host memory pointers
    int *deltaT_h;
    float *input_h;
    float *activity_h;
    float *error_h;
    float *zeroError_h;
    float *delta_h;
    float *previousDelta_h;
    float *individualError_h;
    float *mse_h;
    float *previousDeltaWeight_h;
    float *deltaWeight_h[MAX_GPU_DEVICES];
    float *weight_h;
    float *previousPotential_h;
    float *potential_h;
    float *zeroWeight_h;
    float *zeroNeuron_h;

    //device memory pointers
    int *deltaT_d[MAX_GPU_DEVICES];
    float *mse_d[MAX_GPU_DEVICES];
    float *activity_d[MAX_GPU_DEVICES];
    float *input_d[MAX_GPU_DEVICES];
    float *error_d[MAX_GPU_DEVICES];
    float *delta_d[MAX_GPU_DEVICES];
    float *buffer_d[MAX_GPU_DEVICES];
    float *previousDelta_d[MAX_GPU_DEVICES];
    float *individualError_d[MAX_GPU_DEVICES];
    float *deltaWeight_d[MAX_GPU_DEVICES];
    float *previousDeltaWeight_d[MAX_GPU_DEVICES];
    float *weight_d[MAX_GPU_DEVICES];
    float *potential_d[MAX_GPU_DEVICES];
    float *previousPotential_d[MAX_GPU_DEVICES];

    int feedbackInterval;
    int showProgress;
    int maxThreads;
    int ioDeltaT;
    int fastDeltaT;
    int slowDeltaT;
    int iteration;
    int maxIterations;
    int numSequences;
    int sequenceWidth;
    int totalSequenceSteps;
    int maxSequenceSteps;
    int numIONeurons;
    int numFastNeurons;
    int numSlowNeurons;
    int numControlNeurons;
    int numLinguisticNeurons;
    int numVisionNeurons;
    int numActionNeurons;
    int numNeurons;
    int numWeights;
    int *sequenceSteps;
    int *sequenceOffsets;
    float minValue;
    float maxValue;
    float *errors;

    //grid block
    int neuronThreads;
    int neuronBlocks;
    int seqNeuronBlocks;
    int numIoBlocks;
    int threads2D;
    int numFThreads;
    int numHBlocks;
    int numFBlocks;
    int numEThreads;
    int numEBlocks;
    int smemSize;
    int smemESize;
    dim3 dim2DBlock;
    dim3 dim2DGrid;
    dim3 dim2DWBlock;
    dim3 dim2DWGrid;

    float initWeightRange;
    float threshold;
    float learningRate;
    float momentum;

    void run();
    void allocateMemory();
    void copyMemoryToDevice();

    void forwardPass(int step, int sequenceOffset, float *activity, float *input, float *weight, float *previousPotential, float *potential, float *error, int *deltaT, int numNeurons, int numIONeurons);
    void backwardPass(int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *previousDelta, float *error, float *weight, float *deltaWeight, float *mse, int *deltaT);
    void updateWeights(float learningRate, float momentum, float *weight, float *deltaWeight, float *previousDeltaWeight, int numWeights);
    void setInitStates(float initState, float *activity, float *zeroActivity, int numNeurons, int numIONeurons, int numFastNeurons);
    void resetParameters(int numNeurons, int maxsequenceSteps, float *delta, float *previousDelta, float *potential, float *previousPotential, float *error, float *zeroNeuron, float *zeroError);
    void resetDeltaWeights(int numWeights, float *deltaWeight, float *zeroWeight);

public:
    void stop();
    void initialise();
    void deinitialise();
    void saveNetwork();
    void testNetwork();
    void randomiseWeights();
    bool loadTrainingData();

    void setTrainingData(Bottle data);

    void setTrainingFile(QString fileName);
    void setNetworkFile(QString fileName);
    void setShowProgress(bool show);
    void setMaxThreads(int threads);
    void setMaxIterations(int iterations);
    void setIODeltaT(int value);
    void setFastDeltaT(int value);
    void setSlowDeltaT(int value);
    void setNumFastNeurons(int fastNeurons);
    void setNumSlowNeurons(int slowNeurons);
    void setInitWeightRange(float value);
    void setThreshold(float value);
    void setLearningRate(float value);
    void setMomentum(float value);
    void setDeltaT();
    void setGridBlock();
    void setFeedbackInterval(int interval);

    int getProgress();
    int getMaxIterations();
    int getNumFastNeurons();
    int getNumSlowNeurons();
    int getIODeltaT();
    int getFastDeltaT();
    int getSlowDeltaT();
    int getDebuggingLevel();
    int getFeedbackInterval();
    float getLearningRate();
    float getMomentum();
    float getWeightRange();
    float getThreshold();
    QString getNetworkFile();
    QString getTrainingFile();
    QVector<float> getWeights();

signals:
    void trainingStarted();
    void trainingStopped(QVector<float> errors);
    void messageSet(QString message);
    void progressSet(int progress);
    void errorSet(int iteration, float error);
};
}

#endif//LIB_AQUILA_NNET_MTRNN_H
