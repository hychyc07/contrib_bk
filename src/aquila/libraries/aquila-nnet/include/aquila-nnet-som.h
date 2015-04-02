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

#ifndef LIB_AQUILA_NNET_SOM_H
#define LIB_AQUILA_NNET_SOM_H

#define MAX_GPU_DEVICES 4

#include <QVector>
#include <cuda.h>
#include <iostream>
#include <cuda_runtime.h>
#include <yarp/os/all.h>
#include <QThread>
#include <QStringList>
#include "aquila-utility.h"

using namespace yarp::os;
using namespace std;
namespace aquila
{
class SelfOrganisingMap : public QThread
{
    Q_OBJECT

public:
    SelfOrganisingMap();
    ~SelfOrganisingMap();
    GPU *gpu;
    Math *math;

private:
    QString trainingFileName;
    QString mapFileName;

    //host memory pointers
	int *winner_h;
	float *input_h;
	float *weight_h;
	float *output_h;
	float *average_h;
	float *distance_h;

    //device memory pointers
	int *winner_d;
	float *input_d;
	float *weight_d;
	float *output_d;
	float *average_d;
	float *distance_d;

	int numThreads;
	int numBlocks;
    int maxThreads;
	int numInputs;
	int numSamples;
	int numOutputs;
	int numSubiterations;
	int iterationPause;
	int numIterations;
	int neighbourhoodSize;
	int currentNeighbourhoodSize;
    int randomSequenceId;
    int liveCurrentIteration;
	float trainingTime;
	float highestDataPoint;
	float lowestDataPoint;
	float initLearningRate;
	float sigma;
    bool running;
    bool visualiseProgress;
    bool memoryAlocated;

    void run();
	void setNumBlocks();
	void allocateMemory();
	void updateProgress();
	void findBestMatch(float *outputs,  int *winner, int numOutputs);
	void propogateInput(float *inputs, float *weights, float *outputs, int numInputs, int sequenceId, int numOutputs);
	void updateWeights(float *inputs, float *weights, int *winner, float sigma, int numInputs, int sequenceId, int numOutputs, int neighbourhoodSize, float initLearningRate,int numIterations, int currentIteration);

public:
	void loadTrainingSet(Bottle portData = 0);
    void randomiseWeights();
    void saveMap();
    void singleStep(int learningOn);
    void setLiveInput(Bottle portData);
    void setMaxThreads(int threads);
	void setNumInputs(int inputs);
	void setNumSamples(int samples);
	void setNumOutputs(int outputs);
	void setNumSubiterations(int subIterations);
	void setInitLearningRate(float learningRate);
	void setSigma(float sigma);
	void setVisualiseProgress(bool visualise);
	void setIterationPause(int value);
	void setNeighbourhoodSize(int value);
	void setNumIterations(int value);
    void setTrainingFile(QString fileName);
    void setMapFile(QString fileName);
    void stop();

    int getMaxThreads();
	int getNumInputs();
	int getNumSamples();
	int getNumOutputs();
    int getNeighbourhoodSize();
	int getNumSubiterations();
	int getIterationPause();
	float getInitLearningRate();
	float getSigma();
	float getTrainingTime();
	float getLowestDataPoint();
	float getHighestDataPoint();
    bool getVisualiseProgress();
    QVector<float> getCurrentMap();
    QVector<float> getDataPoints();
    QString getTrainingFile();
    QString getMapFile();

public slots:
    void gpuModeChanged(bool mode);
    void freeMemory(bool gpu);

signals:
    void trainingStarted();
    void trainingStopped(QVector<float> map, QVector<float> limits, float time);
    void messageSet(QString message);
    void mapSet(QVector<float> map);
    void numInputsSet(int numInputs);
    void progressSet(int progress);
    void winnerSet(int winnerID);
};
}
#endif//LIB_AQUILA_NNET_SOM_H
