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

#include <fstream>
#include <math.h>
#include <omp.h>
#include "aquila-nnet-som.h"
#include "somKernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

namespace aquila
{
/*!
 * \brief Constructor.
 * \note This constructor saves a pointer to the Interface object,
 * \note looks for GPU devices and sets default execution mode.
 */
SelfOrganisingMap::SelfOrganisingMap()
{
    gpu = new GPU();
    math = new Math();
    QObject::connect(gpu, SIGNAL(gpuModeSet(bool)), this, SLOT(gpuModeChanged(bool)));

    liveCurrentIteration = 0;
    memoryAlocated = false;

    winner_h = NULL;
    input_h = NULL;
    weight_h = NULL;
    output_h = NULL;
    average_h = NULL;
    distance_h = NULL;
    winner_d = NULL;
    input_d = NULL;
    weight_d = NULL;
    output_d = NULL;
    average_d = NULL;
    distance_d = NULL;
}

/*!
 * \brief Destructor.
 */
SelfOrganisingMap::~SelfOrganisingMap()
{
}

/*!
 * \brief Thread loop.
 * \note This loop runs the self-organising map training.
 */
void SelfOrganisingMap::run()
{
	cudaEvent_t startGPUTime;
	cudaEvent_t stopGPUTime;
	double startCPUTime;
	double endCPUTime;
	int currentIteration = 0;
    running  = true;

    emit trainingStarted();

    if(gpu->active)
    {
        //set the GPU for context
        gpu->setDevice(gpu->current);

		//start timer
		gpuAssert(cudaEventCreate(&startGPUTime));  
		gpuAssert(cudaEventCreate(&stopGPUTime));
		gpuAssert(cudaEventRecord(startGPUTime, 0));

        while(running && currentNeighbourhoodSize>0)
        {
            //pick a random input sequence used for the current training step
            randomSequenceId = math->getRandomIntegerUpTo(numSamples);

			//propagates the input
            propogateInputOnDevice(numBlocks, numThreads, input_d, weight_d, output_d, numInputs, randomSequenceId, numOutputs);

			//finds the best matching unit
            findBestMatchOnDevice(numBlocks, numThreads, output_d, winner_d, numOutputs);

			//modify the weights
            updateWeightsOnDevice(numBlocks, numThreads, input_d, weight_d, winner_d, sigma, numInputs, randomSequenceId, numOutputs, currentNeighbourhoodSize, initLearningRate, numIterations, currentIteration);

			//adjust plasticity
			currentIteration ++;
            if(currentIteration==numSubiterations && currentNeighbourhoodSize>0)
			{
				currentIteration = 0;
				currentNeighbourhoodSize--;
				updateProgress();
			}

			//visualise progress
			if(visualiseProgress)
			{
                gpuAssert(cudaMemcpy(weight_h, weight_d, (numInputs * numOutputs) * sizeof(float), cudaMemcpyDeviceToHost));
                emit mapSet(getCurrentMap());
				yarp::os::Time::delay((float)iterationPause/1000.0);
			}
		}

		//stop timer
		gpuAssert(cudaEventRecord(stopGPUTime, 0));
		gpuAssert(cudaEventSynchronize(stopGPUTime));
		gpuAssert(cudaEventElapsedTime(&trainingTime, startGPUTime, stopGPUTime));
		gpuAssert(cudaEventDestroy(startGPUTime));
		gpuAssert(cudaEventDestroy(stopGPUTime));
		gpuAssert(cudaMemcpy(weight_h, weight_d, (numInputs * numOutputs) * sizeof(float), cudaMemcpyDeviceToHost));
	}
    else
	{
		//start timer
		startCPUTime = Time::now();

        while(running && currentNeighbourhoodSize>0)
		{
			//pick a random input sequence used for the current training step
            randomSequenceId = math->getRandomIntegerUpTo(numSamples);

			//propagates the input
			propogateInput(input_h, weight_h, output_h, numInputs, randomSequenceId, numOutputs);

			//finds the best matching unit
            findBestMatch(output_h, winner_h, numOutputs);

			//modify the weights
            updateWeights(input_h, weight_h, winner_h, sigma, numInputs, randomSequenceId, numOutputs, currentNeighbourhoodSize, initLearningRate, numIterations, currentIteration);

			//adjust plasticity
			currentIteration ++;
            if(currentIteration==numSubiterations && currentNeighbourhoodSize>0)
			{
				currentIteration = 0;
				currentNeighbourhoodSize--;
				updateProgress();
			}

			//visualise progress
			if(visualiseProgress)
            {
                emit mapSet(getCurrentMap());
				yarp::os::Time::delay((float)iterationPause/1000.0);
            }
		}

		//stop timer
		endCPUTime = Time::now()-startCPUTime;
        trainingTime = (float)endCPUTime*1000;
	}

    if(!mapFileName.isEmpty())
    {
        saveMap();
    }

    emit trainingStopped(getCurrentMap(), getDataPoints(), trainingTime);
}

/*!
 * \brief Stops the thread.
 */
void SelfOrganisingMap::stop()
{
    running  = false;
}

/*!
 * \brief Updates progress.
 */
void SelfOrganisingMap::updateProgress()
{
	int progress = (int)(100-((float)(currentNeighbourhoodSize)/(float)neighbourhoodSize)*100);
    emit progressSet(progress);
}

/*!
 * \brief accepts live training data and runs 1 cycle of the SOM with that data.
 * \param[in] portData - training data sent via port
 */
void SelfOrganisingMap::setLiveInput(Bottle portData)//use QVectors!
{
    Bottle bSample;

    bool dataFromPort = false;
    bool success = false;

    if(portData.size()!=0) //loading data from port
    {
        dataFromPort = true;
        success = true;
    }

   if(success)
    {
        bSample = portData;
        setNumSamples(1);
        setNumInputs(bSample.get(2).asInt());

        //allocate new memory
        if(!memoryAlocated)
        {
            allocateMemory();
        }

         //read in the data
        for(int j=0; j<numInputs; j++)
        {
            input_h[j] = (float)bSample.get(6+j).asDouble();

            if(input_h[j] > highestDataPoint)
            {
                highestDataPoint = input_h[j];
            }

            if(input_h[j] < lowestDataPoint)
            {
                lowestDataPoint = input_h[j];
            }
        }

        if(gpu->active)
        {
            gpuAssert(cudaMemcpy(input_d, input_h, (numInputs * numSamples) * sizeof(float), cudaMemcpyHostToDevice));
        }
    }
}

/*!
 * \brief executes a single step or cycle of the SOM
 * \note This loop runs the self-organising map training for live data.
 */
void SelfOrganisingMap::singleStep(int learningOn)
{
    if(gpu->active)
    {
        //set the GPU for context
        gpu->setDevice(gpu->current);//should not need this!

        //propagates the input
        propogateInputOnDevice(numBlocks, numThreads, input_d, weight_d, output_d, numInputs, 0, numOutputs);

        //finds the best matching unit
        findBestMatchOnDevice(numBlocks, numThreads, output_d, winner_d, numOutputs);

        if(learningOn == 1)
        {
            //modify the weights
            updateWeightsOnDevice(numBlocks, numThreads, input_d, weight_d, winner_d, sigma, numInputs, 0, numOutputs, currentNeighbourhoodSize, initLearningRate, numIterations, liveCurrentIteration);

            //adjust plasticity if neighbourhood size is > 1
            if(currentNeighbourhoodSize > 1)
            {
                liveCurrentIteration ++;
                if(liveCurrentIteration%numSubiterations == 0)
                {
                    //liveCurrentIteration = 0;
                    currentNeighbourhoodSize--;
                    updateProgress();
                }
            }
        }
    }
    else
    {
        //propagates the input
        propogateInput(input_h, weight_h, output_h, numInputs, 0, numOutputs);

        //finds the best matching unit
        findBestMatch(output_h, winner_h, numOutputs);

        if(learningOn == 1)
        {
            //modify the weights
            updateWeights(input_h, weight_h, winner_h, sigma, numInputs, 0, numOutputs, currentNeighbourhoodSize, initLearningRate, numIterations, liveCurrentIteration);

            //adjust plasticity if neighbourhood size is > 1
            if(currentNeighbourhoodSize > 1)
            {
                liveCurrentIteration ++;
                if(liveCurrentIteration%numSubiterations == 0)
                {
                    //liveCurrentIteration = 0;
                    currentNeighbourhoodSize--;
                    qDebug("neighbourhood size reduced to %i", currentNeighbourhoodSize);
                    updateProgress();
                }
            }
        }
    }
    if(gpu->active)
    {
        gpuAssert(cudaMemcpy(winner_h, winner_d, numOutputs * sizeof(int), cudaMemcpyDeviceToHost));
    }
    emit winnerSet(winner_h[0]);
}

/*!
 * \brief Loads training data.
 * \note This function can load the training data either from a local file or remotely via port
 * \note in which case portData bottle is passed as an input parameter.
 * \param[in] portData - training data sent via port
 */
void SelfOrganisingMap::loadTrainingSet(Bottle portData)
{
	ResourceFinder rf;
    Property config;
	Bottle bSample;
	Bottle *pS;

	highestDataPoint = -100000;
	lowestDataPoint  = 100000;

	bool dataFromPort = false;
	bool success = false;

	if(portData.size()!=0) //loading data from port
	{
		dataFromPort = true;
		success = true;
	}
	else //loading data from file
	{
        if(config.fromConfigFile(rf.findFile(trainingFileName.toStdString().c_str())))
		{
			success = true;
		}
		else
		{
			success = false;
            printf(" - ERROR: %s was not found\n",trainingFileName.toStdString().c_str());
		}
	}

	if(success)
	{
		if(!dataFromPort)
		{
			bSample = config.findGroup("SAMPLES");

            if(bSample.isNull())
			{
                printf(" - ERROR: [SAMPLES] group is missing in %s\n",trainingFileName.toStdString().c_str());
			}

			setNumSamples(bSample.size()-1);
			setNumInputs(bSample.get(1).asList()->size());
		}
		else
		{
			bSample = portData;
			setNumSamples(bSample.get(2).asInt());
			setNumInputs(bSample.get(3).asInt());
		}

		//send number of inputs Aquila
        emit numInputsSet(numInputs);

		//free memory that was previously allocated
        freeMemory(gpu->active);

		//allocate new memory
        allocateMemory();

		//get the training data
		for (int i=0; i<numSamples; i++)
		{
            if(!dataFromPort)
            {
                pS = bSample.get(i+1).asList();
            }

            for(int j=0; j<numInputs; j++)
			{
                if(!dataFromPort)
                {
                    input_h[(i*numInputs)+j] = (float)pS->get(j).asDouble();
                }
                else
                {
                    input_h[(i*numInputs)+j] = (float)bSample.get(4+(i+(j*numSamples))).asDouble();
                }

                if(input_h[(i*numInputs)+j] > highestDataPoint)
                {
                    highestDataPoint = input_h[(i*numInputs)+j];
                }

                if(input_h[(i*numInputs)+j] < lowestDataPoint)
                {
                    lowestDataPoint = input_h[(i*numInputs)+j];
                }
			}
		}

        //scale all the input values between 0 and 1 based on the maximum and minimum values that were detected in the file
        for(int i=0; i<numSamples; i++)
        {
            for(int j=0; j<numInputs; j++)
            {
                input_h[(i*numInputs)+j] = math->scaleRange(input_h[(i*numInputs)+j], lowestDataPoint, highestDataPoint, 0.0f, 1.0f);
            }
        }

        if(gpu->active)
		{
			gpuAssert(cudaMemcpy(input_d, input_h, (numInputs * numSamples) * sizeof(float), cudaMemcpyHostToDevice));
		}

		setNumOutputs(numOutputs);
    }
}

/*!
 * \brief Saves trained self-organising map.
 */
void SelfOrganisingMap::saveMap()
{
    char buf[100];
    ofstream myfile;
    myfile.open(mapFileName.toStdString().c_str());

    if(myfile.is_open())
    {
        myfile<<"[INFORMATION]\n";

        if(gpu->active)
        {
            myfile<<"GPU "<<gpu->current<<"\n";
        }
        else
        {
            myfile<<"CPU \n";
        }

        myfile<<"trainingTtime "<<trainingTime<<" ms"<<"\n";
        myfile<<"learningRate "<<initLearningRate<<"\n";
        myfile<<"subIterations "<<numSubiterations<<"\n";
        myfile<<"outputs "<<numOutputs<<"\n";
        myfile<<"highestSourceDataValue "<<highestDataPoint<<"\n";
        myfile<<"lowestSourceDataValue "<<lowestDataPoint<<"\n";
        myfile<<"[WEIGHTS]\n";

        //write the weights of the currently loaded self-organising map
        for(int j=0;j<numOutputs;j++)
        {
            for(int i=0;i<numInputs;i++)
            {
                sprintf(buf,"%.6f", weight_h[(i*numOutputs)+j]);
                myfile<<buf;

                if(i<numInputs-1)
                {
                    myfile<<"\t";
                }
            }
            myfile<<"\n";
        }
        myfile.close();
        printf(" - the self-organising map was saved to: %s\n", mapFileName.toStdString().c_str());
        mapFileName.clear();
    }
    else
    {
        printf(" - ERROR: unable to open %s\n", mapFileName.toStdString().c_str());
    }
}

/*!
 * \brief Allocates memory.
 */
void SelfOrganisingMap::allocateMemory()
{
    if(gpu->active)
	{
        gpuAssert(cudaMallocHost((void**)&input_h, (numInputs * numSamples)  * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&weight_h, (numInputs * numOutputs) * sizeof(float)));
		gpuAssert(cudaMallocHost((void**)&average_h, sizeof(float)));
		gpuAssert(cudaMallocHost((void**)&distance_h, numOutputs * sizeof(float)));
		gpuAssert(cudaMallocHost((void**)&output_h, numOutputs * sizeof(float)));
		gpuAssert(cudaMallocHost((void**)&winner_h, numOutputs * sizeof(int)));	
		gpuAssert(cudaMalloc((void **)&average_d, sizeof(float)));
		gpuAssert(cudaMalloc((void **)&input_d, (numInputs  * numSamples) * sizeof(float)));
		gpuAssert(cudaMalloc((void **)&distance_d, numOutputs * sizeof(float)));
		gpuAssert(cudaMalloc((void **)&output_d, numOutputs * sizeof(float)));
		gpuAssert(cudaMalloc((void **)&weight_d, (numInputs  * numOutputs) * sizeof(float)));
		gpuAssert(cudaMalloc((void **)&winner_d, numOutputs * sizeof(int)));
	}
	else
	{
        input_h = (float*)malloc((numInputs * numSamples)  * sizeof(float));
        weight_h = (float*)malloc((numInputs * numOutputs) * sizeof(float));
		average_h = (float*)malloc(sizeof(float));
		distance_h = (float*)malloc(numOutputs * sizeof(float));
		output_h = (float*)malloc(numOutputs * sizeof(float));
		winner_h = (int*)malloc(numOutputs * sizeof(int));	
	}
    memoryAlocated = true;
}

/*!
 * \brief Deallocates memory.
 */
void SelfOrganisingMap::freeMemory(bool gpu)
{
    if(gpu)
	{
        if(winner_h)   gpuAssert(cudaFreeHost(winner_h));
        if(input_h)    gpuAssert(cudaFreeHost(input_h));
        if(weight_h)   gpuAssert(cudaFreeHost(weight_h));
        if(output_h)   gpuAssert(cudaFreeHost(output_h));
        if(average_h)  gpuAssert(cudaFreeHost(average_h));
        if(distance_h) gpuAssert(cudaFreeHost(distance_h));
        if(winner_d)   gpuAssert(cudaFree(winner_d));
        if(input_d)    gpuAssert(cudaFree(input_d));
        if(weight_d)   gpuAssert(cudaFree(weight_d));
        if(output_d)   gpuAssert(cudaFree(output_d));
        if(average_d)  gpuAssert(cudaFree(average_d));
        if(distance_d) gpuAssert(cudaFree(distance_d));
	}
	else
    {
        if(winner_h)   free(winner_h);
        if(input_h)    free(input_h);
        if(weight_h)   free(weight_h);
        if(output_h)   free(output_h);
        if(average_h)  free(average_h);
        if(distance_h) free(distance_h);
	}

	winner_h = NULL;
	input_h = NULL;
	weight_h = NULL;
	output_h = NULL;
	average_h = NULL;
	distance_h = NULL;
	winner_d = NULL;
	input_d = NULL;
	weight_d = NULL;
	output_d = NULL;
	average_d = NULL;
    distance_d = NULL;
}

/*!
 * \brief Deallocates memory on GPU mode change.
 */
void SelfOrganisingMap::gpuModeChanged(bool mode)
{
    freeMemory(!mode);
}

/*!
 * \brief Randomises self-organising map's weights.
 */
void SelfOrganisingMap::randomiseWeights()
{
    for(int i=0; i<numInputs * numOutputs; i++)
    {
        weight_h[i] = math->getRandomFloatInRange(0.0f, 1.0f);
    }

    if(gpu->active)
    {
        gpuAssert(cudaMemcpy(weight_d, weight_h, (numInputs*numOutputs)*sizeof(float), cudaMemcpyHostToDevice));
    }
}

/*!
 * \brief Sets maximum number of threads per blocks.
 * \param[in] value - maximum number of threds
 */
void SelfOrganisingMap::setMaxThreads(int value)
{
    maxThreads = value;
}

/*!
 * \brief Sets the number of inputs of self-organising map.
 * \param[in] value - number of inputs
 */
void SelfOrganisingMap::setNumInputs(int value)
{
    numInputs = value;
}

/*!
 * \brief Sets the total number of samples present in the training data.
 * \param[in] value - number of samples
 */
void SelfOrganisingMap::setNumSamples(int value)
{
    numSamples = value;
}

/*!
 * \brief Sets the number of outputs of self-organising map.
 * \note This function sets the outputs and automatically updates
 * \note the neighbourhood size and number of blocks.
 * \param[in] value - number of outputs
 */
void SelfOrganisingMap::setNumOutputs(int value)
{
    numOutputs = value;
    setNeighbourhoodSize((int)sqrt((float)numOutputs));
    setNumBlocks();
}

/*!
 * \brief Sets the required number of blocks give selected number of threads and outputs.
 */
void SelfOrganisingMap::setNumBlocks()
{
    numThreads = maxThreads;
    numBlocks = numOutputs/numThreads+(numOutputs%numThreads==0 ? 0:1);
}

/*!
 * \brief Sets self-organising map's neighbourhood size.
 * \param[in] value - neighbourhood size
 */
void SelfOrganisingMap::setNeighbourhoodSize(int value)
{
    neighbourhoodSize = value;
    currentNeighbourhoodSize = value;
    setNumIterations(numSubiterations*neighbourhoodSize);
}

/*!
 * \brief Sets the number of iterations used during the training.
 * \param[in] value - number of iterations
 */
void SelfOrganisingMap::setNumIterations(int value)
{
    numIterations = value;
}

/*!
 * \brief Sets the number of sub-iterations used during the training.
 * \param[in] value - number of sub-iterations
 */
void SelfOrganisingMap::setNumSubiterations(int value)
{
    numSubiterations = value;
}

/*!
 * \brief Sets the initial learning rate used during the training.
 * \note This learning rate is then slowly lowered during the training.
 * \param[in] value - initial learning rate
 */
void SelfOrganisingMap::setInitLearningRate(float value)
{
    initLearningRate = value;
}

/*!
 * \brief Sets the sigma value used during the training.
 * \note Sigma is set to zero by default. This function is experimental.
 * \param[in] value - sigma
 */
void SelfOrganisingMap::setSigma(float value)
{
    sigma = value;
}

/*!
 * \brief Sets the progress visualisation on/off.
 * \note If the visualisation is on then this module sends the weights
 * \note of the self-organising map to the output port, which is typically
 * \note read and visualised by Aquila,.
 * \param[in] visualise - progress visualisation (on/off)
 */
void SelfOrganisingMap::setVisualiseProgress(bool visualise)
{
    visualiseProgress = visualise;
}

/*!
 * \brief Sets the file name containing the training data.
 * \param[in] fileName - training file name
 */
void SelfOrganisingMap::setTrainingFile(QString fileName)
{
    trainingFileName = fileName;
}

/*!
 * \brief Sets the file name used for saving the trained self-organising map.
 * \param[in] fileName - file name used for saving self-organising map
 */
void SelfOrganisingMap::setMapFile(QString fileName)
{
    mapFileName = fileName;
}

/*!
 * \brief Sets the iteration pause.
 * \note This iteration pause is used to make the training slower
 * \note so that Aquila can visualise the training process.
 * \param[in] value - iteration pause
 */
void SelfOrganisingMap::setIterationPause(int value)
{
    iterationPause = value;
}

/*!
 * \brief Returns current map.
 * \return map - current map
 */
QVector<float> SelfOrganisingMap::getCurrentMap()
{
    QVector<float> map;
    for(int i=0; i<(numInputs * numOutputs); i++)
    {
        map.push_back(weight_h[i]);
    }
    return map;
}

/*!
 * \brief Returns current maximum number of threads per block.
 * \return maxThreads - maximum number of threads
 */
int SelfOrganisingMap::getMaxThreads()
{
	return maxThreads;
}

/*!
 * \brief Returns current number of inputs.
 * \return numInputs - number of inputs
 */
int SelfOrganisingMap::getNumInputs()
{
	return numInputs;
}

/*!
 * \brief Returns current number of samples.
 * \return numSamples - number of samples
 */
int SelfOrganisingMap::getNumSamples()
{
	return numSamples;
}

/*!
 * \brief Returns current number of outputs.
 * \return numOutputs - number of outputs
 */
int SelfOrganisingMap::getNumOutputs()
{
	return numOutputs;
}

/*!
 * \brief Returns current neighbourhood size.
 * \return neighbourhoodSize - neighbourhood size
 */
int SelfOrganisingMap::getNeighbourhoodSize()
{
    return neighbourhoodSize;
}


/*!
 * \brief Returns current number of sub-iterations.
 * \return numSubiterations - number of sub-iterations
 */
int SelfOrganisingMap::getNumSubiterations()
{
	return numSubiterations;
}

/*!
 * \brief Returns current initial learning rate.
 * \return initLearningRate - initial learning rate
 */
float SelfOrganisingMap::getInitLearningRate()
{
	return initLearningRate;
}

/*!
 * \brief Returns current sigma.
 * \return sigma - sigma value
 */
float SelfOrganisingMap::getSigma()
{
	return sigma;
}

/*!
 * \brief Returns current progress visualisation mode.
 * \return visualiseProgress - progress visualisation
 */
bool SelfOrganisingMap::getVisualiseProgress()
{
	return visualiseProgress;
}

/*!
 * \brief Returns current iteration pause value.
 * \return iterationPause - iteration pause
 */
int SelfOrganisingMap::getIterationPause()
{
	return iterationPause;
}

/*!
 * \brief Returns the lowest data point found in the training data file.
 * \return lowestDataPoint - lowest data point
 */
QVector<float> SelfOrganisingMap::getDataPoints()
{
    QVector<float> dataPoints;
    dataPoints.push_back(lowestDataPoint);
    dataPoints.push_back(highestDataPoint);
    return dataPoints;
}

/*!
 * \brief Returns the trianing file name.
 * \return trainingFileName - training file name
 */
QString SelfOrganisingMap::getTrainingFile()
{
	return trainingFileName;
}

/*!
 * \brief Returns the file name used for saving the trained self-organising map.
 * \return mapFileName - file name used for saving self-organising map
 */
QString SelfOrganisingMap::getMapFile()
{
	return mapFileName;
}

/*!
 * \brief Calculates the euclidean distance between input vector and weight vector of a neuron and saves these distances in the output array.
 * \param[in] inputs - inputs
 * \param[in] weights - weights
 * \param[in] numInputs - number of inputs
 * \param[in] sequenceId - sequence id
 * \param[in] numOutputs - number of outputs
 * \param[out] outputs - outputs
 */
void SelfOrganisingMap::propogateInput(float *inputs, float *weights, float *outputs, int numInputs, int sequenceId, int numOutputs)
{
    //euclidean distance
    float distance = 0.0;

	//calculate the euclidean distance between each node's weight vector and the current input vector
	#pragma omp parallel for reduction(+: distance)
	for(int idx=0; idx<numOutputs; idx++)
    {
        for(int j=0; j<numInputs; j++)
		{
			distance += (inputs[(sequenceId * numInputs) + j] - weights[idx+(numOutputs*j)]) * (inputs[(sequenceId * numInputs) + j] - weights[idx+(numOutputs*j)]);
		}

        distance = sqrt(distance);
		outputs[idx] = distance;
		distance = 0.0;
	}    
}

/*!
 * \brief Finds the best matching unit having the lowest euclidean distance.
 * \param[in] outputs - outputs
 * \param[in] numOutputs - number of outputs
 * \param[out] winner - winner
 */
void SelfOrganisingMap::findBestMatch(float *outputs,  int *winner, int numOutputs)
{
    float lowestDistance = 999999.00;
    
    //find to lowest distance
    for (int idx=0; idx<numOutputs; idx++)
    {
        if(outputs[idx] < lowestDistance)
        {
			lowestDistance = outputs[idx];
			winner[0] = idx;
        }
    }
    
    //find out X and Y coordinates and save them in activity_index[1] and activity_index[2] respectively
    int y = -1;
    int x = int(winner[0]);
    int dim = int(sqrt((float)numOutputs));

    while(x > -1)
    {
        x -= dim;
        y++;
    }

    //assign X,Y possitions
    winner[1] = x + dim;
    winner[2] = y;
}

/*!
 * \brief Updates weights of the self-organising map.
 * \param[in] inputs - inputs
 * \param[in] winner - winner
 * \param[in] sigma - sigma
 * \param[in] numInputs - number of inputs
 * \param[in] sequenceId - sequence id
 * \param[in] numOutputs - number of outputs
 * \param[in] neighbourhoodSize - neighbourhood size
 * \param[in] numIterations - number of iterations
 * \param[in] currentIteration - current iteration
 * \param[out] weights - weights
 */
void SelfOrganisingMap::updateWeights(float *inputs, float *weights, int *winner, float sigma, int numInputs, int sequenceId, int numOutputs, int neighbourhoodSize, float initLearningRate, int numIterations, int currentIteration)
{
	//adjust the learning rate
    float currentLearningRate = initLearningRate*exp(-(float)currentIteration/numIterations);

    //dimension of the map
    int dim = int(sqrt((float)numOutputs));

    //euclidean distance
    float distance = 0.0;

	int curY = 0;
	int curX = 0;

	#pragma omp parallel
	{
		#pragma omp for reduction(+: curX, curY) firstprivate(distance)
		for (int idx=0; idx<numOutputs; idx++)
		{
			//convert current index to x,y
			curY = -1;
			curX = idx;
			do{
				curX -= dim;
				curY ++;
            }while(curX>-1);
			curX += dim;

			//calculate the distance from winner to current in the matrix
            distance = (float)((curX-winner[1])*(curX-winner[1]))+((curY-winner[2])*(curY-winner[2]));

            if(distance > (neighbourhoodSize*neighbourhoodSize))
			{
				distance = 0.0;
			}
			else
			{
                if(sigma>0.0)
				{
                    float s = sigma*exp(-(float)currentIteration/numIterations);
                    distance = exp(-(distance)/(2*(s*s)));
				}
				else
				{
                    distance = exp(-(distance)/(2*dim));
				}
			}

			//update weights
			for(int j=0; j<numInputs; j++) 
			{
                weights[idx+(numOutputs*j)] += (inputs[(sequenceId*numInputs)+j] - weights[idx+(numOutputs*j)]) * distance * currentLearningRate;
			}
		}	
	}
}

}
