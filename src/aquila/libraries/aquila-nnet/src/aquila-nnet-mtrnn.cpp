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

#include <omp.h>
#include <math.h>
#include <fstream>
#include <assert.h>
#include "aquila-nnet-mtrnn.h"
#include "mtrnnKernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

namespace aquila
{
/*!
 * \brief Constructor.
 */
MTRNN::MTRNN()
{
    gpu = new GPU();
    math = new Math();

    deltaT_h = NULL;
    input_h = NULL;
    activity_h = NULL;
    error_h = NULL;
    zeroError_h = NULL;
    delta_h = NULL;
    previousDelta_h = NULL;
    individualError_h = NULL;
    mse_h = NULL;
    previousDeltaWeight_h = NULL;
    weight_h = NULL;
    previousPotential_h = NULL;
    potential_h = NULL;
    zeroWeight_h = NULL;
    zeroNeuron_h = NULL;
    sequenceSteps = NULL;
    sequenceOffsets = NULL;
    errors = NULL;

    for(int i=0; i<MAX_GPU_DEVICES; i++)
    {
        deltaT_d[i] = NULL;
        mse_d[i] = NULL;
        activity_d[i] = NULL;
        input_d[i] = NULL;
        error_d[i] = NULL;
        delta_d[i] = NULL;
        buffer_d[i] = NULL;
        previousDelta_d[i] = NULL;
        individualError_d[i] = NULL;
        deltaWeight_d[i] = NULL;
        previousDeltaWeight_d[i] = NULL;
        weight_d[i] = NULL;
        potential_d[i] = NULL;
        previousPotential_d[i] = NULL;
        deltaWeight_h[i] = NULL;
    }
}

/*!
 * \brief Desstructor.
 */
MTRNN::~MTRNN()
{
}

/*!
 * \brief Initialialises everything and starts the training.
 */
void MTRNN::initialise()
{
    iteration = 0;
    threads2D = 16;

    allocateMemory();
    randomiseWeights();
    setDeltaT();
    setGridBlock();
    copyMemoryToDevice();
    start();
}

/*!
 * \brief Deinitialises everything and deallocates memory
 */
void MTRNN::deinitialise()
{
    //free device memory
    if(gpu->active)
    {
        for(int i=0; i<gpu->numRequestedDevices; i++)
        {
            gpuAssert(cudaFree(deltaT_d[i]));
            gpuAssert(cudaFree(weight_d[i]));
            gpuAssert(cudaFree(potential_d[i]));
            gpuAssert(cudaFree(previousPotential_d[i]));
            gpuAssert(cudaFree(mse_d[i]));
            gpuAssert(cudaFree(activity_d[i]));
            gpuAssert(cudaFree(input_d[i]));
            gpuAssert(cudaFree(error_d[i]));
            gpuAssert(cudaFree(delta_d[i]));
            gpuAssert(cudaFree(previousDelta_d[i]));
            gpuAssert(cudaFree(individualError_d[i]));
            gpuAssert(cudaFree(deltaWeight_d[i]));
            gpuAssert(cudaFree(previousDeltaWeight_d[i]));

            //host memory
            gpuAssert(cudaFreeHost(deltaWeight_h[i]));
        }
        //set back to device zero to have the same context for dealocation of the host memory
        gpu->setDevice(gpu->device.at(0));

        //free host memory
        gpuAssert(cudaFreeHost(deltaT_h));
        gpuAssert(cudaFreeHost(weight_h));
        gpuAssert(cudaFreeHost(previousPotential_h));
        gpuAssert(cudaFreeHost(potential_h));
        gpuAssert(cudaFreeHost(activity_h));
        gpuAssert(cudaFreeHost(input_h));
        gpuAssert(cudaFreeHost(error_h));
        gpuAssert(cudaFreeHost(delta_h));
        gpuAssert(cudaFreeHost(previousDelta_h));
        gpuAssert(cudaFreeHost(mse_h));
        gpuAssert(cudaFreeHost(sequenceSteps));
        gpuAssert(cudaFreeHost(previousDeltaWeight_h));
    }
    else
    {
        //free host memory
        free(deltaT_h);
        free(weight_h);
        free(previousPotential_h);
        free(potential_h);
        free(activity_h);
        free(input_h);
        free(error_h);
        free(delta_h);
        free(previousDelta_h);
        free(mse_h);
        free(sequenceSteps);
        free(previousDeltaWeight_h);
        free(deltaWeight_h[0]);
    }
}

/*!
 * \brief Allocates memory on host and GPU device(s).
 */
void MTRNN::allocateMemory()
{
    if(gpu->active) //allocate pinned host memory
    {
        gpuAssert(cudaMallocHost((void**)&deltaT_h, numNeurons * sizeof(int)));
        gpuAssert(cudaMallocHost((void**)&weight_h, numWeights * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&previousPotential_h, numNeurons * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&potential_h, numNeurons * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&zeroWeight_h, numWeights * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&zeroNeuron_h, numNeurons * sizeof(float)));
    }
    else //allocate normal host memory
    {
        deltaT_h = (int*)malloc(numNeurons * sizeof(int));
        weight_h = (float*)malloc(numWeights * sizeof(float));
        previousPotential_h = (float*)malloc(numNeurons * sizeof(float));
        potential_h = (float*)malloc(numNeurons * sizeof(float));
        zeroWeight_h = (float*)malloc(numWeights * sizeof(float));
        zeroNeuron_h = (float*)malloc(numNeurons * sizeof(float));
    }

    for(int i=0; i<numWeights; i++)
    {
        zeroWeight_h[i] = 0.0;
    }

    if(gpu->active)
    {
        for(int i=0; i<gpu->numRequestedDevices; i++)
        {
            deltaT_d[i] = NULL;
            weight_d[i] = NULL;
            potential_d[i] = NULL;
            previousPotential_d[i] = NULL;

            gpu->setDevice(gpu->device.at(i));

            //allocate device memory
            gpuAssert(cudaMalloc((void **) &weight_d[i], numWeights * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &potential_d[i], numNeurons * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &previousPotential_d[i], numNeurons * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &deltaT_d[i], numNeurons * sizeof(int)));
        }

        //set back to the master device
        if(gpu->numRequestedDevices>1)
        {
            gpu->setDevice(gpu->device.at(0));
        }
    }

    if(gpu->active) //allocate pinned host memory
    {
        gpuAssert(cudaMallocHost((void**)&activity_h, (maxSequenceSteps * numNeurons) * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&error_h, (maxSequenceSteps * numNeurons) * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&zeroError_h, (maxSequenceSteps * numNeurons) * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&delta_h, numNeurons * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&previousDelta_h, numNeurons * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&individualError_h, numNeurons * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&mse_h, sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&previousDeltaWeight_h, numWeights * sizeof(float)));
        gpuAssert(cudaMallocHost((void**)&deltaWeight_h[0], numWeights * sizeof(float)));
    }
    else //allocate normal host memory
    {
        activity_h = (float*)malloc((maxSequenceSteps * numNeurons) * sizeof(float));
        error_h = (float*)malloc((maxSequenceSteps * numNeurons) * sizeof(float));
        zeroError_h = (float*)malloc((maxSequenceSteps * numNeurons) * sizeof(float));
        delta_h = (float*)malloc(numNeurons * sizeof(float));
        previousDelta_h = (float*)malloc(numNeurons * sizeof(float));
        individualError_h = (float*)malloc(numNeurons * sizeof(float));
        mse_h = (float*)malloc(sizeof(float));
        previousDeltaWeight_h = (float*)malloc(numWeights * sizeof(float));
        deltaWeight_h[0] = (float*)malloc(numWeights * sizeof(float));
    }

    for(int i=0; i<(maxSequenceSteps * numNeurons); i++)
    {
        error_h[i] = 0.0;
        zeroError_h[i] = 0.0;
    }

    for(int i=1; i<gpu->numRequestedDevices; i++)
    {
        gpu->setDevice(gpu->device.at(i));
        gpuAssert(cudaMallocHost((void**)&deltaWeight_h[i], numWeights * sizeof(float)));
    }

    if(gpu->active)
    {
        for(int i=0; i<gpu->numRequestedDevices; i++)
        {
            int id = -1;

            gpu->setDevice(gpu->device.at(i));
            cudaGetDevice(&id);

            //allocate device memory
            gpuAssert(cudaMalloc((void **) &mse_d[i], sizeof(float)));
            gpuAssert(cudaMalloc((void **) &activity_d[i], (maxSequenceSteps * numNeurons) * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &input_d[i], (totalSequenceSteps * numIONeurons) * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &error_d[i], (maxSequenceSteps * numNeurons) * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &delta_d[i], numNeurons * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &previousDelta_d[i], numNeurons * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &individualError_d[i], numNeurons * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &deltaWeight_d[i], numWeights * sizeof(float)));
            gpuAssert(cudaMalloc((void **) &previousDeltaWeight_d[i], numWeights * sizeof(float)));

            if(numNeurons<=1024) //buffer is also used for synaptic connection sums
            {
                gpuAssert(cudaMalloc((void **) &buffer_d[i], numWeights * sizeof(float)));
            }
            else //buffer is only used for error summation on input-output units
            {
                gpuAssert(cudaMalloc((void **) &buffer_d[i], numIONeurons * sizeof(float)));
            }
        }

        //set back to the master device
        if(gpu->numRequestedDevices>1)
        {
            gpu->setDevice(gpu->device.at(0));
        }
    }

    //initialise arrays
    for(int idx=0; idx<numWeights; idx++)
    {
        zeroWeight_h[idx] = 0.0;
        weight_h[idx] = 0.0;
    }

    for(int idx=0; idx<numNeurons; idx++)
    {
        zeroNeuron_h[idx] = 0.0;
        potential_h[idx] = 0.0;
        previousPotential_h[idx] = 0.0;
    }

    int sequenceOffset = 0;

    if(gpu->active) //allocate pinned host memory
    {
        gpuAssert(cudaMallocHost((void**)&sequenceOffsets, numSequences * sizeof(int)));
    }
    else //allocate normal host memory
    {
        sequenceOffsets = (int*)malloc(numSequences * sizeof(int));
    }

    //set the inputs
    for (int sequence=0; sequence<numSequences; sequence++)
    {
        //save offsets for the backpropagation through-time training
        sequenceOffsets[sequence] = sequenceOffset;

        //calculate the offset for this sequence so that we write to a correct index
        sequenceOffset += sequenceSteps[sequence] * numIONeurons;
    }
}

/*!
 * \brief Copies host memory to GPU device(s).
 */
void MTRNN::copyMemoryToDevice()
{
    if(gpu->active)
    {
        for(int i=0; i<gpu->numRequestedDevices; i++)
        {
            int id = -1;

            gpu->setDevice(gpu->device.at(i));
            cudaGetDevice(&id);

            //copy memory to device
            gpuAssert(cudaMemcpy(potential_d[i], potential_h, numNeurons * sizeof(float), cudaMemcpyHostToDevice))
            gpuAssert(cudaMemcpy(previousPotential_d[i], previousPotential_h, numNeurons * sizeof(float), cudaMemcpyHostToDevice))
            gpuAssert(cudaMemcpy(weight_d[i], weight_h, numWeights * sizeof(float), cudaMemcpyHostToDevice))
            gpuAssert(cudaMemcpy(deltaT_d[i], deltaT_h, numNeurons * sizeof(int), cudaMemcpyHostToDevice))
            gpuAssert(cudaMemcpy(activity_d[i], activity_h, (maxSequenceSteps * numNeurons) * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(input_d[i], input_h, totalSequenceSteps * numIONeurons * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(error_d[i], error_h, (maxSequenceSteps * numNeurons) * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(delta_d[i], delta_h, numNeurons * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(previousDelta_d[i], previousDelta_h, numNeurons * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(deltaWeight_d[i], deltaWeight_h[0], numWeights * sizeof(float), cudaMemcpyHostToDevice));
            gpuAssert(cudaMemcpy(previousDeltaWeight_d[i], previousDeltaWeight_h, numWeights * sizeof(float), cudaMemcpyHostToDevice));
        }

        //set back to the master device
        if(gpu->numRequestedDevices>1)
        {
            gpu->setDevice(gpu->device.at(0));
        }
    }
}

/*!
 * \brief Loads training data.
 */
bool MTRNN::loadTrainingData()
{
    //initialise
    vector<int> seqLength;
    vector<float> seqData;

    //load the data either from port or file
    if(trainingData.size()>0) //loading training data from received bottle
    {
        int pos = 2;
        numControlNeurons = trainingData.get(pos).asInt();pos++;
        numLinguisticNeurons = trainingData.get(pos).asInt();pos++;
        numVisionNeurons = trainingData.get(pos).asInt();pos++;
        numActionNeurons = trainingData.get(pos).asInt();pos++;
        numSequences = trainingData.get(pos).asInt();pos++;
        sequenceWidth = trainingData.get(pos).asInt();pos++;
        minValue = (float)(trainingData.get(pos).asDouble());pos++;
        maxValue = (float)(trainingData.get(pos).asDouble());pos++;
        int numElements = trainingData.get(pos).asInt();pos++;
        maxSequenceSteps = trainingData.get(pos).asInt();pos++;
        totalSequenceSteps = trainingData.get(pos).asInt();pos++;

        //read the sequence lengths
        for(int i=0; i<numSequences; i++)
        {
            seqLength.push_back(trainingData.get(pos).asInt());
            pos++;
        }

        //read the total number of elements in the data
        for(int i=0; i<numElements; i++)
        {
            seqData.push_back((float)(trainingData.get(pos).asDouble()));
            pos++;
        }
    }
    else //loading training data from a file
    {
        ResourceFinder rf;
        Property config;
        Bottle bSample;
        Bottle *pS;
        totalSequenceSteps = 0;
        maxSequenceSteps = 0;
        maxValue = 0.0f;
        minValue = 99999.0f;

        //find the training set file
        if(!config.fromConfigFile(rf.findFile(trainingFileName.toStdString().c_str())))
        {
            qCritical("failed to load training data from %s", trainingFileName.toStdString().c_str());
            return false;
        }

        //load the architecture specification
        bSample = config.findGroup("ARCHITECTURE");
        if(!bSample.isNull())
        {
            numControlNeurons = bSample.find("controlNeurons").asInt();
            numLinguisticNeurons = bSample.find("linguisticNeurons").asInt();
            numVisionNeurons = bSample.find("visionNeurons").asInt();
            numActionNeurons = bSample.find("actionNeurons").asInt();
        }
        else
        {
            qDebug("group [ARCHITECTURE] missing in %s", trainingFileName.toStdString().c_str());
            assert(bSample.isNull()!=true);
        }

        //check if the file contains first SEQUENCE_0 header
        char buf[20];
        bSample.clear();
        for(int i=0; i<MAX_SEQUENCES; i++)
        {
            sprintf(buf,"SEQUENCE_%i", i);
            bSample = config.findGroup(buf);
            if (bSample.isNull())
            {
                //at least the first sequence header must be found
                if(i==0)
                {
                    qCritical("group [%s] missing in %s",buf, trainingFileName.toStdString().c_str());
                    assert(bSample.isNull()!=true);
                }
            }
            else
            {
                //save the sequence width
                if(i==0)
                {
                    sequenceWidth = bSample.get(i+1).asList()->size();
                }

                //save the sequence length information
                seqLength.push_back(bSample.size()-1);
                totalSequenceSteps+=bSample.size()-1;

                 //current sequence is the new longest sequence
                if(bSample.size()-1 > maxSequenceSteps)
                {
                    maxSequenceSteps = bSample.size()-1;
                }

                //get the training data
                for(int j=0; j<seqLength.at(i); j++)
                {
                    //get the whole row
                    pS = bSample.get(j+1).asList();

                    //add this sequence step to data
                    for(int k=0; k<sequenceWidth; k++)
                    {
                        float value = (float)pS->get(k).asDouble();
                        seqData.push_back(value);

                        //find maximum and minimum values
                        if(value > maxValue)
                        {
                            maxValue = value;
                        }

                        if(value < minValue)
                        {
                            minValue = value;
                        }
                    }
                }
            }
        }

        //save number of sequences
        numSequences = seqLength.size();
    }

    //allocate memory for the host input array and copy the training data
    if(gpu->active) //pinned memory
    {
        gpuAssert(cudaMallocHost((void**)&input_h, (sequenceWidth * totalSequenceSteps) * sizeof(float)));
    }
    else //normal memory
    {
        input_h = (float*)malloc((sequenceWidth * totalSequenceSteps) * sizeof(float));
    }

    for(int i=0; i<(sequenceWidth * totalSequenceSteps); i++)
    {
        input_h[i] = math->scaleRange(seqData.at(i), minValue, maxValue, 0.0f, 1.0f);
    }

    //allocate memory for the host sequenceSteps array and copy the sequence lengths
    if(gpu->active) //pinned memory
    {
        gpuAssert(cudaMallocHost((void**)&sequenceSteps, numSequences * sizeof(int)));
    }
    else //normal memory
    {
        sequenceSteps = (int*)malloc(numSequences * sizeof(int));
    }

    for(int i=0; i<numSequences; i++)
    {
        sequenceSteps[i] = seqLength.at(i);
    }

    //make sure that the sequence size matches the specified architecture
    assert(sequenceWidth==numControlNeurons+numLinguisticNeurons+numVisionNeurons+numActionNeurons);

    //number of input neurons is at the moment equal sequence vector size
    numIONeurons = numControlNeurons+numLinguisticNeurons+numVisionNeurons+numActionNeurons;

    //calculate total number of neurons and synaptic weights
    numNeurons = numIONeurons + numFastNeurons + numSlowNeurons;
    numWeights = (numNeurons * numNeurons) + numNeurons;

    //set number of threads and blocks
    neuronThreads = maxThreads;
    neuronBlocks = numNeurons / neuronThreads + (numNeurons%neuronThreads == 0?0:1);

    return true;
}

/*!
 * \brief Tests the accuracy of the trained neural network.
 */
void MTRNN::testNetwork()
{
    qDebug("testing traned network");
    for(int i=0; i<numSequences; i++)
    {
        qDebug("sequence %i", i);
        float initialState = 0.1f+(i*0.1f);

        setInitStates(initialState, activity_h, zeroNeuron_h, numNeurons, numIONeurons, numFastNeurons);
        resetParameters(numNeurons, maxSequenceSteps, delta_h, previousDelta_h, potential_h, previousPotential_h, error_h, zeroNeuron_h, zeroError_h);

        for(int j=1; j<sequenceSteps[i]-1; j++)
        {
            forwardPass(j, sequenceOffsets[i], activity_h, input_h, weight_h, previousPotential_h, potential_h, error_h, deltaT_h, numNeurons, numIONeurons);

            qDebug("inputs:");
            for(int k=0; k<numIONeurons; k++)
            {
                qDebug("%f", math->scaleRange(input_h[sequenceOffsets[i]+(numIONeurons*j)+k], 0.0f, 1.0f, minValue, maxValue));
            }

            qDebug("outputs:");
            for(int k=0; k<numIONeurons; k++)
            {
                qDebug("%f", math->scaleRange(activity_h[(numNeurons*j)+k], 0.0f, 1.0f, minValue, maxValue));
            }
        }
    }
}

/*!
 * \brief Saves trained neural network.
 */
void MTRNN::saveNetwork()
{
    //save neural network
    ofstream myfile;
    myfile.open(networkFileName.toStdString().c_str());
    if(myfile.is_open())
    {
        char buf[10];
        myfile<<"[SETTINGS]\n";
        if(gpu->active)
        {
            myfile<<"execution mode\t\tGPU\n";
        }
        else
        {
            myfile<<"execution mode\t\tCPU \n";
        }

        myfile<<"\n[ARCHITECTURE]\n";
        myfile<<"control neurons\t\t"<<numControlNeurons<<"\n";
        myfile<<"linguistic neurons\t"<<numLinguisticNeurons<<"\n";
        myfile<<"vision neurons\t\t"<<numVisionNeurons<<"\n";
        myfile<<"action neurons\t\t"<<numActionNeurons<<"\n";
        myfile<<"fast neurons\t\t"<<numFastNeurons<<"\n";
        myfile<<"slow neurons\t\t"<<numSlowNeurons<<"\n";

        myfile<<"\n[TRAINING]\n";
        myfile<<"final error\t\t"<<errors[iteration-1]<<"\n";
        myfile<<"learning rate\t\t"<<learningRate<<"\n";
        myfile<<"momentum\t\t"<<momentum<<"\n";
        myfile<<"weight range\t\t"<<initWeightRange<<"\n";
        myfile<<"threshold\t\t"<<threshold<<"\n";
        myfile<<"iterations\t\t"<<iteration<<"\n";
        myfile<<"io delta-t\t\t"<<ioDeltaT<<"\n";
        myfile<<"fast delta-t\t\t"<<fastDeltaT<<"\n";
        myfile<<"slow delta-t\t\t"<<slowDeltaT<<"\n";
        myfile<<"original min\t\t"<<minValue<<"\n";
        myfile<<"original max\t\t"<<maxValue<<"\n";
        myfile<<"seed\t\t\t"<<math->getSeed()<<"\n";

        myfile<<"\n[WEIGHTS]\n";
        //connections from
        for(int i=0; i<numNeurons; i++)
        {
            //connections to, +1 is for bias weights stored in the last column
            for(int j=0; j<numNeurons+1; j++)
            {
                sprintf(buf,"%.6f", weight_h[(j*numNeurons)+i]);
                myfile<<buf;
                if(j<numNeurons)
                {
                    myfile<<",";
                }
            }
            myfile<<"\n";
        }

        myfile.close();
        qDebug("neural network saved to %s", networkFileName.toStdString().c_str());
    }
    else
    {
        qCritical("failed to open %s", networkFileName.toStdString().c_str());
    }
}

/*!
 * \brief Randomises weights and initialise connectivity.
 * \note Weights that are set to zero are not used.
 */
void MTRNN::randomiseWeights()
{
    //initialise random number generator
    math->setSeed();

    //randomise neural network weights based on the conectivity
    //not using openmp here to keep the weights values consistant for throughout different seeds
    for (int i=0; i<numNeurons; i++)
    {
        //connections from input
        if(i<numIONeurons)
        {
            //to itself and to fast neurons
            for(int j=0; j<numIONeurons+numFastNeurons; j++)
            {
                weight_h[(j*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
            }

            //set the bias
            weight_h[(numNeurons*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
        }

        //connections from fast neurons
        else if(i<(numIONeurons+numFastNeurons))
        {
            //to input-output neurons, to fast and to slow neurons
            for(int j=0; j<numNeurons; j++)
            {
                weight_h[(j*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
            }

            //set the bias
            weight_h[(numNeurons*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
        }

        //connection from slow neurons
        else if(i<numNeurons)
        {
            //to itself and to fast neurons
            for(int j=numIONeurons; j<numNeurons; j++)
            {
                weight_h[(j*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
            }

            //set the bias
            weight_h[(numNeurons*numNeurons)+i] = ((rand() /float(RAND_MAX)) * (2.0f*initWeightRange)) - initWeightRange;
        }
    }
}

/*!
 * \brief Thread loop.
 * \note This loop runs the entire backpropagation through-time training.
 */
void MTRNN::run()
{
    //module is going to change its state to training (1), let Aquila know
    emit trainingStarted();

    yarp::os::Time::delay(0.5);
    running  = true;

    if(gpu->active)
    {
        cudaEvent_t start_d;
        cudaEvent_t stop_d;
        double start_h = 0.0;

        if(gpu->numRequestedDevices==1) //single GPU mode uses CUDA event timer
        {
            gpuAssert(cudaEventCreate(&start_d));
            gpuAssert(cudaEventCreate(&stop_d));
            gpuAssert(cudaEventRecord(start_d, 0));
        }
        else //multi-GPU mode uses OpenMP timer
        {
            start_h = omp_get_wtime();
        }

        //set number of threads, one thread per GPU, one GPU per sequence
        omp_set_num_threads(gpu->numRequestedDevices);

        //sequences are processed by the number of requested GPUs, if only one GPU is used then the number of subiterations
        //will be equal to the number of sequences; if two devices are used this number will be half as long as there the
        //sequences can be divided unifromly amongst GPUs.
        int subIterations = (numSequences / gpu->numRequestedDevices) + (numSequences%gpu->numRequestedDevices==0?0:1);

        //create stream and event arrays
        cudaStream_t cudaStream[MAX_GPU_DEVICES];
        cudaEvent_t cudaEvent[MAX_GPU_DEVICES];

        //assign streams and events to the devices
        for(int i=0; i<gpu->numRequestedDevices; i++)
        {
            gpu->setDevice(gpu->device.at(i));
            cudaStreamCreate(&cudaStream[i]);
            cudaEventCreate(&cudaEvent[i]);
        }

        //main loop
        while(running && iteration < maxIterations)
        {
            //reset parameters
            *mse_h = 0.0;

            #pragma omp parallel
            {
                int threadId = omp_get_thread_num();

                if(gpu->numRequestedDevices>1)
                {
                    gpu->setDevice(gpu->device.at(threadId));
                }

                resetDeltaWeightsOnDevice(dim2DWGrid, dim2DWBlock, cudaStream[threadId], numWeights, numIONeurons, deltaWeight_d[threadId], individualError_d[threadId]);

                if(gpu->numRequestedDevices>1)
                {
                    cudaEventRecord(cudaEvent[threadId], cudaStream[threadId]);
                }
            }

            //iterate for the number of subiterations required to process all the sequences given the number of GPUs
            for(int i=0; i<subIterations; i++)
            {
                #pragma omp parallel
                {
                    int threadId = omp_get_thread_num();
                    int seqId = (i*gpu->numRequestedDevices)+threadId;

                    if(gpu->numRequestedDevices>1)
                    {
                        gpu->setDevice(gpu->device.at(threadId));

                        //make sure previous operations were finished
                        cudaEventSynchronize(cudaEvent[threadId]);
                        #pragma omp barrier
                    }

                    if(seqId<numSequences)
                    {
                        if(gpu->numRequestedDevices>1)
                        {
                            gpu->setDevice(gpu->device.at(threadId));
                        }

                        //calculate initial state value for this sequence
                        float initialState = 0.1f+(seqId*0.1f);

                        //set initial states for different sequences
                        setInitStatesOnDevice(neuronBlocks, neuronThreads, cudaStream[threadId], initialState, activity_d[threadId], numNeurons, numIONeurons, numFastNeurons);

                        //reset parameters
                        resetParametersOnDevice(seqNeuronBlocks, neuronThreads, cudaStream[threadId], numNeurons, maxSequenceSteps, delta_d[threadId], previousDelta_d[threadId], potential_d[threadId], previousPotential_d[threadId], error_d[threadId]);

                        //forward pass
                        for(int j=1; j<sequenceSteps[seqId]-1; j++)
                        {
                            if(numNeurons > 1024)
                            {
                                forwardPassV1onDevice(neuronBlocks, neuronThreads, cudaStream[threadId], j, sequenceOffsets[seqId], activity_d[threadId], input_d[threadId], weight_d[threadId], previousPotential_d[threadId], error_d[threadId], potential_d[threadId], deltaT_d[threadId], numNeurons, numIONeurons);
                            }
                            else
                            {
                                forwardPassV2onDevice(dim2DGrid, dim2DBlock, cudaStream[threadId], j, sequenceOffsets[seqId], activity_d[threadId], input_d[threadId], weight_d[threadId], numNeurons, numIONeurons, buffer_d[threadId]);
                                forwardPassV21onDevice(numFBlocks, numFThreads, smemSize, cudaStream[threadId], j, sequenceOffsets[seqId], activity_d[threadId], input_d[threadId], buffer_d[threadId], potential_d[threadId], weight_d[threadId], previousPotential_d[threadId], error_d[threadId], deltaT_d[threadId], numNeurons, numIONeurons);
                            }
                        }

                        //backward pass
                        for(int j=sequenceSteps[seqId]-2; j>0; j--)
                        {
                            if(numNeurons > 1024)
                            {
                                backwardPassV1onDevice(dim2DGrid, dim2DBlock, cudaStream[threadId], j, sequenceOffsets[seqId], numNeurons, numIONeurons, input_d[threadId], activity_d[threadId], delta_d[threadId], deltaWeight_d[threadId], previousDelta_d[threadId], error_d[threadId], individualError_d[threadId], deltaT_d[threadId], weight_d[threadId]);
                                backwardPassV11onDevice(neuronBlocks, neuronThreads, cudaStream[threadId], j, numNeurons, numIONeurons, activity_d[threadId], delta_d[threadId], previousDelta_d[threadId], deltaT_d[threadId], weight_d[threadId]);
                            }
                            else
                            {
                                backwardPassV2onDevice(dim2DGrid, dim2DBlock, cudaStream[threadId], j, sequenceOffsets[seqId], numNeurons, numIONeurons, input_d[threadId], activity_d[threadId], delta_d[threadId], deltaWeight_d[threadId], previousDelta_d[threadId], error_d[threadId], individualError_d[threadId], deltaT_d[threadId], weight_d[threadId], buffer_d[threadId]);
                                backwardPassV21onDevice(numHBlocks, numFThreads, smemSize, cudaStream[threadId], buffer_d[threadId], delta_d[threadId], numNeurons, numIONeurons);
                            }

                            backwardPassV3onDevice(dim2DGrid, dim2DBlock, cudaStream[threadId], j, numNeurons, numIONeurons, activity_d[threadId], delta_d[threadId], previousDelta_d[threadId], deltaWeight_d[threadId], deltaT_d[threadId], weight_d[threadId]);
                        }

                        //add and save error - threads known at a compile time so we can do the reduction faster
                        reduceOnDevice(numEThreads, numEBlocks, numEThreads, smemESize, cudaStream[threadId], individualError_d[threadId], buffer_d[threadId], numIONeurons, false);

                        int s = numEBlocks;

                        while(s>1)
                        {
                            int numThreads = (s<maxThreads*2) ? math->nextPow2((s+1)/2) : maxThreads;
                            int numBlocks = (s+(numThreads*2-1))/(numThreads*2);
                            int smemSize = (numThreads<=32) ? 2*numThreads*sizeof(float) : numThreads*sizeof(float);

                            //add and save error - threads known at a compile time so we can do the reduction faster
                            reduceOnDevice(numThreads, numBlocks, numThreads, smemSize, cudaStream[threadId], buffer_d[threadId], buffer_d[threadId], s, false);

                            s = (s+(numThreads*2-1))/(numThreads*2);
                        }

                        cudaEventRecord(cudaEvent[threadId], cudaStream[threadId]);
                    }
                }
            }

            //update weights - currently three different strategies are used depending on the number of GPUs and their ability to access each other
            if(gpu->numRequestedDevices>1)
            {
                if(gpu->peerToPeerActive) //two devices can access each other directly via PCIe
                {
                    gpu->setDevice(gpu->device.at(0));
                    cudaEventSynchronize(cudaEvent[1]);

                    sumDeltaWeightsP2PonDevice(dim2DWGrid, dim2DWBlock, numWeights, deltaWeight_d[0], deltaWeight_d[1]);
                    updateWeightsP2PonDevice(dim2DWGrid, dim2DWBlock, numWeights, learningRate, momentum, weight_d[0], weight_d[1], deltaWeight_d[0], previousDeltaWeight_d[0]);
                    sumErrorP2PonDevice(1, 1, buffer_d[0], buffer_d[1]);
                }
                else //two or more devices - P2P not used; deltas are copied from all devices to the host, summed, copied to
                {
                    #pragma omp parallel
                    {
                        int threadId = omp_get_thread_num();
                        gpu->setDevice(gpu->device.at(threadId));
                        gpuAssert(cudaMemcpy(deltaWeight_h[threadId], deltaWeight_d[threadId], numWeights * sizeof(float), cudaMemcpyDeviceToHost));
                    }

                    //sum results on host
                    float sum = 0.0;
                    #pragma omp parallel for reduction(+: sum)
                    for(int i=0; i<numWeights; i++)
                    {
                        sum = 0.0;

                        for(int j=0; j<gpu->numRequestedDevices; j++)
                        {
                            sum += deltaWeight_h[j][i];
                        }

                        //save the delta
                        deltaWeight_h[0][i] = sum;
                    }

                    //copy the delta weights to all other devices asynchronously
                    #pragma omp parallel
                    {
                        int threadId = omp_get_thread_num();

                        gpu->setDevice(gpu->device.at(threadId));

                        //copy the results to all devices
                        gpuAssert(cudaMemcpy(deltaWeight_d[threadId], deltaWeight_h[0], numWeights * sizeof(float), cudaMemcpyHostToDevice));

                        //modify weights
                        updateWeightsOnDevice(dim2DWGrid, dim2DWBlock, learningRate, momentum, weight_d[threadId], deltaWeight_d[threadId], previousDeltaWeight_d[threadId], numWeights);
                    }
                }
            }
            else
            {
                //modify weights
                updateWeightsOnDevice(dim2DWGrid, dim2DWBlock, learningRate, momentum, weight_d[0], deltaWeight_d[0], previousDeltaWeight_d[0], numWeights);
            }

            //show error
            if(showProgress)
            {
                if(gpu->peerToPeerActive)
                {
                    gpuAssert(cudaMemcpy(mse_h, buffer_d[0], sizeof(float), cudaMemcpyDeviceToHost));
                }
                else
                {
                    for(int i=0; i<gpu->numRequestedDevices; i++)
                    {
                        float error = 0.0;

                        gpu->setDevice(gpu->device.at(i));
                        gpuAssert(cudaMemcpy(&error, buffer_d[i], sizeof(float), cudaMemcpyDeviceToHost));
                        mse_h[0] += error;
                    }
                }

                if(iteration%feedbackInterval==0 || iteration==maxIterations-1)
                {
                    //get current progress in %
                    int p = getProgress();

                    //send the current progress and error to the output port
                    if(!terminalMode)
                    {
                        emit progressSet(p);
                        emit errorSet(iteration, mse_h[0]);
                    }

                    qDebug("iteration: %i     error: %f     (%i%% completed)", iteration, mse_h[0], p);
                }

                //save the current error so we can produce graph and analyse the training
                errors[iteration] = mse_h[0];
            }

            if(mse_h[0]<=threshold)
            {
                break;
            }

            iteration++;
        }

        //get the final error if show_progress was switched off
        if(!showProgress)
        {
            for(int i=0; i<gpu->numRequestedDevices; i++)
            {
                float error = 0.0;

                gpu->setDevice(gpu->device.at(i));
                gpuAssert(cudaMemcpy(&error, buffer_d[i], sizeof(float), cudaMemcpyDeviceToHost));
                mse_h[0] += error;
            }
        }

        //stop timer
        if(gpu->numRequestedDevices==1)
        {
            float elapsed_time = 0.0;

            gpuAssert(cudaEventRecord(stop_d, 0));
            gpuAssert(cudaEventSynchronize(stop_d));
            gpuAssert(cudaEventElapsedTime(&elapsed_time, start_d, stop_d));
            gpuAssert(cudaEventDestroy(start_d));
            gpuAssert(cudaEventDestroy(stop_d));
            qDebug("training finished in %fms with %s", elapsed_time, cudaGetErrorString(cudaGetLastError()));
        }
        else
        {
            double end_h = omp_get_wtime();
            qDebug("training finished in %fms with %s", 1000*(end_h-start_h), cudaGetErrorString(cudaGetLastError()));
        }

        //copy data from device to host memory
        if(gpu->numRequestedDevices>1)
        {
            gpu->setDevice(gpu->device.at(0));
        }

        gpuAssert(cudaMemcpy(weight_h, weight_d[0], numWeights * sizeof(float), cudaMemcpyDeviceToHost));
    }
    else
    {
        double start = omp_get_wtime();

        while (running && iteration<maxIterations)
        {
            *mse_h = 0.0;
            resetDeltaWeights(numWeights, deltaWeight_h[0], zeroWeight_h);

            for(int i=0; i<numSequences; i++)
            {
                float initialState = 0.1f+(i*0.1f);
                setInitStates(initialState, activity_h, zeroNeuron_h, numNeurons, numIONeurons, numFastNeurons);
                resetParameters(numNeurons, maxSequenceSteps, delta_h, previousDelta_h, potential_h, previousPotential_h, error_h, zeroNeuron_h, zeroError_h);

                for(int j=1; j<sequenceSteps[i]-1; j++)
                {
                    forwardPass(j, sequenceOffsets[i], activity_h, input_h, weight_h, previousPotential_h, potential_h, error_h, deltaT_h, numNeurons, numIONeurons);
                }

                for(int j=sequenceSteps[i]-2; j>0; j--)
                {
                    backwardPass(j, sequenceOffsets[i], numNeurons, numIONeurons, input_h, activity_h, delta_h, previousDelta_h, error_h, weight_h, deltaWeight_h[0], mse_h, deltaT_h);
                }
            }

            updateWeights(learningRate, momentum, weight_h, deltaWeight_h[0], previousDeltaWeight_h, numWeights);


            if(iteration%feedbackInterval==0 || iteration==maxIterations-1)
            {
                //get current progress in %
                int p = getProgress();

                //send the current progress and error to the output port
                if(!terminalMode)
                {
                    emit progressSet(p);
                    emit errorSet(iteration, mse_h[0]);
                }

                qDebug("iteration: %i     error: %f     (%i%% completed)", iteration, mse_h[0], p);
            }

            //save the current error so we can produce graph and analyse the training
            errors[iteration] = mse_h[0];

            if(mse_h[0]<=threshold)
            {
                break;
            }

            iteration++;
        }

        double end = omp_get_wtime();
        qDebug("training finished in %fms", 1000*(end-start));
    }

    //module is going to change its state to training finished
    QVector<float> tmp;
    for(int i=0; i<iteration; i++)
    {
        tmp.append(errors[i]);//unify variables
    }
    emit trainingStopped(tmp);

    //test how well trained is the neural network
    testNetwork();

    //save the network directly only if the module was launched from a terminal and network file name was specified
    if(terminalMode && !networkFileName.isEmpty())
    {
        saveNetwork();
    }
}

/*!
 * \brief Stops the thread.
 */
void MTRNN::stop()
{
    running  = false;
}

/*!
 * \brief Sets training file.
 * \param[in] fileName - training file name
 */
void MTRNN::setTrainingFile(QString fileName)
{
    trainingFileName = fileName;
}

/*!
 * \brief Sets training data received from port.
 * \param[in] data - training data
 */
void MTRNN::setTrainingData(Bottle data)
{
    trainingData.copy(data);
}

/*!
 * \brief Sets neural network file.
 * \param[in] fileName - file name used for saving of neural network
 */
void MTRNN::setNetworkFile(QString fileName)
{
    networkFileName = fileName;
}

/*!
 * \brief Sets the maximum number of threads per block.
 * \param[in] threads - maximum number of threads
 */
void MTRNN::setMaxThreads(int threads)
{
    maxThreads = math->nextPow2(threads-1);
}

/*!
 * \brief Sets the maximum number of iterations and allocates memory for storing of the errors.
 * \param[in] iterations - maximum number of iterations
 */
void MTRNN::setMaxIterations(int iterations)
{
    maxIterations = iterations;

    //clean the previously allocated memory
    if(errors!=NULL)
    {
        free(errors);
    }

    //allocate new memory
    errors = (float*)malloc(maxIterations*sizeof(float));
}

/*!
 * \brief Sets delta-t value for input-output neurons.
 * \param[in] value - input-output delta-t value
 */
void MTRNN::setIODeltaT(int value)
{
    ioDeltaT = value;
}

/*!
 * \brief Sets delta-t value for fast neurons.
 * \param[in] value - fast delta-t value
 */
void MTRNN::setFastDeltaT(int value)
{
    fastDeltaT = value;
}

/*!
 * \brief Sets delta-t value for slow neurons.
 * \param[in] value - slow delta-t value
 */
void MTRNN::setSlowDeltaT(int value)
{
    slowDeltaT = value;
}

/*!
 * \brief Sets the initial range for weights.
 * \note If the value is for example 1.0 then the range will be from -1.0 to 1.0.
 * \param[in] value - initial weight range value
 */
void MTRNN::setInitWeightRange(float value)
{
    initWeightRange = value;
}

/*!
 * \brief Sets thrshold.
 * \param[in] value - threshold value
 */
void MTRNN::setThreshold(float value)
{
    threshold = value;
}

/*!
 * \brief Sets learning rate.
 * \param[in] value - learning rate value
 */
void MTRNN::setLearningRate(float value)
{
    learningRate = value;
}

/*!
 * \brief Sets momentum.
 * \param[in] value - momentum value
 */
void MTRNN::setMomentum(float value)
{
    momentum = value;
}

/*!
 * \brief Sets progress displaying mode .
 * \param[in] show - if true progress will be shown
 */
void MTRNN::setShowProgress(bool show)
{
    showProgress = show;
}

/*!
 * \brief Sets number of fast neurons.
 * \param[in] fastNeurons
 */
void MTRNN::setNumFastNeurons(int fastNeurons)
{
    numFastNeurons = fastNeurons;
}

/*!
 * \brief Sets number of slow neurons.
 * \param[in] slowNeurons - number of slow neurons
 */
void MTRNN::setNumSlowNeurons(int slowNeurons)
{
    numSlowNeurons = slowNeurons;
}

/*!
 * \brief Sets delta-t values of all neurons.
 */
void MTRNN::setDeltaT()
{
    //input-output neurons delta-t values
    for (int idx=0; idx<numIONeurons; idx++)
    {
        deltaT_h[idx] = ioDeltaT;
    }

    //fast neurons delta-t values
    for (int idx=numIONeurons; idx<numIONeurons + numFastNeurons; idx++)
    {
        deltaT_h[idx] = fastDeltaT;
    }

    //slow neurons delta-t values
    for (int idx=numIONeurons + numFastNeurons; idx<numNeurons; idx++)
    {
        deltaT_h[idx] = slowDeltaT;
    }
}

/*!
 * \brief Sets grid and block sizes.
 * \note This is used for kernel invocations.
 */
void MTRNN::setGridBlock()
{
    seqNeuronBlocks = (numNeurons * maxSequenceSteps) / neuronThreads + ((numNeurons * maxSequenceSteps)%neuronThreads==0?0:1);
    dim2DWBlock.x = maxThreads;
    dim2DWGrid.x = (int)sqrt((float)numWeights/(float)maxThreads)+1;
    dim2DWGrid.y = (int)sqrt((float)numWeights/(float)maxThreads)+1;

    numIoBlocks = numIONeurons / neuronThreads + (numIONeurons%neuronThreads==0?0:1);
    dim2DBlock.x = threads2D;
    dim2DBlock.y = threads2D;
    dim2DGrid.x = (numNeurons+(threads2D-1))/threads2D;
    dim2DGrid.y = (numNeurons+(threads2D-1))/threads2D;

    //parallel reduction params
    numFThreads = math->nextPow2(numNeurons);
    numHBlocks = numNeurons - numIONeurons;
    numFBlocks = numNeurons;
    numEThreads = (numIONeurons < maxThreads * 2) ? math->nextPow2((numIONeurons + 1)/ 2) : maxThreads;
    numEBlocks = (numIONeurons + (numEThreads * 2 - 1)) / (numEThreads * 2);
    smemSize = (numFThreads <= 32) ? 2 * numFThreads * sizeof(float) : numFThreads * sizeof(float);
    smemESize = (numEThreads <= 32) ? 2 * numEThreads * sizeof(float) : numEThreads * sizeof(float);
}

/*!
 * \brief Sets the feedback interval.
 * \param[in] interval - feedback interval
 */
void MTRNN::setFeedbackInterval(int interval)
{
    feedbackInterval = interval;
}

/*!
 * \brief Gets current learning rate.
 * \return learningRate - learning rate
 */
float MTRNN::getLearningRate()
{
    return learningRate;
}

/*!
 * \brief Gets current momentum.
 * \return momentum - current momentum
 */
float MTRNN::getMomentum()
{
    return momentum;
}

/*!
 * \brief Gets the initial range used for generating random number of weights.
 * \return initWeightRange - initial weight range
 */
float MTRNN::getWeightRange()
{
    return initWeightRange;
}

/*!
 * \brief Gets current threshold.
 * \return threshold - current threshold
 */
float MTRNN::getThreshold()
{
    return threshold;
}

/*!
 * \brief Gets the maximum number of iterations.
 * \return maxIterations - maximum number of iterations
 */
int MTRNN::getMaxIterations()
{
    return maxIterations;
}

/*!
 * \brief Gets the number of fast context neurons.
 * \return numFastNeurons - number of fast neurons
 */
int MTRNN::getNumFastNeurons()
{
    return numFastNeurons;
}

/*!
 * \brief Gets the number of slow context neurons.
 * \return numSlowNeurons - number of slow neurons
 */
int MTRNN::getNumSlowNeurons()
{
    return numSlowNeurons;
}

/*!
 * \brief Gets the delta-t value of the input-output neurons.
 * \return ioDeltaT - input-output delta-t
 */
int MTRNN::getIODeltaT()
{
    return ioDeltaT;
}

/*!
 * \brief Gets the delta-t value of the fast context neurons.
 * \return fastDeltaT - fast delta-t
 */
int MTRNN::getFastDeltaT()
{
    return fastDeltaT;
}

/*!
 * \brief Gets the delta-t value of the slow context neurons.
 * \return slowDeltaT - slow delta-t
 */
int MTRNN::getSlowDeltaT()
{
    return slowDeltaT;
}

/*!
 * \brief Gets file name used for saving trained network.
 * \return networkFileName - neural network file name used for saving
 */
QString MTRNN::getNetworkFile()
{
    return networkFileName;
}

/*!
 * \brief Gets file name used for network training.
 * \return trainingFileName - training file name
 */
QString MTRNN::getTrainingFile()
{
    return trainingFileName;
}

/*!
 * \brief Calculates the training progress in %.
 * \return progress - current progress
 */
int MTRNN::getProgress()
{
    int progress = (int)(((float)(iteration+1)/(float)maxIterations)*100);
    return progress;
}

/*!
 * \brief Gets current feedback interval.
 * \return feedbackInterval - feedback interval
 */
int MTRNN::getFeedbackInterval()
{
    return feedbackInterval;
}

/*!
 * \brief Returns neural network weights.
 * \return weights - neural network weights
 */
QVector<float> MTRNN::getWeights()
{
    QVector<float> weights;
    for(int i=0; i<numWeights; i++)
    {
        weights.append(weight_h[i]);
    }
    return weights;
}

/*!
 * \brief Forward pass.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] input - inputs
 * \param[in] weight - weights
 * \param[in] deltaT - delta-t values
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] activity - activities
 * \param[out] error - errors
 */
void MTRNN::forwardPass(int step, int sequenceOffset, float *activity, float *input, float *weight, float *previousPotential, float *potential, float *error, int *deltaT, int numNeurons, int numIONeurons)
{
    float sum = 0.0;

   // #pragma omp parallel for reduction(+: sum)
    for(int idx=0; idx<numNeurons; idx++)
    {
        potential[idx] = 0.0;

        //current membrabe potential on input-output neurons
        for(int j=0; j<numIONeurons; j++)
        {
            sum += input[sequenceOffset+(numIONeurons*(step-1))+j] * weight[(j*numNeurons)+idx];
        }

        //current membrabe potential on context neurons
        for(int j=numIONeurons; j<numNeurons; j++)
        {
            sum += activity[(numNeurons*(step-1))+j] * weight[(j*numNeurons)+idx];
        }

        //add bias to the activity
        sum += weight[(numNeurons*numNeurons)+idx];

        //calculate current membrane potential taking delta-t value as well as the previous membrane potential into account
        potential[idx] = ((1.0f-(1.0f/(float)deltaT[idx])) * previousPotential[idx]) + ((1.0f/(float)deltaT[idx])*sum);

        //save current membrane potential for the next time step where it will be used as the previous membrane potential
        previousPotential[idx] = potential[idx];

        //Sigmoid activation function
        activity[(numNeurons*step)+idx] = 1.0f/(1.0f+exp(-potential[idx]));

        //save error
        if(idx<numIONeurons)
        {
            error[(numNeurons*step)+idx] = activity[(numNeurons*step)+idx] - input[sequenceOffset+(numIONeurons*(step+1))+idx];
        }

        sum = 0.0;
    }
}

/*!
 * \brief Backward pass.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] input - inputs
 * \param[in] activity - activities
 * \param[in] error - errors
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] previousDelta - previous deltas
 * \param[out] mse - mean square error
 * \param[out] delta
 * \param[out] deltaWeight
 */
void MTRNN::backwardPass(int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *previousDelta, float *error, float *weight, float *deltaWeight, float *mse, int *deltaT)
{
    float sumIO  = 0.0;
    float sumHidden = 0.0;

    //#pragma omp parallel for reduction(+: sumIO, sumHidden)
    for(int idx=0; idx<numNeurons; idx++)
    {
        if(idx<numIONeurons) //deltas on IO neurons
        {
            sumIO += (input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]) * (input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]);
            delta[idx] = error[(numNeurons*step)+idx] + (1.0f - (1.0f/deltaT[idx])) * previousDelta[idx];
        }
        else //delatas on hidden neurons
        {
            int kroneckerDelta;

            for (int j=0; j<numNeurons; j++)
            {
                if(weight[(j*numNeurons)+idx]!=0)
                {
                    if (idx==j) kroneckerDelta = 1;
                    else		kroneckerDelta = 0;

                    //see bottom part of the equation 11 in Yamashita & Tani 2008 - SUM(k->N) of prev_hidden_delta * kroneckerDelta * (1-1/hidden-delta) + (1/delta-on-unit-k) * weight(ki) * derivative of activation of neuron i
                    sumHidden += previousDelta[j] * (kroneckerDelta * (1.0f - (1.0f/deltaT[idx])) + (1.0f/deltaT[j]) * weight[(idx*numNeurons)+j] * (activity[(numNeurons*step)+idx] * (1.0f - activity[(numNeurons*step)+idx])));
                }
            }
            delta[idx] = sumHidden;
            sumHidden = 0.0;
        }

        //deltas on weights on all connected neurons
        for(int j=0; j<numNeurons; j++)
        {
            if(weight[(j*numNeurons)+idx]!=0)
            {
                deltaWeight[(j*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx] * activity[(numNeurons*(step-1))+j];
            }
        }

        //deltas on bias weights
        deltaWeight[(numNeurons*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx];
    }

    //save deltas
    memcpy(previousDelta, delta, numNeurons * sizeof(float));

    *mse += sumIO/2;
}

/*!
 * \brief Updates weights.
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] numWeights - number of weights
 * \param[in] deltaWeight - delta weights
 * \param[out] previousDeltaWeight - previous delta weights
 * \param[out] weight - weights
 */
void MTRNN::updateWeights(float learningRate, float momentum, float *weight, float *deltaWeight, float *previousDeltaWeight, int numWeights)
{
   // #pragma omp parallel for
    for(int idx=0; idx<numWeights; idx++)
    {
        weight[idx] -= ((learningRate * deltaWeight[idx]) + (momentum * previousDeltaWeight[idx]));
        previousDeltaWeight[idx] = deltaWeight[idx];
    }
}

/*!
 * \brief Sets initial state on the first few slow neurons.
 * \param[in] initState - initial states
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] numFastNeurons - number of fast neurons
 * \param[in] zeroActivity - zero activity values
 * \param[out] activity - activities
 */
void MTRNN::setInitStates(float initState, float *activity, float *zeroActivity, int numNeurons, int numIONeurons, int numFastNeurons)
{
    memcpy(activity, zeroActivity, numNeurons*sizeof(float));

    for(int idx=numIONeurons+numFastNeurons; idx<numIONeurons+numFastNeurons+5; idx++)
    {
        activity[idx] = initState;
    }
}

/*!
 * \brief Resets deltas, errors and potentials.
 * \param[in] numNeurons - number of neurons
 * \param[in] maxSequenceSteps - maximum sequence steps
 * \param[in] zeroNeuron - zero neuron values
 * \param[in] zeroError - zero error values
 * \param[out] delta - deltas
 * \param[out] previousDelta - previous deltas
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] error - errors
 */
void MTRNN::resetParameters(int numNeurons, int maxSequenceSteps, float *delta, float *previousDelta, float *potential, float *previousPotential, float *error, float *zeroNeuron, float *zeroError)
{
    memcpy(delta, zeroNeuron, numNeurons*sizeof(float));
    memcpy(previousDelta, zeroNeuron, numNeurons*sizeof(float));
    memcpy(potential, zeroNeuron, numNeurons*sizeof(float));
    memcpy(previousPotential, zeroNeuron, numNeurons*sizeof(float));
    memcpy(error, zeroError, (numNeurons * maxSequenceSteps) * sizeof(float));
}

/*!
 * \brief Resets delta weights.
 * \param[in] numWeights - number of weights
 * \param[in] zeroWeight - zero weight values
 * \param[out] deltaWeight - delta weights
 */
void MTRNN::resetDeltaWeights(int numWeights, float *deltaWeight, float *zeroWeight)
{
    memcpy(deltaWeight, zeroWeight, numWeights*sizeof(float));
}

}
