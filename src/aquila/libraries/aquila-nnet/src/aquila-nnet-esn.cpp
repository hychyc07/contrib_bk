//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse																																				//
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
#include "aquila-nnet-esn.h"
#include "esnKernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

namespace aquila
{
/*!
 * \brief Constructor.
 */
ESN::ESN()
{
    gpu = new GPU();
    math = new Math();
}

/*!
 * \brief Destructor.
 */
ESN::~ESN()
{
}

/*!
 * \brief Initialialises everything and starts the training.
 */
void ESN::initialise()
{
    //seed the random number generator
    math->setSeed(5);

    //work out how big everything is
    inputActivitySize = ESN_INPUT_SIZE * sizeof(float);
    activitySize = esnSize * sizeof(float);
    outputActivitySize = ESN_OUTPUT_SIZE * sizeof(float);
    inputWeightsSize = ESN_INPUT_SIZE * esnSize * sizeof(float);
    weightsSize = esnSize * esnSize * sizeof(float);
    outputWeightsSize = ESN_OUTPUT_SIZE * esnSize * sizeof(float);

    //allocate host memory
    weights = (float*)malloc(weightsSize);
    activity = (float*)malloc(activitySize);
    newActivity = (float*)malloc(activitySize);
    inputWeights = (float*)malloc(inputWeightsSize);
    inputActivity = (float*)malloc(inputActivitySize);
    newInputActivity = (float*)malloc(inputActivitySize);
    outputWeights = (float*)malloc(outputWeightsSize);
    outputActivity = (float*)malloc(outputActivitySize);
    newOutputActivity = (float*)malloc(outputActivitySize);
    PERCinputWeights = (float*)malloc(inputWeightsSize);
    PERCoutputWeights = (float*)malloc(outputWeightsSize);
    PERCoutputTarget = (float*)malloc(outputActivitySize);
    PERCinputTarget = (float*)malloc(inputActivitySize);

    //set sparce weights to random values
    for(int i=0; i<(esnSize * esnSize); i++)
    {
        if(math->getRandomIntegerUpTo(100) < ESN_SPARCE)
        {
            weights[i] = ((float(rand()%20000)/10000)-1) * (float)ESN_STRENGTH;
        }
        else
        {
            weights[i] = 0.0f;
        }
    }

    for(int i=0; i<(ESN_INPUT_SIZE * esnSize); i++)
    {
        if(math->getRandomIntegerUpTo(100) < ESN_SPARCE)
        {
            inputWeights[i] = ((float(rand()%20000)/10000)-1) * (float)ESN_STRENGTH;
        }
        else
        {
            inputWeights[i] = 0.0f;
        }
    }

    for(int i=0; i<(ESN_OUTPUT_SIZE * esnSize); i++)
    {
        if(math->getRandomIntegerUpTo(100) < ESN_SPARCE)
        {
            outputWeights[i] = ((float(rand()%20000)/10000)-1) * (float)ESN_STRENGTH;
        }
        else
        {
            outputWeights[i] = 0.0f;
        }
    }

    for(int i=0; i<(ESN_INPUT_SIZE * esnSize); i++)
    {
        PERCinputWeights[i] = ((float(rand()%20000)/10000)-1) * (float)ESN_STRENGTH;
    }

    for(int i=0; i<(ESN_OUTPUT_SIZE * esnSize); i++)
    {
        PERCoutputWeights[i] = ((float(rand()%20000)/10000)-1);
    }

    //set current activity to zero
    for(int i=0; i<esnSize; i++)
    {
        activity[i] = 0.0f;
        newActivity[i] = 0.0f;
    }

    //set input source
    inputSource = 0;

    running = true;
}

/*!
 * \brief Thread loop.
 */
void ESN::run()
{
    initialise();
    emit started(1);

    while(running)
    {
        //get input data and set the training targets for the perceptrons
        for(int i=0; i<ESN_INPUT_SIZE; i++)
        {
            if(inputSource==0)
            {
                inputActivity[i] = ((float(rand()%20000)/10000)-1);
            }
            else if(inputSource==1)
            {
                inputActivity[i] = 1.0;
            }
            else if(inputSource==2)
            {
                inputActivity[i] = 0.0;
            }
        }

        for(int i=0; i<ESN_OUTPUT_SIZE; i++)
        {
            PERCoutputTarget[i] = 0;
        }

        if(inputSource>0)
        {
            PERCoutputTarget[inputSource-1] = 1.0;
        }

        //input-to-ESN, ESN-to-ESN, output-to-ESN...
        calculateActivity(inputActivity, ESN_INPUT_SIZE, inputWeights, newActivity, esnSize);
        calculateActivity(activity, esnSize, weights, newActivity, esnSize);
        calculateActivity(outputActivity, ESN_OUTPUT_SIZE, outputWeights, newActivity, esnSize);

        //ESN-to-output, ESN-to-input...
        calculateActivity(activity, esnSize, PERCoutputWeights, newOutputActivity, ESN_OUTPUT_SIZE);

        //update activities to new time step
        finishStep(newActivity, activity, esnSize, newOutputActivity, outputActivity, ESN_OUTPUT_SIZE, newInputActivity, inputActivity, ESN_INPUT_SIZE, 1, 0); //CLAMP_INPUT, CLAMP_OUTPUT);

        //train Perceptron weights
        PERCweightsUpdate(activity, outputActivity, PERCoutputWeights, PERCoutputTarget, (float)LEARNING_RATE, ESN_OUTPUT_SIZE, esnSize);
    }

    emit stopped(2);
}

/*!
 * \brief Stops the thread.
 */
void ESN::stop()
{
    running = false;
}

/*!
 * \brief Performes basic neural network functions.
 * \param[in] activity - ESN activity
 * \param[in] N - number of units
 * \param[in] weights - ESN weights
 * \param[in] newActivity - new ESN activity
 * \param[in] noElements - number of elements
 * \note new_ai = sum(ai * wij) :Note: sigmoid is elswhere
 */
void ESN::calculateActivity(float *activity, int N, float *weights, float *newActivity, int noElements)
{
    for(int i=0; i<noElements; i++)
    {
        for(int j=0; j<N; j++)
        {
            newActivity[i] += activity[j] * weights[i+(noElements*j)];
        }
    }
}

/*!
 * \brief Performes a simple perceptron based learning function.
 * \param[in] ESNactivity - ESN activity
 * \param[in] PERCactivity - perceptron activity
 * \param[in] target - target training source
 * \param[in] learningRate - learning rate
 * \param[in] noPercUnits - number of perceptron units
 * \param[in] ESNsize - ESN size
 * \param[out] PERCweights - perceptron weights
 */
void ESN::PERCweightsUpdate(float *ESNactivity, float *PERCactivity, float *PERCweights, float *target, float learningRate, int noPercUnits, int ESNsize)
{
    for(int i=0; i<noPercUnits; i++)
    {
        for(int j=0; j<ESNsize; j++)
        {
            //NOTE: no bias implemented yet
            PERCweights[i+(noPercUnits*j)] += learningRate * (1 - (PERCactivity[i] * (1 - PERCactivity[i]))) * (target[i] - PERCactivity[i]) * activity[j];
        }
    }
}

/*!
 * \brief Completes the ESN neural update by copying new_activities to current and applying a sigmoid function.
 * \note This function uses tanh for the ESN and 1/1+(e-a) for the rest
 * \param[in] noElements - number of elements
 * \param[in] noOutElements - number of output elements
 * \param[in] noInputElements - number of input elements
 * \param[in] inputClamp - input clamping
 * \param[in] outputClamp - output clamping
 * \param[out] newActivity - new activity
 * \param[out] activity - activity
 * \param[out] newOutActivity - new output activity
 * \param[out] outActivity - output activity
 * \param[out] newInputActivity - new input activity
 * \param[out] inputActivity - input activity
 */
void ESN::finishStep (float *newActivity, float *activity, int noElements, float *newOutActivity, float *outActivity, int noOutElements, float *newInputActivity, float *inputActivity, int noInputElements, int inputClamp, int outputClamp)
{
    QVector<float> activityOut;
    for(int i=0; i<noElements; i++) //ESN
    {
        activity[i] = (float)tanh((double)newActivity[i]);
        activityOut.push_back(activity[i]);
        newActivity[i] = 0.0;
    }
    emit activitySet(activityOut);

    if(!outputClamp) //output
    {
        for(int i=0; i<noOutElements; i++)
        {
            outActivity[i] = (float)(1/(1+exp((double)-newOutActivity[i])));
            newOutActivity[i] = 0.0;
        }
    }

    if(!inputClamp)
    {
        for(int i=0; i<noInputElements; i++) //input
        {
            inputActivity[i] = (float)(1/(1+exp((double)-newInputActivity[i])));
            newInputActivity[i] = 0.0;
        }
    }
}

/*!
 * \brief Sets ESN size.
 * \param[in] size - ESN size
 */
void ESN::setSize(int size)
{
    esnSize = size;
}

/*!
 * \brief Sets the input source.
 * \param[in] source - input source
 */
void ESN::setInputSource(int source)
{
    inputSource = source;
}

/*!
 * \brief Gets ESN size.
 * \return esnSize - ESN size
 */
int ESN::getSize()
{
    return esnSize;
}

/*!
 * \brief Gets input source size.
 * \return inputSource - input source
 */
int ESN::getInputSource()
{
    return inputSource;
}

}
