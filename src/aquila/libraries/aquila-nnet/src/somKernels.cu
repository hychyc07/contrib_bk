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

#include <cuda.h>
#include <cuda_runtime.h>

/*!
 * \brief Calculates the euclidean distance between input vector and weight vector of a neuron and saves these distances in the output array.
 * \param[in] inputs - inputs
 * \param[in] weights - weights
 * \param[in] numInputs - number of inputs
 * \param[in] sequenceId - sequence id
 * \param[in] numOutputs - number of outputs
 * \param[out] outputs - outputs
 */
__global__ void propogateInputKernel(float *inputs, float *weights, float *outputs, int numInputs, int sequenceId, int numOutputs)
{
    //get the thread id
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

	if(idx<numOutputs)
	{
		//euclidean distance
		float distance = 0.0;

		//calculate the euclidean distance between each node's weight vector and the current input vector
		for(int i=0; i<numInputs; i++)
		{
			distance += (inputs[(sequenceId * numInputs) + i] - weights[idx+(numOutputs*i)]) * (inputs[(sequenceId * numInputs) + i] - weights[idx+(numOutputs*i)]);
		}

		distance = sqrtf(distance);

		//wait until all threads are finished
		__syncthreads();

		//assign the euclidean distance as the output having a particular index
		outputs[idx] = distance;
	}
}

/*!
 * \brief Finds the best matching unit having the lowest euclidean distance - part 1/2.
 * \param[in] outputs - outputs
 * \param[in] numOutputs - number of outputs
 * \param[out] winner - winner
 */
__global__ void findBestMatchPass1Kernel(float *outputs,  int *winner, int numOutputs)
{
    //get local thread id
    int tid  = threadIdx.x;

    //get global thread id
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

    if(idx<numOutputs)
    {
    	//initialise
        winner[idx] = idx;
    	
        //synchronise threads to make sure all data was loaded
        __syncthreads();

        for(int s=1; s<blockDim.x; s*=2)
    	{
        	int index = 2 * s * tid;

            if(index<blockDim.x && (blockIdx.x*blockDim.x)+index+s<numOutputs)
        	{
                if(outputs[winner[(blockIdx.x*blockDim.x)+index]] > outputs[winner[(blockIdx.x*blockDim.x)+index+s]])
                {
                    winner[(blockIdx.x*blockDim.x)+index] = winner[(blockIdx.x*blockDim.x)+index+s];
                }
        	}

        	//wait until all threads are finished with current pass
        	__syncthreads();
    	}
    }
}

/*!
 * \brief Finds the best matching unit having the lowest euclidean distance - part 2/2.
 * \param[in] outputs - outputs
 * \param[in] numOutputs - number of outputs
 * \param[out] winner - winner
 */
__global__ void findBestMatchPass2Kernel(float *outputs, int *winner, int numOutputs)
{
    for(int s=1; s<blockDim.x; s*=2)
    {
        int index = 2*s*threadIdx.x;

        if(index<blockDim.x && 2*(s*(threadIdx.x*blockDim.x))+(s*blockDim.x)<numOutputs)
        {
            if(outputs[winner[2*(s*(threadIdx.x*blockDim.x))]] > outputs[winner[2*(s*(threadIdx.x*blockDim.x))+(s*blockDim.x)]])
			{
				winner[2*(s*(threadIdx.x*blockDim.x))] = winner[2*(s*(threadIdx.x*blockDim.x))+(s*blockDim.x)];
			}
        }

        //wait until all threads are finished with current pass
        __syncthreads();
    }

    if(threadIdx.x==0)
    {
        //find out X and Y coordinates and save them in activity_index[1] and activity_index[2] respectively
        int Y = -1;
        int X = int(winner[0]);

        int dim = int(sqrtf((float)numOutputs));

        while(X>-1)
        {
            X -= dim;
            Y ++;
        }

        //assign X,Y possitions
        winner[1] = X + dim;
        winner[2] = Y;
    }
    __syncthreads();
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
__global__ void updateWeightsKernel(float *inputs, float *weights, int *winner, float sigma, int numInputs, int sequenceId, int numOutputs, int neighbourhoodSize, float initLearningRate,int numIterations, int currentIteration)
{
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

	if(idx<numOutputs)
	{
		//dimension of the map
		int dim = int(sqrtf((float)numOutputs));

		//euclidean distance
		float distance = 0.0;

		//current learning rate
		float currentLearningRate = 0.0;

		//convert current index to x,y
		int curY = -1;
		int curX = idx;
		do{
			curX -= dim;
			curY ++;
        }while(curX>-1);
		curX += dim;

		//calculate the distance from winner to current in the matrix
        distance = (float)((curX-winner[1])*(curX-winner[1]))+((curY-winner[2])*(curY-winner[2]));

        if(distance>(neighbourhoodSize*neighbourhoodSize))
		{
			distance = 0.0;
		}
		else
		{
            if(sigma>0.0)
			{
                float s = sigma*expf(-(float)currentIteration/numIterations);
                distance = expf(-(distance)/(2*(s*s)));
			}
			else
			{
                distance = expf(-(distance)/(2*dim));
			}
		}

		//adjust the learning rate
        currentLearningRate = initLearningRate*__expf(-(float)currentIteration/numIterations);

		//update weights
		for(int i=0; i<numInputs; i++) 
		{
            weights[idx+(numOutputs*i)] += (inputs[(sequenceId*numInputs)+i]-weights[idx+(numOutputs*i)])*distance*currentLearningRate;
		}	
	}
}

/*!
 * \brief Wrapper for findBestMatchPass1Kernel and findBestMatchPass2Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] outputs - outputs
 * \param[in] numOutputs - number of outputs
 * \param[out] winner - winner
 */
void findBestMatchOnDevice(dim3 grid, dim3 block, float *outputs,  int *winner, int numOutputs)
{
    findBestMatchPass1Kernel<<<grid,block>>>(outputs, winner, numOutputs);
    findBestMatchPass2Kernel<<<1,block>>>(outputs, winner, numOutputs);
}

/*!
 * \brief Wrapper for propogateInputKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] inputs - inputs
 * \param[in] weights - weights
 * \param[in] numInputs - number of inputs
 * \param[in] sequenceId - sequence id
 * \param[in] numOutputs - number of outputs
 * \param[out] outputs - outputs
 */
void propogateInputOnDevice(dim3 grid, dim3 block, float *inputs, float *weights, float *outputs, int numInputs, int sequenceId, int numOutputs)
{
    propogateInputKernel<<<grid,block>>>(inputs, weights, outputs, numInputs, sequenceId, numOutputs);
}

/*!
 * \brief Wrapper for updateWeightsKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
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
void updateWeightsOnDevice(dim3 grid, dim3 block, float *inputs, float *weights, int *winner, float sigma, int numInputs, int sequenceId, int numOutputs, int neighbourhoodSize, float initLearningRate,int numIterations, int currentIteration)
{
    updateWeightsKernel<<<grid,block>>>(inputs, weights, winner, sigma, numInputs, sequenceId, numOutputs, neighbourhoodSize, initLearningRate, numIterations, currentIteration);
}
