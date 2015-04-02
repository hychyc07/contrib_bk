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
 * \brief Forward pass.
 * \note Used with forwardPassV21Kernel - faster version for networks of up to 1024 neurons.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] activity - activations
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] buffer - buffer used for storing new activations
 */
__global__ void forwardPassV2Kernel(int step, int sequenceOffset, float *activity, float *input, float *weight, int numNeurons, int numIONeurons, float *buffer)
{
    int idx = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx<numNeurons && j<numNeurons)
    {
        if(j<numIONeurons)
        {
            buffer[(j*numNeurons)+idx] = input[sequenceOffset+(numIONeurons*(step-1))+j] * weight[(j*numNeurons)+idx];
        }
        else
        {
            buffer[(j*numNeurons)+idx] = activity[(numNeurons*(step-1))+j] * weight[(j*numNeurons)+idx];
        }
    }
}

/*!
 * \brief Forward pass.
 * \note Used with forwardPassV2Kernel. Faster version for networks of up to 1024 neurons.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] deltaT - delta-t values
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] activity - activities
 * \param[out] error - errors
 */
__global__ void forwardPassV21Kernel(int step, int sequenceOffset, float *activity, float *input, float *buffer, float *potential, float *weight, float *previousPotential, float *error, int *deltaT, int numNeurons, int numIONeurons)
{
    extern __shared__ float sdata[];

    int tid = threadIdx.x;
    int i = blockDim.x/2;

    //init to zero
    sdata[tid] = 0.0;

    if(tid < numNeurons)
    {
        //load data
        sdata[tid] = buffer[(tid * numNeurons)+blockIdx.x];
        __syncthreads();

        while (i != 0)
        {
            if(tid < i)
            {
                //each thread puts its local sum into shared memory
                sdata[tid] += sdata[tid + i];
                __syncthreads();
            }
            i /= 2;
        }

        //write result for this block to global mem
        if (tid == 0)
        {
            potential[blockIdx.x] = sdata[0];

            //add bias to the activity
            potential[blockIdx.x] += weight[(numNeurons*numNeurons)+blockIdx.x];

            //calculate current membrane potential taking delta-t value as well as the previous membrane potential into account
            potential[blockIdx.x] = ((1.0f-(1.0f/(float)deltaT[blockIdx.x])) * previousPotential[blockIdx.x]) + ((1.0f/(float)deltaT[blockIdx.x])*potential[blockIdx.x]);

            //save current membrane potential for the next time step where it will be used as the previous membrane potential
            previousPotential[blockIdx.x] = potential[blockIdx.x];

            //sigmoid activation
            activity[(numNeurons*step)+blockIdx.x] = 1.0f/(1.0f+__expf(-potential[blockIdx.x]));

            //save error
            if(blockIdx.x<numIONeurons)
            {
                error[(numNeurons*step)+blockIdx.x] = activity[(numNeurons*step)+blockIdx.x] - input[sequenceOffset+(numIONeurons*(step+1))+blockIdx.x];
            }
        }
    }
}

/*!
 * \brief Forward pass.
 * \note Slower version for larger networks.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] deltaT - delta-t values
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] activity - activities
 * \param[out] error - errors
 */
__global__ void forwardPassV1Kernel(int step, int sequenceOffset, float *activity, float *input, float *weight, float *previousPotential, float *error, float *potential, int *deltaT, int numNeurons, int numIONeurons)
{
    int idx = blockIdx.x*blockDim.x + threadIdx.x;
    float sum = 0.0;

    if(idx<numNeurons)
    {
        potential[idx] = 0.0;

        //calculates membrabe potential of a particular neuron given by the idx index
        for(int i=0; i<numNeurons; i++)
        {
            if(i<numIONeurons)
            {
                sum += input[sequenceOffset+(numIONeurons*(step-1))+i] * weight[(i*numNeurons)+idx];
            }
            else
            {
                sum += activity[(numNeurons*(step-1))+i] * weight[(i*numNeurons)+idx];
            }
        }

        //add bias to the activity
        sum += weight[(numNeurons*numNeurons)+idx];

        //calculate current membrane potential taking delta-t value as well as the previous membrane potential into account
        potential[idx] = ((1.0f-(1.0f/(float)deltaT[idx])) * previousPotential[idx]) + ((1.0f/(float)deltaT[idx])*sum);

        //save current membrane potential for the next time step where it will be used as the previous membrane potential
        previousPotential[idx] = potential[idx];

        //Sigmoid activation function
        activity[(numNeurons*step)+idx] = 1.0f/(1.0f+__expf(-potential[idx]));

        //save error
        if(idx<numIONeurons)
        {
            error[(numNeurons*step)+idx] = activity[(numNeurons*step)+idx] - input[sequenceOffset+(numIONeurons*(step+1))+idx];
        }

        sum = 0.0;
    }
}

/*!
 * \brief Calculates deltas, deltas on weights and errors parts.
 * \note Slower version for larger networks over 1024 neurons.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] input - input
 * \param[in] activity - activities
 * \param[in] error - errors
 * \param[in] individualError - error buffer
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] previousDelta - previous deltas
 * \param[out] delta - deltas
 * \param[out] deltaWeight - delta weights
 */
__global__ void backwardPassV1Kernel(int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight)
{
    int idx = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;

    if(idx<numIONeurons && j<numNeurons)
    {
        if(j==0)
        {
            individualError[idx] += ((input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]) * (input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]))/2.0f;
        }

        delta[idx] = error[(numNeurons*step)+idx] + (1.0f - (1.0f/deltaT[idx])) * previousDelta[idx];

        if(weight[(j*numNeurons)+idx]!=0)
        {
            deltaWeight[(j*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx] * activity[(numNeurons*(step-1))+j];
        }
    }
}

/*!
 * \brief Calculates deltas on hidden neurons.
 * \note Slower version for larger networks over 1024 neurons.
 * \param[in] step - current step
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-ouput neurons
 * \param[in] activity - activities
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] previousDelta - previous deltas
 * \param[out] delta - deltas
 */
__global__ void backwardPassV11Kernel(int step,  int numNeurons, int numIONeurons,  float *activity, float *delta, float *previousDelta, int *deltaT, float *weight)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int kroneckerDelta;

    if(idx<numNeurons && idx>=numIONeurons)
    {
        delta[idx] = 0.0;

        for(int j=0; j<numNeurons; j++)
        {
            if(weight[(j*numNeurons)+idx]!=0)
            {
                //set Kronecker's delta
                if (idx==j)
                {
                    kroneckerDelta = 1;
                }
                else
                {
                    kroneckerDelta = 0;
                }

                //see bottom part of the equation 11 in Yamashita & Tani 2008 - SUM(k->N) of prev_hidden_delta * kroneckerDelta * (1-1/hidden-delta) + (1/delta-on-unit-k) * weight(ki) * derivative of activation of neuron i
                delta[idx] += previousDelta[j] * (kroneckerDelta * (1.0f - (1.0f/deltaT[idx])) + (1.0f/deltaT[j]) * weight[(idx*numNeurons)+j] * (activity[(numNeurons*step)+idx] * (1.0f - activity[(numNeurons*step)+idx])));
            }
        }
    }
}

/*!
 * \brief Loads the buffer with deltas on weights fractions calculate deltas, deltas on weights and errors parts.
 * \note Deltas on weights and errors are later summed by parallel reduction.
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] input - input
 * \param[in] activity - activities
 * \param[in] previousDelta - previous deltas
 * \param[in] error - errors
 * \param[in] individualError - error buffer
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[out] delta - deltas
 * \param[out] deltaWeight - delta weights
 * \param[out] buffer - buffer used for storing delta weights
 */
__global__ void backwardPassV2Kernel(int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight, float *buffer)
{
    int idx = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;
    int kroneckerDelta;

    if(idx<numNeurons && j<numNeurons)
    {
        if(idx<numIONeurons)
        {
            if(j==0)
            {
                individualError[idx] += ((input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]) * (input[sequenceOffset+(numIONeurons*(step+1))+idx] - activity[(numNeurons*step)+idx]))/2.0f;
            }

            delta[idx] = error[(numNeurons*step)+idx] + (1.0f - (1.0f/deltaT[idx])) * previousDelta[idx];

            if(weight[(j*numNeurons)+idx]!=0)
            {
                //see eqaution 10 in Yamashita & Tani 2008 - SUM(t) of (1/fast-unit-delta)*delta-output(t) * fast-neuron-activity(t-1)
                deltaWeight[(j*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx] * activity[(numNeurons*(step-1))+j];
            }
        }
        else //load the buffer with deltas on weights fractions
        {
            buffer[(j*numNeurons)+idx] = 0.0;

            //calculate delta on weight if the two neurons are connected
            if(weight[(j*numNeurons)+idx]!=0)
            {
                //set Kronecker's delta
                if (idx==j)
                {
                    kroneckerDelta = 1;
                }
                else
                {
                    kroneckerDelta = 0;
                }

                //see bottom part of the equation 11 in Yamashita & Tani 2008 - SUM(k->N) of prev_hidden_delta * kroneckerDelta * (1-1/hidden-delta) + (1/delta-on-unit-k) * weight(ki) * derivative of activation of neuron i
                buffer[(j*numNeurons)+idx] = previousDelta[j] * (kroneckerDelta * (1.0f - (1.0f/deltaT[idx])) + (1.0f/deltaT[j]) * weight[(idx*numNeurons)+j] * (activity[(numNeurons*step)+idx] * (1.0f - activity[(numNeurons*step)+idx])));
            }
        }
    }
}

/*!
 * \brief Calculates deltas on hidden neurons.
 * \note Used together with backwardPassV2Kerne. Faster version for networks of up to 1024 neurons.
 * \param[in] input - input
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] output - output
 */
__global__ void backwardPassV21Kernel(float *input, float *output, int numNeurons, int numIONeurons)
{
    extern __shared__ float sdata[];

    int tid = threadIdx.x;
    int i = blockDim.x/2;

    //init to zero
    sdata[tid] = 0.0;

    if(tid<numNeurons)
    {
        //load data
        sdata[tid] = input[(tid * numNeurons)+(numIONeurons+blockIdx.x)];
        __syncthreads();

        while(i!=0)
        {
            if(tid<i)
            {
                //each thread puts its local sum into shared memory
                sdata[tid] += sdata[tid+i];
                __syncthreads();
            }
            i/=2;
        }

        //write result for this block to global memory
        if(tid==0)
        {
            output[numIONeurons+blockIdx.x] = sdata[0];
        }
    }
}

/*!
 * \brief Calculates deltas on weights on hidden neurons and biases.
 * \param[in] step - current step
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] activity - activities
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] delta - deltas
 * \param[out] previousDelta - previous deltas
 * \param[out] deltaWeight - delta weights
 */
__global__ void backwardPassV3Kernel(int step, int numNeurons, int numIONeurons, float *activity, float *delta, float *previousDelta, float *deltaWeight, int *deltaT, float *weight)
{
    int idx = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;

    if(j<numNeurons && idx<numNeurons)
    {
        //calculate delta on weights for those neurons that are directly connected to hidden neurons
        if(idx>=numIONeurons)
        {
            if(weight[(j*numNeurons)+idx]!=0)
            {
                //see eqaution 10 in Yamashita & Tani 2008 - SUM(t) of (1/fast-unit-delta)*delta-output(t) * fast-neuron-activity(t-1)
                deltaWeight[(j*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx] * activity[(numNeurons*(step-1))+j];
            }
        }

        //calculate deltas on bias weights connecting to each neuron
        if(j==0)
        {
            deltaWeight[(numNeurons*numNeurons)+idx] += (1.0f/deltaT[idx]) * delta[idx];
            previousDelta[idx] = delta[idx];
        }
    }
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
__global__ void updateWeightsKernel(float learningRate, float momentum, float *weight, float *deltaWeight, float *previousDeltaWeight, int numWeights)
{
    int idx = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;

    if(idx<numWeights)
    {
        weight[idx] -= ((learningRate * deltaWeight[idx]) + (momentum * previousDeltaWeight[idx]));
        previousDeltaWeight[idx] = deltaWeight[idx];
    }
}

/*!
 * \brief Sums delta weights on the master device.
 * \param[in] numWeights - number of weights
 * \param[in] peerDeltaWeight - delta weights from peer device
 * \param[out] masterDeltaWeight - delta weights from master device
 */
__global__ void sumDeltaWeightsP2PKernel(int numWeights, float *masterDeltaWeight, float *peerDeltaWeight)
{
    int idx = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;

    if(idx<numWeights)
    {
        masterDeltaWeight[idx] += peerDeltaWeight[idx];
    }
}

/*!
 * \brief Modifies weights on the master device and copies to the peer device.
 * \param[in] numWeights - number of weights
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] deltaWeight - delta weights
 * \param[out] previousDeltaWeight - previous delta weights
 * \param[out] masterWeight - weigths from master device
 * \param[out] peerWeight - weights from peer device
 */
__global__ void updateWeightsP2PKernel(int numWeights, float learningRate, float momentum, float *masterWeight, float *peerWeight, float *deltaWeight, float *previousDeltaWeight)
{
    int idx = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;

    if(idx<numWeights)
    {
        masterWeight[idx] -= ((learningRate * deltaWeight[idx]) + (momentum * previousDeltaWeight[idx]));
        peerWeight[idx] = masterWeight[idx];
        previousDeltaWeight[idx] = deltaWeight[idx];
    }
}

/*!
 * \brief Modifies weights on the master device and copies to the peer device.
 * \param[in] peerError - error from peer device
 * \param[out] masterError - error from master device
 */
__global__ void sumErrorP2PKernel(float *masterError, float *peerError)
{
    if(threadIdx.x==0)
    {
        masterError[0] += peerError[0];
    }
}

/*!
 * \brief Sets the initial states for all the units on device.
 * \note Slow context units will be initialised with a specific value.
 * \param[in] initState - initial state
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] numFastNeurons - number of fast neurons
 * \param[out] activity - activities
 */
__global__  void setInitStatesKernel(float initState, float *activity, int numNeurons, int numIONeurons, int numFastNeurons)
{
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

    if(idx<numNeurons)
    {
        if(idx>=numIONeurons+numFastNeurons && idx<numIONeurons+numFastNeurons+5)
        {
            activity[idx] = initState;
        }
        else
        {
            activity[idx] = 0.0;
        }
    }
}

/*!
 * \brief Resets delta and error parameters.
 * \param[in] numNeurons - number of neurons
 * \param[in] maxSequenceSteps - maximum number of sequence steps
 * \param[out] delta - deltas
 * \param[out] previousDelta - previous deltas
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] error - errors
 */
__global__ void resetParametersKernel(int numNeurons, int maxSequenceSteps, float *delta, float *previousDelta, float *potential, float *previousPotential, float *error)
{
    int idx = blockIdx.x*blockDim.x + threadIdx.x;

    if(idx<(numNeurons*maxSequenceSteps))
    {
        if(idx<numNeurons)
        {
            delta[idx] = 0.0;
            previousDelta[idx] = 0.0;
            potential[idx] = 0.0;
            previousPotential[idx] = 0.0;
        }

        error[idx] = 0.0;
    }
}

/*!
 * \brief Resets delta weights and errors.
 * \param[in] numWeights - number of weights
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] deltaWeight - delta weights
 * \param[out] individualError - error buffer
 */
__global__ void resetDeltaWeightsKernel(int numWeights, int numIONeurons, float *deltaWeight, float *individualError)
{
    int idx = (blockIdx.y * gridDim.x + blockIdx.x) * blockDim.x + threadIdx.x;

    if(idx<numWeights)
    {
        deltaWeight[idx] = 0.0;

        if(idx<numIONeurons)
        {
            individualError[idx] = 0.0;
        }
    }
}

/*!
 * \brief Parallel reduction sum modified from NVIDIA SDK.
 * \note Number of threads are not known at a compile time, however, we always stick to power of 2 sizes
 * \note so here we are using templates to allow compilation for all known size, which results in higher throughput.
 * \param[in] input - input
 * \param[in] n - number of elements to sum
 * \param[in] nIsPow2 - determines if the number is of power of two
 * \param[out] output - output
 */
template <unsigned int blockSize> __global__ void reduceKernel(float *input, float *output, unsigned int n, bool nIsPow2)
{
    extern __shared__ float sdata[];

    unsigned int tid = threadIdx.x;
    unsigned int i = blockIdx.x*blockSize*2 + threadIdx.x;
    unsigned int gridSize = blockSize*2*gridDim.x;

    float sum = 0.0;

    //perform first level of reduction, reading from global memory, writing to shared memory
    //we reduce multiple elements per thread.  The number is determined by the number of active thread blocks (via gridDim).
    //More blocks will result in a larger gridSize and therefore fewer elements per thread
    while(i<n)
    {
        sum += input[i];

        //ensure we don't read out of bounds -- this is optimised away for powerOf2 sized arrays
        if (nIsPow2 || (i+blockSize)<n)
        {
            sum += input[i+blockSize];
        }

        i+=gridSize;
    }

    //each thread puts its local sum into shared memory
    sdata[tid] = sum;
    __syncthreads();

    //do reduction in shared mem
    if (blockSize >= 512) { if (tid < 256) { sdata[tid] = sum = sum + sdata[tid + 256]; } __syncthreads(); }
    if (blockSize >= 256) { if (tid < 128) { sdata[tid] = sum = sum + sdata[tid + 128]; } __syncthreads(); }
    if (blockSize >= 128) { if (tid <  64) { sdata[tid] = sum = sum + sdata[tid +  64]; } __syncthreads(); }

    if(tid<32)
    {
        //now that we are using warp-synchronous programming (below) we need to declare our shared memory
        //volatile so that the compiler doesn't reorder stores to it and induce incorrect behavior
        volatile float* smem = sdata;
        if (blockSize>=64)
        {
            smem[tid] = sum = sum + smem[tid+32];
        }

        if (blockSize>=32)
        {
            smem[tid] = sum = sum + smem[tid+16];
        }

        if (blockSize>=16)
        {
            smem[tid] = sum = sum + smem[tid+8];
        }

        if (blockSize>=8)
        {
            smem[tid] = sum = sum + smem[tid+4];
        }

        if (blockSize>=4)
        {
            smem[tid] = sum = sum + smem[tid+2];
        }

        if (blockSize>=2)
        {
            smem[tid] = sum = sum + smem[tid+1];
        }
    }

    //write result for this block to global mem
    if(tid==0)
    {
        output[blockIdx.x] = sdata[0];
    }
}

/*!
 * \brief Wrapper for resetDeltaWeightsKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] numWeights - number of weights
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] deltaWeight - delta weights
 * \param[out] individualError - error buffer
 */
void resetDeltaWeightsOnDevice(dim3 grid, dim3 block, cudaStream_t stream, int numWeights, int numIONeurons, float *deltaWeight, float *individualError)
{
    resetDeltaWeightsKernel<<<grid,block,0,stream>>>(numWeights, numIONeurons, deltaWeight, individualError);
}

/*!
 * \brief Wrapper for setInitStatesKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] initState - initial state
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] numFastNeurons - number of fast neurons
 * \param[out] activity - activities
 */
void setInitStatesOnDevice(dim3 grid, dim3 block, cudaStream_t stream, float initState, float *activity, int numNeurons, int numIONeurons, int numFastNeurons)
{
    setInitStatesKernel<<<grid,block,0,stream>>>(initState, activity, numNeurons, numIONeurons, numFastNeurons);
}

/*!
 * \brief Wrapper for resetParametersKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] numNeurons - number of neurons
 * \param[in] maxSequenceSteps - maximum number of sequence steps
 * \param[out] delta - deltas
 * \param[out] previousDelta - previous deltas
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] error - errors
 */
void resetParametersOnDevice(dim3 grid, dim3 block, cudaStream_t stream, int numNeurons, int maxSequenceSteps, float *delta, float *previousDelta, float *potential, float *previousPotential, float *error)
{
    resetParametersKernel<<<grid,block,0,stream>>>(numNeurons, maxSequenceSteps, delta, previousDelta, potential, previousPotential, error);
}

/*!
 * \brief Wrapper for updateWeightsKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] numWeights - number of weights
 * \param[in] deltaWeight - delta weights
 * \param[out] previousDeltaWeight - previous delta weights
 * \param[out] weight - weights
 */
void updateWeightsOnDevice(dim3 grid, dim3 block, float learningRate, float momentum, float *weight, float *deltaWeight, float *previousDeltaWeight, int numWeights)
{
    updateWeightsKernel<<<grid,block>>>(learningRate, momentum, weight, deltaWeight, previousDeltaWeight, numWeights);
}

/*!
 * \brief Wrapper for forwardPassV1Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] deltaT - delta-t values
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] potential - potentials
 * \param[out] previousPotential - previous potentials
 * \param[out] activity - activities
 * \param[out] error - errors
 */
void forwardPassV1onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *weight, float *previousPotential, float *error, float *potential, int *deltaT, int numNeurons, int numIONeurons)
{
    forwardPassV1Kernel<<<grid,block,0,stream>>>(step, sequenceOffset, activity, input, weight, previousPotential, error, potential, deltaT, numNeurons, numIONeurons);
}

/*!
 * \brief Wrapper for forwardPassV2Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] activity - activations
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] buffer - buffer used for storing new activations
 */
void forwardPassV2onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *weight, int numNeurons, int numIONeurons, float *buffer)
{
    forwardPassV2Kernel<<<grid,block,0,stream>>>(step, sequenceOffset, activity, input, weight, numNeurons, numIONeurons, buffer);
}

/*!
 * \brief Wrapper for forwardPassV21Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] smemSize - CUDA shared memory size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] activity - activations
 * \param[in] input - input
 * \param[in] weight - weights
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] buffer - buffer used for storing new activations
 */
void forwardPassV21onDevice(dim3 grid, dim3 block, int smemSize, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *buffer, float *potential, float *weight, float *previousPotential, float *error, int *deltaT, int numNeurons, int numIONeurons)
{
    forwardPassV21Kernel<<<grid,block,smemSize,stream>>>(step, sequenceOffset, activity, input, buffer, potential, weight, previousPotential, error, deltaT, numNeurons, numIONeurons);
}

/*!
 * \brief Wrapper for backwardPassV1Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] input - input
 * \param[in] activity - activities
 * \param[in] error - errors
 * \param[in] individualError - error buffer
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] previousDelta - previous deltas
 * \param[out] delta - deltas
 * \param[out] deltaWeight - delta weights
 */
void backwardPassV1onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight)
{
    backwardPassV1Kernel<<<grid,block,0,stream>>>(step, sequenceOffset, numNeurons, numIONeurons, input, activity, delta, deltaWeight, previousDelta, error, individualError, deltaT, weight);
}

/*!
 * \brief Wrapper for backwardPassV11Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-ouput neurons
 * \param[in] activity - activities
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] previousDelta - previous deltas
 * \param[out] delta - deltas
 */
void backwardPassV11onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step,  int numNeurons, int numIONeurons,  float *activity, float *delta, float *previousDelta, int *deltaT, float *weight)
{
    backwardPassV11Kernel<<<grid,block,0,stream>>>(step,  numNeurons, numIONeurons,  activity, delta, previousDelta, deltaT, weight);
}

/*!
 * \brief Wrapper for backwardPassV2Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] sequenceOffset - sequence offsets
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] input - input
 * \param[in] activity - activities
 * \param[in] previousDelta - previous deltas
 * \param[in] error - errors
 * \param[in] individualError - error buffer
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[out] delta - deltas
 * \param[out] deltaWeight - delta weights
 * \param[out] buffer - buffer used for storing delta weights
 */
void backwardPassV2onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight, float *buffer)
{
    backwardPassV2Kernel<<<grid,block,0,stream>>>(step, sequenceOffset, numNeurons, numIONeurons, input, activity, delta, deltaWeight, previousDelta, error, individualError, deltaT, weight, buffer);
}

/*!
 * \brief Wrapper for backwardPassV21Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] smemSize - CUDA shared memory size
 * \param[in] stream - CUDA stream
 * \param[in] input - input
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[out] output - output
 */
void backwardPassV21onDevice(dim3 grid, dim3 block, int smemSize, cudaStream_t stream, float *input, float *output, int numNeurons, int numIONeurons)
{
    backwardPassV21Kernel<<<grid,block,smemSize,stream>>>(input, output, numNeurons, numIONeurons);
}

/*!
 * \brief Wrapper for backwardPassV3Kernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] stream - CUDA stream
 * \param[in] step - current step
 * \param[in] numNeurons - number of neurons
 * \param[in] numIONeurons - number of input-output neurons
 * \param[in] activity - activities
 * \param[in] deltaT - delta-t values
 * \param[in] weight - weights
 * \param[in] delta - deltas
 * \param[out] previousDelta - previous deltas
 * \param[out] deltaWeight - delta weights
 */
void backwardPassV3onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int numNeurons, int numIONeurons, float *activity, float *delta, float *previousDelta, float *deltaWeight, int *deltaT, float *weight)
{
    backwardPassV3Kernel<<<grid,block,0,stream>>>(step, numNeurons, numIONeurons, activity, delta, previousDelta, deltaWeight, deltaT, weight);
}

/*!
 * \brief Wrapper for reduceKernel.
 * \param[in] size - number of elements to sum
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] smemSize - CUDA shared memory size
 * \param[in] stream - CUDA stream
 * \param[in] input - input
 * \param[in] n - number of elements to sum
 * \param[in] nIsPow2 - determines if the number is of power of two
 * \param[out] output - output
 */
void reduceOnDevice(int size, dim3 grid, dim3 block, int smemSize, cudaStream_t stream, float *input, float *output, unsigned int n, bool nIsPow2)
{
    switch(size)
    {
        case 512:
            reduceKernel<512><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case 256:
            reduceKernel<256><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case 128:
            reduceKernel<128><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case 64:
            reduceKernel<64><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case 32:
            reduceKernel<32><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case 16:
            reduceKernel<16><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case  8:
            reduceKernel<8><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case  4:
            reduceKernel<4><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case  2:
            reduceKernel<2><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;

        case  1:
            reduceKernel<1><<<grid,block,smemSize,stream>>>(input, output, n, nIsPow2);
        break;
    }
}

/*!
 * \brief Wrapper for sumDeltaWeightsP2PKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] numWeights - number of weights
 * \param[in] peerDeltaWeight - delta weights from peer device
 * \param[out] masterDeltaWeight - delta weights from master device
 */
void sumDeltaWeightsP2PonDevice(dim3 grid, dim3 block, int numWeights, float *masterDeltaWeight, float *peerDeltaWeight)
{
    sumDeltaWeightsP2PKernel<<<grid,block>>>(numWeights, masterDeltaWeight, peerDeltaWeight);
}

/*!
 * \brief Wrapper for updateWeightsP2PKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] numWeights - number of weights
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] deltaWeight - delta weights
 * \param[out] previousDeltaWeight - previous delta weights
 * \param[out] masterWeight - weigths from master device
 * \param[out] peerWeight - weights from peer device
 */
void updateWeightsP2PonDevice(dim3 grid, dim3 block, int numWeights, float learningRate, float momentum, float *masterWeight, float *peerWeight, float *deltaWeight, float *previousDeltaWeight)
{
    updateWeightsP2PKernel<<<grid,block>>>(numWeights, learningRate, momentum, masterWeight, peerWeight, deltaWeight, previousDeltaWeight);
}

/*!
 * \brief Wrapper for sumErrorP2PKernel.
 * \param[in] grid - CUDA grid size
 * \param[in] block - CUDA block size
 * \param[in] peerError - error from peer device
 * \param[out] masterError - error from master device
 */
void sumErrorP2PonDevice(dim3 grid, dim3 block, float *masterError, float *peerError)
{
    sumErrorP2PKernel<<<grid,block>>>(masterError, peerError);
}

