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

#include <cuda.h>
#include <cuda_runtime.h>

/*!
 * \brief Basic motion finding algorithm which does not take into account the motion of the robot.
 * \note Consecutive images are subtracted and thresholded to create a black and white image showing which areas in an image have changed significantly.
 * \param[in] image1
 * \param[in] image2
 * \param[in] threshold
 * \param[out] result
 */
__global__ void findMotionKernel(int *image1, int *image2, int *result, int threshold)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x; //red
    int idx1 = (gridDim.x+blockIdx.x) * blockDim.x + threadIdx.x; //green
    int idx2 = (2*gridDim.x+blockIdx.x) * blockDim.x + threadIdx.x; //blue

    if(image1[idx] - image2[idx] > threshold || image1[idx] - image2[idx] < -threshold ||
       image1[idx1] - image2[idx1] > threshold || image1[idx1] - image2[idx1] > threshold ||
       image1[idx2] - image2[idx2] > threshold || image1[idx2] - image2[idx2] > threshold)
    {
        result[idx] = 255;
        result[idx1] = 255;
        result[idx2] = 255;
    }
    else
    {
        result[idx] = 0;
        result[idx1] = 0;
        result[idx2] = 0;
    }
}

/*!
* \brief Search around each and every pixel in the motion image summing the result to give an indication of the relative density of motion around that pixel.
* \param[in] a
* \param[out] result
*/
__global__ void findClustersKernel(int *a, int *result)
{
    int x = threadIdx.x;
    int y = blockIdx.x;
    int comp_idx;
    int idx = y * blockDim.x + x;
    int dist = 4;

    result[idx] = 0.0;

    if(a[idx])
    {
        for(int i=x-dist; i<x+dist+1; i++)
        {
            for(int j=y-dist; j<y+dist+1; j++)
            {
                if(i > -1 && i < blockDim.x && j > -1 && j < gridDim.x)
                {
                    comp_idx = j * blockDim.x + i;
                    if(a[comp_idx] > 0)	result[idx] += 1;
                }
            }
        }
    }
}

/*!
 * \brief Wrapper for findMotionKernel.
 * \param[in] image1
 * \param[in] image2
 * \param[in] threshold
 * \param[out] result
 */
void findMotionOnDevice(int *image1, int *image2, int *result, int threshold)
{
    dim3 block(320,1,1);
    dim3 grid(240,3,1);

    findMotionKernel<<<grid,block>>>(image1, image2, result, threshold);
}

/*!
* \brief Wrapper for findClustersKernel.
* \param[in] a
* \param[out] result
*/
void findClustersOnDevice(int *image, int *result)
{
    findClustersKernel<<<240, 320>>>(image, result);
}
