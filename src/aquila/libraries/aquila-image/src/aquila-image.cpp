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

#include <omp.h>
#include "aquila-image.h"
#include "kernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

namespace aquila
{
/*!
 * \brief Constructor.
 */
Tracker::Tracker()
{
    gpu = new GPU();
    math = new Math();
}

/*!
 * \brief Destructor.
 */
Tracker::~Tracker()
{
    clean();
}

/*!
 * \brief Opens image ports.
 */
bool Tracker::openPorts(QString portPrefix)
{
    //assign port names
    leftCam.inputPortName = portPrefix + QString("/cam/left:i");
    leftCam.outputPortName = portPrefix + QString("/cam/left:o");
    rightCam.inputPortName = portPrefix + QString("/cam/right:i");
    rightCam.outputPortName = portPrefix + QString("/cam/right:o");
    leftMotion.inputPortName = portPrefix + QString("/motion/left:i");
    leftMotion.outputPortName = portPrefix + QString("/motion/left:o");
    rightMotion.inputPortName = portPrefix + QString("/motion/right:i");
    rightMotion.outputPortName = portPrefix + QString("/motion/right:o");

    //open ports
    bool success = (leftCam.outputPort.open(leftCam.outputPortName.toStdString().c_str()) &&
                    rightCam.outputPort.open(rightCam.outputPortName.toStdString().c_str()) &&
                    leftCam.inputPort.open(leftCam.inputPortName.toStdString().c_str()) &&
                    rightCam.inputPort.open(rightCam.inputPortName.toStdString().c_str()) &&
                    leftMotion.inputPort.open(leftMotion.inputPortName.toStdString().c_str()) &&
                    rightMotion.inputPort.open(rightMotion.inputPortName.toStdString().c_str()) &&
                    leftMotion.outputPort.open(leftMotion.outputPortName.toStdString().c_str()) &&
                    rightMotion.outputPort.open(rightMotion.outputPortName.toStdString().c_str()));

    return success;
}

/*!
 * \brief Connects ports.
 */
void Tracker::connectPorts()
{
    if(simulationMode)
    {
        Network::connect("/icubSim/cam/left", leftCam.inputPortName.toStdString().c_str());
        Network::connect("/icubSim/cam/right", rightCam.inputPortName.toStdString().c_str());
    }
    else
    {
        Network::connect("/icub/cam/left", leftCam.inputPortName.toStdString().c_str());
        Network::connect("/icub/cam/right", rightCam.inputPortName.toStdString().c_str());
    }
}

/*!
 * \brief Disconnects ports.
 */
void Tracker::disconnectPorts()
{
    if(simulationMode)
    {
        Network::disconnect("/icubSim/cam/left", leftCam.inputPortName.toStdString().c_str());
        Network::disconnect("/icubSim/cam/right", rightCam.inputPortName.toStdString().c_str());
    }
    else
    {
        Network::disconnect("/icub/cam/left", leftCam.inputPortName.toStdString().c_str());
        Network::disconnect("/icub/cam/right", rightCam.inputPortName.toStdString().c_str());
    }
}

/*!
 * \brief Thread loop.
 */
void Tracker::run()
{
    //initialise images
    leftImage = NULL;
    rightImage = NULL;
    oldLeftImage.resize(320,240);
    oldRightImage.resize(320,240);
    leftMotionImage.resize(320,240);
    rightMotionImage.resize(320,240);

    //allocate CUDA memory
    if(gpu->active)
    {
        size_t imageSize = 320 * 240 * 3 * sizeof(int);
        size_t resultSize = 320 * 240 * sizeof(int);
        cudaMalloc((void **) &aLeftImage_d, imageSize);
        cudaMalloc((void **) &bLeftImage_d, imageSize);
        cudaMalloc((void **) &leftMotionImage_d, imageSize);
        cudaMalloc((void **) &aRightImage_d, imageSize);
        cudaMalloc((void **) &bRightImage_d, imageSize);
        cudaMalloc((void **) &rightMotionImage_d, imageSize);
        cudaMalloc((void **) &leftMotionResult_d, resultSize);
        cudaMalloc((void **) &rightMotionResult_d, resultSize);
        leftImage_h = (int *) malloc(imageSize);
        rightImage_h = (int *) malloc(imageSize);
        leftMotionImage_h = (int *) malloc(imageSize);
        rightMotionImage_h = (int *) malloc(imageSize);
        leftMotionResult_h = (int *) malloc(resultSize);
        rightMotionResult_h = (int *) malloc(resultSize);
        imageFlip = 0;
    }

    running = true;
    emit started(1);

    while(running)
    {
        //read the left and right images from the cam ports
        leftImage = leftCam.inputPort.read(false);
        rightImage = rightCam.inputPort.read(false);

        if(gpu->active)
        {
            //copy yarp images to vectors that are copied to GPU
            if(leftImage!=NULL)
            {
                myCopyImageToVector(leftImage, leftImage_h, false);
            }

            if(rightImage!=NULL)
            {
                myCopyImageToVector(rightImage, rightImage_h, false);
            }

            //flip flop for where to store the CUDA image
            if(imageFlip == 1)
            {
                imageFlip = 0;
            }
            else
            {
                imageFlip = 1;
            }            
        }

        //send left image to port
        if(leftImage!=NULL)
        {
            findMotion(leftImage, &oldLeftImage, &leftMotionImage, leftImage_h, leftMotionImage_h, aLeftImage_d, bLeftImage_d, leftMotionImage_d, leftMotionResult_d, leftMotionResult_h);
            leftCam.outputPort.prepare().copy(*leftImage);
            leftCam.outputPort.write();
            leftMotion.outputPort.prepare().copy(leftMotionImage);
            leftMotion.outputPort.write();
        }

        //send right image to port
        if(rightImage!=NULL)
        {
            findMotion(rightImage, &oldRightImage, &rightMotionImage, rightImage_h, rightMotionImage_h, aRightImage_d, bRightImage_d, rightMotionImage_d, rightMotionResult_d, rightMotionResult_h);
            rightCam.outputPort.prepare().copy(*rightImage);
            rightCam.outputPort.write();
            rightMotion.outputPort.prepare().copy(rightMotionImage);
            rightMotion.outputPort.write();
        }
    }

    emit stopped(2);
}

/*!
 * \brief Stops the thread.
 */
void Tracker::stop()
{
    running  = false;
}

/*!
 * \brief Clean up.
 */
void Tracker::clean()
{
    leftCam.inputPort.interrupt();
    leftCam.outputPort.interrupt();
    rightCam.inputPort.interrupt();
    rightCam.outputPort.interrupt();
    leftMotion.inputPort.interrupt();
    leftMotion.outputPort.interrupt();
    rightMotion.inputPort.interrupt();
    rightMotion.outputPort.interrupt();

    disconnectPorts();

    leftCam.inputPort.close();
    leftCam.outputPort.close();
    rightCam.inputPort.close();
    rightCam.outputPort.close();
    leftMotion.inputPort.close();
    leftMotion.outputPort.close();
    rightMotion.inputPort.close();
    rightMotion.outputPort.close();
}

/*!
 * \brief Basic motion finding algorithm which does not take into account the motion of the robot.
 * \note Consecutive images are subtracted and thresholded to create a black and white image showing which areas in an image have changed significantly.
 * \param[in] - image
 * \param[in] - oldImage
 * \param[in] - motionImage
 * \param[in] - imageVector
 * \param[in] - motionImageVector
 * \param[in] - aDeviceImage
 * \param[in] - bDeviceImage
 * \param[in] - motionDeviceImage
 * \param[out] - motionDeviceResult
 * \param[out] - motionResult
 */
void Tracker::findMotion(ImageOf<PixelRgb> *image, ImageOf<PixelRgb> *oldImage, ImageOf<PixelRgb> *motionImage, int *imageVector, int *motionImageVector, int *aDeviceImage, int *bDeviceImage, int *motionDeviceImage, int *motionDeviceResult, int *motionResult)
{
    int x,y;

    if(gpu->active)
    {
        size_t imageSize = 320 * 240 * 3 * sizeof(int);
        int max = -1;
        int target[2];

        //copy image to device - use flip to determine where to copy the image
        if(imageFlip == 1)
        {
            gpuAssert(cudaMemcpy(aDeviceImage, imageVector, imageSize, cudaMemcpyHostToDevice));
        }
        else
        {
            gpuAssert(cudaMemcpy(bDeviceImage, imageVector, imageSize, cudaMemcpyHostToDevice));
        }

        //run kernel
        findMotionOnDevice(aDeviceImage, bDeviceImage, motionDeviceImage, threshold);
        findClustersOnDevice(motionDeviceImage, motionDeviceResult);

        //copy image and cluster result back to host
        gpuAssert(cudaMemcpy(motionImageVector, motionDeviceImage, imageSize, cudaMemcpyDeviceToHost));
        gpuAssert(cudaMemcpy(motionResult, motionDeviceResult, (320 * 240 * sizeof(int)), cudaMemcpyDeviceToHost));
        myCopyImageToVector(motionImage, motionImageVector, true);

        //find the highest density of movmemnt for a target
        for(x=0; x<image->width(); x++)
        {
            for(y=0; y<image->height(); y++)
            {
                if(motionResult[(y * image->width()) + x] > max)
                {
                    max = motionResult[(y * image->width()) + x];
                    target[0] = x;
                    target[1] = y;
                }
            }
        }

        //draw an X at the target location in the original image
        if(max > 5)
        {
            int y1 = target[1]-5;
            int y2 = target[1]+5;
            for(x=target[0]-5; x<target[0]+5; x++)
            {
                if(image->isPixel(x,y1))
                {
                    PixelRgb& pixel = image->pixel(x,y1);
                    pixel.r = 255;
                    pixel.g = 0;
                    pixel.b = 0;
                }
                y1++;
                if(image->isPixel(x,y2))
                {
                    PixelRgb& pixel = image->pixel(x,y2);
                    pixel.r = 255;
                    pixel.g = 0;
                    pixel.b = 0;
                }
                y2--;
            }
        }
    }
    else
    {
        for(x=0; x<image->width(); x++)
        {
            for(y=0; y<image->height(); y++)
            {
                if(image->isPixel(x,y) && oldImage->isPixel(x,y) && motionImage->isPixel(x,y))
                {
                    PixelRgb& pixel = image->pixel(x,y);
                    PixelRgb& oldPixel = oldImage->pixel(x,y);
                    PixelRgb& motionPixel = motionImage->pixel(x,y);

                    //compare each pixel between the new and old image, and threshold the absolute difference to generate an motion image
                    if(pixel.r - oldPixel.r > threshold || pixel.r - oldPixel.r < -threshold ||
                       pixel.g - oldPixel.g > threshold || pixel.g - oldPixel.g < -threshold ||
                       pixel.b - oldPixel.b > threshold || pixel.b - oldPixel.b < -threshold)
                    {
                        motionPixel.r = 255;
                        motionPixel.g = 255;
                        motionPixel.b = 255;
                    }
                    else
                    {
                        motionPixel.r = 0;
                        motionPixel.g = 0;
                        motionPixel.b = 0;
                    }
                }
            }
        }
        myCopyImage(image, oldImage);
    }
}

/*!
 * \brief Copies one yarp image to another.
 * \param[in] source
 * \param[out] destination
 */
void Tracker::myCopyImage(ImageOf<PixelRgb> *source, ImageOf<PixelRgb> *destination)
{
    int x,y;
    for(x=0; x<source->width(); x++)
    {
        for(y=0; y<source->height(); y++)
        {
            if(source->isPixel(x,y) && destination->isPixel(x,y))
            {
                PixelRgb& sourcePixel = source->pixel(x,y);
                PixelRgb& destinationPixel = destination->pixel(x,y);
                destinationPixel.r = sourcePixel.r;
                destinationPixel.g = sourcePixel.g;
                destinationPixel.b = sourcePixel.b;
            }
        }
    }
}

/*!
 * \brief Copies a yarp image to a vector, or a vector to a yarp image.
 * \param[in] yarpImage
 * \param[in] direction
 * \param[out] vectorImage
 */
void Tracker::myCopyImageToVector(ImageOf<PixelRgb> *yarpImage, int *vectorImage, bool direction)
{
    int x,y;
    if(yarpImage!=NULL)
    {
        for(x=0; x<yarpImage->width(); x++)
        {
            for(y=0; y<yarpImage->height(); y++)
            {
                if(yarpImage->isPixel(x,y))
                {
                    PixelRgb& yarpImagePixel = yarpImage->pixel(x,y);
                    if(direction == false)
                    {
                        vectorImage[x + (y*yarpImage->width()) + (0 * (yarpImage->width() * yarpImage->height()))] = yarpImagePixel.r;
                        vectorImage[x + (y*yarpImage->width()) + (1 * (yarpImage->width() * yarpImage->height()))] = yarpImagePixel.g;
                        vectorImage[x + (y*yarpImage->width()) + (2 * (yarpImage->width() * yarpImage->height()))] = yarpImagePixel.b;
                    }
                    else
                    {
                        yarpImagePixel.r = vectorImage[x + (y*yarpImage->width()) + (0 * (yarpImage->width() * yarpImage->height()))];
                        yarpImagePixel.g = vectorImage[x + (y*yarpImage->width()) + (1 * (yarpImage->width() * yarpImage->height()))];
                        yarpImagePixel.b = vectorImage[x + (y*yarpImage->width()) + (2 * (yarpImage->width() * yarpImage->height()))];
                    }
                }
            }
        }
    }
}

/*!
 * \brief Sets sensitivity treshold.
 * \param[in] thresholdValue
 */
void Tracker::setThreshold(int thresholdValue)
{
    threshold = thresholdValue;
}

/*!
 * \brief Sets simulation mode.
 * \param[in] thresholdValue
 */
void Tracker::setSimulationMode(bool active)
{
    simulationMode = active;
}

/*!
 * \brief Gets sensitivity treshold.
 */
int Tracker::getThreshold()
{
    return threshold;
}

/*!
 * \brief Gets simulation mode.
 */
bool Tracker::getSimulationMode()
{
    return simulationMode;
}

}
