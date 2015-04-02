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

#ifndef AQUILA_IMAGE_LIB_H
#define AQUILA_IMAGE_LIB_H

#include <QVector>
#include <QThread>
#include <QStringList>
#include <cuda.h>
#include <cuda_runtime.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "aquila-utility.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace aquila
{
class Tracker : public QThread
{
    Q_OBJECT

public:
    Tracker();
    ~Tracker();

    GPU *gpu;
    Math *math;

private:
    int threshold;
    int imageFlip;
    bool simulationMode;
    bool running;

    //images
    ImageOf<PixelRgb> *leftImage;
    ImageOf<PixelRgb> *rightImage;
    ImageOf<PixelRgb> oldLeftImage;
    ImageOf<PixelRgb> oldRightImage;
    ImageOf<PixelRgb> leftMotionImage;
    ImageOf<PixelRgb> rightMotionImage;

    struct Image
    {
        BufferedPort<ImageOf<PixelRgb> > inputPort;
        BufferedPort<ImageOf<PixelRgb> > outputPort;
        QString inputPortName;
        QString outputPortName;
    };

    Image leftCam;
    Image rightCam;
    Image leftMotion;
    Image rightMotion;

    //device memory pointers
    int *aLeftImage_d;
    int *bLeftImage_d;
    int *aRightImage_d;
    int *bRightImage_d;
    int *leftMotionImage_d;
    int *leftMotionResult_d;
    int *rightMotionImage_d;
    int *rightMotionResult_d;

    //host memory pointers
    int *leftImage_h;
    int *rightImage_h;
    int *leftMotionImage_h;
    int *rightMotionImage_h;
    int *leftMotionResult_h;
    int *rightMotionResult_h;

    void run();

    void findMotion(ImageOf<PixelRgb> *image, ImageOf<PixelRgb> *oldImage, ImageOf<PixelRgb> *motionImage, int *imageVector, int*motionImageVector, int *aDeviceImage, int *bDeviceImage, int *motionDeviceImage, int *motionDeviceResult, int *motionResult);
    void myCopyImage(ImageOf<PixelRgb> *source, ImageOf<PixelRgb> *destination);
    void myCopyImageToVector(ImageOf<PixelRgb> *source, int *destination, bool direction);

public:
    bool terminalMode;

    void clean();
    void connectPorts();
    void disconnectPorts();
    bool openPorts(QString portPrefix);

    void stop();

    void setThreshold(int thresholdValue);
    void setSimulationMode(bool active);

    int getThreshold();
    bool getSimulationMode();

signals:
    void messageSet(QString message);
    void started(int id);
    void stopped(int id);
};
}
#endif//AQUILA_IMAGE_LIB_H
