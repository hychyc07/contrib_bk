// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahmad Bhat
  * email: ajaz.bhat@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file downSamplerRatethread.cpp
 * @brief Implementation of the eventDriven thread (see downSamplerRatethread.h).
 */

#include <downSamplerRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

#define THRATE 33 //ms

downSamplerRatethread::downSamplerRatethread():RateThread(THRATE) {
    robot = "icub";        
}

downSamplerRatethread::downSamplerRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

downSamplerRatethread::~downSamplerRatethread() {
    // do nothing
}

bool downSamplerRatethread::threadInit() {

    
    if (!inputPort.open(getName("/image:i").c_str())) {
        cout << ": unable to open port to receive camera input image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!outputPort.open(getName("/image:o").c_str())) {
        cout << ": unable to open port to send output image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    outputWidth     =   320;
    outputHeight    =   240;
    return true;
}

void downSamplerRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string downSamplerRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void downSamplerRatethread::setInputPortName(string InpPort) {
        
}


void downSamplerRatethread::run() {


    if (outputPort.getOutputCount() && inputPort.getInputCount()) {
    
        inputImage  =   inputPort.read(false);            
        
        if (inputImage!=NULL)   {
            ImageOf<PixelRgb>& outputImage =  outputPort.prepare();
            outputImage.resize(outputWidth, outputHeight);
            inputWidth      =   inputImage->width();
            inputHeight     =   inputImage->height();
            outputWidth     =   inputWidth>>1; /// one can also use fixed values for 320 x 240
            outputHeight    =   inputHeight>>1;
            outputImage.resize(outputWidth, outputHeight);   
            outputImage.zero();
           
           /* inputIplImage   =   *((IplImage*) inputImage->getIplImage());  
            temp = & inputIplImage;
            thumbnail = Mat(outputHeight,outputWidth,CV_8UC3, CV_RGB(0,0,0));
            cv::resize(temp, thumbnail,Size(),0.5,0.5,CV_INTER_LINEAR);
            outputIplImage  =   thumbnail;          
            outputImage.wrapIplImage(&outputIplImage);
            */
            
            
            
            
            inputPadding    =   inputImage->getPadding();
            outputPadding   =   outputImage.getPadding();
            unsigned char* inputPxl     =   inputImage->getRawImage();
            unsigned char* outputPxl    =   outputImage.getRawImage();
            for (int y = 0; y < outputHeight; y++)  {
                for (int x = 0; x < outputWidth; x++)   {
                    for (int i = 0; i < 3; i++) {
                        *outputPxl   =   (*inputPxl + *(inputPxl + 3) + *(inputPxl + (3 * inputWidth) + inputPadding) + *(inputPxl + (3 * inputWidth) + inputPadding + 3))/4;
                        outputPxl++;
                        inputPxl++;
                    }                 
                    inputPxl += 3;
                }
                inputPxl += inputPadding;
                outputPxl+= outputPadding;
                inputPxl += (3 * inputWidth) + inputPadding;              
            }
            outputPort.write();          
        }        
        
    }                
}

void downSamplerRatethread::threadRelease() {

   
        outputPort.interrupt();
        outputPort.close();
        inputPort.interrupt();
        inputPort.close();
    
    
}


