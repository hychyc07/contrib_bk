// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahamd Bhat
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
 * @file colorQuantizatorThread.cpp
 * @brief Implementation of the eventDriven thread (see colorQuantizatorThread.h).
 */

#include <iCub/colorQuantizatorThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

colorQuantizatorThread::colorQuantizatorThread() {
    robot = "icub";        
}

colorQuantizatorThread::colorQuantizatorThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

colorQuantizatorThread::~colorQuantizatorThread() {
    // do nothing
}

bool colorQuantizatorThread::threadInit() {

    if (!inputPort.open(getName("/img:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
   
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    processedImage = new ImageOf<PixelRgb>;
    return true;
    

}

void colorQuantizatorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string colorQuantizatorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void colorQuantizatorThread::setInputPortName(string InpPort) {
    
}

void colorQuantizatorThread::run() {    
    while (isStopping() != true) {

        if(inputPort.getInputCount()){
            ImageOf<PixelRgb>* inputImage = inputPort.read(false);

            if (outputPort.getOutputCount()) {

                ImageOf<PixelMono>& grayImage = outputPort.prepare(); // image for output
                if(inputImage!=NULL){

                    //processing
                    int width      = inputImage->width();
                    int height     = inputImage->height();
                    int widthCntr  = width/2;
                    int heightCntr = height/2;
                    int scaleVal   = 2;
					//int widthProc  = 40;
					//int heightProc = 
                   
                    processedImage->resize(width/scaleVal, height/scaleVal); 
                    grayImage.resize(40,10);

                    int paddingInput = inputImage->getPadding();
                    int paddingProc  = processedImage->getPadding();
                    int paddingGray  = grayImage.getPadding();
                    int rowSize      = grayImage.getRowSize();

                    unsigned char* pinput = inputImage->getRawImage();
                    unsigned char* pproc  = processedImage->getRawImage();
                    unsigned char* pgray  = grayImage.getRawImage();
                    //printf("input %d processed %d \n", paddingInput, paddingProc);
                    //printf("width %d height %d \n", width, height);
                    
                    //processedImage.zero();

                    int colorFreq[8] = {0};
                    int monoVal[8]   = {0};
                   // char* colorName[8]={"Paynes Gray", "Cerulean Blue","Lime Green","Medium Turquoise","Brick Red","deep Fuchsia","Old Gold","Silver"};
                    
                    //calculate the row value and column value of the pixel from to start copying the input image
                    int rowOne = heightCntr - (processedImage->height()) /2; 
                    int colOne = widthCntr  - (processedImage->width())  /2;  

                    pinput+= 3 * (rowOne * rowSize + colOne); // place the pointer of inputImage at correct location

                    for (int r = 0; r < processedImage->height(); r++) {
                        for (int c = 0; c < processedImage->width(); c++) {
                            int colorBnry[3]; // represents the binary equivalent for calculating the color of an RGB pixel.. see colorName[]
                            for(int i = 0; i < 3; i++){
                                *pproc = *pinput;
                            
                                if (*pproc < 128)
                                    colorBnry[i] = 0;
                                else
                                    colorBnry[i] = 1;
                                //*pproc = (unsigned char) 0;
                                //pproc[c] = 0;
                                //pinput+=3; //pinput++; pinput++; 
                                pinput++;
                                pproc++;
                            }

                            int colorIndex = colorBnry[0] * 4 + colorBnry[1] * 2 + colorBnry[2]; //calculate the correct index for the color of an RGB pixel
                            colorFreq[colorIndex] += 1;
                        }

                        pinput+= 3* (inputImage->width() - processedImage->width());
                        pinput += paddingInput;
                        pproc  += paddingProc;
                    }

                    int total = processedImage->width() * processedImage->height(); //number of pixels in processedImage

                    for(int i = 0; i < 8; i++){
                        monoVal[i] = (colorFreq[i] * 255)  /total; //scale the frequencies of the colors to gray scale
                       // cout<< "Color	" << colorName[i] << "	Frequency	" << colorFreq[i] << endl;
                    }


                    for(int r = 0; r < 10; r++){ // prepare the gray image for output
                        for(int c = 0; c < 8; c++){
                            for(int w = 0; w < 5; w++){
                                *pgray = monoVal[c];
                                pgray++;
                            }
                        }
                        pgray+=paddingGray;
                    }


                    //Time::delay(10);
                    //outputPort.prepare() = *processedImage;
                    outputPort.write();  
                }
            }
        }
    }
}

void colorQuantizatorThread::threadRelease() {
    // nothing
     delete processedImage;
}

void colorQuantizatorThread::onStop() {
    inputPort.interrupt();
    outputPort.interrupt();
    inputPort.close();
    outputPort.close();
}

