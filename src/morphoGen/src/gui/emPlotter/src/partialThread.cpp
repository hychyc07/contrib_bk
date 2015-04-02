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
 * @file partialThread.cpp
 * @brief Implementation of the eventDriven thread (see partialThread.h).
 */

#include <partialThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

partialThread::partialThread():RateThread(THRATE) {
    robot = "icub";        
}

partialThread::partialThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

partialThread::~partialThread() {
    // do nothing
}

bool partialThread::threadInit() {

    idle = false;
    for (int i = 0; i < 20; i++)
        for (int j = 0; j < 50; j++)
            pCue[i][j]=0.0;
            
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void partialThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string partialThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void partialThread::setInputPortName(string InpPort) {
    
}

void partialThread::setSharingBottle1(Bottle *bottleIn) { 
    bottleReceiving = bottleIn;
    idle = false;     
}


void partialThread::setSemaphore(Semaphore *mu) {
    mute = mu;
}

void partialThread::updateCue(Bottle* cueBottle) {

     printf("%s \n", cueBottle->toString().c_str());
    
    
     for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
            int index = i * 50 + j;
            double x  = cueBottle->get(index).asDouble();
            if(x <= 1 || x >= 0) {
                pCue[i][j] = x;
            }
            else {
                printf("bad input bottle    \n");
                return;
                }
              //printf("%f is the value of pCue \n",*pCue[i][j]);
        }
     
     }
        
}

void partialThread::cuePlotting() {

    if (outputPort.getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort.prepare();
        
        // processing of the outputImage
        int width  = 50;
        int height = 20;
        int scale  = 10;
            
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {  
                    *oproc++ = pCue[r][c] * 255;
               }
            }
            oproc+=padding;
            for (int l = 0; l < scale-1; l++){
                for (int y = 0; y < outputImage.width(); y++){
                    *oproc++ = *(temp+y);
                }
                oproc+=padding;
            }
            
                        
        }
   //outputPort.prepare() = *outputImage;
            
                
        outputPort.write();  
    } 
 
}

void partialThread::run() {    
             
        mute->wait();
        if(bottleReceiving->size() > 0){
           printf("received not null function \n");
               
           this->updateCue(bottleReceiving);
        }
        bottleReceiving->clear();
        if(bottleReceiving->size() != 0){
           printf("Error\n");
        }
        mute->post();
            
        this->cuePlotting();
             
        
                  
}

void partialThread::threadRelease() {
    outputPort.interrupt();
    outputPort.close();
}

