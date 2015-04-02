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
 * @file rememberedxThread.cpp
 * @brief Implementation of the eventDriven thread (see rememberedxThread.h).
 */

#include <rememberedxThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

rememberedxThread::rememberedxThread():RateThread(THRATE) {
    robot = "icub";        
}

rememberedxThread::rememberedxThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

rememberedxThread::~rememberedxThread() {
    // do nothing
}

bool rememberedxThread::threadInit() {

    idle = false;
    for (int i = 0; i < 20; i++)
        for (int j = 0; j < 50; j++)
            pCue[i][j]=0.0;
            
    if (!outputPort[0].open(getName("/rem0/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[1].open(getName("/rem1/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    
    if (!outputPort[2].open(getName("/rem2/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    
    if (!outputPort[3].open(getName("/rem3/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    
    if (!outputPort[4].open(getName("/rem4/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }     

    return true;
    

}

void rememberedxThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string rememberedxThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void rememberedxThread::setInputPortName(string InpPort) {
    
}

void rememberedxThread::setSharingBottle(Bottle *bottleIn[]) {

    for (int i =0; i < 5; i++) {  
        bottleReceiving[i] = bottleIn[i];
    
    }
    idle = false;       
}


void rememberedxThread::setSemaphore(Semaphore *mu[]) {

    for (int i = 0; i < 5; i++) {
        mute[i] = mu[i];
    }
}

void rememberedxThread::updateCue(Bottle* remBottle) {

     printf("%s \n", remBottle->toString().c_str());
    
    
     for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
            int index = i * 50 + j;
            double x  = remBottle->get(index).asDouble();
            if(x <= 1 && x >= 0) {
                pCue[i][j] = x;
            }
            else {
                printf("bad input bottle\n");
                return;
                }
              //printf("%f is the value of pCue \n",*pCue[i][j]);
        }
     
     }
        
}

void rememberedxThread::cuePlotting(int i) {

    if (outputPort[i].getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[i].prepare();
        
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
            
                
        outputPort[i].write();  
    } 
 
}

void rememberedxThread::run() {    
  /*     
        if (!idle) {
             printf(" rememberedxThread pointer is %08x \n", bottleReceiving[0]);
             idle = true;
        }
  */      
        for (int i = 0; i < 5; i++) {     
            mute[i]->wait();
            if(bottleReceiving[i]->size() > 0){
                printf("received not null function as remembered experience \n");
               
                this->updateCue(bottleReceiving[i]);
                this->cuePlotting(i);
            }
            bottleReceiving[i]->clear();
            if(bottleReceiving[i]->size() != 0){
                printf("Error\n");
            }
            mute[i]->post();
 
        }
        
             
        
                  
}

void rememberedxThread::threadRelease() {
    for (int i = 0; i < 5; i++) {
        outputPort[i].interrupt();
        outputPort[i].close();
    }
}

