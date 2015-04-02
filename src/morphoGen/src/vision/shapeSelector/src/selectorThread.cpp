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
 * @file selectorThread.cpp
 * @brief Implementation of the eventDriven thread (see selectorThread.h).
 */

#include <selectorThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

selectorThread::selectorThread():RateThread(THRATE) {
    robot = "icub";        
}

selectorThread::selectorThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

selectorThread::~selectorThread() {
    // do nothing
}

bool selectorThread::threadInit() {

    idle = false;
    for (int i = 0; i < 20; i++)
        for (int j = 0; j < 50; j++)
            selected[i][j]=0.0;
            
    if (!outputPort[0].open(getName("/left:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputPort[1].open(getName("/right:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
 

    return true;
    

}

void selectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string selectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void selectorThread::setInputPortName(string InpPort) {
    
}

void selectorThread::setSharingBottle(Bottle *a, Bottle *b) {

    leftSelector       =    a;
    rightSelector      =    b;
}


void selectorThread::setSemaphore(Semaphore *c, Semaphore *d) {

    leftMutex       =   c;
    rightMutex      =   d; 
}

void selectorThread::updateSelector(Bottle* hubBottle) {

   
    
    
     for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
            int index = i * 50 + j;
            double x  = hubBottle->get(index).asDouble();
            if(x <= 1 && x >= 0) {
                selected[i][j] = x;
            }
            else {
                printf("bad input bottle\n");
                return;
                }
              //printf("%f is the value of selector \n",*selector[i][j]);
        }
     
     }
        
}

void selectorThread::selectorPlotting(int i) {

    if (outputPort[i].getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[i].prepare();
        
        // processing of the outputImage
        int height = 20;
        int width  = 50;
        int scale  = 10;
            
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {  
                    *oproc++ = selected[r][c] * 255;
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
   //outputPort[i].prepare() = *outputImage;
            
                
        outputPort[i].write();  
    } 
 
}

void selectorThread::run() {    
       
        leftMutex->wait();          
            if(leftSelector->size() > 0){
                printf("received not null function as left selector \n");  
                this->updateSelector(leftSelector);
                this->selectorPlotting(0);
            }
            leftSelector->clear();
            if(leftSelector->size() != 0){
                printf("Error\n");
            }
            leftMutex->post();
            
            
            rightMutex->wait();          
            if(rightSelector->size() > 0){
                printf("received not null function as right selector \n");  
                this->updateSelector(rightSelector);
                this->selectorPlotting(1);
            }
            rightSelector->clear();
            if(rightSelector->size() != 0){
                printf("Error\n");
            }
            rightMutex->post();
        
            
               
}

void selectorThread::threadRelease() {
    for (int i = 0; i < 2; i++) {
        outputPort[i].interrupt();
        outputPort[i].close();
    }
}

