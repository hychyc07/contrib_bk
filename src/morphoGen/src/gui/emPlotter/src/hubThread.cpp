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
 * @file hubThread.cpp
 * @brief Implementation of the eventDriven thread (see hubThread.h).
 */

#include <hubThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33 //ms

hubThread::hubThread():RateThread(THRATE) {
    robot = "icub";        
}

hubThread::hubThread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

hubThread::~hubThread() {
    // do nothing
}

bool hubThread::threadInit() {

    idle = false;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 7; j++)
            hub[i][j]=0.0;
            
    if (!outputPort[0].open(getName("/hub0/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[1].open(getName("/hub1/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort[2].open(getName("/hub2/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[3].open(getName("/hub3/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[4].open(getName("/hub4/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[5].open(getName("/hubTop/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort[6].open(getName("/hubBottom/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
       

    return true;
    

}

void hubThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string hubThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void hubThread::setInputPortName(string InpPort) {
    
}

void hubThread::setSharingBottle(Bottle *top, Bottle *bottom, Bottle *a[]) {

    for (int i = 0; i < 5; i++)
        hubBottom[i]    =   a[i];
        
    hubTop1         = top;
    hubBottomAll1   = bottom;
}


void hubThread::setSemaphore(Semaphore *top, Semaphore *bottom, Semaphore *a[]) {

    for (int i = 0; i < 5; i++)
        mutexBottom[i]    =   a[i];
        
    mutexTop1       =   top;
    mutexBottomAll1 =   bottom;
}

void hubThread::updateHub(Bottle* hubBottle) {

     printf("%s \n", hubBottle->toString().c_str());
    
    
     for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 7; j++) {
            int index = i * 7 + j;
            double x  = hubBottle->get(index).asDouble();
            if(x <= 1 && x >= 0) {
                hub[i][j] = x;
            }
            else {
                printf("bad input bottle\n");
                return;
                }
              //printf("%f is the value of hub \n",*hub[i][j]);
        }
     
     }
        
}

void hubThread::hubPlotting(int i) {

    if (outputPort[i].getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outputImage = outputPort[i].prepare();
        
        // processing of the outputImage
        int height = 6;
        int width  = 7;
        int scale  = 10;
            
        outputImage.resize(width*scale, height*scale);
        int padding = outputImage.getPadding();

        unsigned char* oproc = outputImage.getRawImage();
        unsigned char* temp = oproc;
    
        for (int r = 0; r < height; r++) {
            temp = oproc;
            for (int c = 0; c < width; c++) {
               for (int k = 0; k < scale; k++) {  
                    *oproc++ = hub[r][c] * 255;
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

void hubThread::run() {    
       
  /*      if (!idle) {
             printf(" hubTop pointer is %08x \n", hubTop1);
             idle = true;
        }
  */      
        
        for (int i = 0; i < 5; i++) {
        
        mutexBottom[i]->wait();          
        if(hubBottom[i]->size() > 0){
            printf("received not null function as hub \n");  
            this->updateHub(hubBottom[i]);
            this->hubPlotting(i);
        }
        hubBottom[i]->clear();
        if(hubBottom[i]->size() != 0){
            printf("Error\n");
        }
        mutexBottom[i]->post();

            
        }
        
        
        mutexTop1->wait();          
        if(hubTop1->size() > 0){
            printf("received not null function as hubTop \n");  
            this->updateHub(hubTop1);
        }
        hubTop1->clear();
        if(hubTop1->size() != 0){
            printf("Error\n");
        }
        mutexTop1->post();
        this->hubPlotting(5);
            
                  
        mutexBottomAll1->wait();          
        if(hubBottomAll1->size() > 0){
            printf("received not null function as hubBottom \n");  
            this->updateHub(hubBottomAll1);
        }
        hubBottomAll1->clear();
        if(hubBottomAll1->size() != 0){
           printf("Error\n");
        }
        mutexBottomAll1->post();
        this->hubPlotting(6);  
}

void hubThread::threadRelease() {
    
    for (int i = 0; i < 7; i++) {
        outputPort[i].interrupt();
        outputPort[i].close();
    }
}

