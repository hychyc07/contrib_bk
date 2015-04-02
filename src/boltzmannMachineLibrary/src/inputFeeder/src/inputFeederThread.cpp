// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * Public License fo
 r more details
 */

/**
 * @file inputFeederThread.cpp
 * @brief Implementation of the thread (see header inputFeederThread.h)
 */

#include <iCub/inputFeederThread.h>
#include <cstring>
#include <cassert>
#include <cstdio>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100

inputFeederThread::inputFeederThread() : RateThread(THRATE) {
    resized=false;
    img=0;
    count=0;
}

inputFeederThread::~inputFeederThread() {
}

bool inputFeederThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    return true;
}

void inputFeederThread::interrupt() {
    outPort.interrupt();
}

void inputFeederThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string inputFeederThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void inputFeederThread::resize(int widthp, int heightp) {
    width=widthp;
    height=heightp;
}

void inputFeederThread::run() {
    
    if(outPort.getOutputCount()) {
        filename.clear();
        char numStr[4];
        sprintf(&numStr[0],"%d",count);
        filename.append(filenameStart);
        filename.append("/data/im (");
        filename.append(numStr);
        filename.append(").jpg");
        
        img=cvLoadImage(filename.c_str(),0);  //flag 0 force to be gray scale, flag 1 force to be 3 channel
        
        if(!img) {
            printf("Could not load image file: %s\n",filename.c_str());
        }
        else {
            width=img->width; height=img->height;
            ImageOf<PixelMono>& outImage=outPort.prepare();
            outImage.resize(width,height);
            int padding=outImage.getPadding();
            unsigned char* outPointer=outImage.getRawImage();
            uchar* data    = (uchar *)img->imageData;

            for(int r=0;r<height;r++) {
                for(int c=0;c<width;c++) {
                    *outPointer = (unsigned char) *data;
                    outPointer++; data++;
                }
                outPointer+=padding;
            }
            outPort.write();
        }
        count++;
        if(count%850==0) {
            count=0;
        }
    }
    


}

void inputFeederThread::threadRelease() {
    //closing ports
    outPort.close();
}