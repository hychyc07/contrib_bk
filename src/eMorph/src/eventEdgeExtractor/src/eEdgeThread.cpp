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
 * @file eEdgeThread.cpp
 * @brief Implementation of the thread (see header vaThread.h)
 */

#include <iCub/eEdgeThread.h>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define DIM 10
#define THRATE 30
#define SHIFTCONST 100

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    //int padding = src->getPadding();
    //int channels = src->getPixelCode();
    //int width = src->width();
    //int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    //for (int r=0; r < height; r++) {
        //for (int c=0; c < width; c++) {
            //*pdest++ = (unsigned char) *psrc++;
        //}
        //pdest += padding;
        //psrc += padding;
    //}

    memcpy(psrc, pdest, (src->getRowSize())*(src->height())*(src->getPixelCode()));
}

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    //int padding = src->getPadding();
    //int channels = src->getPixelCode();
    //int width = src->width();
    //int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    //for (int r=0; r < height; r++) {
        //for (int c=0; c < width; c++) {
            //*pdest++ = (unsigned char) *psrc++;
            //*pdest++ = (unsigned char) *psrc++;
            //*pdest++ = (unsigned char) *psrc++;
        //}
        //pdest += padding;
        //psrc += padding;
    //}
    memcpy(psrc, pdest, (src->getRowSize())*(src->height())*(src->getPixelCode()));
}

eEdgeThread::eEdgeThread() : RateThread(THRATE) {
    resized=false;
    count=0;
    leftInputImage = 0;
    rightInputImage = 0;
    shiftValue=20;
}

eEdgeThread::~eEdgeThread() {
    //if(leftInputImage !=0) {
    //    delete leftInputImage;
    //}
    //if(rightInputImage!=0) {
    //   delete rightInputImage;
    //}
}

bool eEdgeThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outLeftPort.open(getName("/edgesLeft:o").c_str());
    outRightPort.open(getName("/edgesRight:o").c_str());
    inLeftPort.open(getName("/left:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    return true;
}

void eEdgeThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    inLeftPort.interrupt();
    inRightPort.interrupt();
}

void eEdgeThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string eEdgeThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void eEdgeThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    //leftInputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(width, height);
    //rightInputImage = new ImageOf<PixelMono>;
    //rightInputImage->resize(width, height);
}

void eEdgeThread::run() {    
    if ((inLeftPort.getInputCount()) && (outLeftPort.getOutputCount())) {       
        leftInputImage = inLeftPort.read();
        
        unsigned char* pin = leftInputImage->getRawImage();
        int width = leftInputImage->width();
        int height = leftInputImage->height();
        ImageOf<PixelMono>& outLeft = outLeftPort.prepare();
        outLeft.resize(width,height);
        unsigned char* pout = outLeft.getRawImage();
        int padding = leftInputImage->getPadding();
        int rowsize = leftInputImage->getRowSize();
        
        for (int r = 0; r < (height - DIM); r++) {
            for (int c = 0; c < (width - DIM); c++) {
                
                Vector p(DIM);
                bool paint254h = true;
                for (int i=0; i < DIM; i++){
                    p[i] = (double) *(pin + i);
                    if(p[i]!=254){
                        paint254h = false;
                        break;
                    }   
                }
                bool paint0h = true;
                for (int i=0; i < DIM; i++){
                    p[i] = (double) *(pin + i);
                    if(p[i]!=0){
                        paint0h = false;
                        break;
                    }
                }
                bool paint254v = true;
                for (int i=0; i<DIM; i++){
                    p[i] = (double) *(pin + rowsize * i);
                    if(p[i]!=254){
                        paint254v = false;
                        break;
                    }
                }
                bool paint0v = true;
                for (int i=0; i<DIM; i++){
                    p[i] = (double) *(pin + rowsize * i);
                    if(p[i]!=0){
                        paint0v = false;
                        break;
                    }
                }
                

                
                if(paint254h){
                    *pout = 254;
                }
                else if(paint0h){
                    *pout = 0;
                }
                else if (paint254v){
                    *pout = 254;
                }
                else if(paint0v){
                    *pout = 0;
                }
                else{
                    *pout = 127;
                }
                

                //*pout = *pin;                
                pout++;
                pin++;
            }
            for (int c=0; c< DIM; c++) {
                *pout++ = 127 ;
                pin++;
            }
            pin += padding;
            pout += padding;
        }
        for (int r=0; r < DIM; r++) {
            for (int c = 0; c < width; c ++) {
                *pout++ = 127; pin++;
            }
            pin += padding;
            pout += padding;
        }
        
        //cvDilate(outLeft.getIplImage(),outLeft.getIplImage(),NULL,4);
        //cvErode(outLeft.getIplImage(),outLeft.getIplImage(),NULL,4);
        
        outLeftPort.write();
    }
    
    /*
    if ((inRightPort.getInputCount()) && (outRightPort.getOutputCount())) {
        rightInputImage = inRightPort.prepare();
        ImageOf<PixelMono>& outRight = outRightPort.prepare();
        unsigned char* pin = rightInputImage->getRawImage();
        unsigned char* pout = outRight.getRawImage();
        int padding = rightInputImage->getPadding();
        for (int r = 0; r < rightInputImage->height(); r++) {
            for (int c = 0; c < rightInputImage->width() - 3; c++) {
                unsigned char p1, p2, p3;
                p1 = *(pin + 1);
                p2 = *(pin + 2);
                p3 = *(pin + 3);
                
                if((p1 == *pin) && (p2 == *pin) && (p3 == *pin)){
                    *pout++ = *pin++;
                }
            }
            pin += padding;
            pout += padding;
        }
        outRightPort.write();
    }
    */
    
}



void eEdgeThread::threadRelease() {
    outLeftPort.close();
    outRightPort.close();
    inLeftPort.close();
    inRightPort.close();
}

