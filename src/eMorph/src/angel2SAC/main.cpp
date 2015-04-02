// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * Public License for more details
 */
  
/**
 * @file main.cpp
 * @brief main code for launching the angel2SAC
 */
 
#define TmpDiff128AngelX_SIZE 128
#define TmpDiff128AngelY_SIZE 128
#define X_SIZE 128
#define Y_SIZE 128
#define MODEA 1
#define MODEB 2
#define MODEC 5
#define MODED 7

//#define TOMAPPER

#ifdef TOMAPPER 
    #define ADDRESS 0x60000
    #define X_MASK 0x0000FE00
    #define X_SHIFT 9
    #define Y_MASK 0x0000007F
    #define Y_SHIFT 0
    #define POLARITY_MASK 0x00000100
    #define POLARITY_SHIFT 8
#else
    #define ADDRESS 0x40000
    #define X_MASK 0x000000FE
    #define X_SHIFT 1
    #define Y_MASK 0x00007F00
    #define Y_SHIFT 8
    #define POLARITY_MASK 0x00000001
    #define POLARITY_SHIFT 0
#endif

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <stdio.h>
#include <string>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int SACcreateAddress(int x, int y) {
    return ((0x001F & y)<<5) | x;
}


int TmpDiff128AngelcreateAddress(int x, int y, bool polarity){
    if( (x >= 0) && (x < X_SIZE) && (y >= 0) && (y < Y_SIZE) ) {
        int address = 0;
        
        int p;
            
        if(polarity){
            p = 1;
        }
        else {
            p = 0;
        }
        
        // flip x
        //x = X_SIZE - x;
 
        
        address = address | (y << Y_SHIFT);
        address = address | (p << POLARITY_SHIFT);
        address = address | (x << X_SHIFT);
        
        return address;
    }
    else {
        printf("Not appropriate pixel dimensions");
        return -1;
    }
}

void mapping41(FILE *pfile) {
    //int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {            
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            // monitoring
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
            // to SAC chip
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
        }
    }
}

void mapping81(FILE *pfile) {
    //int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {            
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            // monitoring
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
            // to SAC chip
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
            fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
        }
    }
}

void center44(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((yin%4 == MODEA)||(yin%4 == MODEB)) {
                if ((xin%4 == MODEA)||(xin%4 == MODEB)){
                    yout = yin>>2;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    xout--;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));                                        
                    
                }
            }                        
        }
    }
}

void horizontal54(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((yin%4 == MODEA)||(yin%4 == MODEB)) {
                if ((xin%4 == 0)&&(xin >= 4)){
                    yout = yin>>2;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    xout--;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));                                        
                    
                }
                else{
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                }
            }                        
        }
    }
}


void horizontal55(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((yin%4==MODEA)||(yin%4==MODEB)) {                
                //-----------------------------------------------------------------------------------------------------------------------
                if ((xin%4 == 0)&&(xin>=4)){
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));

                    xout = (xin>>2) - 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                }
                else if (xin%4 == 3){

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));

                    xout = (xin>>2) + 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                } 
                else{
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                }
                //-----------------------------------------------------------------------------------------------------------------------
            }
        }
    }
}

void horizontal66(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((yin %4 == MODEA)||(yin % 4 == MODEB)) {                
                //-----------------------------------------------------------------------------------------------------------------------
                if ((xin % 4 == 0)&&(xin >= 4)){
                    
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = (xin>>2) - 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                }
                else if ((xin%4 == 1)&&(xin >= 4)){
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = (xin>>2) - 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                }
                else if ((xin%4 == 3)&&(xin < X_SIZE - 4)){
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = (xin>>2) + 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                } 
                else if ((xin%4 == 2)&&(xin < X_SIZE - 4))  {
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = (xin>>2) + 1;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                } 
                else{
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                //-----------------------------------------------------------------------------------------------------------------------
            }
        }
    }
}

void horizfull66(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            //if((yin %4 == MODEA)||(yin % 4 == MODEB)) {                
                //-----------------------------------------------------------------------------------------------------------------------
                if ((xin % 4 == 0)){
                    
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    if(xin >= 4){

                        xout = (xin>>2) - 1;
                        yout = yin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }                                    
                }
                else if ((xin%4 == 1)&&(xin >= 4)){
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    if(xin >= 4){

                        xout = (xin>>2) - 1;
                        yout = yin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }    
                    
                }
                else if ((xin%4 == 3)&&(xin < X_SIZE - 4)){
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    if(xin >= 4){

                        xout = (xin>>2) - 1;
                        yout = yin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }    
                    
                } 
                else if ((xin%4 == 2)&&(xin < X_SIZE - 4))  {
                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    if(xin >= 4){

                        xout = (xin>>2) - 1;
                        yout = yin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }                       
                } 
                /*
                else{
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                */
                //-----------------------------------------------------------------------------------------------------------------------
                // }
        }
    }
}

void horizontal1010(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((yin%8==MODEA)||(yin%8==MODEB)||(xin%8==MODEC)||(xin%8==MODED)) {                
                //-----------------------------------------------------------------------------------------------------------------------
                if ((xin%8 == 0)&&(xin>=8)){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = ((xin>>3)-1) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                else if ((xin%8 == 1)&&(xin>=8)){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = ((xin>>3)-1) * 2 ;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                else if (xin%8 == 7){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = ((xin>>3)+1) * 2;
                                        
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                                    
                } 
                else if (xin%8 == 6){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout = ((xin>>3)+1) * 2;
                                        
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                } 
                else{
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    xout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                }
                //-----------------------------------------------------------------------------------------------------------------------
            }
        }
    }
}

void vertical54(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((xin%4 == MODEA)||(xin%4 == MODEB)){
                if ((yin %4 == 0)&&(yin >= 4)){
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));

                    yout = (yin>>2) - 1;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                }
                else{
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                }
            }
        }
    }
}

void vertical55(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if((xin%4 == MODEA)||(xin%4 == MODEB)){
                if ((yin%4 == 0)&&(yin>=4)){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));

                    yout = (yin>>2) - 1;
                    xout = xin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                    
                }
                else if (yin%4 == 3){

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));

                    yout = (yin>>2) + 1;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                

                    
                } 
                else{
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xin>>2, yin>>2));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xin>>2, yin>>2));
                }
            }
        }
    }
}

void vertical66(FILE *pfile) {
    int xout,yout;
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        
            if((xin%4 == MODEA)||(xin%4 == MODEB)){
                if ((yin%4 == 0)&&(yin>=4)){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout = (yin>>2) - 1;
                    xout = xin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    
                    
                }
                if ((yin%4 == 1)&&(yin>=4)){
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout = (yin>>2) - 1;
                    xout = xin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));    
                    
                }
                else if (yin%4 == 3){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout = (yin>>2) + 1;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                

                    
                }
                else if (yin%4 == 2){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout = (yin>>2) + 1;
                    xout = xin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                

                    
                } 
                else{
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
            }
        }
    }
}

void vertfull66(FILE *pfile) {
    int xout,yout;
    
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            //if((xin%4 == MODEA)||(xin%4 == MODEB)){
                if ((yin%4 == 0)){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    if (yin >= 4) {

                        yout = (yin>>2) - 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                
                    
                    
                }
                if (yin%4 == 1){
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    if (yin >= 4) {

                        yout = (yin>>2) - 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                    
                }
                else if (yin%4 == 3){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    if(yin < Y_SIZE) {
                        
                        yout = (yin>>2) + 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                        
                    }
                    
                }
                else if (yin%4 == 2){

                    xout = xin>>2;
                    yout = yin>>2;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    
                    if(yin < Y_SIZE) {
                        
                        yout = (yin>>2) + 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                        
                    }

                }
                /*
                else{
                    
                    xout = xin>>2;
                    yout = yin>>2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
                */
                //}
        }
    }
}

void vertical1010(FILE *pfile) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if((xin%8==MODEA)||(xin%8==MODEB)||(xin%8==MODEC)||(xin%8==MODED)) {                
                //-----------------------------------------------------------------------------------------------------------------------
                if ((yin%8 == 0)&&(yin>=8)){
                    
                    yout = (yin>>3) * 2;
                    xout = (xin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout = ((xin>>3)-1) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                else if ((yin%8 == 1)&&(yin>=8)){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout = ((yin>>3)-1) * 2 ;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                }
                else if (yin%8 == 7){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout = ((yin>>3)+1) * 2;
                                        
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                                    
                } 
                else if (yin%8 == 6){
                    
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout = ((yin>>3)+1) * 2;
                                        
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                } 
                else{
                    xout = (xin>>3) * 2;
                    yout = (yin>>3) * 2;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));

                    yout++;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout, yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout, yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout, yout));
                    
                }
                //-----------------------------------------------------------------------------------------------------------------------
            }
        }
    }
}

void positive45(FILE *pfile) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if (xin%4 == 3 - yin%4){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
            }
        }
    }
}


void posfull45(FILE *pfile) {
    int xout,yout;    
    int countMap = 0, countUnmap = 0;
    int countMap1 =0, countMap2 = 0, countMap3 = 0;
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
            if (xin%4 == 3 - (yin%4)) {
                countMap1++;
                printf("section1 %d %d \n", xin, yin);
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                
                if ((yin >= 4) && (xin < X_SIZE - 4)) {                
                    yout = (yin>>2) - 1;
                    xout = (xin>>2) + 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }

                if ((xin >= 4) && (yin < Y_SIZE - 4) ) {
                    yout = (yin>>2) + 1;
                    xout = (xin>>2) - 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }
            }
            
            else if (xin%4 == 3 - (yin%4) - 1) {
                countMap2++;
                printf("section2 %d %d \n", xin, yin);
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                if ((yin >= 4) && (xin < X_SIZE - 4)) {                
                    yout = (yin>>2) - 1;
                    xout = (xin>>2) + 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }

                if ((xin >= 4) && (yin < Y_SIZE - 4)) {
                    yout = (yin>>2) + 1;
                    xout = (xin>>2) - 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }
            }
            else if ((xin%4) == 3 - (yin%4) + 1) {
                countMap3++;
                printf("section3: %d %d \n", xin, yin);
                
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                if ((yin >= 4) && (xin < X_SIZE - 4)) {                
                    yout = (yin>>2) - 1;
                    xout = (xin>>2) + 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }

                if ((xin >= 4) && (yin < Y_SIZE - 4)) {
                    yout = (yin>>2) + 1;
                    xout = (xin>>2) - 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));   
                }
            }
            
            /*
            else {
                countUnmap++;
                
                xout = xin>>2;
                yout = yin>>2;
                
                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
            } 
            */
        }
    }
    printf("countMap1 %d countMap2 %d countMap3 %d tot %d \n", countMap1, countMap2, countMap3, countMap1 + countMap2 +countMap3 );
    printf("countUnmap %d \n", countUnmap);
}

void negative45(FILE *pfile) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if ((xin%4 == yin%4) && (xin>= 1) && (yin >= 1)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));                                
                
            }
        }
    }
}

void negfull45(FILE *pfile) {
    int xout,yout;
    for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
        for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {        
            if (xin%4 == yin%4){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                if ((yin < Y_SIZE - 4) && (xin < X_SIZE - 4) /*&& (xin >= 2)*/ ) {

                    yout = (yin >> 2) + 1;
                    xout = (xin >> 2) + 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
                
                if ((yin >=  4) && (xin >= 4) /*&& (xin < 2)*/) {

                    yout = (yin >> 2) - 1;
                    xout = (xin >> 2) - 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
            }
            else if (xin%4 == yin%4 + 1){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));

                if ((yin < Y_SIZE- 4) && (xin < X_SIZE - 4) /*&& (xin >= 2)*/) {

                    yout = (yin >> 2) + 1;
                    xout = (xin >> 2) + 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
                
                if ((yin >=  4) && (xin >= 4) /*&& (xin < 2)*/) {

                    yout = (yin >> 2) - 1;
                    xout = (xin >> 2) - 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
            }
            else if (xin%4 == yin%4 - 1){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                if ((yin < Y_SIZE- 4) && (xin < X_SIZE - 4) /*&& (xin >= 2)*/) {

                    yout = (yin >> 2) + 1;
                    xout = (xin >> 2) + 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
                
                if ((yin >=  4) && (xin >= 4) /*&& (xin < 2)*/) {

                    yout = (yin >> 2) - 1;
                    xout = (xin >> 2) - 1;

                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
            }
        }
    }
}

void cross(FILE *pfile) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if ((xin%4 == 1)||(yin%4 == 1)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                if(xin % 4 == 3) { //east                    
                    if(xin < X_SIZE - 4) {
                        yout = yin>>2;
                        xout = (xin>>2) + 1;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }

                if(xin % 4 == 0) { //west
                    if(xin >= 4) {
                        yout = yin>>2;
                        xout = (xin>>2) - 1;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }
                
                if(yin % 4 == 3) { //south                    
                    if(yin < Y_SIZE - 4) {
                        yout = (yin>>2) + 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }

                if(yin % 4 == 0) { //north
                    if(yin >= 4) {
                        yout = (yin>>2) - 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }
                
                
            }
            else if((xin%4 == 2)||(yin%4 == 2)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                if(xin % 4 == 3) { //east                    
                    if(xin < X_SIZE - 4) {
                        yout = yin>>2;
                        xout = (xin>>2) + 1;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }

                if(xin % 4 == 0) { //west
                    if(xin >= 4) {
                        yout = yin>>2;
                        xout = (xin>>2) - 1;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }
                
                if(yin % 4 == 3) { //south                    
                    if(yin < Y_SIZE - 4) {
                        yout = (yin>>2) + 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }

                if(yin % 4 == 0) { //north
                    if(yin >= 4) {
                        yout = (yin>>2) - 1;
                        xout = xin>>2;
                        
                        // monitoring
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                        // to SAC chip
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                        fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                    }
                }
            }
        }
    }
}


void cornerNE(FILE *pfile) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if ((yin%4 == 0)||(yin%4 == 1)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                                    
                if(yin >= 4) {
                    yout = yin>>2;
                    xout = (yin>>2) - 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }
                               
            }
            else if((xin%4 == 2)||(xin%4 == 3)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
                
                if(xin < X_SIZE - 4) {
                    yout = yin>>2;
                    xout = (xin>>2) + 1;
                    
                    // monitoring
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                    // to SAC chip
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                    fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                }                                
            }
        }
    }
}

void crossNM(FILE *pfile, short n, short m) {
    int xout,yout;
    for(int xin = 0; xin < TmpDiff128AngelX_SIZE; xin++) {
        for(int yin = 0; yin < TmpDiff128AngelY_SIZE; yin++) {
            if ((xin%4 == n) && (yin%4 == m)){
                yout = yin>>2;
                xout = xin>>2;

                // monitoring
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x110000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x110000 | SACcreateAddress(xout,yout));
                // to SAC chip
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, true) , 0x80000 | SACcreateAddress(xout,yout));
                fprintf(pfile,"%x %x\n", ADDRESS | TmpDiff128AngelcreateAddress(xin, yin, false), 0x80000 | SACcreateAddress(xout,yout));
                
            }
        }
    }
}

int main(int argc, char * argv[])
{
    Network yarp;

    Time::turboBoost(); 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("angel2SAC.ini");         //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);


    printf("--mode mapping81 \n");
    printf("options : mapping41, mapping81, center44 \n");  
    printf("options : horizontal54, horizontal55, horizontal66, horizfull66, horizontal1010 \n");  
    printf("options : vertical54, vertical55, vertical66, vertfull66, vertical1010 \n");
    printf("options : positive45, negative45, posfull45, negfull45 \n");
    printf("options : cross, cross00, cross11, cross22, cross33 \n");
    printf("options : cornerNE,");
    
    // get the mode of the mapping 
    std::string mappingName;
    mappingName            = rf.check("mode", 
                           Value("mapping81"), 
                           "mapping mode (string)").asString();
    printf("Choosen mapping: %s \n", mappingName.c_str());
    
    
 
    //opening a file
    FILE *pfile  = NULL;
    //std::string filenamefull("map.txt");
    //char* filename = "map.txt";
    //char* filename = filenamefull.c_str();
    
    
    printf("Opening the file ..... \n");
    mappingName.append(".txt");
    pfile = fopen((char*)mappingName.c_str(), "w");
    if(pfile == NULL) {
        printf("error: file coudn`t open correctly");
    }

    //saving the output
    printf("Saving the mapping .....");
    
    
    if(!strcmp("mapping41.txt",mappingName.c_str())){
        printf("using mapping81 \n");        
        mapping41(pfile);
    }
    else if(!strcmp("mapping81.txt",mappingName.c_str())){
        printf("using mapping81 \n");        
        mapping81(pfile);
    }
    else if(!strcmp("center44.txt",mappingName.c_str())){
        printf("using center44 \n");        
        center44(pfile);
    }
    else if(!strcmp("horizontal54.txt",mappingName.c_str())){
        printf("using horizontal 54 \n");        
        horizontal54(pfile);
    }
    else if(!strcmp("horizontal55.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        horizontal55(pfile);
    }
    else if(!strcmp("horizontal66.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        horizontal66(pfile);
    }
    else if(!strcmp("horizfull66.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        horizfull66(pfile);
    }
    else if(!strcmp("horizontal1010.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        horizontal1010(pfile);
    }
    else if(!strcmp("vertical54.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        vertical54(pfile);
    }
    else if(!strcmp("vertical55.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        vertical55(pfile);
    }
    else if(!strcmp("vertical66.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        vertical66(pfile);
    }
    else if(!strcmp("vertfull66.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        vertfull66(pfile);
    }
    else if(!strcmp("vertical1010.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        vertical1010(pfile);
    }
    else if(!strcmp("negative45.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        negative45(pfile);
    }
    else if(!strcmp("positive45.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        positive45(pfile);
    }
    else if(!strcmp("negfull45.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        negfull45(pfile);
    }
    else if(!strcmp("posfull45.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        posfull45(pfile);
    }
    else if(!strcmp("cross00.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        crossNM(pfile,0,0);
    }
    else if(!strcmp("cross11.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        crossNM(pfile,1,1);
    }
    else if(!strcmp("cross22.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        crossNM(pfile,2,2);
    }
    else if(!strcmp("cross33.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        crossNM(pfile,3,3);
    }
    else if(!strcmp("cross.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        cross(pfile);
    }
    else if(!strcmp("cornerNE.txt",mappingName.c_str())){
        printf("on file %s \n", mappingName.c_str());        
        cornerNE(pfile);
    }
    else{
        printf("not matching in the name found \n");
    }
    

    //closing the file
    printf("Closing the file ..... \n");
    fclose(pfile);
    
    return 0;
}



