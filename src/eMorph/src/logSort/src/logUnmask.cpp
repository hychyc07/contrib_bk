// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * Public License for more details
 */

/**
 * @file logUnmask.cpp
 * @brief A class for unmasking the logEvent (see the header unmask.h)
 */

#include <math.h>
#include <cassert>

#include <iCub/logUnmask.h>
#include <iCub/config.h>

using namespace std;
using namespace yarp::os;

#define MAXVALUE 4294967295
#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000
//#define SIZE_OF_EVENT 4096 //1024        // CHUNKSIZE / 8     
#define X_DIMENSION 144
#define Y_DIMENSION 72


inline int flipBits( int blob, int bits) {
    int out = 0;
    short bitvalue;
 
    for (int i = bits - 1 ; i >= 0 ; i--) {
        int mask = 1;
        int maskout = 1;
        for (int j = 0; j < i; j++) {
            mask *= 2;
        }
        for (int j = 0; j < bits - 1 - i; j++) {
            maskout *= 2;
        }
        
        if (mask & blob) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        //printf("%d %d;", bitvalue, out);
        
        out += bitvalue * maskout;
    } 
    //printf("\n");
    return out;     
}


logUnmask::logUnmask() : RateThread(UNMASKRATETHREAD){
    count = 0;
    maxx  = 0;
    maxy  = 0;
    countCD  = 0;
    countEM1 = 0;
    countEM2 = 0;
    countEM3 = 0;
    countEM4 = 0;
    countIF  = 0;

    verb       = false;
    validLeft  = false;
    validRight = false;
    temp1      = true;

    numKilledEvents    = 0;
    lasttimestamp      = 0;
    previous_timestamp = 0;

    eldesttimestamp = MAXVALUE;
    countEvent  = 0;
    countEvent2 = 0;
    minValue    = 0;
    maxValue    = 0;

    xmask       = 0x000000ff;      // masking for asv
    ymask       = 0x00007f00;
    xmasklong   = 0x000000ff;
    polmask     = 0x00000001;
    cameramask  = 0x00008000;
    
    yshift   = 8;
    yshift2  = 16;
    xshift   = 0;          // shift for asv
    polshift = 0;
    camerashift = 15;
    retinalSize = 128;


    buffer          = new int[retinalSize * retinalSize];      
    bufferRight     = new int[retinalSize * retinalSize];    
    timeBufferRight = new unsigned long[retinalSize * retinalSize];
    timeBuffer      = new unsigned long[retinalSize * retinalSize];    
    //cartEM          = new aer[24 * 24];
    //pEM             = new aer[24 * 24];

    memset(buffer,         0, retinalSize * retinalSize * sizeof(int));
    memset(timeBuffer,     0, retinalSize * retinalSize * sizeof(unsigned long));
    memset(bufferRight,    0, retinalSize * retinalSize * sizeof(int));
    memset(timeBufferRight,0, retinalSize * retinalSize * sizeof(unsigned long));
    //memset(cartEM,         0, 24 * 24 * sizeof(aer));
    //memset(pEM,            0, 24 * 24 * sizeof(aer));
    

    /*fifoEvent=new int[maxPosEvent];
    memset(fifoEvent,0,maxPosEvent*sizeof(int));
    fifoEvent_temp=new int[maxPosEvent];
    memset(fifoEvent_temp,0,maxPosEvent*sizeof(int));
    fifoEvent_temp2=new int[maxPosEvent];
    memset(fifoEvent_temp2,0,maxPosEvent*sizeof(int));
    */

    // setting all the entries to -1 to avoid unexpected mappings
    logChip_LUT = new feature[X_DIMENSION * Y_DIMENSION]; // all the position in the logchip times 4 features (144 by 72)
    feature* plogChip = logChip_LUT;
    for (int i = 0; i< X_DIMENSION * Y_DIMENSION; i++) {
        plogChip[i][0] = -1;
        plogChip[i][1] = -1;
        plogChip[i][2] = -1;
        plogChip[i][3] = -1;
    }

    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);

    bufferCD = (aer *)  malloc(monBufSize_b);
    if ( bufferCD == NULL ) {
        printf("bufferCD malloc failed \n");
    }
    else {
        printf("bufferCD successfully created \n");
    }
    
    bufferEM1 = (aer *)  malloc(monBufSize_b);
    if ( bufferEM1 == NULL ) {
        printf("bufferEM1 malloc failed \n");
    }
    else {
        printf("bufferEM1 successfully created \n");
    }

    bufferEM2 = (aer *)  malloc(monBufSize_b);
    if ( bufferEM2 == NULL ) {
        printf("bufferEM2 malloc failed \n");
    }
    else {
        printf("bufferEM2 successfully created \n");
    }
    bufferEM3 = (aer *)  malloc(monBufSize_b);
    if ( bufferEM3 == NULL ) {
        printf("bufferEM3 malloc failed \n");
    }
    else {
        printf("bufferEM3 successfully created \n");
    }
    
    bufferEM4 = (aer *)  malloc(monBufSize_b);
    if ( bufferEM4 == NULL ) {
        printf("pmon malloc failed \n");
    }
    else {
        printf("bufferEM4 successfully created \n");
    }

    bufferIF = (aer *)  malloc(monBufSize_b);
    if ( bufferIF == NULL ) {
        printf("bufferIF malloc failed \n");
    }
    else {
        printf("bufferIF successfully created \n");
    }

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the logUnmasked_buffer
    //uEvents = fopen("./uevents.txt","w");


}

logUnmask::~logUnmask() {
    printf("logUnmask : freeing memory allocated by buffers \n");
    delete[] buffer;
    delete[] timeBuffer;
    delete[] bufferRight;
    delete[] timeBufferRight;
    delete[] logChip_LUT;
    printf("logUnmask : freeing memory allocated by event buffers \n");
    delete[] bufferCD;
    delete[] bufferIF;
    delete[] bufferEM1;
    delete[] bufferEM2;
    delete[] bufferEM3;
    delete[] bufferEM4;
    //delete[] cartEM;
    //delete[] pEM;
}

bool logUnmask::threadInit() {
    // initialising the logChip_LUT
    fout = fopen("asvLUT.txt", "w");
    
    /*  // structure of the LUT for logpolar Chip
    // type, metaX, metaY, polarity
    logChip_LUT[0  ][0] = 1 ; logChip_LUT[0  ][1] = 0 ; logChip_LUT[0  ][2] = 0 ; logChip_LUT[0 ][3] = 1;
    logChip_LUT[1  ][0] = 1 ; logChip_LUT[1  ][1] = 0 ; logChip_LUT[1  ][2] = 0 ; logChip_LUT[1 ][3] = 0;
    logChip_LUT[6  ][0] = 5 ; logChip_LUT[6  ][1] = 0 ; logChip_LUT[6  ][2] = 0 ; logChip_LUT[6 ][3] = 1;
    logChip_LUT[10 ][0] = 2 ; logChip_LUT[10 ][1] = 0 ; logChip_LUT[10 ][2] = 0 ; logChip_LUT[10][3] = 1;
    logChip_LUT[11 ][0] = 2 ; logChip_LUT[11 ][1] = 0 ; logChip_LUT[11 ][2] = 0 ; logChip_LUT[11][3] = 0;
    logChip_LUT[12 ][0] = 1 ; logChip_LUT[12 ][1] = 0 ; logChip_LUT[12 ][2] = 2 ; logChip_LUT[12][3] = 1;
    logChip_LUT[13 ][0] = 1 ; logChip_LUT[13 ][1] = 0 ; logChip_LUT[13 ][2] = 2 ; logChip_LUT[13][3] = 0;
    logChip_LUT[18 ][0] = 5 ; logChip_LUT[18 ][1] = 0 ; logChip_LUT[18 ][2] = 2 ; logChip_LUT[18][3] = 1;
    logChip_LUT[22 ][0] = 2 ; logChip_LUT[22 ][1] = 0 ; logChip_LUT[22 ][2] = 2 ; logChip_LUT[22][3] = 1;
    logChip_LUT[23 ][0] = 2 ; logChip_LUT[23 ][1] = 0 ; logChip_LUT[23 ][2] = 2 ; logChip_LUT[23][3] = 0;
    logChip_LUT[24 ][0] = 1 ; logChip_LUT[24 ][1] = 0 ; logChip_LUT[24 ][2] = 4 ; logChip_LUT[24][3] = 1;
    logChip_LUT[25 ][0] = 1 ; logChip_LUT[25 ][1] = 0 ; logChip_LUT[25 ][2] = 4 ; logChip_LUT[25][3] = 0;
    logChip_LUT[30 ][0] = 5 ; logChip_LUT[30 ][1] = 0 ; logChip_LUT[30 ][2] = 4 ; logChip_LUT[30][3] = 1;
    logChip_LUT[34 ][0] = 2 ; logChip_LUT[34 ][1] = 0 ; logChip_LUT[34 ][2] = 4 ; logChip_LUT[34][3] = 1;
    logChip_LUT[35 ][0] = 2 ; logChip_LUT[35 ][1] = 0 ; logChip_LUT[35 ][2] = 4 ; logChip_LUT[35][3] = 0;

    logChip_LUT[38 ][0] = 0 ; logChip_LUT[38 ][1] = 0 ; logChip_LUT[38 ][2] = 0 ; logChip_LUT[38 ][3] = 1;
    logChip_LUT[39 ][0] = 0 ; logChip_LUT[39 ][1] = 0 ; logChip_LUT[39 ][2] = 0 ; logChip_LUT[39 ][3] = 0;
    logChip_LUT[50 ][0] = 0 ; logChip_LUT[50 ][1] = 0 ; logChip_LUT[50 ][2] = 2 ; logChip_LUT[50 ][3] = 1;
    logChip_LUT[51 ][0] = 0 ; logChip_LUT[51 ][1] = 0 ; logChip_LUT[51 ][2] = 2 ; logChip_LUT[51 ][3] = 0;
    logChip_LUT[62 ][0] = 0 ; logChip_LUT[62 ][1] = 0 ; logChip_LUT[62 ][2] = 4 ; logChip_LUT[62 ][3] = 1;
    logChip_LUT[63 ][0] = 0 ; logChip_LUT[63 ][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0; 

    logChip_LUT[216][0] = 0 ; logChip_LUT[216][1] = 0 ; logChip_LUT[216][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[217][0] = 0 ; logChip_LUT[217][1] = 0 ; logChip_LUT[217][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[222][0] = 0 ; logChip_LUT[222][1] = 0 ; logChip_LUT[222][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[226][0] = 0 ; logChip_LUT[226][1] = 0 ; logChip_LUT[226][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[227][0] = 0 ; logChip_LUT[227][1] = 0 ; logChip_LUT[227][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[228][0] = 0 ; logChip_LUT[228][1] = 0 ; logChip_LUT[228][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[229][0] = 0 ; logChip_LUT[229][1] = 0 ; logChip_LUT[229][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[232][0] = 0 ; logChip_LUT[232][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[233][0] = 0 ; logChip_LUT[233][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[234][0] = 0 ; logChip_LUT[234][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[235][0] = 0 ; logChip_LUT[235][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[238][0] = 0 ; logChip_LUT[238][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[239][0] = 0 ; logChip_LUT[239][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[240][0] = 0 ; logChip_LUT[240][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[241][0] = 0 ; logChip_LUT[241][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[246][0] = 0 ; logChip_LUT[246][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[250][0] = 0 ; logChip_LUT[250][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[251][0] = 0 ; logChip_LUT[251][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    */

    int type, metax, metay, pol;  // feature listed in the same order as in the LUT
    bool fovea;

    for (int x = 0; x < X_DIMENSION; x++ ) {
        for (int y = 0; y < Y_DIMENSION; y++) {

            // determining whether we are mapping a pixel of the fovea
            
            if((y >= 24) && (y < 48) && (x >= 48) && (x < 96)) {
                fovea = true;
            }
            else {
                fovea = false;
            }
            
            // ------------  CD --------------------------------
            if( ((x - 2) % 6 == 0) || ((x - 3) % 6 == 0)){
            // CD
                type  = 0;
                metax = (int) (x - 2) / 6;
                metay = (int) (y - 1) / 3;
                /*
                if((x == 57)||(x == 69)||(x == 81)||(x == 93)) {
                    pol = 0;
                }
                else if((x == 58)||(x == 70)||(x == 82)||(x == 94)) {
                    pol = 1;
                }
                
                else*/
                if(x % 2 == 0) {
                    pol = 0;
                }
                else {
                    pol = 1; 
                }

            } // CD            
            
            // ------------------  IF   ---------------------------
            // if is only present in perifery
            if(!fovea) {
                if ((x - 6) % 12 == 0){
                    // IFON
                    type  = 5;
                    metax = (int) (x - 6) / 6;
                    metay = (int)  y / 3 ;
                    pol = 0;                                  
                } // IFON
                
                if ((x - 7) % 12 == 0){
                    // IFOFF
                    type  = 5;
                    metax = (int) (x - 7) / 6;
                    metay = (int) (y / 3) - 1;
                    pol = 1;                                  
                } // IFOFF  
            }
            
            // ----------------   EM    --------------------------            
            // EM             
            if(fovea) {
                //fovea
                if(((x -2) % 6 != 0) && ((x-3) % 6 != 0)) {    // avoiding CD position in fovea
                    //if(((x - 48 + 3 ) / 6) % 2!= 0)  {
                    if(((x - 48 ) / 3) % 2 != 0) {
                        //second half of the photoreceptor
                        if (y % 3 == 0) {
                            // EM2
                            type = 2;
                            metax = x / 6;
                            metay = y / 3;
                            if(x % 2 == 0 ) {
                                pol = 1;
                            }
                            else {
                                pol = 0;
                            }
                        }                      
                        else {
                            // EM4
                            type = 4;
                            metax = x / 6;
                            metay = y / 3;
                            if(x % 2 == 0) {
                                pol = 1;
                            }
                            else {
                                pol = 0;
                            }
                        }                    
                    }  // 
                    else {
                        // first part of the photoreceptor
                        if (y % 3 == 0) { 
                            // EM1
                            type = 1;
                            metax = x / 6;                        
                            metay = y / 3;                      
                            if(x % 2 == 0) {
                                pol = 1;
                            }
                            else {
                                pol = 0;
                            }
                        }
                        else { 
                            // EM3
                            type = 3;
                            metax = x / 6;
                            metay = y / 3;
                            if(x % 2 == 0) {
                                pol = 1;
                            }
                            else {
                                pol = 0;
                            }
                        }
                    } // end outer colums
                } // end if not CD                         
            }// end fovea                
            else {
                //periphery
                if ( ( x            % 12 == 0 ) && (  y % 6      == 0) ) {
                    type = 1;
                    metax =  x / 6;
                    metay =  y / 3;
                    pol = 1;
                }
                else if ( ((x - 1 ) % 12 == 0 ) && (  y % 6      == 0) ) {
                    type = 1;
                    metax =  x / 6;
                    metay =  y / 3;
                    pol = 0;
                }
                else if ( ((x - 10) % 12 == 0 ) && (  y % 6      == 0) ) {
                    type = 2 ;
                    metax = (x - 10) / 6;
                    metay =  y / 3;
                    pol = 1;
                    
                }
                else if ( ((x - 11) % 12 == 0 ) &&  (  y % 6      == 0) ) {
                    type = 2 ;
                    metax = (x - 10) / 6;
                    metay =  y / 3;
                    pol = 0;
                }
                else if ( ( x       % 12 == 0 ) &&  ( (y - 5) % 6 == 0) ) { 
                    type= 3;
                    metax =  x / 6;
                    metay = (y - 5)  / 3;
                    pol = 1;
                }
                else if ( ((x - 1)  % 12 == 0 ) && ( (y - 5) % 6 == 0) ) { 
                    type= 3;
                    metax =  x / 6;
                    metay = (y - 5)  / 3;
                    pol = 0;
                }
                else if ( ((x - 10) % 6 == 0 ) && ( (y - 5) % 3 == 0) ) {
                    type= 4;
                    metax = (x - 10) / 6;
                    metay = (y - 5)  / 3;
                    pol = 1;
                }
                else if ( ((x - 11) % 6 == 0 ) && ( (y - 5) % 3 == 0) ) {
                    type= 4;
                    metax = (x - 10) / 6;
                    metay = (y - 5)  / 3;
                    pol = 0;
                }
            } // periphery  

            // covering with -1 regions that are blank            
            if(!fovea){
                int posX = x / 12;
                int withinX = x - posX * 12;
                switch(withinX) {
                case 4:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;                
                case 5:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;                
                case 8:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;
                case 9:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;
                }

                int posY    = y / 6;
                int withinY = y - posY * 6;
                switch(withinY) {
                case 2:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;                
                case 3:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;                
                case 4:{
                    type  = -1;
                    metax = -1;
                    metay = -1;
                    pol   = -1;
                }break;                
                }
                
            }
           
            
            
            // saving the extracted information in a LUT respecting the order 
            // type, metax, metay, polarity
            logChip_LUT[y * X_DIMENSION + x][0] = type;
            logChip_LUT[y * X_DIMENSION + x][1] = metax;
            logChip_LUT[y * X_DIMENSION + x][2] = metay;
            logChip_LUT[y * X_DIMENSION + x][3] = pol;

            //printf("%d %d \n", x, y);
            fprintf(fout,"# %d %d %d %d %d %d \n", x, y, metax, metay, pol, type);

           
        }        
    }

     fclose(fout);

    //starting processing thread    
    pThread = new processingThread();
    pThread->setEM(bufferEM1, bufferEM2, bufferEM3, bufferEM4);
    pThread->start();
    
    return true;
}



void logUnmask::cleanEventBuffer() {
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    memset(timeBuffer, 0, retinalSize * retinalSize * sizeof(unsigned long));
    minValue=0;
    maxValue=0;
}

int logUnmask::getMinValue() {
    return minValue;
}

int logUnmask::getMaxValue() {
    return maxValue;
}

void logUnmask::getCD(aer** pointerCD, int* dimCD) {
    *pointerCD = bufferCD;
    *dimCD = countCD;     
    /*for (int i = 0; i <= countCD ; i++) {        
        unsigned long blob      = (u32) bufferCD[i].address;
        unsigned long timestamp = (u32) bufferCD[i].timestamp;
        fprintf(fout,"%d > %08x %08x \n",i,blob,timestamp);
        }*/
    
}

void logUnmask::getIF(aer** pointerIF, int* dimIF) {
    //printf("counted IF %d \n", countIF);
    *pointerIF = bufferIF;
    *dimIF = countIF;
}

/*
void logUnmask::getEM(aer** pointerEM, int* dimEM) {
    //printf("counted EM %d \n", countEM);
    //*pointerEM = bufferEM1;
    //*dimEM = countEM1 + countEM2 + countEM3 + countEM4;
    *dimEM = 0;
    
    for (int pos = 0; pos < 24 * 24; pos++){
        if(cartEM[pos].address != 0) {
            //printf("cartEM[pos] %08x \n",cartEM[pos].address);
            pEM[*dimEM] = cartEM[pos];
            (*dimEM)++;            
        }
    }
    *pointerEM = pEM;
    //delete[] pEM;
    memset(cartEM,0, 24 * 24 * sizeof(aer));
}
*/

void logUnmask::getEM(aer** pointerEM, int* dimEM) {
    pThread->getEM(pointerEM, dimEM);
}


unsigned long logUnmask::getLastTimestamp() {
    return lasttimestamp;
}

unsigned long logUnmask::getLastTimestampRight() {
    return lasttimestampright;
}

unsigned long logUnmask::getEldestTimeStamp() {
    return eldesttimestamp;
}

void logUnmask::setLastTimestamp(unsigned long value) {
    lasttimestamp = value;
}

int* logUnmask::getEventBuffer(bool camera) {
    if(camera)
        return buffer;
    else
        return bufferRight;
}

void logUnmask::resetTimestamps() {
    for (int i=0 ; i<retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
        timeBufferRight[i] = 0;
    }
    //verb = true;
    //lasttimestamp = 0;
    //lasttimestampright = 0;  
}

unsigned long* logUnmask::getTimeBuffer(bool camera) {
    if (camera)
        return timeBuffer;
    else
        return timeBufferRight;
}

void logUnmask::run() {
}

void logUnmask::logUnmaskData(char* i_buffer, int i_sz, bool verb) {
    //cout << "Size of the received packet to logUnmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    //AER_struct sAER
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //printf("logUnmakData: unmasking %d  \n", num_events);
    //create a pointer that points every 4 bytes
    uint32_t* buf2 = (uint32_t*)i_buffer;
    //eldesttimestamp = 0; 

    

    for (int evt = 0; evt < num_events; evt++) {
        
        // logUnmask the data ( first 4 byte blob, second 4 bytes timestamp)
        unsigned long blob      = buf2[2 * evt];
        unsigned long timestamp = buf2[2 * evt + 1];
        
        lasttimestamp = timestamp;

        //check for addresses greater than 0x0000FFFF
        if(blob > 0x0000FFFF) {
            continue; //skipping this event continuing with the rest of the block
        }
                
        // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        //blob &= 0xFFFF;
        type = -1;
  
        // saving the events
        bool save = false;                
        if (save) {
            fprintf(fout,"=%08X %08X\n",blob,timestamp); 
            //fout<<hex<<a<<" "<<hex<<t<<endl;
        }

        // unsigned short x    = ((blob & xmask) >> xshift);
        //x -= 112;
        //unsigned short y    = ((blob & ymask) >> yshift);        
        //int flipy = flipBits(y,7);
        //y = flipy - 56;
        //printf("####### \n %08X %d %d > %d %d %d \n",blob, x, y, cartX, cartY, type);
                
        logUnmaskEvent((unsigned long)blob, cartX, cartY, polarity, type);               
        
        //cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        //cartX = retinalSize - cartX;
        int metaX, metaY, pol;
        unsigned long int newBlob;

        //if((x >= 144)||(y >= 72)) {
        //    metaX = 0;
        //    metaY = 0;
        //    pol   = 0;
        //}
        //else{    
        metaX = cartX;
        metaY = cartY;
        //after unmasking polarity is 0: off events 1:on events
        pol   = polarity;
        
            //}
        
        // logMaskEvent takes as input 0:off events 1:on events
        logMaskEvent(metaX,metaY,pol,newBlob);        
        
        unsigned long timestamp_diff = timestamp - previous_timestamp;
        //printf(" Error : %08X>%d,%d,%d   %08X \n",blob, cartX, cartY, polarity, newBlob);
        //fprintf(fout," %d %d %d %d %d %08X %08X : %d \n",cartY, cartX, metaY, metaX ,type,previous_timestamp, timestamp, timestamp_diff);
        
        //if((timestamp_diff > 1000000000 )&&(timestamp != 0)&&(timestamp > previous_timestamp)) {
        //    printf(" Error : %08X>%d,%d   \n",blob,cartX,cartY);
        //    printf(" %d %d %d %d %d %08X %08X : %d \n",cartY, cartX, metaY, metaX ,type,previous_timestamp, timestamp, timestamp_diff);
        //}

        if(timestamp!=0) {
            previous_timestamp = timestamp;
        }
                    
        struct aer* temp;        
        switch (type) {
        case 0:{ //CD            
            //if((newBlob!=0)||(timestamp!=0)) {
            //temp = &bufferCD[countCD];                    
            //printf("Unmasked CD  %x \n", bufferCD);
            //printf("CD \n");
            mutex.wait();
            if(countCD < SIZE_OF_EVENT) {
                bufferCD[countCD].address   = (u32) newBlob;
                bufferCD[countCD].timestamp = timestamp;
                //printf("%08X %08X  \n",bufferCD[countCD].address,timestamp);
                //fprintf(fout,">%d %08X %08X\n",countCD,bufferCD[countCD].address,bufferCD[countCD].timestamp);
                countCD++;   
            }
            else {
                printf("bufferCD overflow prevented! %d \n", countCD);
            }
            mutex.post();
        }
            break;
            /*    
        case 1:{ //EM1
            //printf("Unmasked EM1 \n");
            if((blob!=0)||(timestamp!=0)) {
                mutex.wait();
                if(countEM1 < SIZE_OF_EVENT) {
                    //temp = &bufferEM[countEM];
                    //temp->address   = blob;
                    //temp->timestamp = timestamp;
                    //printf("EM1 metax %d metay %d \n", metaX, metaY);
                    bufferEM1[countEM1].address   = (u32) newBlob;
                    bufferEM1[countEM1].timestamp = (u32) timestamp;
                    countEM1++;
                    pThread->setCountEM1(countEM1);
                }
                else {
                    printf("bufferEM1 overflow prevented! %d \n", countEM1);
                }
                mutex.post();
            }
        } //EM1
            break;
        case 2:{ //EM2
            //printf("Unmasked EM2 \n");
            if((blob!=0)||(timestamp!=0)) {
                mutex.wait();
                if(countEM2 < SIZE_OF_EVENT) {
                    //printf("EM2 metax %d metay %d \n", metaX, metaY);
                    //temp = &bufferEM[countEM];
                    //temp->address   = blob;
                    //temp->timestamp = timestamp;
                    bufferEM2[countEM2].address   = (u32) newBlob;
                    bufferEM2[countEM2].timestamp = (u32) timestamp;
                    countEM2++;
                    pThread->setCountEM2(countEM2);
                }
                else {
                    printf("bufferCD overflow prevented! %d \n", countEM2);
                }
                mutex.post();
            }               
        } // EM2
            break;
        case 3:{ //EM3
            //printf("Unmasked EM3 \n");
            if((blob!=0)||(timestamp!=0)) {
                mutex.wait();
                if(countEM3 < SIZE_OF_EVENT) {
                    //printf("EM3 metax %d metay %d \n", metaX, metaY);
                    //temp = &bufferEM[countEM];
                    //temp->address   = blob;
                    //temp->timestamp = timestamp;
                    bufferEM3[countEM3].address   = (u32) newBlob;
                    bufferEM3[countEM3].timestamp = (u32) timestamp;
                    countEM3++;
                    pThread->setCountEM3(countEM3);
                }
                else {
                    printf("bufferEM3 overflow prevented! %d \n", countEM3);
                }
                mutex.post();
            } // end if blob            
        } //EM3
        break;
        case 4:{ //EM4
            //printf("Unmasked EM4 \n");
            if((blob!=0)||(timestamp!=0)) {
                mutex.wait();
                if(countEM4 < SIZE_OF_EVENT) {
                    //printf("EM4 metax %d metay %d \n", metaX, metaY);
                    //temp = &bufferEM[countEM];
                    //temp->address   = blob;
                    //temp->timestamp = timestamp;
                    bufferEM4[countEM4].address   = (u32) newBlob;
                    bufferEM4[countEM4].timestamp = (u32) timestamp;
                    countEM4++;
                    pThread->setCountEM4(countEM4);
                }
                else {
                    printf("bufferEM4 overflow prevented! %d \n", countEM4);
                }
                mutex.post();
            }
        } //EM4
            break;           
        case 5:{ //IF
                //printf("Unmasked IF \n");
                if((blob!=0)||(timestamp!=0)) {
                    mutex.wait();
                    if(countIF < SIZE_OF_EVENT) {
                        //temp = &bufferIF[countIF];
                        //temp->address   = blob;
                        //temp->timestamp = timestamp;
                        bufferIF[countIF].address   = (u32) newBlob;
                        bufferIF[countIF].timestamp = (u32) timestamp;
                        countIF++;
                    }
                    else {
                        printf("bufferIF overflow prevented! %d \n", countIF);
                    }
                    mutex.post();
                }
        }// case 5
            break;
            */
        }// end of switch            
    } // end of for every event    
}

void logUnmask::logUnmaskEvent(unsigned int evPU, short& metax, short& metay, short& pol, short& type) {
    int y = (retinalSize-1) - ((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    int x = ((evPU & ymask) >> yshift);
    // 2.extractiong features 
    int position =  y * X_DIMENSION + x;

    //feature* pFeature = logChip_LUT;
    //pFeature += position;
    //short* pcols = pFeature;
    //type  = pcols;
    //pcols++;
    //metax = pcols;
    //pcols++;
    //metay = pcols;
    //pcols++;
    //pol   = pcols;    

    type  = logChip_LUT[position][0];
    metax = logChip_LUT[position][1];
    metay = logChip_LUT[position][2];
    pol   = logChip_LUT[position][3];
    
    //printf("unmasked event %d %d %d %d \n",type, metax, metay, pol);
    //pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    //type = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
}

void logUnmask::logUnmaskEvent(unsigned long evPU, short& metax, short& metay, short& pol, short& type) {
    //unmasking through LUT    
    // 1.determining the position in the LUT
    short x     = ((evPU & xmask) >> xshift);
    short y     = ((evPU & ymask) >> yshift);
    //printf("y %d ", y);
    short flipy = flipBits(y,7);
    //printf("   flipy %d \n", flipy);
    y = flipy - 56;
    x = x - 112;
    if((y < 0) || (x < 0)) {
        return;
    }
    // 2.extractiong features 
    int position =  y * X_DIMENSION + x;
    
    //feature* pFeature = logChip_LUT;
    //pFeature += position;
    //short* pcols = pFeature;
    //type  = pcols;
    //pcols++;
    //metax = pcols;
    //pcols++;
    //metay = pcols;
    //pcols++;
    //pol   = pcols;    
    if (position > X_DIMENSION * Y_DIMENSION) {
        printf("Error in logUnmaskEvent %d %d \n",x,y);
    }

    type  = logChip_LUT[position][0];
    metax = logChip_LUT[position][1];
    metay = logChip_LUT[position][2];
    pol   = logChip_LUT[position][3];
  
    if (type > 6) {
        printf("error in logUnmask Event %d %d \n ",x,y);
        printf("%d %d %d %d %x \n", type, metax, metay,pol, logChip_LUT);
        type  = 0;
        metax = 0;
        metay = 0;
        pol = 0;
    }
    if((metax >= 24)||(metay >= 24)){
        printf("Error in max dimension out \n");
        metax = 0; metay = 0;
    }
  
    //printf("unmasked event %d %d %d %d from %d %d \n",type, metax, metay, pol, x, y);
}

void logUnmask::logMaskEvent( short metax, short metay, short pol, unsigned long& evPU) {
    //metax += retinalSize +1;
    //evPU = 0;
    //evPU += (pol   << polshift) ;
    //evPU += (metax & 0x0000007f ) << 1   ;//& xmask;   //shifting one bit
    //evPU += (metay & 0x000000ff ) << 8   ;//& ymask;   // shifting 8 bits
    //& polmask;   

    evPU = 0;
    evPU |= pol;
    evPU |= (metax & 0x0000007f ) << 1;
    evPU |= (metay & 0x000000ff ) << 8;
 
}

void logUnmask::threadRelease() {
    //no istruction in threadInit
    printf("logUnmask::threadRelease closing the dumping file \n");
    //fclose(fout);
    printf("logUnmask::threadRelease successfully close the dumping file \n");
    pThread->stop();
    printf("logUnmask::threadRelease: success in stoping processing thread");
}




//----- end-of-file --- ( next line intentionally left blank ) ------------------

