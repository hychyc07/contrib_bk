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
 * @file fileFeederThread.cpp
 * @brief Implementation of the thread (see header fileFeederThread.h)
 */

#include <iCub/fileFeederThread.h>
#include <fstream>
#include <cstring>
#include <cassert>
#include <cstdio>
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 500
#define LIMIT 390
#define EXTRA_LIMIT 2

fileFeederThread::fileFeederThread() : RateThread(THRATE) {
    resized=false;
    img=0;
    count=0;
}

fileFeederThread::~fileFeederThread() {
}

bool fileFeederThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/data:o").c_str());

    std::string filenameTrainingData ("data.txt");

    file = new ifstream(filenameTrainingData.c_str(), std::ios::in);
    if (!file->is_open()) {
        cout << "Could not open file " << filenameTrainingData.c_str() << "for reading!" << endl;
        return false;
    }
    // reading the file.
    double e;
    maxcount = 0;
    while (!file->eof()) {
        (*file) >> e;
        if(!file->eof()) {
            printf("%f ",e);
            maxcount++;
            samplesE.push_back(e);
        }
    }
    printf(" \n maxCount %d", maxcount);
    return true;
}

void fileFeederThread::interrupt() {
    outPort.interrupt();
}

void fileFeederThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string fileFeederThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void fileFeederThread::resize(int widthp, int heightp) {
    width=widthp;
    height=heightp;
}

void fileFeederThread::run() {

    Bottle b;
    if(!outPort.getOutputCount()){
        return;
    }
    Bottle& databottle = outPort.prepare();
    double e;
    //databottle.clear();
    //b.addDouble(1);
    //databottle.append(b);
    //outPort.write();
    //databottle.clear();

    
    int cnt = 0;
    databottle.clear();
    while((cnt < LIMIT)&&(count < maxcount)) {
        e = samplesE(count);
        cnt++;
        count++;
        printf("%d ",count);
        
        //b.clear();
        b.addDouble(e);
        databottle.append(b);       
    }   
    
    
    outPort.write();
    printf("* \n");
    cnt = 0;
    while(cnt < EXTRA_LIMIT) {
        e = samplesE(count);
        cnt++;
        count++;
    }
}

void fileFeederThread::threadRelease() {
    outPort.close();
}
