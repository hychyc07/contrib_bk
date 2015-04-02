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
 * @file cudaThread.cpp
 * @brief Implementation of the thread (see header cudaThread.h)
 */

#include <iCub/cudaThread.h>
#include <cstring>
#include <cassert>
#include <cstdio>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// declaration, forward
extern "C" void runTest(const int argc, const char** argv, 
                        char* data, int2* data_int2, unsigned int len);


#define THRATE 100

cudaThread::cudaThread() : Thread() {
    resized=false;
    count=0;
}

cudaThread::~cudaThread() {
}

bool cudaThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    return true;
}

void cudaThread::interrupt() {
    outPort.interrupt();
}

void cudaThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

void cudaThread::setArgc(int a) {
    argc = a;
}

void cudaThread::setArgv(char* a) {
    for (int i=0;i<argc;i++)
        argv[i] = a++;
}

std::string cudaThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void cudaThread::resize(int widthp, int heightp) {
    width=widthp;
    height=heightp;
}

void cudaThread::run() {
    /*
    // input data
    int len = 16;
    // the data has some zero padding at the end so that the size is a multiple of
    // four, this simplifies the processing as each thread can process four
    // elements (which is necessary to avoid bank conflicts) but no branching is
    // necessary to avoid out of bounds reads
    char str[] = { 82, 111, 118, 118, 121, 42, 97, 121, 124, 118, 110, 56,
                   10, 10, 10, 10};

    // Use int2 showing that CUDA vector types can be used in cpp code
    int2 i2[16];
    for( int i = 0; i < len; i++ )
    {
        i2[i].x = str[i];
        i2[i].y = 10;
    }

    // run the device part of the program
    runTest(argc, (const char**)argv, str, i2, len);

    std::cout << str << std::endl;
    for( int i = 0; i < len; i++ )
    {
        std::cout << (char)(i2[i].x);
    }
    std::cout << std::endl;
    */

   
}

void cudaThread::threadRelease() {
    //closing ports
    outPort.close();
    //cutilExit(argc, argv);
}
