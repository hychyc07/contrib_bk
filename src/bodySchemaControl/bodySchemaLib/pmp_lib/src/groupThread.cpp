// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file jointThread.cpp
 * @brief Implementation of the visual filter thread (see earlyMotionThread.h).
 */

#include <pmp_lib/groupThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

using namespace iCub::pmplib;


jointThread::jointThread() {
    idle  = true;
}

jointThread::~jointThread() {
  
}

bool jointThread::threadInit() {
    /* open ports */ 
   // printf("Initialization of the %s \n", getName(" ").c_str());
    return true;
}

void jointThread::activate(){
    idle = false;
    jointSemaphore->check();
  //  printf("%s just activated..... \n", getName(" ").c_str());
}

void jointThread::deactivate(){
    idle = true;
  //  printf("%s just deactivated..... \n", getName(" ").c_str());
}

void jointThread::setName(string str) {
    this->name=str;
    //printf("name: %s \n", name.c_str());
}

void jointThread::setId(int id) {
    this->id=id;
    //printf("id: %s \n", name.c_str());
}

std::string jointThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

int jointThread::getId() {
    return id;
}

void jointThread::run()
{
	while (!isStopping()) {
		if (!idle)	while(runnable());//{cout << "..." << endl;Time::delay(0.1);}

		idle = true;
		jointSemaphore->post();		
		//Time::delay(0.1);
	}
};

void jointThread::threadRelease() {
 
}

void jointThread::onStop() {
}

