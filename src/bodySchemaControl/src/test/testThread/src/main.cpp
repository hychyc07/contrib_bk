// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @brief main code for the early filter module; this is part of the logpolar attention system.
 */


#include <yarp/os/all.h>
#include <iCub/jointThread.h>

int main(int argc, char * argv[])
{
    
    yarp::os::Network  yarp;
    yarp::os::Semaphore*   s = new yarp::os::Semaphore();
    
    jointThread a , b , c;
    std::string name;
    name = "threadA";
    a.setName(name);
    a.setResource(s);
    a.start();

    
    name = "threadB";
    b.setName(name);
    b.setResource(s);
    b.start();
    

    
    name = "threadC";
    c.setName(name);
    c.setResource(s);
    c.start();
    
    yarp::os::Time::delay(3.0);
    a.activate();
    b.activate();
    c.activate();
    
    // waiting for the resource to be ready
    printf("waiting for the resource to be ready .......... \n");
    s->wait();
    printf("resource freed \n");
    s->post();
    

    while(true) {}
    return 0;
}


