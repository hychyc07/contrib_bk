// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Katrin Lohan
  * email: katrin.lohan@iit.it
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



#ifndef simulatedTracker_H
#define simulatedTracker_H


#include <iostream>
#include <stdio.h>
#include <yarp/os/all.h>
#include <algorithm>



using namespace std;
using namespace yarp::os;


class simulatedTracker {
private:
    
    std::string file;                      
public:
   // simulatedTracker();
	//void annotationFileReader();
    //void init();
    int main(); 
    
};


#endif // simulatedTracker_H
