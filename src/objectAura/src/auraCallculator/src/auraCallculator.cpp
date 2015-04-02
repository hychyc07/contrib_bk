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



#include <iostream>
#include "iCub/auraCallculator.h"
#include <stdio.h>
#include <math.h>

using namespace std;

//*********************//
//For the target ObjectA=1; ObjectB=2; ObjectC=3; Hand =4; Bucket=5; Body = 6
//*********************//
auraCallculator::auraCallculator(){
printf ("auraCallculator is running.\n");

}

bool auraCallculator::movement(double objectx, double objecty, double objectx_old, double objecty_old){

    double d1, d2, d;

    if(objectx != -1.0 && objecty != -1.0 && objectx_old != -1.0 && objecty_old != -1.0){
	    d1 = (objectx-objectx_old)*(objectx-objectx_old);
	    d2 = (objecty-objecty_old)*(objecty-objecty_old);
	    d = sqrt(d1+d2);
        if(d > 3){
            return true;
        }
        else{return false;}
    }
    if(objectx != -1.0 && objecty != -1.0 || objectx_old != -1.0 && objecty_old != -1.0){
	    d1 = (objectx-objectx_old)*(objectx-objectx_old);
	    d2 = (objecty-objecty_old)*(objecty-objecty_old);
	    d = sqrt(d1+d2);
        if(d > 4){
            return true;
        }
        else{return false;}
    }
    else{return false;}
   	
}

double auraCallculator::size(double leftCornerx, double leftCornery, double rightCornerx, double rightCornery){
    double ax,ay,a;
    ax = (leftCornerx- rightCornerx);
    ay = (leftCornery- rightCornery);
    a = (ax*ay);
	return a;
	
	
}

void auraCallculator::init(){
    
    
    }
