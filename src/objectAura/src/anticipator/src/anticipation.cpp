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
#include "iCub/anticipation.h"
#include <stdio.h>
#include <math.h>

using namespace std;

//*********************//
//For the target ObjectA=1; ObjectB=2; ObjectC=3; Hand =4; Bucket=5; Body = 6
//*********************//
anticipation::anticipation(){

}

void anticipation::anticipate( double x, double y){

   
    Vector px(2);
    px(0) = x;
    px(1) = y;
    int camSel = 0;
    igaze->lookAtMonoPixel(camSel,px,0.5); 
    
	printf ("Anticipation is running.\n");
}

void anticipation::init(){
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/ObjectAura/gaze");
    localCon.append("");
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    }
