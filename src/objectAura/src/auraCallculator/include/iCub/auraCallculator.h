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



#ifndef auraCallculator_H
#define auraCallculator_H


#include <iostream>
#include <stdio.h>




using namespace std;


class auraCallculator {
private:
    
public:
	auraCallculator();
    void init();
	bool movement(double Objectx, double Objecty, double Objectx_old, double Objecty_old);
    double size(double leftCornerx, double leftCornery, double rightCornerx, double rightCornery);
    
};


#endif // auraCallculator_H
