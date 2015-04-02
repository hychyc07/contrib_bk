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
 * @file Element.h
 * @brief definition of the module that creates a structure from scratch
 */


#ifndef _ELEMENT_H_
#define _ELEMENT_H_

#include <iostream>
#include <list>
#include <map>
#include <string>

using namespace std;

class Element{
protected:
    std::string name;       // specific name of the row
    long energy;            // energy of the row globally calculated
    //map<std::string,Unit> unitList; 
    //map<std::string,Connection> connectionList;
public:
    Element();
    ~Element();     //default destructor
    virtual std::string toString();
};

#endif //_ELEMENT_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
