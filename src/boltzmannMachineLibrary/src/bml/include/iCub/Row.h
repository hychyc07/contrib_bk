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
 * @file Row.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _ROW_H_
#define _ROW_H_


#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Element.h>

#include <iostream>
#include <list>
#include <map>
#include <string>

using namespace std;

class Row : public Element{	
private:
    static const int BINARY_THR=1;
    static const int PROBAB_THR=2;
    static const int BOTH_RULES=3;
public:
    Row();//default constructor

    ~Row();//default destructor

    /**
    * constructor of n stardard units and null connections
    * @param name name of the row
    * @param n number of element in the row
    */
    Row(std::string name,int n); 

    /**
    * returns the energy level of the machine
    */
    int getEnergy();

    /**
    * returns the name of this row
    */
    std::string getName();//

    //int getNumUnits(); 
    /**
    * add a unit to the unitList
    * @param unitA reference to a Unit
    */
    void addUnit(Unit unitA);

    /**
    * add a unit to the ConnectionList
    * @param unitA reference to the first Unit
    * @param unitB reference to the second Unit
    * @param weight the weight of the connection
    */
    void addConnection(Unit unitA, Unit unitB, int weight); 

    map<std::string,Connection>::iterator getConnectionListIteratorBegin();     //get the iterator of the connectionList
    map<std::string,Connection>::iterator getConnectionListIteratorEnd();       //get the iterator of the connectionList
    map<std::string,Unit>::iterator getUnitListIteratorBegin();                 //get the iterator of the connectionList
    map<std::string,Unit>::iterator getUnitListIteratorEnd();                   //get the iterator of the connectionList

    std::string toString(); //code the class into a std::string

    /**
    * the states of the units in the Row are updated based on the choosen rules
    * @param rule indicates that evolution rule choosen
    */
    void evolve(int rule); 

    map<std::string,Unit> unitList;                 //list of units of the actual row
    map<std::string,Connection> connectionList;     //list of connection of the actual row
};
#endif //_UNIT_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
