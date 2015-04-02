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
 * @file Unit.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _UNIT_H_
#define _UNIT_H_



#include <iCub/Connection.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Random.h>

#include <iostream>
#include <map>
#include <string>
#include <math.h>
using namespace std;
using namespace yarp::math;
using namespace yarp::os;

class Unit{
private:
    int T;                          //"temperature" parameter direct referenced to the temperature of the Boltzmann Machine
    int bias;
    long DEnergy;                   //the Energy GAP of the unit locally determined
    std::string name;               //the coded name of the Unit
    static const int BINARY_THR=1;
    static const int PROBAB_THR=2;
    static const int BOTH_RULES=3;
    double probFired;               //probability of the unit to be ON
    bool evolveBinary;
    bool evolveStochastic;
    double stochasticThrHIGH;
    double stochasticThrLOW;
public:
    /**
    * default constructor
    */
    Unit();

    /**
    * default destructor
    */
    ~Unit();

    /**
    * constructor
    * @param name of the unit
    */
    Unit(std::string name);

    /**
    * constructor
    * @param name of the unit
    * @param state of the unit
    */
    Unit(std::string name,int state);

    /**
    * return the probability that the unit is fired
    */
    double getProbFired();

    /**
    * returns the name of the unit
    * @return string for the name
    */
    std::string getName();

    int getBias();

    /**
    * returns the state of the unit
    */
    int getState();

    /**
    * add a connection to the unit
    * @param connection connection that is going to be added
    */
    void addConnection(Connection* connection);

    /**
    * set the weight(int) in the row of connections in a precise position
    * @param pos position in the collection of weights
    * @param weight new weight to be set
    */
    void setConnectionWeight(int pos, int weight); 

    /**
    * get the weight(int) in the row of connections in a precise position
    * @param pos position in the collection of weights
    */
    int getConnectionWeight(int pos);

    /**
    * add a series of connection Weight to the listWeight
    */
    void addConnectionWeight(int nConnectionWeight);

    /**
    * add a series of connection Weight to the listWeight
    */
    void addConnectionWeight(int nConnectionWeight,int weight);

    /**
    *calculates the local energy of a single unit
    */
    void calculateDEnergy();

    /**
    *returns the local energy of a single unit
    */
    int getDEnergy(); 

    std::string toString(); //code the class into a std::string

    /**
    * the state of this units is updated based on the choosen rules
    * @param rule const that indicates which rule is used for evolution
    */
    void evolve(int rule); 

    map<std::string,Connection> connectionList;  //list of connections
    list<int> weightList;                        //list of weights
    bool stateChanged;
    bool connectionChanged;
    int state;                                   //state of the unit; it can assume either value 0 or  value 1
};

#endif //_UNIT_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
