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
 * @file Connection.cpp
 * @brief definition of the module that creates a structure from scratch
 */

#include <iCub/Connection.h>
#include <cstdio>
#define MEAN_WEIGHT 1 //-->check the variable in layer.cpp


/**
*default constructor
*/
Connection::Connection(){
    //double value=double(rand())/RAND_MAX;
    double value=0.1;
    double weight_rnd=((value-0.5)*MEAN_WEIGHT);
    this->weight=weight_rnd;
}
/**
*default destructor
*/
Connection::~Connection(){
}

/**
*constructor 
*/
Connection::Connection(double weight){
    this->weight=weight;
    this->name="";
}

/** 
*constructor that takes the name of the two units and combine them toghether.
* Finally the weight is set.
*/
Connection::Connection(std::string unitAName, std::string unitBName,double weight){
    this->unitAName=unitAName;
    this->unitBName=unitBName;
    this->name=unitAName+unitBName;
    this->weight=weight;
}

/** 
* constructor that takes the name of the connection and
* finally set the weight
*/
Connection::Connection(std::string unitABName, double weight){
    this->name=unitABName;
    this->weight=weight;
}

/** 
*get the weight of this connection as double
*/
double Connection::getWeight(){
    return this->weight;
}

/**
* set the weight of the connection
* @param w new weight of the connection
*/
void Connection::setWeight(double w){
    this->weight=w;
}

/**
* get the reference to the unitA of this connection
*/
std::string Connection::getUnitAName(){
    return unitAName;
}

/** 
*get the reference to the unitB of this connection
*/
std::string Connection::getUnitBName(){
    return unitBName;
}

/**
*get the complete name composition of A-end name and B-end name
*/
std::string Connection::getName(){
    if(name!="")
        return name;
    else
        return unitAName+unitBName;
}

/**
*set the A-end of this connection
*/
void Connection::setUnitA(std::string unitAName){
    this->unitAName=unitAName;
}
/**
*set the B-end of this connection
*/
void Connection::setUnitB(std::string unitBName){
    this->unitBName=unitBName;
}

/**
*set the name of this connection
*/
void Connection::setName(std::string name){
    this->name=name;
}

/**
*code the class into a std::string
*/
std::string Connection::toString(){
    //_____
    char number[3];
    int n=sprintf(number,"%f",this->weight);
    std::string w_str(number,n);
    //_____
    if(this->getName()!="")
        return this->getName()+"("+w_str+")";	
    
    else
        return this->getUnitAName()+this->getUnitBName()+"("+w_str+")";
}
