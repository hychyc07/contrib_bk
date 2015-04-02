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
 * @file Connection.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _CONNECTION_H_
#define _CONNECTION_H_


//#include <iCub/Unit.h>
#include <list>
#include <string>
#include <iostream>

using namespace std;



class Connection{
private:
    double weight;              //weight of the this connection
    std::string unitAName;      //name of A-end of this connection
    std::string unitBName;      //name of B-end of this connection
    std::string name;           //name of the connection
public:
    /**
    * default constructor
    */
    Connection();//default constructor

    /**
    * default destructor
    */
    ~Connection();//default destructor

    /**
    * constructor
    * @param weight
    */
    Connection(double weight);

    /**
    * constructor
    * @param name of the unit A side
    * @param name of the unit B side
    * @param weight
    */
    Connection(std::string unitAName, std::string unitBName, double weight); //constructor

    /**
    * constructor
    * @param name of the unit A side
    * @param weight
    */
    Connection(std::string unitABName, double weight); //constructor

    /**
    * get the weight of the connection
    */
    double getWeight();

    /**
    * get the  the A-end of this connection
    */
    std::string getUnitAName();

    /**
    * get the  the B-end of this connection
    */
    std::string getUnitBName();

    /**
    * get the complete name composition of A-end name and B-end name
    */
    std::string getName();

    /**
    * set the A-end of this connection
    * @param unitAName the name of the unitAName
    */
    void setUnitA(std::string unitAName);

    /**
    * set the B-end of this connection
    * @param unitBName name of the B unit
    */
    void setUnitB(std::string unitBName);

    /**
    * set the Name of the connection
    * @param unitABName name of the connection as whole
    */
    void setName(std::string unitABName);

    /**
    * set the weight of the connection
    * @param weight new weight of the connection
    */
    void setWeight(double weight);

    /**
    * code the class into a std::string
    */
    std::string toString();

};

#endif //_CONNECTION_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------


