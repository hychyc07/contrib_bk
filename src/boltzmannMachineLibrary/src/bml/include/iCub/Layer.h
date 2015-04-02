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
 * @file Layer.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _LAYER_H_
#define _LAYER_H_


#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>
#include <iCub/Element.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>


#include <iostream>
#include <fstream>
#include <list>
#include <map>
#include <string>

/**
* Class that represent a layer of units.
* @author Francesco Rea
*/

class Layer : public Element{
private:
    std::string name;               // name of the layer
    static const int BINARY_THR=1;  // evolution rule 1
    static const int PROBAB_THR=2;  // evolution rule 2
    static const int BOTH_RULES=3;  // evolution rule 3
    map<std::string,Unit>::iterator iterEvolve; //iterator for the evolution process
    int row;                        // number of rows
    int col;                        // number of columns
    int rowWeights;                 // number of rows in the weight matrix
    int colWeights;                 // number of columns in the weight matrix
    
    yarp::sig::Vector* hidBiases;   // biases of the next layer
    

public:
    
    yarp::sig::Vector* stateVector; //vector storing the state of every single unit in the layer

    /**
    *default constructor
    */
    Layer();//

    /**
    * default destructor
    */
    ~Layer();//

    /**
    * default constructor of a layer composed of nRows of nUnits
    * @param name name of the layer e.g.: L1
    * @param nRows number of rows present in this layer
    * @param nUnits number of units present in each row
    */
    Layer(std::string name,int nRows,int nUnits);//

    /**
    * default constructor of the layer, with no information about the size
    * @param name name of the layer
    */ 
    Layer(std::string name);

    /**
    * function that returns the reference to the weights of connection between this layer and the next
    * @return vishid reference to the connection weights.
    */
    yarp::sig::Matrix* getVisHid() { return vishid; };

    /**
    * function that returns the reference to the biases of connection between this layer and the next
    * @return vishid reference to the connection biases
    */
    yarp::sig::Vector* getVisBias() { return visBiases; };

    /**
    * function that returns the reference to the biases of connection between this layer and the next
    * @return vishid reference to the connection biases
    */
    yarp::sig::Vector* getHidBias() { return hidBiases; };

    /**
    * returns the value of the internal energy
    */
    int getEnergy();

    /**
    * get back data as doubles
    * @return double as data
    */
    double* getData();

    /**
    * returns the number of Rows
    */
    int getRow();

    /**
    * returns the number of Columns
    */
    int getCol();

    /**
    * returns the number of Rows in the weight matrix
    */
    int getRowWeights();

    /**
    * sets the number of Columns in the weight matrix
    * @param value integer that is going to be the new value for colWeights
    */
    void setColWeights(int value) { colWeights = value; };

    /**
    * sets the number of Rows in the weight matrix
    * @param value integer that is going to be the new value for rowWeights
    */
    void setRowWeights(int value) { rowWeights = value; };

    /**
    * returns the number of Columns in the weight matrix
    */
    int getColWeights();

    /**
    * returns the name of this row
    */
    std::string getName();//

    /**
    * returns the number of units present in the layer
    */
    int getNumUnits(); 

    /**
    * add a unit to the unitList
    */
    void addUnit(Unit unitA); //

    /**
    * add a unit to the ConnectionList
    */
    void addConnection(Unit unitA, Unit unitB, double weight); //

    /**
    * add a connection 
    * @param unitName name of the unit
    * @param weight weight of the connection
    */
    void addConnection(std::string unitName, double weight); //

    /**
    * add a connection 
    * @param connection reference to the connection
    */
    void addConnection(Connection connection);

    /**
    * add a row 
    * @param row reference to the row
    */
    void addRow(Row row);

    /**
    * function that creates the connection within the layer
    */
    void interConnectUnits();

    /**
    * set the iterator need for the evolution to the beginning of the listUnit
    */
    void setIterEvolve(); //

    /**
    * code the class into a std::string
    */
    std::string toString(); //

    /**
    * save the layer into IOS::BINARY file
    * @param filename name of the file where the configuration will be saved
    */
    void saveConfiguration(std::string filename); //

    /**
    * load the configuration of a new layer from the binary files
    */
    void loadConfiguration(); //

    /**
    * the states of the units in the Layer are updated based on the choosen rules
    * @param rule reference to the rule used for evolution
    */
    void evolveFreely(int rule); //

    /**
    * The states that are not clamped can evolve
    * @param rule the modality by which the evolution is executed
    */
    void evolveClamped(int rule); 

    map<std::string,Unit> unitList;                 //map of all the units
    map<std::string,Unit> clampedList;              //map of the clamped units
    map<std::string,Connection> connectionList;     //map of all the connections
    map<std::string,Row> rowList;                   //map of all the row of units present in the layer
    yarp::sig::Vector* visBiases;   // biases of this layer
    yarp::sig::Matrix* vishid;      // matrix storing the state of every connection to the higher layer
};

#endif //_LAYER_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
