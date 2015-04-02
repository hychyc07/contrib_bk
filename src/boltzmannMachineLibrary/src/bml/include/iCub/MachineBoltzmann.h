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
 * @file MachineBoltzmann.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _MACHINEBOLTZMANN_H_
#define _MACHINEBOLTZMANN_H_


#include <iCub/Connection.h>
#include <iCub/Unit.h>
#include <iCub/Row.h>
#include <iCub/Layer.h>
#include <iCub/Element.h>

#include <stdlib.h>
#include <iostream>
#include <list>
#include <map>
#include <vector>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/matUtil.h>


static int standard_layer_dim_rows=10;
static int standard_layer_dim_units=10;

/**
  \defgroup bml bml
  @ingroup icub_boltzmannMachineLibrary 
   
  Classes for the management of Boltzmann Machines.

  The user can :
  - add connections
  - add layers
  - evolve freely the Boltzmann Machine
  - evolve clamped the Boltzmann Machine
  - save the Boltzmann Machine
  - load the Boltzmann Machine
   
  \note <b>Boltzmann Machine</b>: 
  <li>
  <ul><a href=http://www.cs.toronto.edu/~hinton/absps/pdp7.pdf>first reference</a></ul>
  <ul><a href=http://www.cs.toronto.edu/~hinton/science.pdf>the milestone in the field</a> </ul>
  <ul><a href=http://www.cs.toronto.edu/~hinton/absps/dbm.pdf>the last outstanding work</a></ul>
  </li>

  \image html boltzmannMachineLibrary.png
 
  \section dep_sec Dependencies 
  - none 
 
  
  \defgroup bml bml 
   
  @ingroup bml  
 
  Date: first release 19/02/2010
   
  \author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class MachineBoltzmann{
private:
    double T; //temperature of the machine
    int countElements; //counter of the element present in the boltzmann machine

    static const int BINARY_THR=1;
    static const int PROBAB_THR=2;
    static const int BOTH_RULES=3;
    int sizeVisibleLayer;                           // size of the visible layer
    int meanWeight;                                 // mean value of the weight of connection
    map<std::string,Layer*>::iterator iterEvolve;   // iterator for the evolution of the machine boltzmann
    map<std::string,double> probabilityFreely;      // probability result of the freely evolution
    double epsilon;                                 // scales the difference of probabilities in the learning process
    int maxepoch;                                   // number of epoches for every rbm execution
    int countSample;                                // count the number of sample already saved in the time interval
    double weightcost;                              // cost of a connection
    double epsilonw;                                // learning rate for the weight
    int initialmomentum;                            // momentum at the initial phase of evolution
    int finalmomentum;                              // momentum at the final phase of evolution
    double epsilonhb;                               // lerning rate for the hidden biases
    double epsilonvb;                               // lerning rate for the visible biases
    yarp::sig::Matrix* dataMat;                     // matrix containing the actual state of the sensorial acquisition in time
    std::vector<yarp::sig::Matrix>* dataSet;        // vector of matrices where the whole dataset is stored

    yarp::os::Semaphore mutex;                      // semaphore for handling copy 

public:
    /**
    * default constructor
    */
    MachineBoltzmann();//

    /**
     * default destructor
     */
    ~MachineBoltzmann();//

    /**
    * constructor that creates a Boltzmann machine of n Layers
    * @param nLayers number of layers in the initialization of Machine Boltzmann
    */
    MachineBoltzmann(int nLayers);//

    /**
    *list of Layer in the Boltzmann Machine
    */
    map<std::string,Layer*> elementList; //

    /**
    * Add a Layer to the Boltzmann Machine
    * @param layer reference to the layer to be added
    */
    void addLayer(Layer* layer); //

    /**
    * return a Layer to the Boltzmann Machine
    * @param name name of the layer to be returned
    */
    Layer* getLayer(int name); 

    /**
    * returns the temperature of the Boltzmann Machine
    */
    int getTemperature();

    /**
    * sets the temperature of the Boltzmann Machine
    * @param value new temperature of the Boltzmann Machine
    */
    void setTemperature(int value);

    /**
    * all the connections and units of a layer are trasfered to the Bolzmann Machine lists
    * @param reference to the layer to be migrated
    */
    void migrateLayer(Layer layer); // 

    /**
    * creates the Connections between different layers and save them into the connectionList of Boltzmann Machine
    */
    void interconnectLayers(); //

    /**
    * creates the Connections between 2 precise layers and saves them into the connectionList of the Boltzmann Machine
    * @param layerA sideA of the connection between layers
    * @parame layerB sideB of the connection between layers
    */
    void interconnectLayers(Layer* layerA, Layer* layerB); //

    /**
    *creates the Connections between the layers passed as parameter with the already present layers
    * @param layerNumber the reference to the allocated layer that has to be interconnected
    */
    void interconnectLayer(int layerNumber);

    /**
    * Add a connection to the connectionList
    * @param connection reference to the connection to be added
    */
    void addConnection(Connection connection); //

    /**
    * Add an unit to the unitList
    * @param unit reference to the unit to be added
    */
    void addUnit(Unit unit); //

    /**
    * Add an unit to the _unitList
    * @param unit reference to the unit to be added
    */
    void addUnit2Matrix(Unit unit); //

    /**
    * Add a connection to the _connectionList
    * @param connection reference to the connection to be added
    */
    void addConnection2Matrix(Connection connection);

    /**
    * add a value to the _probFreely
    */
    void addProbFreely2Matrix(double value);

    /**
    * add a value to the _probClamped
    */
    void addProbClamped2Matrix(double value);

    /**
    * save the configuration of the Boltzmann Machine into a FILE
    */
    void saveConfiguration();  //

    /**
    * load the configuration of the Boltzmann Machine from a FILE
    */
    void loadConfiguration(); //

    /**
    * returns the number of elements
    */
    int getCountElements(); //

    /**
    * average probability of two units both being in on-state when the system is clamped
    */
    void setProbabilityClamped(); //

    /**
    * average probability of two units both being in on-state when the netowork is running freely
    */
    void setProbabilityFreely(); 

    /**
    * Function that perform training in the boltzmann machine 
    */
    void training();
    
    /**
    * function that adds inputs as sample in time to the collection of data
    * @param sample sample in the dataset 
    */
    void addSample(Vector sample);

    /**
    * function that reinitialise the data set to a novel matrix with dimension equal to the visible layer size
    * @param size dimension of the single sample of the input
    */
    void reinitData(int size);

    /**
    * function which creates the collection of batches given the input and the time interval
    * it creates a vector of matrices that have dimension equal to input dimensionality
    */
    void makeBatches();

    /**
    * ALL the states of the units in the Boltzmann Machine are updated based on the choosen rules
    * @param rule constant integer indicating the decision rule (,probabil)
    * @param random indicates if the unit to evolve is randomly choosen or is one step of a loop over units
    */
    void evolveFreely(int rule,int random); //

    /**
    * The states that are not clamped can evolve
    * @param rule integer number indicating the decision rule for evolution
    * @param random indicates if the unit to evolve is randomly choosen or is one step of a loop over units
    */
    void evolveClamped(int rule, int random); //

    /**
    *add a reference to the unit of the Boltzmann Machine which is going to be clamped
    * @param position position in relation to the unit to be clamped
    * @param value state of the clamped unit
    */
    void addClampedUnit(int position,int value);   //

    /**
    * set the weights of the connections based on the energy in clamped and freely mode
    */
    void learn(); 

    /**
    * function that trains a layer as it was a restricted boltzmann machine
    * @param data training data
    * @param layer_reference  ref to the hidden layer
    */
    void rbm(double* data,int layer_reference){};

    /**
    * restricted boltzmann machine evolution
    * @param data vector of data that has to have the same dimension of the layer where it is clamped in
    * @param layer pointer to the layer where the data is clamped in
    * @param numhid dimension of the linked layer( hidden layer)
    * @param output matrix which represents the value of the state of higher layer
    * @param output matrix which represents the weights between this layer and the upper one
    * @param output matrix which represents the biases of this layer
    * @param output matrix which represents the biases of the next layer
    */
    void rbm(yarp::sig::Matrix* data,Layer *layer,int numhid,yarp::sig::Matrix* output, yarp::sig::Matrix* output2,yarp::sig::Vector* output3, yarp::sig::Vector* output4);


    /**
    * function that performs the generative part of the network
    * @param layer id number corresponding to the level at which the layer belongs to.
    * @param code code of n layer that is input of the generative process
    * @param generated vector representing the result of the generative process
    */
    void generate(int layer, yarp::sig::Matrix* code, yarp::sig::Matrix* generated);

    map<std::string,Unit> unitList;
    list<Unit> _unitList;                           //list of all the units present in the Boltzmann Machine
    list<int> _clampedList;                         //list of the units that are clamped in the Boltzmann Machine
    list<int>_clampedListValue;                     //value of the clamped unit in the corrispective position
    map<std::string,Connection> connectionList;     //map of all the connections present in the Boltzmann Machine
    list<Connection> _connectionList;               //list of all the connections present in the Boltzmann Machine
    list<double> _probFreely;                       //probabilities of the freely evolution
    list<double> _probClamped;                      //probabilities of the clamped evolution
};
#endif //_MACHINEBOLTZMANN_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------

