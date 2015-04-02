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
 * @file bmlEngine.h
 * @brief definition of the module that creates a structure from scratch
 */

#ifndef _BMLEngine_H_
#define _BMLEngine_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//IPP include
//#include <ipp.h>

//bml library includes
#include <iCub/MachineBoltzmann.h>

//within Project Include
#include <iCub/YARPImgRecv.h>
#include <iCub/imageThread.h>
#include <iCub/dataCollectorThread.h>
#include <string>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

// general command vocab's
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SHUT VOCAB4('s','h','u','t')
#define COMMAND_VOCAB_DOWN VOCAB4('d','o','w','n')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_CHILD_COUNT VOCAB2('c','c')
#define COMMAND_VOCAB_WEIGHT VOCAB1('w')
#define COMMAND_VOCAB_CHILD_WEIGHT VOCAB2('c','w')
#define COMMAND_VOCAB_CHILD_WEIGHTS VOCAB3('c','w','s')
#define COMMAND_VOCAB_NAME VOCAB2('s','1')
#define COMMAND_VOCAB_CHILD_NAME VOCAB2('c','n')
#define COMMAND_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define COMMAND_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')

// directional saliency filter vocab's
#define COMMAND_VOCAB_EVOL VOCAB4('e','v','o','l')
#define COMMAND_VOCAB_CLAM VOCAB4('c','l','a','m')
//#define COMMAND_VOCAB_EVOL VOCAB4('e','v','o','l')
#define COMMAND_VOCAB_FREE VOCAB4('f','r','e','e')
#define COMMAND_VOCAB_PERF VOCAB4('p','e','r','f')
#define COMMAND_VOCAB_CONN VOCAB4('c','o','n','n')
#define COMMAND_VOCAB_CLAM VOCAB4('c','l','a','m')


/**
*

\defgroup icub_bmlEngine bmlEngine
@ingroup icub_boltzmannMachineLibrary

This class implements a process able of getting command from a controller 
interpreting them in term of callings to function of the library BM(BOLZMANN MACHINE LIBRARY)
the module reads any command on the port /inCmd whereas the input image for any clamping is read on /inputImage

\section intro_sec Description
This module receives commands as bottle from the bmlInterface GUI. The command respect a communication stardard based
on bottle composed of vocabols


The module does:
- reads commands from the bmlInterface GUI
- produces images representing the state of every allocated layer
- clamp an input into the selected layer

\image html boltzmannMachineLibrary.png

\section lib_sec Libraries
YARP
OPENCV

\section parameters_sec Parameters
--name:defines the name of the module and the rootname of every port
 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
Input ports:
- <name>/cmd:i
- <name>/image:i

Outports
- <name>/cmd:o


\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
bmlEngine --name bmlEngine


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/


class BMLEngine : public RFModule {
private:
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string robotName;                      //name of the robot
    std::string robotPortName;                  //reference to the head of the robot
    std::string handlerPortName;                //name of the handler port (comunication with respond function)

    BufferedPort<ImageOf<PixelMono> > portMono;      // input port for possible coming images
    BufferedPort<ImageOf<PixelRgb> > port;      // input port for possible coming images
    BufferedPort<ImageOf<PixelRgb> > port0;     //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > port1;     //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > port2;     //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > port3;     //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > weight01;  //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > weight12;  //port for writing the image out
    BufferedPort<ImageOf<PixelRgb> > weight23;  //port for writing the image out
    BufferedPort<yarp::os::Bottle> portCmd;
    imageThread* layer0Image;
    imageThread* layer1Image;
    imageThread* layer2Image;
    imageThread* layer3Image;
    imageThread* weights01Image;
    imageThread* weights12Image;
    imageThread* weights23Image;
    BufferedPort<ImageOf<PixelMono> > port_plane; 
    int ct;
    Property options;                               // options of the connection
    MachineBoltzmann *mb;                           // Object that recalls the Boltzmann Machine Library
    dataCollectorThread* dataCollector;             // thread responsable for continuosly saving data from the input port
    int scaleFactorX;                               // scale factor for the output image representing a layer (X axis)
    int scaleFactorY;                               // scale factor for the output image representing a layer (Y axis)
    int currentLayer;                               // sinchronized with the number of layer active
    int count;                                      // counter incremented inside the updateModule
    map<std::string,Layer>::iterator iterE;         // iterator for the elements
    map<std::string,Unit>::iterator iterU;          // interator for units
    map<std::string,Connection>::iterator iterC;    // iterator for the units
    ImageOf<PixelRgb> img_tmp;                      //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> img;                          //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> *img0;                        //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> *img2;                        //=new ImageOf<PixelRgb>;
    bool enableDraw;                                // flag that enable the drawing of the layer present in the simulation
    bool runFreely;                                 // flag that regulates the execution of the freely mode
    bool runClamped;                                // flag that regulates the execution of the clamped mode
    bool inputColour;                               // flag that indicates whether the input is 3channel or mono channel
    yarp::os::Port handlerPort;                     // a port to handle messages 
    int psb;
    int clampingThreshold;                          // value that indicates whether an area can be visually clamped 
    Semaphore mutex;                                // semaphore for the respond function
public:

    /**
    * configure all the module parameters and return true if successful
    */
    bool configure(yarp::os::ResourceFinder &rf);

    /** 
    * open the ports and initialise
    */
    bool open(); //open the port

    /** 
    * try to interrupt any communication or resource usage
    */
    bool interruptModule(); // try to interrupt any communications or resource usage

    /** 
    * close all the ports
    */
    bool close(); //closes all the ports

    /** 
    * active control of the module
    */
    bool updateModule(); //active control of the Module

    /**
    * function that interprets the vocab and respond to the commands
    */
    bool respond(const Bottle &command,Bottle &reply);

    /**
    * uses a bottle to comunicate a command to the module linked on 
    */
    bool outCommandPort();

    /**
    * closes port image
    */
    bool closePortImage();

    /** 
    * function that sends the image of the layers
    */
    void outLayers();

    /**
    * opens port Image
    */
    bool openPortImage();
    /**
    * open the port necessary to send commands
    */
    void openCommandPort();

    /**
    * close the port necessary to send commands
    */
    void closeCommandPort();

    /**
    * function necessary to set the name of the class
    * @param name new name of the class
    */
    void setName(const char *name);

    /** 
    * set the attribute options of class Property
    * @param options series of options as properties which the class will be set with
    */
    void setOptions(Property options); //set the attribute options of class Property

    /** 
    * function that sets the scaleFactorX
    * @param value new value of the scaleFactorX
    */
    void setScaleFactorX(int value);

    /** 
    * function that sets the scaleFactorY
    * @param value new value of the scaleFactorY
    */
    void setScaleFactorY(int value);
    
    /** 
    * function that set the number of the layer active 
    * @param value number of the layer actually active
    */
    void setCurrentLayer(int value);

    /** 
    * function that loads the configuration file with a specific filename 
    * param filename name of the file where the configuration is loaded
    * 
    */
    void loadConfiguration(string filename);

    /** 
    * function that adds a layer to boltzmann machine
    * @param the number of the already istantiated layers
    * @param the number of the columns in the layer
    * @param the number of rows in the layer
    */
    void addLayer(int number,int colDimension, int rowDimension);

    /** 
    * function that adds a sample to dataSet
    */
    void addSample();

    /** 
    * function that creates a thread that adds samples with a constant time
    */
    void startMakeBatches();

    /** 
    * function that stops an already created thread for making batches
    */
    void stopMakeBatches();

    /** 
    * function that clamp a Layer mapping an image onto it
    * param layerNumber reference to the layer name
    */
    void clampLayer(int LayerNumber);

    /** 
    * function that clamp an image as input of a Layer
    * @param layer reference to the layer name
    */
    void clampLayer(Layer* layer);

    /**
    * function that performs evolution within the network
    */
    void evolve1();

    /**
    * function that performs evolution within the network
    */
    void evolve2();
    ImageOf<PixelRgb> *tmp;                                 //input Image which is mapped onto the selected layer
    ImageOf<PixelMono> *tmpMono;                             //input Image which is mapped onto the selected layer
    ImageOf<PixelMono> *ptr_inputImageMono;                  //input Image which is mapped onto the selected layer
    ImageOf<PixelRgb> *ptr_inputImage;                      //input Image which is mapped onto the selected layer
    ImageOf<PixelRgb> *ptr_inputImage2;                     //input Image which is mapped onto the selected layer
    int countLayer;                                         //number of layers already istantiated
    char* name;
    yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;    //Output Port for commands
    string *outCommand;                                     //command that is send throgh the command port
    Bottle bOptions;                                        //bottle containing the option of the command
    bool probFreely_flag;                                   //flag that tells if the probability of the freely mode has to be calculated
    bool probClamped_flag;                                  //flag that tells if the probability of the clamped mode has to be calculated
    bool inputImage_flag;                                   //flag that indicates if the inputImage is ready for clamping
    bool resized;                                           //flag that indicates whether 
};

#endif //_BMLEngine_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
