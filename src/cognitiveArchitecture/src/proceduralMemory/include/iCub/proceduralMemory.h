/** 
 * @ingroup icub_module
 *
 * \defgroup icub_proceduralMemory proceduralMemory
 *
 * Learn and recall associations between actions and pairs of image-based perceptions.
 *
 * The procedural memory has three modes of operation, one concerned with learning and two concerned with recall.
 *
 * In the learning mode, the memory learns to associate a temporally-ordered pair of images (perceptions) and
 * the action that led from the first image perception to the second.
 *
 * In recall mode, the memory is presented with just one image perception and an associated perception-action-perception
 * (Pi, Aj, Pk) triple is recalled.  There are two possibilities in the mode:
 *
 * - The image perception presented to the memory represents the first perception in the (Pi, Aj, Pk) triple; 
 *   in this case the recalled triple is a prediction of the next perception and the associated action leading to it.
 *
 * - The image perception presented to the memory represents the second perception in the (Pi, Aj, Pk) triple;
 *   in this case the recalled triple is a reconstruction that recalls a perception and an action that could have led to
 *   the presented perception.
 *
 * In both prediction and reconstruction recall modes, the procedural memory produces as output a (Pi, Aj, Pk) triple, 
 * effectively completing the missing tuple (X, Aj, Pk) or (Pi, Aj, X).
 *
 * To assist with visualization of the state of the procedural memory, the module also produces an output image
 * representing the graph of Pi and Aj nodes.  The size of this image can be specified through module arguments.
 *
 * Image perceptions, P, are respesented as a pair of numbers, the first being the integer index of the image in the 
 * Episodic Memory, and the second being a measure in the range (0,1) of the maximum similarity between 
 * the image presented and images previously stored in the episodic memory.
 *
 * Actions, A, are represented by a quadruplet of numbers, the first three being the relative gaze angles (azimuth, elevation, and vergence)
 * in a body-centred frame of reference and the fourth being an integer tag denoting one of five possible associated actions
 * (reach, push, grasp, locomote, or no action).
 *
 * In summary, the procedural memory either learns the association between(Pi, Aj, Pk) triples or it recalls a triple
 * when presented with just a single perception, completing the triple, with the presented perception being either
 * the first P in the recalled (Pi, Aj, Pk) (prediction) or the second P in the recalled (Pi, Aj, Pk)) (reconstruction).
 *
 * The proceduralMemory module has the following inputs: 
 * 
 * - image vector containing 
 *
 *   -# an image id. number for the matched image
 *   -# a match value r, 0 <= r <= 1, for the matched image
 *   -# the relative azimuth angle of the gaze at which the image was acquired
 *   -# the relative elevation angle of the gaze at which the image was acquired
 *   -# the vergence angle of the gaze at which the image was acquired
 *
 * - mode vector containing a single value denoting the mode of operation
 *
 *   -# learning    (0)
 *   -# prediction  (1) 
 *   -# reconstruction (2) 
 *
 * The proceduralMemory module has the following outputs: 
 * 
 * - recalled image vector containing 
 *
 *   -# an image id. number x of the recalled image
 *   -# a value s, 0 <= s <= 1, indicating the strength of the coupling between the recalled (X, A, P) or(P, A, X) triple.
 *
 *
 *   This is output only in prediction and reconstruction modes. 
 *
 * - recalled action bottle containing four values ["rel" azimuth elevation vergence]
 *
 *   This format of this bottle is exactly that defined by the /angles:i port of the iKinGazeCtrl module,
 *   with the angles being specified relative to the current gaze angles.
 *   This bottle is output only in prediction and reconstruction modes.
 *
 * - graph image showing the memory structure.
 *
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b>
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini ). The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c proceduralMemory.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c proceduralMemory/conf   \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c proceduralMemory \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c proceduralMemory.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c imageIdInPort       \c /imageId:i       \n  
 *   The input port name for the current image id. number and match value
 *
 * - \c modePort            \c /mode:i          \n  
 *   The input port name for the mode (corresponding to learning, prediction, reconstruction) 
 *
 * - \c imageIdOutPort      \c /imageId:o       \n 
 *   The output port name for the current image id. number and match value  
 *
 * - \c actionOutPort       \c /action:o   \n 
 *   The output port name for the gaze angles; the action tag value is not yet implemented
 *
 * - \c graphPort           \c /graph:o   \n 
 *   The output port name for the image with a graphical visualization of the memory,
 *
 * - \c graphWidth          \c 320   \n 
 *   The horizontal size of the graphical visualization image
 *
 * - \c graphHeight         \c 320   \n 
 *   The vertical size of the graphical visualization image
 *
 * - \c database            \c proceduralDatabase.txt  \n
 *   specifies the file name of the file in which the database of perception-action associations is saved/retrieved.
 *   If it does not exist, it will be created by the module. 
 * 
 * - \c path                \c ~/iCub/app/proceduralMemory  \n
 *   specifies the path the to the database file. This must be a valid path.
 *
 * - \c clear               \n         
 *   a flag which, if present, causes specifies the memory to be cleared on start up. If it is not present the memory is initialized from the database
 * 
 *
 * \section portsa_sec Ports Accessed
 * 
 * - none
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /proceduralMemory \n
 *   This port is used to change the parameters of the module at run time or stop the module
 *   The following commands are available
 * 
 *   \c help \n
 *   \c quit
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /rectification
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /proceduralMemory/imageId:i 
 * - \c /proceduralMemory/mode:i
 *
 *
 * <b>Output ports</b>
 *
 * - \c /proceduralMemory 
 * - \c /proceduralMemory/imageId:o 
 * - \c /proceduralMemory/action:o 
 * - \c /proceduralMemory/graph:o
 *
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - \c BufferedPort<VectorOf<double> >  \c imageIdInPort;          \c // \c image_id, \c match_value, \c azimuth, \c elevation, \c vergence 
 * - \c BufferedPort<VectorOf<double> >  \c modePort;               \c // \c mode  
 * - \c BufferedPort<VectorOf<double> >  \c imageIdOutPort;         \c // \c image_id 
 * - \c BufferedPort<VectorOf<double> >  \c actionOutPort;          \c // \c azimuth,  \c elevation \c 'r' \c 0 \c 0
 * - \c BufferedPort<ImageOf<PixelRgb> > \c graphPort;
 *
 * Note that the protocol used for the actionOutPort is the same as that used by 
 * the overtAttention module when controlling the iKinGazeCtrl  module using the /angles:i port.
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c proceduralMemory.ini  in \c $ICUB_ROOT/app/proceduralMemory/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>proceduralMemory --name proceduralMemory --context proceduralMemory/conf --from proceduralMemory.ini</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/rectification/include/iCub/proceduralMemory.h
 * 
 */


/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
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



#ifndef __ICUB_PROCEDURALMEMORY_MODULE_H__
#define __ICUB_PROCEDURALMEMORY_MODULE_H__


/* System includes */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <string>


/* YARP includes */

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::dev;


/* fourierVision includes */

#include "iCub/fourierVision.h"


/* proceduralMemory definitions */

#define LEARNING_MODE                   0
#define PREDICTION_MODE                 1
#define RECONSTRUCTION_MODE             2


#define MAX_NUMBER_OF_NODES             100 
#define CARTESIAN                       0
#define POLAR                           1
#define ACTION_NODE                     0
#define PERCEPTION_NODE                 1

#define NODE_RADIUS                     10  
#define ARROW_HEAD_SIZE (8.0)
#define ARROW_HEAD_ANGLE (3.14159 / 5.0)

typedef struct {
    int    nodeType;
    int    imageId;
    int    saccadeFrameOfReference;
    int    saccadeCoordinate1;   // we keep the saccade values as integers to allow comparison
    int    saccadeCoordinate2;   // need to make sure that the loss of accuracy doesn't impact negatively
    int    layoutX;
    int    layoutY;
} proceduralMemoryModeType;


class ProceduralMemoryGraph
{
private:
    bool debug;
    
    string databaseName;                  // name of the database folder in which memory data will be stored
    string databaseContext;               // database context
    int    saccadeFrameOfReference;       // either CARTESIAN or POLAR

    int numberOfNodes;
    proceduralMemoryModeType nodeList[MAX_NUMBER_OF_NODES];   // used to store information about each node

    int adjacencyMatrix[MAX_NUMBER_OF_NODES][MAX_NUMBER_OF_NODES];                         // used to store graph structure
    int associationMatrix[MAX_NUMBER_OF_NODES][MAX_NUMBER_OF_NODES][MAX_NUMBER_OF_NODES];  // used to store associations between perception-action-perception triples

    bool processed[MAX_NUMBER_OF_NODES];   // used for traversal
    bool discovered[MAX_NUMBER_OF_NODES];  // ibid
    int  parent[MAX_NUMBER_OF_NODES];      // ibid
 
public:
    ProceduralMemoryGraph();
    ~ProceduralMemoryGraph();

    void   setDatabaseContext(string);
    string getDatabaseContext();
    void   setDatabaseName(string);
    string getDatabaseName();
    void   loadDatabase();
    void   saveDatabase();
    void   updateGraph(int previousImageId, int saccadeCoordinate1, int saccadeCoordinate2, int nextImageId);
    int    getPerceptionNode(int previousImageId);
    int    getActionNode(int saccadeFrameOfReference, int saccadeCoordinate1, int saccadeCoordinate2);
    void   generateGraphImage(ImageOf<PixelRgb> &graphImage);
    void   layoutDepthFirstTraveral(int nodeNumber);
    void   predict(int node1, int & node2, int & node3, double & coupling);
    void   reconstruct(int & node1, int & node2, int node3, double & coupling);
    double coupling(int node1, int ndoe2, int node3);
    void   lineCircleIntersection(int x1, int y1, int x2, int y2, int cx, int cy, int r, int & xa, int & ya, int & xb, int & yb);
    void   shortestSegment(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, int & xa, int & ya, int & xb, int & yb);
    void   drawEdge(ImageOf<PixelRgb> &graphImage, PixelRgb rgbPixel, int x1, int y1, int x2, int y2, int r);
    void   layoutActionNodes(int nodeNumber);

    // DV Also need to write variants of predict() and reconstruct() with two known arguments instead of one

};


class ProceduralMemoryThread : public Thread
{
private:

   /* class variables */

   int                   x, y;
   int                   width, height, depth;    // dimensions of the image
   PixelRgb              rgbPixel;
   ImageOf<PixelRgb>     *graphImage;

   VectorOf<double>      *imageIdIn;
   VectorOf<double>      *modeIn;

   int                   imageIdCurrent;
   double                imageSimilarityCurrent;
   double                azimuth;
   double                elevation;
   double                vergence;
   int                   imageIdPrevious;
   double                imageSimilarityPrevious;
   double                coupling;
   int                   mode;
   double                temp1, temp2;
   int                   temp3;

   ProceduralMemoryGraph graph;

   bool debug;
  	    
   /* thread parameters: they are pointers so that they refer to the original variables in rectification */

   BufferedPort<VectorOf<double> >   *imageIdInPort;
   BufferedPort<VectorOf<double> >   *modeInPort;
   BufferedPort<VectorOf<double> >   *imageIdOutPort;
   BufferedPort<Bottle>              *actionOutPort;   
   BufferedPort<ImageOf<PixelRgb> >  *graphOutPort;
   int                               *graphWidthValue;
   int                               *graphHeightValue;
   string                            *databaseNameValue;
   string                            *pathValue;
   bool                              *clearMemoryValue;

public:

   /* class methods */

   ProceduralMemoryThread(BufferedPort<VectorOf<double> >  *imgIdIn,
                          BufferedPort<VectorOf<double> >  *modeIn,
                          BufferedPort<VectorOf<double> >  *imgIdOut,
                          BufferedPort<Bottle>             *actionOutPort,  
                          BufferedPort<ImageOf<PixelRgb> > *graphOut,
                          int                              *graphWidth,
                          int                              *graphHeight,
                          string                           *databaseName,
                          string                           *path,
                          bool                             *clearMemory
                         );
   bool threadInit();     
   void threadRelease();
   void run (); 
};


class ProceduralMemory:public RFModule
{
   /* port names */

   string moduleName;
   string imageIdInputPortName;
   string modeInputPortName;
   string imageIdOutputPortName;  
   string actionOutputPortName;
   string graphOutputPortName;
   string handlerPortName;

   /* class variables */

   bool debug;

   /* parameters */

   string databaseName;
   string path;
   int graphWidth;
   int graphHeight;
   bool clearMemory;

   /* ports */

   BufferedPort<VectorOf<double> >   imageIdIn;
   BufferedPort<VectorOf<double> >   modeIn;
   BufferedPort<VectorOf<double> >   imageIdOut;
   BufferedPort<Bottle>              actionOut;   
   BufferedPort<ImageOf<PixelRgb> >  graphOut;
   Port                              handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   ProceduralMemoryThread *proceduralMemoryThread;


public:
   ProceduralMemory();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_PROCEDURALMEMORY_MODULE_H__
//empty line to make gcc happy

