// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup icub_module
 * \defgroup icub_episodicMemory episodicMemory
 *
 * The episodicMemory module maintains as associative memory of images: 
 *
 * -  when an image is presented to the memory, it attempts to recall that image; 
 * -  if a previously-stored image matches the presented image sufficiently well, the stored image is _recalled_.
 * -  if no previously-stored image matches sufficiently well, the presented image is stored; 
 * -  in both cases, the module outputs:
 *    -# the recalled or stored image;
 *    -# an image identification number and the match value for the presented image  
 *    -# an image identification number and the match value for whatever image was previously presented 
 *
 * -  Alternatively, when an image identification number is presented to the memory, 
 *    the associated image is _retrieved_.  
 *    If both image and image identification number are presented, the image identification number takes precedence.
 *   
 * The threshold defining whether or not an input image adequately matches a stored imageis provided as a module parameter, 
 * set in the episodicMemory configuration file. The thresold can also be set interactively via the episodicMemory port.
 * 
 * The episodicMemory module has the following inputs: 
 * 
 * - an input image 
 * - an input image vector containing an image id. number for the required image
 * - an action bottle input containing four values: ["rel" azimuth elevation vergence]  
 *   This protocol corresponds to that of the overtAttention module and iKinGazeCntrl module for controlling gaze.
 *   Note that other actions such as reach and grasp have not yet implemented
 *
 * The episodicMemory has the following outputs: 
 * 
 * - recalled/stored image
 * - recalled/stored image vector containing 
 *   -# an image id. number for the matched image
 *   -# a match value r, 0 <= r <= 1, for the matched image
 *   -# the azimuth angle of the gaze at which the image was acquired
 *   -# the elevation angle of the gaze at which the image was acquired
 *   -# the vergence angle of the gaze at which the image was acquired
 * - retrieved image (i.e. the image indexed by the image id. number)
 * - mosaic image (i.e. an image comprising thumbnail versions of all images in the memory)
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * <b>Command Line Parameters </b> 
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c from \c episodicMemory.ini       \n     
 *   specifies the configuration file
 * 
 * - \c context \c episodicMemory/conf   \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c episodicMemory           \n                              
 *   specifies the name of the module (used to form the stem of module port names)
 * 
 * - \c robot \c icub                    \n                         
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters </b> 
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c imageInPort           \c /image:i  \n
 *   specifies the port for input of an image
 *
 * - \c imageIdInPort         \c /imageId:i  \n
 *   specifies the port for input of an image identification number
 *
 * - \c actionInPort          \c /action:i        \n  
 *   The input port name for the gaze angles; the action tag value is not yet implemented
 *
 * - \c recalledImageOutPort \c /recalledImage:o  \n
 *   specifies the image output port for an image recalled associatively by an input image
 *
 * - \c retrievedImageOutPort \c /retrievedImage:o  \n
 *   specifies the image output port for an image retrieved from memory by an image identification number
 *
 * - \c mosaicImageOutPort    \c /mosaicImage:o  \n
 *   specifies the image output port for an image comprising thumbnails of all images in the memory
 *
 * - \c imageIdOutPort        \c /imageId:o  \n
 *   specifies the port for output of the image identification number corresponding to the output image
 *
 * - \c path                  \c ~/iCub/app/episodicMemory  \n
 *   specifies the path to the directory in which the database of images will be stored. THIS MUST BE A VALID PATH.
 *
 * - \c database              \c episodicDatabase.txt  \n
 *   specifies the name of the file in which the index of images will be stored
 * 
 * - \c threshold             \c 0.75  \n
 *   specifies the value defining whether or not an input image matches adequately matches a stored image   
 *
 * - \c limit                 \c 100     \n         
 *   specifies the maximum number of entries (i.e. images) in the memory. 
 *
 * - \c clear               \n         
 *   a flag which, if present, causes specifies the memory to be cleared on start up. If it is not present the memory is initialized from the image database
 * 
 * The database parameter specifies the name of the directory in which the database of images is stored. 
 * This directory must be created before running the module. 
 * 
 * The path parameter specifies the full path to the database directory. 
 * This is where the where the database of image files is stored. 
 *
 * The threshold parameter determines which, if any, of the stored images are recalled:  images which match the input image
 * with a value that equals or exceeds the threshold are recalled and if more than one image exceeds the threshold, 
 * the image with the highest match is recalled.
 * 
 * For example, if the configuration file \c episodicMemory.ini is located in \c C:/iCub/app/episodicMemory/conf 
 * and the database is \c C:/iCub/app/episodicMemory/episodicDatabase.txt then 
 *
 * - \c episodicMemory module must be invoked with \c --context \c episodicMemory/conf 
 * - \c episodicMemory.ini must contain \c "path C:/iCub/app/episodicMemory"
 * - the directory \c C:/iCub/app/episodicMemory/episodicDatabase.txt must exist. 
 *
 *
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b> 
 *
 *  - \c /episodicMemory
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *    \c set \c thr    \c <n>   ... set the threshold for image recall (where \c <n> is a real number in the range 0-1)
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /episodicMemory
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *
 * - \c /episodicMemory/image:i
 * - \c /episodicMemory/imageId:i
 * - \c /episodicMemory/action:i 
 *
 * <b>Output ports</b> 
 *
 *  - \c /episodicMemory
 *  - \c /episodicMemory/recalledImage:o
 *  - \c /episodicMemory/retrievedImage:o
 *  - \c /episodicMemory/mosaicImage:o
 *  - \c /episodicMemory/imageId:o
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 *
 * <b>I/O Port Types & Naming</b> 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> > \c imageInPort;
 * - \c BufferedPort<VectorOf<double> >  \c imageIdInPort;          \c // \c image_id 
 * - \c BufferedPort<Bottle>             \c actionInPort:           \c // \c ["rel" \c azimuth \c elevation \c vergence]  
 * - \c BufferedPort<ImageOf<PixelRgb> > \c recalledImageOutPort;
 * - \c BufferedPort<ImageOf<PixelRgb> > \c retrievedImageOutPort;
 * - \c BufferedPort<ImageOf<PixelRgb> > \c mosaicImageOutPort;
 * - \c BufferedPort<VectorOf<double> >  \c imageIdOutPort;         \c // \c image_id \c match_value \c azimuth \c elevation  \c vergence  
 *
 * Note that the format of the actionInPort is exactly that defined by the /iKinGazeCtrl/head/angles:i port 
 * in the iKinGazeCtrl module.
 *
 * \section in_files_sec Input Data Files
 *
 * \c episodicDatabase.txt  (see above)
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c episodicMemory.ini (see above)
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * \c episodicMemory \c --context \c episodicMemory/conf  \c --from episodicMemory.ini
 *
 * \author David Vernon 
 * (based on code written by Alberto Bietti, Logan Niehaus, and Gionvanni Saponaro for the autoAssociativeMemory module)
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/episodicMemory/include/iCub/episodicMemoryModule.h
**/
  


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Alberto Bietti, Logan Niehaus, Giovanni Saponaro, David Vernon
 * email:   <david.vernon>@robotcub.org
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


#ifndef __ICUB_episodicMemory_MODULE_H__
#define __ICUB_episodicMemory_MODULE_H__

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/*
 * Histogram Matching data holder class. Stores information that will be used by the HistMatch implementation of the episodicMemory. 
 */

class HistMatchData
{
private:
    std::vector<ImageOf<PixelRgb> > imgs; //image database (see episodicMemory specification)
    double threshold;                     //threshold (see episodicMemory specification)
    double matchValue;                    //returned match value (see episodicMemory specification)
    string databaseName;                  //name of the database folder in which memory images will be stored
    string databaseContext;               //database context

public:
    Semaphore thrMutex;                   //threshold semaphore
    Semaphore imgMutex;                   //image semaphore

    /** HistMatchData constructor */  
    HistMatchData();
    /** HistMatchData destructor */
    ~HistMatchData();


    
    vector<ImageOf<PixelRgb> >& images();
    void setThreshold(double);
    void getThreshold(double&);

    void setDatabaseContext(string);
    string getDatabaseContext();
    void setDatabaseName(string);
    string getDatabaseName();

    void loadDatabase();

};


class EpisodicMemoryThread : public Thread
{
private:

   /* class variables */

   bool debug;

   double            gazeAzimuth, gazeElevation, gazeVergence;
   ImageOf<PixelRgb> *imgIn;
   VectorOf<double>  *imgInId;
   Bottle            *actionIn;


   int               imageId;
   double            imageMatch;
   int               matchId;
   std::vector<ImageOf<PixelRgb> >::iterator it;
   ImageOf<PixelRgb> matchImage;
   ImageOf<PixelRgb> mosaicImage;
   bool              found;
   double            matchValue;
   int               numberOfImages;
   int               imageNumber;
   bool              border;
 
   HistMatchData     data;


   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *imageInPort;
   BufferedPort<VectorOf<double> >  *imageIdInPort;
   BufferedPort<Bottle>             *actionInPort;
   BufferedPort<ImageOf<PixelRgb> > *recalledImageOutPort;
   BufferedPort<ImageOf<PixelRgb> > *retrievedImageOutPort;
   BufferedPort<ImageOf<PixelRgb> > *mosaicImageOutPort;
   BufferedPort<VectorOf<double> >  *imageIdOutPort;
   string                           *databaseNameValue;
   string                           *pathValue;
   double                           *thresholdValue;
   int                              *memoryLimitValue;
   bool                             *clearMemoryValue;

public:

   /* class methods */

   EpisodicMemoryThread (BufferedPort<ImageOf<PixelRgb> > *imageIn,
                         BufferedPort<VectorOf<double> >  *imageIdIn,
                         BufferedPort<Bottle>             *actionIn,
                         BufferedPort<ImageOf<PixelRgb> > *recalledImageOut,
                         BufferedPort<ImageOf<PixelRgb> > *retrievedImageOut,
                         BufferedPort<ImageOf<PixelRgb> > *mosaicImageOut,
                         BufferedPort<VectorOf<double> >  *imageIdOut,
                         string                           *databaseName,
                         string                           *path,
                         double                           *threshold,
                         int                              *memoryLimit,
                         bool                             *clearMemory);
   bool threadInit();     
   void threadRelease();
   void run(); 

};


class EpisodicMemory : public RFModule
{
private:
   /* class variables */

   bool debug; 

   /* port names */

   string imageInPortName;
   string imageIdInPortName;
   string actionInputPortName;
   string recalledImageOutPortName;
   string retrievedImageOutPortName;
   string mosaicImageOutPortName;
   string imageIdOutPortName;
   string handlerPortName;
   string moduleName;

   /* parameters */

   string databaseName;
   string path;
   double threshold;
   int    memoryLimit;
   bool   clearMemory;

   // ports

   BufferedPort<ImageOf<PixelRgb> > imageIn;
   BufferedPort<VectorOf<double> >  imageIdIn;
   BufferedPort<Bottle>             actionIn;
   BufferedPort<ImageOf<PixelRgb> > recalledImageOut;
   BufferedPort<ImageOf<PixelRgb> > retrievedImageOut;
   BufferedPort<ImageOf<PixelRgb> > mosaicImageOut;
   BufferedPort<VectorOf<double> >  imageIdOut;
   Port                             handlerPort; 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   EpisodicMemoryThread *episodicMemoryThread;

public:
   virtual bool configure(yarp::os::ResourceFinder &rf);
   virtual bool updateModule();
   virtual bool interruptModule();
   virtual bool close();
   virtual double getPeriod();
   virtual bool respond(const Bottle& command, Bottle& reply);
};

#endif // __ICUB_episodicMemory_MODULE_H__
//empty line to make gcc happy

