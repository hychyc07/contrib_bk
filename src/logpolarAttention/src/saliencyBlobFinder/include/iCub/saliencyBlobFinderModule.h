// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file saliencyBlobFinderModule.h
 * @brief module class definition for the blob finder module.
 */

#ifndef _SALIENCYBLOBFINDERMODULE_H_
#define _SALIENCYBLOBFINDERMODULE_H_

/** 
 *
 * \defgroup icub_saliencyBlobFinder saliencyBlobFinder
 * @ingroup icub_logpolarAttention
 *
 * This is a a blob finder module that works on logpolar images. It takes an edge image as input (e.g. as produced by the
 * visualFilter module), runs the watershed operator (to find blobs), quantizes the colors and then computes a measure
 * of saliency for each blob. The output can be used and combined with other image based measures of saliency.
 *
 *  
 * \image html visualAttentionBUTD.jpg 
 * 
 * More in details, this module extracts visual blobs in the scene and gives a saliency value to any of them.
 * The process of assigning a saliency value is based on a top-down algorithm and bottom-up algorithm.(see <a href="http://">here</a>).
 * The bottom-up gives high saliency to isolated blobs whereas the top-down gives high saliency to blobs which have mean colour close to 
 * the target colour. The module receives an edge image and the respective red plane, green plane and yellow plane. It needs the opponency 
 * maps composed as R+G-, G+R-, B+Y- as well. These are maps made by the difference of gaussians appied on the three colour planes and
 * represent colour sensitive photoreceptors of the human visual system. Subsequently, the module applies a watershed method to 
 * segment blobs from the image and calculates a saliency map based on the mean color of the blob, its dimension and the the color distance 
 * to the target object (in case of top-down attention).
 *
 * \section intro_sec Description
 * Speicfically, the module:
 * - stream the mean color image of all the blobs\n
 * - stream the image of the fovea blob\n
 * - stream the saliency map as a gray scale image\n
 * - stream the most salient blob\n
 * 
 * \image html saliencyBlobFinder.png
 *
 * \section lib_sec Libraries
 *
 * YARP\n
 * IPP
 *
 * \section parameters_sec Parameters
 * Here is a  comprehensive list of the parameters you can pass to the module.\n
 * --name (string) name of the module. The name of all the ports will be istantiated starting from this name\n
 * --from (string) name of the configuration file.\n
 * --context (string) name of the application\n
 * 
 * \section portsa_sec Ports Accessed
 * - /blobFinderInterface/command:o : once manually connected to the <name>/cmd the graphical interface is able to control this module (for further information press Help Button in the interface)\n
 * - /visualFilter/rg:o :  colour opponency map of the input image (difference of gaussian)\n
 * - /visualFilter/gr:o : colour opponency map of the input image (difference of gaussian)\n
 * - /visualFilter/by:o : colour opponency map of the input image (difference of gaussian)\n
 * - /visualFilter/edges:o : edges extracted from the input image\n
 *
 * \section portsc_sec Ports Created
 *
 * Output ports:
 * - <name>/imageC3:o: image output (color)
 * - <name>/saliencyMap:o : saliency map of the blobs
 * - <name>/rg:o : color opponency (debug)
 * - <name>/gr:o : color opponency (debug)
 * - <name>/by:o : color opponency (debug)
 * - <name>/foveaBlob:o : fovea blob
 * - <name>/foveaRgbBlob:o : fovea blob (color)
 *
 * Input ports:
 * - <name>/image:i: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>\n
 * - <name>/rg:i: acquires the input stream of the R+G- opponency map\n
 * - <name>/gr:i: acquires the input stream of the G+R- opponency map\n
 * - <name>/by:i: acquires the input stream of th B+Y- opponency map\n
 * - <name>/by:i: edges for watershed \n
 * 
 * InOut ports:
 * - <name> : port for the input rpc commands (for further command send Help command)
 * 
 * This module is able to respond to the following set of commands:
 * - set mea: select the output which represents all the blobs in mean colour\n
 * - set tag: select the output which represents all the blobs\n
 * - set max: select the output which represents the most salient blob\n
 * - set clp: select the output which represents all the blobs intensified by the saliency\n
 * - set fov: select the output which represents the fovea blob\n
 *
 * - set kbu: weight of the bottom-up algorithm\n
 * - set ktd: weight of top-down algorithm\n
 * - set rin: red intensity value\n
 * - set gin: green intensity value\n
 * - set bin: blue intensity value\n
 * - set Mdb: Maximum dimension of the blob analysed\n
 * - set mdb: minimum dimension of the blob analysed\n
 * - set tco: set the timeconstant in second for output format (x y z) 3d space\n
 * - set tce: set the time constant in second for output forma (img x y) imageplane\n
 * - set mBA: set minimum bounding area\n
 * - set pAR: set percentage of the blob dimension considered surrounding area\n
 * 
 * - rset flt : reset the filter\n
 *
 * - get kbu : weight of the bottom-up algorithm\n
 * - get ktd : weight of top-down algorithm\n
 * - get rin : red intensity value\n
 * - get gin : green intensity value\n
 * - get bin : blue intensity value\n
 * - get Mdb : Maximum dimension of the blob analysed \n
 * - get mdb : minimum dimension of the blob analysed \n
 * - get frgb: returns the color of the blob in fovea \n
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
 * \c saliencyBlobFinderLeft.ini in \c $ICUB_ROOT/app/logpolarAttention/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * <tt>saliencyBlobFinder --from saliencyBlobFinder.ini --context attentionMechanism/conf --name /blobFinder/icub/left_cam</tt>
 *
 * \author Francesco Rea, Giorgio Metta, based on existing code by Francesco Orabona
 * 
 * Copyright (C) 2009 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logpolarAttention/src/saliencyBlobFinder/include/iCub/saliencyBlobFinderModule.h
 * 
 */

/** /section change_log CHANGE_LOG
 * 21/12/10 : added the formulas and kinematic constraints necessary to work out the 3D point where the protoobject is seen  author:Rea \n
 * 05/01/11 : kinamatic calculus for 3d position of the blob in the virtual word                                             author:Rea \n
 * 12/01/11 : added lifeTimer property for correct forgetness of the object                                                  author:Rea \n
 * 02/02/11 : introduced configFile for camera setting in the thread for object localization                                 author:Rea \n
 * 05/02/11 : filtered blob with a nbivariate normal                                                                         author:Rea \n
 * 21/03/11 : added list of the attended locations in the 3D space                                                           author:Rea \n
 * 22/03/11 : added the correct procedure to represent the blob in the GUI                                                   author:Rea \n
 * 23/03/11 : procedure to correctly populate the list of attended locations                                                 author:Rea \n     
 * 03/01/12 : adding top-down parameter search                                                                               author:Rea \n  
 * 15/02/12 : added respond function for command fovea RGB                                                                   author:Rea \n
 * 18/02/12 : added commands to get fovea blob color (red, blue, green)                                                      author:Rea \n         
 * 05/12/12 : removed all the references to the motor control                                                                author : Rea \n
 */

#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/blobFinderThread.h>

// general command vocab's
#define COMMAND_VOCAB_IS     VOCAB2('i','s')
#define COMMAND_VOCAB_OK     VOCAB2('o','k')

#define COMMAND_VOCAB_HELP   VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_TRED   VOCAB4('t','r','e','d')
#define COMMAND_VOCAB_TGRE   VOCAB4('t','g','r','e')
#define COMMAND_VOCAB_TBLU   VOCAB4('t','b','l','u')
#define COMMAND_VOCAB_FRED   VOCAB4('f','r','e','d')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FBLU   VOCAB4('f','b','l','u')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FGRE   VOCAB4('f','g','r','e')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FRGB   VOCAB4('f','r','g','b')       // request of fovea blob color (rgb)


#define COMMAND_VOCAB_MAXDB  VOCAB3('M','d','b')           // maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB  VOCAB3('m','d','b')           // minimum dimension of the blob drawn
#define COMMAND_VOCAB_MBA    VOCAB3('m','B','A')           // minimum dimension of the bounding area
#define COMMAND_VOCAB_SET    VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET    VOCAB3('g','e','t')
#define COMMAND_VOCAB_WTD    VOCAB3('w','t','d')
#define COMMAND_VOCAB_WBU    VOCAB3('w','b','u')

/*
 * @brief: module interfacing with the user
 */
class saliencyBlobFinderModule : public yarp::os::RFModule{
private:
    Port cmdPort;               // port necessary for rpc commands
    
    int threadRate;             // rate of the processor Thread
    int minBoundingArea;        // minimum bounding area around the blob for neighbourhood definition
    //IppiSize srcsize;           // ipp reference to the size of the input image
    
    int width;                  // width of the input image
    int height;                 // height of the input image
    
    Semaphore mutex;            // semaphore for the respond function (UNUSED?)
    blobFinderThread* blobFinder; //main thread responsable to process the input images and produce an output
    Bottle* command;            // bottle where the command received is saved ready for the respond
    Bottle* reply;              // bottle where the reply will be stored for further purpose
    std::string configFile;     // file representing characteristic of the camera
    std::string camerasFile;    // file representing characteristic of the camera
    std::string moduleName;     // name of the module read from configuration 

public:
    /**
    * default constructor
    */
    saliencyBlobFinderModule();
    
    /**
    * destructor
    */
    ~saliencyBlobFinderModule(){};
    
    /**
     * open the port and intialise the module
     * @param config configuration of the module
     */
    bool open(Searchable& config); 
    
    /**
     * tries to interrupt any communications or resource usage
     */
    bool interruptModule(); // 
    
    /**
     * function for initialization and configuration of the RFModule
     * @param rf resourceFinder reference
     * @return true iff ???
     */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    
    /**
     * catches all the commands that have to be executed when the module is closed
     * @return ...
     */
    bool close(); 
    
    /**
     * respond to command coming from the command port
     * @param command command received by the module
     * @param reply answer that this module gives to the command (if any)
     * @return
     */
    bool respond(const Bottle &command, Bottle &reply);
    
    /**
     * updates the module
     * return true iff...
     */
    bool updateModule();
};

#endif //__SALIENCYBLOBFINDERMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
