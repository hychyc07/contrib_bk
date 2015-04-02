/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __ICUB_STEREO_ALIGN_H__
#define __ICUB_STEREO_ALIGN_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/os/Stamp.h>
#include <cv.h>
#include <highgui.h>

#ifdef USING_GPU
    #include "GL/gl.h"
    #if !defined(SIFTGPU_STATIC) && !defined(SIFTGPU_DLL_RUNTIME)
    // SIFTGPU_STATIC comes from compiler
    #define SIFTGPU_DLL_RUNTIME
    // Load at runtime if the above macro defined
    // comment the macro above to use static linking
    #endif

    #ifdef SIFTGPU_DLL_RUNTIME
       #include <dlfcn.h>
       #define FREE_MYLIB dlclose
       #define GET_MYPROC dlsym
    #endif

    #include "SiftGPU.h"
#endif

struct setup {
    std::string     name;
    bool            shouldSend;
    double          descriptorRadius;
    int             distortionOffset;
};

class WorkerClass : public yarp::os::Thread
{
private:
    /* module parameters */
    std::string                 moduleName;                     //string containing the name of the module
    std::string                 matchPortName;                  //string containing the name of the match port
    std::string                 dataPortName;                   //string containing the name of the data port
    double                      mean;                           //integer which contains the mean of the image displacement
    cv::Mat                     leftMat, rightMat, matMatches;  //images for the left right and matches
    yarp::os::Semaphore         workermutex;                    //mutex for the worker thus avoiding conflicts
    yarp::os::Event             event;                          //event flag for signaling stopping of the module
    yarp::os::Port              matchOutPort;                   //output port Image
    yarp::os::Port              dataOutPort;                    //output port Data containing shift numSifts matched and numSifts matched used

    bool                        shouldSend;                     //boolean flag for sending (or not) the match image
    double                      descriptorRadius;               //double containing value for the descriptor radius
    int                         distortionOffset;               //integere offset for conpensating the uncalibrated cameras

    #ifdef USING_GPU
        SiftMatchGPU                            *matcher;
        SiftGPU                                 *sift;
        std::vector<float >                     descriptors1, descriptors2;
        std::vector<SiftGPU::SiftKeypoint>      keys1, keys2;    
        int num1, num2;
        void                                    *hsiftgpu;
    #endif
public:
    /**
     * constructor
     * @param struct data is passed to the thread in order to initialise all the ports and parameteres correctly
     */
    WorkerClass( setup &dataSetup );
    /**
     * destructor
     */
    ~WorkerClass();
    /**
     * virtual void classes thread functions
     */
    virtual void onStop();
    virtual void threadRelease();
    virtual void run();
    /**
     * update the images for the worker class
     */
    void updateImages(cv::Mat &left, cv::Mat &right);
    /**
     * retreive the mean calculated by the worker class
     */
    bool getMean(double &shift);
    /**
     * function which only conserve the bi-directional matches, only the strongest correspondence will be kept
     */
    void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
                            const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                            std::vector<cv::DMatch>& filteredMatches12, double radius, int knn=1 );
    
    /**
     * update the distortion offset from the rpc port
     */
    void updateDistortion(int dist);
};

/**********************************************************/

class IMAGEThread : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:
    /* module parameters */
    std::string                 moduleName;                             //string containing module name
    std::string                 inLeftPortName;                         //string containing input port name
    std::string                 inRightPortName;                        //string containing input port name
    std::string                 outputPortName;                         //string containing output port name
    cv::Mat                     leftMat, rightMat, matMatches, final;   //cv MAT for images
    yarp::os::Semaphore         mutex;                                  //semaphore for the worker thread
    double                      shift;                                  //integer containing the mean of the worker class
    double                      shiftXaxis;                                  //integer containing the mean of the worker class
    yarp::os::Stamp             st;                                     //stamp to propagate time stamps from original image

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageInPortRight;       //input image port
    yarp::os::Port                                                      imageOutPort;           //output image port

    /* pointer to the working class */
    WorkerClass                 *worker;

public:
    /**
     * constructor
     * @param struct data is passed to the thread in order to initialise all the ports and parameteres correctly
     */
    IMAGEThread(setup &dataSetup);

    /**
     * destructor
     */
    ~IMAGEThread();
    /**
     * virtual void classes thread functions
     */
    virtual bool open();
    virtual void close();
    virtual void interrupt();
    /**
     * callback function using incoming yarp port
     */
    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img);

    /**
     * update the distortion offset from the rpc port
     */
    void updateDistortion(int dist);
    void updateTranslation(int trans);

    void afterStart(bool s)
    {
        if (s)
            std::cout<<"Thread started successfully"<< std::endl;
        else
            std::cout<<"Thread did not start"<< std::endl;
    }
};

/**********************************************************/

class StereoAlign : public yarp::os::RFModule
{

private:
    /* module parameters */
    std::string         handlerPortName;            //string containing the name of the handler port

    yarp::os::Port      handlerPort;                //port to handle messages

    /* pointer to the image thread */
    IMAGEThread         *imageThread;

    setup               dataSetup;                  //struct containing all the port and parameter information 

public:

    bool    configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool    interruptModule();                       // interrupt, e.g., the ports
    bool    close();                                 // close and shut down the module
    bool    respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double  getPeriod();
    bool    updateModule();
};


#endif

//empty line to make gcc happy
