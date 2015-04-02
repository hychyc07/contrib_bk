// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file tutorialThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _COLOR_VISION_THREAD_H_
#define _COLOR_VISION_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "segment.h"
#include <math.h>


class colorVisionThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string colorMapPath; 
    std::string modelPath; 
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortRTL;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortRT;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot RGB image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;      // input port to receive RGB image
    yarp::os::BufferedPort<yarp::os::Bottle > dataPort;                              // output port to send detected objects
    std::string name;                                                                // rootname of all the ports opened by this thread

    //add your private stuff here ...

    double a[3][3];
    double det;
    double l1[11];
    double l2[11],bias1[84],weight1[77][2],bias2[3],weight2[2][77],Y1,Y2,Y3,Y4;
	double rw,rx,ImagX,ImagY,ImagZ,FinXpos[2];
	double Sconf[4][3],SconfT[3][4],C[3][3],point[5][4];
	float s[5000];
	int iLocX,iLocY,iLocXL,iLocYL;
    yarp::os::Semaphore runSem;

    double tdGpmp[52];                                                                 // data trasmitted from this module

    real* d_trws;

public:
    /**
    * constructor default
    */
    colorVisionThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    colorVisionThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~colorVisionThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /*
    * function that sets the inputPort name
    */
    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
    void setModelPath(std::string inp) { modelPath = inp; };

    // add your public stuff ...

	// have not added
	//    VisionSystem();
		//~VisionSystem();

	double* Vision(int ObjectType);
	bool ModelLoad();
	int colSegMainR();
	void colSegMainL();
    
    /*
    real trws_potts( // returns residual
                    const unsigned *E, // edges
                    int nE, // nr of edges
                    real w, // edge weights (non-negative)
                    real *q, // array (nK,nT), unary potentials
                    int nT, // nr of pixels
                    int nK, // nr of labels
                    real *f, // array (nK,2,nE), messages
                    real gamma // TRW-S edge occurence probability (for grid graph, set gamma=1/2)
                     );
    */
                     
                     
	void init_onsize(int width_, int height_);
	double Determinant(double **a,int n);
	void CoFactor(double **a,int n,double **b);
	void Transpose(double **a,int n);
	void ConvImg(double UUC1,double UUV1,double UUC2,double UUV2);
	void RBF(double UUC1,double UUV1,double UUC2,double UUV2);
	void initHead(double headzero);

    int *bboxL;
	int width, height; // image dimensions
	int nK; // number of labels
	std::string colormapFile, svmFile;
	unsigned char *colormap; // maps labels to RGB-triplets
	float *W; // segmentation model (SVM weights)
	float smoothness; // smoothness prior
	int niter; // number of iterations of MRF optimizer (TRW-S)
	unsigned minsize; // minimmum size of connected components
	int nE; // number of edges in the image graph
	unsigned *E; // edges of the image graph
	real *q; // MRF unary potentials
	real *f; // TRW-S messages
	unsigned char *K; // image with labels (output of MRF optimizer)
	unsigned *J; // image with labels (output of connected components algorithm)
	//====================================
	//=====================================
	double imageDetails[10][8];
	double imageDetailsL[10][8];
	double ObjectIdentityR[10][8];
	double ObjectIdentityL[10][8];
	int bbvals[100];	
    int countee; 
	int bbvalsL[100];	
    int counteeL; 
	int numobjectsR;
	int numobjectsL;
};

#endif  //_COLOR_VISION_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

