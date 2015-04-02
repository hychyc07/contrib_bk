// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file mosaicThread.cpp
 * @brief Implementation of the mosaic thread (see mosaicThread.h).
 */

#include <iCub/mosaicThread.h>
#include <yarp/math/SVD.h>
#include <cv.h>
#include <highgui.h>
#include <cstring>

#define MAXMEMORY 100
#define LEFT_EYE 0
#define RIGHT_EYE 1
#define THRATE 100
#define RATIOX 3
#define BIASX 0

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace yarp::math;
using namespace iCub::iKin;

/************************************************************************/
bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy")) {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble(); printf("fx %f \n", fx);
                double fy = parType.find("fy").asDouble(); printf("fy %f \n", fy);

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }
    return false;
}

/**************************************************************************/
mosaicThread::mosaicThread(): RateThread(THRATE) {
    inputImageLeft = new ImageOf<PixelRgb>;
    inputImageRight = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    warpImLeft = new ImageOf<PixelRgb>;
    warpImRight = new ImageOf<PixelRgb>;
    
    
    robot = "icub"; 
    resized = false;
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
    countMemory = 0;
    azimuth = 0.0;
    elevation = 0.0;
    rectified = false;
    forgettingFactor = false;
}

mosaicThread::mosaicThread(string _robot, string _configFile): RateThread(THRATE) {
    //initialisation of variables
    countMemory = 0;
    elevation = 0.0;
    azimuth = 0.0;
    robot = _robot;
    configFile = _configFile;
    resized = false;
    rectified = false;
    forgettingFactor = false;
    //allocating memory
    outputImageMosaic = new ImageOf<PixelRgb>;
    
    inputImageLeft    = new ImageOf<PixelRgb>;
    inputImageRight   = new ImageOf<PixelRgb>;
    warpImLeft        = new ImageOf<PixelRgb>;
    warpImRight       = new ImageOf<PixelRgb>;

    inputMonoLeft     = new ImageOf<PixelMono>;
    inputMonoRight    = new ImageOf<PixelMono>;
    warpMnLeft        = new ImageOf<PixelMono>;
    warpMnRight       = new ImageOf<PixelMono>;
 
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
}

mosaicThread::~mosaicThread() {
    // freeing memory
    delete outputImageMosaic;
    delete inputImageLeft;
    delete inputImageRight;
    delete warpImLeft;
    delete warpImRight;
    delete inputMonoLeft;
    delete inputMonoRight;
    delete warpMnLeft;
    delete warpMnRight;
    free(memory);   
}

bool mosaicThread::threadInit() {
    /* open ports */ 
    if (!imagePortInLeft.open(getName("/left:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!imagePortInRight.open(getName("/right:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imageMonoInLeft.open(getName("/leftMono:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!imageMonoInRight.open(getName("/rightMono:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!portionPort.open(getName("/portion:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 

    printf("\n robotname: %s \n",robot.c_str());

    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());
    
    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;
    
    
    //initialising the head polydriver
    printf("starting the polydrive for the head.... of the robot %s \n", robot.c_str());
    Property optHead("(device remote_controlboard)");
    string remoteHeadName="/" + robot + "/head";
    string localHeadName = name + "/head";
    optHead.put("remote",remoteHeadName.c_str());
    optHead.put("local",localHeadName.c_str());
    drvHead =new PolyDriver(optHead);
    if (!drvHead->isValid()) {
        fprintf(stdout,"Head device driver not available!\n");        
        delete drvHead;
        return false;
    }
    drvHead->view(encHead);

    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    
    optPolyTorso.put("remote",("/" + robot + "/torso").c_str());
    optPolyTorso.put("local",(name + "/torso/position").c_str());
    
    printf("creating a new polydriver \n");
    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
        printf("polydriver of the torso did not start correctly \n");
        return false;
    }
    polyTorso->view(encTorso);

    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);

    printf("trying to CAMERA projection from %s.......... ", configFile.c_str());
    // get left camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        printf("SUCCESS in finding configuration of camera param \n");
        Matrix &Prj=*PrjL;
        cxl = Prj(0,2);
        cyl = Prj(1,2);
        fxl = Prj(0,0);
        fyl = Prj(1,1);
        printf("configuration param of the left camera %f %f %f %f \n",cxl,cyl,fxl,fyl);          
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else { 
        printf("did not find the configuration file for the camera \n");
        return false; //PrjL=invPrjL=NULL;
    }

    // get right camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR)) {
        printf("SUCCESS in finding configuration of camera param \n");
        Matrix &Prj=*PrjR;
        cxr = Prj(0,2);
        cyr = Prj(1,2);
        fxr = Prj(0,0);
        fyr = Prj(1,1);
        printf("configuration param of the right camera %f %f %f %f \n",cxr,cyr,fxr,fyr);                
        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else { 
        printf("did not find the configuration file for the camera \n");
        return false; 
    }

    //initilization of the backprojection to the cyclopic plane
    Vector q(8);
    double ratio = M_PI /180;
    q[0]=0 * ratio;
    q[1]=0 * ratio;
    q[2]=0 * ratio;
    q[3]=0  * ratio;
    q[4]=0  * ratio;
    q[5]=0  * ratio;
    q[6]=0  * ratio;
    q[7]=0  * ratio;
    
    eyeH0 = new Matrix(4,4);
    eyeCyclopic = new iCubEye(*eyeL);
    iKinChain* chainCyclopic  = eyeCyclopic->asChain();
    iKinLink* link = &(chainCyclopic-> operator ()(6));
    link->setD(0.0);
    double dleft = link->getD();
    *eyeH0 = eyeCyclopic->getH(q);  
    //*eyeH0 = eyeL->getH(q);
    Matrix& eyeH_ref = *eyeH0;
    
    inveyeH0   = new Matrix(pinv(eyeH_ref.transposed()).transposed());
    startTimer = Time::now();
    printf("initilisation successfully ended \n");
    return true;
}


void mosaicThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string mosaicThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void mosaicThread::setMosaicDim(int w, int h) {
    printf("setting mosaic dimension \n");
    //mosaic image default size
    width = w;
    height = h;
    //default position of input image's center
    xcoord = floor((double)height / 2.0);
    ycoord = floor((double)width  / 2.0);   
    
    memoryLocation = new int[w*h];
    outputImageMosaic->resize(width,height);
    outputImageMosaic->zero();
}

void mosaicThread::resize(int width_orig,int height_orig) {        
    this->width_orig = width_orig;
    this->height_orig = height_orig;

    printf("resizing using %d %d \n",width_orig,height_orig);
    inputImageLeft->resize(width_orig,height_orig);
    inputImageRight->resize(width_orig,height_orig);
    warpImLeft->resize(width_orig,height_orig);
    warpImRight->resize(width_orig,height_orig);
   
    printf("successfully resized \n");
}

void mosaicThread::resizeMono(int width_orig,int height_orig) {        
    this->width_orig = width_orig;
    this->height_orig = height_orig;
    
    printf("resizing mono images \n");
    inputMonoLeft->resize(width_orig,height_orig);
    inputMonoRight->resize(width_orig,height_orig);
    warpMnLeft->resize(width_orig,height_orig);
    warpMnRight->resize(width_orig,height_orig);
    
    printf("successfully resized \n");
}


bool mosaicThread::placeInpImage(int X, int Y) {
    if(X > width || X < 0 || Y > height || Y < 0) return false;
    xcoord = X;
    ycoord = Y;
    return true;
}

void mosaicThread::setInputDim(int w, int h) {
    //input image default size
    width_orig = w;
    height_orig = h;
}

void mosaicThread::plotObject(float x,float y,float z) {
    printf("saving in memory the 3dlocation \n");
    float* pointer = memory ;
    printf("countMemory %d \n", countMemory);
    pointer += countMemory * 3;
    *pointer = x; pointer++;
    *pointer = y; pointer++;
    *pointer = z;
    printf("saved the position %f,%f,%f \n",x,y,z);
    countMemory++;
}


void mosaicThread::setFetchPortion(float a, float e) {
    elevation = e;
    azimuth   = a;
}

void mosaicThread::fetchPortion(ImageOf<PixelRgb> *image) {
    //printf("trying to fetch a particular position on the mosaic %f %f \n", azimuth, elevation);
    //determine the shift based on the focal length
    int mosaicX, mosaicY;
    int shiftx =(int) floor( fxl   * 1.0 * tan(((azimuth + BIASX)  * 3.14) / 180));
    int shifty =(int) floor( -fyl  * 1.0 * tan ((elevation * 3.14) / 180));
    
    unsigned char* mosaic = outputImageMosaic->getRawImage();
    unsigned char* portion =  image->getRawImage();
    int mosaicRowSize = outputImageMosaic->getRowSize();
    int mosaicPadding = outputImageMosaic->getPadding();
    int portionPadding = image->getPadding();

    //printf("fetching focal lenght  %f %f \n", fxl, fyl);
    ycoord = 320 + shiftx ;
    xcoord = 240 + shifty;
    mosaicX = ycoord - 160 ;
    //mosaicX -= floor(height_orig / 2);
    mosaicY = xcoord - 120;
    //mosaicY -= floor(width_orig / 2);
    
    
    mosaic += mosaicX * 3 +  mosaicY * mosaicRowSize; // + (width>>1) * 3 + (height>>1) * mosaicRowSize;
    //printf("height_orig width_orig %d %d mosaicRowSize %d \n", height_orig, width_orig,mosaicRowSize );
    for ( int row = 0 ; row < height_orig; row++) { 
        for (int cols = 0; cols< width_orig; cols++) {
            *portion++ = *mosaic++;  //red
            *portion++ = *mosaic++;  //green
            *portion++ = *mosaic++;  //blue
        }
        portion += portionPadding;
        mosaic  += mosaicRowSize - width_orig * 3;
    }
   
}

bool mosaicThread::setMosaicSize(int width = DEFAULT_WIDTH, int height = DEFAULT_HEIGHT) {
    if(width > MAX_WIDTH || width < width_orig || width <= 0 
       || height > MAX_HEIGHT || height < height_orig || height <= 0 ) {
        printf("setMosaicSize returned FALSE \n");
        return false;
    }
    else{
        printf("setMosaicSize returned TRUE \n");
    }
    this->width = width;
    this->height = height;

    printf("initialisation of the memoryLocation pointer \n");
    memoryLocation = new int[width * height];
    for (int j=0; j < width*height; j++) {
        memoryLocation[j] = 0;
    }

    //extraction the projection for the cyclopic plane
    double cx = width / 2; 
    double cy = height / 2; 
    double fx = fxl;        
    double fy = fyl;        
    
    
    Matrix K=eye(3,3);
    Matrix Pi=zeros(3,4);
    
    K(0,0)=fx; K(1,1)=fy;
    K(0,2)=cx; K(1,2)=cy; 
    
    Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 
    
    cyclopicPrj  = new Matrix;
    *cyclopicPrj = K * Pi;

    printf("resizing the image with dimension %d %d \n", width, height);
    outputImageMosaic->resize(width,height);
    outputImageMosaic->zero();
    return true;    
}

void mosaicThread::run() {
    //printf("Initialization of the run function %d %d .... \n", width, height);
    count++;

    
    //while (isStopping() != true)  {                

    
        inputImageLeft = imagePortInLeft.read(false); // do not wait                          
        if (inputImageLeft != NULL ) {
            printf("found RGB image \n");
            if(!resized) {
                resize(inputImageLeft->width(),inputImageLeft->height());
                resized = true;
            }
            
            //inputImageRight = imagePortInRight.read(false);                
            makeMosaic(inputImageLeft, inputImageLeft); 
            
            if(imagePortOut.getOutputCount()) {
                imagePortOut.prepare() = *outputImageMosaic;
                imagePortOut.write();
            }
            if(portionPort.getOutputCount()) {
                ImageOf<PixelRgb>& portionImage = portionPort.prepare();
                portionImage.resize(320,240);
                fetchPortion(&portionImage);
                portionPort.write();
            }
        }
    
        

        //----------------------------------------------------------------

        
        inputMonoLeft = imageMonoInLeft.read(false); // do not wait                          
        //printf("Mono image passed \n");
        if (inputMonoLeft != NULL ) {    
            printf("Found MONO image, resizing \n");
            if(!resized) {
                resizeMono(inputMonoLeft->width(),inputMonoLeft->height());
                resized = true;
            }
                           
            makeMosaic(inputMonoLeft, inputMonoLeft);    
        } 
        
        if(imagePortOut.getOutputCount()) {
            imagePortOut.prepare() = *outputImageMosaic;
            imagePortOut.write();
        }
        if(portionPort.getOutputCount()) {
            ImageOf<PixelRgb>& portionImage = portionPort.prepare();
            portionImage.resize(320,240);
            fetchPortion(&portionImage);
            portionPort.write();
        } 
        
       
}


void mosaicThread::makeMosaic(ImageOf<yarp::sig::PixelRgb>* inputImageLeft, ImageOf<PixelRgb>* inputImageRight) {
    //recalculing the position in the space
    double u = 160;
    double v = 120;
    double z = 0.01;
    
    bool isLeft = true;
    Vector fp(3);
    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);
    Vector prec;
    double shift_prev;

    CvPoint2D32f *c1 = new CvPoint2D32f[4];
    CvPoint2D32f *c2 = new CvPoint2D32f[4];
    CvPoint2D32f *cr = new CvPoint2D32f[4];

    int originalPixelSize = inputImageLeft-> getPixelSize();
    printf("The original pixel size for the image %d \n", originalPixelSize);
    
    
    double dimensionX,dimensionY;
    
    if ((invPrj)&&(rectified)) {
        
        Vector torso(3);
        encTorso->getEncoder(0,&torso[0]);
        encTorso->getEncoder(1,&torso[1]);
        encTorso->getEncoder(2,&torso[2]);
        Vector head(5);
        encHead->getEncoder(0,&head[0]);
        encHead->getEncoder(1,&head[1]);
        encHead->getEncoder(2,&head[2]);
        encHead->getEncoder(3,&head[3]);
        encHead->getEncoder(4,&head[4]);
                
        
        Vector q(8);
        double ratio = M_PI /180;
        q[0]=torso[0] * ratio;
        q[1]=torso[1] * ratio;
        q[2]=torso[2] * ratio;
        q[3]=head[0]  * ratio;
        q[4]=head[1]  * ratio;
        q[5]=head[2]  * ratio;
        q[6]=head[3]  * ratio;
        q[7]=head[4]  * ratio;
        double ver = head[5];
        //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
               
                        
        Vector x(3);
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
                
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xeLeft(4),xeRight(4);
        Vector xe = yarp::math::operator *(*invPrj, x);
        xeLeft  = yarp::math::operator *(*invPrjL, x);
        xeRight = yarp::math::operator *(*invPrjR, x);
        xe[3]     = 1.0;  // impose homogeneous coordinates                
        xeLeft[3] = 1.0;
        xeRight[3]= 1.0;
                
        // update position wrt the root frame
        eyeHL = new Matrix(4,4);
        *eyeHL = eyeL->getH(q);        
        eyeHR = new Matrix(4,4);
        *eyeHR = eyeR->getH(q);
        //Matrix* inveyeHL  = new Matrix(pinv(eyeHL->transposed()).transposed());
        
        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
        Vector xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        Vector xoRight = yarp::math::operator *(*eyeHR,xeRight);
        
        prec = fp;
        fp.resize(3,0.0);
        fp[0]=xoLeft[0];
        fp[1]=xoLeft[1];
        fp[2]=xoLeft[2];
        printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);
        
        c2[0].x = 0;    c2[0].y = 0;   
        c2[1].x = 320;  c2[1].y = 0;   
        c2[2].x = 0;    c2[2].y = 240; 
        c2[3].x = 320;  c2[3].y = 240; 

        Vector x_hat(4);

        //___________________________________________________________
        //
        // extracting the warp perspective vertix
        //

        u = 0;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        
        //------------------------------------------
        //printf("vertix %f,%f,%f \n",xoLeft[0],xoLeft[1],xoLeft[2]); 
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*PrjL, xeLeft);
        c1[0].x = x_hat[0]/z;   c1[0].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight);
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[0].x = x_hat[0]/z;   cr[0].y = x_hat[1]/z;
        

        //___________________________________________________________
        
        u = 320;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);                
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*PrjL, xeLeft);
        c1[1].x = x_hat[0]/z;   c1[1].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[1].x = x_hat[0]/z;   cr[1].y = x_hat[1]/z;
        

        //__________________________________________________________

        u = 0;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);       
        x_hat = yarp::math::operator *(*PrjL, xeLeft);
        c1[2].x = x_hat[0]/z;   c1[2].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight);
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[2].x = x_hat[0]/z;   cr[2].y = x_hat[1]/z;
        
        //________________________________________________________

        u = 320;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;        
        //------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);               
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*PrjL, xeLeft);
        c1[3].x = x_hat[0]/z;   c1[3].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[3].x = x_hat[0]/z;   cr[3].y = x_hat[1]/z;

        printf("dimension %f,%f %f,%f %f,%f %f,%f", c1[0].x,c1[0].y, c1[1].x,c1[1].y,c1[2].x,c1[2].y, c1[3].x,c1[3].y);
        dimensionX = abs(c1[1].x - c1[0].x);
        dimensionY = abs(c1[2].y - c1[0].y);
    }


    printf("out of the point project \n");
    
    // double distancey = fp[1] - prec[1];
    //Vector angles = eye->getAng();
    Vector x(3), o(4);
    //igaze->getLeftEyePose(x,o);
    igaze->getAngles(x);
    //printf("angles %f, %f, %f, %f, %f, %f, %f, %f \n", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6],  angles[7] );
    //printf("o %f %f %f \n",o[0],o[1],o[2]);
    //printf("distancey %f   ",distancey);
    //calculating the shift in pixels
    double focalLenght = 200;
    double distance = z;
    double baseline = 0.068;

    // making the mosaic
    int i,j;
    
    unsigned char* outTemp = outputImageMosaic->getRawImage();
    unsigned char* lineOutTemp;

    int iW = inputImageLeft->width();
    int iH = inputImageLeft->height();
    int mPad = outputImageMosaic->getPadding();
    int inputPadding = inputImageLeft->getPadding();
    int rowSize = outputImageMosaic->getRowSize();   
   
    if(rectified) {
        CvMat* mmat = cvCreateMat(3,3,CV_32FC1);
        CvMat* mmatRight = cvCreateMat(3,3,CV_32FC1);        
        float azimuth = ((x[0] * 3.14) / 180);
        float elevation = ((x[1] * 3.14) / 180);
        printf("angles %f %f",azimuth , elevation);

        double deltaX = dimensionX / 320;
        double deltaY = dimensionY / 240;
        printf("dimensionX %f delta %f ", dimensionX ,abs(1 - deltaX));
        double valueA = 120 - 120 * (1 - abs(1 - deltaX));
        double valueB = 120 + 120 * (1 - abs(1 - deltaX));
        double valueC = 160 - 160 * (1 - abs(1 - deltaY));
        double valueD = 160 + 160 * (1 - abs(1 - deltaY));
        
        if(azimuth<0){            
            c1[0].y = c2[0].y;
            c1[1].y = valueA;
            c1[2].y = c2[2].y;
            c1[3].y = valueB;
                         
            cr[0].y = c2[0].y;            
            cr[1].y = valueA;
            cr[2].y = c2[2].y;           
            cr[3].y = valueB;
        }
        else {            
            c1[0].y = valueA;            
            c1[1].y = c2[1].y;            
            c1[2].y = valueB;            
            c1[3].y = c2[3].y;
                        
            cr[0].y = valueA;            
            cr[1].y = c2[1].y;            
            cr[2].y = valueB;            
            cr[3].y = c2[3].y;
        }

        if( elevation < 0) {
            c1[0].x = valueC; 
            c1[1].x = valueD; 
            c1[2].x = c2[2].x;
            c1[3].x = c2[3].x;

            cr[0].x = valueC;
            cr[1].x = valueD;
            cr[2].x = c2[2].x;
            cr[3].x = c2[3].x; 
        }
        else {
            c1[0].x = c2[0].x; 
            c1[1].x = c2[1].x; 
            c1[2].x = valueC;
            c1[3].x = valueD;

            cr[0].x = c2[0].x;
            cr[1].x = c2[1].x;
            cr[2].x = valueC;
            cr[3].x = valueD; 
        }
          
        printf("getting perspective transform \n");   
        mmat = cvGetPerspectiveTransform(c2, c1, mmat);
        mmatRight = cvGetPerspectiveTransform(c2,cr,mmatRight);
        
        //float* dataLeft = mmat->data.fl;
        //float* dataRight = mmatRight->data.fl;
        //for (int i=0; i<3; i++) {
        //    for (int j=0; j<3; j++){
        //        dataLeft[i * 3 + j] = eyeHL->operator()(i,j);
        //        dataRight[i * 3 + j] = eyeHR->operat1or()(i,j);
        //    }
        // }
        
        cvWarpPerspective((CvArr*)inputImageLeft->getIplImage(),(CvArr*)warpImLeft->getIplImage(),mmat,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(0,0,0));
        cvWarpPerspective((CvArr*)inputImageRight->getIplImage(),(CvArr*)warpImRight->getIplImage(),mmatRight,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(0,0,0));
    }
    else {
        warpImLeft = inputImageLeft;
        warpImRight = inputImageRight;
    }
    
    unsigned char* inpTemp = warpImLeft->getRawImage();     
    int inpRowsize = warpImLeft->getRowSize();
    int pixelSizeLeft = warpImLeft->getPixelSize();
    if(pixelSizeLeft == 0) {
        pixelSizeLeft = 1;
    }
    int mosaicX, mosaicY;
    int mosaicXRight, mosaicYRight;
    shift_prev = shiftx;
    shiftx =  fxl * 1.0  * tan((x[0] * 3.14) / 180);
    shifty =  -fyl * 1.0  * tan((x[1] * 3.14) / 180);
    shiftxRight = fxr * 1.0 * tan((x[0] * 3.14) / 180);
    shiftyRight = -fyr * 1.0 * tan((x[1] * 3.14) / 180);
    printf(" shiftx %f shifty %f  shiftx %f shifty %f  \n",shiftx,shifty, shiftxRight,shiftyRight);        
        
    ycoord = shiftx + 320;
    xcoord = shifty + 240;
    ycoordRight = shiftxRight + 320;
    xcoordRight = shiftyRight + 240;

    mosaicX = ycoord - 160 ;   
    mosaicY = xcoord - 120 ;
    mosaicXRight = ycoordRight - 160 ;   
    mosaicYRight = xcoordRight - 120 ;

    float alfa = 0.80;

    //int dimWarpX = max(c1[1].x, c1[3].x) - min(c1[0].x,c1[2].x);
    //int dimWarpY = max(c1[2].y, c1[3].y) - min(c1[0].y,c1[1].y);    
    //printf("%f %f %f %f", c1[0].x,c1[1].x,c1[2].x,c1[3].x);
    //printf("%f %f %f %f", c1[0].y,c1[1].y,c1[2].y,c1[3].y);
    //printf("dimWarp: %d %d ", dimWarpX, dimWarpY);
    
    outTemp = lineOutTemp = outTemp + mosaicY * (rowSize + mPad) + pixelSizeLeft * mosaicX;
    printf("pixelSize for the left image %d %d %d \n", pixelSizeLeft, iH , iW);
    for(i = 0 ; i < iH ; ++i) {
        for(j = 0 ; j < iW ; ++j) {
             
            *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp);
            inpTemp++; outTemp++;
            
            
            *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp); 
            inpTemp++;
            outTemp++;
            
            *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp);
            inpTemp++;
            
            outTemp++;
        }
        inpTemp      += inputPadding;
        outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
    }
        
    if(warpImRight!=NULL) {
        unsigned char* inpTempRight = warpImRight->getRawImage();
        int pixelSizeRight = warpImRight->getPixelSize();
        if(pixelSizeRight == 0) {
            pixelSizeRight = 1;
        }
        outTemp = outputImageMosaic->getRawImage();
        outTemp = lineOutTemp = outTemp + mosaicYRight * (rowSize + mPad) + pixelSizeRight * mosaicXRight;
        for(i = 0 ; i < iH ; ++i) {
            for(j = 0 ; j < iW ; ++j) {
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight); 
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
            }
            inpTempRight += inputPadding;
            outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
        }
    }
    

    //deleting previous locations in the image plane
    //representing new locations on the image plane
    float* pointer = memory;
    //int* pointerimage = memoryimage;
    double distanceEyeObject ;
    Vector xeye(4);
    
    /*
    for(int k = 0; k < countMemory; k++) {
        Vector xoi(4);
        Vector xei_or(4);
        xoi[0] = *pointer++;
        xoi[1] = *pointer++;
        xoi[2] = *pointer++;
        xoi[3] = 1.0; // impose homogeneous coordinate 
        igaze->getLeftEyePose(xeye,xei_or);
        printf("eyePose %f %f %f \n",xeye[0],xeye[1],xeye[2]);
        distanceEyeObject = sqrt ( (xoi[0] - xeye[0]) * (xoi[0] - xeye[0]) +
                          (xoi[1] - xeye[1]) * (xoi[1] - xeye[1]) + 
                          (xoi[2] - xeye[2]) * (xoi[2] - xeye[2]) );

        // update position wrt the eye frame
        Matrix* inveyeH=new Matrix(pinv(eyeHL->transposed()).transposed());         
        Vector xei = yarp::math::operator *(*inveyeH, xoi);        
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector x = yarp::math::operator *(*PrjL, xei);
        printf("x: %f, %f, %f distanceEyeObject %f \n", x[0],x[1], x[2], distanceEyeObject);
        int ui = (int) floor(x[0] / distanceEyeObject);
        int vi = (int) floor(x[1] / distanceEyeObject);
        printf ("x %f y %f z %f , mosaicx %d mosaicy %d >>>> u %d v %d \n", xei[0], xei[1], xei[2],mosaicX, mosaicY,ui, vi);
        //pointerimage = pointerimage + counterMemoryImage * 2;
        //*pointerimage = ui; pointerimage++;
        //*pointerimage = vi;
        unsigned char* outTemp = outputImageMosaic->getRawImage();
        outTemp = outTemp + (height / 2) * (rowSize + mPad) + 3 * (width / 2) + vi * (rowSize + mPad) + ui * 3;
        *outTemp++ = 255;
        *outTemp++ = 0;
        *outTemp++ = 0;
    }
    */
}

void mosaicThread::makeMosaic(ImageOf<yarp::sig::PixelMono>* iImageLeft, ImageOf<PixelMono>* iImageRight) {
    printf("entering the make mosaic for mono image \n");
    //recalculing the position in the space
    double u = 160;
    double v = 120;
    double z = 0.5;
    
    bool isLeft = true;
    Vector fp(3);
    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);
    Vector prec;
    double shift_prev;

    CvPoint2D32f *c1 = new CvPoint2D32f[4];
    CvPoint2D32f *c2 = new CvPoint2D32f[4];
    CvPoint2D32f *cr = new CvPoint2D32f[4];

    int originalPixelSize = iImageLeft-> getPixelSize();
    printf("The original pixel size for the image %d \n", originalPixelSize);
    
    
    if ((invPrj)&&(rectified)) {
        
        Vector torso(3);
        encTorso->getEncoder(0,&torso[0]);
        encTorso->getEncoder(1,&torso[1]);
        encTorso->getEncoder(2,&torso[2]);
        Vector head(5);
        encHead->getEncoder(0,&head[0]);
        encHead->getEncoder(1,&head[1]);
        encHead->getEncoder(2,&head[2]);
        encHead->getEncoder(3,&head[3]);
        encHead->getEncoder(4,&head[4]);
                
        
        Vector q(8);
        double ratio = M_PI /180;
        q[0]=torso[0] * ratio;
        q[1]=torso[1] * ratio;
        q[2]=torso[2] * ratio;
        q[3]=head[0]  * ratio;
        q[4]=head[1]  * ratio;
        q[5]=head[2]  * ratio;
        q[6]=head[3]  * ratio;
        q[7]=head[4]  * ratio;
        double ver = head[5];
        //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
               
                        
        Vector x(3);
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
                
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xeLeft(4),xeRight(4);
        Vector xe = yarp::math::operator *(*invPrj, x);
        xeLeft  = yarp::math::operator *(*invPrjL, x);
        xeRight = yarp::math::operator *(*invPrjR, x);
        xe[3]     = 1.0;  // impose homogeneous coordinates                
        xeLeft[3] = 1.0;
        xeRight[3]= 1.0;
                
        // update position wrt the root frame
        eyeHL = new Matrix(4,4);
        *eyeHL = eyeL->getH(q);        
        eyeHR = new Matrix(4,4);
        *eyeHR = eyeR->getH(q);
        //Matrix* inveyeHL  = new Matrix(pinv(eyeHL->transposed()).transposed());
        
        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
        Vector xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        Vector xoRight = yarp::math::operator *(*eyeHR,xeRight);
        
        prec = fp;
        fp.resize(3,0.0);
        fp[0]=xoLeft[0];
        fp[1]=xoLeft[1];
        fp[2]=xoLeft[2];
        printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);
        
        c2[0].x = 0;    c2[0].y = 0;   
        c2[1].x = 320;  c2[1].y = 0;   
        c2[2].x = 0;    c2[2].y = 240; 
        c2[3].x = 320;  c2[3].y = 240; 

        Vector x_hat(4);

        //___________________________________________________________
        //
        // extracting the warp perspective vertix
        //

        u = 0;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        
        //------------------------------------------
        //printf("vertix %f,%f,%f \n",xoLeft[0],xoLeft[1],xoLeft[2]); 
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[0].x = x_hat[0]/z;   c1[0].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight);
        xeRight  = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat    = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[0].x  = x_hat[0]/z;   cr[0].y = x_hat[1]/z;
        

        //___________________________________________________________
        
        u = 320;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);                
        xeLeft  = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat   = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[1].x = x_hat[0]/z;   c1[1].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight  = yarp::math::operator *(*invPrjR, x);
        xeRight[3] = 1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight  = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat    = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[1].x  = x_hat[0]/z;   cr[1].y = x_hat[1]/z;
        

        //__________________________________________________________

        u = 0;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft  = yarp::math::operator *(*inveyeH0,xoLeft);       
        x_hat   = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[2].x = x_hat[0]/z;   c1[2].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]= 1.0;  // impose homogeneous coordinates
        xoRight   = yarp::math::operator *(*eyeHR,xeRight);
        xeRight   = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat     = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[2].x   = x_hat[0]/z;   cr[2].y = x_hat[1]/z;
        
        //________________________________________________________

        u = 320;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;        
        //------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft   = yarp::math::operator *(*eyeHL,xeLeft);               
        xeLeft   = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat    = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[3].x  = x_hat[0]/z;   c1[3].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]= 1.0;  // impose homogeneous coordinates
        xoRight   = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight   = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat     = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[3].x   = x_hat[0]/z;   cr[3].y = x_hat[1]/z;

        printf("dimension %f %f",  c1[1].x - c1[0].x , c1[3].x - c1[2].x);
        double dimensionX = c1[1].x - c1[0].x;              
    }

   
    
    //double distancey = fp[1] - prec[1];
     Vector x(3);
     Vector o(4);
    

     igaze->getAngles(x);
     printf("before iGaze \n");
    //printf("angles %f, %f, %f, %f, %f, %f, %f, %f \n", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6],  angles[7] );
    
//printf("o %f %f %f \n",o[0],o[1],o[2]);
    //printf("distancey %f   ",distancey);
    
    //calculating the shift in pixels
    double focalLenght = 200;
    double distance = z;
    double baseline = 0.068;
    
    
    // making the mosaic
    int i,j;
    printf("extracting the features of the image \n");
    

    unsigned char* outTemp = outputImageMosaic->getRawImage();
    unsigned char* lineOutTemp;
    int iW = iImageLeft->width();
    int iH = iImageLeft->height();
    int mPad = outputImageMosaic->getPadding();
    int inputPadding = iImageLeft->getPadding();
    int rowSize = outputImageMosaic->getRowSize();   
    

    printf("extracted the features of the image \n");
    
    if(rectified) {
        CvMat* mmat = cvCreateMat(3,3,CV_32FC1);
        CvMat* mmatRight = cvCreateMat(3,3,CV_32FC1);
        
        float azimuth = ((x[0] * 3.14) / 180);
        float elevation = ((x[1] * 3.14) / 180);
        //printf("angles %f %f",azimuth , elevation);
        
        c1[0].x = c1[0].x * 0.6; 
        c1[0].y = c1[0].y * 0.6;//  - (dimensionX-50)  * 0.1;
        c1[1].x = c1[1].x * 0.6; 
        c1[1].y = c1[1].y * 0.6;//  - (dimensionX-50)  * 0.1;
        c1[2].x = c1[2].x * 0.6; 
        c1[2].y = c1[2].y * 0.6;//  + (dimensionX+50)  * 0.1;
        c1[3].x = c1[3].x * 0.6; 
        c1[3].y = c1[3].y * 0.6;//  + (dimensionX+50)  * 0.1;
    
        cr[0].x = cr[0].x * 0.6; 
        cr[0].y = cr[0].y * 0.6;//  - (dimensionX-50)  * 0.1;
        cr[1].x = cr[1].x * 0.6; 
        cr[1].y = cr[1].y * 0.6;//  - (dimensionX-50)  * 0.1;
        cr[2].x = cr[2].x * 0.6; 
        cr[2].y = cr[2].y * 0.6;//  + (dimensionX+50)  * 0.1;
        cr[3].x = cr[3].x * 0.6; 
        cr[3].y = cr[3].y * 0.6;//  + (dimensionX+50)  * 0.1;
        
        //c1[0].x = 0.5 * c2[0].x - abs(azimuth - 0.1)  * 100; 
        //c1[0].y = 0.5 * c2[0].y - abs(azimuth - 0.1)  * 60;
        //c1[1].x = 0.5 * c2[1].x - abs(azimuth + 0.1)  * 100;
        //c1[1].y = 0.5 * c2[1].y - abs(azimuth + 0.1)  * 60;
        //c1[2].x = 0.5 * c2[2].x - abs(azimuth - 0.1)  * 100;
        //c1[2].y = 0.5 * c2[2].y + abs(azimuth - 0.1)  * 60;
        //c1[3].x = 0.5 * c2[3].x - abs(azimuth + 0.1)  * 100;
        //c1[3].y = 0.5 * c2[3].y + abs(azimuth + 0.1)  * 60;
        
        //cr[0].x = c2[0].x;   cr[0].y = c2[0].y;
        //cr[1].x = c2[1].x;   cr[1].y = c2[1].y;
        //cr[2].x = c2[2].x;   cr[2].y = c2[2].y;
        //cr[3].x = c2[3].x;   cr[3].y = c2[3].y;
        
        printf("getting perspective transform \n");   
        mmat = cvGetPerspectiveTransform(c2, c1, mmat);
        mmatRight = cvGetPerspectiveTransform(c2,cr,mmatRight);
        
        //float* dataLeft = mmat->data.fl;
        //float* dataRight = mmatRight->data.fl;
        //for (int i=0; i<3; i++) {
        //    for (int j=0; j<3; j++){
        //        dataLeft[i * 3 + j] = eyeHL->operator()(i,j);
        //        dataRight[i * 3 + j] = eyeHR->operat1or()(i,j);
        //    }
        // }
        
        cvWarpPerspective((CvArr*)inputImageLeft->getIplImage(),(CvArr*)warpImLeft->getIplImage(),mmat,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(255,3,3));
        cvWarpPerspective((CvArr*)inputImageRight->getIplImage(),(CvArr*)warpImRight->getIplImage(),mmatRight,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(255,3,3));
    }
    else {
        warpMnLeft = iImageLeft;
        warpMnRight = iImageRight;
    }

    
    printf("connecting the warp mono images \n");
    unsigned char* inpTemp = warpMnLeft->getRawImage();     
    int inpRowsize = warpMnLeft->getRowSize();
    int pixelSizeLeft = warpMnLeft->getPixelSize();
    if(pixelSizeLeft == 0) {
        pixelSizeLeft = 1;
    }
    int mosaicX, mosaicY;
    int mosaicXRight, mosaicYRight;
    shift_prev = shiftx;
    shiftx =  (int) floor(  fxl * RATIOX  * tan(((x[0] + BIASX) * 3.14) / 180));
    shifty =  (int) floor( -fyl * 1.0  *    tan((x[1] * 3.14) / 180));
    
    shiftxRight =  fxr * 1.0 * ((x[0] * 3.14) / 180);
    shiftyRight = -fyr * 1.0 * ((x[1] * 3.14) / 180);
    printf(" ---------------------------------------x[0] %f  x[1] %f----- shiftx %f shifty %f  fxl %f fyl %f  \n",x[0],x[1],shiftx,shifty, fxl,fyl);        
        
    ycoord = shiftx + 320 + 320;
    xcoord = shifty + 240;
    ycoordRight = shiftxRight + floor((double)width  / 2.0);
    xcoordRight = shiftyRight + floor((double)height / 2.0);

    mosaicX = ycoord - 160 ;
    //mosaicX -= floor(iH / 2);
    mosaicY = xcoord - 120;
    //mosaicY -= floor(iW / 2);
    mosaicXRight = ycoordRight ;
    mosaicXRight -= floor( (double)iH / 2.0);
    mosaicYRight = xcoordRight ;
    mosaicYRight -= floor((double)iW / 2.0);
    float alfa = 0.96;

    //int dimWarpX = max(c1[1].x, c1[3].x) - min(c1[0].x,c1[2].x);
    //int dimWarpY = max(c1[2].y, c1[3].y) - min(c1[0].y,c1[1].y);    
    //printf("%f %f %f %f", c1[0].x,c1[1].x,c1[2].x,c1[3].x);
    //printf("%f %f %f %f", c1[0].y,c1[1].y,c1 [2].y,c1[3].y);
    //printf("dimWarp: %d %d ", dimWarpX, dimWarpY);
    
    outTemp = lineOutTemp = outTemp + mosaicY * (rowSize + mPad) + pixelSizeLeft * mosaicX;
    printf("pixelSize for the left image %d %d %d \n", pixelSizeLeft, iH, iW);
    int* pMemoryLocation = memoryLocation + (mosaicY * 640 + mosaicX);

    for(i = 0 ; i < iH ; ++i) {
        for(j = 0 ; j < iW ; ++j) {
            unsigned char red = *outTemp;
            unsigned char green = *(outTemp + 1);
            unsigned char blue  = *(outTemp + 2);
            
            if((red <= 20) && (green <= 20) && (blue <= 20)) {           
                if(*inpTemp > 10) {
                    unsigned char value = (unsigned char) *inpTemp;
                    *outTemp = value;
                    outTemp++;
                    *outTemp = value;
                    outTemp++;
                    *outTemp = value;
                    outTemp++;        
                }
                else {
                    *outTemp = 0; outTemp++; *outTemp = 0; outTemp++; *outTemp = 0; outTemp++; 
                    /*
                    if(outTemp > 0) {
                        *outTemp -= 1; outTemp++;
                    }
                    if(outTemp > 0) {
                        *outTemp -= 1; outTemp++;
                    }
                    if(outTemp > 0) {
                        *outTemp -= 1; outTemp++;
                    }
                    */
                }
            }
            else {
                //*outTemp -= 1; outTemp++; *outTemp -= 1; outTemp++; *outTemp -= 1; outTemp++; 

                outTemp += 3;   
            }           
            pMemoryLocation++;
            inpTemp++;
        }
        inpTemp      += inputPadding;
        outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
    }
    
    
    forgettingFactor = true;
    if(forgettingFactor) {
        //extracting temporal decay
        endTimer = Time::now();
        double diff =  endTimer - startTimer;
        temporalDecay = diff;
        int temporalDecayInt = (int) temporalDecay;

        printf("forgetting fCT image %d %d %d \n",width, height, temporalDecayInt);
        outTemp = outputImageMosaic->getRawImage();
        int paddingOut = outputImageMosaic->getPadding();
        for(i = 0 ; i < width ; ++i) {
            for(j = 0 ; j < height ; ++j) {
                unsigned char red = *outTemp;
                unsigned char green = *(outTemp + 1);
                unsigned char blue  = *(outTemp + 2);
                
                if((red > temporalDecayInt) && (green > temporalDecayInt) && (blue > temporalDecayInt) ) {               
                    
                    *outTemp = (unsigned char)  red   - temporalDecayInt;
                    outTemp++;
                    *outTemp = (unsigned char)  green - temporalDecayInt;
                    outTemp++;
                    *outTemp = (unsigned char)  blue  - temporalDecayInt;
                    outTemp++;                 
                }
                else {
                    outTemp += 3;                   
                }
            }
            outTemp      += paddingOut;
        }
        startTimer = Time::now();
    }
    
    
    
    /*
    if(warpImRight!=NULL) {
        unsigned char* inpTempRight = warpImRight->getRawImage();
        outTemp = outputImageMosaic->getRawImage();
        outTemp = lineOutTemp = outTemp + mosaicYRight * (rowSize + mPad) + 3 * mosaicXRight;
        for(i = 0 ; i < iH ; ++i) {
            for(j = 0 ; j < iW ; ++j) {
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight); 
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
            }
            inpTempRight += inputPadding;
            outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
        }
    }
    */

    //deleting previous locations in the image plane
    //representing new locations on the image plane
    float* pointer = memory;
    //int* pointerimage = memoryimage;
    double distanceEyeObject ;
    Vector xeye(4);
    
    /*
    for(int k = 0; k < countMemory; k++) {
        Vector xoi(4);
        Vector xei_or(4);
        xoi[0] = *pointer++;
        xoi[1] = *pointer++;
        xoi[2] = *pointer++;
        xoi[3] = 1.0; // impose homogeneous coordinate 
        igaze->getLeftEyePose(xeye,xei_or);
        printf("eyePose %f %f %f \n",xeye[0],xeye[1],xeye[2]);
        distanceEyeObject = sqrt ( (xoi[0] - xeye[0]) * (xoi[0] - xeye[0]) +
                          (xoi[1] - xeye[1]) * (xoi[1] - xeye[1]) + 
                          (xoi[2] - xeye[2]) * (xoi[2] - xeye[2]) );

        // update position wrt the eye frame
        Matrix* inveyeH=new Matrix(pinv(eyeHL->transposed()).transposed());         
        Vector xei = yarp::math::operator *(*inveyeH, xoi);        
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector x = yarp::math::operator *(*PrjL, xei);
        printf("x: %f, %f, %f distanceEyeObject %f \n", x[0],x[1], x[2], distanceEyeObject);
        int ui = (int) floor(x[0] / distanceEyeObject);
        int vi = (int) floor(x[1] / distanceEyeObject);
        printf ("x %f y %f z %f , mosaicx %d mosaicy %d >>>> u %d v %d \n", xei[0], xei[1], xei[2],mosaicX, mosaicY,ui, vi);
        //pointerimage = pointerimage + counterMemoryImage * 2;
        //*pointerimage = ui; pointerimage++;
        //*pointerimage = vi;
        unsigned char* outTemp = outputImageMosaic->getRawImage();
        outTemp = outTemp + (height / 2) * (rowSize + mPad) + 3 * (width / 2) + vi * (rowSize + mPad) + ui * 3;
        *outTemp++ = 255;
        *outTemp++ = 0;
        *outTemp++ = 0;
    }
    */
}


void mosaicThread::threadRelease() {
    resized = false;     
}

void mosaicThread::onStop() {
    imagePortInLeft.interrupt();
    imagePortInRight.interrupt();
    imageMonoInLeft.interrupt();
    imageMonoInRight.interrupt();
    imagePortOut.interrupt();
    portionPort.interrupt();
        
    imagePortOut.close();
    imagePortInLeft.close();
    imagePortInLeft.close();
    imageMonoInLeft.close();
    imageMonoInLeft.close();
    portionPort.close();
}

