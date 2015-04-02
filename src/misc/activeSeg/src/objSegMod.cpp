/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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

#include "iCub/objSegMod.h"
#include <math.h>

//OPENCV
#include <cv.h>
#include <highgui.h>


using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

bool objSegMod::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("objSegMod"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
    setName(moduleName.c_str());

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerName =  "/";
    handlerName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    //attachTerminal();                     // attach to terminal (maybe not such a good thing...)
    /* create the thread and pass pointers to the module parameters */
    objSegThread = new OBJSEGThread( moduleName );

    /* now start the thread to do the work */
    objSegThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;// let the RFModule know everything went well
}


bool objSegMod::interruptModule()
{
    handlerPort.interrupt();
    return true;
}

bool objSegMod::close()
{
	handlerPort.interrupt();
    cout << "deleting thread " << endl;
    objSegThread->stop();
    
    delete objSegThread;
    return true;
}

bool objSegMod::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else{
			cout << "command not known - type help for more info" << endl;
	}
    return true;
}

/* Called periodically every getPeriod() seconds */

bool objSegMod::updateModule()
{
    return true;
}

double objSegMod::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    
    return 0.1;
}

OBJSEGThread::~OBJSEGThread()
{
}

OBJSEGThread::OBJSEGThread( string moduleName )
{
    cout << "initialising Variables" << endl;
    this->moduleName = moduleName;

    inputNameImage = "/" + moduleName + "/image:i";
    imageIn.open( inputNameImage.c_str() );

    outputNameSeg = "/" + moduleName + "/image:o";
    imageOut.open( outputNameSeg.c_str() );

    outputNameSegCrop = "/" + moduleName + "/imageCrop:o";
    imageOutCrop.open( outputNameSegCrop.c_str() );

    fixationNamePort = "/" + moduleName + "/fixPoint:i";
    fixationPort.open( fixationNamePort.c_str() );
}

bool OBJSEGThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */
    seg = NULL;
    segOnly = NULL;
    segOnlyRgb = NULL;
    segRgb= NULL;
    allocated = false;
    once = false;
    first = true;
    fix_x_prev = 0.0;
    fix_y_prev = 0.0;
    cout << "ObjSeg Started..." << endl;
    return true;
}

void OBJSEGThread::run(){

    while (isStopping() != true) { // the thread continues to run until isStopping() returns true
       
    	//get the images and run
        cout << "ok, waiting for new fixation point..." << endl;
        Bottle fixIn;
        fixIn.clear();
        
        fixationPort.read(fixIn);

        if (fixIn!=NULL){
            if (fixIn.get(0).asString()=="again"){
                ImageOf<PixelBgr> &segCrop = imageOutCrop.prepare();        
                segCrop.wrapIplImage(segOnlyRgb);
                imageOutCrop.write();
                cout << "sent the previous segmentation as a template" << endl;
            }
            else{
                double fix_x = fixIn.get(0).asDouble();
                double fix_y = fixIn.get(1).asDouble();
                double cropSizeWidth = fixIn.get(2).asInt();
                double cropSizeHeight = fixIn.get(3).asInt();

                if (!cropSizeWidth)
                    cropSizeWidth = 50;

                if (!cropSizeHeight)
                    cropSizeHeight = 50;

                cout << "Fixation point is " << fix_x << " " << fix_y << endl;
                cout << fix_x_prev << " " << fix_y_prev << " " << fix_x << " " << fix_y << endl;
                
                IplImage *img_in = cvCloneImage((IplImage *) imageIn.read(true)->getIplImage());
                if( img_in != NULL ){

                    double start = Time::now();
                    cvSetImageROI(img_in, cvRect(fix_x-cropSizeWidth/2, fix_y-cropSizeHeight/2, cropSizeWidth, cropSizeHeight));
                    IplImage *img_crop = cvCreateImage(cvGetSize(img_in), img_in->depth, img_in->nChannels);
                    cvCopy(img_in, img_crop, NULL);
                    
                    cvResetImageROI(img_in);

                    //segmentWithFixation(img_crop, fix_x, fix_y);
                    segmentWithFixation(img_crop, img_crop->width/2, img_crop->height/2);

                    //Vector twobeewaved = frame.temp;

                    sendSegOnly(seg, img_crop);
                    
                    cvCircle(segRgb, cvPoint( img_crop->width/2, img_crop->height/2 ), 2, cvScalar( 0,255,0 ), 3 );

                    //--------WAVED

                    //waveletEncoder enc;
                    //Vector wavedCoeffs = enc.encode(twobeewaved, WAVE_R);

                    printf( "\n\nTime elapsed: %f seconds\n", ( Time::now() - start ) );
                    //Send and clean  
                    ImageOf<PixelBgr> *segImg = new ImageOf<PixelBgr>;
                    segImg->resize( segRgb->width, segRgb->height );
                    cvCopyImage(segRgb, (IplImage*)segImg->getIplImage());
                    imageOut.prepare() = *segImg;
                    imageOut.write();

                    ImageOf<PixelBgr> *segCrop = new ImageOf<PixelBgr>;
                    segCrop->resize(segOnlyRgb->width, segOnlyRgb->height);
                    cvCopyImage(segOnlyRgb, (IplImage*)segCrop->getIplImage());    
                    imageOutCrop.prepare() = *segCrop;             
                    imageOutCrop.write();

                    //cvSaveImage("test.jpg" ,segRgb);
                    delete segImg;
                    delete segCrop;
                    cvReleaseImage(&segRgb);
                    cvReleaseImage(&seg);
                    //
                    segRgb = NULL;
                    seg = NULL;
                    frame.deallocateMemForContours();
                    
                    once = false;
                }
                fix_x_prev = fix_x;
                fix_y_prev = fix_y;
            }
        }
    }
}
void OBJSEGThread::segmentWithFixation(IplImage *img_in, double x, double y){

    cout << "setting the image" << endl;
    //set the image
    frame.setImage(img_in);
    cout << "finished setting and processing edge started " << endl;
    // Edge detection!
    frame.edgeCGTG();
    cout << "finished processing and generating boundaries" << endl;
    frame.generatePbBoundary();
    cout << "finished generating boundaries and setting fixation point" << endl;
    //use fixation point!
	frame.assignFixPt(x,y);
    //segment
    frame.allocateMemForContours();
    frame.segmentCurrFix();
    //get the seg
    seg = frame.getSeg();
    segRgb = cvCreateImage( cvGetSize(seg), seg->depth, seg->nChannels );
    cvCvtColor(seg, segRgb, CV_BGR2RGB);
}

void OBJSEGThread::sendSegOnly(IplImage *img, IplImage *imgOrig){

    int top = -1;
    int left = -1;
    int right = -1;
    int bottom = -1;

    cv::Mat imgMat = img;    
 
    for (int j=0;j<imgMat.rows;j++){
        for (int i=0;i<imgMat.cols;i++){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 255 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    top = j; 
                    goto topFound;
            }
        }
    }

    topFound:
    for (int j=imgMat.rows-1; j>0; j--){
        for (int i=imgMat.cols-1; i>0 ;i--){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 255 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    bottom = j; 
                    goto bottomFound;
            }
        }
    }

    bottomFound:
    for (int i=0;i<imgMat.cols;i++){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 255 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    left = i; 
                    goto leftFound;
            }        
       }
    }
    
    leftFound:
    for (int i=imgMat.cols-1;i>0;i--){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] == 0 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] == 255 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] == 0 )
            {
                    right = i; 
                    goto rightFound;
            }        
       }
    }
    
    rightFound:
    //segOnly = img
    cvSetImageROI (imgOrig, cvRect(left, top, right-left, bottom-top) );   
    segOnly = cvCreateImage(cvSize(right-left,  bottom-top), IPL_DEPTH_8U, 3 );
    cvCopy(imgOrig, segOnly); 
    if (segOnlyRgb!=NULL)
        cvReleaseImage(&segOnlyRgb);
      
    segOnlyRgb = cvCreateImage(cvSize(right-left,  bottom-top), IPL_DEPTH_8U, 3 );
    cvCvtColor(segOnly, segOnlyRgb, CV_BGR2RGB);

}

void OBJSEGThread::onStop(){

    cout << "cleaning up..." << endl;
    cout << "attempting to close ports" << endl;
    imageIn.interrupt();
    imageOut.interrupt();
    fixationPort.interrupt();
    imageIn.close();
    imageOut.close();
    fixationPort.close();
    cout << "finished closing ports" << endl;
}

void OBJSEGThread::threadRelease() 
{
    /* for example, delete dynamically created data-structures */
}

