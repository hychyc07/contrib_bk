// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
@ingroup icub_controlbasis_architecture
\defgroup icub_blobTracker blobTracker
 
Detects independent moving points of a grid used to sample the 
input images. 

Copyright (C) 2010 RobotCub Consortium
 
Authors: Stephen Hart
 
Date: 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module takes a binary FG image and returns a list of (tracked) blobs
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. By default \e blobTracker is
  useds
--filter \e on|off
- This parameter turns on the Kalman Filter for the blobs.  The default is off
--smooth \e on|off
- This parameter turns on the Gaussian smoothing of the input image.  Default is on
--num_blobs \e <num>
- This parameter sets the number of blobs the tracker will track.  Deafult is 10.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
- <i> /<stemName>/img:i </i> accepts the incoming FG/BG image. 
 
- <i> /<stemName>/img:o </i> outputs image with the blobs colored by ID

- <i> /<stemName>/blobs:o </i> outputs the list of blogs and their information
 
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux.

\author Stephen Hart
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cv.h>

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "BlobKalmanFilter.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class ProcessThread : public Thread  {
private:
    ResourceFinder &rf;
    
    string name;
    int size_threshold;
    int filter_threshold;
    int width, height;
    int max_tracked_blobs;
    bool filterOn;
    bool smoothingOn;

    BufferedPort< ImageOf<PixelMono> >  inPort;
    BufferedPort< ImageOf<PixelBgr> >  outPort;
    BufferedPort<Bottle> blobPort;
    
    CvSeq *contours, *ptr;
    CvMoments moments;    
    CvScalar color;
	
    BlobUtilities util;

    double t0,t1,dt;

    //Kalman filter for each of the blob
    vector<BlobKalmanFilter*> filters;
    BlobKalmanFilter *filterSingleBlob;   
    vector<FeatureBlob*> rawBlobs;

    CvPoint p, p_last;
    CvPoint tip, tip_left, tip_right;

    double alpha;
    int scale_factor;
    
public:
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }
    
    virtual bool threadInit()
    {
        
        string str;
        
        name=rf.check("name",Value("blobTracker")).asString().c_str();
        size_threshold=rf.check("size_threshold",Value(20)).asInt();
        filter_threshold=rf.check("filter_threshold",Value(100)).asInt();
        max_tracked_blobs=rf.check("num_blobs",Value(10)).asInt();

        str=rf.check("filter",Value("off")).asString().c_str();       
        filterOn = !(str=="off");

        str=rf.check("smooth",Value("off")).asString().c_str(); 
        smoothingOn = !(str=="off");

        inPort.open(("/"+name+"/img:i").c_str());
        outPort.open(("/"+name+"/img:o").c_str());
        blobPort.open(("/"+name+"/blobs:o").c_str());
        
        for(int i=0; i<max_tracked_blobs; i++) {
            rawBlobs.push_back(new FeatureBlob());
        }

        width=0; height=0;
        p_last.x=p_last.y=0;

        alpha = 0.2;
        scale_factor = 3;

        return true;
    }
    
    void afterStart(bool s)
    {
        if (s) {
            fprintf(stdout,"Process started successfully\n");
            fprintf(stdout,"\n");
            fprintf(stdout,"Using ...\n");
            fprintf(stdout,"name\t\t\t = %s\n",name.c_str());
            fprintf(stdout,"num_blobs\t\t = %d\n",max_tracked_blobs);
            fprintf(stdout,"filter\t\t\t = %d\n",filterOn);
            fprintf(stdout,"smoothing\t\t\t = %d\n",smoothingOn);
            fprintf(stdout,"size_threshold\t\t = %d\n",size_threshold);
            fprintf(stdout,"filter_threshold\t = %d\n",filter_threshold);
            fprintf(stdout,"\n");
        }
        else
            fprintf(stdout,"Process did not start\n");
    }
    
    virtual void run()
    {

        bool valid;

        t0=Time::now();                
        while (!isStopping()) {

            valid = false;
 
            // acquire new image
            if (ImageOf<PixelMono> *pImgIn=inPort.read(false)) {

                t1=Time::now();                
                dt = t1-t0;
                t0 = t1;
                                
                // consistency check
                if (pImgIn->width()!=width ||
                    pImgIn->height()!=height) {

                    width = pImgIn->width();
                    height = pImgIn->height();

                    // log message
                    fprintf(stdout,"Detected image of size %dx%d\n", width,height);
                    
                    filterSingleBlob = new BlobKalmanFilter(width,height);
                    filterSingleBlob->setMeasurementNoiseCovariance(1e-6);
                    filterSingleBlob->reset();
                    
                    for(int i=0; i<max_tracked_blobs; i++) {
                        filters.push_back(new BlobKalmanFilter(width,height));
                        filters[i]->setMeasurementNoiseCovariance(1e-6);
                        filters[i]->reset();
                        rawBlobs.push_back(new FeatureBlob());	
                    }
                        
                    // skip to the next cycle
                    continue;
                }                           

                cout << "preparing image..." << endl;

                // blur the image
                if(smoothingOn) {
                    cvSmooth((IplImage*)pImgIn->getIplImage(), (IplImage*)pImgIn->getIplImage(), CV_BLUR, 5, 5);
                }

                // threshold the image
                cvThreshold( (IplImage*)pImgIn->getIplImage(), (IplImage*)pImgIn->getIplImage(), filter_threshold, 255, CV_THRESH_BINARY );
                
                // set up mem storage and find connected components
                CvMemStorage *mem = cvCreateMemStorage(0);
                cvFindContours( (IplImage*)pImgIn->getIplImage(), mem, &contours );

                // get output image
                ImageOf<PixelBgr> &imgOut=outPort.prepare();	
                imgOut.resize(width,height);
                imgOut.zero();
                
                // set up bottle output port for blob info

                Bottle &b=blobPort.prepare();
                b.clear();
                
                // get all the blob information
                CvPoint center;
                Matrix covar(2,2);
                CvBox2D box;
                
                filterSingleBlob->measured.reset();
                
                // reset blob features
                for(unsigned int i=0;i<rawBlobs.size(); i++) {
                    rawBlobs[i]->reset();
                }
                for(unsigned int i=0;i<filters.size(); i++) {
                    filters[i]->measured.reset();
                }                
                
                double area = 0;
                double filtered_area = 0;
                int num_blobs = 0;
                double x,y,x2,y2,xy;

                for (ptr = contours; ptr != NULL; ptr = ptr->h_next) {

                    cout << "checking first contour with size: " << fabs(cvContourArea(ptr)) << endl;
                    // check size of contour
                    if( fabs(cvContourArea(ptr)) >= size_threshold ) {
                        
                        // get a random color for this blob
                        color = CV_RGB( rand()&255, rand()&255, rand()&255 );
                        
                        // draw the contour on the output image
                        cvDrawContours(imgOut.getIplImage(), ptr, color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
                        
                        // retrieve the moments			
                        cvContourMoments(ptr, &moments);
                        
                        // calculate centroid and covariance
                        x = (moments.m10 / moments.m00);
                        y = (moments.m01 / moments.m00);
                        x2 = x*x;
                        y2 = y*y;
                        xy = x*y;

                        covar[0][0] = (moments.m20 / moments.m00) - x2;
                        covar[0][1] = (moments.m11 / moments.m00) - xy;
                        covar[1][0] = covar[0][1];
                        covar[1][1] = (moments.m02 / moments.m00) - y2;

                        //cout << "blob[" << (num_blobs+1) << "] center: (" << center.x << "," << center.y << "), area = "  << area << endl;
                        //cout  << "blob[" << (num_blobs+1) << "] covar: [" << covar[0][0] << " " << covar[0][1] << " " << covar[1][0] << " " << covar[1][1] << "]" << endl;

                        center.x = (int)x;
                        center.y = (int)y;
                        
                        rawBlobs[num_blobs]->firstMoment.x = center.x;
                        rawBlobs[num_blobs]->firstMoment.y = center.y;	
                        rawBlobs[num_blobs]->roi = covar;			
                        rawBlobs[num_blobs]->isValid = true;
                        
                        num_blobs++;
                        
                        //Limit in blobs
            			if(num_blobs==max_tracked_blobs) {
                            //cout << "blob tracker breaking at " << num_blobs << " blobs..." << endl;				
                            break;
                        }
                    }
                    
                }

                // release the memory used for the contour finder
                cvReleaseMemStorage(&mem);
                                
                //update kalman filters
                if(num_blobs>0) {
                    
                    Vector singleBlobCenter(2);
                    Matrix singleBlobCovar(2,2);
                    util.getSingleBlobCharacteristics((IplImage*)imgOut.getIplImage(), &singleBlobCenter, &singleBlobCovar);
                    
                    filterSingleBlob->measured.firstMoment.x = (int)singleBlobCenter[0];
                    filterSingleBlob->measured.firstMoment.y = (int)singleBlobCenter[1];
                    filterSingleBlob->measured.firstMomentDot.x = 0;
                    filterSingleBlob->measured.firstMomentDot.y = 0;
                    filterSingleBlob->measured.roi = singleBlobCovar;
                    filterSingleBlob->measured.isValid = true;		
                    
                    //cout  << "blob[0] covar: [" << singleBlobCovar[0][0] << " " << singleBlobCovar[0][1] << " " << singleBlobCovar[1][0] << " " << singleBlobCovar[1][1] << "]" << endl;
                    //cout << "x: " << singleBlobCenter[0] << endl; 
                    //cout << "y: " << singleBlobCenter[1] << endl; 

                } 
                
                if(filterOn) {
                    filterSingleBlob->update(dt);
                    
                    if(filterSingleBlob->filtered.isValid) {

                        // set the valid flag
                        valid = true;
                        b.addInt(1);

                        // get centroid
                        p = filterSingleBlob->filtered.firstMoment;

                        // filter the centroid
                        p_last.x = (int)(p_last.x + alpha*(p.x - p_last.x));
                        p_last.y = (int)(p_last.y + alpha*(p.y - p_last.y));        

                        // get covar
                        filtered_area = util.getBox2DFromCov(p, filterSingleBlob->filtered.roi, &box);                                                

                        // draw ellipse
                        color = CV_RGB( 255, 0, 0 );
                        if (box.size.width > 0 && box.size.height > 0 && box.size.width < width && box.size.height < height)
        		       		cvEllipseBox((IplImage*)imgOut.getIplImage(), box, color, 3, 16, 0);
        		       	cvCircle((IplImage*)imgOut.getIplImage(), p, 3, color, -1, 8, 0);

                        // get and draw arrow points
                        getArrowPoints(p, p_last, scale_factor, tip, tip_left, tip_right);
                        cvLine((IplImage*)imgOut.getIplImage(), p, tip, CV_RGB(0,255,0), 3, CV_AA, 0);
                        cvLine((IplImage*)imgOut.getIplImage(), tip_left, tip, CV_RGB(0,255,0), 3, CV_AA, 0);
                        cvLine((IplImage*)imgOut.getIplImage(), tip_right, tip, CV_RGB(0,255,0), 3, CV_AA, 0);

                        // add blob to output bottle
                        Bottle &blob = b.addList();
                        blob.addDouble(filterSingleBlob->filtered.firstMoment.x - width/2.0);
                        blob.addDouble(filterSingleBlob->filtered.firstMoment.y - height/2.0);
                        blob.addDouble(filterSingleBlob->filtered.firstMomentDot.x);
                        blob.addDouble(filterSingleBlob->filtered.firstMomentDot.y);
                        blob.addDouble(filterSingleBlob->filtered.roi[0][0]);
                        blob.addDouble(filterSingleBlob->filtered.roi[0][1]);
                        blob.addDouble(filterSingleBlob->filtered.roi[1][0]);
                        blob.addDouble(filterSingleBlob->filtered.roi[1][1]);
                    }
                    
                } else {
                    
                    if(filterSingleBlob->measured.isValid) {
     
                        // set the valid flag
                        valid = true;
                        b.addInt(1);

                        // get centroid
                        p = filterSingleBlob->measured.firstMoment;                  

                        // filter the centroid
                        p_last.x = (int)(p_last.x + alpha*(p.x - p_last.x));
                        p_last.y = (int)(p_last.y + alpha*(p.y - p_last.y));        

                        // get blob covar
                        area = util.getBox2DFromCov(filterSingleBlob->measured.firstMoment, filterSingleBlob->measured.roi, &box);

                        // draw the ellipse
                        color = CV_RGB( 0, 0, 255 );
                        if (box.size.width > 0 && box.size.height > 0 && box.size.width < width && box.size.height < height)
        		       		cvEllipseBox((IplImage*)imgOut.getIplImage(), box, color, 3, 16, 0);
        		       	cvCircle((IplImage*)imgOut.getIplImage(), filterSingleBlob->measured.firstMoment, 3, color, -1, 8, 0);

                        // get and draw arrow points
                        getArrowPoints(p, p_last, scale_factor, tip, tip_left, tip_right);
                        cvLine((IplImage*)imgOut.getIplImage(), p, tip, CV_RGB(0,255,0), 3, CV_AA, 0);
                        cvLine((IplImage*)imgOut.getIplImage(), tip_left, tip, CV_RGB(0,255,0), 3, CV_AA, 0);
                        cvLine((IplImage*)imgOut.getIplImage(), tip_right, tip, CV_RGB(0,255,0), 3, CV_AA, 0);

                        // send blob to output bottle
                        Bottle &blob = b.addList();
                        blob.addDouble(filterSingleBlob->measured.firstMoment.x - width/2.0);
                        blob.addDouble(filterSingleBlob->measured.firstMoment.y - height/2.0);
                        blob.addDouble(filterSingleBlob->measured.firstMomentDot.x);
                        blob.addDouble(filterSingleBlob->measured.firstMomentDot.y);
                        blob.addDouble(filterSingleBlob->measured.roi[0][0]);
                        blob.addDouble(filterSingleBlob->measured.roi[0][1]);
                        blob.addDouble(filterSingleBlob->measured.roi[1][0]);
                        blob.addDouble(filterSingleBlob->measured.roi[1][1]);

                    }
                }

                                
                for(int i=0; i<num_blobs;i++) {
                    
                    if(filterOn) {
                        
                        matchRawDataToEstimates(false, num_blobs);
                        filters[i]->update(dt);
                
                        if(filters[i]->filtered.isValid) {
                            
                            // get ellipse characteristics for drawing				
                            area = util.getBox2DFromCov(filters[i]->filtered.firstMoment, filters[i]->filtered.roi, &box);
		                	color = CV_RGB( 255, 255, 255 );
                            if (box.size.width > 0 && box.size.height > 0 && box.size.width < width && box.size.height < height)
                                cvEllipseBox(imgOut.getIplImage(), box, color, 3, 16, 0);
		                	cvCircle(imgOut.getIplImage(), filters[i]->filtered.firstMoment, 3, color, -1, 8, 0);

                            if(valid) {                            
                                Bottle &blob = b.addList();
                                //blob.addInt(blobID);
                                blob.addDouble(filters[i]->filtered.firstMoment.x - width/2.0);
                                blob.addDouble(filters[i]->filtered.firstMoment.y - height/2.0);
                                blob.addDouble(filters[i]->filtered.firstMomentDot.x);
                                blob.addDouble(filters[i]->filtered.firstMomentDot.y);
                                blob.addDouble(filters[i]->filtered.roi[0][0]);
                                blob.addDouble(filters[i]->filtered.roi[0][1]);
                                blob.addDouble(filters[i]->filtered.roi[1][0]);
                                blob.addDouble(filters[i]->filtered.roi[1][1]);
                            }
                        }
                
                    } else {		
                        
                        // get ellipse characteristics for drawing				
                        area = util.getBox2DFromCov(rawBlobs[i]->firstMoment, rawBlobs[i]->roi, &box);
	                	color = CV_RGB( 0, 255, 0 );
                        if (box.size.width > 0 && box.size.height > 0 && box.size.width < width && box.size.height < height)
                            cvEllipseBox(imgOut.getIplImage(), box, color, 3, 16, 0);
	                	cvCircle(imgOut.getIplImage(), rawBlobs[i]->firstMoment, 3, color, -1, 8, 0);
                        
                        if(valid) {
                            Bottle &blob = b.addList();
                            //blob.addInt(blobID);
                            blob.addDouble(rawBlobs[i]->firstMoment.x - width/2.0);
                            blob.addDouble(rawBlobs[i]->firstMoment.y - height/2.0);
                            blob.addDouble(rawBlobs[i]->firstMomentDot.x);
                            blob.addDouble(rawBlobs[i]->firstMomentDot.y);
                            blob.addDouble(rawBlobs[i]->roi[0][0]);
                            blob.addDouble(rawBlobs[i]->roi[0][1]);
                            blob.addDouble(rawBlobs[i]->roi[1][0]);
                            blob.addDouble(rawBlobs[i]->roi[1][1]);
                        }
                    }
                }		
                                
                // send outputs over YARP
                outPort.write();

                if(!valid) {
                    b.addInt(0);                
                }
                if(b.size() > 0) {	
                    blobPort.write();	
                }
                                                
                //fprintf(stdout,"cycle timing [ms]: %g\n", 1000.0*dt);
            }     
        }
    }
    
    virtual void threadRelease()
    {
        inPort.close();
        outPort.close();
        blobPort.close();
        filterSingleBlob->reset();
        delete filterSingleBlob;
        
        for(unsigned int i=0; i<filters.size(); i++) {
            delete filters[i];
        }
        filters.clear();
        for(unsigned int i=0; i<rawBlobs.size(); i++) {
            delete rawBlobs[i];
        }
        rawBlobs.clear();
    }

    void interrupt()
    {
        inPort.interrupt();
        outPort.interrupt();
        blobPort.interrupt();
    }

	void matchRawDataToEstimates(bool pass, int count)
    {
        double threshold = 60; //150;
        double max_error = 1000000;
        int i, j, k;
        double min_error;
        int mini = 0, minj = 0;
        vector<int> m_assignment(max_tracked_blobs);  //keep track of which raw blob has been assigned to which measurement slot
        vector<int> assignment(max_tracked_blobs);
        bool isInitialize = false;
        
        if (pass) {
            // just pass through raw to the same id measurement
            for (int id = 0; id < max_tracked_blobs; id++) {                    
                filters[id]->measured.firstMoment.x = rawBlobs[id]->firstMoment.x;
                filters[id]->measured.firstMoment.y = rawBlobs[id]->firstMoment.y;
                filters[id]->measured.roi = rawBlobs[id]->roi;
                filters[id]->measured.isValid = rawBlobs[id]->isValid;
            }
        } else {
            int selected;
            for (i = 0; i < max_tracked_blobs; i++) {
                filters[i]->measured.isValid = false;  //initialize all measurements to false
                assignment[i] = -1;
                m_assignment[i] = -1;
            }
            
            Matrix distmat(max_tracked_blobs,max_tracked_blobs);
            Matrix assignments(max_tracked_blobs,2);
            for (i = 0; i < max_tracked_blobs; i++) {
                for (j = 0; j < count; j++) {
                    if (filters[i]->filtered.isValid == true && rawBlobs[j]->isValid == true)  {
                        distmat[i][j] = (int)sqrt((double)(util.sqr((rawBlobs[j]->firstMoment.x - filters[i]->filtered.firstMoment.x)) +
                                                           util.sqr((rawBlobs[j]->firstMoment.y - filters[i]->filtered.firstMoment.y))));
                        isInitialize = true;
                    } else {
                        distmat[i][j] = (int)max_error;
                    }
                }
            }
            
            //find the globally minimum distance assignment
            if (isInitialize) {
                for (k = 0; k < count; k++) {
                    min_error = 100000.0;
                    for (i = 0; i < max_tracked_blobs; i++) {
                        // for (j = 0; j < count; j++)
                        // {
                        if (distmat[i][k] < min_error) {
                            min_error = distmat[i][k];
                            mini = i;
                            minj = k;
                        }
                        // }
                    }
                    if (distmat[mini][minj] > threshold) {
                        assignments[k][0] = -1;
                        assignments[k][1] = -1;
                    } else {
                        assignments[k][0] = mini;
                        assignments[k][1] = minj;
                        for (j = 0; j < count; j++) {
                            distmat[mini][j] = (int)max_error;
                        }
                        for (i = 0; i < max_tracked_blobs; i++) {
                            distmat[i][minj] = (int)max_error;
                        }
                    }
                }
                
                //assign raw blobs to matched nearest measurement slots
                for (k = 0; k < count; k++) {
                    i = (int)assignments[k][0]; j = (int)assignments[k][1];
                    if (i != -1 && j != -1) {
                        assignment[j] = i; //assign jth raw blob to ith measurement slot
                        filters[i]->measured.firstMoment.x = rawBlobs[j]->firstMoment.x;
                        filters[i]->measured.firstMoment.y = rawBlobs[j]->firstMoment.y;
                        filters[i]->measured.roi = rawBlobs[j]->roi;
                        filters[i]->measured.isValid = rawBlobs[j]->isValid;
                    }
                }
            }
            //go through non-assigned raw blobs, and create new filters for them
            for (i = 0; i < count; i++) {
                if (assignment[i] == -1) {
                    //find an available measurment slot
                    selected = -1;
                    for (j = 0; j < max_tracked_blobs; j++) {
                        if (filters[j]->measured.isValid == false){
                            selected = j;
                            break;
                        }
                    }
                    if (selected != -1) {
                        filters[selected]->measured.firstMoment.x = rawBlobs[i]->firstMoment.x;
                        filters[selected]->measured.firstMoment.y = rawBlobs[i]->firstMoment.y;
                        filters[selected]->measured.roi = rawBlobs[i]->roi;
                        filters[selected]->measured.isValid = rawBlobs[i]->isValid;
                        filters[i]->stop();
                        assignment[i] = selected;
                        //cout << "blob[" << i << "] assigned to filter[" << selected << "]" << endl;
                    }
                }
                // if no measurement slot is available, then we cannot filter this raw blob
            }
        }
    }


    void getArrowPoints(CvPoint p_current, CvPoint p_last, int scale_factor, CvPoint &tip, CvPoint &tip_left, CvPoint &tip_right) { 

        CvPoint dp;
        double pi = 3.14159265;
        // compute 1-step length
        dp.x = p_current.x - p_last.x;
        dp.y = p_current.y - p_last.y;

        // compute initial tip location
        tip.x = p_current.x + dp.x;
        tip.y = p_current.y + dp.y;

        // modify tip to reflect scale factor 
        double angle = atan2( (double) p_current.y-tip.y, (double) p_current.x-tip.x );
        double hypot = sqrt( (double) (p_current.y-tip.y)*(p_current.y-tip.y) + (p_current.x-tip.x)*(p_current.x-tip.x) );

        tip.x = (int) (p_current.x - scale_factor * hypot * cos(angle));
        tip.y = (int) (p_current.y - scale_factor * hypot * sin(angle));

        tip_left.x = (int) (tip.x + 9 * cos(angle + pi / 4));
        tip_left.y = (int) (tip.y + 9 * sin(angle + pi / 4));

        tip_right.x = (int) (tip.x + 9 * cos(angle - pi / 4));
        tip_right.y = (int) (tip.y + 9 * sin(angle - pi / 4));

    }
 
};


class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    
public:
    ProcessModule() : thr(NULL) { }
    
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        
        thr=new ProcessThread(rf);
        if (thr->start())
            return true;
        else
            {
                delete thr;    
                return false;
            }
    }
    
    virtual bool close()
    {
        if (thr) 
            {
                thr->interrupt();
                thr->stop();
                delete thr;
            }
        
        return true;
    }
    
    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("filter","off");
    rf.setDefault("smooth","on");
    rf.setDefault("size_threshold","25");
    rf.setDefault("filter_threshold","100");
    rf.setDefault("num_blobs", "10");
    rf.setDefaultContext("blobTracker/conf");
    rf.setDefaultConfigFile("blobTracker.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    ProcessModule mod;

    return mod.runModule(rf);
}


