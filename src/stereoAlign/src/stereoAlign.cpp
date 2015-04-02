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


#include "iCub/stereoAlign.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

bool StereoAlign::configure(yarp::os::ResourceFinder &rf)
{
    /* Process all parameters from both command-line and .ini file */

    dataSetup.name = rf.check("name", Value("stereoAlign"), "module name (string)").asString();
    dataSetup.distortionOffset = rf.check("camDistortionOffset", Value(3), "distortion offset (int)").asInt();
    dataSetup.descriptorRadius = rf.check("descriptorRadius", Value(150), "descriptor radius (double)").asDouble();

    string sendMatch = rf.check("sendMatch", Value("off"), "trigger to send match image (string)").asString().c_str();

    if (sendMatch == "off")
        dataSetup.shouldSend = false;
    else if (sendMatch == "on")
        dataSetup.shouldSend = true;
    else
    {
        fprintf(stderr,"\nERROR, something is wrong with the config.ini file \n");
        return false;
    }

    fprintf(stdout,"\n Will now run with: \n moduleName: %s \n sendMatch: %s \n descriptorRadius: %lf \n distortionOffset: %d \n\n",dataSetup.name.c_str(), sendMatch.c_str(), dataSetup.descriptorRadius, dataSetup.distortionOffset);

    setName(dataSetup.name.c_str());

    handlerPortName =  "/";
    // use getName() rather than a literal
    handlerPortName += getName();

    if (!handlerPort.open(handlerPortName.c_str()))
    {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);

    /* create the thread and pass pointers to the module parameters */
    imageThread = new IMAGEThread( dataSetup );

    /* now start the thread to do the work */
    imageThread->open();

    return true ;
}

/************************************************************************/

bool StereoAlign::interruptModule()
{
    imageThread->interrupt();
    handlerPort.interrupt();
    return true;
}

/************************************************************************/

bool StereoAlign::close()
{
    handlerPort.close();
    /* stop the thread */
    fprintf(stdout, "MODULE: starting the shutdown proceedure \n");
    imageThread->close();
    fprintf(stdout, "MODULE: deleting thread \n");
    delete imageThread;
    fprintf(stdout, "MODULE: done deleting thread \n");
    return true;
}

/************************************************************************/

bool StereoAlign::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();

    if (command.get(0).asString()=="quit")
    {
        reply.addString("quitting");
        return false;
    }
    if (command.get(0).asString()=="dist")
    {
        int dist = command.get(1).asInt();
        imageThread->updateDistortion(dist);
        reply.addString("ok");
        return true;
    }
    if (command.get(0).asString()=="trans")
    {
        int trans = command.get(1).asInt();
        imageThread->updateTranslation(trans);
        reply.addString("ok");
        return true;
    }
    else if (command.get(0).asString()=="help")
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name       name: module name (default: stereoAlign)"<< endl;
        reply.addString("ok");
    }
    else
    {
        cout << "command not known - type help for more info" << endl;
    }
    return true;
}

/************************************************************************/

bool StereoAlign::updateModule()
{
    return true;
}

/************************************************************************/

double StereoAlign::getPeriod()
{
    return 0.1;
}

/************************************************************************/

IMAGEThread::~IMAGEThread()
{
}

/************************************************************************/

IMAGEThread::IMAGEThread( setup &dataSetup )
{
    //set up module name
    this->moduleName = dataSetup.name;
    /* create the class thread*/
    worker = new WorkerClass( dataSetup );
}

/************************************************************************/

bool IMAGEThread::open()
{
    //start the worker thread
    worker->start();
    this->useCallback();

    //create all ports
    inLeftPortName = "/" + moduleName + "/imageRef:i";
    BufferedPort<ImageOf<PixelRgb> >::open( inLeftPortName.c_str() );

    inRightPortName = "/" + moduleName + "/imageMod:i";
    imageInPortRight.open( inRightPortName.c_str() );

    outputPortName = "/" + moduleName + "/imgRect:o";
    imageOutPort.open( outputPortName.c_str() );

    //yarp::os::Network::connect("/icub/cam/left",    inLeftPortName.c_str());
    //yarp::os::Network::connect("/icub/cam/right",   inRightPortName.c_str());
    shift = 0.0;
    shiftXaxis = 0.0;
    return true;
}
/************************************************************************/
void IMAGEThread::updateDistortion(int dist)
{
    mutex.wait();
    worker->updateDistortion(dist);
    mutex.post();
}

/************************************************************************/

void IMAGEThread::updateTranslation(int trans)
{
    mutex.wait();
    this->shiftXaxis = trans;
    mutex.post();
}

/************************************************************************/

void IMAGEThread::onRead(ImageOf<yarp::sig::PixelRgb> &img)
{
    mutex.wait();
    ImageOf<PixelRgb> *rightImg = imageInPortRight.read();
    this->getEnvelope(st);

    cvtColor( Mat((IplImage*)img.getIplImage()), leftMat, CV_RGB2BGR);
    cvtColor( Mat((IplImage*)rightImg->getIplImage()), rightMat, CV_RGB2BGR);
    cvtColor( Mat((IplImage*)rightImg->getIplImage()), final, CV_RGB2BGR);

    worker->updateImages(leftMat, rightMat);
    double tmp = 0.0;

    if ( worker->getMean(tmp) )
    {
        shift = tmp;
        fprintf(stdout,"shift is: %lf \n", shift);
    }

    if (shift < 240)
    {
        for( int j = 0; j < rightMat.rows ; j++ )
        {
            for( int i = 0; i < rightMat.cols; i++ )
            {
                int k = max( (j-(int)shift), 0);
                k = min(k,rightMat.rows);

                int h = max( (i-(int)shiftXaxis), 0);
                h = min(h,rightMat.cols);

                final.at<Vec3b>(j,i) = rightMat.at<Vec3b>(k,h);
            }
        }
    }
    else
        fprintf(stdout,"ERROR WITH THE SHIFT\n");

    ImageOf<PixelRgb> imgRight;
    imgRight.resize(rightMat.cols, rightMat.rows);
    cvtColor( final, final, CV_BGR2RGB);
    IplImage tmpR = final;
    cvCopyImage( &tmpR, (IplImage *) imgRight.getIplImage());
    imageOutPort.setEnvelope(st);
    imageOutPort.write(imgRight);

    mutex.post();
}

/************************************************************************/

void IMAGEThread::close()
{
    mutex.wait();
    fprintf(stdout, "THREAD: now closing ports... \n");
    imageInPortRight.close();
    imageOutPort.close();
    worker->stop();

    delete worker;

    fprintf(stdout, "THREAD: attempting to close read port... \n");
    BufferedPort<ImageOf<PixelRgb> >::close();
    fprintf(stdout, "THREAD: finished closing the read port... \n");
    mutex.post();
    fprintf(stdout, "THREAD: done... \n");
}

/************************************************************************/

void IMAGEThread::interrupt()
{
    mutex.wait();
    fprintf(stdout, "THREAD: cleaning up... \n");
    fprintf(stdout, "THREAD: interrupting ports... \n");
    imageInPortRight.interrupt();
    BufferedPort<ImageOf<PixelRgb> >::interrupt();
    fprintf(stdout, "THREAD: finished interrupting ports... \n");
    mutex.post();
}

/************************************************************************/

WorkerClass::WorkerClass( setup &dataSetup )
{
    this->moduleName        = dataSetup.name;
    this->descriptorRadius  = dataSetup.descriptorRadius;
    this->distortionOffset  = dataSetup.distortionOffset;
    this->shouldSend        = dataSetup.shouldSend;
    mean = 0.0;

    if (shouldSend)
    {
        matchPortName = "/" + moduleName + "/match:o";
        matchOutPort.open( matchPortName.c_str() );
    }
    dataPortName = "/" + moduleName + "/data:o";
    dataOutPort.open( dataPortName.c_str() );
}

/************************************************************************/

WorkerClass::~WorkerClass()
{
}

/************************************************************************/

void WorkerClass::onStop()
{
    fprintf(stdout, "stopping....\n");
    this->event.signal();
    fprintf(stdout, "stopping done....\n");
}

/************************************************************************/

void WorkerClass::threadRelease()
{
    if (shouldSend)
        matchOutPort.close();
    dataOutPort.close();
}
/************************************************************************/

void WorkerClass::updateImages(Mat &left, Mat &right)
{
    if (workermutex.check())
    {
        leftMat = left;
        rightMat = right;
        event.signal();
        workermutex.post();
    }
}

/***********************************************************************/

void WorkerClass::updateDistortion(int dist)
{
    workermutex.wait();
    this->distortionOffset = dist;
    workermutex.post();
}

/************************************************************************/

void WorkerClass::run()
{
    while (isStopping() != true) {
        event.wait();
        workermutex.wait();

        if (leftMat.cols > 0 && rightMat.cols>0)
        {
            Bottle data;
            Ptr<FeatureDetector> detector = FeatureDetector::create( "SIFT" );
            Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("SIFT");
            Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create("BruteForce" );

            vector<KeyPoint> keypoints1;
            detector->detect( leftMat, keypoints1 );
            Mat descriptors1;
            descriptorExtractor->compute(leftMat, keypoints1, descriptors1 );

            vector<KeyPoint> keypoints2;
            detector->detect( rightMat, keypoints2 );
            Mat descriptors2;
            descriptorExtractor->compute( rightMat, keypoints2, descriptors2 );

            vector<DMatch> filteredMatches;
            crossCheckMatching( descriptorMatcher, descriptors1, descriptors2, filteredMatches, descriptorRadius, 1 );
            fprintf(stdout, "SIFT matches: %lu ", filteredMatches.size());

            vector<Point2f> key2fPoints1, key2fPoints2 ;
            vector<int> pointIndexes1;
            vector<int> pointIndexes2;

            vector<char> matchMask(filteredMatches.size(),1);
            for(int i=0; i<(int)filteredMatches.size(); i++)
            {
                Point2f pointL=keypoints1[filteredMatches[i].queryIdx].pt;
                Point2f pointR=keypoints2[filteredMatches[i].trainIdx].pt;

                if(abs(pointL.y-pointR.y)<10)//displacement 10 320x240 20 640x480
                {
                    if(abs(pointR.x-pointL.x)<50)
                    {
                        pointIndexes1.push_back(filteredMatches[i].queryIdx);
                        pointIndexes2.push_back(filteredMatches[i].trainIdx);
                    }
                    else
                        matchMask[i]=0;
                } else
                    matchMask[i]=0;
            }

            KeyPoint::convert(keypoints1, key2fPoints1, pointIndexes1);
            KeyPoint::convert(keypoints2, key2fPoints2, pointIndexes2);

            double temp = 0.0;
            mean = 0.0;
            for(size_t i=0; i<key2fPoints1.size(); i++)
            {
                temp = key2fPoints1[i].y - key2fPoints2[i].y;
                if ( (temp < 10.0) && (temp > -10.0) )
                    mean += temp;
            }

            mean/=(double)key2fPoints1.size();
            mean = ceil(mean);
            mean = mean - distortionOffset;

            data.addDouble(mean);
            data.addInt(filteredMatches.size());
            data.addInt(key2fPoints1.size());
            dataOutPort.write(data);

            if ( shouldSend )
            {
                drawMatches(leftMat, keypoints1, rightMat, keypoints2, filteredMatches, matMatches, NULL, Scalar(0,0,255,0), matchMask);
                cvtColor( matMatches, matMatches, CV_BGR2RGB);
                ImageOf<PixelRgb> imgMatch;
                imgMatch.resize(matMatches.cols, matMatches.rows);
                IplImage tmpR = matMatches;
                cvCopyImage( &tmpR, (IplImage *) imgMatch.getIplImage());
                matchOutPort.write(imgMatch);
            }
        }
        workermutex.post();
        event.reset();
    }
}

/************************************************************************/
void WorkerClass::crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                                     const Mat& descriptors1, const Mat& descriptors2,
                                     vector<DMatch>& filteredMatches12, double radius, int knn )
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    //descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    //descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
    descriptorMatcher->radiusMatch( descriptors1, descriptors2, matches12, (float) radius );
    descriptorMatcher->radiusMatch( descriptors2, descriptors1, matches21, (float) radius );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}

/************************************************************************/
bool WorkerClass::getMean(double &shift)
{
    if (workermutex.check())
    {
        shift = mean;
        workermutex.post();
        return true;
    }
    return false;
}

//empty line to make gcc happy

