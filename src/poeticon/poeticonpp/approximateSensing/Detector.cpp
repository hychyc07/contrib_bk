/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Vadim Tikhanoff, Ali Paikan
 * email:  vadim.tikhanoff@iit.it, ali.paikan@iit.it
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

#include "Detector.h"
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace cv;


void Detector::loop()
{   
    ImageOf<PixelRgbFloat> *world  = stereoWorldPort.read();  // read an stereo world image
    if(world == NULL)
        return;

    IplImage* worldImg = (IplImage*) world->getIplImage();

    yarp::sig::ImageOf<PixelMono> *disparity = disparityPort.read();
    if(!disparity)
        return;

    //cvCvtColor(gray, yarpImage.getIplImage(), CV_GRAY2RGB); 
    IplImage *gray = (IplImage*) disparity->getIplImage();

    ImageOf<PixelMono> &outImage = outImgPort.prepare(); //get an output image
    outImage = *disparity;    
    //outImage.resize(gray->width, gray->height);
    //outImage.wrapIplImage((IplImage*)cvCloneImage(gray));
 
    cvSmooth(gray,gray, CV_GAUSSIAN, gaussian_winsize);
    cvThreshold(gray, gray,threshold, 255.0, CV_THRESH_BINARY);
    cvEqualizeHist(gray,gray); //normalize brightness and increase contrast.            
    cvErode(gray,gray,NULL,erode_itr);
    cvDilate(gray,gray,0,dilate_itr);   

    Bottle blobs, non_blobs;
    int w_offset = cvRound(0.5*gray->width*(1.0-window_ratio));
    int h_offset = cvRound(0.5*gray->height*(1.0-window_ratio));

    int centX=0;
    int centY=0;
    double area=0.0;
    int bigBlobIndex = 0;

    for(int row=h_offset; row<gray->height-h_offset; row++)
    {
        uchar *ptr=(uchar*) gray->imageData + row*gray->widthStep;
        for(int col=w_offset; col<gray->width-w_offset; col++)
        {
            if(ptr[col]==255)
            {
                CvConnectedComp comp;
                //cvResetImageROI(gray);
                cvFloodFill(gray,cvPoint(col,row),cvScalar(255-(blobs.size()+non_blobs.size()+1)),cvScalar(0),cvScalar(0),&comp);
                if(20<comp.rect.width /*&&*comp.rect.width<150*/ && 20<comp.rect.height /*&& comp.rect.height<150*/)
                {
                    
                    double x1 = comp.rect.x;
                    double y1 = comp.rect.y;
                    double x2 = comp.rect.x+comp.rect.width;
                    double y2 = comp.rect.y+comp.rect.height;
                    
                    Bottle &b = blobs.addList();
                    if (comp.rect.height * comp.rect.width > area)
                    {
                        bigBlobIndex = blobs.size()-1;
                        area = comp.rect.height * comp.rect.width;
                        centX = comp.rect.x+comp.rect.width / 2;
                        centY = comp.rect.y+comp.rect.height / 2;
                    }
                                  
                    cvRectangle(outImage.getIplImage(),
                                cvPoint(comp.rect.x, comp.rect.y), cvPoint(comp.rect.x+comp.rect.width, comp.rect.y+comp.rect.height), 
                                CV_RGB(255,255,255));
                    
                    IplImage *clone = cvCloneImage(gray);
                    cvSetImageROI(clone, cvRect(comp.rect.x-10, comp.rect.y-10, comp.rect.width+10, comp.rect.height+10));                        
                    cvSetImageROI((IplImage*)outImage.getIplImage(), cvRect(comp.rect.x-10, comp.rect.y-10, comp.rect.width+10, comp.rect.height+10));

                    CvMemStorage *stor = cvCreateMemStorage(0);
                    CvSeq *cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
                    cvFindContours(clone, stor, &cont, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));

                    for(;cont;cont = cont->h_next)
                         cvDrawContours( outImage.getIplImage(), cont, CV_RGB(255,255,255), CV_RGB(255,255,255), -1, CV_FILLED, 8 );

                    cvResetImageROI((IplImage*)outImage.getIplImage());
                    
                    float X = 0.0;
                    float Y = 0.0;
                    float Z = 0.0;

                    int cnt = 0;
                    for (double u = x1; u < x2; u++)
                    {
                        for (double v = y1; v < y2; v++)
                        {
                            PixelMono& pixel = outImage.pixel(u,v);

                            if (pixel==255)
                            {
                                X+=((float *)(worldImg->imageData + (int)v*worldImg->widthStep))[(int)u*worldImg->nChannels + 0];
                                Y+=((float *)(worldImg->imageData + (int)v*worldImg->widthStep))[(int)u*worldImg->nChannels + 1];
                                Z+=((float *)(worldImg->imageData + (int)v*worldImg->widthStep))[(int)u*worldImg->nChannels + 2];
                                cnt++;
                            }
                        }
                    }
                    
                    if (cnt > 0)
                    {
                        X/=cnt;
                        Y/=cnt;
                        Z/=cnt;
                    }
                    
                    b.addDouble(x1);
                    b.addDouble(y1);
                    b.addDouble(x2);
                    b.addDouble(y2);
                    b.addDouble(X);
                    b.addDouble(Y);
                    b.addDouble(Z);

                    cvRelease((void **)&cont); 
                    cvClearMemStorage( stor ); 
                    cvReleaseMemStorage(&stor);
                    cvReleaseImage(&clone);                       
                }
            }
        }
    }

    if(blobs.size())
    {
        
        blobPort.write(blobs);
        
        Bottle &cmd=faceExpPort.prepare();
        cmd.addVocab(Vocab::encode("set"));
        cmd.addVocab(Vocab::encode("all"));
        cmd.addVocab(Vocab::encode("cur"));
        faceExpPort.write();

        /*yarp::sig::Vector uv(2);
        yarp::sig::Vector posRoot;
        uv[0] = centX;
        uv[1] = centY;
        bool ret = iGaze->get3DPoint(0, uv, 0.5, posRoot );

       // fprintf(stdout,"Valid %lf %lf %lf \n",X, Y, Z ); 
        if ( ( abs(posRoot[1]) < 0.50 ) && ( abs(posRoot[2]-0.40) < 0.18 ) )
        {
            Bottle &target=targetPort.prepare();
            target.clear();
            target.addDouble(posRoot[0]);
            target.addDouble(posRoot[1]);
            target.addDouble(posRoot[2]);
            targetPort.write();

            Bottle &tmp=fixationPort.prepare();
            tmp.clear();
            tmp.addDouble(centX);
            tmp.addDouble(centY);
            fixationPort.write();
        }*/

        Bottle *tmp = blobs.get(bigBlobIndex).asList();
        double Y = tmp->get(5).asDouble();
        double Z = tmp->get(6).asDouble();
        if ( ( abs(Y) < 0.50 ) && ( abs(Z-0.40) < 0.18 ) )
        {
            Bottle &target=targetPort.prepare();
            target.clear();
            target.addDouble(-0.5);// X
            target.addDouble(Y);
            target.addDouble(Z);
            targetPort.write();
            Bottle &tmp=fixationPort.prepare();
            tmp.clear();
            tmp.addDouble(centX);
            tmp.addDouble(centY);
            fixationPort.write();
        }
    }
    outImgPort.write();
}



bool Detector::open(yarp::os::ResourceFinder &rf)
{
    range = rf.check("range", Value(0.6)).asDouble();
    threshold = rf.check("threshold", Value(10.0)).asDouble();
    erode_itr = rf.check("erode_itr", Value(8)).asInt();
    dilate_itr = rf.check("dilate_itr", Value(3)).asInt();
    window_ratio = rf.check("window_ratio", Value(0.6)).asDouble();
    gaussian_winsize = rf.check("gaussian_winsize", Value(9)).asInt();
    faceExpression = rf.check("expression", Value("cun")).asString().c_str();

    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/approximateSensing/gazeClient");

    if (!clientGaze.open(optGaze))
        return false;

    clientGaze.view(iGaze);
    iGaze->blockNeckRoll(0.0);

    bool ret=true;
    ret = disparityPort.open("/approximateSensing/stereo/disparity:i"); 
    ret = ret && stereoWorldPort.open("/approximateSensing/stereo/world:i");
    ret = ret && blobPort.open("/approximateSensing/blobs:o");
    ret = ret && outImgPort.open("/approximateSensing/img:o");  
    ret = ret && faceExpPort.open("/approximateSensing/face:rpc");
    ret = ret && targetPort.open("/approximateSensing/gazeXd");
    ret = ret && fixationPort.open("/approximateSensing/fixation:o");

    return ret;
}

bool Detector::close()
{
    clientGaze.close();
    disparityPort.close();
    stereoWorldPort.close();
    blobPort.close();
    outImgPort.close();
    faceExpPort.close();
    targetPort.close();
    fixationPort.close();
    return true;
}


bool Detector::interrupt()
{
    disparityPort.interrupt();
    stereoWorldPort.interrupt();
    return true;
}


