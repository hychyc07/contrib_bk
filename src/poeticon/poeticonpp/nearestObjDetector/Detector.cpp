// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff, Ali Paikan
 * email:  vadim.tikhanoff@iit.it, ali.paikan@iit.it
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

#define IN(u, p1, p2) ((u>=p1) && (u<=p2)) 

void Detector::loop()
{   
    
    Bottle* blobListIn = blobInPort.read();
    Bottle* targetIn   = targetInPort.read();
    
    if ( !blobListIn || !targetIn )
        return; 

    bool tagetValid = false;

    double cx = 0.0;
    double cy = 0.0;
    float X =0.0;
    float Y =0.0;
    float Z =0.0;

    for (int i = 0; i < blobListIn->size(); i++)
    {
        Bottle *blob = blobListIn->get(i).asList();
        if (blob)
        {
            double x1 = blob->get(0).asDouble();
            double y1 = blob->get(1).asDouble();
            double x2 = blob->get(2).asDouble();
            double y2 = blob->get(3).asDouble();

            X = blob->get(4).asDouble();
            Y = blob->get(5).asDouble();
            Z = blob->get(6).asDouble();
            
            double u1 = targetIn->get(2).asDouble();
            double v1 = targetIn->get(3).asDouble();
            double u2 = targetIn->get(4).asDouble();
            double v2 = targetIn->get(5).asDouble();
            
            cx = targetIn->get(1).asDouble();
            cy = targetIn->get(2).asDouble();           
            

            fprintf(stdout,"The blob is:   %.2lf %.2lf %.2lf %.2lf\n",x1, y1, x2, y2 );
            fprintf(stdout,"The target is: %.2lf %.2lf %.2lf %.2lf\n\n\n",u1, v1, u2, v2 );  
            
            if ( (IN(u1,x1,x2) || IN(u2,x1,x2) ) && 
                (IN(v1,y1,y2) || IN(v2,y1,y2)) )
            {
                    tagetValid = true;
                    break;
            }
            
            if ( (IN(x1,u1,u2) || IN(x2,u1,u2) ) && 
                (IN(y1,v1,v2) || IN(y2,v1,v2)) )
            {
                    tagetValid = true;
                    break;
            }
                
            /*if ( (cx >= x1) && (cx <= x2) && (cy >= y1) && (cy <= y2) )
            {
                tagetValid = true;
                break;
            }*/
        }            
    }    

    if (tagetValid)
    {
        /*ImageOf<PixelRgbFloat> *world  = stereoWorldPort.read();  // read an stereo world image
        if(world == NULL)
            return;
        
        IplImage* worldImg = (IplImage*) world->getIplImage();
        float X=((float *)(worldImg->imageData + (int)cy*worldImg->widthStep))[(int)cx*worldImg->nChannels + 0];
        float Y=((float *)(worldImg->imageData + (int)cy*worldImg->widthStep))[(int)cx*worldImg->nChannels + 1];
        float Z=((float *)(worldImg->imageData + (int)cy*worldImg->widthStep))[(int)cx*worldImg->nChannels + 2];
        */
        if ( (X != 0) && (Y != 0) && (Z != 0) )
        {
            fprintf(stdout,"Valid %lf %lf %lf \n",X, Y, Z ); 
            if ( ( X < -0.25 ) && ( X > -1.20 ) && ( fabs(Y) < 0.50 ) && ( fabs(Z-0.40) < 0.18 ) )
            {
                Bottle &b = targetOutPort.prepare();
                b.clear();
                b.addDouble(X);
                b.addDouble(Y);
                b.addDouble(Z);
                targetOutPort.write();
            }
        }
    }
}

bool Detector::open(yarp::os::ResourceFinder &rf)
{
    faceExpression = rf.check("expression", Value("cun")).asString().c_str();

    bool ret=true;
    ret = ret && stereoWorldPort.open("/nearestObjDetector/stereo/world:i");
    ret = ret && blobInPort.open("/nearestObjDetector/blobs:i");
    ret = ret && targetInPort.open("/nearestObjDetector/target:i"); 
    ret = ret && targetOutPort.open("/nearestObjDetector/target:o");

    return ret;
}

bool Detector::close()
{
    stereoWorldPort.close();
    blobInPort.close();
    targetInPort.close();
    targetOutPort.close();
    return true;
}


bool Detector::interrupt()
{
    stereoWorldPort.interrupt();
    blobInPort.interrupt();
    targetInPort.interrupt();
    return true;
}
