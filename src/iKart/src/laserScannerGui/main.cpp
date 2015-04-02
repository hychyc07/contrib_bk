/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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
\defgroup laserScannerGui laserScannerGui
 
@ingroup icub_tools
@ingroup icub_guis
 
A simple GUI to display the distance mesaurements of iKart's laser scanner.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
A simple GUI to display the distance mesaurements of iKart's laser scanner.
 
\section lib_sec Libraries 
- YARP libraries. 
- OpenCV libraries.

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
None. 
 
\section portsc_sec Ports Created 
The module creates the port /laserScannerGui:i used to receive the laser data.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include<iostream>
#include<iomanip>
#include<sstream>
#include<fstream>
#include<string>
#include<stdio.h>

#define _USE_MATH_DEFINES
#include<math.h>

#include<cv.h>
#include<highgui.h>

#include <yarp/dev/Drivers.h>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

double scale =100; //global scale factor 
double robot_radius = 715.0/2.0/1000.0; //m
double laser_position = 245.0/1000.0; //m
bool verbose = false;
bool absolute = false;
bool compass  = true;
int rate=50; //ms
CvFont font;
CvFont fontBig;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

const CvScalar color_bwhite = cvScalar(200,200,255);
const CvScalar color_white  = cvScalar(255,255,255);
const CvScalar color_red    = cvScalar(0,0,255);
const CvScalar color_yellow = cvScalar(0,255,255);
const CvScalar color_black  = cvScalar(0,0,0);
const CvScalar color_gray   = cvScalar(100,100,100);

struct lasermap_type
{
    double x;
    double y;
    lasermap_type() {x=y=0.0;}
};

void drawGrid(IplImage *img)
{
    cvLine(img,cvPoint(0,0),cvPoint(img->width,img->height),color_black);
    cvLine(img,cvPoint(img->width,0),cvPoint(0,img->height),color_black);
    cvLine(img,cvPoint(img->width/2,0),cvPoint(img->width/2,img->height),color_black);
    cvLine(img,cvPoint(0,img->height/2),cvPoint(img->width,img->height/2),color_black);
    const int step = (int)(0.5 * scale); //mm
/*
    for (int xi=0; xi<img->width; xi+=step)
        cvLine(img,cvPoint(xi,0),cvPoint(xi,img->height),color_black);
    for (int yi=0; yi<img->height; yi+=step)
        cvLine(img,cvPoint(0,yi),cvPoint(img->width,yi),color_black);
*/
    char buff [10];
    int  rad_step=0;
    if   (scale>60) 
        rad_step=1;
    else             
        rad_step=2;
    for (int rad=0; rad<20; rad+=rad_step)
    {
        sprintf (buff,"%3.1fm",float(rad)/2);
        cvPutText(img, buff, cvPoint(img->width/2,int(float(img->height)/2.0-float(step)*rad)), &font, cvScalar(0, 0, 0, 0));
        cvCircle(img,cvPoint(img->width/2,img->height/2),step*rad,color_black);
    }

}

void drawRobot (IplImage *img)
{
    cvRectangle(img,cvPoint(0,0),cvPoint(img->width,img->height),cvScalar(0,0,0),CV_FILLED);

    //draw a circle
    cvCircle(img,cvPoint(img->width/2,img->height/2),(int)(robot_radius*scale),color_gray,CV_FILLED);
    cvCircle(img,cvPoint(img->width/2,img->height/2),(int)(robot_radius*scale-1),color_black);
    cvCircle(img,cvPoint(img->width/2,img->height/2),(int)(robot_radius*scale-2),color_black);
}

void drawCompass(const Vector *comp, IplImage *img)
{
    int sx = 0;
    int sy = 0;
    int ex = 0;
    int ey = 0;
    int tx = 0;
    int ty = 0;
    char buff [20];
    cvCircle(img,cvPoint(img->width/2,img->height/2),250,color_black);
    for (int i=0; i<360; i+=10)
    {
        double ang;
        if  (absolute) ang = i+180;
        else           ang = i+(*comp)[0]+180;
        sx = int(-250*sin(ang/180.0*3.14)+img->width/2);
        sy = int(250*cos(ang/180.0*3.14)+img->height/2);
        ex = int(-260*sin(ang/180.0*3.14)+img->width/2);
        ey = int(260*cos(ang/180.0*3.14)+img->height/2);
        tx = int(-275*sin(ang/180.0*3.14)+img->width/2);
        ty = int(275*cos(ang/180.0*3.14)+img->height/2);
        cvLine(img,cvPoint(sx,sy),cvPoint(ex,ey),color_black);
        CvSize tempSize;
        int lw;
        if      (i==0)     {sprintf(buff,"N");    cvGetTextSize( buff, &fontBig, &tempSize, &lw ); cvPutText(img, buff, cvPoint(tx-tempSize.width/2,ty+tempSize.height/2), &fontBig, cvScalar(0, 0, 0, 0));}
        else if (i==90)    {sprintf(buff,"E");    cvGetTextSize( buff, &fontBig, &tempSize, &lw ); cvPutText(img, buff, cvPoint(tx-tempSize.width/2,ty+tempSize.height/2), &fontBig, cvScalar(0, 0, 0, 0));}
        else if (i==180)   {sprintf(buff,"S");    cvGetTextSize( buff, &fontBig, &tempSize, &lw ); cvPutText(img, buff, cvPoint(tx-tempSize.width/2,ty+tempSize.height/2), &fontBig, cvScalar(0, 0, 0, 0));}
        else if (i==270)   {sprintf(buff,"W");    cvGetTextSize( buff, &fontBig, &tempSize, &lw ); cvPutText(img, buff, cvPoint(tx-tempSize.width/2,ty+tempSize.height/2), &fontBig, cvScalar(0, 0, 0, 0));}
        else               {sprintf(buff,"%d",i); cvGetTextSize( buff, &font   , &tempSize, &lw ); cvPutText(img, buff, cvPoint(tx-tempSize.width/2,ty+tempSize.height/2), &font, cvScalar(0, 0, 0, 0));}
    }
}

void drawNav(const yarp::os::Bottle *display, IplImage *img)
{
    if (display->size()==8)
    {
        printf ("wrong format!\n");
        return;
    }
    double c0 = display->get(0).asDouble();
    double c1 = display->get(1).asDouble();
    double c2 = display->get(2).asDouble();
    double angle_f = display->get(3).asDouble();
    double angle_t = display->get(4).asDouble();
    double w_f = display->get(5).asDouble();
    double w_t = display->get(6).asDouble();
    double max_obs_dist = display->get(7).asDouble();
    double angle_g = display->get(8).asDouble();

    CvPoint center;
    center.x = (int)(img->width/2  );
    center.y = (int)(img->height/2 );

    CvPoint ray;
    ray.x=int(200*sin(DEG2RAD*angle_f));
    ray.y=-int(200*cos(DEG2RAD*angle_f));
    ray.x += center.x;
    ray.y += center.y;

    int thickness = 3;
    cvLine(img,center,ray,color_bwhite,thickness);

    ray.x=int(100*sin(DEG2RAD*c0));
    ray.y=-int(100*cos(DEG2RAD*c0));
    ray.x += center.x;
    ray.y += center.y;
    cvLine(img,center,ray,color_red,thickness);

    ray.x=int(150*sin(DEG2RAD*angle_g));
    ray.y=-int(150*cos(DEG2RAD*angle_g));
    ray.x += center.x;
    ray.y += center.y;
    cvLine(img,center,ray,color_yellow,thickness);

    cvCircle(img,cvPoint(img->width/2,img->height/2),(int)(max_obs_dist*scale-1),color_black);
}

void drawLaser(const Vector *comp, const Vector *las, const lasermap_type *lmap, IplImage *img)
{
    cvZero(img);
    cvRectangle(img,cvPoint(0,0),cvPoint(img->width,img->height),cvScalar(255,0,0),-1);
    CvPoint center;

    double center_angle;
    if (!absolute) center_angle=0;
    else center_angle = -180-(*comp)[0];
    center.x = (int)(img->width/2  + (laser_position*scale)*sin(center_angle/180*3.14) );
    center.y = (int)(img->height/2 - (laser_position*scale)*cos(center_angle/180*3.14) );

    double angle =0;
    double lenght=0;
    static double old_time=0;

    if (!las || !comp) 
    {
        return;
    }

    double curr_time=yarp::os::Time::now();
    if (verbose) fprintf(stderr,"received vector size:%d ",int(las->size()));
    static int timeout_count=0;
    if (curr_time-old_time > 0.40) timeout_count++;
    if (verbose) fprintf(stderr,"time:%f timeout:%d\n",curr_time-old_time, timeout_count);
    old_time = curr_time;
    for (int i=0; i<1080; i++)
    {
        lenght=(*las)[i];
        if      (lenght<0)     lenght = 0;
        else if (lenght>15)    lenght = 15; //15m maximum
        angle=i/1080.0*270.0-(90-(360-270)/2);

        //lenght=i; //this line is for debug only
        angle-=center_angle;
        double x = lenght*scale*cos(angle/180*3.14);
        double y = -lenght*scale*sin(angle/180*3.14);

        CvPoint ray;
        ray.x=int(x);
        ray.y=int(y);
        ray.x += center.x;
        ray.y += center.y;

        int thickness = 2;
        //draw a line
        cvLine(img,center,ray,color_white,thickness);

        if (lmap)
        {
            CvPoint ray2;
            ray2.x=int(lmap[i].x*scale);
            ray2.y= -int(lmap[i].y*scale);
            ray2.x += (center.x - int((laser_position*scale)*sin(center_angle/180*M_PI)));
            ray2.y += (center.y + int((laser_position*scale)*cos(center_angle/180*M_PI)));
            cvLine(img,center,ray2,color_bwhite,thickness);
        }
    }
}


int main(int argc, char *argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;

    string laser_port_name;
    laser_port_name = "/laserScannerGui/laser:i";
    string laser_map_port_name;
    laser_map_port_name = "/laserScannerGui/laser_map:i";
    string compass_port_name;
    compass_port_name = "/laserScannerGui/compass:i";
    string nav_display;
    nav_display = "/laserScannerGui/nav_display:i";

    int width = 600;
    int height = 600;

    BufferedPort<yarp::sig::Vector> laserInPort;
    laserInPort.open(laser_port_name.c_str());
    BufferedPort<yarp::os::Bottle> laserMapInPort;
    laserMapInPort.open(laser_map_port_name.c_str());
    BufferedPort<yarp::sig::Vector> compassInPort;
    compassInPort.open(compass_port_name.c_str());
    BufferedPort<yarp::os::Bottle> navDisplayInPort;
    navDisplayInPort.open(nav_display.c_str());

    IplImage *img  = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    IplImage *img2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    cvNamedWindow("Laser Scanner GUI",CV_WINDOW_AUTOSIZE);
    cvInitFont(&font,    CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4, 0, 1, CV_AA);
    cvInitFont(&fontBig, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 1, CV_AA);

    bool exit = false;
    yarp::sig::Vector compass_data;
    compass_data.resize(3, 0.0);

    yarp::sig::Vector laser_data;
    lasermap_type     lasermap_data [1080];
    laser_data.resize(1080, 0.0);

    while(!exit)
    {
        if (compass)
        {
            yarp::sig::Vector *cmp = compassInPort.read(false);
            if (cmp) compass_data = *cmp;
        }

        yarp::sig::Vector *las = laserInPort.read(false);
        if (las) laser_data = *las;

        yarp::os::Bottle *las_map = laserMapInPort.read(false);
        if (las_map)
        {
            for (unsigned int i=0; i<1080; i++)
            {
                Bottle* b = las_map->get(i).asList();
                lasermap_data[i].x = b->get(0).asDouble();
                lasermap_data[i].y = b->get(1).asDouble();
            }
        }

        //The drawing functions.
        {
            if (las_map)
            {drawLaser(&compass_data,&laser_data,lasermap_data,img);}
            else
            {drawLaser(&compass_data,&laser_data,0,img);}
            drawRobot(img2);
            drawGrid(img);
            if (compass) drawCompass(&compass_data,img);

            yarp::os::Bottle *nav_display = navDisplayInPort.read(false);
            if (nav_display)
            {
                drawNav(nav_display,img);
            }

            cvAddWeighted(img, 0.7, img2, 0.3, 0.0, img);
            cvShowImage("Laser Scanner GUI",img);
        }

        Time::delay(double(rate)/1000.0+0.005);

        //if ESC is pressed, exit.
        int keypressed = cvWaitKey(2); //wait 2ms. Lower values do not work under Linux
        keypressed &= 0xFF; //this mask is required in Linux systems
        if(keypressed == 27) exit = true;
        if(keypressed == 'w' && scale <500)
        {
            //scale+=0.001;
            scale*=1.02;
            fprintf(stderr,"scale factor is now:%.3f\n",scale);
        }
        if(keypressed == 's' && scale >15) 
        {
           //scale-=0.001;
           scale/=1.02;
            fprintf(stderr,"scale factor is now:%.3f\n",scale);
        }
        if(keypressed == 'v' ) 
        {
           verbose= (!verbose);
            if (verbose) fprintf(stderr,"verbose mode is now ON\n");
        else         fprintf(stderr,"verbose mode is now OFF\n");
        }
        if(keypressed == 'a' )
        {
            absolute= (!absolute);
            if (absolute) fprintf(stderr,"display is now in ABSOLUTE mode\n");
            else          fprintf(stderr,"display is now in RELATIVE mode\n");
        }
        if(keypressed == 'r' )
        {
            if      (rate==0)  rate = 50;
            else if (rate==50) rate = 100;
            else if (rate==100) rate = 200;
            else if (rate==200) rate = 0;
         fprintf(stderr,"refresh rate set to %d ms.\n", rate);
        }
        if(keypressed == 'c' )
        {
            compass= (!compass);
            if (compass) {fprintf(stderr,"compass is now ON\n"); }
            else         {fprintf(stderr,"compass is now OFF\n"); compass_data.zero();}
        }
        if(keypressed == 'h' || 
            keypressed == 'H')
        {
            fprintf(stderr,"\n");
            fprintf(stderr,"available commands:\n");
            fprintf(stderr,"c ...... enables/disables compass.\n");
            fprintf(stderr,"a ...... set absolute/relative mode.\n");
            fprintf(stderr,"w ...... zoom in.\n");
            fprintf(stderr,"s ...... zoom out.\n");
            fprintf(stderr,"v ...... set verbose mode on/off.\n");
            fprintf(stderr,"r ...... set refresh rate.\n");
           fprintf(stderr,"\n");
        }
    }

    laserInPort.close();
    compassInPort.close();
    laserMapInPort.close();
    navDisplayInPort.close();
    cvDestroyAllWindows();
    cvReleaseImage(&img);
}
