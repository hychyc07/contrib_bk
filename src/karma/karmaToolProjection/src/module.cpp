/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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
#include <sstream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

#if (CV_MAJOR_VERSION<=2)
    #define KMEANS_WITH_POINTER
    #if (CV_MAJOR_VERSION==2) && (CV_MINOR_VERSION>2)
        #undef KMEANS_WITH_POINTER
    #endif
#endif

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/**********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    name=rf.find("name").asString().c_str();

    //incoming
    motionFeatures.open(("/"+name+"/motionFilter:i").c_str());   //port for incoming blobs from motionCut

    //outgoing
    toolPoint.open(("/"+name+"/target:o").c_str());             //port to send off target Points to segmentator
    imgOutPort.open(("/"+name+"/img:o").c_str());             //port to send off target Points to segmentator

    //rpc
    rpcHuman.open(("/"+name+"/human:rpc").c_str());             //rpc server to interact with the user

    motionFeatures.setManager(this);
    lineDetails = new lineData [10];
    //attach(rpcHuman);
    return true;
}
/**********************************************************/
bool Manager::interruptModule()
{
    motionFeatures.interrupt();
    toolPoint.interrupt();
    imgOutPort.interrupt();
    rpcHuman.interrupt();
    return true;
}
/**********************************************************/
bool Manager::close()
{
    delete[] lineDetails;
    motionFeatures.close();
    toolPoint.close();
    imgOutPort.close();
    rpcHuman.close();
    return true;
}
/**********************************************************/
int Manager::processHumanCmd(const Bottle &cmd, Bottle &b)
{
    int ret=Vocab::encode(cmd.get(0).asString().c_str());
    b.clear();
    if (cmd.size()>1)
    {
        if (cmd.get(1).isList())
            b=*cmd.get(1).asList();
        else
            b=cmd.tail();
    }
    return ret;
}
/**********************************************************/
bool Manager::updateModule()
{
    if (isStopping())
        return false;

    Bottle cmd, val, reply;
    rpcHuman.read(cmd, true);
    if (cmd != NULL)
    {
        int rxCmd=processHumanCmd(cmd,val);
        if (rxCmd==Vocab::encode("action"))
        {
            reply.addString("ack");
            rpcHuman.reply(reply);
        }
    }
    return true;
}
/**********************************************************/
double Manager::getPeriod()
{
    return 0.1;
}

/**********************************************************/
void Manager::processMotionPoints(Bottle &b)
{
   // fprintf(stdout,"create mat\n");

    //create MAT image
    cv::Mat imgMat(Size(320,240),CV_8UC3);
    cv::Mat imgClean(Size(320,240),CV_8UC3);
    //vector<Point> points;
    imgMat = Scalar::all(0);
    imgClean = Scalar::all(255);
   // fprintf(stdout,"filling up the data\n");
    for (int x=1; x<b.size(); x++)
    {
        Point pt;
        pt.x = b.get(x).asList()->get(0).asInt();
        pt.y = b.get(x).asList()->get(1).asInt();
        imgMat.at<cv::Vec3b>(pt.y,pt.x)[0] = 255 ;
        imgMat.at<cv::Vec3b>(pt.y,pt.x)[1] = 0 ;
        imgMat.at<cv::Vec3b>(pt.y,pt.x)[2] = 0 ;
        imgClean.at<cv::Vec3b>(pt.y,pt.x)[0] = 255 ;
        imgClean.at<cv::Vec3b>(pt.y,pt.x)[1] = 0 ;
        imgClean.at<cv::Vec3b>(pt.y,pt.x)[2] = 0 ;
    }
    //imgClean = imgMat;
    int n = 10;
    int an = n > 0 ? n : -n;
    int element_shape = MORPH_RECT;
    Mat element = getStructuringElement(element_shape, Size(an*2+1, an*2+1), Point(an, an) );
    morphologyEx(imgMat, imgMat, CV_MOP_CLOSE, element);

    Bottle data;
    //fprintf(stdout,"before process image\n");
    data = processImage(b, imgMat, imgClean, lineDetails); //image analisis and bottle cleaning
    //fprintf(stdout,"before process blobs\n");
    
    if (data.size() > 0)
        processBlobs(data, imgClean, lineDetails); // kmeans

    ImageOf<PixelRgb> outImg;// = new ImageOf<PixelRgb>;
    //fprintf(stdout,"done1 with data %d %d\n", imgClean.cols, imgClean.rows);
    outImg.resize( imgClean.cols, imgClean.rows );
    IplImage ipl_img = imgClean;
    cvCopyImage(&ipl_img, (IplImage*)outImg.getIplImage());
    imgOutPort.prepare() = outImg;
    imgOutPort.write();
    //delete[] outImg;
    //fprintf(stdout,"ok\n");
    }

/**********************************************************/
void Manager::processBlobs(Bottle &b, cv::Mat &dest, lineData *lineDetails)
{
    int sampleCount = b.size();
    int dimensions = 2;
    int clusterCount = 2;
    Mat points(sampleCount, dimensions, CV_32F);
    Mat labels;
    Mat centers(clusterCount, dimensions, points.type());
    for(int i = 0; i<sampleCount;i++)
    {
        points.at<float>(i,0) = (float) b.get(i).asList()->get(0).asInt();
        points.at<float>(i,1) = (float) b.get(i).asList()->get(1).asInt();
    }
#ifdef KMEANS_WITH_POINTER
    kmeans(points, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, &centers);
#else
    kmeans(points, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
#endif
    Point pts[10];
    for (int i = 0; i < clusterCount; i++)
    {
        int clusterIdx = labels.at<int>(i);
        Point ipt;
        ipt.x = (int) centers.at<float>(i,0);
        ipt.y = (int) centers.at<float>(i,1);

        pts[i] = ipt;
        circle( dest, ipt, 5, CV_RGB(255,255,255), CV_FILLED, CV_AA );
    }
    double gradient = 0;
    double intercept = 0;
    //fprintf(stdout,"dbg4      cluster cnt %d,   %d %d      %d %d\n", clusterCount, pts[0].x,  pts[0].y, pts[1].x,  pts[1].y);
    //check that pt0 is the lowest point
    if (pts[1].y < pts[0].y )
    {
        double pow1 = pow( fabs((double)pts[0].x - (double)pts[1].x),2);
        double pow2 = pow( fabs((double)pts[0].y - (double)pts[1].y),2);
        double lenAB = sqrt( pow1 + pow2 );
        Point endPoint;
        endPoint.x = (int)(pts[1].x + (double)(pts[1].x - pts[0].x) / lenAB * 50);
        endPoint.y = (int)(pts[1].y + (double) (pts[1].y - pts[0].y) / lenAB * 50);
        line(dest, pts[0], endPoint, Scalar(0,0,0), 2, CV_AA);
        //fprintf(stdout,"dbg4.444 %d    %d \n", endPoint.x, endPoint.y);
        gradient  = (double)( endPoint.y - pts[0].y ) / (double)( endPoint.x - pts[0].x );
        intercept = (double)( pts[0].y - (double)(pts[0].x * gradient) );
        lineDetails[1].gradient = gradient;
        lineDetails[1].intercept = intercept;
    }
    else
    {
       
        double pow1 = pow( fabs((double)pts[1].x - (double)pts[0].x), 2);
        double pow2 = pow( fabs((double)pts[1].y - (double)pts[0].y), 2);
        double lenAB = sqrt( pow1 + pow2 );
        Point endPoint;
        endPoint.x = (int)(pts[0].x + (double)(pts[0].x - pts[1].x) / lenAB * 50);
        endPoint.y = (int)(pts[0].y + (double)(pts[0].y - pts[1].y) / lenAB * 50);
        line(dest, pts[1], endPoint, Scalar(0,0,0), 2, CV_AA);
       //fprintf(stdout,"dbg4.888 %d    %d \n", endPoint.x, endPoint.y);
        
        gradient  = (double)( endPoint.y - pts[1].y) / (double)(endPoint.x - pts[1].x);
        intercept = (double)( endPoint.y - (double)(endPoint.x * gradient) );
        lineDetails[1].gradient = gradient;
        lineDetails[1].intercept = intercept;
    }
    //find
    getIntersection(dest, lineDetails);
}
/**********************************************************/
void Manager::getIntersection(cv::Mat &dest, lineData *lineDetails)
{
    int thickness = -1;
    int lineType  = 8;
    Point intersect;

    //fprintf(stdout,"line gradient = %lf and line gradient = %lf\n",lineDetails[0].gradient, lineDetails[1].gradient);
    //fprintf(stdout,"line intercept = %lf and line intercept = %lf\n",lineDetails[0].intercept, lineDetails[1].intercept);

    intersect.x =  (int)( (lineDetails[1].intercept - lineDetails[0].intercept) / (lineDetails[0].gradient-lineDetails[1].gradient));
    intersect.y =  (int)( (lineDetails[0].gradient * intersect.x) + lineDetails[0].intercept);

    //fprintf(stdout,"the point is %d %d     %d %d\n",intersect.x, intersect.y, dest.cols, dest.rows);
    if (intersect.x > 0 && intersect.y >0 && intersect.x < dest.cols && intersect.y < dest.rows)
    {
        circle( dest, intersect, dest.rows/(int)32.0, Scalar( 255, 0, 0 ), thickness, lineType );
        Bottle output;
        output.clear();
        output.addInt(intersect.x);
        output.addInt(intersect.y);
        toolPoint.write(output);
    }
}

/**********************************************************/
Bottle Manager::processImage(Bottle &b, cv::Mat &dest, cv::Mat &clean, lineData *lineDetails)
{
    Bottle botListDel, botPointsDel;
    Bottle correctList, correctElements;
    vector<vector<Point> > contours;
    cv::Mat imgGray(Size(320,240),CV_8UC1);
    cv::cvtColor( dest, imgGray, CV_RGB2GRAY);
    double gradient = 0;
    double intercept = 0;
    findContours(imgGray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);
        
        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;

        //ellipse(dest, box, Scalar(0,0,255), 1, CV_AA);
        //ellipse(dest, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);

        double area =  contourArea( Mat(contours[i]) );
        if ( area > 10 && area < 4000)
        {
            for (int x = (int)box.center.x - (int)box.size.width; x < (int)box.center.x + (int)box.size.width; x++){
                for (int y = (int)box.center.y - (int)box.size.height; y < (int)box.center.y + (int)box.size.height; y++)
                {
                    //fprintf(stdout,"x: %d y: %d    %d    %d\n",x,y,imgGray.rows-1, imgGray.cols-1);
                    if (y< imgGray.rows-1 &&  x< imgGray.cols-1 && y > 0 && x > 0)
                    {
                        if( dest.at<cv::Vec3b>(y,x)[0] == 255 )
                        {
                            dest.at<cv::Vec3b>(y,x)[0] = 0;
                            botPointsDel.clear();
                            botPointsDel.addInt(x);
                            botPointsDel.addInt(y);
                            botListDel.addList() = botPointsDel;
                        }
                    }
                }
            }
        }
        else
        {
            for (int v = 1; v<b.size(); v++)
            {
                bool found=false;
                for (int w = 0; w<botListDel.size(); w++)
                {
                    if ( ((float) b.get(v).asList()->get(0).asInt() == botListDel.get(w).asList()->get(0).asInt() ) &&
                            ((float) b.get(v).asList()->get(1).asInt() == botListDel.get(w).asList()->get(1).asInt() ) )
                    {
                        found=true;
                        break;
                    }
                }
                if(!found)
                    correctList.addList() = *b.get(v).asList();
            }
            drawContours(dest, contours, (int)i, Scalar::all(255), 1, 8);
            Point2f vtx[4];
            box.points(vtx);

            int j = 0;
            if ( vtx[1].y < vtx[3].y )
            {
                j = 1;
                fprintf(stdout,"3 < 0 %lf smaller than %lf \n", vtx[3].y, vtx[1].y);
            }
            else
            {
                j = 3;
                fprintf(stdout,"0 < 3 %lf  smaller than %lf  \n", vtx[1].y, vtx[3].y);
            }
            //line(clean, vtx[1], vtx[(1+1)%4], Scalar(255,0,0), 2, CV_AA);
            line(clean, vtx[j], vtx[(j+1)%4], Scalar(0,0,0), 2, CV_AA);

            /*for( int j = 3; j < 4; j++ )//only draw last line
            {
                line(clean, vtx[j], vtx[(j+1)%4], Scalar(0,0,0), 2, CV_AA);
                //circle( dest, intersect, dest.rows/(int)32.0, Scalar( 255, 0, 0 ), thickness, lineType );
            }*/

            gradient = ( vtx[(j+1)%4].y - vtx[j].y) / (vtx[(j+1)%4].x - vtx[j].x);
            //using the first point
            intercept = ( vtx[j].y - (vtx[j].x *gradient) );

            lineDetails[0].gradient = gradient;
            lineDetails[0].intercept = intercept;
        }
    }
    return correctList;
}
