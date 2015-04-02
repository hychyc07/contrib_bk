/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff & Ajay Mishra
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


#include<cstdio>
#include<cv.h>
#include<highgui.h>
#include<math.h>

#ifndef M_PI
#define M_PI acos(-1.0)
#endif

void calcSobelEdge(IplImage* imgGray_uchar, IplImage* edgeGrad_float, IplImage* edgeOri_float){
  IplImage* tmp = cvCreateImage(cvGetSize(imgGray_uchar),IPL_DEPTH_16S,1);
  IplImage* gx  = cvCreateImage(cvGetSize(imgGray_uchar),IPL_DEPTH_32F,1);
  IplImage* gy  = cvCreateImage(cvGetSize(imgGray_uchar),IPL_DEPTH_32F,1);
  
  // calculate canny Edge and remove where there is no edge
  IplImage* cannyEdge_uchar = cvCreateImage(cvGetSize(edgeGrad_float),IPL_DEPTH_8U,1);
  cvCanny(imgGray_uchar, cannyEdge_uchar, 30, 60);
 
  //cvNamedWindow("canny",1); cvShowImage("canny", cannyEdge_uchar); cvWaitKey(-1);

  cvSmooth(imgGray_uchar, imgGray_uchar, CV_GAUSSIAN, 5, 5, 0.5, 0);
  cvSobel(imgGray_uchar, tmp, 1, 0, CV_SCHARR);
  cvScale(tmp, gx);	
  cvSobel(imgGray_uchar, tmp, 0, 1, CV_SCHARR);
  cvScale(tmp, gy);
  
  for(int i=0; i<imgGray_uchar->height; i++){
    for(int j=0; j<imgGray_uchar->width; j++){
      if (CV_IMAGE_ELEM(cannyEdge_uchar, uchar, i, j)!=0){
	float dx  = CV_IMAGE_ELEM(gx,float,i,j);
	float dy  = CV_IMAGE_ELEM(gy,float,i,j);
	CV_IMAGE_ELEM(edgeGrad_float,float,i,j) = sqrt(dx*dx+ dy*dy);
	
	float theta;
	if (dy !=0){
	  theta = atan(-dx/dy);
	}
	else{
	  theta = atan(-dx/0.00000000001);
	}
	CV_IMAGE_ELEM(edgeOri_float,float,i,j)  = theta < 0 ? M_PI + theta: theta;
      }
    }
  }
  
  double minVal, maxVal;
  cvMinMaxLoc(edgeGrad_float,&minVal,&maxVal);
  cvScale(edgeGrad_float, edgeGrad_float, 1.0/maxVal);
  
  cvReleaseImage(&tmp);
  cvReleaseImage(&gx);
  cvReleaseImage(&gy);
  cvReleaseImage(&cannyEdge_uchar);
}
