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



/***********************************************************************************************/
/* Calculates a probabilistic boundary edge map using brightness information in the image      */
/* Author: Ajay Mishra (mishraka@gmail.com)     					       */
/* Date  : Oct 20, 2009									       */
/* Copyright 2009 University of Maryland at College Park. All rights reserved.                 */
/***********************************************************************************************/   

#include <cv.h>
#include "cxcore.h"
#include "highgui.h"
#include "iCub/savgol.h"

//------------------------------------------------------------------------------
//   Color and Gradient based edge detector
//------------------------------------------------------------------------------
void pbCG(IplImage *im, IplImage* grad, IplImage* ori){
	double BETA[] = {-2.9216, 1.5439, 2.7643};

	//1. Calculate color and texture gradient gradient
	int norient = 8;
	double *gtheta =(double *) malloc (sizeof(double)*norient);
        fprintf(stdout,"\nCalculating Brightness gradient"); fflush(stdout);
	CvMat** cg     = detCG(im, norient, gtheta);
	
	//2. Compute oriented Pb
	CvMat* b;
	CvMat* a;
	CvMat* l;
	CvMat** orientedPb = new CvMat* [norient];
	for(int i=0; i< norient; i++){
		orientedPb[i] = cvCreateMat(im->height, im->width, CV_32FC1);
		l = cg[i];
		a = cg[norient+i];
		b = cg[2*norient+i];
		for(int h=0; h < im->height; h++){
			for(int w=0; w < im->width; w++){
				double b_grad = cvGetReal2D(b,h,w);
				double a_grad = cvGetReal2D(a,h,w);
				double tmpSum = BETA[0] + a_grad*BETA[1] + b_grad*BETA[2]; 
				cvSetReal2D(orientedPb[i], h, w, 1/(1+exp(-tmpSum)) );
			}
		}
		//Release Matrices
		cvReleaseMat(&b);
		cvReleaseMat(&a);
		cvReleaseMat(&l);
	}

	//3. nonmax suppression and max over orientations
	CvMat* maxOri   = cvCreateMat(im->height,im->width,CV_32FC1);
	for(int h=0; h < im->height; h++){
		for(int w=0; w < im->width; w++){
			int maxOriInd=0;
			for(int oriInd=0; oriInd < norient; oriInd++){
				if ( cvGetReal2D(orientedPb[maxOriInd],h,w) < cvGetReal2D(orientedPb[oriInd],h,w) )
					maxOriInd = oriInd;
			}
			cvSetReal2D(maxOri,h,w,(float)maxOriInd);
		}
	}
	CvMat* pb     = cvCreateMat(im->height, im->width, CV_32FC1); cvSetZero(pb);	
	CvMat* pbi    = cvCreateMat(im->height, im->width, CV_32FC1); cvSetZero(pb);
	CvMat* theta  = cvCreateMat(im->height, im->width, CV_32FC1); cvSetZero(theta);
	double r =2.5;
	CvMat* tmp; // = cvCreateMat(im->height, im->width, CV_32FC1);
	for(int i=0; i < norient; i++){
		tmp = fitparab(*(orientedPb[i]),r,r,gtheta[i]);
		pbi = nonmaxsup(tmp,gtheta[i]);
		for(int h=0; h < pb->rows; h++){
			for(int w=0; w < pb->cols; w++){
				if( cvGet2D(maxOri,h,w).val[0] == i && cvGet2D(pb,h,w).val[0] < cvGet2D(pbi,h,w).val[0]){
					cvSetReal2D(pb,h,w,cvGetReal2D(pbi,h,w));
				}
				if (cvGet2D(maxOri,h,w).val[0] == i){
					cvSetReal2D(theta,h,w,gtheta[i]);
				}
			}	
		}
	}


	// "pb" should be between 0 and 1 
	for(int h=0; h < pb->rows; h++){
		for( int w=0; w< pb->cols; w++){
			if(cvGet2D(pb,h,w).val[0] < 0)
				cvSet2D(pb,h,w,cvScalar(0.0));
			if(cvGet2D(pb,h,w).val[0] > 1)
				cvSet2D(pb,h,w,cvScalar(1.0));
		}
	}


	// Copy cvMat into IplImages
	for(int i=0; i<im->height; i++){
		for(int j=0; j<im->width; j++){
			cvSetReal2D(grad,i,j,cvGetReal2D(pb,i,j));
			cvSetReal2D(ori,i,j,cvGetReal2D(theta,i,j));
		}
	}


	//release matrices!!
	for(int i=0; i<norient; i++)
		cvReleaseMat(&orientedPb[i]);
	
	cvReleaseMat(&pb);
	cvReleaseMat(&pbi);
	cvReleaseMat(&theta);
	cvReleaseMat(&maxOri);
	free(gtheta);

	return;

}
