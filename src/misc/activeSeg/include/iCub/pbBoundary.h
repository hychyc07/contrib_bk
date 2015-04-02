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

#ifndef _BIAS
#define _BIAS
#include<cv.h>

void calcPbBoundaryWtDisparity(IplImage* , IplImage* , IplImage* , int );

void calcPbBoundaryWtObjMask(IplImage* , IplImage* , IplImage* , IplImage*, int );

void calcPbBoundaryWtFlow(IplImage* , IplImage* , IplImage* , IplImage* , IplImage* ,int );

void calcPbBoundaryWtSurface(IplImage* , IplImage* , IplImage* , IplImage* , double* ,const int , const int , const int , const int );

void getColorHist(IplImage** , const int, double* , const int, const int, const int);
#endif
