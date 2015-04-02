/*
 *  savgol.h
 *  
 *
 *  Created by Leonardo Claudino on 8/3/09.
 *  Modified by Ajay Mishra on Oct 10, 2009
 *  Copyright 2009 University of Maryland at College Park. All rights reserved.
 *
 */

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



#ifndef _SAVGOL_H
#define _SAVGOL_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>

CvMat *savgolFilter(CvMat &, double, double, double);
CvMat *fitparab(CvMat &, double, double, double);
CvMat *tgso (CvMat &, int, double, double, CvMat &, int);
CvMat **tgmo(CvMat &, int, double, double*&, int, CvMat &, int);
CvMat **cgmo (IplImage* , int , double* );
void  pbCGTG (IplImage *,IplImage*, IplImage*);
void  pbCGTG_NoMaxSup (IplImage *,IplImage*, IplImage*);
void  pbCG (IplImage *,IplImage*, IplImage*);
void  pbBG (IplImage *,IplImage*, IplImage*);
CvMat *colorsim(int, double);
void pbCG(IplImage *, IplImage* , IplImage* );
CvMat** detCG(IplImage*, int, double*);
CvMat** detBG(IplImage*, int, double*);
void detCGTG(IplImage* , int , double* , CvMat** , CvMat** );
CvMat* nonmaxsup(CvMat* , double );

#define max(a, b) ((a)>(b)?(a):(b))

#ifndef M_PI
#define M_PI acos(-1.0)
#endif

#endif

//make gcc happy
