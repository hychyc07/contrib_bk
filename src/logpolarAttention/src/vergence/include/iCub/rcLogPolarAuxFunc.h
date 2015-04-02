// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
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
#ifndef RC_Logpolar_AuxFunct_h
#define RC_Logpolar_AuxFunct_h

#include <yarp/sig/Image.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// std
#include <stdio.h>
#include <iostream>

#include <iCub/RC_DIST_FB_logpolar_mapper.h>

using namespace yarp::sig;
using namespace std;

namespace _logpolarParams {
    const int _xsize = 320;
    const int _ysize = 240;
    const int _xsizeR = 240;
    const int _ysizeR = 240;
    const int _srho = 152;
    const int _stheta = 252;
    const double _overlap = 0.00;
    const int _fmode = 2;
};

struct Image_Data {
    // Log Polar Metrics
    int Size_Rho;
    int Size_Theta;
    int Size_LP;
    int Fovea_Mode;
    double overlap;

    // Remapped Cartesian Metrics
    int Size_X_Remap;
    int Size_Y_Remap;
    int Size_Img_Remap;

    // Original Cartesian Metrics
    int Size_X_Orig;
    int Size_Y_Orig;
    int Size_Img_Orig;

    //Computed Parameters
    double scale_Orig;
    double scale_Remap;
    double LogIndex;
    double firstRing;
    double r0;
    int Size_Fovea;

};

struct shift_Struct {
    int index;
    double disp;
    double corr;

};

Image_Data SetParam(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr);
void Build_Shift_Table(Image_Data par, char *path);
int Load_Shift_Table(Image_Data par, int *shiftTab, double *stepList, char *path);
double getX(Image_Data par, int rho, int theta);
double getY(Image_Data par, int rho, int theta);
int getRho(Image_Data par, double x, double y);
int getTheta(Image_Data par, double x, double y);
void img2unpaddedVect(unsigned char *v, Image img );
void img2unpaddedVectMultiple(unsigned char *vl, Image imgl, unsigned char *vr, Image imgr);
void unpaddedVect2img(unsigned char *v, Image &img);
bool isWithin(int v,int max, int min);
double find_max_value(double *v, int size);
int find_max_index(double *v, int size);
double find_min_value(double *v, int size);
int find_min_index(double *v, int size);

inline double _max(double x, double y) {

    return (x > y) ? x : y;
}

inline double _min (double x, double y) {

    return (x < y) ? x : y;
}

#endif
