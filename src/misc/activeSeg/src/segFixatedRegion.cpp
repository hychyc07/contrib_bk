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


/***********************************/
/* Segment the fixated region      */
/* Author:   Ajay K Mishra         */
/* Date  :   Aug 10, 2009	   	*/
/* Modified on: Nov 30, 2010       */
/* email id: mishraka@gmail.com    */
/***********************************/
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <time.h>
//#include <sys/time.h>
#include "iCub/segFixatedRegion.h"
#include "iCub/createPolarPlot.h"
#include "iCub/segPolarPlot.h"

#define _PI acos(-1.0)

static int width_cart;
static int height_cart;
static int width_polar;
static int height_polar;

//---------------------------------->
void polarToCart(	const unsigned char* binaryMaskPol_in,
				unsigned char* binaryMaskCart_out, 
				const int fixPt_x_in, 
				const int fixPt_y_in,
				const int width_cart_in,
				const int height_cart_in,
				const int width_polar_in,
				const int height_polar_in)
{    
    	double theta, r;
    	for(int x=0; x < width_cart; x++){
        for(int y=0; y < height_cart; y++){
            r      = sqrt(double((x-fixPt_x_in))*double(x-fixPt_x_in)+double(y-fixPt_y_in)*double(y-fixPt_y_in));
            theta  = floor(atan2(-double(y-fixPt_y_in), double(x-fixPt_x_in))*180/_PI);
            if( theta < 0 ){
                theta = theta + 360;
            }
            else if( theta > 180){
                theta = 180;
            }
            
            *(binaryMaskCart_out+ width_cart*y + x) = *(binaryMaskPol_in + width_polar*int(theta) + int(r));
        }
    	}
    
}

//------------------------------------->
static int largestRadius(int x, int y, int width, int height){
	double max_r = 0;
 	if (max_r < (x*x+y*y)){
 		max_r = (x*x+y*y);
 	}
 	if ( max_r < ((width-x)*(width-x)+y*y) ){
 		max_r = (width-x)*(width-x)+y*y;
 	}
 	if (max_r  <  (x*x + (height-y)*(height-y)) ){
 		max_r =  (x*x + (height-y)*(height-y));
 	}
 	if (max_r < ((width-x)*(width-x) + (height-y)*(height-y)) ){
 		max_r = ((width-x)*(width-x) + (height-y)*(height-y));
 	}
 	
 	return (int)((ceil(sqrt(max_r))+1));
}


//---------------------------------------> (everything in cartesian space)
void segFixatedRegion(	double* edgeGrad_in, 
				  	double* edgeOri_in, 
					int 	fixPt_x_in,
					int 	fixPt_y_in,
					int  width_in,
					int 	height_in, 
					unsigned char* fgMap_out)
{
	width_cart 	= width_in;
	height_cart 	= height_in;
	
	//for time measurements..
//	struct timeval start_time, end_time;
	double tot_time;

    // Add image borders
    for(int i=0; i < height_cart; i++){
        for(int j=0; j < width_cart; j++){
            if(i==0 ||  i == (height_cart-1) ){
                *(edgeGrad_in + width_cart*i + j)=20.0/255.0;
                *(edgeOri_in + width_cart*i + j)=0.0;
            }
            else if(j==0 || j == width_cart-1){
                *(edgeGrad_in + width_cart*i + j)=20.0/255.0;
                *(edgeOri_in + width_cart*i + j)=_PI/2;
            }
        }
    }
    
    	// polar transformation
   	width_polar 			= largestRadius(fixPt_x_in, fixPt_y_in, width_cart, height_cart);        
    	height_polar 			= 360;
    
	double* rThetaPlot 		= (double*)malloc(width_polar*height_polar*sizeof(double));
    
    	// Step 1: r-theta plot generation!!    
//    	gettimeofday(&start_time, NULL);
    	genRThetaPlot( 	edgeGrad_in, 
					edgeOri_in,
					rThetaPlot,
					fixPt_x_in,
					fixPt_y_in,
					width_polar, 
					height_polar,
					width_cart, 
					height_cart);
    
//    	gettimeofday(&end_time, NULL);
//    	tot_time 		= (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec)/1000000.0;    	
//	    fprintf(stdout,"\n polar conversion took %f seconds", tot_time); 
    
    	// Step 2: segmentation in the polar domain    
//    	gettimeofday(&start_time, NULL);
 	    unsigned char* fgMap_polar = (unsigned char*)malloc(width_polar*height_polar*sizeof(unsigned char));

    	findTheCut(rThetaPlot, width_polar, height_polar, fgMap_polar); 
    
//    	gettimeofday(&end_time, NULL);
//    	tot_time = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec)/1000000.0 ;
//    	fprintf(stdout,"\n polar segmentation took %f seconds", tot_time); 
    
    	// Final Step: transformation from polar back to cartesian
    	polarToCart(	fgMap_polar,  
				fgMap_out,   
				fixPt_x_in,
				fixPt_y_in,
				width_cart, 
				height_cart, 
				width_polar, 
				height_polar);
    	
    	//release images..
    	free(rThetaPlot);
    	free(fgMap_polar);       
}
