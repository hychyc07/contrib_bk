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


/********************************************************/
/* Create the polar edge map  			     	*/
/* Author:   		Ajay K Mishra      		*/
/* Modified On :  	Nov 02, 2010  		   	*/
/* email id: 		mishraka@gmail.com 		*/
/********************************************************/

#include <cmath>

#include "iCub/createPolarPlot.h"

#define _PI acos(-1.0)
#define GAUSS2D(xdiff, ydiff, var) exp(-((xdiff*xdiff)/(2*var*var)+(ydiff*ydiff)/(4)))
static double maxIntensity=0; 

/*********************************/
/* Calculate circular histogram  */
/*********************************/
void calc2DHist(int origin_x, 
			 int origin_y, 
			 int edgept_x,
			 int edgept_y,
			 double grad, 
			 double ori,
			 const int width,
			 const int height, 
			 double* rThetaPlot_ptr){

	int nbins  = height;
	int nrbins = width;
	
	//each edge point contributes to all the bins
	double delx = -origin_x + edgept_x;
	double dely = -origin_y + edgept_y;
	double dist = sqrt(delx*delx+dely*dely);
	
	// center point and edgept should not be the same
	if(dist == 0 || ceil(dist) >= width-5) 
		return;
    	double theta=(atan2(-dely, delx)*(180/_PI));
    	theta = (theta<0) ? 360+theta :theta;
  
	double var  = 90.0/pow(dist, 0.7); 
    
    	int intv = var < 4 ? 4 :var;     
    	int min_bin  =  ceil(theta)- intv;
    	int max_bin  =  ceil(theta)+ intv;
    	int max_rbin =  ceil(dist) + intv;
    	int min_rbin =  ceil(dist) - intv;
    
	double theta_t;
    	double theta_a       = (_PI - ori);
    	double theta_n ;   
    
	min_rbin = (min_rbin <=1) ? 1 :min_rbin;
	max_rbin = (max_rbin >= width)? width-1 : max_rbin;
	double varAlongMajorAxis;
    	for (int rbin=min_rbin; rbin <= max_rbin; rbin++){
        	for(int bin=min_bin;bin<max_bin;bin++){
            	int binInd;
            	if(bin < 0){
                	binInd =bin+360;
            	}
            	else if(bin >= 360){
                	binInd =bin-360;
            	}
            	else{
                	binInd=bin;
            	}
            
            	if (binInd > 180){
                	theta_t   = (binInd-360)*_PI/180;
            	}
            	else{
                	theta_t   = binInd*_PI/180;
            	}            
            	theta_n       = theta_a - theta_t;
            	theta_n       = theta_n < 0 ? theta_n + _PI : theta_n;
		theta_n       = theta_n >= _PI ? theta_n - _PI : theta_n;
		double u_n    = cos(theta_n); 
            	double v_n    = sin(theta_n);
            
            	//find delX, and delY
            	double xChng			=	(rbin*cos(binInd*_PI/180))-delx;
            	double yChng			=	(rbin*sin(binInd*_PI/180))+dely;

            	double xChng_new        		=   xChng*cos(-ori)+yChng*sin(-ori);
            	double yChng_new        		=  -xChng*sin(-ori)+yChng*cos(-ori);

	    	double polar_angle_deg = u_n != 0 ? atan(v_n/u_n) : _PI/2.0 ;            
	    	varAlongMajorAxis   = 1.0*(1+fabs(polar_angle_deg)*180/_PI)/pow(dist, 0.7*fabs(sin(polar_angle_deg)));

            	*(rThetaPlot_ptr + width*binInd + rbin) += grad*GAUSS2D(xChng_new, yChng_new, varAlongMajorAxis);

		maxIntensity = (maxIntensity < *(rThetaPlot_ptr + width*binInd + rbin) ) ? *(rThetaPlot_ptr + width*binInd + rbin) : maxIntensity;
        }
    }	
}	



/******************************************************/
/* Generate polar plot using directional procession!! */
/******************************************************/
void genRThetaPlot( double* edgeGrad_ptr,
		    double* edgeOri_ptr,
                    double* rThetaPlot_ptr, 
		    const int origin_x,
		    const int origin_y,
		    const int width_polar,
		    const int height_polar,
		    const int width_cart,
		    const int height_cart){	    

	// initialize the rThetaPlot to zero
	for(int i=0; i < height_polar; i++)
		for(int j=0; j < width_polar; j++)
			*(rThetaPlot_ptr + width_polar*i + j) = 0; 

	for (int x=0; x < width_cart; x++){
		for (int y=0; y < height_cart; y++){
			if (*(edgeGrad_ptr + width_cart*y + x) > 5.0/255){
				calc2DHist(	origin_x, origin_y,
							x, y,
							*(edgeGrad_ptr + width_cart*y + x), 
							*(edgeOri_ptr + width_cart*y + x), 
							width_polar, height_polar,
							rThetaPlot_ptr);
			}
		}
	  }

	// rescale the max value of the theta plot to lie betweeen 0 and 1
	for(int i=0; i < height_polar; i++)
		for(int j=0; j < width_polar; j++)
			*(rThetaPlot_ptr + width_polar*i + j) /= maxIntensity; 
      
}


