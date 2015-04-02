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

#ifndef _POLAR_PLOT
#define _POLAR_PLOT

void genRThetaPlot( 	double* edgeGrad_ptr,
			double* edgeOri_ptr,
                    	double* rThetaPlot_ptr, 
			const int origin_x,
			const int origin_y,
			const int width_polar,
			const int height_polar,
			const int width_cart,
			const int height_cart);

void genRThetaPlot( 	double* edgeGrad_ptr,
			double* edgeOri_ptr,
                    	double* rThetaPlot_ptr, 
			double* polarOri_U_ptr, 
			double* polarOri_V_ptr, 				
			const int origin_x,
			const int origin_y,
			const int width_polar,
			const int height_polar,
			const int width_cart,
			const int height_cart);


#endif
