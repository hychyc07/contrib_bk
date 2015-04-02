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


//----
#include "iCub/misc.h" 
#include <cv.h>
#include <math.h>
#include <cstdio>
void selectFixPt(int event, int x, int y, int flags, void* fixPt){
  CvPoint* fixPt_local = (CvPoint *)fixPt;
  
  if(event == CV_EVENT_LBUTTONDOWN){
    fixPt_local->x  = x;
    fixPt_local->y  = y;
  }
  
}

void bgrToc1c2c3(IplImage* bgrImg_float, IplImage* c1c2c3Img_float){
	for(int y=0; y < bgrImg_float->height; y++){
		for(int x=0; x <bgrImg_float->width; x++){
			float B = CV_IMAGE_ELEM(bgrImg_float, float, y, 3*x);
			float G = CV_IMAGE_ELEM(bgrImg_float, float, y, 3*x+1);
			float R = CV_IMAGE_ELEM(bgrImg_float, float, y, 3*x+2);
			CV_IMAGE_ELEM(c1c2c3Img_float, float, y, 3*x)   = atan(R/(std::max(G,B)+0.000000000001));
			CV_IMAGE_ELEM(c1c2c3Img_float, float, y, 3*x+1) = atan(G/(std::max(R,B)+0.000000000001));
			CV_IMAGE_ELEM(c1c2c3Img_float, float, y, 3*x+2) = atan(B/(std::max(G,R)+0.000000000001));
		}
	}

} 
