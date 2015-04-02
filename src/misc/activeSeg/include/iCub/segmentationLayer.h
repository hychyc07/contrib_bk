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


//-----------------------------------------------
// Segmentation Layer
// Author: Ajay K Mishra
// Created on July 19, 2010
// Modified on Dec 01, 2010
//-----------------------------------------------

#ifndef _SEG_LAYER
#define _SEG_LAYER

#include <cv.h>
#include <highgui.h>
#include "iCub/savgol.h"
#include "iCub/sobelEdge.h"
#include "iCub/pbBoundary.h"
#include "iCub/misc.h"
#include "iCub/segFixatedRegion.h"
#include "math.h"
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/functionEncoder.h>

#define WAVE_R 10.0

using namespace iCub::ctrl;

class segLayer{
 protected:
  IplImage* img;
  IplImage* flowU;
  IplImage* flowV;
  IplImage* dispMap;
  IplImage* edgeGrad;
  IplImage* edgeOri;
  IplImage* pbBoundary;
  IplImage* mask;
  CvMemStorage* storage;
  double*   groundColorHist_3D;

  int width;
  int height;
  int numOri;
  int nL, na, nb;

  bool flowFLAG;
  bool surfaceFLAG;
  bool disparityFLAG;
  
  std::vector<CvPoint> fixPts;
  CvPoint currFixPt;

  

  public:
  //constuctor & destructors
  segLayer();
  ~segLayer();
  std::vector<CvSeq *> allContours;
  yarp::sig::Vector temp;
  // I/O
  int readImage(char* imgFileName);
  int setImage(IplImage* imgPtr);
  int getImgWidth();
  int getImgHeight();
  int copyToEdgeGrad(IplImage* tmp);
  int copyToEdgeOri(IplImage* tmp);
  int copyEdgeGrad(IplImage* tmp);
  int copyEdgeOri(IplImage* tmp);
  int copyToMask(IplImage* tmp);
  
  //Cues
  int edgeCGTG();
  int edgeBG();
  int edgeSobel();
  int setDisparity(IplImage* );
  int readDisparity(char*);
  int readFlow(char* txtFileName);
  int readFlow_flo(char* floFileName);
  int setU(IplImage* u);
  int setV(IplImage* v);
  int learnTableSurface(IplImage *tableImg, int nL, int na, int nb);
  int learnTableSurface(char *tableImgName, int nL, int na, int nb);
  
  //pb
  int generatePbBoundary();
  int generatePbBoundary(IplImage* mask);

  //fixation
  int selectFixPt_interactive();
  int selectFixPt_auto();
  int readFixPts(char* fixTxtFile);
  int assignFixPt(int x, int y);
  int getNumFixPts();
  int getFixPt(CvPoint&, int ind);
  CvPoint getFixPt(int ind);
  
  //Memory
  int allocateMemForContours();	
  int deallocateMemForContours();

  //Segmentation
  int segmentCurrFix();
  int segmentSpecificFix(int ind);
  int segmentAllFixs();
  int generateRegionMask(IplImage* mask, int contourInd);  
  int clearAllContours();
  
  //Save outputs
  int saveRegions(char* prefixTag);
  int saveEdgeMap(char* prefixTag);
  int savePbBoundary(char* prefixTag);
    
  //Display
  int displayImg(int delay);
  int displayMask(int delay);
  int displayEdge(int delay);
  int displayPbBoundary(int delay);
  int displayRegion(int ind);
  int displayFlowMag(int delay);
  int displayCurrSegs(int delay);
  int displayCurrFixPt(int delay);
  int displayAllFixPts(int delay);
  int saveFixPtsImage(char* prefixTag);
  IplImage* getSeg();
};


#endif
