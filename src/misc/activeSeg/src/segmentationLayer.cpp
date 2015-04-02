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


//segmentation Layer 
//Author: Ajay K Mishra
//Date: July 19, 2010
//Modified on: Dec 01, 2010
//-------------------------------------------------
#include "iCub/segmentationLayer.h"
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;
ofstream fout ("ciao.txt");

//--------------------------**
//Constructor
segLayer::segLayer(){
  flowFLAG = false;
  surfaceFLAG = false; 
  disparityFLAG = false;
  img = NULL;
  flowU = NULL;
  flowV = NULL;
  edgeGrad = NULL;
  edgeOri = NULL;
  pbBoundary = NULL;
  groundColorHist_3D = NULL;
  numOri = 8;
}


//---------------------------**
//Destructor
segLayer::~segLayer(){
  if(img!=NULL) cvReleaseImage(&img);
  if(edgeGrad!=NULL) cvReleaseImage(&edgeGrad);
  if(edgeOri!=NULL) cvReleaseImage(&edgeOri);
  if(flowU!=NULL) cvReleaseImage(&flowU);
  if(flowV!=NULL) cvReleaseImage(&flowV);
  if(surfaceFLAG) free(groundColorHist_3D);
  if(pbBoundary != NULL) cvReleaseImage(&pbBoundary);	
}


//---------------------------**
//Read images
int segLayer::readImage(char *imgFileName){
  IplImage* tmp = cvLoadImage(imgFileName,-1);
  if (tmp == NULL){
    fprintf(stderr, "\n unable to read %s",imgFileName);
    exit(1);
  }
  if (tmp->nChannels != 3){
    fprintf(stderr, "\nerror: program accepts color images only");
    exit(1);
  }

  img    = cvCloneImage(tmp);
  width  = img->width;
  height = img->height;

  cvReleaseImage(&tmp);
  return 0;
}

int segLayer::setImage(IplImage *tmp){
  if (tmp == NULL){
    fprintf(stderr, "\n image is null");
    exit(1);
  }
  if (tmp->nChannels != 3){
    fprintf(stderr, "\nerror: program accepts color images only");
    exit(1);
  }

  img    = tmp;
  width  = img->width;
  height = img->height;
  return 0;
}

int segLayer::getImgWidth(){
	return width;
}
int segLayer::getImgHeight(){
	return height;
}
int segLayer::copyToEdgeGrad(IplImage* tmp){   
	if (tmp == NULL ){
		return 1;
	}
	else if( tmp->width != width || tmp->height != height){
		return 1;
	}
	if (edgeGrad == NULL ){
		edgeGrad = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
	}
	cvCopy(tmp, edgeGrad);
	return 0;
}

int segLayer::copyToEdgeOri(IplImage* tmp){
	if (tmp == NULL ){
		return 1;
	}
	else if( tmp->width != width || tmp->height != height){
		return 1;
	}
	if (edgeOri == NULL){
		edgeOri = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
	}
	cvCopy(tmp, edgeOri);
	return 0;
}

int segLayer::copyEdgeGrad(IplImage* tmp){
	if (tmp == NULL ){
		return 1;
	}
	else if( tmp->width != width || tmp->height != height){
		return 1;
	}
	else{
		cvCopy(edgeGrad, tmp);
		return 0;
	}
}

int segLayer::copyEdgeOri(IplImage* tmp){
	if (tmp == NULL ){
		return 1;
	}
	else if( tmp->width != width || tmp->height != height){
		return 1;
	}
	else{
		cvCopy(edgeOri, tmp);
		return 0;
	}
}

int segLayer::copyToMask(IplImage* tmp){
	if (tmp == NULL){
		return 1;
	}
	else if( tmp->width != width || tmp->height != height || tmp->depth != IPL_DEPTH_8U){
		printf("\n in setMask: either the mask is of wrong size or it is not uchar \n");
		return 1;
	}
	else{
		mask = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
		if(tmp->nChannels == 3){
			cvCvtColor(tmp, mask, CV_BGR2GRAY);
			cvThreshold(mask, mask, 100, 255,CV_THRESH_BINARY);
		}
		else{
			cvThreshold(tmp, mask,  100, 255,CV_THRESH_BINARY);		
		}		
		return 0;
	}
}

//----------------------------------**
//Cues: color, intensity and texture
int segLayer::edgeCGTG(){
  if(edgeGrad == NULL){
    edgeGrad = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);    
  }
  if (edgeOri == NULL){
    edgeOri = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  }
  IplImage* img_float = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
  cvConvertScale(img, img_float, 1.0);
  pbCGTG(img_float, edgeGrad, edgeOri);
  cvReleaseImage(&img_float);
  return 0;
}
 

int segLayer::edgeBG(){
  if(edgeGrad == NULL){
    edgeGrad = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);    
  }
  if (edgeOri == NULL){
    edgeOri = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  } 
  IplImage* img_float = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
  cvConvertScale(img, img_float, 1.0);
  pbBG(img_float, edgeGrad, edgeOri);
  cvReleaseImage(&img_float);
  return 0;
} 

int segLayer::edgeSobel(){
  if(edgeGrad == NULL){
    edgeGrad = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);    
  }
  if (edgeOri == NULL){
    edgeOri = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  }  
  IplImage* imgGray = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  cvCvtColor(img, imgGray, CV_BGR2GRAY);
  calcSobelEdge(imgGray, edgeGrad, edgeOri);
  return 0;
}


//---------------------------**
//Cues: optic flow
//1)
int segLayer::readFlow(char *flowTxtFile){
  	flowFLAG = true; 
  	flowU = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  	flowV = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);

  	FILE* fp_flow = fopen(flowTxtFile,"r");
  	if (fp_flow == NULL){
    		fprintf(stderr,"\n error: could not load %s ", flowTxtFile);
    		exit(1);
  	}	
  	char line[100];
  	for(int y=0; y<flowU->height; y++){
    		for(int x=0; x<flowU->width; x++){
      		if(fgets(line,90,fp_flow)!=NULL){
				float u,v;
				sscanf(line,"%f%f",&u,&v);
				CV_IMAGE_ELEM(flowU,float,y,x)=u;
				CV_IMAGE_ELEM(flowV,float,y,x)=v;
      		}
      		else{
				fprintf(stderr,"\n error: %s does not contain sufficient data",flowTxtFile);
				exit(1);
      		}
    		}
  	}
  	fclose(fp_flow);
  	return 0;
}

int segLayer::readDisparity(char* dispTxtFile){
	disparityFLAG = true;
	dispMap = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 1);
	FILE* fp_disp = fopen(dispTxtFile, "r");
	if (fp_disp == NULL){
		fprintf(stderr,"\n error: could not load %s ", dispTxtFile);
		exit(1);
	}
	char line[100];	
	for(int y=0; y<dispMap->height; y++){
		for(int x=0; x<dispMap->width; x++){
			if(fgets(line,90,fp_disp) != NULL){
				float d;
				sscanf(line,"%f", &d);
				CV_IMAGE_ELEM(dispMap, float, y, x) = d;
			}
			else{
				fprintf(stderr,"\n error: %s does not contain sufficient data", dispTxtFile);
				exit(1);
			}
		}
	}	
}

//1.5)
int segLayer::readFlow_flo(char *filename){
  flowFLAG = true; 
  float TAG_FLOAT = 202021.25;
  flowFLAG = true; 
  if (flowU != NULL)
  	cvReleaseImage(&flowU);
  if (flowV != NULL)
  	cvReleaseImage(&flowV);

  flowU = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  flowV = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F,1);
  if (filename == NULL)
    fprintf(stderr,"ReadFlowFile: empty filename");
  char *dot = strrchr(filename, '.');
  if (strcmp(dot, ".flo") != 0)
    fprintf(stderr, "ReadFlowFile (%s): extension .flo expected", filename);
  FILE *stream = fopen(filename, "rb");
  if (stream == 0)
    fprintf(stderr,"ReadFlowFile: could not open %s", filename);
  int width, height;
  float tag;    
  if ((int)fread(&tag,    sizeof(float), 1, stream) != 1 ||
      (int)fread(&width,  sizeof(int),   1, stream) != 1 ||
      (int)fread(&height, sizeof(int),   1, stream) != 1)
    fprintf(stderr, "ReadFlowFile: problem reading file %s", filename);  
  if (tag != TAG_FLOAT) // simple test for correct endian-ness
    fprintf(stderr,"ReadFlowFile(%s): wrong tag (possibly due to big-endian machine?)", filename);  
  // another sanity check to see that integers were read correctly (99999 should do the trick...)
  if (width < 1 || width > 99999)
    fprintf(stderr,"ReadFlowFile(%s): illegal width %d", filename, width);
  if (height < 1 || height > 99999)
    fprintf(stderr, "ReadFlowFile(%s): illegal height %d", filename, height);
  if (height != flowU->height || width != flowU->width)
    fprintf(stderr, "image size in %s does not match the frame size", filename);
 for (int y = 0; y < height; y++){
    for(int x= 0; x < width ; x++){
      float* ptr_u = (float*)((flowU)->imageData + y*((flowU)->widthStep)) + x;
      float* ptr_v = (float*)((flowV)->imageData + y*((flowV)->widthStep)) + x;
      if ((int)fread(ptr_u, sizeof(float), 1, stream) != 1)
	fprintf(stderr,"y=%d, ReadFlowFile(%s): file is too short\n", y, filename);  
      if ((int)fread(ptr_v, sizeof(float), 1, stream) != 1)
	fprintf(stderr,"ReadFlowFile(%s): file is too short", filename);
    }
  }  
  if (fgetc(stream) != EOF)
    fprintf(stderr,"ReadFlowFile(%s): file is too long", filename);
  fclose(stream);	
  return 0;
}

int segLayer::setDisparity(IplImage* inputDispMap){
	if (	inputDispMap->nChannels == 1 && 
		inputDispMap->depth == IPL_DEPTH_32F && 
		inputDispMap->width == width && 
		inputDispMap->height==height){
		
		disparityFLAG = true;
		dispMap = cvCloneImage(inputDispMap);
		cvReleaseImage(&inputDispMap);
		return 0;
	}	
	else{
		return 1;
	}
}

//2)
int segLayer::setU(IplImage* u){
  flowFLAG = true;

  if (u == NULL){
    fprintf(stderr,"\nNo optic flow \"u\" present");
    exit(1);
  }
  if (u->width != width || u->height != height){
    fprintf(stderr,"\nDimension mismatch between flow and rgb image");
    exit(1);
  }
  flowU = u;
  return 0;
}

//3)
int segLayer::setV(IplImage* v){
  flowFLAG = true;
  if (v == NULL){
    fprintf(stderr,"\nNo optic flow \"v\" present");
    exit(1);
  }
  if (v->width != width || v->height != height){
    fprintf(stderr,"\nDimension mismatch between flow and rgb image");
    exit(1);
  }
  flowV = v;
  return 0;
}


//-----------------------------**
//Cues: table distribution
//1)
int segLayer::learnTableSurface(IplImage *tableImg, int nL, int na, int nb){
  if(tableImg->depth != IPL_DEPTH_8U){
    fprintf(stderr, "error: table image should be uint8");
    exit(1);
  }
  this->nL = nL;
  this->na = na;
  this->nb = nb;
  if (groundColorHist_3D != NULL){
    delete [] groundColorHist_3D;
  }

  groundColorHist_3D = new double[nL*na*nb];
  getColorHist(&tableImg, 1, groundColorHist_3D, nL, na, nb);   
  surfaceFLAG = true;
  return 0;
}

int segLayer::learnTableSurface(char* tableImgName, int nL, int na, int nb){
  IplImage* tableImg = cvLoadImage(tableImgName, -1);
  if(tableImg->depth != IPL_DEPTH_8U || tableImg->nChannels !=3){
    fprintf(stderr, "error: table image should be uint8 and 3-channels");
    exit(1);
  }
  this->nL = nL;
  this->na = na;
  this->nb = nb;
  groundColorHist_3D = (double*)calloc(nL*na*nb, sizeof(double));
  
  IplImage* tableImg_c1c2c3 = cvCreateImage(cvGetSize(tableImg), IPL_DEPTH_32F, 3);
  IplImage* tableImg_float  = cvCreateImage(cvGetSize(tableImg), IPL_DEPTH_32F, 3);
  cvScale(tableImg, tableImg_float);
  bgrToc1c2c3(tableImg_float, tableImg_c1c2c3);
  getColorHist(&tableImg_c1c2c3, 1, groundColorHist_3D, nL, na, nb);   

  surfaceFLAG = true;
  cvReleaseImage(&tableImg);
  cvReleaseImage(&tableImg_float);
  cvReleaseImage(&tableImg_c1c2c3);
  return 0;
}



//-----------------------------**
//pb: probabilistic boundary edge map
//1)
int segLayer::generatePbBoundary(){
  	IplImage* pbBoundaryFlow; 
  	IplImage* pbBoundarySurface;
  	IplImage* pbBoundaryDisparity;
  	IplImage* bdOwnershipFlow;
  	IplImage* bdOwnershipSurface;

  	if (flowFLAG){
    		pbBoundaryFlow  = cvCloneImage(edgeGrad);
    		bdOwnershipFlow = cvCloneImage(edgeGrad); 
		cvSetZero(bdOwnershipFlow); 
    		calcPbBoundaryWtFlow(	pbBoundaryFlow, 
							edgeOri, 
							bdOwnershipFlow, 
							flowU, 
							flowV, 
							numOri);   
  	}

  	if (surfaceFLAG){
		IplImage* img_float = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);  
    		cvScale(img, img_float);
    		IplImage* img_c1c2c3 = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
    		bgrToc1c2c3(img_float, img_c1c2c3);

    		bdOwnershipSurface   = cvCloneImage(edgeGrad); 
		cvSetZero(bdOwnershipSurface);
    		pbBoundarySurface    = cvCloneImage(edgeGrad);
    		calcPbBoundaryWtSurface( img_c1c2c3, 
							pbBoundarySurface, 
							edgeOri, 
							bdOwnershipSurface, 
							groundColorHist_3D, 
							numOri, 
							nL, na, nb);
		cvReleaseImage(&img_c1c2c3);	
    		cvReleaseImage(&img_float);
  	}

  	if (disparityFLAG){
		pbBoundaryDisparity = cvCloneImage(edgeGrad);
		calcPbBoundaryWtDisparity( 	pbBoundaryDisparity,
								edgeOri,	
								dispMap,
								numOri);
	}

  // final probabilistic boundary!
  	if (flowFLAG && surfaceFLAG){
    		pbBoundary = cvCloneImage(edgeGrad);
    		cvMax(pbBoundarySurface, pbBoundaryFlow, pbBoundary);    
    		cvReleaseImage(&pbBoundarySurface);
    		cvReleaseImage(&pbBoundaryFlow);
    		cvReleaseImage(&bdOwnershipSurface);
    		cvReleaseImage(&bdOwnershipFlow);
	}
  	else if(disparityFLAG && surfaceFLAG){
		pbBoundary = cvCloneImage(edgeGrad);
    		cvMax(pbBoundarySurface, pbBoundaryDisparity, pbBoundary);    
    		cvReleaseImage(&pbBoundarySurface);
    		cvReleaseImage(&pbBoundaryDisparity);
  	}
  
  	if (flowFLAG && !surfaceFLAG){
    		pbBoundary = cvCloneImage(edgeGrad);
    		cvCopy(pbBoundaryFlow, pbBoundary);
    		cvReleaseImage(&pbBoundaryFlow);
    		cvReleaseImage(&bdOwnershipFlow);
  	}
  	else if(disparityFLAG && !surfaceFLAG){
		pbBoundary = cvCloneImage(edgeGrad);
    		cvCopy(pbBoundaryDisparity, pbBoundary);
    		cvReleaseImage(&pbBoundaryDisparity);
	}
  
  	if (!flowFLAG && !disparityFLAG && surfaceFLAG){
    		pbBoundary = cvCloneImage(edgeGrad);
    		cvCopy(pbBoundarySurface, pbBoundary);
    		cvReleaseImage(&pbBoundarySurface);
    		cvReleaseImage(&bdOwnershipSurface);
  	}
  
  	if(!flowFLAG && !surfaceFLAG && !disparityFLAG){
    		pbBoundary = cvCloneImage(edgeGrad);
  	}
   	return 0; 
}


//2)
int segLayer::generatePbBoundary(IplImage* mask){
	if(mask == NULL || mask->nChannels > 2){ 
		fprintf(stderr,"mask is empty or is not a gray image");
		exit(1);
	}
	pbBoundary  = cvCloneImage(edgeGrad);
    	IplImage* bdOwnership_tmp = cvCloneImage(edgeGrad); 
	cvSetZero(bdOwnership_tmp); 
    	calcPbBoundaryWtObjMask(	pbBoundary, 
						edgeOri, 
						mask, 
						bdOwnership_tmp, 
						numOri);  	
	cvReleaseImage(&bdOwnership_tmp);
}



//----------------------------------**
//Fixation Strategy
//1)
int segLayer::selectFixPt_interactive(){
 	while(true){ 	
		cvShowImage("Select a fixation point on this image",img);
		cvSetMouseCallback("Select a fixation point on this image", &selectFixPt, &currFixPt);
  		cvWaitKey();	
		if (currFixPt.x < width && currFixPt.y < height){
			break;
		}
	}
	IplImage* tmpImg = cvCloneImage(img);
  	cvCircle(tmpImg, currFixPt, 5, CV_RGB(0, 255, 0), -1);
	cvShowImage("Select a fixation point on this image",tmpImg);
	cvReleaseImage(&tmpImg);

  	//add to the list
  	fixPts.push_back(currFixPt);
    return 1;
}

//3)
int segLayer::readFixPts(char *fixTxtFile){
  return 0;
}

//4)
int segLayer::assignFixPt(int x, int y){
	currFixPt.x = x;
	currFixPt.y =y;
	fixPts.push_back(currFixPt);
	return 0;
}

//5)
int segLayer::getNumFixPts(){
	return fixPts.size();
}

//6)
int segLayer::getFixPt(CvPoint& fixPt, int ind){
	if (ind < fixPts.size()){
		fixPt.x = fixPts[ind].x;
		fixPt.y = fixPts[ind].y;		
	}
	else{
		fprintf(stderr,"\n getFixPt(): error index out of the range of possible fixation points"); 
	}
	return 0;
}

CvPoint segLayer::getFixPt(int ind){
	if (ind < fixPts.size()){
		return fixPts[ind];				
	}
	else{
		fprintf(stderr,"\n getFixPt(): error index out of the range of possible fixation points"); 
	}
}

//-------------------------------**
//Segmentation
//1)
int segLayer::segmentAllFixs(){  
  if (storage == NULL){
    fprintf(stdout,"\n allocate memory before segmenting"); fflush(stdout);
    return 1;
  }

  IplImage* fgMap_uchar       = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,   1);
  cvSetZero(fgMap_uchar);
  
  double* pb_ptr 		= (double*)malloc(sizeof(double)*width*height);
  double* ori_ptr 		= (double*)malloc(sizeof(double)*width*height);
  unsigned char* fg_ptr	= (unsigned char*)malloc(sizeof(unsigned char)*width*height);

  for(int y=0; y<height; y++)
	for(int x=0; x<width; x++){
		*(pb_ptr + width*y + x)  = CV_IMAGE_ELEM(pbBoundary, float, y, x);
		*(ori_ptr + width*y + x) = CV_IMAGE_ELEM(edgeOri, float, y, x);
		*(fg_ptr + width*y + x)  = 0;
	}

  for(int i=0; i< fixPts.size(); i++){
	 if (fixPts[i].x >= width || fixPts[i].y >= height){
		fprintf(stdout,"\n %dth fixation point outside of the image!",i+1); fflush(stdout);
    		continue;	
  	}
    	printf("\n segmenting fixNo.%d",i+1);
    	segFixatedRegion(	pb_ptr, 
				 	ori_ptr,
				 	fixPts[i].x,
				 	fixPts[i].y,
					width,
					height,
					fg_ptr);
	 for(int y=0; y<height; y++)
		for(int x=0; x<width; x++){
   			CV_IMAGE_ELEM(fgMap_uchar, unsigned char, y, x) = *(fg_ptr + width*y + x);
		}
    
    //store the contour!
    int countWtColor   = cvCountNonZero(fgMap_uchar);

    // extract the exterior contour and store it!
    CvSeq* first_contour;
    int Nc = cvFindContours(
			    fgMap_uchar,
			    storage,
			    &first_contour,
			    sizeof(CvContour),
			    CV_RETR_EXTERNAL);
    if (Nc == 1){
      allContours.push_back(first_contour);
    }
  }

  //release
  cvReleaseImage(&fgMap_uchar);  
  free(pb_ptr);
  free(ori_ptr);
  free(fg_ptr);
  return 0;  
}



//2)
int segLayer::segmentCurrFix(){  


  if (storage == NULL){
    fprintf(stdout,"\n allocate memory before segmenting"); fflush(stdout);
    return 1;
  }
  if (currFixPt.x >= width || currFixPt.y >= height){
	fprintf(stdout,"\n the fixation point outside of the image!"); fflush(stdout);
    	return 1;	
  }
  IplImage* fgMap_uchar       = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  cvSetZero(fgMap_uchar);

  double* pb_ptr 		= (double*)malloc(sizeof(double)*width*height);
  double* ori_ptr 		= (double*)malloc(sizeof(double)*width*height);
  unsigned char* fg_ptr	= (unsigned char*)malloc(sizeof(unsigned char)*width*height);

  for(int y=0; y<height; y++)
	for(int x=0; x<width; x++){
		*(pb_ptr + width*y + x)  = CV_IMAGE_ELEM(pbBoundary, float, y, x);
		*(ori_ptr + width*y + x) = CV_IMAGE_ELEM(edgeOri, float, y, x);
		*(fg_ptr + width*y + x)  = 0;
	}

  printf("\n segmenting for the fixation point = (%d, %d)\n",currFixPt.x, currFixPt.y);
  segFixatedRegion(	pb_ptr, 
				ori_ptr,
				currFixPt.x,
				currFixPt.y,
				width,
				height,
				fg_ptr); 
	
  for(int y=0; y<height; y++)
	for(int x=0; x<width; x++){
   		CV_IMAGE_ELEM(fgMap_uchar, unsigned char, y, x) = *(fg_ptr + width*y + x);
	}

  //store the contour!
  int countWtColor   = cvCountNonZero(fgMap_uchar);

  // extract the exterior contour and store it!
  CvSeq* first_contour;
  cvClearMemStorage(storage);
  
  int Nc = cvFindContours(
			  fgMap_uchar,
			  storage,
			  &first_contour,
			  sizeof(CvContour),
			  CV_RETR_EXTERNAL);

    if (Nc == 1){
        allContours.push_back(first_contour);
    }

    fprintf (stdout, "\n\n####### %d \n\n", first_contour->total );

    CvSeqReader readerSeq;
    cvStartReadSeq(first_contour, &readerSeq);

    IplImage *dummy = cvCreateImage(cvSize(360, 360), IPL_DEPTH_8U, 3);
    cvZero(dummy);
    CvBox2D32f box;
    CvPoint cog;
    float *line = new float[4];
    float linearity=0.0f;
    CvPoint *prev = NULL;
    
    std::vector<CvPoint> allPoints;

    std::vector<double> angles;
    std::vector<double> radii;
    
    for (int x=0; x< first_contour->total; x++){

        box = cvMinAreaRect2(first_contour, storage); 
        CvPoint center = *((CvPoint*) readerSeq.ptr);
        CV_NEXT_SEQ_ELEM( readerSeq.seq->elem_size, readerSeq );
        cog.x = cvRound(box.center.x);
	    cog.y = cvRound(box.center.y);
        double ang = atan2( (double)center.x- cog.x, (double)center.y- cog.y )*180/CV_PI;
        if (ang < 0)
            ang = 180-ang;
    
        unsigned int i=0;
    
        for (i=0; i<allPoints.size(); i++){
            if (ang < allPoints[i].x)
                break;
        }
        double rad = sqrt((double)(center.x - cog.x)*(center.x - cog.x) + (center.y - cog.y) * (center.y - cog.y));
        CvPoint polar =  cvPoint( ang , dummy->height - 10 * rad);
        allPoints.insert(allPoints.begin() + i, polar);

        angles.insert(angles.begin()+i, ang);
        radii.insert(radii.begin()+i, rad);
    }

    for (unsigned int i=0; i<allPoints.size(); i++){

        if (prev!=NULL)
        {
            cvLine(dummy,allPoints[i],*prev,cvScalar(0,255,100),1);
            delete prev;        
        }
        prev= new CvPoint;
        prev->x=allPoints[i].x;
        prev->y=allPoints[i].y;

        cvCircle(dummy, allPoints[i], 1, cvScalar( 0,0,255), 1);
    }

    int bin = 200;

    for (int x=0; x<bin; x++){
        int i = 0;

        double theta = x*360.0/bin;

        for (i=0; i<angles.size(); i++){
            if (angles[i] >= theta)
                break;
        }
        
        double x1, x2, y1, y2;
        if ( i==0 || i== angles.size() ){
            x1 = angles[angles.size()-1];
            y1 = radii[radii.size()-1];
            x2 = angles[0];            
            y2 = radii[0];
        }
        else{
            
            x1 = angles[i-1];
            y1 = radii[i-1];            
            x2 = angles[i];
            y2 = radii[i];
        }
        temp.push_back( (theta-x1)/(x2-x1) * (y2-y1) + y1 );
        cvCircle(dummy, cvPoint(theta,dummy->height - 10 * temp[x] ), 2, cvScalar( 255,0,0), 2);
    }

    if(prev==NULL)
        delete prev;   

    //--------WAVED
    waveletEncoder enc;
    yarp::sig::Vector wavedCoeffs = enc.encode(temp, WAVE_R);

    for (int x=0; x<bin; x++){
        double theta = x*360.0/bin;
        double raw = enc.decode(wavedCoeffs, WAVE_R, theta/360);
        cvCircle(dummy, cvPoint(theta,dummy->height - 10 * raw ), 2, cvScalar( 0,255,255), 2);
    }

    fprintf(stdout,"%s \n\n", wavedCoeffs.toString().c_str());
    temp.clear();

    fout << wavedCoeffs.toString().c_str() << endl;

    cvSaveImage("dummy.jpg", dummy);
    //release
    cvReleaseImage(&dummy);
    cvReleaseImage(&fgMap_uchar);
    free(pb_ptr);
    free(ori_ptr);
    free(fg_ptr);
    return 0;  
}

//3)
int segLayer::allocateMemForContours(){
  storage = cvCreateMemStorage();
  return 1;
}
//4)
int segLayer::deallocateMemForContours(){
  if (storage != NULL)
  	cvReleaseMemStorage(&storage);
  return 1;
}
//5)
int segLayer::clearAllContours(){
	allContours.clear();
    return 1;
}


//--------------------------------**
// Saving regions
//1)
int segLayer::saveRegions(char* prefixTag){
	IplImage* tmp = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
	char segName[100];
     int numContours = allContours.size();
	for(int i=0; i<numContours; i++){
		cvSetZero(tmp);
		cvDrawContours(
		 			tmp,
		 			allContours[i],
		 			CV_RGB(255,255,255),
			 		CV_RGB(255,255,255),
		 			-1,
		  			-1);
		sprintf(segName,"%s_region_%d.png",prefixTag, i+1);
		cvSaveImage(segName, tmp);		
	}
	cvReleaseImage(&tmp);
	return 0;
}

//2)
int segLayer::saveEdgeMap(char *prefixTag){
  IplImage* tmpImg = cvCreateImage(cvGetSize(edgeGrad), IPL_DEPTH_8U, 1);
  cvScale(edgeGrad, tmpImg, 255);	
  if (prefixTag == NULL){
    char edgeMapName_noprefix[100];
    sprintf(edgeMapName_noprefix,"gradientMap.png");
    cvSaveImage(edgeMapName_noprefix, tmpImg);
  }
  else{
    char edgeMapName[100];
    sprintf(edgeMapName,"%s_gradientMap.png",prefixTag);
    cvSaveImage(edgeMapName, tmpImg);    
  }
  cvReleaseImage(&tmpImg);
  return 0;  
}

//3)
int segLayer::savePbBoundary(char *prefixTag){
  IplImage* tmpImg = cvCreateImage(cvGetSize(pbBoundary), IPL_DEPTH_8U, 1);
  cvScale(pbBoundary, tmpImg, 255);	
  if (prefixTag == NULL){
    char pbMapName_noprefix[100];
    sprintf(pbMapName_noprefix,"pbBoundary.png");
    cvSaveImage(pbMapName_noprefix, tmpImg);
  }
  else{
    char pbMapName[100];
    sprintf(pbMapName,"%s_pbBoundary.png",prefixTag);
    cvSaveImage(pbMapName, tmpImg);    
  }
  cvReleaseImage(&tmpImg);
  return 0;  
}


//-----------------------------**
//Display:
//1) 
int segLayer::displayImg(int delay){ 
  if (img == NULL){
    fprintf(stdout, "\nWarning: no image assigned yet"); fflush(stdout);
    return 1;
  }
  cvShowImage("RGB image",img);
  cvWaitKey(delay);
  return 0;
}

int segLayer::displayMask(int delay){ 
  if (img == NULL){
    fprintf(stdout, "\nWarning: no mask assigned yet"); fflush(stdout);
    return 1;
  }
  cvShowImage("object mask",mask);
  cvWaitKey(delay);
  return 0;
}

//2)
int segLayer::displayEdge(int delay){
  if (edgeGrad == NULL){
    fprintf(stdout, "\nWarning: no edge assigned yet"); fflush(stdout);
    return 1;
  }
  cvShowImage("edge", edgeGrad);
  cvWaitKey(delay);
  return 0;
}
//3)
int segLayer::displayPbBoundary(int delay){
  if (pbBoundary == NULL){
    fprintf(stdout, "\nWarning: probabilistic boundary not calculated yet"); fflush(stdout);
    return 1;
  }
  cvShowImage("final probabilistic boundary", pbBoundary);
  cvWaitKey(delay);
  return 0;  
}
//4)
int segLayer::displayFlowMag(int delay){
  IplImage* flowU_tmp = cvCloneImage(flowU);
  IplImage* flowV_tmp = cvCloneImage(flowV);
  IplImage* flowMag   = cvCloneImage(flowU); 
  IplImage* flowMag_uchar = cvCreateImage(cvGetSize(flowU), IPL_DEPTH_8U, 1);
  cvSetZero(flowMag);

  cvPow(flowU_tmp, flowU_tmp, 2);
  cvPow(flowV_tmp, flowV_tmp, 2);
  cvAdd(flowU_tmp, flowV_tmp, flowMag);
  cvPow(flowMag,flowMag, 0.5);

  double minVal, maxVal;
  cvMinMaxLoc(flowMag, &minVal, &maxVal);
  cvScale(flowMag, flowMag_uchar, 255/(maxVal-minVal), -minVal);
  cvShowImage("flow magnitude", flowMag_uchar);
  cvWaitKey(delay);
  
  cvReleaseImage(&flowU_tmp);
  cvReleaseImage(&flowV_tmp);
  cvReleaseImage(&flowMag);
  cvReleaseImage(&flowMag_uchar);
  
  return 0;
}
//5)
int segLayer::displayCurrSegs(int delay){
  IplImage* tmp      = cvCloneImage(img);
  int len = allContours.size();
  cvDrawContours(
		 tmp,
		 allContours[len-1],
		 CV_RGB(0,255,0),
		 CV_RGB(255,255,255),
		 -1,
		  5);
  cvShowImage("segmented object mask", tmp);
  cvWaitKey(delay);
  cvReleaseImage(&tmp);
  return 0;  
}
//5.5
IplImage* segLayer::getSeg(){
  IplImage* tmp      = cvCloneImage(img);
  tmp      = cvCloneImage(img);
  int len = allContours.size();
  cvDrawContours(
		 tmp,
		 allContours[len-1],
		 CV_RGB(0,255,0),
		 CV_RGB(255,255,255),
		 -1,
		  2);
  
  return tmp;  
}

//6)
int segLayer::displayCurrFixPt(int delay){
  IplImage* tmpImg = cvCloneImage(img);
  cvCircle(tmpImg, currFixPt, 5, CV_RGB(0, 255, 0), -1);
  cvShowImage("image with fixation", tmpImg);
  cvWaitKey(delay);
  cvReleaseImage(&tmpImg);
  return 0;
}

//7)
int segLayer::displayAllFixPts(int delay){
  IplImage* tmpImg = cvCloneImage(img);  
  for(int i=0; i<fixPts.size(); i++){
    cvCircle(tmpImg, fixPts[i], 5, CV_RGB(0, 255, 0), -1);
  }
  cvShowImage("image with fixations", tmpImg);
  cvWaitKey(delay);
  cvReleaseImage(&tmpImg);
  return 0;
}
//8)
int segLayer::saveFixPtsImage(char* prefixTag){
	fprintf(stdout, "\n inside saveFixPtsImage function : number of fixation points= %d", (int)fixPts.size()); fflush(stdout);
 	IplImage* tmpImg = cvCloneImage(img);  
  	for(int i=0; i<fixPts.size(); i++){
    	cvCircle(tmpImg, fixPts[i], 5, CV_RGB(0, 255, 0), -1);
  	}
  	if (prefixTag == NULL){
    	char pbMapName_noprefix[100];
    	sprintf(pbMapName_noprefix,"fixPts.png");
    	cvSaveImage(pbMapName_noprefix, tmpImg);
  	}
  	else{
    	char pbMapName[100];
    	sprintf(pbMapName,"%s_fixPts.png",prefixTag);
    	cvSaveImage(pbMapName, tmpImg);    
  	}
  	//save image!
  	cvReleaseImage(&tmpImg);
  	return 0;  
}
