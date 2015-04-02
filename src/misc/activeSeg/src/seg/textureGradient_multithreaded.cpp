#include "iCub/savgol.h"
#include "iCub/textureGradient.h"
#include <pthread.h>
#define filterBin_dirLoc ""

typedef struct {
	CvMat* filterBank[24];
	CvMat* textons;
}textonData;

typedef struct{
	CvMat* filterBank;
	CvMat* grayImg_pad;
	CvMat* imgResponse_pad;
	CvMat* imgResponse;
}filterArgType;

void* threadFn_filter13x13(void* arg){
	filterArgType* filterArg = (filterArgType *)arg;
	CvMat* tmp_pad           = cvCreateMatHeader(filterArg->grayImg_pad->rows+6,filterArg->grayImg_pad->cols+6,CV_32FC1); 

	//convolve and get the response..	
	cvFilter2D(filterArg->grayImg_pad,filterArg->imgResponse_pad,filterArg->filterBank,cvPoint(6,6));
	cvGetSubRect(filterArg->imgResponse_pad,tmp_pad,cvRect(6,6,filterArg->imgResponse->cols,filterArg->imgResponse->rows));
	cvCopy(tmp_pad,filterArg->imgResponse);	
	
	cvReleaseMat(&tmp_pad);
	pthread_exit(NULL);
}

void* threadFn_filter19x19(void* arg){
	filterArgType* filterArg = (filterArgType *)arg;
	CvMat* tmp_pad           = cvCreateMatHeader(filterArg->grayImg_pad->rows+9,filterArg->grayImg_pad->cols+9,CV_32FC1); 

	//convolve and get the response..	
	cvFilter2D(filterArg->grayImg_pad,filterArg->imgResponse_pad,filterArg->filterBank,cvPoint(9,9));
	cvGetSubRect(filterArg->imgResponse_pad,tmp_pad,cvRect(9,9,filterArg->imgResponse->cols,filterArg->imgResponse->rows));
	cvCopy(tmp_pad,filterArg->imgResponse);	
	
	cvReleaseMat(&tmp_pad);
	pthread_exit(NULL);
}

void assignTextons(CvMat* tmap, CvMat* grayImg){
	// Run the filter bank on the Gray Image!!
	// 1. Read the filter Bank parameters
	// 2. convolve the image with the filter to get the response.
		char filterBankFileName[1000];
	strcpy(filterBankFileName, filterBin_dirLoc);
	strcat(filterBankFileName,"filterbank.bin");
	FILE* fp = fopen(filterBankFileName,"rb");
	//FILE* fp = fopen("filterbank.bin","rb");
	if (fp == NULL){
		fprintf(stderr,"filterbank.bin not found\n");
		exit(1);
	}
	double*    tmp1= (double*)malloc(13*13*sizeof(double));
	double*    tmp2= (double*)malloc(19*19*sizeof(double));
	CvMat*     filterBank13x13[12]; //= cvCreateMat(13,13,CV_32FC1);
	CvMat*     filterBank19x19[12]; //= cvCreateMat(19,19,CV_32FC1);

	CvMat* imgResponse[24];
	CvMat* grayImg_pad13x13[12]; //    = cvCreateMat(grayImg->rows+6,grayImg->cols+6,CV_32FC1);
	CvMat* grayImg_pad19x19[12]; //     = cvCreateMat(grayImg->rows+9,grayImg->cols+9,CV_32FC1);
	CvMat* imgResponse_pad13x13[12]; // = cvCreateMat(grayImg->rows+6,grayImg->cols+6,CV_32FC1);
	CvMat* imgResponse_pad19x19[12]; // = cvCreateMat(grayImg->rows+9,grayImg->cols+9,CV_32FC1);
	//CvMat* tmp_pad13x13         = cvCreateMatHeader(grayImg->rows+6,grayImg->cols+6,CV_32FC1); 
	//CvMat* tmp_pad19x19         = cvCreateMatHeader(grayImg->rows+9,grayImg->cols+9,CV_32FC1);
	for(int i=0; i< 24; i++){
		imgResponse[i] = cvCreateMat(grayImg->rows,grayImg->cols,CV_32FC1);
	}
	
	for(int i=0; i<12; i++){
		grayImg_pad13x13[i]     = cvCreateMat(grayImg->rows+6,grayImg->cols+6,CV_32FC1);
		imgResponse_pad13x13[i] = cvCreateMat(grayImg->rows+6,grayImg->cols+6,CV_32FC1);
		filterBank13x13[i]      = cvCreateMat(13,13,CV_32FC1);
	}
	
	
	for(int i=0; i<12; i++){
		grayImg_pad19x19[i]     = cvCreateMat(grayImg->rows+9,grayImg->cols+9,CV_32FC1);
		imgResponse_pad19x19[i] = cvCreateMat(grayImg->rows+9,grayImg->cols+9,CV_32FC1);
		filterBank19x19[i]      = cvCreateMat(19,19,CV_32FC1);
	}

	// threads to execute parallel filtering
	pthread_t pth[24];			
	filterArgType filterArg[24];
	
	for(int i=0; i< 24; i++){
		if (i < 12){
			fread(tmp1,sizeof(double),13*13,fp);
			for(int r=0; r < 13; r++){
				for(int c=0; c<13; c++){
					cvSetReal2D(filterBank13x13[i],r,c,(float)tmp1[r*13+c]);
				}
			}
			cvCopyMakeBorder(grayImg,grayImg_pad13x13[i],cvPoint(6,6),IPL_BORDER_REPLICATE);
			filterArg[i].filterBank      = filterBank13x13[i];
			filterArg[i].grayImg_pad     = grayImg_pad13x13[i];
			filterArg[i].imgResponse_pad = imgResponse_pad13x13[i];
			filterArg[i].imgResponse     = imgResponse[i];
			pthread_create(&pth[i],NULL,threadFn_filter13x13,(void*)&filterArg[i]);

		}
		else{
			fread(tmp2,sizeof(double),19*19,fp);
			for(int r=0; r < 19; r++){
				for(int c=0; c<19; c++){
					cvSetReal2D(filterBank19x19[i-12],r,c,(float)tmp2[r*19+c]);
				}
			}
			cvCopyMakeBorder(grayImg,grayImg_pad19x19[i-12],cvPoint(9,9),IPL_BORDER_REPLICATE);
			filterArg[i].filterBank      = filterBank19x19[i-12];
			filterArg[i].grayImg_pad     = grayImg_pad19x19[i-12];
			filterArg[i].imgResponse_pad = imgResponse_pad19x19[i-12];
			filterArg[i].imgResponse     = imgResponse[i];
			// step 2: convolve and get the response..
			pthread_create(&pth[i],NULL,threadFn_filter19x19,(void*)&filterArg[i]);
		}	
	}
	//Wait for threads to end!!
	for(int i=0; i< 24; i++)
		pthread_join(pth[i],NULL);
	
	//Release matrices
	for(int i=0; i< 12; i++){
		if(filterBank13x13[i] != NULL) cvReleaseMat(&filterBank13x13[i]);
		if(grayImg_pad13x13[i] != NULL) cvReleaseMat(&grayImg_pad13x13[i]);
		if(imgResponse_pad13x13[i] != NULL) cvReleaseMat(&imgResponse_pad13x13[i]);
		if(filterBank19x19[i] != NULL) cvReleaseMat(&filterBank19x19[i]);
		if(grayImg_pad19x19[i] != NULL) cvReleaseMat(&grayImg_pad19x19[i]);
		if(imgResponse_pad19x19[i] != NULL) cvReleaseMat(&imgResponse_pad19x19[i]);
	}
	fclose(fp);




	// Read the textons
	char textonsFileName[1000];
	strcpy(textonsFileName, filterBin_dirLoc);
	strcat(textonsFileName,"textons.bin");
	fp = fopen(textonsFileName,"rb");
	//fp = fopen("textons.bin","rb");
	double* tmp3 = (double*)malloc(64*24*sizeof(double));
	if (fp == NULL){
		fprintf(stderr,"textons.bin not found");
		exit(1);
	}
	fread(tmp3, sizeof(double),64*24, fp);
	CvMat*  textons = cvCreateMat(64,24,CV_32FC1);
	for(int r=0; r < 64;r++){
		for(int c=0; c < 24; c++){
			cvSetReal2D(textons,r,c,(float)tmp3[r*24+c]);
		}
	}
	fclose(fp);
	free(tmp3);

	// find the closest texton for every pixel !!
	CvMat* colFrmTextonMat = cvCreateMatHeader(64,1,CV_32FC1);
	CvMat* totSum          = cvCreateMat(64,1,CV_32FC1);
	CvMat* tmp			   = cvCreateMat(64,1,CV_32FC1);
	for(int r=0; r<grayImg->rows; r++){
		for(int c=0; c<grayImg->cols; c++){
			cvSetZero(totSum);
			for(int i=0; i<24; i++){
				//find the distance of the pixel response from all other 
				cvGetCol(textons,colFrmTextonMat, i);
				cvSubS(colFrmTextonMat,cvGet2D(imgResponse[i],r,c),tmp);
				cvPow(tmp,tmp,2.0);
				cvAdd(totSum,tmp,totSum);
			}
			// find the texton with minimum distance
			CvPoint minLoc, maxLoc;
			double minVal, maxVal;
			cvMinMaxLoc(totSum,&minVal,&maxVal,&minLoc,&maxLoc);
			cvSetReal2D(tmap,r,c,(double)minLoc.y);
		}
	}

	//free matrices
	for(int i=0; i< 24; i++)
		cvReleaseMat(&imgResponse[i]);
	cvReleaseMat(&totSum);
	cvReleaseMat(&tmp);
	cvReleaseMat(&textons);
}

//-------------------------------------------------------
//-------------------------------------------------------
CvMat** detTG(IplImage* im, int orient, double* gtheta){

	double imDiag    = sqrt((double)((im->height)*(im->height)+(im->width)*(im->width)));
	
	// RGB -> gray conversion!
	IplImage* imgGray = cvCreateImage(cvSize(im->width,im->height),im->depth,1);
	cvCvtColor(im,imgGray,CV_BGR2GRAY);
	cvConvertScale(imgGray,imgGray,1.0/255); // gray scale values between [0 and 1]
	CvMat* imgGrayMat = cvCreateMatHeader(im->height,im->width,CV_32FC1);
	imgGrayMat = cvGetMat(imgGray,imgGrayMat);
	cvReleaseImage(&imgGray);

	// Assign textons
	CvMat*   tmap = cvCreateMat(im->height,im->width,CV_32FC1);
	assignTextons(tmap, imgGrayMat);

	int     ntex  = 64;
	double sigma  = imDiag*0.02;
	int   norient =  8;
	CvMat*   tsim = cvCreateMatHeader(im->height,im->width,CV_32FC1);
	CvMat**  tg   = tgmo(*tmap, ntex, sigma, gtheta, norient, *tsim, 1); 

	cvReleaseMat(&tmap);
	return tg;
}
