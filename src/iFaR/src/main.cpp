/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
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
  

/*
 * Audit Trail
 * -----------
 * 26/07/07  First version validated   DV
 */ 

#include <include/iCub/iFaR.h> 
#include "include/iCub/convolve.h"
#include "include/iCub/centerSurround.h"

static float Gab0[25] = {
 0.001021835456597,   0.001486761816501,   0.001684721852447,   0.001486761816501,   0.001021835456597,
   0.000743409214995,   0.001081654025366,   0.001225674585597,   0.001081654025366,   0.000743409214995,
                   0,                   0,                   0,                   0,                   0,
  -0.000743409214995,  -0.001081654025366,  -0.001225674585597,  -0.001081654025366,  -0.000743409214995,
  -0.001021835456597,  -0.001486761816501,  -0.001684721852447,  -0.001486761816501,  -0.001021835456597
   };
   
 static float Gab45[25] = {
 0.000000000000000,   0.000525673033146,   0.001191308487465,   0.001576939035873,   0.001445020194082,
  -0.000525673033146,   0.000000000000000,   0.000866688311182,   0.001529670377021,   0.001576939035873,
  -0.001191308487465,  -0.000866688311182,                   0,   0.000866688311182,   0.001191308487465,
  -0.001576939035873,  -0.001529670377021,  -0.000866688311182,  -0.000000000000000,   0.000525673033146,
  -0.001445020194082,  -0.001576939035873,  -0.001191308487465,  -0.000525673033146,  -0.000000000000000
 };
 static float Gab90[25] = {
-0.001021835456597,  -0.000743409214995,   0.000000000000000,   0.000743409214995,   0.001021835456597,
  -0.001486761816501,  -0.001081654025366 ,  0.000000000000000,   0.001081654025366,   0.001486761816501,
  -0.001684721852447,  -0.001225674585597,                   0,   0.001225674585597,   0.001684721852447,
  -0.001486761816501,  -0.001081654025366,  -0.000000000000000,   0.001081654025366,   0.001486761816501,
  -0.001021835456597,  -0.000743409214995,  -0.000000000000000,   0.000743409214995,   0.001021835456597
   };
   
   
static float GabM45[25] = {
0.001445020194082,   0.001576939035873,   0.001191308487465,   0.000525673033146,   0.000000000000000,
   0.001576939035873,   0.001529670377021,   0.000866688311182,   0.000000000000000,  -0.000525673033146,
   0.001191308487465,   0.000866688311182,                   0,  -0.000866688311182,  -0.001191308487465,
   0.000525673033146,  -0.000000000000000,  -0.000866688311182,  -0.001529670377021,  -0.001576939035873,
  -0.000000000000000,  -0.000525673033146,  -0.001191308487465,  -0.001576939035873,  -0.001445020194082
};

// Gaussian matrix is always separable, sigma here is 1
static float G5[5] = { -0.0545f,
                       -0.2442f,
                       -0.4026f,
                       -0.2442f,
                       -0.0545f
                     };
// Gaussian matrix is always separable, sigma here is 3 times that of positive Gaussian
static float GN7[7] = {      -0.1063f,
                           -0.1403f,
                           -0.1658f,
                           -0.1752f,
                           -0.1658f,
                           -0.1403f,
                           -0.1063f
                    };


int main(int argc, char * argv[])
{
   /* initialize yarp network */ 
  Network yarp;   
  /* create your module */   
  IfarModule myModule; 
  /* prepare and configure the resource finder */
  ResourceFinder rf;
  rf.setVerbose(true);
  //  rf.configure("ICUB_ROOT", argc, argv);
  rf.setDefaultConfigFile("iFar.ini");
  rf.setDefaultContext("/default/conf");   //overridden by --context parameter
  rf.configure("ICUB_ROOT", argc, argv);
  /* run the module: runModule() calls configure first and, if successful, it then runs */
  myModule.runModule(rf);
  return 0;
}
 

Ifar::Ifar(int ratePeriod, BufferedPort<ImageOf<PixelMono> > *imageInL, BufferedPort<ImageOf<PixelMono> > *imageInR, BufferedPort<Bottle> *attentionOut, yarp::os::BufferedPort<ImageOf<PixelBgr> > *salMapPort):RateThread(ratePeriod)
{
	imagePortInL = imageInL;
	imagePortInR = imageInR;
	attentionPortOut = attentionOut;
	salienceMapOutPort = salMapPort;
}

bool Ifar::threadInit() 
{

  imageL = new ImageOf<PixelMono>;
  imageL->resize(320, 240);
  imageR = new ImageOf<PixelMono>;
  imageR->resize(320, 240);
  salienceMap = new ImageOf<PixelBgr>;
  salienceMap->resize(320, 240);
  leftOut8u = cvCreateImage(cvSize(320,240),8,1);
  rightOut8u = cvCreateImage(cvSize(320,240),8,1);

  filterA = new CenterSurround(320, 240, 0.2);
  
  leftFilter1 = new MonoFilter(imageL, leftOut8u, filterA);
  rightFilter1 = new MonoFilter(imageR, rightOut8u, filterA);
  binocFilter1 = new BinocFilter(leftOut8u, rightOut8u, imageBoth);
  mySalienceMap = new SalienceMap(imageBoth, attentionX, attentionY);
  attentionOutBottle = new Bottle();
  
  gaborPosHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
  gaborPosVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
  gaborNegHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
  gaborNegVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);
  
  gaborFiveByFive[0] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab0,1.0,0);
  gaborFiveByFive[1] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab45,1.0,0);
  gaborFiveByFive[2] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab90,1.0,0);
  gaborFiveByFive[3] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,GabM45,1.0,0);
  
  
  return true;
}

void Ifar::threadRelease()
{
  // TODO free memory allocated in thread init
  delete leftFilter1;
  delete rightFilter1;
  delete binocFilter1;
  delete filterA;
  delete mySalienceMap;
  delete attentionOutBottle;
  delete salienceMapOutPort;
  // delete convolution classes 
  delete gaborPosHorConvolution;
  delete gaborPosVerConvolution;
  delete gaborNegHorConvolution;
  delete gaborNegVerConvolution;
  delete[] gaborFiveByFive;
  cvReleaseImage(&leftOut8u );
  cvReleaseImage(&rightOut8u );
  delete imageL;
  delete imageR; 
}

void Ifar::run()
{
	imageL = NULL; imageR = NULL;
	do {
	  imageL = imagePortInL->read(true);
	  imageR = imagePortInR->read(true);
	} 
	while (imageL == NULL || imageR == NULL);
	printf("out of the while cycle \n");
	leftFilter1->filter();
	printf("after filter left \n");
	rightFilter1->filter();
	printf("after filter right \n");
	//binocFilter1->filter();
	//printf("after filter binoc \n");
	
	//example of 1 dimension convolution over redPlane. Result is stored in tempMonoImage
	// where tmpMonoLPImage      = new ImageOf<PixelMono>;
        // the same process can be performed using either ImageOf<PixelMono16> or ImageOf<PixelFloat>
	//gaborNegHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
	
	//sending bottle out
	if(attentionPortOut->getOutputCount()) {
	  mySalienceMap->setROI();
	  attentionOutBottle->clear();
	  attentionOutBottle->addInt(*attentionX);
	  attentionOutBottle->addInt(*attentionY);
	  *attentionOutBottle = attentionPortOut->prepare();
	  attentionPortOut->write();
	}
	//sending imageBoth out
	if(salienceMapOutPort->getOutputCount()) {
	  printf("sending image \n");
	  //salienceMap->wrapIplImage(imageBoth);

	  IplImage *cvImage = cvCreateImage(cvSize(320,240), 
					    IPL_DEPTH_8U, 3 );
	  cvCvtColor(imageL->getIplImage(), cvImage, CV_GRAY2BGR);
	  salienceMap->wrapIplImage(cvImage);
	  printf("after wrap \n");
	  salienceMapOutPort->prepare()= *salienceMap;
	 
	  printf("after prepare \n");
	  
	  salienceMapOutPort->write();
	}
} 

MonoFilter::MonoFilter(ImageOf<PixelMono> *imageInput, IplImage *imageOutput, CenterSurround *csFilter) /*:convolve*/
{
	imageIn = imageInput;
	imageIPLOut = imageOutput;
	myFilter = csFilter;
}

void MonoFilter::filter()
{
  // override this function in subclasses to implement specific filters
  //PERFORM FILTERING HERE
  printf("MonoFilter::filter \n");
  imageIPLIn = (IplImage*) imageIn->getIplImage();
  printf("getting the IplImage %d %d %d  \n", imageIPLIn->width, imageIPLIn->height, imageIPLIn->depth );
  myFilter->proc_im_8u(imageIPLIn, imageIPLOut);
  printf("filtering just happened \n");
}

SalienceMap::SalienceMap(IplImage *imageInput, int *attX, int *attY)
{
	imageIn = imageInput;
	attentionX = attX;
	attentionY = attY;
}

void SalienceMap::setROI()
{
	// set current region of interest in retinal coordinates
	//x = ...
	//y = ...

	int i,j,k,l, winnerX=0, winnerY = 0;
	float currentPixelScore=0, maxScore=0, tempFloat, tempFloat2, tempFloat3, tempFloat4; 

	for(i=1;i<imageIn->width-1;i++){
		for(j=1;j<imageIn->height-1;j++){
			for(k=0;k<2;k++){
				for(l=0;l<2;l++){
					tempFloat = cvGetReal2D(imageIn,i+k,j+l);
					tempFloat2 = cvGetReal2D(imageIn,i-k,j-l);
					tempFloat3 = cvGetReal2D(imageIn,i+k,j-l);
					tempFloat4 = cvGetReal2D(imageIn,i-k,j+l);
					currentPixelScore += (tempFloat+tempFloat2+tempFloat3+tempFloat4); 
				}
			}
			if(currentPixelScore > maxScore){
				winnerX = i;
				winnerY = j;
			}
		}
	}
	*attentionX = winnerX; *attentionY = winnerY; //
	
}

BinocFilter::BinocFilter(IplImage *imageInputL, IplImage *imageInputR, IplImage *imageOutput)
{  
	imageL = imageInputL;
	imageR = imageInputR;
	imageBoth = imageOutput;
}

void BinocFilter::filter()
{
	//Perform filtering here
	// this version just multiplies the the two inputs pixelwise
	cvConvertScale(imageL,matL,0.003922,0); //0.003922 = 1/255
	cvConvertScale(imageR,matR,0.003922,0);
	cvMul(matL, matR, filtBoth);
	cvPow(filtBoth, filtBoth2, 4);// just raise to pow 4 to exaggerate distribution 
	cvConvertScale(filtBoth2, imageBoth, 255,0);
}

bool IfarModule::configure(yarp::os::ResourceFinder &rf)
{    
   moduleName            = rf.check("name", 
                           Value("ifarModule"), 
                           "module name (string)").asString(); 
	setName(moduleName.c_str());

   robotName             = rf.check("robot", 
                           Value("icubSIM"), 
                           "Robot name (string)").asString();
 
   robotPortName         = "/" + robotName + "/head";
  
   inputPortNameL         = "/";
   inputPortNameL        += getName(
                           rf.check("myInputPortL", 
                           Value("ifar/imageL:i"),
                           "Input left image port (string)").asString()
                           );
   inputPortNameR         = "/";
   inputPortNameR        += getName(
                           rf.check("myInputPortR", 
                           Value("ifar/imageR:i"),
                           "Input right image port (string)").asString()
                           );

   salMapPortName        = "/";
   salMapPortName       += getName(
                           rf.check("mySalienceMapPort", 
                           Value("/ifar/salMap:o"),
                           "Output image salience port (string)").asString()
                           );

   outputPortName        = "/";
   outputPortName       += getName(
                           rf.check("myOutputPort", 
                           Value("/ifar/attention:o"),
                           "Output retinal attention port (string)").asString()
                           );

   if (!attentionOut.open(outputPortName.c_str())) {
      cout << getName() << ": unable to open port " << outputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!salienceMapOut.open(salMapPortName.c_str())) {
      cout << getName() << ": unable to open port " << outputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!imageInL.open(inputPortNameL.c_str())) {
      cout << getName() << ": unable to open port " << inputPortNameL << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!imageInR.open(inputPortNameR.c_str())) {
      cout << getName() << ": unable to open port " << inputPortNameR << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   handlerPortName =  "/";
   handlerPortName += getName();         // use getName() rather than a literal 

   if (!handlerPort.open(handlerPortName.c_str())) {           
      cout << getName() << ": Unable to open port " << handlerPortName << endl;  
      return false;
   } 

   attach(handlerPort);                  // attach to port
  // attachTerminal();                     // attach to terminal


   /* create the thread and pass pointers to the module parameters */
   	myIfarThread = new Ifar(10,&imageInL, &imageInR, &attentionOut, &salienceMapOut);
   /* now start the thread to do the work */

 	myIfarThread->start();

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
} 


bool IfarModule::interruptModule()
{
   imageInL.interrupt();
   imageInR.interrupt();
   attentionOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool IfarModule::close()
{
   imageInL.close();
   imageInR.close();
   attentionOut.close();
   handlerPort.close();

   /* stop the thread */

  
   myIfarThread->stop();
	
   return true;
}


bool IfarModule::respond(const Bottle& commandIn, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" + 
                         "help \n" + 
                         "quit \n" + 
                         "set gain <n> ... set the gain \n" + 
                         "(where <n> is an integer number) \n";
 
   reply.clear(); 

   if (commandIn.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
   }
   else if (commandIn.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
/*  
   else if (commandIn.get(0).asString()=="set") {
      if (commandIn.get(1).asString()=="gain") {
         gain = commandIn.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
*/
   return true;
} 
bool IfarModule::updateModule()
{
   return true;
} 



double IfarModule::getPeriod()
{
   /* module periodicity (seconds), called implicitly by myModule */
   return 0.5;
}
