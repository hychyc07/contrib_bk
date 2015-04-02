/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.vernon.eu 
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
 * 29/03/11  Started development.   DV
 *
 * 09/04/11  Completed development. DV
 *
 * 11/01/12  Added module parameters:
 * 
 *           --contrastStretch      A flag which, if present, causes the optical flow magnitude image to be 
 *                                  contrast stretched before output
 * 
 *           --noMagnitude          A flag which, if present, inhibits the generation and output of the magnitude image 
 *
 *           --noPhase              A flag which, if present, inhibits the generation and output of the phase image 
 *
 *           --noPlot               A flag which, if present, inhibits the generation and output of the plot image 
 *
 *           Changed the plotOnInput parameter to be a flag with no value.
 *
 * 02/02/12  Acquire only one image at t1 each iteration, copying it to the old (t0) image after computing the flow field in preparation for the next iteration
 *
 * 03/01/12  Added input from motor encoders and associated module parameters:
 * 
 *           --encoderSamplingPeriod <value>      the period to allow between two successive samples of the motor encoders when determining whether or not
 *                                                the motors have stopped and the gazed is fixed
 *           --headStateInPort       <value>      specifies the input port name of the head encoder values  
 * **/ 


#include "iCub/opticalFlow.h"


opticalFlow::opticalFlow() {
   debug = false;
   encoderSamplingPeriod         = 0.05;  // time interval between two successive polls of the motor encoders 
                                           // when checking to see if the motion is complete

}

bool opticalFlow::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - opticalFlow.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("opticalFlow"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */

   robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   visualImageInputPortName   = "/";
   visualImageInputPortName  += getName(
                                rf.check("visualImageInPort", 
                                Value("/visualImage:i"),
                                "visual input image port (string)").asString()
                                );

   magnitudeOutputPortName    = "/";
   magnitudeOutputPortName   += getName(
                                rf.check("magnitudeImageOutPort", 
                                Value("/magnitudeImage:o"),
                                "Flow magnitude output image port (string)").asString()
                                );
   
   phaseOutputPortName        = "/";
   phaseOutputPortName       += getName(
                                rf.check("phaseImageOutPort", 
                                Value("/phaseImage:o"),
                                "Flow phase output image port (string)").asString()
                                );

   plotOutputPortName         = "/";
   plotOutputPortName        += getName(
                                rf.check("plotImageOutPort", 
                                Value("/plotImage:o"),
                                "Flow plot output image port (string)").asString()
                                );

   robotPortName              = "/";
   robotPortName             += getName(
                                rf.check("headStateInPort", 
                                Value("/headState:i"),
                                "robot head encoder state port (string)").asString()
                                );

   samplingPeriod             = rf.check("samplingPeriod",
                                Value(16),
                                "Sampling period value (int)").asInt();

   windowSize                 = rf.check("windowSize",
                                Value(64),
                                "Window size value (int)").asInt();

   sigma                      = (float) rf.check("sigma",
                                Value(16),
                                "Gaussian standard deviation value (int)").asDouble();

   x1ROI                      = rf.check("x1ROI",
                                Value(0),
                                "x coordinate of top left region of interest (int)").asInt();

   y1ROI                      = rf.check("y1ROI",
                                Value(0),
                                "y coordinate of top left region of interest (int)").asInt();

   x2ROI                      = rf.check("x2ROI",
                                Value(0),
                                "x coordinate of bottom right region of interest (int)").asInt();

   y2ROI                      = rf.check("y2ROI",
                                Value(0),
                                "y coordinate of bottom right region of interest (int)").asInt();

   plotOnInput                = rf.check("plotOnInput");

   contrastStretch            = rf.check("contrastStretch");

   noMagnitude                = rf.check("noMagnitude");

   noPhase                    = rf.check("noPhase");

   noPlot                     = rf.check("noPlot");

   encoderSamplingPeriod      = rf.check("encoderSamplingPeriod", 
                                         0.05, 
                                         "Time interval between two successive polls of the motor encoders (seconds, possibly fractional) ").asDouble();


 
   if (debug) {
      printf("opticalFlow: module name is                    %s\n",moduleName.c_str());
      printf("opticalFlow: robot name is                     %s\n",robotName.c_str());
      printf("opticalFlow: sampling period is                %d\n",samplingPeriod);
      printf("opticalFlow: window size is                    %d\n",windowSize);
      printf("opticalFlow: Gaussian standard deviation is    %f\n",sigma);
      printf("opticalFlow: ROI is                           (%d, %d) - (%d, %d)\n",x1ROI, y1ROI, x2ROI, y2ROI);
      printf("opticalFlow: encoder sampling period is        %f\n",encoderSamplingPeriod);

                                                          

      if (plotOnInput)     printf("opticalFlow: plotOnInput flag is set\n");
      else                 printf("opticalFlow: plotOnInput flag is not set\n");
      if (contrastStretch) printf("opticalFlow: contrastStretch flag is set\n");
      else                 printf("opticalFlow: contrastStretch flag is not set\n");
      if (noMagnitude)     printf("opticalFlow: noMagnitude flag is set\n");
      else                 printf("opticalFlow: noMagnitude flag is not set\n");
      if (noPhase)         printf("opticalFlow: noPhase flag is set\n");
      else                 printf("opticalFlow: noPhase flag is not set\n"); 
      if (noPlot)          printf("opticalFlow: noPlot flag is set\n");
      else                 printf("opticalFlow: noPlot flag is not set\n");
      printf("opticalFlow: port names are\n%s\n%s\n%s\n%s\n%s\n\n",visualImageInputPortName.c_str(),
                                                                   magnitudeOutputPortName.c_str(),
                                                                   phaseOutputPortName.c_str(),
                                                                   plotOutputPortName.c_str(),
                                                                   robotPortName.c_str() );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!visualImageIn.open(visualImageInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << visualImageInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!magnitudeImageOut.open(magnitudeOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << magnitudeOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!phaseImageOut.open(phaseOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << phaseOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!plotImageOut.open(plotOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << plotOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

    if (!robotPortIn.open(robotPortName.c_str())) {
      cout << getName() << ": unable to open port " << robotPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

   handlerPortName =  "/";
   handlerPortName += getName();         // use getName() rather than a literal 
 
   if (!handlerPort.open(handlerPortName.c_str())) {           
      cout << getName() << ": Unable to open port " << handlerPortName << endl;  
      return false;
   }

   attach(handlerPort);                  // attach to port
 
   /* create the thread and pass pointers to the module parameters */

   opticalFlowThread = new OpticalFlowThread(&visualImageIn, 
                                             &robotPortIn, 
                                             &magnitudeImageOut, 
                                             &phaseImageOut, 
                                             &plotImageOut,
                                             &samplingPeriod, 
                                             &windowSize, 
                                             &sigma,
                                             &x1ROI,
                                             &y1ROI,
                                             &x2ROI,
                                             &y2ROI,
                                             &encoderSamplingPeriod, 
                                             &plotOnInput,
                                             &contrastStretch,
                                             &noMagnitude,
                                             &noPhase,
                                             &noPlot);

   /* now start the thread to do the work */

   opticalFlowThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return false;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool opticalFlow::interruptModule()
{

	// BUG .... for some reason, trying to interrupt these ports stops the module stopping.
	//          so I'm not bothering to try anymore DV 02/02/12

   return true;

   visualImageIn.interrupt();
   magnitudeImageOut.interrupt();
   phaseImageOut.interrupt();
   plotImageOut.interrupt();
   handlerPort.interrupt();
   robotPortIn.interrupt();

   return true;
}


bool opticalFlow::close()
{
   

   /* stop the thread */
  
   opticalFlowThread->stop();
 
   visualImageIn.close();
   magnitudeImageOut.close();
   phaseImageOut.close();
   plotImageOut.close();
   handlerPort.close();
   robotPortIn.close();

   return true;
}


bool opticalFlow::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set period <p>   ... set the sampling period optical flow estimation \n" +   
                        "set window <q>   ... set the size of the window in which to estimate a flow vector \n" +
                        "set sigma  <r>   ... set the standard deviation of the Gaussian used to apodize the window \n" +
                        "(where  <p> is an integer number in the range [4,32], \n" +
                        "        <q> is either 32 or 64, \n" +
                        "        <r> is an real number in the range [16,64])\n";


  reply.clear(); 

  if (command.get(0).asString()=="quit") {
       reply.addString("quitting");
       return false;     
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="period") {
         samplingPeriod = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="window") {
         windowSize = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="sigma") {
         sigma = (float) command.get(2).asDouble(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool opticalFlow::updateModule()
{
   return true;
}



double opticalFlow::getPeriod()
{
   /* module periodicity (seconds), called implicitly by opticalFlow */
    
   return 0.1;
}

OpticalFlowThread::OpticalFlowThread(BufferedPort<ImageOf<PixelRgb> >      *visualImageIn, 
                                     BufferedPort<Vector>                  *robotIn,
                                     BufferedPort<ImageOf<PixelRgbFloat> > *magnitudeImageOut, 
                                     BufferedPort<ImageOf<PixelRgbFloat> > *phaseImageOut,
                                     BufferedPort<ImageOf<PixelRgb> >      *plotImageOut,
                                     int *samplingPeriodValue, 
                                     int *windowSizeValue,
                                     float *sigmaValue,
                                     int *x1ROIValue,
                                     int *y1ROIValue,
                                     int *x2ROIValue,
                                     int *y2ROIValue,
                                     double *encoderSamplingPeriodValue,                                 
                                     bool *plotOnInputValue,
                                     bool *contrastStretchValue,
                                     bool *noMagnitudeValue,
                                     bool *noPhaseValue,
                                     bool *noPlotValue)
{
   visualImagePortIn       = visualImageIn;
   robotPortIn             = robotIn;
   magnitudeImagePortOut   = magnitudeImageOut;
   phaseImagePortOut       = phaseImageOut;
   plotImagePortOut        = plotImageOut;
   samplingPeriod          = samplingPeriodValue; 
   windowSize              = windowSizeValue;
   sigma                   = sigmaValue; 
   x1ROI                   = x1ROIValue;
   y1ROI                   = y1ROIValue;
   x2ROI                   = x2ROIValue;
   y2ROI                   = y2ROIValue;
   plotOnInput             = plotOnInputValue;
   contrastStretch         = contrastStretchValue;
   noMagnitude             = noMagnitudeValue;
   noPhase                 = noPhaseValue;
   noPlot                  = noPlotValue;
   encoderSamplingPeriod   = encoderSamplingPeriodValue;

}

bool OpticalFlowThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug =  true;
    t0Input                      = NULL;
    t1Input                      = NULL;
    magnitudeOutput              = NULL;
    filteredMagnitudeOutput      = NULL;
    interpolatedMagnitudeOutput  = NULL;
    phaseOutput                  = NULL;
    interpolatedPhaseOutput      = NULL;
    plotOutput                   = NULL;
     
    allowableEncoderVariation     = 0.00;  // if all successive encoder readings are less than this value, 
                                           // we assume that head is stable and that the saccade is complete 


    return true;
}

void OpticalFlowThread::run(){

   /* 
    * Compute the optical flow
    */ 
      
   unsigned char pixel_value;
   float         float_pixel_value;

   if (debug) {
      printf("opticalFlowThread: parameters are %d %d %4.1f %d %d %d %d %4.1f\n\n",
             *samplingPeriod, *windowSize, *sigma, *x1ROI, *y1ROI, *x2ROI, *y2ROI, *encoderSamplingPeriod);
      if (*plotOnInput)     printf("opticalFlowThread: plotOnInput flag is set\n");
      else                  printf("opticalFlowThread: plotOnInput flag is not set\n");
      if (*contrastStretch) printf("opticalFlowThread: contrastStretch flag is set\n");
      else                  printf("opticalFlowThread: contrastStretch flag is not set\n");
      if (*noMagnitude)     printf("opticalFlowThread: noMagnitude flag is set\n");
      else                  printf("opticalFlowThread: noMagnitude flag is not set\n");
      if (*noPhase)         printf("opticalFlowThread: noPhase flag is set\n");
      else                  printf("opticalFlowThread: noPhase flag is not set\n"); 
      if (*noPlot)          printf("opticalFlowThread: noPlot flag is set\n");
      else                  printf("opticalFlowThread: noPlot flag is not set\n");

   }



   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
         

      
      /* 
       * Step 1: check to seem if the gaze is stable 
       *         this is optional; the robot port might not be connected
       * ===============================================================
       */
            
 
      do {
         do {
            tempEncoderPositions = robotPortIn->read(true);
         } while ((tempEncoderPositions == NULL) && (isStopping() != true));

         if (isStopping()) break; 

         yarp::os::Time::delay(*encoderSamplingPeriod);

         do {
            encoderPositions = robotPortIn->read(true);
         } while ((encoderPositions == NULL) && (isStopping() != true));

         if (isStopping()) break;
 
         stable = true;
         
         if (tempEncoderPositions != NULL  && encoderPositions != NULL) {

            for (i=0; i<6; i++) {
               if (fabs(encoderPositions->data()[i] - tempEncoderPositions->data()[i]) > allowableEncoderVariation) {
     
                  // if two sequential encoder readings are less than this value, 
                  // we assume that head is stable and that the saccade is complete  
            
                  stable = false;  
                  break;
              }
            }   
         }
         if (false) {
            if (!stable) 
               printf("overtAttentionThread: ............. HEAD MOVING\n");
         }
   
      } while (!stable);  
   

      /* 
       * Step 2: grab images at time t0 and time t1 and copy images to local format
       * ==========================================================================
       */

      if (debug) cout << "opticalFlowThread: grabbing images " << endl;
      do {
         t0Image = visualImagePortIn->read(true);
      } while ((t0Image == NULL) && (isStopping() != true));  // exit loop if shutting down
	 
      if (isStopping()) {
		        if (debug) cout << "opticalFlowThread: STOPPING at t0 " << endl;
                break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
	   }

      if (debug) cout << "opticalFlowThread: grabbed 1 " << endl;

      yarp::os::Time::delay(FRAME_SAMPLE_PERIOD);  // wait a couple of frames to make sure that the motion can be detected 

      do {
         t1Image = visualImagePortIn->read(true);
      } while ((t1Image == NULL) && (isStopping() != true));  // exit loop if shutting down
	  if (isStopping()) {
		        if (debug) cout << "opticalFlowThread: STOPPING at t1 " << endl;
                break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
	  }

      if (debug) cout << "opticalFlowThread: grabbed 2 " << endl;


      width  = t0Image->width();
      height = t0Image->height();
      depth = 3;
      if (debug) printf("opticalFlowThread: width = %d, height = %d, depth = %d\n",width, height, depth);
    
      if (t0Input == NULL) {
          t0Input = new DVimage(width, height, depth);
      }

      if (t1Input == NULL) {
          t1Input = new DVimage(width, height, depth);
      }

      if (magnitudeOutput == NULL) {
          magnitudeOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (interpolatedMagnitudeOutput == NULL) {
         if (!(*noMagnitude))
            interpolatedMagnitudeOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (filteredMagnitudeOutput == NULL) {
         if (!(*noMagnitude))
            filteredMagnitudeOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (phaseOutput == NULL) {
         phaseOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }
    
      if (interpolatedPhaseOutput == NULL) {
         if (!(*noPhase))
            interpolatedPhaseOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }
    
      if (plotOutput == NULL) {
         if (!(*noPlot))
            plotOutput = new DVimage(width, height, COLOUR_IMAGE);
      }


      /* make sure the region of interest lies within the image bounds */
      
      if (*x1ROI < 0)      *x1ROI = 0;
      if (*x1ROI > width)  *x1ROI = width;
      if (*y1ROI < 0)      *y1ROI = 0;
      if (*y1ROI > height) *y1ROI = height;

      if (*x2ROI < 0)      *x2ROI = 0;
      if (*x2ROI > width)  *x2ROI = width;
      if (*y2ROI < 0)      *y2ROI = 0;
      if (*y2ROI > height) *y2ROI = height;

      /* check for default configuration when ROI is the same as the entire image */

      if (*x1ROI == 0 && *x2ROI == 0)      *x2ROI = width;
      if (*y1ROI == 0 && *y2ROI == 0)      *y2ROI = height;

      if (debug) {
         printf("opticalFlowThread: parameters are %d %d %4.1f %d %d %d %d\n\n",
                *samplingPeriod, *windowSize, *sigma, *x1ROI, *y1ROI, *x2ROI, *y2ROI);
      }

      /* and now copy the ROI */

      for (x = *x1ROI; x < *x2ROI; x++) {
         for (y = *y1ROI; y < *y2ROI; y++) {
            rgbPixel = t0Image->safePixel(x,y);
            t0Input->put_pixel(x, y, rgbPixel.r, 0);
            t0Input->put_pixel(x, y, rgbPixel.g, 1);
            t0Input->put_pixel(x, y, rgbPixel.b, 2);

            rgbPixel = t1Image->safePixel(x,y);
            t1Input->put_pixel(x, y, rgbPixel.r, 0);
            t1Input->put_pixel(x, y, rgbPixel.g, 1);
            t1Input->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 


      /* 
       * Step 3: compute the optical flow
       * ================================ 
       *
       */
 
      if (debug) cout << "opticalFlowThread: computing optical flow " << endl;

      optical_flow (t0Input, t1Input, *windowSize, *samplingPeriod, *sigma, *x1ROI, *y1ROI, *x2ROI, *y2ROI, magnitudeOutput, phaseOutput);


      /* 
       * Step 4: plot the flow field  
       * =========================== 
       */

      if (!(*noPlot)) {

         if (debug) cout << "opticalFlowThread: plotting flow field " << endl;

         plotOutput->initialize();
            
         /* and now copy the ROI to superimpose plot field on the visual image */

         if (*plotOnInput && !(*noPlot)) {
            for (x = *x1ROI; x < *x2ROI; x++) {
               for (y = *y1ROI; y < *y2ROI; y++) {
                  t0Input->get_pixel(x, y, &pixel_value,0); 
                  plotOutput->put_pixel(x, y, pixel_value, 0);
                  t0Input->get_pixel(x, y, &pixel_value,1); 
                  plotOutput->put_pixel(x, y, pixel_value, 1);
                  t0Input->get_pixel(x, y, &pixel_value,2); 
                  plotOutput->put_pixel(x, y, pixel_value, 2);
              }
            } 
         }

         plot_field(magnitudeOutput, phaseOutput, plotOutput, 1, 0, 255, 0);  // plot with a scale factor = 1 and colour = (0, 255, 0)
      }

      /* 
       * Step 5: interpolate the flow field magnitude & direction
       * ========================================================
       */

      if (!(*noMagnitude)) {

         if (debug) cout << "opticalFlowThread: interpolating flow field magnitude" << endl;

         interpolate2(magnitudeOutput,interpolatedMagnitudeOutput);  // interpolate the velocity magnitude
      }
    

      if (!(*noPhase)) {

         if (debug) cout << "opticalFlowThread: interpolating flow field phase" << endl;

         interpolate2(phaseOutput,interpolatedPhaseOutput);          // interpolate the velocity direction
      }


      /* 
       * Step 6: copy flow images back to YARP format and write them out
       * =================================================================
       */

      if (debug) cout << "opticalFlowThread: sending output images " << endl;
     
      if (!(*noMagnitude) ) {

          
         if (debug) cout << "opticalFlowThread: performing distance from background filter " << endl;

         distance_from_background_transform(interpolatedMagnitudeOutput, filteredMagnitudeOutput);

		   if (*contrastStretch) {
            if (debug) cout << "opticalFlowThread: constast stretching magnitude " << endl;
		      filteredMagnitudeOutput->contrast_stretch();
		   } 

         if (debug) cout << "opticalFlowThread: sending output images " << endl;

      
         ImageOf<PixelRgbFloat> &magnitudeImage  = magnitudeImagePortOut->prepare();
         magnitudeImage.resize(width,height);
         for (x = 0; x < width; x++) {
            for (y = 0; y < height; y++) {
               filteredMagnitudeOutput->get_pixel(x, y, &float_pixel_value); 
               floatRgbPixel.r = float_pixel_value; 
               floatRgbPixel.g = float_pixel_value;
               floatRgbPixel.b = float_pixel_value;
               magnitudeImage(x,y) = floatRgbPixel;
            }
         }
         magnitudeImagePortOut->write();
      }

      if (!(*noPhase)) {
         ImageOf<PixelRgbFloat> &phaseImage  = phaseImagePortOut->prepare();
         phaseImage.resize(width,height);
         for (x = 0; x < width; x++) {
            for (y = 0; y < height; y++) {
               interpolatedPhaseOutput->get_pixel(x, y, &float_pixel_value); 
               floatRgbPixel.r = float_pixel_value; 
               floatRgbPixel.g = float_pixel_value;
               floatRgbPixel.b = float_pixel_value;
               phaseImage(x,y) = floatRgbPixel;
            }
         }
         phaseImagePortOut->write();
      }

      if (!(*noPlot)) {
         ImageOf<PixelRgb> &plotImage  = plotImagePortOut->prepare();
         plotImage.resize(width,height);

         for (x = 0; x < width; x++) {
            for (y = 0; y < height; y++) {
               plotOutput->get_pixel(x, y, &pixel_value, 0); 
               rgbPixel.r = pixel_value; 
               plotOutput->get_pixel(x, y, &pixel_value, 1); 
               rgbPixel.g = pixel_value;
               plotOutput->get_pixel(x, y, &pixel_value, 2); 
               rgbPixel.b = pixel_value;
               plotImage(x,y) = rgbPixel;
            }
         }
         plotImagePortOut->write();
      }

      if (debug) cout << "opticalFlowThread: finished" << endl;
 
   }
}

void OpticalFlowThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
 
   if (t0Input != NULL)                      delete t0Input;
   if (t1Input != NULL)                      delete t1Input;
   if (magnitudeOutput != NULL)              delete magnitudeOutput;
   if (filteredMagnitudeOutput != NULL)      delete filteredMagnitudeOutput;
   if (interpolatedMagnitudeOutput != NULL)  delete interpolatedMagnitudeOutput;
   if (phaseOutput != NULL)                  delete phaseOutput;
   if (interpolatedPhaseOutput != NULL)      delete interpolatedPhaseOutput;
   if (plotOutput != NULL)                   delete plotOutput;
}

