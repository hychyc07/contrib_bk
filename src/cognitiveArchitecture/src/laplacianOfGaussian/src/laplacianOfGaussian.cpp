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
 * 14/02/12  Started development.   DV
 */ 


#include "iCub/laplacianOfGaussian.h"


LaplacianOfGaussian::LaplacianOfGaussian() {
   debug = false;
}

bool LaplacianOfGaussian::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - laplacianOfGaussian.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("laplacianOfGaussian"), 
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

   logOutputPortName          = "/";
   logOutputPortName         += getName(
                                rf.check("logImageOutPort", 
                                Value("/logImage:o"),
                                "Flow magnitude output image port (string)").asString()
                                );
   
   zcOutputPortName           = "/";
   zcOutputPortName          += getName(
                                rf.check("zcImageOutPort", 
                                Value("/zcImage:o"),
                                "Flow phase output image port (string)").asString()
                                );

   plotOutputPortName         = "/";
   plotOutputPortName        += getName(
                                rf.check("plotImageOutPort", 
                                Value("/plotImage:o"),
                                "Flow plot output image port (string)").asString()
                                );

   sigma                      = (float) rf.check("sigma",
                                Value(16),
                                "Gaussian standard deviation value (int)").asDouble();

   significantZC              = rf.check("significantZC");

   noZC                       = rf.check("noZC");

   noPlot                     = rf.check("noPlot");

 
   if (debug) {
      printf("laplacianOfGaussian: module name is                    %s\n",moduleName.c_str());
      printf("laplacianOfGaussian: robot name is                     %s\n",robotName.c_str());
      printf("laplacianOfGaussian: Gaussian standard deviation is    %f\n",sigma);
      if (significantZC)   printf("laplacianOfGaussian: significantZC flag is set\n");
      else                 printf("laplacianOfGaussian: significantZC flag is not set\n");
      if (noZC)            printf("laplacianOfGaussian: noZC flag is set\n");
      else                 printf("laplacianOfGaussian: noZC flag is not set\n"); 
      if (noPlot)          printf("laplacianOfGaussian: noPlot flag is set\n");
      else                 printf("laplacianOfGaussian: noPlot flag is not set\n");
      printf("laplacianOfGaussian: port names are\n%s\n%s\n%s\n%s\n\n",visualImageInputPortName.c_str(),
                                                                   logOutputPortName.c_str(),
                                                                   zcOutputPortName.c_str(),
                                                                   plotOutputPortName.c_str()
                                                                  );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!visualImageIn.open(visualImageInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << visualImageInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!logImageOut.open(logOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << logOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!zcImageOut.open(zcOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << zcOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!plotImageOut.open(plotOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << plotOutputPortName << endl;
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

   laplacianOfGaussianThread = new LaplacianOfGaussianThread(&visualImageIn, 
                                                             &logImageOut, 
                                                             &zcImageOut, 
                                                             &plotImageOut,
                                                             &sigma,
                                                             &significantZC,
                                                             &noZC,
                                                             &noPlot);

   /* now start the thread to do the work */

   laplacianOfGaussianThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool LaplacianOfGaussian::interruptModule()
{
   visualImageIn.interrupt();
   logImageOut.interrupt();
   zcImageOut.interrupt();
   plotImageOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool LaplacianOfGaussian::close()
{
   
   /* stop the thread */

   visualImageIn.close();
   logImageOut.close();
   zcImageOut.close();
   plotImageOut.close();
   handlerPort.close();


   laplacianOfGaussianThread->stop();


   return true;
}


bool LaplacianOfGaussian::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set sigma  <r>   ... set the standard deviation of the Gaussian used to apodize the window \n" +
                        "(where  <r> is an real number)\n";


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
      if (command.get(1).asString()=="sigma") {
         sigma = (float) command.get(2).asDouble(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool LaplacianOfGaussian::updateModule()
{
   return true;
}



double LaplacianOfGaussian::getPeriod()
{
   /* module periodicity (seconds), called implicitly by laplacianOfGaussian */
    
   return 0.1;
}

LaplacianOfGaussianThread::LaplacianOfGaussianThread(BufferedPort<ImageOf<PixelRgb> >      *visualImageIn, 
                                                     BufferedPort<ImageOf<PixelRgbFloat> > *logImageOut, 
                                                     BufferedPort<ImageOf<PixelRgbFloat> > *zcImageOut,
                                                     BufferedPort<ImageOf<PixelRgb> >      *plotImageOut,
                                                     float *sigmaValue,
                                                     bool *significantZCValue,
                                                     bool *noZCValue,
                                                     bool *noPlotValue)
{
   visualImagePortIn       = visualImageIn;
   logImagePortOut         = logImageOut;
   zcImagePortOut          = zcImageOut;
   plotImagePortOut        = plotImageOut;
   sigma                   = sigmaValue; 
   significantZC           = significantZCValue;
   noZC                    = noZCValue;
   noPlot                  = noPlotValue;
}

bool LaplacianOfGaussianThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    intensityInput               = NULL;
    logOutput                    = NULL;
    zcOutput                     = NULL;
    plotOutput                   = NULL;
  
    return true;
}

void LaplacianOfGaussianThread::run(){

   /* 
    * Compute the optical flow
    */ 
      
   unsigned char pixel_value;
   float         float_pixel_value;

   if (debug) {
      printf("laplacianOfGaussianThread: parameters are %4.1f \n",*sigma);
      if (*significantZC)   printf("laplacianOfGaussianThread: significantZC flag is set\n");
      else                  printf("laplacianOfGaussianThread: significantZC flag is not set\n");
      if (*noZC)            printf("laplacianOfGaussianThread: noZC flag is set\n");
      else                  printf("laplacianOfGaussianThread: noZC flag is not set\n"); 
      if (*noPlot)          printf("laplacianOfGaussianThread: noPlot flag is set\n");
      else                  printf("laplacianOfGaussianThread: noPlot flag is not set\n");

   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
         
      /* 
       * Step 1: grab images at time t0 and time t1 and copy images to local format
       * ==========================================================================
       */

      if (debug) cout << "laplacianOfGaussianThread: grabbing images " << endl;

      do {
         intensityImage = visualImagePortIn->read(true);
      } while ((intensityImage == NULL) && (isStopping() != true));  // exit loop if shutting down

      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 

      if (debug) cout << "laplacianOfGaussianThread: grabbed intensity image " << endl;

      width  = intensityImage->width();
      height = intensityImage->height();
      depth = 3;
      if (debug) printf("laplacianOfGaussianThread: width = %d, height = %d, depth = %d\n",width, height, depth);
    
      if (intensityInput == NULL) {
          intensityInput = new DVimage(width, height, depth);
      }

      if (logOutput == NULL) {
          logOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (zcOutput == NULL) {
         if (!(*noZC))
            zcOutput = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }
    
      if (plotOutput == NULL) {
         if (!(*noPlot))
            plotOutput = new DVimage(width, height, COLOUR_IMAGE);
      }

      if (debug) {
         printf("laplacianOfGaussianThread: parameters are %4.1f\n",*sigma);
      }

      /* and now copy the image */

      for (x = 0; x < width; x++) {
         for (y = 0; y < height; y++) {
            rgbPixel = intensityImage->safePixel(x,y);
            intensityInput->put_pixel(x, y, rgbPixel.r, 0);
            intensityInput->put_pixel(x, y, rgbPixel.g, 1);
            intensityInput->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 


      /* 
       * Step 2: compute the Laplacian of Gaussian and zero-crossings
       * ============================================================
       *
       */
 
      if (debug) cout << "laplacianOfGaussianThread: computing Laplacian of Gaussian " << endl;

      laplacianOfGaussianWrapper(intensityInput, *sigma, *significantZC, *noZC, *noPlot, logOutput, zcOutput, plotOutput);
 

       /* 
       * Step 3: copy images back to YARP format and write them out
       * ==========================================================
       */

      if (debug) cout << "laplacianOfGaussianThread: sending output images " << endl;
     
      ImageOf<PixelRgbFloat> &logImage  = logImagePortOut->prepare();
      logImage.resize(width,height);
      for (x = 0; x < width; x++) {
         for (y = 0; y < height; y++) {
            logOutput->get_pixel(x, y, &float_pixel_value); 
            floatRgbPixel.r = float_pixel_value; 
            floatRgbPixel.g = float_pixel_value;
            floatRgbPixel.b = float_pixel_value;
            logImage(x,y) = floatRgbPixel;
            }
      }
      logImagePortOut->write();
    

      if (!(*noZC)) {
         ImageOf<PixelRgbFloat> &zcImage  = zcImagePortOut->prepare();
         zcImage.resize(width,height);
         for (x = 0; x < width; x++) { 
            for (y = 0; y < height; y++) {
               zcOutput->get_pixel(x, y, &float_pixel_value); 
               floatRgbPixel.r = float_pixel_value; 
               floatRgbPixel.g = float_pixel_value;
               floatRgbPixel.b = float_pixel_value;
               zcImage(x,y) = floatRgbPixel;
            }
         }
         zcImagePortOut->write();
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

      if (debug) cout << "laplacianOfGaussianThread: finished" << endl;
  
   }
}

void LaplacianOfGaussianThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */

   return;


   if (intensityInput != NULL)         delete intensityInput;
   if (logOutput != NULL)              delete logOutput;
   if (zcOutput != NULL)               delete zcOutput;
   if (plotOutput != NULL)             delete plotOutput;
}

/*****************************************************************************/
/*                                                                           */
/*              +------------------------------------------+                 */
/*              |                                          |                 */
/*              |    IMAGE AND MOVEMENT UNDERSTANDING      |                 */
/*              |                                          |                 */
/*              |           ESPRIT PROJECT 419             |                 */
/*              |                                          |                 */
/*              +------------------------------------------+                 */
/*                                                                           */
/*      COPYRIGHT NOTICE                                                     */
/*      ----------------                                                     */
/*                                                                           */
/*      This software was developed at the Department of Computer Science,   */
/*      Trinity College Dublin, Ireland, in co-operation with the            */
/*                                                                           */
/*      Department of Communication, Computer and Systems Science,           */
/*      University of Genoa, Italy;                                          */
/*                                                                           */
/*      Department of Experimental Psychology,                               */
/*      University of Nijmegen, The Netherlands;                             */
/*                                                                           */
/*      Video Display Systems, Florence, Italy;                              */
/*                                                                           */
/*      Computer Applied Techniques Ltd., Dublin, Ireland.                   */
/*                                                                           */
/*      This research was funded by the Commission of the European           */
/*      Communities under the European Strategic Program for Research and    */
/*      Development in Information Technologies (ESPRIT).                    */
/*      This software is classified as foreground information and is subject */
/*      to the copyright conditions laid down by Article 5 and Annex 4 of    */
/*      the Esprit contract for project 419.                                 */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*      Date  86/05/22                                                       */
/*                                                                           */
/*      Module Name:    ipp2  (Image Processing Primitives: Volume 2)        */
/*      Version Number: 1.3    	                                            */
/*      Machine dependent code: YES ... in routines: acq                     */
/*                                                   init                    */
/*                                                   syn                     */
/*                                                   read_pixel              */
/*                                                   write_pixel             */
/*      Module Type:  subroutines.                                           */
/*                                                                           */
/*      Module Description: Library of primitives for low-level image        */
/*                          manipulation and processing.                     */
/*                                                                           */
/*      Module Index:                                                        */
/*                    laplacian_of_gaussian                                  */
/*                    line_fire                                              */
/*                    parallel_projection                                    */
/*                    p_plot                                                 */
/*                    readcol                                                */
/*                    readlin                                                */
/*                  * read_pixel                                             */
/*                    realtime_display                                       */
/*                    sample_framestore                                      */
/*                    syn                                                    */
/* Conditioned for Xenix operating system only :                             */
/*                                                                           */
/*                    open_io                                                */
/*                    open_mem                                               */
/*                    inp                                                    */
/*                    outp                                                   */
/*                    movmem                                                 */
/*                                                                           */
/*                                                                           */
/*      Related Documents:  "Provisional Specification of a Virtual          */
/*                           Image System", Internal Report.                 */
/*                                                                           */
/* Revision 1.2  86/05/22  19:39:07  giulio  				     */
/* Test on image types in routine "valid_transfer_combination"		     */
/* has been splitted to avoid compiler errors due to the		     */
/* dimension of the condition. 						     */
/*                                                                           */
/*                                                                           */
/* Revision 1.3  5-8-86   David.             				     */
/* Routine "zero_crossings" altered so that user-interaction                 */
/* has been removed to the calling routine; contour representation image     */
/* descriptors are now passed explicitly, along with a flag to indicate      */
/* whether or not they are to be constructed.                                */
/* Calling routine (transfer_view) has been altered accordingly.             */
/*                                                                           */
/* Revision 28-8-86   Jon.                                                   */
/* Routines acq,init,read_pixel,syn,write_pixel updated to include FG100AT.  */
/*                                                                           */
/* Revision 10-10-86   Massimo.                                              */
/* Routines acq,init,read_pixel,syn,write_pixel updated to include Virtual   */
/* Frame Store.  Init routine has been modified as to initialise one         */
/* framestore at time.                                                       */
/*                                                                           */
/* Revision 20-10-86   Massimo.                                              */
/* Added regional noise cleaning as convolution to convolution image         */
/* transfer in routine transfer_view; also changed routine valid_transfer_   */
/* combination to allow the transfer.                                        */
/*                                                                           */
/* Revision 06-11-86   Massimo.                                              */
/* Routines acq,init,read_pixel,syn,write_pixel updated to include Eidobrain */
/*                                                                           */
/*                                                                           */
/* Revision 12-12-86   David                                                 */
/* Split ipp.c into two files (ipp1 and ipp2) to facilitate port to Xenix    */
/*                                                                           */  
/* Revision 04-03-87   Massimo.                                              */
/* Inserted conditional compilation for framestores                          */
/*                                                                           */  
/* Revision 02-04-87   David.                                                */
/* Routine to determine binary threshold included.                           */
/*                                                                           */  
/* Revision 04-05-87   Massimo.                                              */
/* Routine for interpolation of depth on contours included                   */
/*                                                                           */  
/* Revision 10-05-87   Sean.                                                 */
/* Fixes for Xenix compiler.                                                 */
/*                                                                           */  
/* Revision 16-11-87   Mairead.                                              */
/* Fixes for eidobrain operation.                                            */
/*                                                                           */  
/* Revision 25-04-88  Mairead.                                               */
/* Replaced all 0. wth 0.0 for transputer operation                          */
/*                                                                           */
/* Revision 1-5-88    David.                                                 */
/* Split ipp1.c into ipp1.c and ipp2.c (renamed old ipp2.c to ipp3.c)        */
/*                                                                           */
/* Revision 8-7-88    David.                                                 */
/* parallel_projection, and p_plot will not be compiled if VIS1              */
/* is defined.                                                               */
/*                                                                           */
/* Revision 2-11-88 David.                                                   */ 
/* Commented out the messages during the execution Laplacian of Gaussian     */
/*                                                                           */
/* Revision 02-03-89   Mairead.                                              */
/* Routines read_pixel,syn updated to include PCVISIONplus framestore        */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/*              +------------------------------------------+                 */
/*              |                                          |                 */
/*              |    IMAGE AND MOVEMENT UNDERSTANDING      |                 */
/*              |                                          |                 */
/*              |           ESPRIT PROJECT 419             |                 */
/*              |                                          |                 */
/*              +------------------------------------------+                 */
/*                                                                           */
/*      COPYRIGHT NOTICE                                                     */
/*      ----------------                                                     */
/*                                                                           */
/*      This software was developed at the Department of Computer Science,   */
/*      Trinity College Dublin, Ireland, in co-operation with the            */
/*                                                                           */
/*      Department of Communication, Computer and Systems Science,           */
/*      University of Genoa, Italy;                                          */
/*                                                                           */
/*      Department of Experimental Psychology,                               */
/*      University of Nijmegen, The Netherlands;                             */
/*                                                                           */
/*      Video Display Systems, Florence, Italy;                              */
/*                                                                           */
/*      Computer Applied Techniques Ltd., Dublin, Ireland.                   */
/*                                                                           */
/*      This research was funded by the Commission of the European           */
/*      Communities under the European Strategic Program for Research and    */
/*      Development in Information Technologies (ESPRIT).                    */
/*      This software is classified as foreground information and is subject */
/*      to the copyright conditions laid down by Article 5 and Annex 4 of    */
/*      the Esprit contract for project 419.                                 */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*      Date **06-07-85**                                                    */
/*                                                                           */
/*      Module Name:    dsdata.h                                             */
/*      Version Number: 1.00                                                 */
/*      Machine dependent code: *No*                                         */
/*                                                                           */
/*      Module Type:  Data Structure.                                        */
/*      Module Description: Global data-structures of Virtual Image System.  */
/*                                                                           */
/*      Module Index: pyramid_list                                           */
/*                    microeye_data                                          */
/*                    views                                                  */
/*                    raw_primal_sketch                                      */  
/*                                                                           */
/*      Related Documents: "Provisional Specification for a Virtual          */
/*                          Image System", Internal Report.                  */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/* ---  File Name: dsdata.h                                                  */
/*                                                                           */
/* ---  Functional Description: Global Data-Structures for                   */
/*                              Virtual Image System.                        */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none                 */
/*                                                                           */
/* ---  Calling Procedure:  #include "dsdata.h"                              */
/*                                                                           */
/* ---  Input Parameters:  n/a                                               */
/*                                                                           */
/* ---  Output Parameters: n/a                                               */
/*                                                                           */
/* ---  Global Parameters: n/a                                               */
/*                                                                           */
/* ---  Local Variables:   n/a                                               */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon - TCD.                                          */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      28/8/86                                                   */
/*      revision:  FG100AT frame grabber added - Jon Kennedy.                */
/*      reason:                                                              */
/*                                                                           */
/*      date:      11/7/88  David Vernon                                     */
/*      revision:  Global variable, installed_framestore, introduced to      */
/*                 identify which framestore was specified in config.vis     */
/*                                                                           */
/*      date:      03/11/88  David Vernon                                    */
/*      revision:  Global variable, tds_framestore, introduced to identify   */
/*                 which of several identical framestores is currently being */
/*                 accessed.                                                 */
/*                                                                           */
/*      date:      03/11/88  David Vernon                                    */
/*      revision:  Global variable, number_of_framestores, introduced to     */
/*                 keep a track of the total number of (identical)           */
/*                 framestores is currently in the system                    */
/*                                                                           */
/*      date:      02/03/89                                                  */
/*      revision:  PCVISIONplus frame grabber added - Mairead Flanagan       */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*                            PYRAMID_LIST                                   */
/*                                                                           */
/*   This global data-structure definition is simply a pointer to the first  */
/*   pyramid descriptor in the system. It is initially NULL as the initial,  */
/*   unconfigured, state of the system has no images or pyramids.            */
/*                                                                           */
/*****************************************************************************/

struct pyramid_descriptor_type *pyramid_list = NULL;
 
/*****************************************************************************/
/*                                                                           */
/*                            PCVISION_DATA                                  */
/*                                                                           */
/*   This global data-structure defines the data required to use the         */
/*   PC-Vision framegrabber.                                                 */
/*                                                                           */
/*****************************************************************************/

struct pcvision_type pcvision_data =

               /* control/status register base address = FF00   */

   { 0xFF00,   /* address of control/status register; low byte  */
     0xFF01,   /* address of control/status register; high byte */
     0xFF02,   /* address of LUT address register               */
     0xFF03,   /* address of LUT data register                  */
     0xFF04,   /* address of mask register                      */
     0xFF05,   /* address of frame memory block select register */
     0xFF06,   /* address of reset vertical blank interrupt reg.*/

    0xA0000l,  /* frame memory base address                     */

     0x0B,     /* init value: written to control/status word.     */
               /*             bit 7 = 0 }                         */
               /*             bit 6 = 0 } ... using output LUT    */
               /*             bit 5 = 0 }                         */
               /*             bit 4 = 1 } ... TERMINATE OPERATION */
               /*             bit 3 = 1   ... phase-lock loop     */
               /*             bit 2 = 0 }                         */
               /*             bit 1 = 1 } ... luminance output    */
               /*             bit 0 = 1   ... board select        */

     0x1B,     /* clear value: written to control/status word.    */
               /*             bit 7 = 0 }                         */
               /*             bit 6 = 0 } ... using output LUT    */
               /*             bit 5 = 0 }                         */
               /*             bit 4 = 1 } ... CLEAR FRAMESTORE    */
               /*             bit 3 = 1   ... phase-lock loop     */
               /*             bit 2 = 0 }                         */
               /*             bit 1 = 1 } ... luminance output    */
               /*             bit 0 = 1   ... board select        */

     0x2B,     /* syn value   written to control/status word.     */
               /*             bit 7 = 0 }                         */
               /*             bit 6 = 0 } ... using output LUT    */
               /*             bit 5 = 1 }                         */
               /*             bit 4 = 0 } ... REAL-TIME DIGITISE  */
               /*             bit 3 = 1   ... phase-lock loop     */
               /*             bit 2 = 0 }                         */
               /*             bit 1 = 1 } ... luminance output    */
               /*             bit 0 = 1   ... board select        */

     0x3B,     /* acq value   written to control/status word.     */
               /*             bit 7 = 0 }                         */
               /*             bit 6 = 0 } ... using output LUT    */
               /*             bit 5 = 1 }                         */
               /*             bit 4 = 1 } ... FREEZE FRAME        */
               /*             bit 3 = 1   ... phase-lock loop     */
               /*             bit 2 = 0 }                         */
               /*             bit 1 = 1 } ... luminance output    */
               /*             bit 0 = 1   ... board select        */

     0x00,     /* LUT 0        */
     0x20,     /* LUT 1        */
     0x40,     /* LUT 2        */
     0x60,     /* LUT 3        */

   };

 
/*****************************************************************************/
/*                                                                           */
/*                                VIEWS                                      */
/*                                                                           */
/*   This global data-structure defines an array of views (see type          */
/*   definition view_type in file dsdef.h) used to delimit sub-regions of    */
/*   an image during primitive image manipulation.                           */
/*                                                                           */
/*   It is possible to define an arbitrary number of views but we assume     */
/*   a maximum given by the symbolic constant MAX_NUMBER_VIEWS (defined in   */
/*   dsdef.h).                                                               */
/*   Each view is defined by its x and y extent and is identified by a       */
/*   view number.                                                            */
/*   Since views are to be used frequently in image manipulation functions,  */
/*   e.g. transfer_view, it is desirable that access to a particular view be */
/*   as efficient as possible.  Thus, the views data-structure is            */
/*   implemented as an array of view_types; the view number is simply the    */
/*   array index.                                                            */
/*                                                                           */
/*****************************************************************************/

struct view_type views[MAX_NUMBER_VIEWS];
struct raw_primal_image_type *raw_primal_sketch = NULL;
int installed_framestore;
int tds_framestore = 1;
int number_of_framestores = 0;


/* global variables for region image processing functions */

static int X_SAVE,Y_SAVE;
struct {
        int X,Y;
       } p_stack [NLOC_STACK];

static int p_stk;
static int stk_n_empty;


/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: append_to_cc                                        */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*     This subroutine appends a node to a Freeman chain code.              */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*     None                                                                 */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     append_to_cc(cc,end,dirn);                                           */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*     - cc ..... address of pointer to head of chain code                  */
/*      - end .... address of pointer to last node in chain code            */
/*      - dirn ... the Freeman chain code of the new node                   */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*     - cc ..... address of pointer to head of chain code after insertion  */
/*     - end .... address of pointer to last node in chain code after       */
/*                insertion.	                                            */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*     None	                                                                */
/*                                                                          */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*     - p  			temporary pointer to a node.                        */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*     None (by now)                                                        */
/*                                                                          */
/* --- Author: D. Vernon (TCD) 	                                            */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*                                                                          */
/****************************************************************************/

void append_to_cc(struct cc_node *(*cc), struct cc_node *(*end),int dirn)
{
   struct cc_node *p;

   if ((p = (struct cc_node *) malloc(CC_NODE_SIZE)) == NULL) {
      system_error("Unsuccessful memory allocation (chain code node)");
   } 
   else {

      p->direction = dirn;
      p->flink = NULL;

      if (*cc == NULL) {

        /* null list */

        p->rlink = NULL;
        *end = p;
        *cc  = p;
      }
      else {

         p->rlink = *end;
         (*end)->flink = p;
         *end = p;
      }
   }
}

/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: build_contour_descriptors                           */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*	Build the contour descriptors associated with contour-based images.     */
/*	The routine must be passed at least two contour-type images; any of     */
/*      the others may be null.  Note that there is a strict order in the   */
/*      image types in the parameter list.                                  */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     build_contour_descriptors( contour_image_descriptor,                 */
/*                                slope_image_descriptor,                   */
/*                                orientation_image_descriptor,             */
/*                                disparity_image_descriptor,               */
/*                                velocity_image_descriptor)                */
/*                                                                          */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*	- contour_image_descriptor,                                             */
/*	- slope_image_descriptor,                                               */
/*	- orientation_image_descriptor,                                         */
/*	- disparity_image_descriptor,                                           */
/*	- velocity_image_descriptor  pointers to respective image descriptors   */
/*                                                                          */
/* --- Output Parameters: none.                                             */
/*                                                                          */
/*                                                                          */
/* --- Global Parameters: none.                                             */
/*                                                                          */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*	- contour, slope, orientation, disparity, velocity                      */
/*                                                                          */
/*                              pointers to contour nodes in contour images */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*	None (by now)                                                           */
/*                                                                          */
/* --- Author: D. Vernon  TCD                                               */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:      26/07/86                                              */
/*         revisor:   David Vernon                                          */
/*         reason:    generate mean and standard deviation of curvature.    */
/*                                                                          */
/*         date:      29/11/86                                              */
/*         revisor:   David Vernon                                          */
/*         reason:    modified to facilitate new definition of velocity     */
/*                    image.                                                */
/*                                                                          */
/*         date:      30/01/87                                              */
/*         revisor:   David Vernon                                          */
/*         reason:    modified to facilitate depth image.                   */
/*                                                                          */
/*         date:      07/08/87                                              */
/*         revisor:   David Vernon                                          */
/*         reason:    depth ststistics are now built.                       */
/*                                                                          */
/****************************************************************************/

void build_contour_descriptors(struct image_descriptor_type *contour_image_descriptor,
                          struct image_descriptor_type *slope_image_descriptor,
                          struct image_descriptor_type *orientation_image_descriptor,
                          struct image_descriptor_type *disparity_image_descriptor,
                          struct image_descriptor_type *velocity_image_descriptor,
                          struct image_descriptor_type *depth_image_descriptor)
{
   struct contour_node_type *contour, *slope, *orientation, 
                            *disparity, *depth,   *p, *q;
      struct velocity_node_type *velocity;
    struct contour_descriptor_type *descr;
   short int count, i;
   double mean, standard_deviation, t1, t2;

   count = 0;

   contour = slope = orientation = disparity = depth = NULL;
   velocity = NULL;

   if (contour_image_descriptor != NULL) 
      if (contour_image_descriptor->image_type == CONTOUR_IMAGE_TYPE) {
         contour = contour_image_descriptor->image.contour_link;
         count++;
      }
   if (slope_image_descriptor != NULL) 
      if (slope_image_descriptor->image_type == SLOPE_IMAGE_TYPE) {
         slope = slope_image_descriptor->image.slope_link;
         count++;
      }
   if (orientation_image_descriptor != NULL) 
      if (orientation_image_descriptor->image_type == ORIENTATION_IMAGE_TYPE) {
         orientation = orientation_image_descriptor->image.orientation_link;
         count++;
      }
   if (disparity_image_descriptor != NULL) 
      if (disparity_image_descriptor->image_type == DISPARITY_IMAGE_TYPE) {
         disparity = disparity_image_descriptor->image.disparity_link;
         count++;
      }
   if (velocity_image_descriptor != NULL) 
      if (velocity_image_descriptor->image_type == VELOCITY_IMAGE_TYPE) {
         velocity = velocity_image_descriptor->image.velocity_link;
         count++;
      }
   if (depth_image_descriptor != NULL) 
      if (depth_image_descriptor->image_type == DEPTH_IMAGE_TYPE) {
         depth = depth_image_descriptor->image.depth_link;
         count++;
      }


   if (count >= 2 ) {

      /* adequate number of non-null parameters. */

      /* tack on the contour descriptors to each contour */
    
      while (contour != NULL  || slope != NULL || orientation != NULL ||
             disparity != NULL || velocity != NULL || depth != NULL) {

         if ((descr = (struct contour_descriptor_type *) 
                      malloc(CONTOUR_DESCRIPTOR_SIZE)) == NULL) {
            system_error("Unsuccessful memory allocation (contour descriptor)");
            break;
         } 
         else {
            if (contour != NULL) contour->descriptor_link = descr;
            descr->contour = contour;
            if (slope != NULL) slope->descriptor_link = descr;
            descr->slope = slope;
            if (orientation != NULL) orientation->descriptor_link = descr;
            descr->orientation = orientation;
            if (disparity != NULL) disparity->descriptor_link = descr;
            descr->disparity = disparity;
            if (velocity != NULL) velocity->descriptor_link = descr;
            descr->velocity = velocity;
            if (depth != NULL) depth->descriptor_link = descr;
            descr->depth = depth;
         }

         if (contour != NULL) contour = contour->next;
         if (slope != NULL) slope = slope->next;
         if (orientation != NULL) orientation = orientation->next;
         if (disparity != NULL) disparity = disparity->next;
         if (velocity != NULL) velocity = velocity->next;
         if (depth != NULL) depth = depth->next;

      }
  
      /* chain all contour descriptors together */

      p = NULL;

      if (contour_image_descriptor != NULL) 
         p = contour_image_descriptor->image.contour_link;
      else if (slope_image_descriptor != NULL) 
         p = slope_image_descriptor->image.slope_link;
      else if (orientation_image_descriptor != NULL) 
         p = orientation_image_descriptor->image.orientation_link;
      else if (disparity_image_descriptor != NULL) 
         p = disparity_image_descriptor->image.disparity_link;
      else if (depth_image_descriptor != NULL) 
         p = depth_image_descriptor->image.depth_link;

         /* note that since there must have been > two images, at least one */
         /* must be an image type other than velocity so we are guaranteed  */
         /* to initialise p as above.                                       */


      descr = NULL;

      if (p != NULL) {

         descr = p->descriptor_link;   

         p->descriptor_link->prev = NULL;   /* first node */

         q = p->next;

         while (q != NULL) {
            p->descriptor_link->next = q->descriptor_link;
            q->descriptor_link->prev = p->descriptor_link;
            p = q;
            q = q->next;
         }

         p->descriptor_link->next = NULL;    /* last node */
      }

      /*** now run along the contour descriptors initialising the fields ***/

      count = 0;

      while (descr != NULL) {

         /* select the contour */
 
         descr->select = TRUE;


         /* label it */

         descr->label = count++;

         
         /* compute length and origin */

         if (descr->contour != NULL) {
  	    descr->length = descr->contour->length;
            descr->start_x = descr->contour->start_x;
            descr->start_y = descr->contour->start_y;
         }
         else if (descr->slope != NULL) {
	    descr->length = descr->slope->length;
            descr->start_x = descr->slope->start_x;
            descr->start_y = descr->slope->start_y;
         }
         else if (descr->orientation != NULL) {
	    descr->length = descr->orientation->length;
            descr->start_x = descr->orientation->start_x;
            descr->start_y = descr->orientation->start_y;
         }
         else if (descr->disparity != NULL) {
	    descr->length = descr->disparity->length;
            descr->start_x = descr->disparity->start_x;
            descr->start_y = descr->disparity->start_y;
         }
         else if (descr->velocity != NULL) {
	    descr->length = descr->velocity->length;
            descr->start_x = descr->velocity->start_x;
            descr->start_y = descr->velocity->start_y;
         }
         else if (descr->depth != NULL) {
	    descr->length = descr->depth->length;
            descr->start_x = descr->depth->start_x;
            descr->start_y = descr->depth->start_y;
         }

         /* compute mean slope */

         mean = 0;
         for (i=0; i < descr->length; i++) 
            mean += descr->slope->value[i];
         mean = mean / i;
         descr->mean_slope = (int) mean;

         /* compute standard deviation of slope */

         t2 = 0.0;
         for (i=0; i < descr->length; i++) { 
            t1 = (descr->slope->value[i] - mean);
            t2 += t1*t1;
         } 
         t2 = t2/i;
         standard_deviation = sqrt(t2);
         descr->s_d_slope = (int) standard_deviation;

         /* compute mean orientation */

         mean = 0;
         for (i=0; i < descr->length; i++) 
            mean += descr->orientation->value[i];
         mean = mean / i;
         descr->mean_orientation =  (int) mean;

         /* compute standard deviation of orientation */

         t2 = 0.0;
         for (i=0; i < descr->length; i++) { 
            t1 = (descr->orientation->value[i] - mean);
            t2 += t1*t1;
         } 
         t2 = t2/i;
         standard_deviation = sqrt(t2);
         descr->s_d_orientation = (int) standard_deviation;


         /* compute mean curvature (first difference of orientation) */

         mean = 0;
         for (i=1; i < descr->length; i++) 
            mean += descr->orientation->value[i] -
                    descr->orientation->value[i-1] ;
         mean = mean / i;
         descr->mean_curvature =  (int) mean;

         /* compute standard deviation of curvature */

         t2 = 0.0;
         for (i=1; i < descr->length; i++) { 
            t1 = ((descr->orientation->value[i] -
                   descr->orientation->value[i-1]) - mean);
            t2 += t1*t1;
         } 
         t2 = t2/i;
         standard_deviation = sqrt(t2);
         descr->s_d_curvature = (int) standard_deviation;


         /* compute mean depth */

         if (descr->depth != NULL) {
            mean = 0;
            for (i=0; i < descr->length; i++) 
               mean += descr->depth->value[i];
            mean = mean / i;
            descr->mean_depth = (int) mean;

            /* compute standard deviation of depth */

            t2 = 0.0;
            for (i=0; i < descr->length; i++) { 
               t1 = (descr->depth->value[i] - mean);
               t2 += t1*t1;
            } 
            t2 = t2/i;
            standard_deviation = sqrt(t2);
            descr->s_d_depth = (int) standard_deviation;
         }
         else {
            descr->mean_depth = 0;
            descr->s_d_depth =  0;
         } 

         /* proceed to the next descriptor */
 
         descr = descr->next;
      }       
   }
}
/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: chain_calc                                          */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*	Computation of the (x,y) coordinates of a pixel according to its        */
/*	chain code and the position of the previous pixel.                      */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*	None                                                                    */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*	chain_calc(code,x,y); code is an input parameter that is the            */
/*	chain code of the pixel, x and y are the coordinates of the             */
/*	previous contour pixel.	                                                */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*	- code			chain code of the current pixel                         */
/*	- *x, *y		coordinates of the previous pixel                       */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*	- *x, *y		coordinates of the pixel with the given                 */
/*                  chain code                                              */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*	None                                                                    */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*	None                                                                    */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*	None (by now)                                                           */
/*                                                                          */
/* --- Author: G. Sandini & M. Tistarelli  UG - DIST                        */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:    26/06/86                                                */
/*         revisor: David Vernon (TCD)                                      */
/*         reason:  Ignore most significant bit of code (now used to select */
/*		    and deselect a contour point).                                  */
/*                                                                          */
/****************************************************************************/

void chain_calc(int code, int *x, int *y)	/* compute pixel coordinates */
{
        code &= 0x7F;   /* mask MSB */

	switch(code)	/* assign pixel coordinates according to the
				chain code */
	{
		case 0 :	(*x)++;	break;
		case 1 :	(*x)++;	(*y)++;	break;
		case 2 :	(*y)++;	break;
		case 3 : 	(*y)++;	(*x)--;	break;
		case 4 :	(*x)--;	break;
		case 5 : 	(*y)--;	(*x)--;	break;
		case 6 : 	(*y)--;	break;
		case 7 : 	(*y)--;	(*x)++;	break;
		default :	break;
	}
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: conv1                                               */
/*                                                                           */
/* ---  Functional Description: conv1 performs the convolution between the   */
/*      a mask and the input data with the sign extension inhibited from the */
/*      input data.                                                          */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none                 */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*              conv1(input_buffer,nptlin,norm_factor,output_buffer,         */
/*                                          maskad,nptmask,nptmas2);         */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*       input_buffer .. character pointer containing the input data.        */
/*       nptlin .. number of elements in the input data.                     */
/*       norm_factor .. normalizing factor.                                  */
/*       int *maskad .. pointer to the mask to be used in the convolution.   */ 
/*       int nptmask,nptmas2 .. number of points and half the number of      */
/*                              points in the mask.                          */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*          output_buffer .. character pointer containing the output data.   */
/*                                                                           */
/* ---  Global Parameters:                                                   */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N                                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      29/12/86  M.T.                                            */
/*      revision:  Blank iserted in expressions like =- to avoid             */
/*                 compiler warnings under Unix                              */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void conv1(char input_buffer[], short nptlin, int norm_factor,char output_buffer[], int *maskad, int nptmask, int nptmas2)
{
   short nout,input,low_word;
   int  *maskptr,i,j;
   long sum,mul;
   float nsum;
   char *outptr,*inpptr;

/******************************************************************************/
/*                                                                            */
/*          Clear an area of the output buffer where data is not available.   */
/*                                                                            */
/******************************************************************************/
   j=(int)(nptlin);
   for (i = 0 ; i != nptmas2 ; i++) {
      output_buffer[i]=0;
      output_buffer[--j]=0;
   }
/******************************************************************************/
/*                                                                            */
/*  Calculate the number of points that the convolution can be performed on.  */
/*                                                                            */
/******************************************************************************/

   nout=nptlin-nptmask+1;

/******************************************************************************/
/*                                                                            */
/*                           Main loop of convolution.                        */
/*                                                                            */
/******************************************************************************/
   for (i=0;i != nout;i++) {
      sum=0;                             /* long integer sum zeroed */ 
      outptr=(output_buffer+i+nptmas2);  /* setup pointer to the output array */
      inpptr=(input_buffer+i);           /* setup pointer to the input data */
      maskptr=maskad;                    /* setup pointer to the mask array */

/******************************************************************************/
/*                                                                            */
/*    Generate one point of the convolution output by multiplying the mask    */
/*    by the corresponding input data.                                        */
/*                                                                            */
/******************************************************************************/

      for (j=0; j != nptmask;j++) {     /* obtain the input data and mask */
         input=((*inpptr++) & ~0177400);
         mul=input*(*maskptr++);          /* multiply mask by input data */
         sum += mul;              /* add result of multiplication to sum  */
      }
/******************************************************************************/
/*                                                                            */
/*    convolution sum for one point obtained, now test this result.           */
/*                                                                            */
/*                                                                            */
/* round contents of sum to integer and place in low word but no loss of      */
/* information                                                                */
/*                                                                            */
/******************************************************************************/
      if (sum != 0) {
         nsum  = (((float)sum)/(norm_factor)) ;/* normalise & round sum*/
         if (nsum < 0) nsum -= 0.5;
         else nsum += 0.5;
         low_word = (short)nsum;
         if (low_word == 0) { /* if result is zero place corret sign on      */
            low_word = 1;      /* value stored in this location               */
            if (nsum  <= 0.0 ) low_word = -low_word; 
         }
      }
      else {
          low_word=0;
      }
/******************************************************************************/
/*                                                                            */
/*                          Output results                                    */
/*                                                                            */
/******************************************************************************/

      if (low_word > 127) { 
         *outptr = 127;                 /* output 127 */
      }
      else if (low_word < -128) { 
         *outptr = -128;                /* output -128 */
      }
      else { 
         *outptr= (char)low_word;    /* output lower byte of low_word */
      }
   }
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: conv2                                               */
/*                                                                           */
/* ---  Functional Description: conv2 performs the convolution between the   */
/*      a mask and the input data without removing the sign extension from   */
/*      the input data.                                                      */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none                 */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*              conv2(input_buffer,nptlin,norm_factor,output_buffer,         */
/*                                           maskad,nptmask,nptmas2);        */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*       input_buffer .. character pointer containing the input data.        */
/*       nptlin .. number of elements in the input data.                     */
/*       norm_factor .. normalizing factor.                                  */
/*       int *maskad .. pointer to the mask to be used in the convolution.   */ 
/*       int nptmask,nptmas2 .. number of points and half the number of      */
/*                              points in the mask.                          */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*          output_buffer .. character pointer containing the output data.   */
/*                                                                           */
/* ---  Global Parameters:                                                   */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N 24/09/85                                               */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      29/1/86  D.V.                                             */
/*      revision:  input byte forced to be 2's complement integer.           */
/*      reason:    AT char representation is unsigned integer 0-255.         */
/*                                                                           */
/*      date:      29/12/86  M.T.                                            */
/*      revision:  Blank iserted in expressions like =- to avoid             */
/*                 compiler warnings under Unix                              */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void conv2(char input_buffer[], short nptlin, int norm_factor, char output_buffer[], int *maskad, int nptmask, int nptmas2)
{
   short nout,input,low_word;
   int  *maskptr,i,j;
   long sum,mul;
   float nsum;
   char *outptr,*inpptr;

/******************************************************************************/
/*                                                                            */
/*          Clear an area of the output buffer where data is not available.   */
/*                                                                            */
/******************************************************************************/
   j=(int)(nptlin);
   for (i = 0 ; i != nptmas2 ; i++) {
      output_buffer[i]=0;
      output_buffer[--j]=0;
   }
/******************************************************************************/
/*                                                                            */
/*  Calculate the number of points that the convolution can be performed on.  */
/*                                                                            */
/******************************************************************************/

   nout=nptlin-nptmask+1;

/******************************************************************************/
/*                                                                            */
/*                           Main loop of convolution.                        */
/*                                                                            */
/******************************************************************************/
   for (i=0;i != nout;i++) {
      sum=0;                             /* long integer sum zeroed */ 
      outptr=(output_buffer+i+nptmas2);  /* setup pointer to the output array */
      inpptr=(input_buffer+i);           /* setup pointer to the input data */
      maskptr=maskad;                    /* setup pointer to the mask array */

/******************************************************************************/
/*                                                                            */
/*    Generate one point of the convolution output by multiplying the mask    */
/*    by the corresponding input data.                                        */
/*                                                                            */
/******************************************************************************/

      for (j=0; j != nptmask;j++) {     /* obtain the input data */

         /* force the  input byte to be 2's complement representation */

         if (*inpptr > 127)   input = - (char) (~(*inpptr++) + 1);
         else                 input = *inpptr++;

         mul=input*(*maskptr++);          /* multiply mask by input data */
         sum += mul;              /* add result of multiplication to sum  */
      }

/******************************************************************************/
/*                                                                            */
/*    convolution sum for one point obtained, now test this result.           */
/*                                                                            */
/******************************************************************************/
      if (sum != 0 ) {
         nsum  = (((float)sum)/(norm_factor)) ;/* normalise & round sum*/
         if (nsum < 0) nsum -= 0.5;
         else nsum += 0.5;
         low_word = (short)nsum;
         if (low_word == 0) { /* if the result is zero then place correct */
            low_word = 1;     /* sign of the sum on one stored in low_word */
            if (nsum <= 0.0 ) low_word = -low_word;
         }
      }
      else low_word = 0;

/******************************************************************************/
/*                                                                            */
/*                          Output results                                    */
/*                                                                            */
/******************************************************************************/

      if (low_word > 127) { 
         *outptr = 127;                 /* output 127 */
      }
      else if (low_word < -128) { 
         *outptr = -128;                /* output -128  */
      }
      else { 
         *outptr = (char)low_word;    /* output lower byte of low_word */
      }
   }
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_array_image                                */
/*                                                                           */
/* ---  Functional Description:   Delete a 2-D array-type image of a given   */
/*                                size.                                      */
/*                                                                           */
/*      To facilitate efficient access to 2-D image elements, the image is   */
/*      organised as a sequence of 1-D arrays (of bytes) pointed to by a     */
/*      single 1-D array of pointers. Images may still be accessed using the */
/*      convenient and familiar double index as with the more normal 2-D     */
/*      arrays (e.g. image[i][j] = 0;). This implementation also allows the  */
/*      system to be implemented on machines where the maximum single        */
/*      addressable unit must be less than 64K bytes, e.g. 8086-based        */
/*      computers.                                                           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_array_image (size, image);                                    */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*      image .......... an array of pointers ("size" of them) to arrays of  */
/*                       characters (bytes); each array contains "size"      */
/*                       number of bytes.                                    */
/*                                                                           */
/* ---  Output Parameters: none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   i ......... counter.                              */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_array_image (short size, array_image_type image)
{
   int i;

   for (i=0; i<size && image[i] != NULL; i++) {
      free( (char *) image[i]);
   }
   free((char *) image);
}


/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: delete_cc                                           */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*      This subroutine deletes a Freeman chain code.  The cc pointer is    */
/*      set to NULL and the component nodes are freed.                      */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*     None                                                                 */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     delete_cc(cc)                                                        */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*     - cc ..... address of pointer to head of chain code                  */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*     - cc ..... address/pointer to head of chain code after insertion.    */
/*     - end .... address/pointer to last node in chain code after          */
/*	              insertion.	                                            */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*     None	                                                                */
/*                                                                          */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*     - p,q      temporary pointers to a node.                             */	
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*     None (by now)                                                        */
/*                                                                          */
/* --- Author: D. Vernon (TCD)                                              */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*	                                                                        */
/****************************************************************************/

void delete_cc(struct cc_node *(*cc))
{
   struct cc_node *p, *q;

   /* run cc freeing nodes */

   p = *cc;

   while (p != NULL) {
      q = p;
      p = p->flink;
      free( (char *) q);
   }
   *cc = NULL;
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_framestore_descriptor                      */
/*                                                                           */
/* ---  Functional Description:   Delete a framestore descriptor and free    */
/*                                all memory space associated with it.       */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_framestore_descriptor (framestore_descr);                     */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      framestore_descr .... pointer to the framestore descriptor to be     */
/*                            deleted.                                       */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:  none.                                            */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_framestore_descriptor (struct framestore_descriptor_type *framestore_descr)
{
   free((char *)framestore_descr);
}
/*****************************************************************************/
/*                                                                           */
/*              delete_framestore_image                                      */
/*                                                                           */
/*****************************************************************************/

void delete_framestore_image(framestore_image_type *framestore_link)
{
}
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_image_descriptor                           */
/*                                                                           */
/* ---  Functional Description:   Delete an image descriptor and free all    */
/*                                memory space associated with it.           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_image_descriptor (image_descr);                               */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      image_descr .... pointer to the image descriptor to be deleted.      */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:  none.                                            */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*      date:      26/11/86                                                  */
/*      revisor:   David Vernon (TCD)                                        */
/*      reason:    new call to delete_velocity_image                         */
/*                                                                           */
/*      date:    28/01/87                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  Introduce new depth image.                                  */
/*                                                                           */
/*      date:    26/04/87                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  Introduce new range image.                                  */
/*                                                                           */
/*****************************************************************************/

void delete_image_descriptor (struct image_descriptor_type *image_descr)
{

   if (image_descr != NULL) { 

      if (image_descr->framestore != NULL)
         delete_framestore_descriptor(image_descr->framestore);

      switch (image_descr->image_type) {
      case FRAMESTORE_IMAGE_TYPE       : delete_framestore_image(image_descr->image.framestore_link);
                                         break;
      case INTENSITY_IMAGE_TYPE        :
      case CONVOLUTION_IMAGE_TYPE      : 
      case ZERO_CROSSING_IMAGE_TYPE    : delete_array_image(image_descr->size,
                                                            image_descr->image.array_link);
                                         break;
      case CONTOUR_IMAGE_TYPE          : delete_contour_image(&(image_descr->image.contour_link));
                                         break;
      case SLOPE_IMAGE_TYPE            : delete_contour_image(&(image_descr->image.slope_link));
                                         break;
      case ORIENTATION_IMAGE_TYPE      : delete_contour_image(&(image_descr->image.orientation_link));
                                         break;
      case DEPTH_IMAGE_TYPE            : delete_contour_image(&(image_descr->image.depth_link));
                                         break;
      case DISPARITY_IMAGE_TYPE        : delete_contour_image(&(image_descr->image.disparity_link));
                                         break;
      case VELOCITY_IMAGE_TYPE         : delete_velocity_image(&(image_descr->image.velocity_link));
                                         break;
      case REGION_IMAGE_TYPE           : delete_region_image(image_descr->size,
                                                             &(image_descr->image.region_link));
                                         break;
      case RANGE_IMAGE_TYPE            : delete_int_array_image(image_descr->size,
                                                                image_descr->image.range_link);
                                         break;
      default: system_error("delete_image_descriptor - unknown image type");
               break;
      }

   free((char *)image_descr);
   }

   printf("Exiting delete_image_descriptor\n");
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_inte_array_image                           */
/*                                                                           */
/* ---  Functional Description:   Delete a 2-D array-type image of a given   */
/*                                size.                                      */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_int_array_image (size, image);                                */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*      image .......... an array of pointers ("size" of them) to arrays of  */
/*                       short ints; each array contains "size"              */
/*                       number of short ints.                               */
/*                                                                           */
/* ---  Output Parameters: none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   i ......... counter.                              */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_int_array_image (short int size, int_array_image_type image)
{
   int i;

   for (i=0; i<size && image[i] != NULL; i++) {
       free( (char *) image[i]);
 
   }
   free((char *) image);
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_region_image                               */
/*                                                                           */
/* ---  Functional Description:   Delete a region image and free all         */
/*                                memory space associated with it.           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         delete_int_array_image()                                          */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_region_image (size, region_image);                            */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... size of associated region-crossing image.           */
/*      region_image ... address of pointer to the image to be deleted.      */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:  none.                                            */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_region_image (short int size, region_image_type *region_image)
{


   if ((*region_image)->array != NULL)
      delete_int_array_image(size, (*region_image)->array);
   if ((*region_image)->tree != NULL)
      delete_region_tree( &((*region_image)->tree));
   if ((*region_image)->table != NULL)
      free ( (char *) (*region_image)->table);

   free((char *) *region_image);

   *region_image = NULL;

}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_region_tree                                */
/*                                                                           */
/* ---  Functional Description:   Delete a region tree and free all          */
/*                                memory space associated with it.           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_region_tree (region_tree);                                    */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      region_tree .... address of pointer to the base node of tree to be   */
/*                       deleted.                                            */
/*                                                                           */
/* ---  Output Parameters:  none.                                            */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_region_tree  (struct region_tree_type *(*region_tree))
{
   struct region_pointer *p, *prev_p;

   if (*region_tree != NULL)	{

      /* traverse (sub)-tree and then delete this parent node */

      p = (*region_tree)->offspring;
      while (p != NULL) {
         delete_region_tree(&(p->region_tree));
         prev_p = p;
         p = p->link;
         free((char *) prev_p);   
      }
   }

   free((char *) *region_tree);

   *region_tree = NULL;

}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_velocity_image                             */
/*                                                                           */
/* ---  Functional Description:   Delete all velocity contours in a velocity */
/*                                image.                                     */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_velocity_image (velocity_image);                              */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      velocity_image .. address of pointer to the first contour node in    */
/*                        the list of contours.                              */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      velocity_image .. = NULL                                              */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD. (26/11/86)                                */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_velocity_image (velocity_image_type *velocity_image)
{
   struct velocity_node_type *p, *prev_p;

   p = *velocity_image;
   while (p!=NULL) {
      prev_p = p;
      p = p->next;
      free((char *) prev_p->vector_magnitude);
      free((char *) prev_p->phase_angle);
      free((char *) prev_p->related_pyramid);
      free((char *) prev_p->related_image);
      free((char *) prev_p->related_contour);
      free((char *) prev_p->contour_offset);
      free((char *) prev_p);
   }
   *velocity_image = NULL;
}
/****************************************************************************/
/*                                                                          */
/* --- Program or Subprogram name: fillreg                                  */
/*                                                                          */
/* --- Functional Description: That routine fills a region into an array    */
/*                             image, with a given gray level.              */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     fillreg(seed_x,seed_y,valin,valout,_left,_right,_top,_bottom,input); */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*     seed_x, seed_y ...  x and y coordinates of a point internal to the   */
/*                         selected region                                  */
/*     valin ............  integer identifying the gray value to be         */
/*                         changed in the input image                       */
/*     valout............  integer identifying the gray value to which      */
/*                         set the pixels of the selected region in the     */
/*                         input image                                      */
/*     _left, _right, _top, _bottom ... coordinates identifying the view    */
/*                                      of interest within the image        */
/*     input ............  input (convolution) image whose region is to be  */
/*                         cleared.                                         */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*     input ............  input image whose region will be filled          */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*     p_stack ..............  stack of image points used by the filling    */
/*                         algorithm                                        */
/*     stk_n_empty ......  flag that indicates if the stack has been        */
/*                         filled up                                        */
/* --- Local Variables:                                                     */
/*     MANY!                                                                */
/*                                                                          */
/* --- Bugs:                                                                */
/*     Bug-Hunter searched....!	                                            */
/*                                                                          */
/* --- Author: G.Sandini and M.Tistarelli                                   */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*                                                                          */
/****************************************************************************/

void fillreg(int seed_x, int seed_y, int valin, int valout, int _left, int _right, int _top, int _bottom, array_image_type input_image)
{
register int dummy,yref,lxref,rxref;
int w;
int lx,rx;
int x,y;

	p_stk = NLOC_STACK;
	stk_n_empty = 1;

	x = seed_x; y = seed_y;
	yref = y; lxref = rxref = 0;
        push(x,y);

        while(stk_n_empty) {
                pop(&x,&y);

        /*      fill left       */
	        save_x(x);
	        while(((w=input_image[x][y] & CMASK) == valin) &&
			(x >= _left)) x--;

                lx = x + 1; rest_x(x); x++;
        /*      fill right      */
	        while(((w=input_image[x][y] & CMASK) == valin) &&
		(x <= _right)) x++;

	        rx = x - 1; rest_x(x);

           line(lx,rx,y,valout,input_image);

		dummy = (lx >= lxref-1) & (rx <= rxref+1);
		if(!((y == yref+1) && dummy))
			/* scan_low */
		        if( (y-1) >= _bottom) {
			        save_xy(x,y); x = lx; y--;
			        while(x <= rx) {
			                while(((w=input_image[x][y] & CMASK) != valin) &&
						 (x <= rx)) x++;
			                if(x > rx) break;
			                push(x,y);
					while(((w=input_image[x][y] & CMASK) == valin) &&
						(x <= rx)) x++;
			         }
			        rest_xy(x,y);
				}
		if(!((y == yref-1) && dummy))
				/* scan_high */
		        if( (y+1) <= _top) {
			        save_xy(x,y); x = lx; y++;
			        while(x <= rx) {
			                while(((w=input_image[x][y] & CMASK)
					   != valin) && (x <= rx)) x++;
			                if(x > rx) break;
			                push(x,y);
					while(((w=input_image[x][y] & CMASK) == valin) &&
						(x <= rx)) x++;

			                }
			        rest_xy(x,y);
				}
		yref = y; lxref = lx; rxref = rx;
     }
}


/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: follow                                              */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*     This subroutine executes the following of a single contour.	        */
/*     Starting from a given point, the alghorithm search a neighbour	    */
/*     of the pixel in a 3*3 window centered on it, counterclockwise.	    */
/*     The search is done according to the chain-code pattern, checking     */
/*     first the position "0" and all the positions with the least 	        */
/*     distance from the center of the window (i.e. 2, 4, 6), then the	    */
/*     other positions.	                                                    */
/*     If a pixel different from zero is found, then it is deleted from     */
/*     the zero-crossing image and its Freeman chain code is appended to    */
/*     the chain code list.                                                 */
/*                                                                          */
/*     The search goes on moving the 3*3 window on the last contour pixel   */
/*     found and continues pixel by pixel, until a null window (i.e. with   */
/*     all the pixels equal to zero) is found.				                */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*     append_to_cc()                                                       */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     follow(cc,end,npix,nx,ny,z_c);                                       */
/*                                                                          */
/*     cc is a pointer to the first node in the chain code;  end is a       */
/*     pointer to the last node in the chain code.                          */
/*     npix is the number of pixels found.                                  */
/*     Calling the routine nx,ny are the coordinates of the starting 	    */
/*     point at the end of the search are the coordinates of the last 	    */
/*     pixel of the contour.                                                */ 
/*     z_c is the zero-crossing array image.                                */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*     - cc ..... address of pointer to head of chain code                  */
/*      - end .... address of pointer to last node in chain code            */
/*     - *npix			number of pixels of the contour	                    */
/*     - *nx, *ny		coordinates of the starting point                   */
/*     - z_c  	      		zer-crossing array image                        */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*     - cc ..... address of pointer to head of chain code after insertion  */
/*     - end .... address of pointer to last node in chain code after       */
/*                insertion.                                                */
/*     - *npix			number of pixels of the contour                     */
/*     - *nx, *ny		coordinates of the final point	                    */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*     None	                                                                */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*     	- c_pix			pixel counter                                       */
/*     - ch_code		chain code of the pixel	                            */
/*     	- pixel  		grey level of the pixel                             */
/*     - newx, newy		coordinates of the pixel                            */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*     In some cases the contour is not followed properly and some pixels   */
/*     are lost (in particular configurations) but always preserving the    */
/*     connection.	                                                        */
/*                                                                          */
/* --- Author: G. Sandini & M. Tistarelli  UG - DIST                        */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:     16-4-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Adapt routine to generate a linked-list representation */
/*		     of the chain code rather than the original file.               */
/*                                                                          */
/*                                                                          */
/****************************************************************************/

void follow(struct cc_node *(*cc),struct cc_node *(*end), int *npix, int *nx, int *ny, array_image_type z_c)                    
{
        register int newx,newy;
        int c_pix,pixel,ch_code;

	newx = *nx; newy = *ny;
	c_pix = *npix;

	do
	{

/* first check the positions with least distance from the center of a
3*3 window. It is done to preserve continuity */

	  newx++ ; ch_code = 0;	/* position 0 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code); 
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
				
	  newy++ ; newx-- ; ch_code += 2;	/* position 2 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
				
	  newx-- ; newy-- ; ch_code += 2;	/* position 4 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
				
	  newx++ ; newy-- ; ch_code += 2;	/* position 6 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}

/* check the positions with greater distance */
				
	  newx++ ; newy += 2; ch_code = 1;	/* position 1 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
				
	  newx -= 2; ch_code += 2;	/* position 3 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
				
	  newy -= 2; ch_code += 2;	/* position 5 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}

	  newx += 2; ch_code += 2;	/* position 7 */
	  if ((pixel=z_c[newx][newy])!=0)
		{
		append_to_cc(cc,end,ch_code);
                c_pix++;
		z_c[newx][newy]=ZERO;
		continue;
		}
	 }	
	while(pixel != 0);
	
	newy++ ; newx-- ;	/* coordinates of the last pixel */
	
	*nx = newx; *ny = newy; *npix = c_pix;	/* number of pixels of 
							the contour */
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_array_image                              */
/*                                                                           */
/* ---  Functional Description:   Generate a 2-D array-type image of a given */
/*                                size.                                      */
/*                                                                           */
/*      To facilitate efficient access to 2-D image elements, the image is   */
/*      organised as a sequence of 1-D arrays (of bytes) pointed to by a     */
/*      single 1-D array of pointers. Images may still be accessed using the */
/*      convenient and familiar double index as with the more normal 2-D     */
/*      arrays (e.g. image[i][j] = 0;). This implementation also allows the  */
/*      system to be implemented on machines where the maximum single        */
/*      addressable unit must be less than 64K bytes, e.g. 8086-based        */
/*      computers.                                                           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*      malloc()                                                             */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      generate_array_image (size, image);                                  */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      image .......... an array of pointers ("size" of them) to arrays of  */
/*                       characters (bytes); each array contains "size"      */
/*                       number of bytes. Note the address of the array is   */
/*                       actually returned.                                  */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   i,j ....... counters.                             */
/*                         success ... a flag.                               */
/*                         nbytes .... temporary storage.                    */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      13/4/87   M.T.                                            */
/*      revision:  Added image clearing after the allocation of the image    */
/*      reason:  It seems reasonable to have a the new image empty           */
/*                                                                           */
/*****************************************************************************/

int generate_array_image (short int size, array_image_type *image)
{
   int i,j;
   char success;
   unsigned nbytes;

   /*                    */
   /* generate new image */
   /*                    */

   /* first generate the array of pointers */

   success = TRUE;

   nbytes = sizeof(image_row) * size;
   if ((*image = (array_image_type) malloc(nbytes)) == NULL) {
      printf("unsuccessful memory allocation (array image: pointer array)\n");
      success = FALSE;
   }
   else {

      for (i=0; i<size; i++)
         (*image)[i] = NULL;

      /* now generate the actual image storage, one row at a time */

      for (i=0; i<size && success == TRUE; i++) {

         (*image)[i] = (image_row) malloc(size);

         /* check if allocation was successful */

         if ((*image)[i] == NULL) {
            printf("unsuccessful memory allocation (array image)\n");
            success = FALSE;
            break;
         }
      }
   }
   if (success == FALSE) {
       delete_array_image(size,*image); 
       *image = NULL;
   } else {
       for (i=0; i<size; i++)
          for (j=0; j<size; j++) (*image)[i][j] = 0;
   }
   return(0);
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   delete_contour_image                              */
/*                                                                           */
/* ---  Functional Description:   Delete all contours in a contour image.    */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         free()                                                            */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      delete_contour (contour_image);                                      */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      contour_image .. address of pointer to the first contour node in the */
/*                       list of contours.                                   */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      contour_image .. = NULL                                              */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   none.                                             */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void delete_contour_image (contour_image_type *contour_image)
{
   struct contour_node_type *p, *prev_p;

   p = *contour_image;
   while (p!=NULL) {
      prev_p = p;
      p = p->next;
      free((char *) prev_p->value);
      free((char *) prev_p);
   }
   *contour_image = NULL;
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_contour_node                             */
/*                                                                           */
/* ---  Functional Description:   Generate and initialise a contour node.    */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         malloc()                                                          */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      ptr = generate_contor_node (x,y,length);                             */
/*                                                                           */
/*         Function returns a pointer (e.g. ptr) to the contour node.        */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      x, y ........... coordinates of origin of contour                    */
/*                                                                           */
/*      length ......... length of contour.                                  */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      Function returns pointer new contour node.                           */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   p ... temporary pointer to contour node.          */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

struct contour_node_type *generate_contour_node (int x, int y, int length)
{
   struct contour_node_type *p;

   /* generate new image descriptor */

   if ((p = (struct contour_node_type *) malloc(CONTOUR_NODE_SIZE)) == NULL) {
      system_error("unsuccessful memory allocation (contour node)");
      return(NULL);
   }
   else {

      p->start_x = x;
      p->start_y = y;
      p->length = length;
      p->next = NULL;
      p->prev = NULL;
      p->descriptor_link = NULL;

      /* generate integer array for the chain code */
 
      if ((p->value = (int *) malloc(length * sizeof(int))) == NULL) {
         system_error("unsuccessful memory allocation (contour array)");
         return(NULL);
      }
      return(p);
   }
}
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_framestore_descriptor                    */
/*                                                                           */
/* ---  Functional Description:   Generate and initialise a framestore       */
/*                                descriptor.                                */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         malloc()                                                          */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      ptr = generate_framestore_descriptor (framestore_id, access_type,    */
/*                                            mask, number_of_bits, size);   */
/*                                                                           */
/*      Function returns a pointer (e.g. ptr) to the framestore descriptor.  */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      framestore_id .. internal data-type identification number.           */
/*                                                                           */
/*      access_type..... internal data-type identification number.           */
/*                                                                           */
/*      mask ........... image bit-plane mask (type integer => 32 bits).     */
/*                                                                           */
/*      number_of_bits . number of bits in image.                            */
/*	                                                                         */
/*      size ........... image size.                                         */
/*	                                                                         */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      Function returns pointer new framestore descriptor.                  */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: microeye_data.                                    */
/*                                                                           */
/* ---  Local Variables:   p ... temporary pointer to framestore_descriptor  */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      10/10/86   Massimo Tistarelli (at TCD)                    */
/*      revision: Added generation of Virtual Frame Store                    */
/*      reason: Use of VIS with systems not capable of image acquisition     */
/*                                                                           */
/*      date:      29/10/86   Massimo Tistarelli (at TCD)                    */
/*      revision: Now the VFS file is checked to verify if it exists         */
/*                                                                           */
/*      date:      31/10/86   Massimo Tistarelli (at TCD)                    */
/*      revision: Added generation of Eidobrain framestore. Now also the     */
/*      size of the image is passed as imput parameter.                      */
/*                                                                           */
/*      date:    04/04/88                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  New parameter: vfs_data                                     */
/*                                                                           */
/*      date:    03/11/88                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  New field in the framestore descriptor, serial_number,      */
/*               is initialised to the current number of (identical)         */
/*               framestore plus one.                                        */
/*                                                                           */
/*      date:    02/03/89                                                    */
/*      revisor: Mairead Flanagan                                            */
/*      reason:  Included PCVISIONplus option                                */
/*                                                                           */
/*****************************************************************************/

struct framestore_descriptor_type *generate_framestore_descriptor (short int framestore_id, short int access_type,long int mask, short int number_of_bits, short int size, struct VFS_type *vfs_data)
{
   struct framestore_descriptor_type *p;
   extern struct microeye_type microeye_data;
   extern struct pcvision_type pcvision_data;
   extern struct pcplus_type pcplus_data;
   extern struct fg100at_type fg100at_data;
   extern int number_of_framestores;

   /* generate new framestore descriptor */

   if ((p = (struct framestore_descriptor_type *) malloc(FRAMESTORE_DESCRIPTOR_SIZE)) == NULL) {
      system_error("unsuccessful memory allocation (framestore descriptor)");
      return(NULL);
   }
   else {

      if (framestore_id != VFS) {

         /* update the number of framestores in the system      */
         /* initialise the framestore serial number accordingly */

         number_of_framestores++;
         p->serial_number = number_of_framestores;

      }
         
      p->framestore_id = framestore_id;
      p->access_type = access_type;
      p->mask = mask;
      p->number_of_bits = number_of_bits;
#ifdef XENIX
      if(open_io() == -1) return(NULL);
#endif
      switch (framestore_id) {
#ifdef MEYE_DEV
      case MICROEYE    : p->framestore_data.microeye = &microeye_data;
                         break;
#endif
#ifdef PCVIS_DEV
      case PCVISION    : 
                         p->framestore_data.pcvision = &pcvision_data;
                         break;
#endif
#ifdef PCPLUS_DEV
      case PCPLUS      : 
                         p->framestore_data.pcplus = &pcplus_data;
                         break;
#endif
#ifdef VDS701_DEV
      case VDS_701     :
                         system_error("VDS701 framestore is not supported... yet");
                         break;
#endif
#ifdef XW_BITMAP
      case X_WINDOW   :
			if((open_Xwindow_device()) == -1) return(NULL);
   			if ((p->framestore_data.Xwindow = (struct Xwindow_type *) malloc(sizeof(struct Xwindow_type))) == NULL) {
      				system_error("unsuccessful memory allocation (framestore descriptor)");
      			 	return(NULL);
   			}
			p->framestore_data.Xwindow->size = size;
			blank_menu();
			if((create_Xwindow(size)) == -1) return(NULL);
			break;
#endif
#ifdef EIDO_DEV
      case EIDOBRAIN   :
			if((check_eido_device()) == -1) return(NULL);
   			if ((p->framestore_data.eidobrain = (struct eido_type *) malloc(sizeof(struct eido_type))) == NULL) {
      				system_error("unsuccessful memory allocation (framestore descriptor)");
      			 	return(NULL);
   			}
			p->framestore_data.eidobrain->size = size;
			blank_menu();
			if((select_eido_parameters(p->framestore_data.eidobrain)) == -1) return(NULL);
			break;
#endif
#ifdef VICOM_DEV
      case VICOM       : break;
#endif
#ifdef FG100_DEV
      case FG100AT     : p->framestore_data.fg100at = &fg100at_data;
                         break;
#endif
      case VFS	       :
			                p->framestore_data.vfs = vfs_data;	 
	                      break;

      default: system_error("generate_framestore_descriptor - unknown framestore type");

               break;
      }
      return(p);
   }
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_image_descriptor                         */
/*                                                                           */
/* ---  Functional Description:   Generate and initialise an image           */
/*                                descriptor.                                */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         malloc()                                                          */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      ptr = generate_image_descriptor (image_type, description, size,      */
/*                               level_number, window, mask, number_of_bits, */
/*                               framestore, pyramid_link, image_link);      */
/*                                                                           */
/*         Function returns a pointer (e.g. ptr) to the image descriptor.    */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      image_type ..... internal data-type label.                           */
/*                                                                           */
/*      description .... pointer to alphanumeric description of image.       */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*      level_number ... level in pyramid that image occupies.               */
/*                                                                           */
/*      window ......... structure containing window specification;          */
/*                       coordinates of bottom left-hand and top right-hand  */
/*                       corners of window (refer to datadef.h for complete  */
/*                       specification).                                     */
/*                                                                           */
/*      mask ........... image bit-plane mask (type integer => 32 bits).     */
/*                                                                           */
/*      number_of_bits . number of bits in image.                            */
/*                                                                           */
/*      framestore ..... pointer to framestore descriptor (NULL if not       */
/*                       applicable).                                        */
/*                                                                           */
/*      pyramid_link ... pointer to pyramid descriptor (parent structure).   */
/*                                                                           */
/*      image_link ..... pointer to appropriate (type of) image.             */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      Function returns pointer new image descriptor.                       */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   p ... temporary pointer to image_descriptor;      */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:    28/01/87                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  Introduce new depth image.                                  */
/*                                                                           */
/*      date:    26/04/87                                                    */
/*      revisor: David Vernon                                                */
/*      reason:  Introduce new range image.                                  */
/*                                                                           */
/*****************************************************************************/

struct image_descriptor_type *generate_image_descriptor
    (short int image_type, char description[], short int size, short int level_number, struct window_type *window, long int mask,
     short int number_of_bits, struct framestore_descriptor_type *framestore, struct pyramid_descriptor_type *pyramid_link, union image_pointer_type *image_link)
  
{
   struct image_descriptor_type *p;

   /* generate new image descriptor */

   if ((p = (struct image_descriptor_type *) malloc(IMAGE_DESCRIPTOR_SIZE)) == NULL) {
      system_error("unsuccessful memory allocation (image descriptor)");
      return(NULL);
   }
   else {

      p->image_type = image_type;
      strcpy(p->description,description);
      p->size = size;
      p->level_number = level_number;
      p->window.x1 = window->x1;
      p->window.y1 = window->y1;
      p->window.x2 = window->x2;
      p->window.y2 = window->y2;
      p->mask = mask;
      p->number_of_bits = number_of_bits;
      p->framestore = framestore;
      p->pyramid = pyramid_link;

      switch (image_type) {
      case FRAMESTORE_IMAGE_TYPE       : p->image.framestore_link =
                                         image_link->framestore_link;
                                         break;
      case INTENSITY_IMAGE_TYPE        : p->image.array_link =
                                         image_link->array_link;
                                         break;
      case CONVOLUTION_IMAGE_TYPE      : p->image.array_link =
                                         image_link->array_link;
                                         break;
      case ZERO_CROSSING_IMAGE_TYPE    : p->image.array_link =
                                         image_link->array_link;
                                         break;
      case CONTOUR_IMAGE_TYPE          : p->image.contour_link =
                                         image_link->contour_link;
                                         break;
      case SLOPE_IMAGE_TYPE            : p->image.slope_link =
                                         image_link->slope_link;
                                         break;
      case ORIENTATION_IMAGE_TYPE      : p->image.orientation_link =
                                         image_link->orientation_link;
                                         break;
      case DEPTH_IMAGE_TYPE            : p->image.depth_link =
                                         image_link->depth_link;
                                         break;
      case DISPARITY_IMAGE_TYPE        : p->image.disparity_link =
                                         image_link->disparity_link;
                                         break;
      case VELOCITY_IMAGE_TYPE         : p->image.velocity_link =
                                         image_link->velocity_link;
                                         break;
      case REGION_IMAGE_TYPE           : p->image.region_link =
                                         image_link->region_link;
                                         break;
      case RANGE_IMAGE_TYPE            : p->image.range_link =
                                         image_link->range_link;
                                         break;
      default: system_error("generate_image_descriptor - unknown image type");
               break;
      }
      return(p);
   }
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_int_array_image                          */
/*                                                                           */
/* ---  Functional Description:   Generate a 2-D short int array image of    */
/*                                a given size.                              */
/*                                                                           */
/*      To facilitate efficient access to 2-D image elements, the image is   */
/*      organised as a sequence of 1-D arrays (of short) pointed to by a     */
/*      single 1-D array of pointers. Images may still be accessed using the */
/*      convenient and familiar double index as with the more normal 2-D     */
/*      arrays (e.g. image[i][j] = 0;). This implementation also allows the  */
/*      system to be implemented on machines where the maximum single        */
/*      addressable unit must be less than 64K bytes, e.g. 8086-based        */
/*      computers.                                                           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*      malloc()                                                             */
/*      system_error()                                                       */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      generate_int_array_image (size, image);                              */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      image .......... an array of pointers ("size" of them) to arrays of  */
/*                       short integers; each array contains "size"          */
/*                       number of shorts. Note the address of the array is  */
/*                       actually returned.                                  */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:   i,j ....... counters.                             */
/*                         success ... a flag.                               */
/*                         nbytes .... temporary storage.                    */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      13/4/87   M.T.                                            */
/*      revision:  Added image clearing after the allocation of the image    */
/*      reason:  It seems reasonable to have a the new image empty           */
/*                                                                           */
/*****************************************************************************/

int generate_int_array_image (short int size, int_array_image_type *image)
{
   int i,j;
   char success;
   unsigned nbytes;

   /*                    */
   /* generate new image */
   /*                    */

   /* first generate the array of pointers */

   success = TRUE;

   nbytes = sizeof(int_image_row) * size;
   if ((*image = (int_array_image_type) malloc(nbytes)) == NULL) {
      system_error("unsuccessful memory allocation (int array image: pointer array)");
      success = FALSE;
   }
   else {

      for (i=0; i<size; i++)
         (*image)[i] = NULL;

      /* now generate the actual image storage, one row at a time */

      for (i=0; i<size && success == TRUE; i++) {

         nbytes = sizeof(short int) * size;
         (*image)[i] = (int_image_row) malloc(nbytes);

         /* check if allocation was successful */

         if ((*image)[i] == NULL) {
            system_error("unsuccessful memory allocation (int array image)");
            success = FALSE;
            break;
         }
      }
   }
   if (success == FALSE) {
       delete_int_array_image(size,*image); 
       *image = NULL;
   } else {
       for (i=0; i<size; i++)
          for (j=0; j<size; j++) (*image)[i][j] = 0;
   }
   return(0);
}
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   generate_region_image                             */
/*                                                                           */
/* ---  Functional Description:   Generate a region image comprising         */
/*                                region_crossing_image, region_table, and   */
/*                                region_tree.                               */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*      generate_int_array_image()                                           */
/*      malloc()                                                             */
/*      system_error()                                                       */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      ptr =  generate_region_image (size);                                 */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      size ........... = rows = columns = 1024 / integer value.            */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      function returns pointer to  region_image_type                       */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

region_image_type generate_region_image (short int size)
{
   region_image_type p;

   /*                    */
   /* generate new image */
   /*                    */


   if ((p = (region_image_type) malloc(REGION_IMAGE_SIZE)) == NULL) {
      system_error("unsuccessful memory allocation (region image)");
      return(NULL);
   }
   else {

      p->table = NULL;
      p->tree = NULL;
		p->number_of_regions = 0;

      /* generate integer array for region crossings */
 		
      generate_int_array_image(size,&(p->array));

      return(p);
      
   }

}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   insert_contour                                    */
/*                                                                           */
/* ---  Functional Description:   Insert a contour node in a linked list of  */
/*                                contour nodes (i.e., a contour image).     */
/*      The contour node is inserted such that the linked-list remains       */
/*      ordered by contour length (longest contours first).                  */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*         malloc()                                                          */
/*         system_error()                                                    */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      insert_contour (contour_image, contour_node);                        */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      contour_image .. address of pointer to head of the contour list      */
/*                       (may be NULL).                                      */
/*                                                                           */
/*      contour_node ... pointer to the contour node to be inserted.         */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      contour_image .. address of pointer to head of the contour list      */
/*                       after insertion                                     */
/*                                                                           */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/*      p, q ........ temporary pointers to contour nodes.                   */
/*      finished .... flag.                                                  */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void insert_contour (contour_image_type *contour_image, struct contour_node_type *contour_node)  
//contour_image_type *contour_image; /* this is equivalent to struct contour_node_type *(*contour_image); */
{

   extern char *malloc();
   struct contour_node_type *p, *q;
   int finished;

   /* run the list of contours and insert */
   /* this new contour before the first   */
   /* node with a larger contour length   */
   
   if (*contour_image == NULL) {

      /* empty list of contours: this one becomes the first in the list */

      contour_node->next = NULL;
      contour_node->prev = NULL;
      contour_node->descriptor_link = NULL;

      *contour_image = contour_node;

   }
   else if ((*contour_image)->length < contour_node->length) {

      /* insert new pyramid at head of the list */

      contour_node->next = *contour_image;
      contour_node->prev = NULL;
      contour_node->descriptor_link = NULL;

      *contour_image = contour_node;

   }
   else {

      /* run */
         
      p = *contour_image;
      q = (*contour_image)->next;
      finished = FALSE;
      do {
         if (q == NULL)
            finished = TRUE;
         else if (q->length < contour_node->length)
            finished = TRUE;
         else {
            /* proceed */
            p = q;
            q = q->next;
         }
      } while (finished == FALSE);

      /* insert after p */

      contour_node->next = p->next;
      if (contour_node->next != NULL)
         contour_node->next->prev = contour_node;
      contour_node->prev = p;
      p->next = contour_node;

      contour_node->descriptor_link = NULL;

   }
}
/****************************************************************
* 
*  Routine Name: laplacianOfGaussianWrapper
* 
*      Purpose: provide an interface to the original VIS 
*      - laplacian_of_gaussian(), 
*      - zero_crossings()
*      - select_significant_contours()
*      - zc_plot()
*
*      functions
*                        
*      Input:  intensity        - pointer to DVimage (INT) 
*              sigma            - float parameter; standard_deviation of Gaussian 
*              significantZC    - bool flag: select significant zero-crossing contours if true (and if noZC is false)
*              noZC             - bool flag: skip zero-crossing generation if true
*
*      Output: log              - pointer to DVimage (INT) with Laplacian of Gaussian result
*              zc               - pointer to DVimage (INT) with zero-crossings result
*              plot             - pointer to DVimage (INT) with zero-crossings superimposed on input intensity image
*
* Written By: David Vernon
* Date:       15 February 2012
*
* Modifications: 
* 
****************************************************************/
 
void laplacianOfGaussianWrapper (DVimage *intensity, float std_dev, bool significantZC, bool noZC, bool noPlot, DVimage *log, DVimage *zc, DVimage *plot)
{

   int width, height, depth; 
   int x, y;
   float temp;
   unsigned char pixel_value;
   char debug;
  
   // VIS variables

   short int size, level_number;
   long int mask;
   short int number_of_bits;

   static array_image_type image_0, image_1, image_2;
   static int_array_image_type image_7;
   static struct image_descriptor_type *i_d_0 = NULL;
   static struct image_descriptor_type *i_d_1 = NULL;
   static struct image_descriptor_type *i_d_2 = NULL;
   static struct image_descriptor_type *i_d_3 = NULL;
   static struct image_descriptor_type *i_d_4 = NULL;
   static struct image_descriptor_type *i_d_5 = NULL;
   static struct image_descriptor_type *i_d_6 = NULL;
   static struct image_descriptor_type *i_d_7 = NULL;
   static union image_pointer_type     image_link_0,
                                       image_link_1,
                                       image_link_2,
                                       image_link_3,
                                       image_link_4,
                                       image_link_5,
                                       image_link_6,
                                       image_link_7;
   static struct framestore_descriptor_type *f_s_null_ptr = NULL;
   static struct pyramid_descriptor_type *p_d_null_ptr = NULL;
   struct window_type window;
   double sigma;
   int i,j;  
  
   /* set debug flags */

   debug = FALSE;
 
   if (debug) printf("laplacianOfGaussianWrapper: entering \n");  
   if (debug) printf("laplacianOfGaussianWrapper: standard_deviation = %f\n",std_dev);  
 
   sigma = std_dev;

   if (intensity != NULL) {

      intensity->get_size(&width,&height);
      size = max(width,height);
      level_number = 1024 / size;
      depth = intensity->get_image_mode();
      mask = 0L;
      number_of_bits = 8;

      if (debug) cout << "laplacianOfGaussianWrapper: generating VIS data-structures " << endl;

      if (image_link_0.array_link == NULL) 
         generate_array_image(size,&(image_link_0.array_link));

      if (image_link_1.array_link == NULL) 
         generate_array_image(size,&(image_link_1.array_link));

      if (image_link_2.array_link == NULL) 
         generate_array_image(size,&(image_link_2.array_link));

      delete_contour_image(&(image_link_3.contour_link));
      delete_contour_image(&(image_link_4.slope_link));
      delete_contour_image(&(image_link_5.orientation_link));

      if (image_link_6.array_link == NULL) 
         generate_array_image(size,&(image_link_6.array_link));

      if (image_link_7.array_link == NULL) 
         generate_int_array_image(size,&(image_link_7.range_link));

      if (i_d_0 == NULL)
         i_d_0 = generate_image_descriptor(INTENSITY_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_0);  
      
      if (i_d_1 == NULL)
         i_d_1 = generate_image_descriptor(CONVOLUTION_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_1);
 
      if (i_d_2 == NULL)
        i_d_2 = generate_image_descriptor(ZERO_CROSSING_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_2);

      if (i_d_3 == NULL)
         i_d_3 = generate_image_descriptor(CONTOUR_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_3);

     if (i_d_4 == NULL)
        i_d_4 = generate_image_descriptor(SLOPE_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_4);

     if (i_d_5 == NULL)
         i_d_5 = generate_image_descriptor(ORIENTATION_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_5);

      if (i_d_6 == NULL)
        i_d_6 = generate_image_descriptor(ZERO_CROSSING_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_6);

      if (i_d_7 == NULL)
        i_d_7 = generate_image_descriptor(RANGE_IMAGE_TYPE,
                                        "dummy descriptor",
                                        size,
                                        level_number,
                                        &window,
                                        mask,
                                        number_of_bits,
                                        f_s_null_ptr,
                                        p_d_null_ptr, 
                                        &image_link_7);


      image_0 = i_d_0->image.array_link;

      if (debug) cout << "laplacianOfGaussianWrapper: copying input image " << endl;

      for (i = 0; i < width; i++) {
         for (j = 0; j < height; j++) {

            intensity->get_pixel(i,j,&pixel_value, 0);

            if (depth == COLOUR_IMAGE) {              // convert to grey-scale if necessary
                temp = pixel_value;
                intensity->get_pixel(i,j,&pixel_value, 1);
                temp += pixel_value;
                intensity->get_pixel(i,j,&pixel_value, 2);
                temp += pixel_value;
                temp = temp / 3;
                pixel_value = (unsigned char) temp;	
			   }

			   image_0[i][j] = pixel_value;	      
         }
      }

      set_view(0,width,height);

      if (debug) cout << "laplacianOfGaussianWrapper: executing laplacian_of_gaussian() " << endl;

      laplacian_of_gaussian(0,0,0,i_d_0,
                            0,0,0,i_d_1,
                            sigma);

      if (debug) cout << "laplacianOfGaussianWrapper: executing zero-crossings() " << endl;

      zero_crossings(0, 0, 0, i_d_1,
                     0, 0, 0, i_d_2,
                     sigma,
                     1,     /* true, we want to generate the contour-based images       */
                     i_d_3, i_d_4, i_d_5, 
                     0,     /* false, we don't want the feature values in the zc image  */
                     i_d_6);

      if (significantZC) {

         if (debug) cout << "laplacianOfGaussianWrapper: executing select_significant_contours() " << endl;

         select_significant_contours(i_d_3);

         /* transfer selected contours to zero-crossing image */

         if (debug) cout << "laplacianOfGaussianWrapper: executing zc_plot() " << endl;

         zc_plot(0, 0, 0, i_d_4,    // plot slope
                 0, 0, 0, i_d_2,  
                 255);              // 0 => use slope value; otherwise plot with value given - 255 in this case
      }

      // slope_interp(0, 0, 0, i_d_4, 0, 0, 0, i_d_7);  // interpolation between slope contours gives reasonable results 
                                                        // but the LoG image is better

      /* and now copy the images to the outputs */

     if (debug) cout << "laplacianOfGaussianWrapper: copying Laplacian of Gaussian image to output " << endl;

      image_1 = i_d_1->image.array_link;

      for (x = 0; x < width; x++) {
         for (y = 0; y < height; y++) {  
            pixel_value = image_1[x][y];                        // on-centre/off-surround response has high value
                                                                // off-centre/on-surround response has low value
                                                                // this is the normal mode and yields a maximum on the bright side of a zero-crossing

            //pixel_value = abs((int)image_1[x][y] - (int)128); // on-centre/off-surround and off-centre/on-surround responses
                                                                // both have high values
                                                                // this yields maxima on either side of a zero-crossing

            log->put_pixel(x, y, pixel_value, 0);            			
            if (depth == COLOUR_IMAGE) {           
               log->put_pixel(x, y, pixel_value, 1);
               log->put_pixel(x, y, pixel_value, 2);
            }
         } 
      }
      log->contrast_stretch();
           
      if (!(noZC)) {

         if (debug) cout << "laplacianOfGaussianWrapper: copying zero-crossings to output" << endl;

         image_2 = i_d_2->image.array_link;
         //image_7 = i_d_7->image.range_link; // don't use the interpolated slope image

         for (x = 0; x < width; x++) {
            for (y = 0; y < height; y++) {
               pixel_value = image_2[x][y];
               // pixel_value = (unsigned char) image_7[x][y]; 
               zc->put_pixel(x, y, pixel_value, 0);            			
               if (depth == COLOUR_IMAGE) {           
                   zc->put_pixel(x, y, pixel_value, 1);
                   zc->put_pixel(x, y, pixel_value, 2);
               }
            } 
         } 
         zc->contrast_stretch();
      }

      if (!(noPlot)) {

         if (debug) cout << "laplacianOfGaussianWrapper: plotting zero-crossings on input image" << endl;

         plot->initialize();

         image_2 = i_d_2->image.array_link;

         for (x = 0; x < width; x++) {
            for (y = 0; y < height; y++) {
               if (image_2[x][y] != 0) {
                  pixel_value = 255;
                  plot->put_pixel(x, y, pixel_value, 0);            			
                  if (depth == COLOUR_IMAGE) {           
                     plot->put_pixel(x, y, pixel_value, 1);
                     plot->put_pixel(x, y, pixel_value, 2);
                  }
               }
               else {
                  intensity->get_pixel(x, y, &pixel_value,0);           
                  plot->put_pixel(x, y, pixel_value, 0);            			
                  if (depth == COLOUR_IMAGE) {           
                     intensity->get_pixel(x, y, &pixel_value,1); 
                     plot->put_pixel(x, y, pixel_value, 1);
                     intensity->get_pixel(x, y, &pixel_value,2); 
                     plot->put_pixel(x, y, pixel_value, 2);
                  }
               }
            }
         } 
      }
 /* 
      delete_image_descriptor(i_d_0); i_d_0 = NULL;
      delete_image_descriptor(i_d_1); i_d_1 = NULL;
      delete_image_descriptor(i_d_2); i_d_2 = NULL;
      delete_image_descriptor(i_d_3); i_d_3 = NULL;
      delete_image_descriptor(i_d_4); i_d_4 = NULL;
      delete_image_descriptor(i_d_5); i_d_5 = NULL;
      delete_image_descriptor(i_d_6); i_d_6 = NULL;
      delete_image_descriptor(i_d_7); i_d_7 = NULL;
*/
   }
         
   if (debug) printf("laplacianOfGaussianWrapper: exiting\n");

   return;
   
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: laplacian_of_gaussian                               */
/*                                                                           */
/* ---  Functional Description: laplacian_of_gaussian performs the           */
/*      convolution of an image with the Laplacian of Gaussian operator.     */
/*      The program is a modified version of Con256.c written by             */
/*      Giulio Sandini. The modifications consist of replacing the           */
/*      PDP-11 assembler routines by C modules and removing other machine    */ 
/*      dependent code. The program has also been expanded so that images    */
/*      up to 1024 by 1024 can be processed. Temporary memory is now         */
/*      allocated and used for the storage of intermediate data.             */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*      readlin() .. read a row from an image.                               */
/*      readcol() .. read a column from an image.                            */
/*      writelin() .. write a row to an image.                               */
/*      writecol() .. write a column to an image.                            */
/*      conv1() .. convolve a mask with data with its sign removed.          */
/*      conv2() .. convolve a mask with data without its sign removed.       */
/*      generate_array_image() .. dynamic allocation of image workspace.     */
/*      delete_array_image() .. dynamic deallocation of image workspace.     */
/*      stdio.h, math.h                                                      */
/*      dsdef.h                                                              */
/*                                                                           */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*     i = laplacian_of_gaussian(view_number_1, x_1, y_1,image_descriptor_1, */
/*                              view_number_2, x_2, y_2, image_descriptor_2, */
/*                              sigma);                                      */
/* ---  Input Parameters:                                                    */
/*         view_number_1 .. source view number.                              */
/*         x_1, y_1      .. source view start coordinates.                   */
/*         image_descriptor_1 .. pointer to the structure containing source  */
/*                               array.                                      */
/*         view_number_2 .. destination view number.                         */
/*         x_2, y_2      .. destination view start coordinates.              */
/*         image_descriptor_2 .. pointer to the structure containing         */
/*                               destination array.                          */
/*         sigma         .. amplitude of the positive lobe of the D2G mask.  */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*            i= 0 sucessful completion of laplacian of gaussian.            */
/*            i= -1 unsucessful completion of laplacian of gaussian.         */
/*                                                                           */
/* ---  Global Parameters:                                                   */
/*                                                                           */
/*      struct view_type views[]                                             */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/*      static int D2gauint[100],gauint[100] arrays containing masks.        */
/*      static float D2gau[100],gau[100] arrays used in the generation of    */
/*                                   the masks.                              */ 
/*      static char inpb[1024],outb[1024] arrays required for the storage    */
/*                               of row and column data.                     */
/*  Plus many more.                                                          */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: Giulio Sandini                                               */
/*                                                                           */
/* ---  Revisions: S.O'N. Remove machine dependent code for use with V.I.S.. */
/*                                                                           */
/*      date:      29/1/86  D.V.                                             */
/*      revision:  char bytes forced to be 2's complement integer.           */
/*      reason:    AT char representation is unsigned integer 0-255.         */
/*                                                                           */
/*      date:      6/4/87   M.T.                                             */
/*      revision:  Zero values in convolution forced to 0+ or 0- (129 or 127)*/
/*      reason:    Zero is a singular value in convolution that leads to     */
/*                 errors in the boundary detection from regions.            */
/*                                                                           */
/*      date:      10/08/87 M.T.                                             */
/*      revisor: Massimo Tistarelli                                          */
/*      revision:  Added code for execution on Eidobrain.......              */
/*                                                                           */
/*      date: 16/12/88 Paul                                                  */
/*      revision: added code for execution on TCAM                           */
/*                                                                           */
/*      date: 22-04-89 Paul                                                  */
/*      revision: added code for execution on RVP                            */
/*                                                                           */
/*****************************************************************************/
int laplacian_of_gaussian(int view_number_1, int x_1, int y_1, struct image_descriptor_type *image_descriptor_1,
                          int view_number_2, int x_2, int y_2, struct image_descriptor_type *image_descriptor_2,
                          double sigma)
{
   extern struct view_type views[];
   extern short check_eido_device(), eido_convolution(),
                close_eido_device();

   short size,sizex,sizey,opsizex;
   short x_limit,y_limit;
   int sumint    = 1;         /* initialization */
   int normG     = 0;
   int normD2G   = 0;
   double sumG    = 0;
   double sumD2G  = 0;
   double sumqG   = 0;
   double sumqD2G = 0;
   double delta,func,arg;
   double inc,x2;
   int iarg,add;
   int val, valin, valout;
   int hampli,i;
   int npt;
   int nptmask,nptmas2;
   int *maskad;
   register int x,y;
   static int D2gauint[100],gauint[100];   /* static variables */
   static double D2gau[100],gau[100];
   double tD2g;
   static char inpb[1024],outb[1024];
   array_image_type image_store1,image_store2;

/*****************************************************************************/
/*                                                                           */
/*                   Test parameters passed.                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef TDS 
   if(A100ok(sigma)) 
    return( A100laplacian_of_gaussian(view_number_1,x_1,y_1,
            image_descriptor_1,view_number_2,x_2,y_2,image_descriptor_2,sigma) );
#endif

   if (image_descriptor_1 == NULL || image_descriptor_2 == NULL) return (-1);

   if (view_number_1 < 0 || view_number_2 > MAX_NUMBER_VIEWS ||
       view_number_2 < 0 || view_number_2 > MAX_NUMBER_VIEWS ) return (-1);

/*                                                                           */
/*          Test views to see if they are the same extent.                   */
/*                                                                           */
   if (views[view_number_1].x != views[view_number_2].x ||
       views[view_number_1].y != views[view_number_2].y) return (-1);

   if (image_descriptor_1->image_type != INTENSITY_IMAGE_TYPE ||
       image_descriptor_2->image_type != CONVOLUTION_IMAGE_TYPE ) return(-1);

/*****************************************************************************/
/*                                                                           */
/*  determine half the number of points for the convolution masks            */
/*                                                                           */
/*****************************************************************************/

   arg = sigma*sigma;                  
   delta  =  1 / arg;
   hampli=0;
   i=0;
   do {
      x2   = (float)(i*i);
      arg  = -delta*x2;
      func =  exp(arg);
      tD2g =  func*(1-2*x2*delta); 
      if (tD2g < 0.0 ) tD2g = tD2g*127.- .5;
      else  tD2g = tD2g*127.+.5;
      i++;
   }  while ((int)tD2g != 0);

   hampli=i-2; /* half the mask size determined */
/*
   printf("sigma = %f\n",sigma);
   printf("Half amplitude of masks : %d\n ",hampli);       
*/
   iarg   = hampli - 1;

/*****************************************************************************/
/*                                                                           */
/*    calculus of the Gaussian and Laplacian of Gaussian operators           */
/*    calculates only an half of the operators                               */
/*                                                                           */
/*****************************************************************************/

   for ( i=0; i<=hampli ; i++ ) {

      x2   = (float)(i*i);
      arg  = -delta*x2;
      func =  exp(arg);
      iarg ++;
      gau[iarg]   = func;                  /* Gaussian 'G' */
      D2gau[iarg] =  func*(1-2*x2*delta);  /* Laplacian of Gaussian 'D2G' */
   }

   npt=2 * hampli + 1;   /* the number of points of the masks is odd */
   iarg    = npt;        /* hampli is the center of the masks*/

   for ( i=0 ; i<hampli ; i++ ) {
      iarg --;
      gau[i]   = gau[iarg];

/*****************************************************************************/
/*                                                                           */
/* the masks are symmetric so copy an half on                                */
/*                                    the other preserving the central point */
/*                                                                           */
/*****************************************************************************/
      D2gau[i] = D2gau[iarg];
   }

/*****************************************************************************/
/*                                                                           */
/* calculates the sum of values and of the square values                     */
/*                                                                           */
/*****************************************************************************/
         
   for ( i=0 ; i<npt ; i++ ) {
      sumG    += gau[i];
      sumD2G  += (float)fabs((double)D2gau[i]);
      sumqG   += gau[i] * gau[i];
      sumqD2G += D2gau[i] * D2gau[i];
   }

/*****************************************************************************/
/*                                                                           */
/*                output of the results                                      */
/*                                                                           */
/*****************************************************************************/
/*
   printf("Sum D2 -> %e , Sum G -> %e\n",sumD2G,sumG);         
   printf("Sum of square D2 -> %e ",sumqD2G);
   printf("Sum of square G  -> %e \n",sumqG);
*/
/*****************************************************************************/
/*                                                                           */
/* rounds the masks to the nearest integer, normalizing the values to 128    */
/*                                                                           */
/*****************************************************************************/

   for ( i=0 ; i<npt ; i++) {
      inc         =  .5;
      if(D2gau[i] < 0) inc = - .5;
      D2gauint[i] = (int)(D2gau[i] * 127. + inc);
      gauint[i]  = (int)(gau[i] * 255.);
   }

/*****************************************************************************/
/*                                                                           */
/* calculates the integer integral of the operators, it must be equal to     */ 
/*     zero for the Laplacian of Gaussian                                    */
/*                                                                           */
/* calculates also the sum of values and the sum of square values for the    */
/*                                integer masks                              */
/*                                                                           */
/*****************************************************************************/

   while( sumint != 0 ) {
      sumint  =  0;
      sumqG   =  0;         /* initialization */
      sumqD2G =  0;

      for (i=0 ; i<npt ; i++) {
         sumint += D2gauint[i];
         arg     = (float)(gauint[i]);
         sumqG  += arg * arg;
         arg     = (float)(D2gauint[i]);
         sumqD2G += arg * arg;
      }

      if (abs(sumint) > 1) {
         printf("Integral of D2G not equal to zero \n");
         return(-1);
      }
/*****************************************************************************/
/*                                                                           */
/*         correction of the integral                                        */
/*                                                                           */
/*****************************************************************************/

      if(sumint == 1)  D2gauint[npt/2] = D2gauint[npt/2] - 1;

      if(sumint == -1) D2gauint[npt/2] = D2gauint[npt/2] + 1;
         
   }         /* end while */
/*
   printf(" D2G Integral -> %d\n",sumint);
   printf("integer D2G mask : \n");
*/
/*****************************************************************************/
/*                                                                           */
/*        prints the values of the integer Laplacian of Gaussian mask        */
/*                                                                           */
/*****************************************************************************/
/*
   y=0;
   for (i=0 ; i<npt/20 ; i++) {
      for (x=0 ; x<20 ; x++) {
         printf(" %d",D2gauint[y]);
         y ++;
      }
      printf("\n");
   }
   if (npt - y != 0){
      for (i=y ; i<npt ; i++) printf(" %d",D2gauint[i]);
      printf("\n");
   }
*/

   normG   = (int)(sumqG/128);         /* normalizing factors for G and D2G */
   normD2G = (int)(sumqD2G/128);
/*
   printf("Norm_factor D2G -> %d ",normD2G); 
   printf(" Norm_factor G -> %d\n",normG);
*/

/*                                                                           */
/*         Determine the size of the temporary storage array.                */
/*                                                                           */
/*                                                                           */
/*       x extent with part of view external to image domain                 */
/*                                                                           */

   sizex = (short) views[view_number_1].x; /* x view extent */
   if (sizex > (image_descriptor_1->size - x_1)) 
      sizex = image_descriptor_1->size - x_1; 


/*                                                                           */
/*       y extent with part of view external to image domain                 */
/*                                                                           */

   sizey = (short) views[view_number_1].y; /* y view extent */
   if (sizey > (image_descriptor_1->size - y_1)) 
      sizey = image_descriptor_1->size - y_1;

   if (sizex > sizey) size=sizex; /* determine size of temporary array */
   else               size=sizey;
 
   /*                              */
   /*  determine limits on loops.  */ 
   /*                              */

   x_limit = sizex + x_1;
   y_limit = sizey + y_1;

/*****************************************************************************/
/*                                                                           */
/*          Now include conditional compilation for usage of                 */
/*                     Eidobrain-fast-convolution                            */
/*                                                                           */
/*****************************************************************************/

#ifdef EIDO_DEV
   check_eido_device();
   image_store1 = image_descriptor_1->image.array_link;
   image_store2 = image_descriptor_2->image.array_link;

/*****************************************************************************/
/*                                                                           */
/*            convolution of the image with the D2G operator                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*       There is the implicit assumption that the window is TOTALLY         */
/*                         inside the image                                  */
/*                                                                           */
/*****************************************************************************/

   eido_convolution(image_store1,(short)x_1,(short)y_1,
                    image_store2,(short)x_2,(short)y_2,
                    (short)(views[view_number_1].x),
                    (short)(views[view_number_1].y), sigma);
/*****************************************************************************/
/*                       The device MUST be closed!                          */
/*****************************************************************************/
   close_eido_device();
#else

/*                                                                           */
/*       Determine output view x extent considering that part of the view    */
/*       may be external to the image domain.                                */
/*                                                                           */

   opsizex = (short) views[view_number_2].x;
   if (opsizex > image_descriptor_2->size - x_2)
      opsizex = image_descriptor_2->size - x_2;

                /* generate temporary arrays */
   if((generate_array_image(size,&image_store1)) == -1) return(-1);
   if((generate_array_image(size,&image_store2)) == -1) return(-1);

/*****************************************************************************/
/*                                                                           */
/*            convolution of the image columns with the D2G operator         */
/*                                                                           */
/*****************************************************************************/
         /* sets the convolution mask */
   maskad = D2gauint;
   nptmask = npt;   
   nptmas2 = npt/2;
   //printf("convolution 1; mask 1 \n");
   for (y=y_1; y < y_limit; y++) {
     /*if (y%50 == 0)
     printf("processing element %d\r",y);*/
                 /* get column from input image */

      readcol(x_1,y,sizex,inpb,image_descriptor_1->image.array_link); 

      conv1(inpb,sizex,normD2G,outb,maskad,nptmask,nptmas2);       /* vertical convolution */

                 /* store intermediate column */

      writecol(0,y-y_1,sizex,outb,image_store1);
   }  
/*****************************************************************************/
/*                                                                           */
/*            convolution of the previous result rows with the G operator    */
/*                                                                           */
/*****************************************************************************/
        /* sets the convolution mask */
   maskad = gauint;
   nptmask = npt;   
   nptmas2 = npt/2;
   //printf("convolution 1; mask 2 \n");
   for (x = x_1; x < x_limit; x++) {
/*     if (x%50 == 0)
printf("processing element %d\r",x);*/

      readlin(x-x_1,0,sizey,inpb,image_store1);

      conv2(inpb,sizey,normG,outb,maskad,nptmask,nptmas2);   /* horizontal convolution */

      writelin(x-x_1,0,sizey,outb,image_store1);
   }  

/*****************************************************************************/
/*                                                                           */
/*            convolution of the image rows with the D2G operator            */
/*                                                                           */
/*****************************************************************************/

                  /* sets the convolution mask */

   maskad = D2gauint;
   nptmask = npt;   
   nptmas2 = npt/2;


   //printf("convolution 2; mask 1 \n");

   for (x = x_1; x < x_limit; x++) {          
/*      if (x%50 == 0)
printf("processing element %d\r",x);*/

      readlin(x,y_1,sizey,inpb,image_descriptor_1->image.array_link);

      conv1(inpb,sizey,normD2G,outb,maskad,nptmask,nptmas2);     /* horizontal convolution */

      writelin(x-x_1,0,sizey,outb,image_store2);
   }  

/*****************************************************************************/
/*                                                                           */
/*            convolution of the image columns with the G operator           */
/*                                                                           */
/*****************************************************************************/

         /* sets the convolution mask */
   maskad = gauint;
   nptmask = npt;   
   nptmas2 = npt/2;

   //printf("convolution 2; mask 2 \n");

   for (y = y_1; y < y_limit; y++) {
/*      if (y%50 == 0)
printf("processing element %d\r",y);*/

      readcol(0,y-y_1,sizex,inpb,image_store2);

      conv2(inpb,sizex,normG,outb,maskad,nptmask,nptmas2);     /* vertical convolution */

/*****************************************************************************/
/*                                                                           */
/* reads the column that corresponds to the vertical convolution of the      */
/*           image with the D2G operator                                     */
/*                                                                           */
/*****************************************************************************/

      readcol(0,y-y_1,sizex,inpb,image_store1);

      for (i = 0; i < sizex; i++) {       /* sum of the columns */

/*
         add = inpb[i] + outb[i] + 128;       original code
*/

         /* Lattice C char types are in the range 0-255 NOT -128 -> 127 */
         /* so we must effect the addition/subtraction explicitly       */

         add = 128;
         if (inpb[i] > 127)  add -= (char)(~inpb[i] + 1); /* subtract 2's complement */
         else                add +=  inpb[i];
         if (outb[i] > 127)  add -= (char)(~outb[i] + 1); /* subtract 2's complement */
         else                add +=  outb[i];
         
/*****************************************************************************/
/*                                                                           */
/* the filtered image must be in the range 0 : 255,                          */
/*               with 128 corresponding to zero                              */
/*                                                                           */
/*****************************************************************************/
         if( add > 255 ) add = 255;
         if( add < 0 ) add = 0;
         if( add == 128 )add = 129;
         if (inpb[i]==0 && outb[i]==0) add = 128;

         inpb[i] = (char)add;
      }
      writecol(x_2,y_2,opsizex,inpb,image_descriptor_2->image.array_link);

      y_2++;  /*  move to next column of the output view */

/*                                                                           */
/*    Finish if the view extends beyond the image domain                     */
/*                                                                           */
      if(y_2 >= image_descriptor_2->size) break;
   }
   delete_array_image(size,image_store1);
   delete_array_image(size,image_store2);

   image_store2 = image_descriptor_2->image.array_link;

#endif

/*                                                                          */
/*     In the convolution could be zero value pixels (128), that are        */
/*     singular values in the convolution. Substitute them with the first   */
/*     region value different from 128.                                     */
/*                                                                          */
   for (x = x_1+nptmas2+1; x < x_limit-nptmas2-1; x++)
        for (y = y_1+nptmas2+1; y < y_limit-nptmas2-2; y++) {
             valin = (int)(image_store2[x][y] & 0377);
             valout = (int)(image_store2[x][y+1] & 0377);
             if(valin == 128 && valout != 128) {
                      val = (valout > 128) ? 129 : 127;
                      fillreg(x,y,128,val,x_1+nptmas2+1,x_limit-nptmas2-2,
                              y_limit-nptmas2-2,y_2+nptmas2+1,image_store2);
              } else	
              if(valin != 128 && valout == 128) {
                      val = (valin > 128) ? 129 : 127;
                      fillreg(x,y,128,val,x_1+nptmas2+1,x_limit-nptmas2-2,
                              y_limit-nptmas2-2,y_2+nptmas2+1,image_store2);
              }
          }
   return(0);
}
/****************************************************************************/
/*                                                                          */
/* --- Program or Subprogram name: line                 		    */
/*                                                                          */
/* --- Functional Description: Draw an horizontal line on an array image    */
/*     at the given y position from left_x to right_x.                      */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*     line(left_x,right_x,y_start,val,image)                               */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*     left_x, right_x ......  extreme x coordinates of the line            */
/*     y_start ..............  y coordinate of line                         */
/*     val ..................  gray value of line                           */
/*     image.................  array image to draw line                     */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/* --- Local Variables:                                                     */
/*     i,j .........  counter integers                                      */
/*                                                                          */
/* --- Bugs:                                                                */
/*     Tell us if you find any!                                             */
/*                                                                          */
/* --- Author: G. Sandini and M. Tistarelli                                 */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*                                                                          */
/****************************************************************************/
void line(int left_x, int right_x, int y_start, int val, array_image_type image)
{
register int i,j;
j=y_start;
for(i=left_x;i<=right_x;i++)
	  image[i][j] = val;
}


/****************************************************************************/
/*                                                                          */
/* --- Program or Subprogram name:  line_fire                               */
/*                                                                          */
/* --- Functional Description:    This sub-routine traces a star centered   */
/*     in (x,y).  Eight beams are considered and if the informa-            */
/*     tion is expressive for more than five beams a interpolation          */
/*     is calculated. The information for a beam is the value of            */
/*     the disparity in the point of the intersection between a             */
/*     beam and a contour (if exist!!)                                      */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*     line_fire(x0,y0, x1,y1, x,y, depth, target);                         */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*     x, y...................coordinates of the star center	            */
/*      x0,y0, x1,y1...........window coordinates within view               */
/*      depth..................depth array, the depth values at contour     */
/*                             points are actually on it... to be extended  */
/*                             on the rest of the image itself.             */
/*      target.................array image, it is an array used as target   */
/*                             to find the position of contour points.      */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*     image, outimage..................virtual images for I/O operations   */
/*                                                                          */
/* --- Local Variables:                                                     */
/*     x_incr, y_incr.........indicates the current traced ray              */
/*     curr_x, curr_y.........current coordinates                           */
/*     val....................current contour number                        */
/*     cont...................counter to signal absence of information	    */
/*     nominator, denominator.interpolation's variables                     */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/* --- Author: M. Tistarelli  UG - DIST                                     */
/*                                                                          */
/* --- Revisions                                                            */
/*         date: 10/08/87                                                   */
/*         revisor: M.Tistarelli                                            */
/*         reason: Rationalization of code                                  */
/*                                                                          */
/****************************************************************************/

void line_fire(int x0, int y0, int x_limit, int y_limit, int x, int y, int_array_image_type depth_image, array_image_type image)
{
int cont;
double nominator, denominator, depth_val;
double distance, vx, vy;
int x_incr, y_incr;
int oldx, oldy;
int currx,curry,val;
int val2,val3;

cont = 0; nominator = denominator = 0;
for(x_incr = -1 ; x_incr <= 1 ; x_incr++)
   for(y_incr = -1 ; y_incr <= 1 ; y_incr++) {
      if(!(y_incr == 0 && x_incr == 0)) {
            oldx = currx = x; oldy = curry = y; val = 0;
            if (cont > 1) return;
            do {
               currx += x_incr;
               curry += y_incr;
               val = (int)((image[currx][curry])&0377);
               if(val == 0) {
                  val2 = (int)((image[oldx][curry])&0377);
                  val3 = (int)((image[currx][oldy])&0377);
                  if(val2 == 0) {
                     val = val3;
                     if(val3 != 0) curry = oldy;
                  } else {
                     val = val2;
                     currx = oldx;
                  }
               }
               oldx = currx;
               oldy = curry;
             } while((currx < x_limit-1) && (curry < y_limit-1) && 
                  (currx > x0) && (curry > y0) && (val == 0));
             if (val != 0) {
                vx = (double)(currx - x);
                vy = (double)(curry - y);
                distance = sqrt(vx*vx + vy*vy);
                depth_val = (double)(depth_image[currx][curry]);
                if(distance > 0.00009) {
                       denominator += (1./distance);
                       nominator += depth_val/distance;
                }
            } else cont++;
        }
    }
    if(denominator > 0.00009)
              depth_image[x][y] = (int)( nominator/denominator + .5);
}

/* Special functions for stack manipulation */

 
/****************************************************************************/
/*                                                                          */
/* --- Program or Subprogram name: pop                  		    */
/*                                                                          */
/* --- Functional Description: get a pixel coordinate from a stack          */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:  		    */
/*                                                                          */
/* --- Calling Procedure:                    				    */
/*     pop(&x,&y)                                                           */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*     x, y .... pointers to the image point coordinates to be "popped out" */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*     x, y .... pointers to the image point coordinates "popped out"       */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*     p_stk ......... index in the two-dimensional stack                   */
/*     p_stack ..............  stack of image points used by the filling    */
/*                             algorithm                                    */
/*     stk_n_empty ......  flag that indicates if the stack has been        */
/*                         filled up                                        */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/* --- Bugs:                                                                */
/*              None (I hope)                                               */
/*                                                                          */
/* --- Author: G. Sandini and M. Tistarelli                                 */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*                                                                          */
/****************************************************************************/

void pop(int *curr_x, int *curr_y)
{
        *curr_x = p_stack[p_stk].X;
        *curr_y = p_stack[p_stk].Y;
        if(++p_stk >= NLOC_STACK) stk_n_empty = 0;
         else stk_n_empty = 1;
}

/****************************************************************************/
/*                                                                          */
/* --- Program or Subprogram name: push                 		    */
/*                                                                          */
/* --- Functional Description: put a pixel coordinates into a stack         */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:  		    */
/*                                                                          */
/* --- Calling Procedure:                    				    */
/*     push(x,y)                                                            */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*     x, y .... pointers to the image point coordinates to be "pushed in"  */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*     x, y .... pointers to the image point coordinates "pushed in"        */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*     p_stk ......... index in the two-dimensional stack                   */
/*     p_stack ..............  stack of image points used by the filling    */
/*                             algorithm                                    */
/*     stk_n_empty ......  flag that indicates if the stack has been        */
/*                         filled up                                        */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/* --- Bugs:                                                                */
/*              None (I hope)                                               */
/*                                                                          */
/* --- Author: G. Sandini and M. Tistarelli                                 */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:                                                            */
/*         revisor:                                                         */
/*         reason:                                                          */
/*                                                                          */
/****************************************************************************/
void push(int curr_x, int curr_y)
{
        if(--p_stk >= 0) {
		stk_n_empty = 1;
                p_stack[p_stk].X = curr_x;
                p_stack[p_stk].Y = curr_y;
        }
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: readcol                                             */
/*                                                                           */
/* ---  Functional Description: readcol reads a specified length of an image */
/*      column starting at a given position into a character array.          */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none.                */
/*                                                                           */
/* ---  Calling Procedure: readcol(irow,icol,length,buffer,image);           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      irow,icol .. start row and column of read in the image. (integers)   */
/*      length .. buffer size (short integer)                                */
/*      buffer .. character array containing output data.                    */
/*      image  .. pointer to an image of array_image_type.                   */
/*                                                                           */
/* ---  Output Parameters: none                                              */
/*                                                                           */
/* ---  Global Parameters: none                                              */
/*                                                                           */
/* ---  Local Variables: i .. local variable.                                */
/*                        .. character pointer which removes one level   */
/*                       of indirection and enables standard array access.   */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N                                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void readcol(int irow,int icol, short length,char buffer[], array_image_type image)
{
   short i;

   i=0;
   while ( i < length) {
      buffer[i++] = image[irow++][icol];
   }
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: readlin                                             */
/*                                                                           */
/* ---  Functional Description: readlin reads a specified length of an image */
/*      row starting at a given position into a character array.             */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none.                */
/*                                                                           */
/* ---  Calling Procedure: readlin(irow,icol,length,buffer,image);           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      irow,icol .. start row and column of read in the image. (integers)   */
/*      length .. buffer size (short integer)                                */
/*      buffer .. character array containing output data.                    */
/*      image  .. pointer to an image of array_image_type.                   */
/*                                                                           */
/* ---  Output Parameters: none                                              */
/*                                                                           */
/* ---  Global Parameters: none                                              */
/*                                                                           */
/* ---  Local Variables: i .. local variable.                                */
/*                       image .. character pointer                          */
/*                                         enabling standard array access.   */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N                                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void readlin(int irow, int icol, short length, char buffer[], array_image_type image)
{
   int i;

   i=0;
   while ( i < length) {
      buffer[i++] = image[irow][icol++];
   }
}

/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   select_significant_contours                       */
/*                                                                           */
/* ---  Functional Description:                                              */
/*                                                                           */
/*      Select significant contours in a contour image.                      */
/*                                                                           */
/*      Significant contours are those that exhibit an average slope         */
/*      greater than the global mean slope minus 0.5 a standard deviation && */
/*      which have a standard deviation of slope less than 1.5 the global    */
/*      standard deviation.                                                  */
/*                                                                           */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      select_significant_contours(contour_id)                              */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      contour_id ..... pointer to image descriptor of contour-type image   */
/*                                                                           */
/* ---  Output Parameters: none                                              */
/*                                                                           */
/* ---  Global Parameters: none.                                             */
/*                                                                           */
/* ---  Local Variables:                                                     */
/*                                                                           */
/*                            MANY!                                          */
/*                                                                           */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.  (1-4-87)                                 */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:          4/10/88                                               */
/*      revisor:       David Vernon                                          */
/*      reason:        commented out "<CR> to continue" code                 */
/*                                                                           */
/*****************************************************************************/

int select_significant_contours(struct image_descriptor_type *contour_id)
{
   struct contour_descriptor_type *c;
   struct contour_node_type *contour;
   short int low[8], high[8];
   int i,k;  
   float cum_l;
   double mean, s_d, t1, t2;
 

	if (contour_id->image_type == CONTOUR_IMAGE_TYPE)
      contour = contour_id->image.contour_link;
	else if (contour_id->image_type == SLOPE_IMAGE_TYPE)
      contour = contour_id->image.slope_link;
	else if (contour_id->image_type == ORIENTATION_IMAGE_TYPE)
      contour = contour_id->image.orientation_link;
   else {
      system_error("Select_significant_contours: not a contour-type image");
      return(1);
   }

   /* contour properties 

       1:  Length
       2:  Mean Slope
       3:  Standard Deviation of Slope
       4:  Mean Orientation
       5:  Standard Deviation of Orientation
       6:  Mean Curvature
       7:  Standard Deviation of Curvature

   */

   for (i=0; i<8; i++) {
       low[i] = -9999;
       high[i] = 9999;
   }

   /*** determine selection criteria ***/

   i = 0;
   mean = 0.0;
   s_d = 0.0;

   cum_l = 0.0;

   for (c = contour->descriptor_link; c != NULL; c = c->next) {
      for (k=0; k < c->length; k++)
         mean += (double) c->slope->value[k];
      i++;  /* compute total number of contours */
      cum_l += (float) c->length;
   }            

   mean = mean / (double) cum_l;

   t2 = 0.0;
   for (c = contour->descriptor_link; c != NULL; c = c->next) {
      for (k=0; k < c->length; k++) {
         t1 = (double) c->slope->value[k] - mean;
         t2 += t1 * t1;
      }
   }
   t2 = t2 /  (double) cum_l;
   s_d = sqrt(t2);

   t1 = mean - 0.5 * s_d + 0.5;	  /* round */
   low[2] = (int) t1; 

   t1 = 1.5 * s_d + 0.5;           /* round */  
   high[3] = (int) t1; 

//printf("\n\nmean slope = %f      s.d. slope = %f",mean,s_d);
//printf("\nmean slope low limit = %d  s.d. slope high limit = %d\n",low[2],high[3]);
/*
            cursor_position(21,24); printf("Enter <CR> to continue >>");
            get_response(reply,10);
*/
   /*** select the contours ***/  
     
   for (c = contour->descriptor_link; c != NULL; c = c->next) 

      if ((c->length > low[1] && c->length < high[1])  &&
          (c->mean_slope > low[2] && c->mean_slope < high[2])  &&
          (c->s_d_slope >  low[3] && c->s_d_slope < high[3]) &&
          (c->mean_orientation > low[4] && 
                                      c->mean_orientation < high[4]) &&
          (c->s_d_orientation > low[5] && 
                                           c->s_d_orientation < high[5]) &&
          (c->mean_curvature  > low[6] && 
                                           c->mean_curvature < high[6]) &&
          (c->s_d_curvature   > low[7] && 
                                           c->s_d_curvature  < high[7]) ) {

           c->select = TRUE; 
      }
      else
           c->select = FALSE; 

   return(0);

}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   set_view                                          */
/*                                                                           */
/* ---  Functional Description:                                              */
/*                                                                           */
/*      This routine sets the bounds on a view (window) which defines a      */
/*      specific area of interest to which image functions are confined,     */
/*      in particular, the set_view finction is used to delineate a          */
/*      rectangular region which may be accessed/transferred using the       */
/*      transfer_view function.                                              */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none.                */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*      set_view(view_number, size_x, size_y);                               */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      view_number .... an integer number identifying the view to be        */
/*                       defined.                                            */
/*                                                                           */
/*      size_x, size_y . integer pair defining the size of the rectangular   */
/*                       view.                                               */
/*                                                                           */
/* ---  Output Parameters: none.                                             */
/*                                                                           */
/* ---  Global Parameters:                                                   */
/*                                                                           */
/*      views ... an array of view_type structures comprising x and y extent */
/*                of view.                                                   */
/*                                                                           */
/* ---  Local Variables: none.                                               */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, TCD.                                           */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void set_view(int view_number,int size_x, int size_y)      
 
{
   extern struct view_type views[];                    

   views[view_number].x = size_x;
   views[view_number].y = size_y;
}

/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: slope_interp                                        */
/*                                                                          */
/* --- Functional Description:   This sub-program interpolate a slope       */
/*     contour image considering the values on the edges. The result is a   */
/*     slope array image with the clope values computed for each pixel.     */ 
/*                                                                          */
/*     This function is a direct adaptation of Massimo Tistarelli's         */
/*     depth_interp() function                                              */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*     generate_array_image                                                 */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*     slope_interp(view_1,xv,yv,id_1, view_2,xc,yc,id_2)                   */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*   - view_1             view in slope contour type (the first) image      */
/*   - xv, yv             first view origin                                 */
/*   - id_1               pointer to image descriptor of slope contour      */
/*   - view_2             view in range array type (the second) image       */
/*   - xc, yc             second view origin                                */
/*   - id_2               pointer to image descriptor of second image       */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/* Function returns value 0 if contour matching is successfully completed,  */
/* -1 otherwise.                                                            */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*      struct view_type views[]                                            */
/*                                                                          */
/* --- Local Variables:                                                     */
/*      Really many .... see code listing for details.                      */
/*                                                                          */
/* --- Bugs:                                                                */
/*          Who can say? (you must be BRAVE!)                               */
/*                                                                          */
/* --- Author: D. Vernon (based on code written by M. Tistarelli  UG - DIST */
/*                                                                          */
/*                                                                          */
/****************************************************************************/

int slope_interp(int view1, int x1, int y1, struct image_descriptor_type *id_1, int view2, int x2, int y2, struct image_descriptor_type *id_2)
{
   extern struct view_type views[];

   int nx,ny;
   struct contour_node_type *contour, *slope;
   struct contour_descriptor_type *descriptor;
   array_image_type target_image;
   int_array_image_type slope_image;
   int cont, label;
   int x,y, x_limit,y_limit;

   /*  Test parameters passed. */

   if ((id_1 == NULL) || (id_2 == NULL) ||
      ((id_1->image_type != SLOPE_IMAGE_TYPE) &&
       (id_2->image_type != RANGE_IMAGE_TYPE))  ||
       (view1 < 0 || view1 > MAX_NUMBER_VIEWS ) ||
       (view2 < 0 || view2 > MAX_NUMBER_VIEWS )) return(-1);

/*                                                                            */
/*   let image be used instead of image_descriptor->......                    */
/*                                                                            */

   if((slope_image = id_2->image.range_link) == NULL) return(-1);
   if((slope = id_1->image.slope_link) == NULL) return (-1);

   descriptor = slope->descriptor_link;

   if(descriptor == NULL) {
	   system_error("Impossible to find a contour descriptor link");
       return(-1);
   }

   contour = descriptor->contour;

/*                                                                            */
/*    Generate the target image for the matching of the contour points        */
/*                                                                            */

   x_limit = x2 + views[view2].x;
   y_limit = y2 + views[view2].y;

   if((generate_array_image(id_2->size,&target_image)) == -1) return(-1);
   for(nx=x2; nx<x_limit; nx++)
        for(ny=y2; ny<y_limit; ny++) {
                      target_image[nx][ny] = 0;
                      slope_image[nx][ny] = 0;
        }
   label = 100;

   while (contour != NULL) {
     descriptor = contour->descriptor_link;
     if ((descriptor != NULL) && (descriptor->select == TRUE)) {

         printf("Contour Number %d", descriptor->label);

         for (cont = 0, nx = contour->start_x, ny = contour->start_y; 
              cont < contour->length; cont++) {
/*                                                                            */
/*                      compute the coordinates                               */
/*                                                                            */
               chain_calc(contour->value[cont],&nx,&ny);

               if (nx >= x1 && nx < x1 + views[view1].x  &&
                   ny >= y1 && ny < y1 + views[view1].y )
 
                  /* consider only selected parts of contour */

                  if (((contour->value[cont]) & '\200')  == 0) {
                       x = nx - x1 + x2;
                       y = ny - y1 + y2;
                       target_image[x][y] = label;
                       slope_image[x][y] = slope->value[cont];
                  }
         }
       }
       contour = contour->next;
       slope = slope->next;
    }

/*  Locate the "zero border" around the image... unuseful for interpolation */

for (x = x2; x < x_limit ; x++) {
   for (y = y2; y < y_limit ; y++) if((slope_image[x][y]) != 0) break;
   if((slope_image[x][y]) != 0) break;
}
x2 = x;

for (x = x_limit-1; x > x2 ; x--) {
   for (y = y2; y < y_limit ; y++) if((slope_image[x][y]) != 0) break;
   if((slope_image[x][y]) != 0) break;
}
x_limit = x+1;

for (y = y2; y < y_limit ; y++) {
   for (x = x2; x < x_limit ; x++) if((slope_image[x][y]) != 0) break;
   if((slope_image[x][y]) != 0) break;
}
y2 = y;

for (y = y_limit-1; y > y2 ; y--) {
   for (x = x2; x < x_limit ; x++) if((slope_image[x][y]) != 0) break;
   if((slope_image[x][y]) != 0) break;
}
y_limit = y+1;

/*  In the slope_image there are the values of slope as from slope contour. */
/*  Target is an array image with only the position on it. It is used to    */
/*  locate the contour points; the values are actually read from the other. */


    for(x = x2+2; x < x_limit-2; x++)
       for(y = y2+2; y < y_limit-2; y++)
           if (slope_image[x][y] == 0) 
               line_fire(x2+2,y2+2, x_limit-2,y_limit-2, x,y,slope_image, target_image);

/*
    for(x = x2+1; x < x_limit-1; x++) {
       for(y = y2+1; y < y_limit-1; y++)
          if (slope_image[x][y] == 0) {
               line_fire(x2,y2, x_limit,y_limit, x,y,slope_image, target_image);
          }
    }
    */
    return(0);
}
/*****************************************************************************/
/*                                                                           */
/* ---  subprogram name: system_error                                        */
/*                                                                           */
/* ---  Functional Description :                                             */
/*                                                                           */
/*      Report system error and offer option to abort session.               */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*                blank_menu()                                               */
/*                blank_line()                                               */
/*                cursor_position()                                          */
/*                                                                           */
/*                                                                           */
/* ---  Calling Procedure : system_error(error_description);                 */
/*                                                                           */
/* ---  Input Parameters : error_description ... string containing a         */
/*                                               description of the error.   */
/*                                                                           */
/* ---  Output Parameters : none                                             */
/*                                                                           */
/* ---  Global Parameters : none                                             */
/*                                                                           */
/* ---  Local Variables :   none                                             */
/*                                                                           */
/* ---  Bugs :                                                               */
/*                                                                           */
/* ---  Author: David Vernon                                                 */
/*                                                                           */
/* ---  Revisions                                                            */
/*            date: 16-10-86                                                 */
/*            revisor: Massimo Tistarelli                                    */
/*            revision: Substituted exit system call with a "vis_exit" call  */
/*                      to reset terminal and eventually other devices       */
/*                                                                           */
/*            date: 16-02-12                                                 */
/*            revisor: David Vernon                                          */
/*            revision: Substituted body of function for printf()            */
/*                                                                           */
/*****************************************************************************/

void system_error(char *error_description)
{
/*
   blank_menu();
   cursor_position(10,34);
   putstr("SYSTEM ERROR:");
   cursor_position(12,((80-strlen(error_description))/2));
   putstr(error_description);
   cursor_position(15,1);
   blank_line();
   cursor_position(15,21);
   putstr("Do you wish to abort the session (y/n)?");
   getstring(reply,40);
   if (reply[0] == 'y' || reply[0] =='Y')
      vis_exit();
*/
   printf("%s \n", error_description);
}
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: writecol                                            */
/*                                                                           */
/* ---  Functional Description: writecol writes the contents of a buffer of  */
/*      a specified length into an image column starting at a given position.*/
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none.                */
/*                                                                           */
/* ---  Calling Procedure: writecol(irow,icol,length,buffer,image);          */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      irow,icol .. start row and column. (integers)                        */
/*      length .. buffer size (short integer)                                */
/*      buffer .. character array containing input data.                     */
/*      image  .. pointer to an image of array_image_type.                   */
/*                                                                           */
/* ---  Output Parameters: none                                              */
/*                                                                           */
/* ---  Global Parameters: none                                              */
/*                                                                           */
/* ---  Local Variables: i .. integer                                        */
/*                       image .. character pointer                          */
/*                                         enabling standard array access.   */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N                                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void writecol(int irow, int icol, short length, char buffer[], array_image_type image)
{
   int i;
   
   i=0;
   while ( i < length) {
      image[irow++][icol] = buffer[i++];
   }
}
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: writelin                                            */
/*                                                                           */
/* ---  Functional Description: writelin writes the contents of a buffer of  */
/*      a specified length into an image row starting at a given position.   */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none.                */
/*                                                                           */
/* ---  Calling Procedure: writelin(irow,icol,length,buffer,image);          */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      irow,icol .. start row and column. (integers)                        */
/*      length .. buffer size (short integer)                                */
/*      buffer .. character array containing input data.                     */
/*      image  .. pointer to an image of array_image_type.                   */
/*                                                                           */
/* ---  Output Parameters: none                                              */
/*                                                                           */
/* ---  Global Parameters: none                                              */
/*                                                                           */
/* ---  Local Variables: i .. integer                                        */
/*                       image.. character pointer enabling standard array   */
/*                               access.                                     */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: S.O'N                                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      dd/mm/yy                                                  */
/*      revision:                                                            */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

void writelin(int irow, int icol, short length, char buffer[], array_image_type image)
{
   int i;

   i=0;
   while ( i < length) {
     image[irow][icol++]=buffer[i++];   
   }
}


/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: zc_features                                         */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*	This subroutine generates a slope and an orientation image              */
/*	based on a contour image and two array images representing the          */
/*	slope and orientation information.                                      */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*	- delete_contour_image()                                                */
/*	- generate_contour_node()                                               */
/*	- insert_contour()                                                      */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     zc_features(contour_image_descriptor, slope_image_descriptor,        */
/*                 orientation_image_descriptor, slope, orientation)        */
/*                                                                          */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*	- contour_image_descriptor   pointer to image descriptor of contour     */
/*                               image 	                                    */
/*	- slope                      array image containing required            */
/*                               slope information.	                        */
/*	- orientation                array image containing required            */
/*	                             orientation information.                   */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*       - slope_image_descriptor,                                          */
/*      - orientation_image_descriptor                                      */
/*                                   pointers to image descriptor of slope  */
/*                                   and orientation images to be generated */
/*                                                                          */
/*	Function returns value 0 if following is successfully completed,        */
/*	-1 otherwise                                                            */
/*                                                                          */
/* --- Global Parameters: none.                                             */
/*                                                                          */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*	- code			chain code of the current contour pixel                 */
/*	- cont,nx,ny,c_pix	scratch variables                                   */
/*	- contour               pointer to contour node in contour image        */
/*	- slope_contour         pointer to contour node in slope image          */
/*	- orientation_contour   pointer to contour node in orientation image    */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*	The present code for building the feature images is inefficent in       */
/*      that it uses the insert_contour routine rather than an              */
/*      append_contour routine (which has yet to be written).   D.V.        */
/*                                                                          */
/* --- Author: G. Sandini & M. Tistarelli  UG - DIST                        */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:     18-4-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Adapt routine to generate a feature images rather      */
/*		     than the original file of features.                            */
/*                                                                          */
/****************************************************************************/

int zc_features(struct image_descriptor_type *contour_image_descriptor, struct image_descriptor_type *slope_image_descriptor,
                struct image_descriptor_type *orientation_image_descriptor, array_image_type slope, array_image_type orientation)
{
   int cont;
   int nx,ny;
   struct contour_node_type *contour, *slope_contour, *orientation_contour;


   /*  Test parameters passed. */

   if (contour_image_descriptor == NULL || 
       slope_image_descriptor == NULL || 
       orientation_image_descriptor == NULL)
      return (-1);

   if (contour_image_descriptor->image_type != CONTOUR_IMAGE_TYPE || 
       slope_image_descriptor->image_type != SLOPE_IMAGE_TYPE || 
       orientation_image_descriptor->image_type != ORIENTATION_IMAGE_TYPE)
      return (-1);


   /* delete the existing slope and orientation images (if they exist) */

   delete_contour_image(&(slope_image_descriptor->image.slope_link));
   delete_contour_image(&(orientation_image_descriptor->image.orientation_link));

   contour = contour_image_descriptor->image.contour_link;

   while (contour != NULL) {

      if ((slope_contour = generate_contour_node(contour->start_x,contour->start_y,contour->length))!=NULL) {
         if ((orientation_contour = generate_contour_node(contour->start_x,contour->start_y,contour->length))!=NULL) {

            /* generate feature list (slope/orientation) */

            for (cont = 0, nx = contour->start_x, ny = contour->start_y; cont < contour->length; cont++) {

		          chain_calc(contour->value[cont],&nx,&ny); /* compute the coordinates */
                slope_contour->value[cont] = (int) slope[nx][ny];
                orientation_contour->value[cont] = (int) orientation[nx][ny];
            }

            insert_contour(&(slope_image_descriptor->image.slope_link),slope_contour);
            insert_contour(&((orientation_image_descriptor->image).orientation_link),orientation_contour);

         } 
      }
      contour = contour->next;
   }
   return(0);
}

                
/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: zc_fol                                              */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*	This subroutine executes a "contour-following" of all the               */
/*	contours in a zero-crossing image, building a contour image.            */
/*	The routine scans all the pixels of the chosen window bottom-up         */
/*	and left-right; when it reaches a pixel different from zero, (i.e.      */
/*	a contour pixel) then the contour-following begins.                     */
/*	First a forward following is executed checking the neighbourhood of     */
/*	the current "contour-pixel" in a 3*3 window counterclockwise:           */
/*                                                                          */
/*                                       <-------                           */
/*                                -------------  |                          */
/*                                | 3 | 2 | 1 |  |                          */
/*                                -------------  |                          */
/*                                | 4 | P | 0 |  * starting point           */
/*                                -------------	                            */
/*                                | 5 | 6 | 7 |	                            */
/*                                -------------	                            */
/*                                                                          */
/*	the central pixel is the last contour pixel found; the numbers          */
/*	identify the chain code of the possible contour pixels connected        */
/*	with the pixel "P".                                                     */
/*                                                                          */
/*	The search starts from the position "0" and first considering the       */
/*	positions with the least distance from the center of the window         */
/*	(i.e. positions 0 , 2 , 4 , 6 ) then the other positions,               */
/*	beginning from that marked as 1.                                        */
/*	The contour pixel found is deleted from the image, its position         */
/*	code and the original grey level are stored in two temporary files.     */
/*	The search goes on until no more connected contour pixels are found     */
/*	(i.e. until we find a contour pixel with all neighbours in a 3*3        */
/*	window, centered on it, equal to 0 ).                                   */
/*	If the contour followed is closed, then the following algorithm         */
/*	terminates.                                                             */
/*	If the contour is not closed, the final pixel recorded is taken as      */
/*	the first pixel of the contour and the existing chain code is           */
/*	reversed.                                                               */
/*	Finally a second following is done starting from the "original"         */
/*	initial pixel of the contour (i.e. the final pixel in the newly         */
/*	reversed partial contour) completing the contour.                       */
/*	Once the contour has been extracted, the number of chain codes is       */
/*	checked; if it is under a given threshold then the contour is           */
/*	deemed insignificant and is not included in the contour image.          */
/* 	Otherwise, the contour is converted from a linked list                  */
/*	representation to a string representation (since we now know the        */
/*	length of the chain).  A contour descriptor is generated and the        */
/*	contour is inserted into the contour image.                             */
/*	                                                                        */
/*	Note that the first chain code in the list is the code 8 and means      */
/*      "no move"; this is used to synchonise a chain code list with a      */
/*      corresponding contour property list which will be generated later   */
/*      (the number of chain codes is always one less than the number of    */
/*      pixels on the contour it represents.                                */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*	- append_to_cc()        function to append a chain code to list         */
/*      - follow()              function to follow contour, building chain  */
/*                              code.                                       */
/*	- delete_contour_image()                                                */
/*	- generate_contour_node()                                               */
/*	- insert_contour()                                                      */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     zc_fol(view_number,xv,yv, image_descriptor_1, image_descriptor_2)    */
/*                                                                          */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*	- view_number          view in zero-crossing image for following        */
/*	- xv ,yv               view origin                                      */
/*	- image_descriptor_1   pointer to image descriptor of zero-crossing     */
/*			       image	                                                */
/*	- image_descriptor_2   pointer to image descriptor of contour image     */
/*	                       to be generated.                                 */
/*                                                                          */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*	Function returns value 0 if following is successfully completed,        */
/*	-1 otherwise.                                                           */
/*                                                                          */
/* --- Global Parameters:                                                   */
/*                                                                          */
/*      struct view_type views[]                                            */
/*                                                                          */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*	- x0,y0,xf,yf   coordinates of the window's corners	                    */
/*	- c_cont        number of contours followed	                            */
/*	- c_pix			pixel's number of the current contour                   */
/*	- pixel			grey level of the current pixel	                        */
/*	- thres			threshold on the number of pixels                       */
/*	- start_x		x coordinate of the first pixel of the contour          */
/*	- start_y		y coordinate "   "    "     "   "   "                   */
/*	- newx			x coordinate of the last contour pixel                  */
/*	- newy			y     "      "   "   "      "      "                    */
/*	- cont,i,x,y		scratch variables                                   */
/*	- code,value		    "       "	                                    */
/*	- cc                    pointer to linked-list chain code.              */
/*	- last_node             pointer to last node in chain code.             */
/*	- temp_cc               pointer to linked-list chain code.              */
/*	- temp_last_node        pointer to last node in chain code.             */
/*	- z_c                   temporary pointer to array image; used to       */
/*	                        avoid excessive indirection.                    */
/*	- contour               pointer to contour node in contour image        */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*	None (by now)                                                           */
/*                                                                          */
/* --- Author: G. Sandini & M. Tistarelli  UG - DIST                        */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:     16-4-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Adapt routine to generate a contour image rather       */
/*		     than the original file of contours.                            */
/*                                                                          */
/*         date:     11-8-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Delete prompt for threshold length: use default of 4   */
/*		     (user should utilise selection mechanism to discard            */
/*		     short contours).                                               */
/*                                                                          */
/****************************************************************************/

int zc_fol(int view_number, int xv, int yv, struct image_descriptor_type *image_descriptor_1, struct image_descriptor_type *image_descriptor_2)
{
   extern struct view_type views[];
   static int x0,y0,xf,yf,x,y,ch_code;
   int value;
   int c_cont,c_pix,newx,newy,pixel;
   int thres,start_x,start_y;
   static struct cc_node *cc,*temp_cc, *last_node,*temp_last_node;
   static struct contour_node_type *contour;
   array_image_type z_c;


   /*  Test parameters passed. */

   if (image_descriptor_1 == NULL || image_descriptor_2 == NULL) return (-1);

   if (view_number < 0 || view_number > MAX_NUMBER_VIEWS) return (-1);

   /* let z_c be used instead of image_descriptor_1->image.array_link  */
   /* to avoid indirection and increase efficiency.                    */

   z_c = image_descriptor_1->image.array_link;


   /* delete the existing contour image (if it exists) */

   delete_contour_image(&(image_descriptor_2->image.contour_link));


   /* assign window coordinates, defining frame for computation */

   x0 = xv + 1;
   y0 = yv + 1;
   xf = xv + views[view_number].x - 2;
   yf = yv + views[view_number].y - 2;

   /* threshold on the minimum number of pixel of the contour */

   thres = 4;

	
	c_cont=0;	/* counter of the contours */
	
	/* begin principal scanning */

	for(x = x0; x <= xf; x++)
	{
	    for(y = y0; y <= yf; y++)
	    {
		if((pixel = z_c[x][y]) != 0)	/* new contour found */
		{
			c_pix = 0;	/* initialization */
                        cc = NULL;        /* null chain code initially */
                        last_node = NULL;

                	/* write a code equal to 8 for the first pixel */
                        /* that means "no move" *; this is used to     */
                        /* synchonise a chain code list with a corres- */
                        /* ponding contour property list which will be */
                        /* generated later (the number of chain codes  */
                        /* is always one less than the number of       */
                        /* pixels on the contour it represents.        */

			ch_code = 8;
			append_to_cc(&cc,&last_node,ch_code);
                        c_pix++;  /* update the total length of chain code */

			z_c[x][y] = ZERO;  /* delete zero-crossing pixel */

			newx = x; newy = y;

                	follow(&cc,&last_node,&c_pix,&newx,&newy,z_c); /* forward following */

			start_x = x; start_y = y;	/* first pixel of the current */
							/* contour if it is closed    */

			if((abs(newx-x)>1) || (abs(newy-y)>1))
			{ 

  		        	/* the contour is not closed so we reverse the chain code */
                                /* and begin again at the original point                  */

				/* the last pixel of the previous following */
				/* is the first of the contour              */	
				
		 		start_x = newx; start_y = newy;

				/**************/
                                /* reverse cc */
				/**************/

				temp_cc = NULL;
				temp_last_node = NULL;
				ch_code = 8;
				append_to_cc(&temp_cc,&temp_last_node,ch_code);

				/* run back along cc reversing the direction codes */
				
				while (last_node->rlink != NULL) { /* skip last node (= 8) */
				   append_to_cc(&temp_cc,&temp_last_node,
                                                (last_node->direction + 4) % 8);                                
                                   last_node = last_node->rlink;
                                }
				delete_cc(&cc);
                                
    				cc = temp_cc;
                                last_node = temp_last_node;

				newx = x; newy = y;

                                /* last following beginning from the point */
                                /* where the previous search started       */

                   		follow(&cc,&last_node,&c_pix,&newx,&newy,z_c); 
   		        } 
	

			if (c_pix > thres) /* check the number of pixels */
			{

				/* valid contour ... include it is the  list */

         			if ((contour = generate_contour_node(start_x,start_y,c_pix))!=NULL) {

                                   /* generate string representation of linked- */
                                   /* list chain code.                          */

				   temp_last_node = cc;
				   for (temp_last_node = cc, value=0;
				        temp_last_node != NULL; 
				        temp_last_node = temp_last_node->flink, value++) 
                                       contour->value[value] = temp_last_node->direction;
                         
				   insert_contour(&(image_descriptor_2->image.contour_link),contour);

				   c_cont++;   /* update the number of contours */

				} 
			}
 
      		        delete_cc(&cc);
                                        

		}
	    }   
	}
 return(0);
}

/****************************************************************************/
/*                                                                          */
/* --- Subprogram name: zc_plot                                             */
/*                                                                          */
/* --- Functional Description:                                              */
/*                                                                          */
/*	This routine plots the contours of a contour image with the             */
/*	grey level of a slope or orientation image, or with an arbitrary        */
/*      8-bit value                                                         */
/*      Note: only selected points on the contour are plotted.  A contour   */
/*      point if selected iff the most significant bit of the BCC is 0.     */
/*                                                                          */
/*                                                                          */
/* --- Libraries and External Sub-Programs Referenced:                      */
/*                                                                          */
/*                                                                          */
/* --- Calling Procedure:                                                   */
/*                                                                          */
/*     zc_plot    (contour_view_number, xc, yc, contour_image_descriptor,   */
/*                 zc_view_number, xz, yz, zero-crossing_image_descriptor,  */
/*                 grey_scale_value)                                        */
/*                                                                          */
/*                                                                          */
/* --- Input Parameters:                                                    */
/*                                                                          */
/*	- contour_view_number        view in contour type image.                */
/*	- xc, yc                     view origin                                */
/*	- contour_image_descriptor   pointer to image descriptor of contour     */
/*				     type image.                                            */
/*	- zc_view_number             view in zero-crossing type image.          */
/*	- xz, yz                     view origin                                */
/*      - zero-crossing_image_descriptor                                    */
/*                                   pointers to image descriptor of        */
/*                                   zero-crossing image in which contours  */
/*	- grey_scale_value           if this parameter is zero then the         */
/*                                   feature value is plotted, otherwise    */
/*                                   the contour is plotted with this grey- */
/*                                   scale.                                 */
/* --- Output Parameters:                                                   */
/*                                                                          */
/*				     are to be plotted.                                     */
/*                                                                          */
/*	Function returns value 0 if following is successfully completed,    */
/*	-1 otherwise. 							    */
/*                                                                          */
/* --- Global Parameters: none.                                             */
/*                                                                          */
/*      struct view_type views[]                                            */
/*                                                                          */
/* --- Local Variables:                                                     */
/*                                                                          */
/*	- z_c                   temporary pointer to array image; used to       */
/*	- cont,nx,ny            scratch variables                               */
/*	- contour               pointer to contour node in contour image        */
/*	- feature_contour       pointer to contour node in feature image        */
/*	- thresh                threshold on the number of pixels that a        */
/*				            must have to be plotted.                        */
/*  - buffer	            character buffer for interactive I/O            */
/*                                                                          */
/*                                                                          */
/* --- Bugs:                                                                */
/*                                                                          */
/*	None (by now)                                                           */
/*                                                                          */
/* --- Author: G. Sandini & M. Tistarelli  UG - DIST                        */
/*                                                                          */
/* --- Revisions                                                            */
/*         date:     02-5-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Adapt routine to utilise features images  rather       */
/*                   than the original file of features.                    */
/*                                                                          */
/*         date:     26-6-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Adapt routine to plot selected points only.            */
/*                                                                          */
/*         date:     11-8-86                                                */
/*         revisor:  D. Vernon TCD                                          */
/*         reason:   Delete prompt for threshold length: use default of 4   */
/*		     (user should utilise selection mechanism to discard            */
/*		     short contours).                                               */
/*                                                                          */
/****************************************************************************/

int zc_plot(int contour_view_number, int xc, int yc,struct image_descriptor_type *contour_image_descriptor,
            int zc_view_number, int xz, int yz, struct image_descriptor_type *zero_crossing_image_descriptor,
            int grey_scale_value)
{
   extern struct view_type views[];

   int nx,ny,x,y;
   struct contour_node_type *contour, *feature_contour;
   int cont;
   array_image_type z_c;
   int thres;

   /*  Test parameters passed. */

   if (contour_image_descriptor == NULL || 
       zero_crossing_image_descriptor == NULL)
      return (-1);

   if ((contour_image_descriptor->image_type != CONTOUR_IMAGE_TYPE && 
        contour_image_descriptor->image_type != SLOPE_IMAGE_TYPE   && 
        contour_image_descriptor->image_type != DEPTH_IMAGE_TYPE   && 
        contour_image_descriptor->image_type != ORIENTATION_IMAGE_TYPE) ||
    (zero_crossing_image_descriptor->image_type != ZERO_CROSSING_IMAGE_TYPE &&
       zero_crossing_image_descriptor->image_type != INTENSITY_IMAGE_TYPE)) {
      printf("incompatible images ....");
      return (-1);
   }

   /* let z_c be used instead of zero_crossing_image_descriptor->image.array_link  */
   /* to avoid indirection and increase efficiency.                                */

   z_c = zero_crossing_image_descriptor->image.array_link;


   /* threshold on the default minimum number of pixel of the contour */

   thres = 4;


   /* blank the view in the zero-crossing image */
if(zero_crossing_image_descriptor->image_type != INTENSITY_IMAGE_TYPE) {
   for (nx = xz; nx < xz + views[zc_view_number].x; nx++)
      for (ny = yz; ny < yz + views[zc_view_number].y; ny++)
         z_c[nx][ny] = 0;
}

   /* find out if the source image is a contour image, a slope image, or a */
   /* orientation image. 						   */

   if (contour_image_descriptor->image_type == CONTOUR_IMAGE_TYPE) {

      contour = contour_image_descriptor->image.contour_link;

      /* plot contours with grey-scale value */

      while (contour != NULL) {
         if (contour->length < thres)
            break;
      
         if ((contour->descriptor_link != NULL) && 
             (contour->descriptor_link->select == TRUE))
            for (cont = 0, nx = contour->start_x, ny = contour->start_y; 
                 cont < contour->length; cont++) {
               chain_calc(contour->value[cont],&nx,&ny); /* compute coordinates */

               x = nx - xc + xz;   /* compute coordinates of translated point */
               y = ny - yc + yz;
               if (x >= xz && x < xz + views[zc_view_number].x  &&
                   y >= yz && y < yz + views[zc_view_number].y )
 
                  /* plot only selected parts of contour */

                  if (((contour->value[cont]) & '\200')  == 0)               
                     z_c[x][y] = grey_scale_value;
            }
         contour = contour->next;
      }

   }
   else {

      /* it is a feature image so we must use the associated contour image */
      /* direction codes to facilitate the plotting of the feature value.  */

      if (contour_image_descriptor->image_type == SLOPE_IMAGE_TYPE)   
         feature_contour = contour_image_descriptor->image.slope_link;
      else
         feature_contour = contour_image_descriptor->image.orientation_link;

      if (feature_contour != NULL  && 
          feature_contour->descriptor_link != NULL)
         contour = feature_contour->descriptor_link->contour;

      /* plot contours with feature value */

      while (contour != NULL) {
         if (contour->length < thres)
            break;

         if ((contour->descriptor_link != NULL) && 
             (contour->descriptor_link->select == TRUE))
            for (cont = 0, nx = contour->start_x, ny = contour->start_y; 
                 cont < contour->length; cont++) {
               chain_calc(contour->value[cont],&nx,&ny); /* compute the coordinates */

               x = nx - xc + xz;   /* compute coordinates of translated point */
               y = ny - yc + yz;
               if (x >= xz && x < xz + views[zc_view_number].x  &&
                   y >= yz && y < yz + views[zc_view_number].y )
 
                  /* plot only selected parts of contour */

                  if (((contour->value[cont]) & '\200')  == 0)               
                     if (grey_scale_value == 0)
                        z_c[x][y] = feature_contour->value[cont];
                     else
                        z_c[x][y] = grey_scale_value;
            }
         contour = contour->next;
         feature_contour = feature_contour->next;
      }
   }
   return(0);
}


/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name: zero_crossings                                      */
/*                                                                           */
/* ---  Functional Description: Detects zero crossing gradients and angles   */
/*      in an image convolved with a Laplacian of Gaussian mask.             */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*                   stdio.h                                                 */
/*                   math.h                                                  */
/*                   dsdef.h                                                 */
/*                   blank_menu()                                            */
/*                   cursor_position()                                       */
/*                   get_response()                                          */
/*                   request_image_identification()                          */
/*                                                                           */
/* ---  Calling Procedure:                                                   */
/*                                                                           */
/*     i= zero_crossings(view_number_1, x_1, y_1 image_descriptor_1,         */
/*                       view_number_2, x_2, y_2 image_descriptor_2,         */
/*                       sigma,                                              */
/*                       contour_flag,                                       */
/*                       contour_image_descriptor,                           */
/*                       slope_image_descriptor,                             */
/*                       orientation_image_descriptor);                      */
/*                                                                           */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*      sigma         .. amplitude of the positive lobe of the D2g mask.     */
/*      view_number_1 .. source view paramenters.                            */
/*      x_1, y_1      .. source view start coordinates.                      */
/*      image_descriptor_1 .. pointer to a structure containing the          */
/*                            Laplacian of Gaussian source image.            */
/*                                                                           */
/*      view_number_2 .. zero-crossing destination view parameters.          */
/*      x_2, y_2      .. zero-crossing destination view start coordinates.   */
/*      image_descriptor_2 .. pointer to a structure where the zero-crossing */
/*                            destination image will be placed.              */
/*      contour_flag ... TRUE (1) => generate contour_based representations  */
/*                       (contour, slope, and orientation)                   */
/*      contour_image_descriptor,                                            */
/*      slope_image_descriptor,                                              */
/*      orientation_image_descriptor .. pointers to image descriptors.       */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*      i= 0 if zero_crossings is completed successfully.                    */
/*      i= -1 if zero_crossings is completed unsuccessfully.                 */
/*                                                                           */
/* ---  Global Parameters:                                                   */
/*                                                                           */
/*      struct view_type views[]                                             */
/*                                                                           */
/*                                                                           */
/* ---  Local Variables: Many.                                               */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author:  Guilio Sandini                                              */
/*                                                                           */
/* ---  Revisions  S.O'N.                                                    */
/*                                                                           */
/*      date:      24/09/85                                                  */
/*      revision:  Replaced machine dependant code.                          */
/*      reason:                                                              */
/*                                                                           */
/*      date:      5/8/86                                                    */
/*      revisor:   David Vernon.                                             */
/*      reason:    pass contour representation image descriptors explicitly  */
/*                 so that user interaction is removed to the                */
/*                 calling routine.                                          */
/*                                                                           */
/*      date:      3/12/86                                                   */
/*      revisor:   Massimo Tistarelli (at TCD)                               */
/*      reason:    THe computation of the orientation values is modified     */
/*                 so that the angular resolution is increased up to 254     */
/*                 different values (the maximum allowed with 8 bits)        */
/*                                                                           */
/*      date:      10/08/87                                                  */
/*      revisor:   Massimo Tistarelli                                        */
/*      reason:    Added code for execution on Eidobrain.....                */
/*                                                                           */
/*      date:      29-05-89                                                  */
/*      revisor:   Paul                                                      */
/*      reason:    removed printf statements for TDS operation               */
/*                                                                           */
/*      date:      14-11-89                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    Removed rudundant code and increase speed by 15%          */
/*                                                                           */
/*      date:      14-11-89                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    added two additional parameters to the routine:           */
/*                 iconic_flag and zero_crossing_image_descriptor.           */
/*                 If iconic_flag is true it means that the calling routine  */
/*                 requires that the zero-crossing slope and orientation     */
/*                 values are to be represented iconically and they are      */
/*                 placed in image_descriptor_2 and                          */
/*                 zero_crossing_image_descriptor, respectively.             */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

int zero_crossings(int view_number_1, int x_1, int y_1, struct image_descriptor_type *image_descriptor_1,
                   int view_number_2, int x_2, int y_2, struct image_descriptor_type *image_descriptor_2,
                   double sigma,
                   int contour_flag,
                   struct image_descriptor_type *contour_image_descriptor, struct image_descriptor_type *slope_image_descriptor,
                   struct image_descriptor_type *orientation_image_descriptor,
                   int iconic_flag, struct image_descriptor_type *zero_crossing_image_descriptor)
{
   extern struct view_type views[];
   extern short check_eido_device(), eido_zc();

   float delta,func,x2;
   float arg,deltax,deltay;
   float fnorm,tD2g;
   float fang;
   float half_ang_val, factor;
   int grad,
       ang,
       minimum,
       isave,
       jsave; 
   int flag;
   int nptmas2;
   int pixel;
   int pleft2,prigh2,pup2,pdown2;
   int pleft,pright,pup,pdown;
   int size_1_x, size_1_y; /* extents of the views with respect to the image */
   int size_2_x, size_2_y; /* domains are stored in these  4  locations.     */
                           /* Hence, the views may be located anywhere in an */
                           /* image including views that are partially       */
                           /* outside the image domain. */ 


   register int i,j,k,i_limit,j_limit;
   array_image_type image_1, image_2, image_3, image_4;

/******************************************************************************/
/*                                                                            */
/*              Test parameters passed.                                       */
/*                                                                            */
/******************************************************************************/

   if (image_descriptor_1 == NULL || image_descriptor_2 == NULL) return (-1);

   if (view_number_1 < 0 || view_number_2 > MAX_NUMBER_VIEWS ||
       view_number_2 < 0 || view_number_2 > MAX_NUMBER_VIEWS ) return (-1);

/*                                                                           */
/*          Test views to see if they are the same extent.                   */
/*                                                                           */

   if (views[view_number_1].x != views[view_number_2].x ||
       views[view_number_1].y != views[view_number_2].y) return (-1);

/*                                                                            */
/*   normalizes so that if there is an ideal step of luminance, the           */
/*   zero crossing slope is equal to an half of the step amplitude            */
/*                                                                            */
   factor = (float) (1./12.);
   fnorm= (float) (0.7272 * sigma);
#ifndef TDS
   //printf("zero-crossings: sigma  %f, fnorm  %f\n",sigma,fnorm);
#endif
/*  determine half the number of points in the mask */

   arg = (float)(sigma*sigma);                  
   delta  =  1 / arg;
   nptmas2=0;
   i=0;
   do {
      x2  = (float)(i*i);
      arg  = -delta*x2;
      func =  exp(arg);
      tD2g =  func*(1-2*x2*delta); 
      if (tD2g < 0.0 ) tD2g = (float)(tD2g*127.- .5);
      else  tD2g = (float)(tD2g*127.+.5);
      i++;
   }  while ((int)tD2g != 0);

   nptmas2=i-2; /* half the mask size determined */

   /* let image_x be used instead of image_descriptor_x->image.array_link */
   /* to avoid indirection and increase efficiency.                       */

   image_1 = image_descriptor_1->image.array_link;
   image_2 = image_descriptor_2->image.array_link;

   if (iconic_flag) {
      if (zero_crossing_image_descriptor == NULL)
         return(-1);
      else
         image_3 = zero_crossing_image_descriptor->image.array_link;
   }
   else {
      if((generate_array_image(image_descriptor_2->size, &(image_3))) == -1) return(-1);
   }

/*                                                                            */
/*     Find the extent of the views within the image domains.                 */
/*                                                                            */

   size_1_x = views[view_number_1].x;
   if ((size_1_x + x_1) > image_descriptor_1->size)
      size_1_x = image_descriptor_1->size - x_1;
   size_1_y = views[view_number_1].y;
   if ((size_1_y + y_1) > image_descriptor_1->size)
      size_1_y = image_descriptor_1->size - y_1;

   i_limit = size_1_x-2-nptmas2+x_1;
   j_limit = size_1_y-2-nptmas2+y_1;

/*****************************************************************************/
/*                                                                           */
/*          Now include conditional compilation for usage of                 */
/*               Eidobrain-fast-zero_crossings extraction                    */
/*                                                                           */
/*****************************************************************************/

#ifdef DUMMY_DEV
   check_eido_device();

/*****************************************************************************/
/*                                                                           */
/*            zero crossings extraction from the inpute image                */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*       There is the implicit assumption that the window is TOTALLY         */
/*                         inside the image                                  */
/*                                                                           */
/*****************************************************************************/

   eido_zc(image_1,(short)x_1,(short)y_1,
           image_2,(short)x_2,(short)y_2,
           image_3,(short)x_2,(short)y_2,
           (short)(views[view_number_1].x),(short)(views[view_number_1].y),
           sigma, nptmas2);
   close_eido_device();
#else

   half_ang_val = ANGULAR_VALUES/2.; /* half the number of angular values */
/*                                                                        */
/* ANGULAR_VALUES is a costant (actually equal to 254) defined in dsdef.h */

/*                                                                            */
/*     Find the extent of the views within the image domains.                 */
/*                                                                            */
   size_2_x = views[view_number_2].x;
   if ((size_2_x + x_2) > image_descriptor_2->size)
      size_2_x = image_descriptor_2->size - x_2;
   size_2_y = views[view_number_2].y;
   if ((size_2_y + y_2) > image_descriptor_2->size)
      size_2_y = image_descriptor_2->size - y_2;

   /* clear borders */

   for (i=0; i<2+nptmas2; i++)  {   
      for (k=y_2; k<size_2_y+y_2;  k++) { 
         image_2[x_2+i][k]=0;
         image_3[x_2+i][k]=0;
         image_2[(size_2_x+x_2-i-1)][k]=0;
         image_3[(size_2_x+x_2-i-1)][k]=0;
      }
   }        
   for (i=x_2; i<size_2_x+x_2; i++)  {   
      for (k=0; k<nptmas2+2;  k++) { 
         image_2[i][k+y_2]=0;
         image_3[i][k+y_2]=0;
         image_2[i][size_2_y+y_2-k-1]=0;
         image_3[i][size_2_y+y_2-k-1]=0;
      }
   }


/*                                                                            */
/*                  main loop for scanning rows and columns.                  */
/*                                                                            */
   for ( i=x_1 + 2 + nptmas2; i<i_limit; i++)  {
#ifndef TDS
      //if (i % 50 == 0) printf("zero-crossings: row %d\n",i);
#endif
      for (j=y_1 + 2 + nptmas2; j<j_limit; j++) { 

/*                                                                            */
/*                      initialization                                        */
/*                                                                            */
         grad = 0;        
         ang =  0;
         pixel = (/*0xff & */image_1[i][j])-128;    /* current pixel */
         pright = (/*0xff & */image_1[i][j+1])-128; /* pixel at right */
         pup = (/*0xff & */image_1[i-1][j])-128;    /* pixel up */
/*                                                                            */
/*     reads a 5*5 cross centered at the current pixel position (i,j)         */
/*                                                                            */

         if (!(pixel*pright>=0 && pixel*pup>=0)) {  

/*                                                                            */
/* if first neighbours up and right and the current pixel have opposite sign, */
/*                                                        then executes       */

            minimum=abs(pixel);
            flag = FALSE;
            isave = i;
            jsave = j;        
/*                                                                          */
/* search for the nearest value to 128                                      */
/*                                                                          */
            if (abs(pright)<=minimum) {
               minimum = abs(pright);
               isave = i+1;
               flag = TRUE;
            }
            if (abs(pup)<=minimum) {
               isave = i;
               jsave = j+1;
               flag = TRUE;
            }
/*                                                                            */
/* isave and jsave points to the nearest value to 128, reads only if          */
/*                        they aren't equal to i and j                        */
/*                                                                            */

            if (flag == TRUE) {
               pixel = (/*0xff & */image_1[isave][jsave])-128;    /* current pixel */
               pright = (/*0xff & */image_1[isave][jsave+1])-128; /* pixel at right */
               pup = (/*0xff & */image_1[isave-1][jsave])-128;    /* pixel up */
               pleft = (/*0xff & */image_1[isave][jsave-1])-128;  /* pixel at left */
               pdown = (/*0xff & */image_1[isave+1][jsave])-128;  /* pixel down */
               pleft2 = (/*0xff & */image_1[isave][jsave-2])-128; /* pixel two positions left */
               pdown2 = (/*0xff & */image_1[isave+2][jsave])-128; /* pixel two positions down */
               prigh2 = (/*0xff & */image_1[isave][jsave+2])-128; /* pixel two positions right */
               pup2 = (/*0xff & */image_1[isave-2][jsave])-128;   /* pixel two position up */
            }
            else {
               pleft = (/*0xff & */image_1[i][j-1])-128;  /* pixel at left */
               pdown = (/*0xff & */image_1[i+1][j])-128;  /* pixel down */
               pleft2 = (/*0xff & */image_1[i][j-2])-128; /* pixel two positions left */
               pdown2 = (/*0xff & */image_1[i+2][j])-128; /* pixel two positions down */
               prigh2 = (/*0xff & */image_1[i][j+2])-128; /* pixel two positions right */
               pup2 = (/*0xff & */image_1[i-2][j])-128;   /* pixel two position up */
            }

/*                                                                            */
/* calculates the gradient modulus with an approximation over 5 points        */
/*                                                                            */
              deltax=(factor)*((float)(pleft2-8*pleft+8*pright-prigh2));
              deltay=(factor)*((float)(pdown2-8*pdown+8*pup-pup2));
              arg=deltax*deltax+deltay*deltay;
              grad=(int)(fnorm*(sqrt(arg))+.5); 
/*                                                                            */
/*          rounds to the nearest integer                                     */
/*                                                                            */
              fang=(float)sign(.5,deltax); 
/*                                                                            */
/*      initializes the angle according to the sign of deltax                 */
/*                                                                            */
              if (deltay!=0.0) fang=(float)(atan2(deltax,deltay)/3.141592654);
/*                                                                            */
/*         __pi is pi greco                                                   */
/*    normalizes the angle with a precision of about 2 degrees                */
/*        (254 different values), so that 1 <= ang <= 254                     */
/*                                                                            */

              ang=(int)((fang + 1.)*half_ang_val + 1.5); 
              if (ang > ANGULAR_VALUES) ang=1;

         }  /* end if */ 
/*                                                                            */
/*       Write the gradient and the angle if it is within view.               */
/*                                                                            */

/*
         if (i < x_2 + size_2_x && j < x_2 + size_2_y) {
*/
            image_2[i][j] = grad;
            image_3[i][j] = ang;

/*         } */
      } /* end column */
   } /* end rows */

#endif

/* image_2 and image_3 now contain the gradient (slope) and angle (orientation) values */
/* If required to do so, we now generate the slope and orientation images, before      */
/* we generate the thresholded zero-crossing image in image_2.                */

   if (contour_flag) {

      /* since zc_fol destroys the image data (zero-crossings) */
      /* we need to retain a copy of it                        */

      if((generate_array_image(image_descriptor_2->size, &(image_4))) == -1) return(-1);
      for (i=0; i<image_descriptor_2->size; i++) 
         for (j=0; j<image_descriptor_2->size; j++)
            image_4[i][j] = image_2[i][j];

      /* pass the temporary image; not the real one */

      image_descriptor_2->image.array_link = image_4;

      /*** generate contour image ***/

      zc_fol(view_number_1,x_1,y_1,image_descriptor_2,contour_image_descriptor);

      /* now restore the original */

      image_descriptor_2->image.array_link = image_2;
      delete_array_image(image_descriptor_2->size,image_4);

      /*** generate slope and orientation images ***/
       
      zc_features(contour_image_descriptor,slope_image_descriptor,orientation_image_descriptor, image_2, image_3);

      /*** generate contour descriptors ***/

      build_contour_descriptors(contour_image_descriptor,
                                slope_image_descriptor,
                                orientation_image_descriptor,
                                NULL,  /* no disparity image */
                                NULL,  /* no velocity image  */
                                NULL); /* no depth image     */

   }

   /* threshold the zero-crossing image */
   
   if (!iconic_flag){
      for (i=x_2; i<i_limit; i++)
         for (j=y_2; j<j_limit; j++)
            if (image_2[i][j] != 0) {
               //printf("zero-crossing at %d %d\n",i,j);
               image_2[i][j] =  (unsigned char) 255;
            }

      delete_array_image(image_descriptor_2->size,image_3);
   }
   return (0);
}



