/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * 17/06/11  Started work on the development of the module DV
 *
 * 26/07/11  Completed and tested DV
 *
 * 14/08/11  Added covert mode DV
 *
 * 24/08/11  Added post-saccade image input and output, with image shift in covert mode DV
 *
 * 24/10/11  Saccade values are now relative to the previous saccade location in covert mode DV
 *
 * 04/01/12  Removed the reconstruction of the IOR image with the new fixation point at the centre
 *           This was being used (incorrectly) as a form of instantaneous habituation. 
 *
 * 09/01/12  Completely re-written with a new sequence of processing steps.  DV
 *
 * 10/01/12  Added module parameters:
 * 
 *           --imageProcessingLatency <value>     the image processing latency: period to allow processing to complete after motor encoders have stabilized
 *                                                before writing IOR bias to selectiveTuning and then reading fixation coordinates
 *           --encoderSamplingPeriod <value>      the period to allow between two successive samples of the motor encoders when determining whether or not
 *                                                the motors have stopped and the gazed is fixed
 *
 * 25/07/12  The IOR image is now sent several times to avoid deadlock with selectiveTuning
 *           Also IOR image output is now localized in one place
 *           The Inhibition of Return / selectiveTuning behaviour is now a one would expect, with attention scanning salient points in the input visual image
 *           the exact behaviour depends on the value of the iorDecay parameter and the delay incurred during the saccade. 
 *           It also depends on the value of the iorStdDev parameter as this dictates how much of the salient region is inhibited.
 *           Typically, we need fairly low values to allow for micro-saccades when correcting for inaccurate servo control (see below)
 *
 * 27/07/12  Changed frame of reference for saccades from image position to gaze angle in advance of the next phase of development,
 *           i.e. to learn the mapping from image position to gaze angles.  
 *           At present, the angle is computed as the arctan of the focal length and the x or y image position.
 *           However, the wide angle camera lens distorts the image, so that the required angle actually varies as a non-linear function of the x and y position
 *           This mapping has to be learned from errors in the computed saccade angles.
 *
 * 02/08/12  Added new output image: the input visual image weighted by the IOR functions.
 *
 * 17/08/12  Added module parameters     
 *  
 *           --habituationGrowth <value>       The time interval over which the mean value of the Gaussian function should grow to a value of 1.0 (seconds).
 *
 *           --habituationStdDev <value>       The standard deviation of the Gaussian function used to determine the time-varying level of habituation (pixels).
 *
 *           --lateralInhibitionStdDev <value> The time interval over which the mean value of the Gaussian function should decay to zero (seconds).
 * 
 *           --dampingFactor  <value>          multiplicative constant to compute an underestimate of the required gaze angles and avoid overshoot (0 <= dF <= 1.0)
 *
 *           --minimumAngularChange <value>    a saccade is executed only if the change in azimuth or elevation angles is greater than this value (degrees)
 *
 * 17/08/12  Removed module parameters (!)
 *
 *           --imageProcessingLatency <value>  
 *                                           
 *           --encoderSamplingPeriod <value>  
 *
 *           --iorSupport <value>
 *                                           
 */ 
 
/*  includes */

#include "iCub/overtAttention.h"



OvertAttention::OvertAttention()
/* -------------------------------------------------- */
{
    /* intialize private class variables */

    iorDecay                      = 1;     // decay of Gaussian inhibition function
    iorStdDev                     = 5;     // standard deviation of Gaussian inhibition function
    habituationGrowth             = 5;     // growth period of the Gaussian habituation function (seconds)
    habituationStdDev             = 5;     // standard deviation of the Gaussian habituation function (pixels)
    lateralInhibitionStdDev       = 100;   // standard deviation of the Gaussian function for lateral inhibition (pixels)
    dampingFactor                 = 0.4;   // multiplicative constant < 1 to compute an underestimate of the required gaze angles and avoid overshoot
    minimumAngularChange          = 1.0;   // a saccade is executed only if the change in azimuth or elevation is greater than this value (degrees)
    covertMode                    = 0;     // engage overt attention; disengage covert attention
    fxRight                       = 224;   // focal length, x axis, right camera
    fyRight                       = 224;   // focal length, y axis, right camera

    debug = false;
}

OvertAttention::~OvertAttention(){}
/* ----------------------------------------------------- */


 
/*
 * Configure function. This handles all the module initialization
 * It takes as a parameter a previously initialized resource finder object. 
 * This object is used to configure the module
 */

bool  OvertAttention::configure(yarp::os::ResourceFinder &rf)
/* ------------------------------------------------------------------- */
{
 
    /* Process overtAttention module arguments */
 
    
    /* first, get the module name which will form the stem of all module port names */

    moduleName                         = rf.check("name", 
                                         Value("overtAttention"), 
                                         "module name (string)").asString();

    /* 
     * before continuing, set the module name before getting any other parameters,  
     * specifically the port names which are dependent on the module name
     */
    
    setName(moduleName.c_str());

    /* 
     * get the cameraConfig file and read the required parameter values, fx, fy, cx, cy 
     * in both the groups [CAMERA_CALIBRATION_LEFT] and [CAMERA_CALIBRATION_RIGHT]
     */

    cameraConfigFilename  = rf.check("cameraConfig", 
                            Value("icubEyes.ini"), 
                            "camera configuration filename (string)").asString();

    cameraConfigFilename = (rf.findFile(cameraConfigFilename.c_str())).c_str();

    Property cameraProperties;

    if (cameraProperties.fromConfigFile(cameraConfigFilename.c_str()) == false) {
       cout << "overtAttention: unable to read camera configuration file" << cameraConfigFilename;
       return 0;
    }
    else {
       fxRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx", Value(224.0), "fx right").asDouble();
       fyRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fy", Value(224.0), "fy right").asDouble();
    }

    /* now continue getting the rest of the parameters */


    iorDecay                           = rf.check("iorDecay",  
                                         1, 
                                         "Time for Gaussian to decay to zero (seconds, possibly fractional)").asDouble();
   
    iorStdDev                          = rf.check("iorStdDev", 
                                         5, 
                                         "Standard deviation of Gaussian used for inhibition (pixels, possibly fractional) ").asDouble();

    habituationGrowth                  = rf.check("habituationGrowth",  
                                         5, 
                                         "Time for Gaussian to grow to one(seconds, possibly fractional)").asDouble();
   
    habituationStdDev                  = rf.check("habituationStdDev", 
                                         5, 
                                         "Standard deviation of Gaussian used for habituation (pixels, possibly fractional) ").asDouble();
 
    lateralInhibitionStdDev            = rf.check("lateralInhibitionStdDev", 
                                         100, 
                                         "Standard deviation of Gaussian used for lateral inhibition (pixels, possibly fractional) ").asDouble();
   
    dampingFactor                      = rf.check("dampingFactor", 
                                         0.4, 
                                         "control damping factor (pixels, possibly fractional) ").asDouble();

    minimumAngularChange               = rf.check("minimumAngularChange", 
                                         1.0, 
                                         "control damping factor (pixels, possibly fractional) ").asDouble();

    covertMode                         = rf.check("covertMode",  
                                         0, 
                                         "Engage covert mode (0 for overt; 1 for covert)").asInt();
        
    visualImagePortName                = "/";
    visualImagePortName               += getName(
                                         rf.check("visualImageInPort", 
                                         Value("/visualImage:i"), 
                                         "visual image input from the right camera (string)").asString()
                                         );

    wtaUnitsPortName                = "/";
    wtaUnitsPortName               += getName(
                                         rf.check("wtaUnitsInPort",
                                         Value("/wtaUnits:i"), 
                                         "number and coordinates of the potential foci of attention  (string)").asString()
                                         );

    iorImagePortName                  = "/";
    iorImagePortName                 += getName(
                                         rf.check("iorImageOutPort", 
                                         Value("/iorImage:o"), 
                                         "output image of the IOR function values (string)").asString()
                                         );

    postSaccadeImagePortName          = "/";
    postSaccadeImagePortName         += getName(
                                         rf.check("visualImageOutPort", 
                                         Value("/visualImage:o"), 
                                         "output visual image after the completion of the saccade (string)").asString()
                                         );

    weightedImagePortName             = "/";
    weightedImagePortName            += getName(
                                         rf.check("weightedImageOutPort", 
                                         Value("/weightedImage:o"), 
                                         "output the input visual image after weighting it with the IOR function values (string)").asString()
                                         );

    gazeAnglesPortName                 = "/";
    gazeAnglesPortName                += getName(
                                         rf.check("gazeAnglesOutPort", 
                                         Value("/gazeAngles:o"), 
                                         "azimuth, elevation, and vergence angles of the fixation point on which the eye gaze should converge(string)").asString()
                                         );

    robotPortName                 = "/";
    robotPortName                += getName(
                                         rf.check("headStateInPort", 
                                         Value("/headState:i"),
                                         "robot head encoder state port (string)").asString()
                                         );


    if (debug) {
       printf("overtAttention: module name is %s\n",moduleName.c_str());
       printf("overtAttention: parameter values are\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",
                                                                                   iorDecay, 
                                                                                   iorStdDev,
                                                                                   habituationGrowth,
                                                                                   habituationStdDev,
                                                                                   lateralInhibitionStdDev, 
                                                                                   dampingFactor,
                                                                                   minimumAngularChange,
                                                                                   fxRight,fyRight);
       printf("overtAttention: port names are\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n\n",    visualImagePortName.c_str(),
                                                                                   wtaUnitsPortName.c_str(),
                                                                                   iorImagePortName.c_str(),
                                                                                   postSaccadeImagePortName.c_str(),
                                                                                   weightedImagePortName.c_str(),
                                                                                   gazeAnglesPortName.c_str(),
                                                                                   robotPortName.c_str() );
    }
  

    /*
     * open the ports, returning false in the event of any failure
     * returning false means that the module will not be run
     */

    if (!visualImagePortIn.open(visualImagePortName.c_str())) return false;
    if (!wtaUnitsPortIn.open(wtaUnitsPortName.c_str())) return false;
    if (!iorImagePortOut.open(iorImagePortName.c_str())) return false;
    if (!visualImagePortOut.open(postSaccadeImagePortName.c_str())) return false;
    if (!weightedImagePortOut.open(weightedImagePortName.c_str())) return false;
    if (!gazeAnglesPortOut.open(gazeAnglesPortName.c_str())) return false;
    if (!robotPortIn.open(robotPortName.c_str())) return false;
 

    /*
     * Open a port with the same name as the module and attach it to the respond() message handler 
     * Also attach the terminal to the respond() message handler
     * This allows other modules or a user to send commands to the module (in bottles)
     */
   
    handlerPortName =  "/";
    handlerPortName += getName();                                   // use getName() rather than a literal 
    if (!handlerPort.open(handlerPortName.c_str())) return false;   // because the configuration might have changed the name
                                                              
    attach(handlerPort);                                            // attach to port

    /* now start the thread to do the work */

    overtAttentionThread = new OvertAttentionThread(&visualImagePortIn, 
                                                    &wtaUnitsPortIn, 
                                                    &robotPortIn, 
                                                    &iorImagePortOut, 
                                                    &visualImagePortOut,
                                                    &weightedImagePortOut, 
                                                    &gazeAnglesPortOut, 
                                                    &iorDecay, 
                                                    &iorStdDev, 
                                                    &habituationGrowth, 
                                                    &habituationStdDev, 
                                                    &lateralInhibitionStdDev, 
                                                    &dampingFactor, 
                                                    &minimumAngularChange,
                                                    &covertMode, 
                                                    &fxRight, 
                                                    &fyRight);

    overtAttentionThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    if (debug) cout << getName() << " running ..." << endl;
  
    return true;                               // let RFModule know whether everything opened successfuly
}


bool  OvertAttention::close()
/* ----------------------------------- */
{
    /* stop the work thread */

    overtAttentionThread->stop();
   
    /* close ports */

    visualImagePortIn.close();
    wtaUnitsPortIn.close();
    iorImagePortOut.close();
    visualImagePortOut.close();
    weightedImagePortOut.close();
    gazeAnglesPortOut.close();
    handlerPort.close();
    robotPortIn.close();

    return true;
}
 

bool  OvertAttention::interruptModule()
/*---------------------------------------------- */
{
    /* interrupt ports gracefully */

    visualImagePortIn.interrupt();
    wtaUnitsPortIn.interrupt();
    iorImagePortOut.interrupt();
    visualImagePortOut.interrupt();
    weightedImagePortOut.interrupt();
    gazeAnglesPortOut.interrupt();
    handlerPort.interrupt();
    robotPortIn.interrupt();

    return true;
}


/*
 * Message handler. 
 * This allows other modules or a user to send commands to the module (in bottles)
 * This is used to change the parameters of the module at run time or stop the module
 * The following commands are available
 * 
 * help
 * quit
 * set iordecay  <r>   ... set the IOR Gaussian decay time in seconds (seconds; possibly fractional)  
 * set iorstd    <r>   ... set the IOR Gaussian standard deviation (pixels, possibly fractional)  
 * set habgrowth <r>   ... set the habituation Gaussian growth time (seconds, possibly fractional)   
 * set habstd    <r>   ... set the habituation Gaussian standard deviation (pixels, possibly fractional)  
 * set listd     <r>   ... set the lateral inhibition Gaussian standard deviation (pixels, possibly fractional)  
 * set damping   <r>   ... set the saccade damping factor(fractional)  
 * set minang    <r>   ... set the minimum angular  change implemented (degrees; possibly fractional) 
 * set covert    <r>   ... set the covert attention mode (0 for overt; 1 for covert)  
 * (where <n> is an integer number and <r> is a real number)
 */

bool  OvertAttention::respond(const Bottle& command, Bottle& reply) 
/* ------------------------------------------------------------------------- */
{
   ConstString helpMessage = getName() + " commands are: \n" +  
                             "help \n" + "quit \n" + 
                             "set iordecay <r>    ... set the IOR Gaussian decay time in seconds (seconds; possibly fractional)\n" + 
                             "set iorstd <r>      ... set the IOR Gaussian standard deviation (pixels, possibly fractional) \n" +
                             "set habgrowth <r>   ... set the habituation Gaussian growth time (seconds, possibly fractional)\n" + 
                             "set habstd <r>      ... set the habituation Gaussian standard deviation (pixels, possibly fractional) \n" + 
                             "set listd <r>       ... set the lateral inhibition Gaussian standard deviation (pixels, possibly fractional)\n" + 
                             "set damping <r>     ... set the saccade damping factor(fractional) \n" + 
                             "set minang <r>      ... set the minimum angular  change implemented (degrees; possibly fractional) \n" +
                             "set covert <r>      ... set the covert attention flag (0 overt; 1 covert) \n" + 
                             "(where <n> is an integer number and <r> is a real number) \n";
 
   reply.clear(); 
   if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
       cout << helpMessage;
       reply.addString("set iordecay | iorstd | habgrowth | habstd | listd | damping | minang | covert <value>");
    }
    else if (command.get(0).asString()=="set") {
       if      (command.get(1).asString()=="iordecay") {
          iorDecay = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       }
       else if (command.get(1).asString()=="iorstd") {
          iorStdDev = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
       else if (command.get(1).asString()=="habgrowth") {
          habituationGrowth = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
       else if (command.get(1).asString()=="habstd") {
          habituationStdDev = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
       else if (command.get(1).asString()=="listd") {
          lateralInhibitionStdDev = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
       else if (command.get(1).asString()=="damping") {
          dampingFactor = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
      else if (command.get(1).asString()=="minang") {
          minimumAngularChange = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
       else if (command.get(1).asString()=="covert") {
          covertMode = command.get(2).asInt(); // set parameter value
          reply.addString("ok");
       }
    }
  
    return true;
}
  
bool OvertAttention::updateModule()
/* ----------------------------------------- */
{
   return true;
}

 
double  OvertAttention::getPeriod()
/* ----------------------------------------- */
{
    return 0.1; //module periodicity (seconds)
}
 

OvertAttentionThread::OvertAttentionThread(BufferedPort<ImageOf<PixelRgb> > *visualImagePortInValue,  
                                           BufferedPort<Bottle >            *wtaUnitsPortInValue,  
                                           BufferedPort<Vector>             *robotPortInValue,
                                           BufferedPort<ImageOf<PixelRgb> > *iorImagePortOutValue,  
                                           BufferedPort<ImageOf<PixelRgb> > *visualImagePortOutValue,  
                                           BufferedPort<ImageOf<PixelRgb> > *weightedImagePortOutValue,  
                                           BufferedPort<Bottle>             *gazeAnglesPortOutValue, 
                                           double *iorDecayValue,
                                           double *iorStdDevValue, 
                                           double *habituationGrowthValue,
                                           double *habituationStdDevValue,
                                           double *lateralInhibitionStdDevValue,
                                           double *dampingFactorValue, 
                                           double *minimumAngularChangeValue,
                                           int    *covertModeValue,
                                           double *fxRightValue,
                                           double *fyRightValue)
 
/* ------------------------------------------------------------------------------------------------------ */
{
    visualImagePortIn              = visualImagePortInValue;
    wtaUnitsPortIn                 = wtaUnitsPortInValue;
    iorImagePortOut                = iorImagePortOutValue;
    visualImagePortOut             = visualImagePortOutValue;
    weightedImagePortOut           = weightedImagePortOutValue;
    gazeAnglesPortOut              = gazeAnglesPortOutValue;
    robotPortIn                    = robotPortInValue;
    iorDecay                       = iorDecayValue;
    iorStdDev                      = iorStdDevValue;
    habituationGrowth              = habituationGrowthValue;
    habituationStdDev              = habituationStdDevValue;
    lateralInhibitionStdDev        = lateralInhibitionStdDevValue;
    dampingFactor                  = dampingFactorValue;
    minimumAngularChange           = minimumAngularChangeValue;
    covertMode                     = covertModeValue;
    fxRight                        = fxRightValue;
    fyRight                        = fyRightValue;
}

bool OvertAttentionThread::threadInit() 
/* --------------------------------------------- */
{
    /* initialize variables */

    debug = true;

    iorImage         = NULL;

    for (i=0; i<MAX_SACCADES; i++) {
       saccades[i].azimuth   = 0.0;
       saccades[i].elevation = 0.0;
       saccades[i].t = 0;
    }

    number_of_saccades = 0;

    max_azimuth   =  45.0;
    min_azimuth   = -45.0;
    max_elevation =  30.0;
    min_elevation = -30.0;

    previous_azimuth   = 0;
    previous_elevation = 0;

    allowableEncoderVariation     = 0.0;   // if all successive encoder readings are less than this value, 
                                            // we assume that head is stable and that the saccade is complete 
    encoderSamplingPeriod         = 0.05;   // time interval between two successive polls of the motor encoders 
                                            // when checking to see if the motion is complete
    return true;
}


void OvertAttentionThread::run() 
/* -------------------------------------- */
{

   if (debug) {
      printf("overtAttentionThread: parameters  %f %f %f %f %f %f %f %d %f %f\n\n",
             *iorStdDev, *iorDecay, *habituationGrowth, *habituationStdDev, *lateralInhibitionStdDev , *dampingFactor, *minimumAngularChange, *covertMode, *fxRight, *fyRight);
   }

   // initialize parameters of the mapping between image position and gaze angles

   mapping_parameters.fx = *fxRight;
   mapping_parameters.fy = *fyRight;
   initializeMappingCoefficients(&mapping_parameters);

 
   do {
     visualImage = visualImagePortIn->read(true);
   } while ((visualImage == NULL) && (isStopping() != true));  // exit loop if shutting down
   if (isStopping()) return; // abort this loop to avoid make sure we don't continue and possibly use NULL images 

   visual_width  = visualImage->width();
   visual_height = visualImage->height();
   visual_depth  = 3;
   //if (debug) printf("overtAttentionThread: visual width = %d, visual height = %d, visual depth = %d\n",visual_width, visual_height, visual_depth);


   // create the IOR image if necessary

   if (iorImage == NULL) {
      iorImage = new DVimage(visual_width, visual_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
   }
 
   fixating = true;
   habituation_time = yarp::os::Time::now();

   while (!isStopping()) {
       
      if (*covertMode == false) {
         do {
               do {
                  tempEncoderPositions = robotPortIn->read(false);
               } while ((tempEncoderPositions == NULL) && (isStopping() != true));

               if (isStopping()) return; 

               yarp::os::Time::delay(encoderSamplingPeriod);

               do {
                  encoderPositions = robotPortIn->read(false);
               } while ((encoderPositions == NULL) && (isStopping() != true));

               if (isStopping()) return;
   
               stable = true;
         
		         for (i=0; i<6; i++) {
                  if (fabs(encoderPositions->data()[i] - tempEncoderPositions->data()[i]) > allowableEncoderVariation) {
     
                     // if two sequential encoder readings are less than this value, 
                     // we assume that head is stable and that the saccade is complete  
            
                     stable = false;  
                     break;
                  }
               }

         } while (!stable);  
      }

      /* 
       * Step 1: get the new fixation point (typically from selectiveTuning() 
       * ====================================================================
       */

     // do {
         wtaBottle = wtaUnitsPortIn->read(false);
     // } while ((wtaBottle == NULL) && (isStopping() != true));  // exit loop if shutting down

      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
         
 
      /* 
       * Step 2: process the new fixation point  
       * ====================================== 
       */


      if (wtaBottle != NULL) {

         number_of_winning_units = wtaBottle->get(1).asInt();
          
         if (number_of_winning_units > MAX_WINNING_UNITS) 
            number_of_winning_units = MAX_WINNING_UNITS;

         for (i=0; i<number_of_winning_units; i++) {
            winning_units[i].x = wtaBottle->get(4*i+3).asInt();
            winning_units[i].y = wtaBottle->get(4*i+5).asInt();
         }

         if (debug) {
            //cout << "overtAttentionThread: wtaBottle " << wtaBottle->toString() << endl;
            //cout << "overtAttentionThread: wtaBottle " << wtaBottle->get(0).asString() << " " << number_of_winning_units << endl;
            // for (i=0; i<number_of_winning_units; i++) 
             //   cout <<   "overtAttentionThread: wtaBottle " << wtaBottle->get(4*x+2).asString() << " " << winning_units[i].x << " " << wtaBottle->get(4*x+4).asString() << " " << winning_units[i].y << endl;
         }
      }
      else {
         number_of_winning_units = 0;
         if (debug) {
            //cout << "overtAttentionThread: no covert points of attention received" << endl;
         }
      }

	   if (number_of_winning_units > 0) {
         
         selected_saccade = 0; // choose first winning unit 
	 	   
         /* 
          * Step 3: Compute the gaze angles for the selected focus of attention and output them
          * ===================================================================================
          */  

         /* compute the angles */

         controlGaze_x   = (winning_units[selected_saccade].x - (visual_width/2));  // shift position offset to centre of image
         controlGaze_y   = (winning_units[selected_saccade].y - (visual_height/2));
  
         imagePositionToGazeAngles(controlGaze_x, controlGaze_y, mapping_parameters, &azimuth, &elevation); 

         vergence =  0;

         //if (debug) printf("overtAttentionThread: fixation = %d %d; delta gaze    = %4.1f %4.1f\n", controlGaze_x, controlGaze_y, azimuth, elevation );
         //gazeAnglesToImagePosition(azimuth, elevation, mapping_parameters, &controlGaze_x, &controlGaze_y);
         //if (debug) printf("overtAttentionThread: fixation = %d %d; relative gaze = %4.1f %4.1f\n", controlGaze_x, controlGaze_y, azimuth, elevation);
           
         // saccade only if the angular change is greater than some minimum tolerance 

         if (!((fabs(azimuth) < *minimumAngularChange) && (fabs(elevation) < *minimumAngularChange))) {  

            if (*covertMode == 0) {

               do {
                  encoderPositions = robotPortIn->read(true);
               } while ((encoderPositions == NULL) && (isStopping() != true));
               if (isStopping()) break;

               current_azimuth   = (double) encoderPositions->data()[2];   // Neck yaw   ... azimuth      ... positive angle => left 
               current_elevation = (double) encoderPositions->data()[0];   // Neck pitch ... elevation    ... positive angle => up 
               current_vergence  = (double) encoderPositions->data()[5];   // eye vergence

               // damp the gaze change

               azimuth   = azimuth   * *dampingFactor;
               elevation = elevation * *dampingFactor;
                          
               // if (debug) printf("overtAttentionThread: fixation = %d %d; delta gaze    = %4.1f %4.1f\n\n", controlGaze_x, controlGaze_y, azimuth, elevation );

            }
            else {
               current_azimuth   = (double) 0;   // Neck yaw   ... azimuth
               current_elevation = (double) 0;   // Neck pitch ... elevation
               current_vergence  = (double) 0;   // eye vergence
            }

            //if (debug) printf("overtAttentionThread: saccade encoder values = %4.1f %4.1f %4.1\n", current_azimuth, current_elevation, current_vergence);
            //if (debug) printf("overtAttentionThread: encoder limits = %4.1f %4.1f; %4.1f %4.1f\n", min_azimuth, max_azimuth, min_elevation, max_elevation);
            //if (debug) printf("overtAttentionThread: relative gaze angles    = %4.1f %4.1f %4.1f\n", azimuth, elevation, vergence);

            // new gaze 

            new_azimuth   = current_azimuth   + azimuth;    
            new_elevation = current_elevation + elevation;
            new_vergence  = current_vergence  + vergence;
 
            /* limit the relative movement */
 

            if (new_azimuth > max_azimuth)  
               new_azimuth = max_azimuth;
            else if (new_azimuth  < min_azimuth)  
               new_azimuth = min_azimuth;

            if (new_elevation  > max_elevation)  
               new_elevation = max_elevation;
            else if (new_elevation  < min_elevation)  
               new_elevation = min_elevation;

            if (*covertMode == 0) {

               Bottle& bot = gazeAnglesPortOut->prepare();
   
               bot.clear();
               bot.addString("abs");       
               bot.addDouble( -(new_azimuth) );       /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   BUG in iKinGazeCtrl     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
                                                   /* a positive set point angle causes servo to a negative angle (as reported by the endcoder) and vice versa */
               bot.addDouble(new_elevation);
               bot.addDouble(new_vergence);    

               gazeAnglesPortOut->write(); 
 
               //if (debug) printf("overtAttentionThread: target gaze angles           = %4.1f %4.1f\n", new_azimuth, new_elevation);

               do {
                  do {
                     tempEncoderPositions = robotPortIn->read(false);
                  } while ((tempEncoderPositions == NULL) && (isStopping() != true));

                  if (isStopping()) return; 

                  yarp::os::Time::delay(encoderSamplingPeriod);

                  do {
                     encoderPositions = robotPortIn->read(false);
                  } while ((encoderPositions == NULL) && (isStopping() != true));

                  if (isStopping()) return;
   
                  stable = true;
         
		            for (i=0; i<6; i++) {
                     if (fabs(encoderPositions->data()[i] - tempEncoderPositions->data()[i]) > allowableEncoderVariation) {
     
                        // if two sequential encoder readings are less than this value, 
                        // we assume that head is stable and that the saccade is complete  
            
                        stable = false;  
                        break;
                     }
                  }
               } while (!stable);  

               // now read the new position ... this is where we really are, i.e. the actual saccade rather than the desired one
 
               new_azimuth   = (double) encoderPositions->data()[2];   // Neck yaw   ... azimuth      ... positive angle => left 
               new_elevation = (double) encoderPositions->data()[0];   // Neck pitch ... elevation    ... positive angle => up 
               new_vergence  = (double) encoderPositions->data()[5];   // eye vergence
 		  
               //if (debug) printf("overtAttentionThread: new gaze angles              = %4.1f %4.1f %4.1\n", new_azimuth, new_elevation, new_vergence);
            }

            /* 
             * Step 4: add saccade to the list 
             * =============================== 
             */          

            if (number_of_saccades >= MAX_SACCADES) {
               printf("overtAttentionThread: ERROR ... EXCEEDED NUMBER OF SACCADES\n");
            }
            else {

               saccades[number_of_saccades].azimuth   = (new_azimuth    - current_azimuth);
               saccades[number_of_saccades].elevation = (new_elevation  - current_elevation);

               saccades[number_of_saccades].t = yarp::os::Time::now();   // inhibition interval begins after completion of saccade

               if (fixating == true) {
                  fixating = false;
                  saccades[number_of_saccades].start = true;             // a saccade really starts after a period of fixation; other 'saccades' are just corrections
                  printf("overtAttentionThread: Fixation stopped ... \n");
               }
               else {
                  if (*covertMode == 0) {
                     saccades[number_of_saccades].start = false; 
                  }
                  else {
                     saccades[number_of_saccades].start = true; 
                  }
               }

               if (false  &&  debug) {
                  printf("overtAttentionThread: Adding saccade %d: %4.2f %4.2f %4.2f - ", number_of_saccades, saccades[number_of_saccades].azimuth, saccades[number_of_saccades].elevation, saccades[number_of_saccades].t);
                  if (saccades[number_of_saccades].start)
                     printf(" start\n");
                  else
                     printf(" \n");
               }

               number_of_saccades++;
            }

            habituation_time = yarp::os::Time::now(); // reset habituation timer 

         } // if (!((fabs(azimuth) < minimumAngularChange) && (fabs(elevation) < minimumAngularChange))) {  
         else {

            // we have reached the saccade destination so now we start the habituation timer
            
            if (fixating == false) {
               fixating = true;
               habituation_time = yarp::os::Time::now();   // set 
               printf("overtAttentionThread: Fixating started\n");
           }
         }
      } // if (number_of_winning_units > 0)

        

      /* 
       * Step 5: generate the new IOR + Habituation + Lateral Inhibition image 
       * =====================================================================
       */

      // first remove saccades that have decayed to zero

      for (i=0; i<number_of_saccades; i++) {
         if ((yarp::os::Time::now() - saccades[i].t) > *iorDecay) {

            //if (debug) cout << "overtAttentionThread: removing saccade " << i << endl;

            for (j=i; j<number_of_saccades-1; j++) {
               saccades[j].azimuth   = saccades[j+1].azimuth;
               saccades[j].elevation = saccades[j+1].elevation;
               saccades[j].t = saccades[j+1].t;
               saccades[j].start = saccades[j+1].start;
            }
            number_of_saccades--;
         }

         if (false && debug) {
            printf("overtAttentionThread: saccade %d: %4.2f %4.2f %4.2f - ", i, saccades[i].azimuth, saccades[i].elevation, saccades[i].t);
            if (saccades[i].start)
               printf(" start\n");
            else
               printf(" \n");
         }

      }

      // for each saccade point, superimpose the decayed IOR function 
   
      iorImage->initialize();
 
      azimuth   = 0;
      elevation = 0;
      for (i=number_of_saccades-1; i>=0; i--) { //  
         
         if (*covertMode == 0) { // adjust the coordinates of all previous saccades to reflect the shift involved in the current saccade 
            azimuth   = azimuth   - saccades[i].azimuth;
            elevation = elevation - saccades[i].elevation;
         }
         else {
            azimuth   = saccades[i].azimuth;
            elevation = saccades[i].elevation;
         }

         gazeAnglesToImagePosition(azimuth, elevation, mapping_parameters, &controlGaze_x, &controlGaze_y);

         // only create IOR for the first saccade that follows a fixation

         if (saccades[i].start == true) {
            decay = true; // the Gaussian decays with time; let the support be 6 times the std. deviation
            superimposeGaussian (iorImage, *iorStdDev, (int) (*iorStdDev * 6), *iorDecay, controlGaze_x + (visual_width/2), controlGaze_y   + (visual_height/2), saccades[i].t, decay);
         }
         //if (debug) printf("overtAttentionThread: IOR      = %d %d; delta gaze    = %4.1f %4.1f\n", controlGaze_x, controlGaze_y, saccades[i].azimuth,  saccades[i].elevation );
      }

      // now the habituation function 
      // ============================

      decay = false; // the Gaussian grows with time

      if (*covertMode == 0) {

         // overt mode

         superimposeGaussian (iorImage, *habituationStdDev, (int) (*habituationStdDev * 6), *habituationGrowth, visual_width/2, visual_height/2, habituation_time, decay);
      }
      else {

         // covert mode 

         azimuth   = saccades[number_of_saccades-1].azimuth;
         elevation = saccades[number_of_saccades-1].elevation;
         gazeAnglesToImagePosition(azimuth, elevation, mapping_parameters, &controlGaze_x, &controlGaze_y);
         superimposeGaussian (iorImage, *habituationStdDev, (int) (*habituationStdDev * 6), *habituationGrowth, controlGaze_x + visual_width/2, controlGaze_y + visual_height/2, habituation_time, decay);
      }

      iorImage->subtract_from_constant(1.0); // force a uniform zero image to have the value 1.0
 
      // lateral inhibition

      //if (!fixating)
         gaussianApodization (iorImage, (float)*lateralInhibitionStdDev, iorImage);


      /* 
       * Step 6: write out the IOR image, weighted visual image, and raw visual image (probably for storing in / recall from episodic memory)   
       * ==================================================================================================================================== 
       */
           
      do {
         visualImage = visualImagePortIn->read(false);
      } while ((visualImage == NULL) && (isStopping() != true));  // exit loop if shutting down
 
      if (isStopping()) return; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
     
      visual_width  = visualImage->width();
      visual_height = visualImage->height();
      visual_depth  = 3;


  	   ImageOf<PixelRgb> &reverseIorImage  = iorImagePortOut->prepare();
      reverseIorImage.resize(visual_width,visual_height);

      ImageOf<PixelRgb> &weightedImage  = weightedImagePortOut->prepare();
      weightedImage.resize(visual_width,visual_height);

      ImageOf<PixelRgb> &postSaccadeImage  = visualImagePortOut->prepare();
      postSaccadeImage.resize(visual_width,visual_height);
      postSaccadeImage.zero();
 
      for (x = 0; x < visual_width; x++) {
         for (y = 0; y < visual_height; y++) {

            // first, do the IOR image

            iorImage->get_pixel(x,y,&float_pixel_value); 

            pixel_value = 255;
            rgbPixel.r = (int)((float) pixel_value * float_pixel_value);
            rgbPixel.g = (int)((float) pixel_value * float_pixel_value);
            rgbPixel.b = (int)((float) pixel_value * float_pixel_value);

            reverseIorImage(x,y) = rgbPixel;

            // next, do the weighted visual image

            rgbPixel = visualImage->safePixel(x,y);  

            rgbPixel.r = (int)((float) rgbPixel.r * float_pixel_value);
            rgbPixel.g = (int)((float) rgbPixel.g * float_pixel_value);
            rgbPixel.b = (int)((float) rgbPixel.b * float_pixel_value);

            weightedImage(x,y) = rgbPixel;

            // finally, the raw visual image

            // if covert mode, shift image so that focus of attention is in the centre
        
            rgbPixel = visualImage->safePixel(x,y);  

            if (*covertMode == 0) {
               postSaccadeImage(x,y) = rgbPixel;
            }
            else {

               gazeAnglesToImagePosition(saccades[number_of_saccades-1].azimuth, saccades[number_of_saccades-1].elevation, mapping_parameters, &controlGaze_x, &controlGaze_y);

               if ((x - controlGaze_x >= 0) && 
                   (x - controlGaze_x < visual_width)  &&
                   (y - controlGaze_y >= 0) && 
                   (y - controlGaze_y < visual_height)) {
                   postSaccadeImage(x - controlGaze_x,y - controlGaze_y) = rgbPixel;
               }
            }
         }
      }
      
      // put a red cross-hair on the last saccade, if it exists
           
      if(number_of_saccades >0) {

         rgbPixel.r = 255;
         rgbPixel.g = 0;
         rgbPixel.b = 0;

         gazeAnglesToImagePosition(saccades[number_of_saccades-1].azimuth, saccades[number_of_saccades-1].elevation, mapping_parameters, &controlGaze_x, &controlGaze_y);
         addCrossHair(reverseIorImage, rgbPixel, controlGaze_x + (visual_width/2),  controlGaze_y + (visual_height/2), (int) 5);
         addCrossHair(weightedImage,   rgbPixel, controlGaze_x + (visual_width/2),  controlGaze_y + (visual_height/2), (int) 5);
      }
      
      iorImagePortOut->write();
      weightedImagePortOut->write();   
      visualImagePortOut->write();
               
   } // while (!isStopping)
}

void OvertAttentionThread::threadRelease() 
/* ------------------------------------------------ */
{
    // delete dynamically created images

   if (iorImage         != NULL)   delete iorImage;

}

void imagePositionToGazeAngles(int x, int y, mapping_data_type mapping_parameters, double *azimuth, double *elevation) 
{

   *azimuth   = x  * mapping_parameters.a[1];
   *elevation = y  * mapping_parameters.b[1];

}

void gazeAnglesToImagePosition(double azimuth, double elevation, mapping_data_type mapping_parameters, int *x, int *y) 
{
   *x    = (int)(azimuth   / mapping_parameters.a[1]);
   *y    = (int)(elevation / mapping_parameters.b[1]);
}

void initializeMappingCoefficients(mapping_data_type *mapping_parameters) 
{
   int i; 
   
   for (i=0; i<MAX_MAPPING_COEFFICIENTS;i++) {
       mapping_parameters->a[i] = 0;
       mapping_parameters->b[i] = 0;
   }

   mapping_parameters->a[1] = -(1/mapping_parameters->fx) * (180.0/3.14159);  // negative because left is a positive angle so a positive coordinate requires a negative angle
   mapping_parameters->b[1] = -(1/mapping_parameters->fy) * (180.0/3.14159);  // negative because  up  is a positive angle so a positive coordinate requires a negative angle

}
 
/* empty line to make gcc happy */


 