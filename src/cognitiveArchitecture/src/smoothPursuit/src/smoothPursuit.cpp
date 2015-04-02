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
 * 22/12/11  Started work on the development of this module for Rong Li DV
 * 23/07/12  Fixed problem with drifting vergence by using absolute angles instead of relative angles DV
*/ 

 
/*  includes */

#include "iCub/smoothPursuit.h"



SmoothPursuit::SmoothPursuit()
/* -------------------------------------------------- */
{
    /* intialize private class variables with some assumed values */

    gain                        = 0.5;   // constant of proportionality
    fxRight                     = 224;   // focal length, x axis, right camera
    fyRight                     = 224;   // focal length, y axis, right camera
    cxRight                     = 160;   // camera centre, x axis, right camera
    cyRight                     = 120;   // camera centre, y axis, right camera

    debug = false;
}

SmoothPursuit::~SmoothPursuit(){}
/* ----------------------------------------------------- */
 
/*
 * Configure function. This handles all the module initialization
 * It takes as a parameter a previously initialized resource finder object. 
 * This object is used to configure the module
 */

bool  SmoothPursuit::configure(yarp::os::ResourceFinder &rf)
/* ------------------------------------------------------------------- */
{
 
    /* Process smoothPursuit module arguments */
 
    
    /* first, get the module name which will form the stem of all module port names */

    moduleName                         = rf.check("name", 
                                         Value("smoothPursuit"), 
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
       cout << "smoothPursuit: unable to read camera configuration file" << cameraConfigFilename;
       return 0;
    }
    else {
       fxRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx", Value(224.0), "fx right").asDouble();
       fyRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fy", Value(224.0), "fy right").asDouble();
       cxRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cx", Value(160.0), "cx right").asDouble();
       cyRight = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cy", Value(120.0), "cy right").asDouble();
    }

    /* now continue getting the rest of the parameters */

    gain                            = rf.check("gain",  
                                         0.5, 
                                         "constant of proportionality in the angule computation").asDouble();
   
    wtaUnitsPortName                = "/";
    wtaUnitsPortName               += getName(
                                         rf.check("wtaUnitsInPort",
                                         Value("/wtaUnits:i"), 
                                         "number and coordinates of the potential foci of attention  (string)").asString()
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
       printf("smoothPursuit: module name is %s\n",moduleName.c_str());
       printf("smoothPursuit: parameter values are\n%f\n%f\n%f\n%f\n%f\n",gain,fxRight,fyRight,cxRight,cyRight);
       printf("smoothPursuit: port names are\n%s\n%s\n%s\n\n", wtaUnitsPortName.c_str(),
                                                               gazeAnglesPortName.c_str(),
                                                               robotPortName.c_str() );
    }
  

    /*
     * open the ports, returning false in the event of any failure
     * returning false means that the module will not be run
     */

    if (!wtaUnitsPortIn.open(wtaUnitsPortName.c_str())) return false;
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

    smoothPursuitThread = new SmoothPursuitThread(&wtaUnitsPortIn, 
                                                  &robotPortIn, 
                                                  &gazeAnglesPortOut, 
                                                  &gain,                                                  \
                                                  &fxRight, 
                                                  &fyRight,
                                                  &cxRight, 
                                                  &cyRight);

    smoothPursuitThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    if (debug) cout << getName() << " running ..." << endl;
  
    return true;                               // let RFModule know whether everything opened successfuly
}


bool  SmoothPursuit::close()
/* ----------------------------------- */
{
    /* stop the work thread */

    smoothPursuitThread->stop();
   
    /* close ports */

    wtaUnitsPortIn.close();
    gazeAnglesPortOut.close();
    handlerPort.close();
    robotPortIn.close();

    return true;
}
 

bool  SmoothPursuit::interruptModule()
/*---------------------------------------------- */
{
    /* interrupt ports gracefully */

    wtaUnitsPortIn.interrupt();
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
 * set gain <r>    ... set the gain for the transformation from position to angles
 * (where <n> is an integer number and <r> is a real number)
 */

bool  SmoothPursuit::respond(const Bottle& command, Bottle& reply) 
/* ------------------------------------------------------------------------- */
{
   ConstString helpMessage = getName() + " commands are: \n" +  
                             "help \n" + "quit \n" + 
                             "set gain <r>      ... et the gain for the transformation from position to angles \n" +
                             "(where <r> is a real number) \n";
  
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
       if (command.get(1).asString()=="gain") {
          gain = command.get(2).asDouble(); // set parameter value
          reply.addString("ok");
       } 
    }
    return true;
}
  
bool SmoothPursuit::updateModule()
/* ----------------------------------------- */
{
   return true;
}

 
double  SmoothPursuit::getPeriod()
/* ----------------------------------------- */
{
    return 0.05; //module periodicity (seconds)
}
 

SmoothPursuitThread::SmoothPursuitThread(BufferedPort<Bottle >  *wtaUnitsPortInValue,  
                                         BufferedPort<Vector>   *robotPortInValue,
                                         BufferedPort<Bottle>   *gazeAnglesPortOutValue, 
                                         double                 *gainValue,
                                         double                 *fxRightValue,
                                         double                 *fyRightValue,
                                         double                 *cxRightValue,
                                         double                 *cyRightValue)
/* ------------------------------------------------------------------------------------------------------ */
{
    wtaUnitsPortIn                 = wtaUnitsPortInValue;
    gazeAnglesPortOut              = gazeAnglesPortOutValue;
    robotPortIn                    = robotPortInValue;
    gain                           = gainValue;
    fxRight                        = fxRightValue;
    fyRight                        = fyRightValue;
    cxRight                        = cxRightValue;
    cyRight                        = cyRightValue;
}

bool SmoothPursuitThread::threadInit() 
/* --------------------------------------------- */
{
    /* initialize variables */

    debug = false;

    max_azimuth   =  55.0;
    min_azimuth   = -55.0;
    max_elevation =  40.0;
    min_elevation = -40.0;

    previous_azimuth   = 0;
    previous_elevation = 0;

    encoderSamplingPeriod = 0.01; // time interval between two successive polls of the motor encoders 
                                  // when checking to see if the motion is complete

    allowableEncoderVariation = 0.0;  // if all successive encoder readings are less than this value, 
                                      // we assume that head is stable and that the saccade is complete 

    return true;
}


void SmoothPursuitThread::run()
/* -------------------------------------- */
{

   if (debug) {
      printf("smoothPursuitThread: parameters %f %f %f %f %f\n\n", *gain, *fxRight, *fyRight,*cxRight, *cyRight);
   }

   while (!isStopping()) {

      /* 
       * Step 1: get the WTA fixation points input 
       * =========================================
       */

      do {
         wtaBottle = wtaUnitsPortIn->read(true);
      } while ((wtaBottle == NULL) && (isStopping() != true));  // exit loop if shutting down

      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 

      if (wtaBottle != NULL) {

         number_of_winning_units = wtaBottle->get(1).asInt();
          
         if (number_of_winning_units > MAX_WINNING_UNITS) 
            number_of_winning_units = MAX_WINNING_UNITS;

         for (i=0; i<number_of_winning_units; i++) {
            winning_units[i].x = wtaBottle->get(4*i+3).asInt();
            winning_units[i].y = wtaBottle->get(4*i+5).asInt();
         }

         if (debug) {
            //cout << "smoothPursuitThread: wtaBottle " << wtaBottle->toString() << endl;
            //cout << "smoothPursuitThread: wtaBottle " << wtaBottle->get(0).asString() << " " << number_of_winning_units << endl;
            //for (i=0; i<number_of_winning_units; i++) {
            //   cout <<   "smoothPursuitThread: wtaBottle " << wtaBottle->get(4*x+2).asString() << " " << winning_units[i].x << " " << wtaBottle->get(4*x+4).asString() << " " << winning_units[i].y << endl;
            //}
         }
      }
      else {
         number_of_winning_units = 0;
         if (debug) {
            cout << "smoothPursuitThread: no fixation points received" << endl;
         }
      }

      /* 
       * Step 2: Compute the gaze angles for the selected focus of attention and output them
       *         This implement a very basic proportional feedback control loop, 
       *         i.e. the rotation (gaze angle correction) is directly proportional to 
       *         the difference between the position of the current fixation point 
       *         and the position of the camera centre
       * ===================================================================================
       */  

      /* for simplicity, just use the first of the winner-take-all points as the desired fixation point */

      selected_fixation_point = 0;

      /* compute the angles */
  
      controlGaze_x   = (float) (winning_units[selected_fixation_point].x - *cxRight);
      controlGaze_y   = (float) (winning_units[selected_fixation_point].y - *cyRight);
  
      azimuth   = *gain *  (atan2((double)controlGaze_x, (double)*fxRight) * (180.0/3.14159));  // positive because right is a positive angle
      elevation = *gain * -(atan2((double)controlGaze_y, (double)*fyRight) * (180.0/3.14159));  // negative because up is a positive angle
      vergence =  0;
            
      // check to make sure the rotation won't take us outside the allowed limit
      // that is, we place a limit on the absolute gaze angles

      do {
         encoderPositions = robotPortIn->read(true);
      } while ((encoderPositions == NULL) && (isStopping() != true));
      if (isStopping()) break;

      current_azimuth   = (double) encoderPositions->data()[2];   // Neck yaw   ... azimuth
      current_elevation = (double) encoderPositions->data()[0];   // Neck pitch ... elevation
      current_vergence  = (double) encoderPositions->data()[5];   // eye vergence

      if (debug) printf("smoothPursuitThread: encoder values = %4.1f %4.1f %4.1f\n", current_azimuth, current_elevation, current_vergence);
      if (debug) printf("smoothPursuitThread: gaze angles    = %4.1f %4.1f %4.1f\n", azimuth, elevation, vergence);
      //if (debug) printf("smoothPursuitThread: encoder limits = %4.1f %4.1f; %4.1f %4.1f\n", min_azimuth, max_azimuth, min_elevation, max_elevation);

      /* make sure the movement is within bounds */

      Bottle& bot = gazeAnglesPortOut->prepare();
 
      if ((current_azimuth + azimuth     > max_azimuth) ||
          (current_azimuth + azimuth     < min_azimuth) ||
          (current_elevation + elevation > max_elevation) ||
          (current_elevation + elevation < min_elevation))
      {
         azimuth = 0; 
         elevation = 0; 
         vergence =  0;

         bot.clear();
         bot.addString("abs");     
         bot.addDouble(azimuth);    
         bot.addDouble(elevation);
         bot.addDouble(vergence);    
         gazeAnglesPortOut->write();

         limitingMotion = true;
      } 
      else { 
         bot.clear();
         
         /* WARNING ... abs mode doesn't work */

         //bot.addString("abs");     
         //bot.addDouble(current_azimuth    + azimuth);    
         //bot.addDouble(current_elevation  + elevation);
         //bot.addDouble(current_vergence   + vergence);
         //if (debug) printf("smoothPursuitThread: gaze command   = %4.1f %4.1f %4.1f (gain %4.1f)\n", current_azimuth+azimuth, current_elevation+elevation, current_vergence+vergence, *gain);

         bot.addString("rel");     
         bot.addDouble(azimuth);    
         bot.addDouble(elevation);
         bot.addDouble(vergence);
         if (debug) printf("smoothPursuitThread: gaze command   = %4.1f %4.1f %4.1f (gain %4.1f)\n", azimuth, elevation, vergence, *gain);
 
         gazeAnglesPortOut->write();
         
         limitingMotion = false;
      }
             
          
   }
}

void SmoothPursuitThread::threadRelease() 
/* ------------------------------------------------ */
{

}

 
/* empty line to make gcc happy */

