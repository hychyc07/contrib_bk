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
 * 30/05/11  Started implementation DV
 *
 */ 


#include "iCub/selectiveTuning.h"


selectiveTuning::selectiveTuning() {
   debug = false;
}

bool selectiveTuning::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - selectiveTuning.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("selectiveTuning"), 
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

   visualImageInputPortName       = "/";
   visualImageInputPortName      += getName(
                                    rf.check("visualImageInPort", 
                                    Value("/visualImage:i"),
                                    "Visual input image port (string)").asString()
                                    );

   biasImageInputPortName         = "/";
   biasImageInputPortName        += getName(
                                    rf.check("biasImageInPort", 
                                    Value("/biasImage:i"),
                                    "Bias input image port (string)").asString()
                                    );

    
   wtaImageOutputPortName         = "/";
   wtaImageOutputPortName         += getName(
                                    rf.check("wtaImageOutPort", 
                                    Value("/wtaImage:o"),
                                    "Visual image with cross-hairs output image port (string)").asString()
                                    );

   wtaUnitsOutputPortName         = "/";
   wtaUnitsOutputPortName        += getName(
                                    rf.check("wtaUnitsOutPort", 
                                    Value("/wtaUnits:o"),
                                    "Number and coordinates of the winning units port(string)").asString()
                                    );

   numberOfLevels                 = rf.check("numberOfLevels",
                                    Value(4),
                                    "Number of levels in the STM pyramid (int)").asInt();

   resolutionRatio                = (float) rf.check("resolutionRatio",
                                    Value(0.5),
                                    "Ratio of the resolution of level i+1 to the resolution of level i in the STM pyramid (int)").asDouble();

   bottomLevelRFSize              = rf.check("bottomLevelRFSize",
                                    Value(32), //32
                                    "Size of the receptive fields of each interpretive unit at level 1 (int)").asInt();

   topLevelRFSize                 = rf.check("topLevelRFSize",
                                    Value(4),
                                    "Size of the receptive fields of each interpretive unit at level L (int)").asInt();

   gamma                          = rf.check("gamma",
                                    Value(4),
                                    "maximum number of iterations in the winner-take-all competition (int)").asInt();
 
 
   if (debug) {
      printf("selectiveTuning: module name is                    %s\n",moduleName.c_str());
      printf("selectiveTuning: robot name is                     %s\n",robotName.c_str());
      printf("selectiveTuning: number of levels is               %d\n",numberOfLevels);
      printf("selectiveTuning: resolution ratio is               %f\n",resolutionRatio);
      printf("selectiveTuning: interpretive RF size level 1 is   %d\n",bottomLevelRFSize);
      printf("selectiveTuning: interpretive RF size level L is   %d\n",topLevelRFSize);
      printf("selectiveTuning: gamma is                          %d\n",gamma);
      printf("selectiveTuning: port names are\n%s\n%s\n%s\n%s\n\n",visualImageInputPortName.c_str(),
                                                                   biasImageInputPortName.c_str(),
                                                                   wtaImageOutputPortName.c_str(),
                                                                   wtaUnitsOutputPortName.c_str()
                                                                  );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!visualImageIn.open(visualImageInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << visualImageInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!biasImageIn.open(biasImageInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << biasImageInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!wtaImageOut.open(wtaImageOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << wtaImageOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!wtaUnitsOut.open(wtaUnitsOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << wtaUnitsOutputPortName << endl;
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

   selectiveTuningThread = new SelectiveTuningThread(&visualImageIn, 
                                                     &biasImageIn, 
                                                     &wtaImageOut,
                                                     &wtaUnitsOut, 
                                                     &numberOfLevels, 
                                                     &resolutionRatio, 
                                                     &bottomLevelRFSize,
                                                     &topLevelRFSize,
                                                     &gamma);

   /* now start the thread to do the work */

   selectiveTuningThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool selectiveTuning::interruptModule()
{
   visualImageIn.interrupt();
   biasImageIn.interrupt();
   wtaImageOut.interrupt();
   wtaUnitsOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool selectiveTuning::close()
{
   
   /* stop the thread */

   selectiveTuningThread->stop();

   visualImageIn.close();
   biasImageIn.close();
   wtaImageOut.close();
   wtaUnitsOut.close();
   handlerPort.close();

   return true;
}


bool selectiveTuning::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set gamma <p>   ... set the maximum number of iterations in the winner-take-all competition \n";   


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
      if (command.get(1).asString()=="gamma") {
         gamma = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool selectiveTuning::updateModule()
{
   return true;
}



double selectiveTuning::getPeriod()
{
   /* module periodicity (seconds), called implicitly by selectiveTuning */
    
   return 0.1;
}

SelectiveTuningThread::SelectiveTuningThread(BufferedPort<ImageOf<PixelRgbFloat> > *visualImageIn, 
                                             BufferedPort<ImageOf<PixelRgbFloat> > *biasImageIn, 
                                             BufferedPort<ImageOf<PixelRgbFloat> > *wtaImageOut,
                                             BufferedPort<Bottle >                 *wtaUnitsOut,
                                             int *numberOfLevelsValue, 
                                             float *resolutionRatioValue,
                                             int *bottomLevelRFSizeValue,
                                             int *topLevelRFSizeValue,
                                             int *gammaValue)
{
   visualImagePortIn                = visualImageIn;
   biasImagePortIn                  = biasImageIn;
   wtaImagePortOut                  = wtaImageOut;
   wtaUnitsPortOut                  = wtaUnitsOut;
   numberOfLevels                   = numberOfLevelsValue; 
   resolutionRatio                  = resolutionRatioValue;
   bottomLevelRFSize    = bottomLevelRFSizeValue; 
   topLevelRFSize       = topLevelRFSizeValue;
   gamma                            = gammaValue;
}

bool SelectiveTuningThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;
    visualInput = NULL;  
    biasInput  = NULL;

    return true;
}

void SelectiveTuningThread::run(){

   /* 
    * Compute the focus of attention
    */ 

   if (debug) {
      printf("selectiveTuningThread: parameters are %d %f %d %d %d\n\n",
             *numberOfLevels, *resolutionRatio, 
             *bottomLevelRFSize, *topLevelRFSize, 
             *gamma);
   }


   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
         
      attention_x = 0;
      attention_y = 0;
      visual_width = 0;
      visual_height = 0;
      visual_depth = 0;
      bias_width = 0;
      bias_height = 0;
      bias_depth = 0;

      /* 
       * Step 1: grab the input images
       * =============================
       */

      if (debug) cout << "selectiveTuningThread: grabbing images " << endl;

      do {
         visualImage = visualImagePortIn->read(true);
      } while ((visualImage == NULL) && (isStopping() != true));  // exit loop if shutting down
      
      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      
      if (debug) cout << "selectiveTuningThread: grabbed visual image " << endl;

      visual_width  = visualImage->width();
      visual_height = visualImage->height();
      visual_depth  = 3;
      if (debug) printf("selectiveTuningThread: visual width = %d, visual height = %d, visual depth = %d\n",visual_width, visual_height, visual_depth);
    

      biasImage = biasImagePortIn->read(false); // the bias image is optional

      if (debug) cout << "selectiveTuningThread: grabbed bias image " << endl;

      if (biasImage != NULL) {
         bias_width  = biasImage->width();
         bias_height = biasImage->height();
         bias_depth  = 3;
      }
      if (debug) printf("selectiveTuningThread: bias width = %d, bias height = %d, bias depth = %d\n",bias_width, bias_height, bias_depth);

      if (visualInput == NULL) {
          visualInput = new DVimage(visual_width, visual_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (biasInput == NULL) {
          biasInput = new DVimage(bias_width, bias_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }


      /*  now copy the images */

      for (x = 0; x < visual_width; x++) {
         for (y = 0; y < visual_height; y++) {
            floatRgbPixel = visualImage->safePixel(x,y); // convert to grey-scale
            float_temp =  (float)((floatRgbPixel.r + floatRgbPixel.g + floatRgbPixel.b) / 3.0);
            visualInput->put_pixel(x, y, float_temp);
        }
      } 

      for (x = 0; x < bias_width; x++) {
         for (y = 0; y < bias_height; y++) {
            floatRgbPixel = biasImage->safePixel(x,y);
            float_temp =  (float)((floatRgbPixel.r + floatRgbPixel.g + floatRgbPixel.b) / 3.0);
            biasInput->put_pixel(x, y, float_temp);
        }
      }

         
      // NB should really check here that the resolution of the bias image matches that of level L in the pyramid

      for (x=0; x<*numberOfLevels; x++) {
         resolution[x] = (int) (visual_width * pow( *resolutionRatio, x ));
         if (debug) cout << "selectiveTuningThread: pyramid resolutions " << resolution[x] << endl;
      }

      for (x=0; x<*numberOfLevels; x++) {
         interpretiveUnitReceptiveFields[x] = (int) (*bottomLevelRFSize + (*topLevelRFSize - *bottomLevelRFSize) * ((float)x /(float)(*numberOfLevels-1)));
         if (debug) cout << "selectiveTuningThread: interpretive units receptive fields " << interpretiveUnitReceptiveFields[x] << endl;
      }

      /* 
       * Step 2: compute focus of attention
       * ==================================
       *
       */
 
      if (debug) cout << "selectiveTuningThread: computing focus of attention" << endl;
      

      selective_tuning(*numberOfLevels,
                       resolution,
                       interpretiveUnitReceptiveFields,
                       biasInput,
                       visualInput,
                       *gamma,
                       &number_of_winning_units,
                       winning_units);
 
            
      /* 
       * Step 3: build and write out the bottle with the winning unit data
       * =================================================================
       */

      if (debug) cout << "selectiveTuningThread: building and writing WTA bottle " << endl;

      wta_bottle.clear();
      wta_bottle.addString("size");
      wta_bottle.addInt(number_of_winning_units);

      for (x=0; x<number_of_winning_units; x++) {

         wta_bottle.addString("x");
         wta_bottle.addInt(winning_units[x].x);

         wta_bottle.addString("y");
         wta_bottle.addInt(winning_units[x].y);
      }
   
      // now cross check to see if the bottle contents are sensible

      if (debug) {
         cout << "wta_bottle " << wta_bottle.toString() << endl;
         cout << "wta_bottle " << wta_bottle.get(0).asString() << " " << wta_bottle.get(1).asInt() << endl;
         for (x=0; x<number_of_winning_units; x++) {
            cout <<   "wta_bottle " << wta_bottle.get(4*x+2).asString() << " " << wta_bottle.get(4*x+3).asInt() << " " << wta_bottle.get(4*x+4).asString() << " " << wta_bottle.get(4*x+5).asInt() << endl;
         }
      }

      wtaUnitsPortOut->prepare() = wta_bottle;
      wtaUnitsPortOut->write();
    

      /* 
       * Step 4: copy input image back to output, add cross-hairs on winning units, and write it out
       * =================================================================================================
       */

      if (debug) cout << "selectiveTuningThread: sending output images " << endl;
     
      ImageOf<PixelRgbFloat> &wtaImage  = wtaImagePortOut->prepare();
      wtaImage.resize(visual_width,visual_height);

      for (x = 0; x < visual_width; x++) {
         for (y = 0; y < visual_height; y++) {

            floatRgbPixel = visualImage->safePixel(x,y);
            wtaImage(x,y) = floatRgbPixel;
         }
      }
      
      /* draw cross-hairs on each winning unit              */
      /* red for the first, green for the remaining ones */

      floatRgbPixel.r=(float) 255;
      floatRgbPixel.g=(float) 0;
      floatRgbPixel.b=(float) 0;

      if (number_of_winning_units >= 1)
         addCrossHair(wtaImage, floatRgbPixel, winning_units[0].x, winning_units[0].y, (int) 5);

      floatRgbPixel.r=(float) 0;
      floatRgbPixel.g=(float) 255;
      floatRgbPixel.b=(float) 0;

      for (x=1; x<number_of_winning_units; x++) {
         addCrossHair(wtaImage, floatRgbPixel, winning_units[x].x, winning_units[x].y, (int) 5);
      }
      wtaImagePortOut->write();

      if (debug) cout << "selectiveTuningThread: finished" << endl;
  
   }
}

void SelectiveTuningThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */

   if (biasInput != NULL)     delete biasInput;
   if (visualInput != NULL)   delete visualInput;

}

