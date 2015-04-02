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
 * 20/07/11  First implementation working DV
 * 03/08/11  Final implementation working (with rectangular RF and max value options) DV
 * 26/07/12  Added biasImageRequired parameter and blocking version of bias image input DV
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

   minRFSize                      = rf.check("minRFSize",
                                    Value(6), 
                                    "Minimum value of the length of the rectangular receptive field (int)").asInt();

   maxRFSize                      = rf.check("maxRFSize",
                                    Value(25),
                                    "Maximum value of the length of the rectangular receptive field (int)").asInt();

   rectangularRF                  = rf.check("rectangularRF",
                                    Value(0),
                                    "Flag to invoke use of rectangular receptive fields (int)").asInt();

   localMax                       = rf.check("localMax",
                                    Value(0),
                                    "Flag to invoke use of local maximum instead of local average when constructing the image pyramid (int)").asInt();
  
   biasImageRequired              = rf.check("biasImageRequired");

 
   if (debug) {
      printf("selectiveTuning: module name is                    %s\n",moduleName.c_str());
      printf("selectiveTuning: robot name is                     %s\n",robotName.c_str());
      printf("selectiveTuning: number of levels is               %d\n",numberOfLevels);
      printf("selectiveTuning: minimum side of RF is             %d\n",minRFSize);
      printf("selectiveTuning: maximum side of RF is             %d\n",maxRFSize);
      printf("selectiveTuning: rectangular RF flag is            %d\n",rectangularRF);
      if (biasImageRequired) printf("selectiveTuning: biasImageRequired flag is set\n");
      else                   printf("selectiveTuning: biasImageRequired flag is not set\n");
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
                                                     &minRFSize,
                                                     &maxRFSize,
                                                     &rectangularRF,
                                                     &localMax,
                                                     &biasImageRequired);

   /* now start the thread to do the work */

   selectiveTuningThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool selectiveTuning::interruptModule()
{
    // BUG .... for some reason, trying to interrupt these ports stops the module stopping.
	//          so I'm not bothering to try anymore DV 02/02/12

   return true;

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
                        "set minRFSize <p>   ... set the maximum number of iterations in the winner-take-all competition \n";   


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
      if (command.get(1).asString()=="minRFSize") {
         minRFSize = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="maxRFSize") {
         maxRFSize = command.get(2).asInt(); // set parameter value
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
    
   return 0.01;
}

SelectiveTuningThread::SelectiveTuningThread(BufferedPort<ImageOf<PixelRgbFloat> > *visualImageIn, 
                                             BufferedPort<ImageOf<PixelRgbFloat> > *biasImageIn, 
                                             BufferedPort<ImageOf<PixelRgbFloat> > *wtaImageOut,
                                             BufferedPort<Bottle >                 *wtaUnitsOut,
                                             int *numberOfLevelsValue, 
                                             int *minRFSizeValue,
                                             int *maxRFSizeValue,
                                             int *rectangularRFValue,
                                             int *localMaxValue, 
                                             bool *biasImageRequiredValue)
{
   visualImagePortIn                = visualImageIn;
   biasImagePortIn                  = biasImageIn;
   wtaImagePortOut                  = wtaImageOut;
   wtaUnitsPortOut                  = wtaUnitsOut;
   numberOfLevels                   = numberOfLevelsValue; 
   minRFSize                        = minRFSizeValue; 
   maxRFSize                        = maxRFSizeValue;
   rectangularRF                    = rectangularRFValue;
   localMax                         = localMaxValue;
   biasImageRequired                = biasImageRequiredValue;
}

bool SelectiveTuningThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;
    visualInput = NULL; 
    biasInput  = NULL;
    biasImage = NULL;

    return true;
}

void SelectiveTuningThread::run(){

   /* 
    * Compute the focus of attention
    */ 

   if (debug) {
      printf("selectiveTuningThread: parameters are %d %f %d %d\n\n", *numberOfLevels, *minRFSize, *maxRFSize);
      if (*biasImageRequired)  printf("selectiveTuningThread: biasImageRequired flag is set\n");
      else                     printf("selectiveTuningThread: biasImageRequired flag is not set\n");
   }

   // initialization
   
   attention_x = 0;
   attention_y = 0;
   visual_width = 0;
   visual_height = 0;
   visual_depth = 0;
   bias_width = 0;
   bias_height = 0;
   bias_depth = 0;
        
 
   do {
      visualImage = visualImagePortIn->read(true);
   } while ((visualImage == NULL) && (isStopping() != true));  // exit loop if shutting down
   if (isStopping()) return; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      
   bias_width  = visualImage->width();
   bias_height = visualImage->height();
   bias_depth  = 3;

   biasInput = new DVimage(bias_width, bias_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);

   for (x = 0; x < bias_width; x++) {
      for (y = 0; y < bias_height; y++) {
         float_temp =  255.0;            // initially no bias: all pixels equally and fully weighted
         biasInput->put_pixel(x, y, float_temp);
      }
   }
 

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      /* 
       * Step 1: grab the input images
       * =============================
       */

      if (debug) cout << "selectiveTuningThread: grabbing images " << endl;


      if (*biasImageRequired) {

         // block until bias image is input

         do {
            biasImage = biasImagePortIn->read(false);
         } while ((biasImage == NULL) && (isStopping() != true));  // exit loop if shutting down
         if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      
      }
      else {
         if (debug) cout << "selectiveTuningThread: optional bias image " << endl;
         biasImage = biasImagePortIn->read(false); // the bias image is optional
      }
 
      if ((biasImage != NULL) && debug) cout << "selectiveTuningThread: grabbed bias image " << endl;

  
      do {
         visualImage = visualImagePortIn->read(true);
      } while ((visualImage == NULL) && (isStopping() != true));  // exit loop if shutting down
      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      
      if (debug) cout << "selectiveTuningThread: grabbed visual image " << endl;

      visual_width  = visualImage->width();
      visual_height = visualImage->height();
      visual_depth  = 3;


      //if (debug) printf("selectiveTuningThread: visual width = %d, visual height = %d, visual depth = %d\n",visual_width, visual_height, visual_depth);
      
      if (biasImage != NULL) {
         bias_width  = biasImage->width();
         bias_height = biasImage->height();
         bias_depth  = 3;
      }
	   else {
		   bias_width  = visualImage->width();
         bias_height = visualImage->height();
         bias_depth  = 3;
	   }

      if (debug) printf("selectiveTuningThread: bias width = %d, bias height = %d, bias depth = %d\n",bias_width, bias_height, bias_depth);

          
      if (visualInput == NULL) {
          visualInput = new DVimage(visual_width, visual_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }

      if (biasInput == NULL) {
         biasInput = new DVimage(bias_width, bias_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
      }
      else { 
         biasInput->get_size(&width,&height);
         if ((width != bias_width) || (height != bias_height)) {
            delete biasInput;
            biasInput = new DVimage(bias_width, bias_height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
         }
      }


      /*  now copy the images */

      for (x = 0; x < visual_width; x++) {
         for (y = 0; y < visual_height; y++) {
            floatRgbPixel = visualImage->safePixel(x,y); // convert to grey-scale
            float_temp =  (float)((floatRgbPixel.r + floatRgbPixel.g + floatRgbPixel.b) / 3.0);
            visualInput->put_pixel(x, y, float_temp);
        }
      } 

	  if (biasImage != NULL) {
         for (x = 0; x < bias_width; x++) {
            for (y = 0; y < bias_height; y++) {
               floatRgbPixel = biasImage->safePixel(x,y);
               float_temp =  (float)((floatRgbPixel.r + floatRgbPixel.g + floatRgbPixel.b) / 3.0);
               biasInput->put_pixel(x, y, float_temp);
           }
         }
	  }
	  
     /*  don't reset the bias image ... use the previous one, i.e. bias is persistent 

     else {
		 for (x = 0; x < bias_width; x++) {
            for (y = 0; y < bias_height; y++) {
               float_temp =  255.0;            // no bias: all pixels equally and fully weighted
               biasInput->put_pixel(x, y, float_temp);
           }
         }
	  }
     */

         

      /* 
       * Step 2: compute focus of attention
       * ==================================
       *
       */
 
      if (debug) cout << "selectiveTuningThread: computing focus of attention" << endl;
      
      selective_tuning(*numberOfLevels,
                       biasInput,
                       visualInput,
                       *minRFSize,
                       *maxRFSize,
                       *rectangularRF,
                       *localMax,
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
         cout << "selectiveTuningThread: wta_bottle " << wta_bottle.toString() << endl;
         cout << "selectiveTuningThread: wta_bottle " << wta_bottle.get(0).asString() << " " << wta_bottle.get(1).asInt() << endl;
         for (x=0; x<number_of_winning_units; x++) {
            cout <<   "selectiveTuningThread: wta_bottle " << wta_bottle.get(4*x+2).asString() << " " << wta_bottle.get(4*x+3).asInt() << " " << wta_bottle.get(4*x+4).asString() << " " << wta_bottle.get(4*x+5).asInt() << endl;
         }
      }

      // SEND THE BOTTLE MANY TIMES .....
      //
      // THIS IS A HACK TO MAKE SURE THE BOTTLE IS RECEIVED BY THE RECEIVER (overtAttention)
      // OTHERWISE overtAttention  GETS OUT OF SYNC WHEN SENDING THE BIAS IMAGE

      for (x=0; x<100; x++) {
         wtaUnitsPortOut->prepare() = wta_bottle;
         wtaUnitsPortOut->write();
      }

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
      /* green for the first, blue for the remaining ones */

      floatRgbPixel.r=(float) 0;
      floatRgbPixel.g=(float) 255;
      floatRgbPixel.b=(float) 0;

      if (number_of_winning_units >= 1) {
         addCrossHair(wtaImage, floatRgbPixel, winning_units[0].x, winning_units[0].y, (int) 5);
         addRectangleOutline(wtaImage, floatRgbPixel, winning_units[0].x, winning_units[0].y,  winning_units[0].rf_x/2, winning_units[0].rf_y/2);
      }

      floatRgbPixel.r=(float) 0;
      floatRgbPixel.g=(float) 0;
      floatRgbPixel.b=(float) 255;

      for (x=1; x<number_of_winning_units; x++) {
         addCrossHair(wtaImage, floatRgbPixel, winning_units[x].x, winning_units[x].y, (int) 5);
         addRectangleOutline(wtaImage, floatRgbPixel, winning_units[x].x, winning_units[x].y,  winning_units[x].rf_x/2, winning_units[x].rf_y/2);
	   }

      /* draw red cross-hair in centre of the image to help with visualization of overt attention */

      floatRgbPixel.r=(float) 255;
      floatRgbPixel.g=(float) 0;
      floatRgbPixel.b=(float) 0;

      addCrossHair(wtaImage, floatRgbPixel, visual_width/2, visual_height/2, (int) 5);
 
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




