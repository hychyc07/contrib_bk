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
 * 18/07/07  Started work on the development of a YARP version of this module      DV
 * 30/07/09  Migrated to the RFModule class to allow use of the resource finder    DV
 * 17/08/09  Amended to comply with iCub Software Development Guidelines           DV
 * 14/10/10  Added vergence_angles output for compatibility with iKinGazeCtrl      DV
 * 11/11/10  Optimized some loops; removed output of raw CPS                       DV
 * 12/11/10  Added check on maximum absolute vergence angle: input from the /head port and added a new module parameter DV
 * 20/04/11  Fixed bug in orderly shutdown of input ports (robotPort, leftImage, rightImage) 
 *           by making acquisition loops include a termination check by calling isStopping() DV
 * 23/07/12  Fixed bug in extraction of n^2 x n^2 image from input image (now properly centred) 
 *           Also changed interface to iKinGazeCtrl to use absolute vergence values 
 *           as well as azimuth and elevation, by first reading encoder values     DV
*/ 
 
/* binocular includes */

#include "iCub/binocularVergence.h"



BinocularVergence::BinocularVergence()
/* -------------------------------------------------- */
{
    /* intialize private class variables */

    std_dev                         = 15;    // % of image width
    threshold                       = 20;    // % of maximum value
    filter_radius                   = 2;     // pixels
    number_of_maxima                = 2;     // cardinal number
    non_maxima_suppression_radius   = 5;     // pixels
    fx_right                        = 224;   // focal length, x axis, right camera
    max_vergence                    = 15;    // maximum absolute vergence angle
    debug = false;
}

BinocularVergence::~BinocularVergence(){}
/* ----------------------------------------------------- */


 
/*
 * Configure function. This handles all the module initialization
 * It takes as a parameter a previously initialized resource finder object. 
 * This object is used to configure the module
 */

bool  BinocularVergence::configure(yarp::os::ResourceFinder &rf)
/* ------------------------------------------------------------------- */
{
 
    /* Process binocularVergence module arguments */
 
    
    /* first, get the module name which will form the stem of all module port names */

    moduleName                         = rf.check("name", 
                                         Value("binocularVergence"), 
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
       cout << "binocularVergence: unable to read camera configuration file" << cameraConfigFilename;
       return 0;
    }
    else {
       fx_right = cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx", Value(225.0), "fx right").asDouble();
    }

    /* now continue getting the rest of the parameters */

    std_dev                            = rf.check("std_dev", 
                                         15, 
                                         "Standard deviation of Gaussian used for centre weighting: % of image width").asInt();

    threshold                          = rf.check("threshold", 
                                         20, 
                                         "Threshold for detection of maxima: integer % of global maximum").asInt();
        
    filter_radius                      = rf.check("filter_radius",  
                                         2, 
                                         "Radius of filter used to amplify local maxima: pixels").asInt();
        
    number_of_maxima                   = rf.check("number_of_maxima", 
                                         2, 
                                         "Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection").asInt();
     
    non_maxima_suppression_radius      = rf.check("non_maxima_suppression_radius", 
                                         5, 
                                         "Radius in pixels of the non-maxima suppression filter").asInt();

    max_vergence                       = rf.check("max_vergence", 
                                         5, 
                                         "Maximum absolute vergence angle").asDouble();

    leftCameraPortName                 = "/";
    leftCameraPortName                += getName(
                                         rf.check("left_camera", 
                                         Value("/left_camera:i"), 
                                         "left camera input (string)").asString()
                                         );

    rightCameraPortName                = "/";
    rightCameraPortName               += getName(
                                         rf.check("right_camera",
                                         Value("/right_camera:i"), 
                                         "left camera input (string)").asString()
                                         );

    leftImagePortName                  = "/";
    leftImagePortName                 += getName(
                                         rf.check("left_output", 
                                         Value("/left_image:o"), 
                                         "left image output (string)").asString()
                                         );

    rightImagePortName                 = "/";
    rightImagePortName                += getName(
                                         rf.check("right_output", 
                                         Value("/right_image:o"), 
                                         "right image output (string)").asString()
                                         );

    binocularPortName         = "/";
    binocularPortName        += getName(
                                         rf.check("cross-power_spectrum", 
                                         Value("/cross-power_spectrum:o"), 
                                         "cross-power spectrum output (string)").asString()
                                         );

    vergenceDisparityPortName          = "/";
    vergenceDisparityPortName         += getName(
                                         rf.check("vergence_disparity", 
                                         Value("/vergence_disparity:o"), 
                                         "vergence disparity output (string)").asString()
                                         );
   
    vergenceAnglesPortName        = "/";
    vergenceAnglesPortName       += getName(
                                         rf.check("vergence_angles", 
                                         Value("/vergence_angles:o"), 
                                         "vergence angles output (string)").asString()
                                         );

    robotPortName                 = "/";
    robotPortName                += getName(
                                         rf.check("headPort", 
                                         Value("/head_state:i"),
                                         "Robot head encoder state port (string)").asString()
                                         );


    if (debug) {
       printf("binocularVergence: module name is %s\n",moduleName.c_str());
       printf("binocularVergence: parameter values are\n%d\n%d\n%d\n%d\n%d\n%f\n%f\n",threshold,filter_radius,number_of_maxima,non_maxima_suppression_radius,std_dev,fx_right,max_vergence);
       printf("binocularVergence: port names are\n%s\n%s\n%s\n%s\n%s\n%s\n\n",leftCameraPortName.c_str(),
                                                                                       rightCameraPortName.c_str(),
                                                                                       leftImagePortName.c_str(),
                                                                                       rightImagePortName.c_str(),
                                                                                       binocularPortName.c_str(),
                                                                                       robotPortName.c_str(),
                                                                                       vergenceDisparityPortName.c_str(),
                                                                                       vergenceAnglesPortName.c_str() );
    }
  

    /*
     * open the ports, returning false in the event of any failure
     * returning false means that the module will not be run
     */

    if (!portIn1.open(leftCameraPortName.c_str())) return false;
    if (!portIn2.open(rightCameraPortName.c_str())) return false;
    if (!portOut1.open(leftImagePortName.c_str())) return false;
    if (!portOut2.open(rightImagePortName.c_str())) return false;
    if (!portOut3.open(binocularPortName.c_str())) return false;
    if (!portOut4.open(vergenceDisparityPortName.c_str())) return false;
    if (!portOut5.open(vergenceAnglesPortName.c_str())) return false;
    if (!robotPort.open(robotPortName.c_str())) return false;
 

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

    binocularVergenceThread = new BinocularVergenceThread(&portIn1, &portIn2, &robotPort, &portOut1, &portOut2, &portOut3, &portOut4, &portOut5, &threshold, &filter_radius, &number_of_maxima, &non_maxima_suppression_radius, &std_dev, &fx_right, &max_vergence);

    binocularVergenceThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    if (debug) cout << getName() << " running ..." << endl;
  
    return true;                               // let RFModule know whether everything opened successfuly
}


bool  BinocularVergence::close()
/* ----------------------------------- */
{
    /* stop the work thread */

    binocularVergenceThread->stop();
   
    /* close ports */

    portIn1.close();
    portIn2.close();
    portOut1.close();
    portOut2.close();
    portOut3.close();
    portOut4.close();
    portOut5.close();
    handlerPort.close();
    robotPort.close();

    return true;
}
 

bool  BinocularVergence::interruptModule()
/*---------------------------------------------- */
{
    /* interrupt ports gracefully */

    portIn1.interrupt();
    portIn2.interrupt();
    portOut1.interrupt();
    portOut2.interrupt();
    portOut3.interrupt();
    portOut4.interrupt();
    portOut5.interrupt();
    handlerPort.interrupt();
    robotPort.interrupt();

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
 * set std <n>   ... set the standard deviation 
 * set max <n>   ... set the number of maxima to detect
 * set thr <n>   ... set the threshold
 * (where <n> is an integer number)
 */

bool  BinocularVergence::respond(const Bottle& command, Bottle& reply) 
/* ------------------------------------------------------------------------- */
{
   ConstString helpMessage = getName() + " commands are: \n" +  
                             "help \n" + "quit \n" + 
                             "set std <n> ... set the standard deviation \n" +
                             "set max <n> ... set the number of maxima to detect \n" +
                             "set thr <n> ... set the threshold \n" + 
                             "(where <n> is an integer number) \n";
 
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
       if      (command.get(1).asString()=="std") {
          std_dev = command.get(2).asInt(); // set parameter value
          reply.addString("ok");
       }
       else if (command.get(1).asString()=="thr") {
          threshold = command.get(2).asInt(); // set parameter value
          reply.addString("ok");
       }
       else if (command.get(1).asString()=="max") {
          number_of_maxima = command.get(2).asInt(); // set parameter value
          reply.addString("ok");
       }
    }
 
    return true;
}
  
bool BinocularVergence::updateModule()
/* ----------------------------------------- */
{
   return true;
}

 
double  BinocularVergence::getPeriod()
/* ----------------------------------------- */
{
    return 0.05; //module periodicity (seconds)
}
 

BinocularVergenceThread::BinocularVergenceThread(BufferedPort<ImageOf<PixelRgb> > *imageIn1,  
                                                 BufferedPort<ImageOf<PixelRgb> > *imageIn2,  
                                                 BufferedPort<Vector>             *robotPortInOut,
                                                 BufferedPort<ImageOf<PixelRgb> > *imageOut1,  
                                                 BufferedPort<ImageOf<PixelRgb> > *imageOut2, 
                                                 BufferedPort<ImageOf<PixelRgb> > *imageOut3,  
                                                 BufferedPort<Vector>             *vergenceOut4, 
                                                 BufferedPort<Bottle>             *vergenceOut5, 
                                                 int *thresh,                             
                                                 int *filterRadius,                 
                                                 int *numMaxima,
                                                 int *suppressionRadius, 
                                                 int *stdDev,
                                                 double *fxRight,
                                                 double *maxVergence)
/* ------------------------------------------------------------------------------------------------------ */
{
    portIn1                        = imageIn1;
    portIn2                        = imageIn2;
    portOut1                       = imageOut1;
    portOut2                       = imageOut2;
    portOut3                       = imageOut3;
    portOut4                       = vergenceOut4;
    portOut5                       = vergenceOut5;
    robotPort                      = robotPortInOut;
    threshold                      = thresh;
    filter_radius                  = filterRadius;
    number_of_maxima               = numMaxima;
    non_maxima_suppression_radius  = suppressionRadius;
    std_dev                        = stdDev;
    fx_right                       = fxRight;
    max_vergence                   = maxVergence;
}

bool BinocularVergenceThread::threadInit() 
/* --------------------------------------------- */
{
    /* initialize variables */

    debug = false;

    if (debug) printf("binocularVergence threadInit: fx_right =  %f\n", *fx_right);

    width = 0;
    height = 0;
    depth  = 0;
    image_size  = 0;


    /* grab an image to set the image size */

    if (debug) printf("binocularVergence threadInit: getting image size\n");

    do {
       imgIn1 = portIn1->read(true);
    } while (imgIn1 == NULL);

    width  = imgIn1->width();
    height = imgIn1->height();
    depth = 3;
 
    if (debug) printf("width = %d, height = %d, depth = %d\n", width, height, depth);

    /* create the input images of the correct resolution  */

    if (width == 1024 && height == 768) {
        image_size = 256; // 512
    }
    else if (width == 640 && height == 480) {
        image_size = 256;  
    }
    else {  // width == 320 && height == 240
        image_size = 256; // need to pad
    }
 
    if (debug) printf("image_size = %d\n",image_size);

    /* set up standard deviation for apodization (centre weighting) */
   
    sigma = (float) image_size * (float) (*std_dev) / 100; 

    image_a = new DVimage(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, DVINT);
    image_b = new DVimage(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, DVINT);
    image_c = new DVimage(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
    image_d = new DVimage(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);

    return true;
}


void BinocularVergenceThread::threadRelease() 
/* ------------------------------------------------ */
{
    // delete dynamically created images

    if (image_a != NULL) delete image_a;
    if (image_b != NULL) delete image_b;
    if (image_c != NULL) delete image_c;
    if (image_d != NULL) delete image_d;
}


void BinocularVergenceThread::run()
/* -------------------------------------- */
{
   /* this is where the main work is done: continue running until isStopping() returns true */

   while (isStopping() != true) {

    /* grab images --------------------------------------------------- */

    do {
        imgIn1 = portIn1->read(true);
    } while ((imgIn1 == NULL) && (isStopping() != true));  // exit loop if shutting down
 
    do {
       imgIn2 = portIn2->read(true);
    } while ((imgIn2 == NULL) && (isStopping() != true));  // exit loop if shutting down
 
    if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 


    /*
     * now compute the cross-power spectrum
     * ------------------------------------
     *
     */

   
    /* step 1: extract an image of size 2^n x 2^n, converting from RGB to grey-scale if necessary                   */
    /* creating two input (visual) and one output (cross-power spectrum) images, all square in size, and greyscale  */
    /* ===========================================================================================================  */
 
    for (i = 0; i<image_size; i++) {
        for (j = 0; j<image_size; j++) {

            p = ((width - image_size) /  2 + i);  // no padding
            q = ((height - image_size) / 2 + j);  // ibid.

            if ((p>=0) && (p<width) && (q>=0) && (q<height)) { // defensive ... always be so
            //if ((q>=0) && (q<height)) {                          // less defensive ... width > image_size, but height may not be

                // first image
 
                rgb_pixel = imgIn1->safePixel(p,q);
                pixel_value = (unsigned char) (((int)rgb_pixel.r + (int)rgb_pixel.g + (int)rgb_pixel.b)/3); 
                image_a->put_pixel(i,j,pixel_value);

                // second image

                rgb_pixel = imgIn2->safePixel(p,q);
                pixel_value = (unsigned char) (((int)rgb_pixel.r + (int)rgb_pixel.g + (int)rgb_pixel.b)/3); 
                image_b->put_pixel(i,j,pixel_value);

            }
        }
    }
 		 
    /* step 2: apodize the image by multiplying by a Gaussian */
    /* ====================================================== */

    sigma = (float) image_size * (float) (*std_dev) / 100; 
    if (debug) printf("BinocularVergenceThread::run: std_dev = %d\n",*std_dev);

    gaussianApodization (image_a, sigma, image_a);
    gaussianApodization (image_b, sigma, image_b);  


    /* step 3: compute the cross_power_spectrum */
    /* ======================================== */
		   
    cross_power_spectrum (image_b, image_a, image_c); // image_c must exist, type FLOAT
			 	   

    /* step 4: filter the cross_power_spectrum to enhance local maxima */
    /* =============================================================== */
    
    enhance_local_maxima (image_c, *filter_radius, image_d); 	 


    /* step 5: locate the local maxima */
    /* =============================== */

    find_maxima (image_d, *number_of_maxima, *non_maxima_suppression_radius, maxima);  
 
    /* display ------------------------------------------------------------- */	  

    /* the CPS image are float so we need to contrast stretch them before displaying ... a pity really as it takes time! */

    image_d->contrast_stretch();

    ImageOf<PixelRgb> &imgOut1 = portOut1->prepare();
    ImageOf<PixelRgb> &imgOut2 = portOut2->prepare();
    ImageOf<PixelRgb> &imgOut3 = portOut3->prepare();
 
    imgOut1.resize(image_size,image_size);
    imgOut2.resize(image_size,image_size);
    imgOut3.resize(image_size,image_size);
 
    /* copy image data */
 
     for (x=0; x<image_size; x++) {
        for (y=0; y<image_size; y++) {
            image_a->get_pixel(x, y, &pixel_value);    // square left image
            rgb_pixel.r=(unsigned char) pixel_value;  
            rgb_pixel.g=(unsigned char) pixel_value;
            rgb_pixel.b=(unsigned char) pixel_value;
            imgOut1(x,y) = rgb_pixel;

            image_b->get_pixel(x, y, &pixel_value);    // square right image
            rgb_pixel.r=(unsigned char) pixel_value;  
            rgb_pixel.g=(unsigned char) pixel_value;
            rgb_pixel.b=(unsigned char) pixel_value;
            imgOut2(x,y) = rgb_pixel;

            image_d->get_pixel(x, y, &float_pixel_value); // filtered cross-power spectrum
            rgb_pixel.r=(unsigned char) float_pixel_value; 
            rgb_pixel.g=(unsigned char) float_pixel_value;
            rgb_pixel.b=(unsigned char) float_pixel_value;
            imgOut3(x,y) = rgb_pixel;
        }
    } 
    /* draw cross-hairs */

    
    rgb_pixel.r=(unsigned char) 200;
    rgb_pixel.g=(unsigned char) 200;
    rgb_pixel.b=(unsigned char) 0;

    for (i=0; i<*number_of_maxima; i++) {
        if (maxima[i].value  > (maxima[0].value * ((float)(*threshold) / 100.0))) {
           addCrossHair(imgOut3, rgb_pixel, maxima[i].x, maxima[i].y, (int) 5);
        }     
    }
    

    /* write out images */

    portOut1->write();
    portOut2->write();
    portOut3->write();

    if (debug) {
        printf("BinocularVergenceThread::run: Maxima ");
        for (i=0; i<*number_of_maxima; i++) {
           printf("(%d, %d) ", maxima[i].x, maxima[i].y);
        }
        printf("\n");
    }  


    /* servo -------------------------------------------------------------	 */  

    /* send the disparity between the two images to the controlGaze module
     * this is equivalent to the offset of the detected maximum from the centre of the CPS image
     *
     * we need a strategy to choose one of the maxima to control the fixation/vergence
     *    
     * the possibilities are: 
     *    
     * - choose the largest maximum (number 0); 
     *   this is probably going to correspond to the 
     *   object that occupies the largest amount of the field of view (or largest energy in the image)
     *    
     * - choose the maximum that is closest to the centre;
     *   the corresponds to the object that is closest to the current fixation distance
     *    
     * - choose the maximum that is furthest to the LEFT of the cross-power spectrum; 
     *   this corresponds to the object that is closest to the cameras
     *
     *  We use option 3 at the moment.
     *
     */
     
    if (maxima[1].value  > (maxima[0].value * ((float)(*threshold) / 100.0))) {
        if (maxima[0].x < maxima[1].x) {
           i = maxima[0].x;
           j = maxima[0].y;
        }
        else {
           i = maxima[1].x;
           j = maxima[1].y;
        }
    }
    else {
        i = maxima[0].x;
        j = maxima[0].y;
    }

    /*** controlGaze2 ***/

    /* normalize the disparity since controlGaze uses normalized image coordinates -1 <= x, y <= +1 */

    controlGaze_x = (float)(2*i)/(float)image_size - 1 ;
    controlGaze_y = (float)(2*j)/(float)image_size - 1 ;

    Vector& vec1 = portOut4->prepare();
  
    vec1.resize(3,0);

    vec1[0] = controlGaze_x;  
    vec1[1] = controlGaze_y;
    vec1[2] = 0;              // flag to indicate the type of image

    if (debug) printf("BinocularVergenceThread::run: disparity = [%4.2f %4.2f]\n", vec1[0], vec1[1]);

    portOut4->write();

    /*** iKinGazeCtrl ***/

    /* compute relative vergence angle from disparity */

    disparity_x   = (float) (i - (image_size/2));
    disparity_y   = (float) (j - (image_size/2));

    azimuth = 0;    
    elevation = 0;  
    vergence = -(atan2((double)disparity_x/2, (double)*fx_right) * (180.0/3.14159)); // negative disparity => positive relative vergence required

    /* now check the current absolute vergence angle */

    do {
       encoderPositions = robotPort->read(true);
    } while ((encoderPositions == NULL) && (isStopping() != true));

    if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 

    current_azimuth   = (double) encoderPositions->data()[2];   // Neck yaw   ... azimuth
    current_elevation = (double) encoderPositions->data()[0];   // Neck pitch ... elevation
    current_vergence  = (double) encoderPositions->data()[5];   // eye vergence
    
    /* limit the relative vergence */

    if (current_vergence + vergence > *max_vergence) {
       vergence = *max_vergence - current_vergence; 
    }

    if (true) {
       // printf("Relative Vergence = %4.1f; Absolute vergence = %4.1f; Fixation distance = %4.0f mm\n", vergence, current_vergence+vergence, (66/2) / tan ((3.14159/180.0) * ((current_vergence+vergence)/2)));
       printf("Relative Vergence = %4.1f; Absolute vergence = %4.1f\n", vergence, current_vergence+vergence, 33 / tan ((3.14159 * (current_vergence+vergence)) /(2 * 180)));
    }

    Bottle& bot = portOut5->prepare();
    bot.clear();
    bot.addString("abs");      // "rel" since vergence is relative to current fixation 
    bot.addDouble(current_azimuth  + azimuth);    
    bot.addDouble(current_azimuth  + elevation);
    bot.addDouble(current_vergence + vergence); 
    portOut5->write();

    if (debug) printf("BinocularVergenceThread::run: gaze angles = %4.2f %4.2f %4.2f\n", azimuth, elevation, vergence);

  }
}

/* empty line to make gcc happy */


