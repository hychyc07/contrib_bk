// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon 
 * (based on code written by Alberto Bietti, Logan Niehaus, and Gionvanni Saponaro for the autoAssociativeMemory module)
 * email:   david.vernon@robotcub.org
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
 *
 * 02/11/09  Started adapting the original autoAssociativeMemory to create the current episodicMemory module
 * 04/11/09  Completed the episodicMemory module
 * 21/11/09  Added a configuration key-value pair - offline - to allow the memory to be used 
 *           without checking that the robot gaze is stable
 * 25/11/09  Recall is now triggered by action input, typically by the attentionSelection module
 *           ImageId for the current and previous images are now output on the same port, together with the 
 *           azimuth and gaze angles at which each image was acquired
 * 13/01/10  Added retrievedImage in addition to recalledImage 
 * 04/09/11  Removed check on status of iCub head since this is now the responsibility of the overtAttention module
 *           Removed the offline key-value pair
 *           Changed protocol and port type of actionInPort to match overtAttention and iKinGazeCntrl
 *           Added a new image output which displays a mosaic of thumbnails of all images in the memory
 *           Converted from RateThread to Thread
 * 16/09/11  Added vergence angle to the output vector
 * 04/10/11  Removed previous image id, match value, and angles   
 * 22/11/11  New protocol for the database folder: the path now specifies the directory in which both the images and
 *           the database file (i.e. index file) are stored. The path must be valid (i.e. the folder must exist) 
 *           but the index file and images are created if necessary.
 * 25/01/12  New module parameter to clear the memory on start up; if not set the memory is initialized from the image database 
 * 25/01/12  New module parameter to limit the number of entries (i.e. images) in the memory
 */ 

// iCub
#include <iCub/episodicMemory.h>

//opencv
#include <cv.h>
#include <highgui.h>

//HistMatchData constructor
HistMatchData::HistMatchData()
{
    setThreshold(0.6);  //default threshold value
    databaseName = "episodicDatabase.txt";
    databaseContext = "";
}

//HistMatchData deconstructor
HistMatchData::~HistMatchData()
{
}


//HistMatchData image getter method
vector<ImageOf<PixelRgb> >& HistMatchData::images()
{
    return imgs;
}

//threshold setter
void HistMatchData::setThreshold(double t)
{
    thrMutex.wait();
    threshold = t;
    thrMutex.post();
}

//threshold getter
void HistMatchData::getThreshold(double& t)
{
    thrMutex.wait();
    t = threshold;
    thrMutex.post();
}

//database context setter
void HistMatchData::setDatabaseContext(string s)
{
    databaseContext = s;
}

//database context getter
string HistMatchData::getDatabaseContext()
{
    return databaseContext;
}

//change database name
void HistMatchData::setDatabaseName(string s)
{
    databaseName = s;
}

//get database name
string HistMatchData::getDatabaseName()
{
    return databaseName; 
}


//Loads a vector of JPG images into a bottle based on our 'database' file
void HistMatchData::loadDatabase()
{

    string file;
    string databaseFolder;

    databaseFolder = databaseContext;

    cout << "HistMatchData::loadDatabase: trying to read from " << (databaseFolder + "/" + databaseName).c_str() << endl;

    ifstream datafile((databaseFolder + "/" + databaseName).c_str());
    if (datafile.is_open()) {
        while (!datafile.eof()) {  //open the file and read in each line
            getline(datafile,file);
            if (file.size() <= 3) break;
            file = databaseFolder + "/" + file;
            IplImage *thisImg = cvLoadImage(file.c_str());  //load image
            ImageOf <PixelRgb> yarpImg;
            yarpImg.wrapIplImage(thisImg);  
            imgs.push_back(yarpImg);
        }
    }
	else {
		cout << "HistMatchData::loadDatabase: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}
}
    
 

bool EpisodicMemory::configure(yarp::os::ResourceFinder &rf)
{

    if (debug)
       printf("episodicMemory::configure\n");

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                            Value("episodicMemory"), 
                            "module name (string)").asString(); 

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

  
    // parse parameters or assign default values (append to getName=="/episodicMemory")

    imageInPortName      = "/";
    imageInPortName     += getName(
                           rf.check("imageInPort",
				           Value("/image:i"),
				           "Input image port (string)").asString()
                           );

    imageIdInPortName    = "/";
    imageIdInPortName   += getName(
                           rf.check("imageIdInPort",
				           Value("/imageId:i"),
				           "Input image id port (string)").asString()
                           );

   actionInputPortName   = "/";
   actionInputPortName  += getName(
                           rf.check("actionInPort", 
                           Value("/action:i"),
                           "saccade and action tag port (string)").asString()
                           );
  
    recalledImageOutPortName  = "/";
    recalledImageOutPortName += getName(
                           rf.check("recalledImageOutPort",
				           Value("/recalledImage:o"),
				           "Output image port (string)").asString()
                           );

    retrievedImageOutPortName  = "/";
    retrievedImageOutPortName += getName(
                           rf.check("retrievedImageOutPort",
				           Value("/retrievedImage:o"),
				           "Output image port (string)").asString()
                           );

    mosaicImageOutPortName = "/";
    mosaicImageOutPortName += getName(
                           rf.check("mosaicImageOutPort",
				           Value("/mosaicImage:o"),
				           "Output image port (string)").asString()
                           );

    imageIdOutPortName   = "/";
    imageIdOutPortName  += getName(
                           rf.check("imageIdOutPort",
				           Value("/imageId:o"),
				           "Output image id port (string)").asString()
                           );

    path                 = rf.check("path",
				           Value("~/iCub/app/episodicMemory"),
    			           "complete path to database directory").asString().c_str(); 

    databaseName         = rf.check("database",
					       Value("episodicDatabase.txt"),
					       "Database name (string)").asString().c_str();

    threshold            = rf.check("threshold",
                           Value(0.75),
                           "initial threshold value (double)").asDouble();

    memoryLimit          = rf.check("limit",
                           Value(100),
                           "maximum number of memory entries (int)").asInt();

    clearMemory         = rf.check("clear");

    printf("episodicMemory: parameters are \n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%f\n%f\n%d\n\n",
           imageInPortName.c_str(),
           imageIdInPortName.c_str(), 
           actionInputPortName.c_str(),
           recalledImageOutPortName.c_str(), 
           retrievedImageOutPortName.c_str(), 
           mosaicImageOutPortName.c_str(), 
           imageIdOutPortName.c_str(), 
           databaseName.c_str(), 
           path.c_str(), 
           threshold,
           memoryLimit);
    if (clearMemory)     printf("episodicMemory: clear flag is set\n");
    else                 printf("episodicMemory: clear flag is not set\n");


    // create episodicMemory ports

    imageIn.open(imageInPortName.c_str());
    imageIdIn.open(imageIdInPortName.c_str());
    actionIn.open(actionInputPortName.c_str());
    recalledImageOut.open(recalledImageOutPortName.c_str());
    retrievedImageOut.open(retrievedImageOutPortName.c_str());
    mosaicImageOut.open(mosaicImageOutPortName.c_str());
    imageIdOut.open(imageIdOutPortName.c_str());
    

   // attach a port to the module
   // so that messages received from the port are redirected
   // to the respond method
   
   handlerPortName =  "/";
   handlerPortName += getName();          
 
   handlerPort.open(handlerPortName.c_str());  
 
   attach(handlerPort);   
	
   /* create the thread and pass pointers to the module parameters */

   episodicMemoryThread = new EpisodicMemoryThread(&imageIn,
                                                   &imageIdIn,
                                                   &actionIn, 
                                                   &recalledImageOut,
                                                   &retrievedImageOut,
                                                   &mosaicImageOut,
                                                   &imageIdOut,
                                                   &databaseName,
                                                   &path,
                                                   &threshold,
                                                   &memoryLimit,
                                                   &clearMemory);

   /* now start the thread to do the work */

   episodicMemoryThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true;
}

bool EpisodicMemory::updateModule()
{		
    return true;
}

bool EpisodicMemory::interruptModule()
{
    // interrupt ports gracefully

    imageIn.interrupt();
    imageIdIn.interrupt();
    actionIn.interrupt();
    recalledImageOut.interrupt();
    retrievedImageOut.interrupt();
    mosaicImageOut.interrupt();
    imageIdOut.interrupt();
    handlerPort.interrupt();

    return true;	
}

bool EpisodicMemory::close()
{
    cout << "Closing EpisodicMemory...\n\n";

    // close episodicMemory ports
    
    //_portThresholdIn->close();   
    
    imageIn.close();
    imageIdIn.close();
    actionIn.close();
    recalledImageOut.close();
    retrievedImageOut.close();
    mosaicImageOut.close();
    imageIdOut.close();
    handlerPort.close();

    /* stop the thread */

    episodicMemoryThread->stop();
   
    return true;
}


//module periodicity (seconds), called implicitly by module

double EpisodicMemory::getPeriod()

{
    return 0.1; //module periodicity (seconds)
}

// Message handler. 

bool EpisodicMemory::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set thr <n>    ... set the threshold for image recall: 0-1\n" + 
                        "(where <n> is an real number) \n";

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
      if (command.get(1).asString()=="thr") {
         threshold = command.get(2).asDouble(); // set parameter value
         reply.addString("threshold set ok");
      }
   }
   return true;
}


EpisodicMemoryThread::EpisodicMemoryThread(BufferedPort<ImageOf<PixelRgb> > *imageIn,
                                           BufferedPort<VectorOf<double> >  *imageIdIn,
                                           BufferedPort<Bottle>             *actionIn,
                                           BufferedPort<ImageOf<PixelRgb> > *recalledImageOut,
                                           BufferedPort<ImageOf<PixelRgb> > *retrievedImageOut,
                                           BufferedPort<ImageOf<PixelRgb> > *mosaicImageOut,
                                           BufferedPort<VectorOf<double> >  *imageIdOut,
                                           string                           *databaseName,
                                           string                           *path,
                                           double                           *threshold,
                                           int                              *memoryLimit,
                                           bool                             *clearMemory)
{
   debug = false;

   imageInPort           = imageIn;
   imageIdInPort         = imageIdIn;
   actionInPort          = actionIn;
   recalledImageOutPort  = recalledImageOut;
   retrievedImageOutPort = retrievedImageOut;
   mosaicImageOutPort    = mosaicImageOut;
   imageIdOutPort        = imageIdOut;
   databaseNameValue     = databaseName;
   pathValue             = path;
   thresholdValue        = threshold;
   memoryLimitValue      = memoryLimit;
   clearMemoryValue      = clearMemory;

   if (debug) {
      cout << "EpisodicMemoryThread: database name  " << *databaseNameValue << endl;
      cout << "EpisodicMemoryThread: path           " << *pathValue << endl;
      cout << "EpisodicMemoryThread: threshold      " << *thresholdValue << endl;
      cout << "EpisodicMemoryThread: limit          " << *memoryLimitValue << endl;
      if (*clearMemoryValue)
         cout << "EpisodicMemoryThread: clear       true"   << endl;
      else
         cout << "EpisodicMemoryThread: clear       false"   << endl;

   }
}

bool EpisodicMemoryThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    imageId               = -1;           // initialize to -1 so that we know it is not valid values
    imageMatch            = 0;
    gazeAzimuth           = 0;
    gazeElevation         = 0;
    gazeVergence          = 0;


    data.setThreshold(*thresholdValue);
   
    data.setDatabaseContext(*pathValue);

    if (debug)
       std::cout << "EpisodicMemoryThread::threadInit: databaseContext " << (*pathValue).c_str() << endl;
    

	data.setDatabaseName(*databaseNameValue);

    if (debug)
       std::cout << "EpisodicMemoryThread::threadInit: databaseName    " << (*databaseNameValue).c_str() << endl;
    
    if (!(*clearMemoryValue)) {
       data.loadDatabase();
    }

    return true;
}

void EpisodicMemoryThread::run(){

   void insertThumbnailImage(ImageOf<PixelRgb> *originalImage, 
                             ImageOf<PixelRgb> *mosaicImage, 
                             int n, int i, bool border);
   if (false) {
      cout << "EpisodicMemoryThread: database name  " << *databaseNameValue << endl;
      cout << "EpisodicMemoryThread: path           " << *pathValue << endl;
      cout << "EpisodicMemoryThread: threshold      " << *thresholdValue << endl;
   }

   while (!isStopping()) {

   if (false)
      std::cout << "EpisodicMemoryThread: the threshold is now: " << *thresholdValue << std::endl;

   data.setThreshold(*thresholdValue);   // set the threshold ... we do this in case it has been changed at run-time


   /* First check to see if we can read an image identification number
    * If so, simply retrieve the image.
    * Otherwise recall the image that best matches the presented image,
    * or store it if none match it
    */
 
   imgInId = imageIdInPort->read(false);  // try to read a vector containing the image id
 
   if (imgInId != NULL) { 

      /* retrieve the image corresponding to the image id */

      imageId = (int)(*imgInId)(0);

      if (debug) {
         std::cout << "EpisodicMemoryThread: retrieving imageId " << imageId << std::endl;
      }
      
      ImageOf<PixelRgb>& img = *imgIn;
      std::vector<ImageOf<PixelRgb> >& images = data.images();
    
      if ( (matchId >= 0) && (matchId <= (images.end() - images.begin())) ) {       // make sure we are accessing an image that exists

	     it = images.begin() + matchId; // set the iterator

	     matchImage = *it;              // retrieve the image
         matchValue = 1.0;              // retrieving an existing image so the match is perfect

         imageId         = matchId;
         imageMatch      = matchValue;

         retrievedImageOutPort->prepare() = matchImage;
         retrievedImageOutPort->write();
   
         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(5,0);
         out[0] = imageId;
         out[1] = imageMatch;
         out[2] = gazeAzimuth;
         out[3] = gazeElevation;
         out[4] = gazeVergence;

         imageIdOutPort->write();

         if (debug) {
            printf("EpisodicMemoryThread: gaze angles = %4.1f %4.1f %4.1f; appended imageId %d (%3.2f)\n",gazeAzimuth,gazeElevation,gazeVergence,imageId,imageMatch);
         }
      }
   }
   else {
   
      /* no imageId is available so we need to read an image
       * 
       * *** BUG WARNING
       * ***
       * *** The order in which we read the image and action inputs is critical.
       * *** Reading an action followed by image very often destroys the synchronization, 
       * *** with an image being paired with an incorrect action.
       * *** Reading an image followed by an action apparently solves this but there is no guarantee that this is stable.
       * *** In the future, some method of guaranteeing synchronization must be devised.
       * *** The problem manifests itself not here but in the procedural memory, with the reported saccade leading to
       * *** a given image not necessarily being the one that did.
       * *** **************************************************************************************************************
       */

      /*  read an image and recall/store it */
    
      do {
         imgIn = imageInPort->read(true);
      } while ((imgIn == NULL) && (isStopping() != true));  // exit loop if shutting down
      
      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      
/*
      do {
         actionIn  = actionInPort->read(true);                 // read a bottle containing the action (gaze command)
      } while ((actionIn == NULL) && (isStopping() != true));  // exit loop if shutting down

      if (isStopping()) break; // abort this loop to avoid make sure we don't continue and possibly use NULL images 
*/

      actionIn  = actionInPort->read(false);                 // read a bottle containing the action (gaze command)
		 
	  if (actionIn != NULL) { 
         gazeAzimuth   = actionIn->get(1).asDouble();
         gazeElevation = actionIn->get(2).asDouble();
         gazeVergence  = actionIn->get(3).asDouble();
      }

   	
      ImageOf<PixelRgb>& img = *imgIn;
    
      data.imgMutex.wait();

      std::vector<ImageOf<PixelRgb> >& images = data.images();
      IplImage* currImg = cvCreateImage(cvSize(img.width(), img.height()), IPL_DEPTH_8U, 3);

      //get the images from the port

      cvCvtColor((IplImage*)img.getIplImage(), currImg, CV_RGB2HSV);

      int arr[2] = { 16, 16 }; // 16x16 histogram bins are used, as that is what is done in the literature
      CvHistogram* currHist = cvCreateHist(2, arr, CV_HIST_ARRAY);

      //convert from RGB to HSV and split the 3 channels
      IplImage *currImgH = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //hue
      IplImage *currImgS = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //saturation
      IplImage *currImgV = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //value (thrown away)
      cvSplit(currImg, currImgH, currImgS, currImgV, NULL);
      IplImage* imgArr[2] = { currImgH, currImgS };
      cvCalcHist(imgArr, currHist);
 
      matchValue = *thresholdValue;
      found = false;
      matchId = 0;
      // std::cout << "threshold: " << *thresholdValue << " ";  //for debugging purposes only
    
      //for each image present in the database
      for (it = images.begin(); it != images.end(); ++it)
      {
         IplImage* refImg = cvCreateImage(cvSize(it->width(), it->height()), IPL_DEPTH_8U, 3);
    
         cvCvtColor((IplImage*)it->getIplImage(), refImg, CV_RGB2HSV);
    
         CvHistogram* refHist = cvCreateHist(2, arr, CV_HIST_ARRAY);
    
         //convert the image to HSV, then split the 3 channels
         IplImage *refImgH = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
         IplImage *refImgS = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
         IplImage *refImgV = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
         cvSplit(refImg, refImgH, refImgS, refImgV, NULL);
         imgArr[0] = refImgH;
         imgArr[1] = refImgS;
         cvCalcHist(imgArr, refHist);
    
         //do a histogram intersection, and check it against the threshold 

         // The Bhattacharyya distance metric seemed to produce better results at the VVV 09 summer school
         // however, it works better with normalized histograms so I've added normalization. DV 14/10/09
           
         //cvNormalizeHist(currHist,1.0);
         //cvNormalizeHist(refHist,1.0);
         //double comp = 1 - cvCompareHist(currHist, refHist, CV_COMP_BHATTACHARYYA);  


         // Alternative is intersection DV 14/10/09
         // this method of intersection is the one proposed by Swain and Ballard
         // the intersection value should be normalized by the number of pixels
         // i.e. img.width() * img.height() which is the same as the integral of the histogram

         double comp = cvCompareHist(currHist, refHist, CV_COMP_INTERSECT)/(img.width() * img.height()); 
                                                                                   
         //printf("Histogram intersection: %3.2f \n",comp);

         if (comp > matchValue) {
            matchValue = comp;
            matchImage = *it;
            matchId = it - images.begin();
            found = true;
         }
         cvReleaseImage(&refImg); 
         cvReleaseImage(&refImgH); 
         cvReleaseImage(&refImgS); 
         cvReleaseImage(&refImgV);
         cvReleaseHist(&refHist);

      }
    
      //if the image produces a match
      if (found)
      {
         imageId            = matchId;
         imageMatch         = matchValue;

         recalledImageOutPort->prepare() = matchImage;
         recalledImageOutPort->write();
        
         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(5,0);
         out[0] = imageId;
         out[1] = imageMatch;
         out[2] = gazeAzimuth;
         out[3] = gazeElevation;
         out[4] = gazeVergence;

         imageIdOutPort->write();
     
         if (debug) {
            printf("EpisodicMemoryThread: gaze angles = %4.1f %4.1f %4.1f; recalled imageId %d (%3.2f)\n",gazeAzimuth,gazeElevation,gazeVergence,imageId,imageMatch);
         }
      }
      else  { //no match found
    
	  if (images.size() < (unsigned) *memoryLimitValue) {

         //add the image to the database in memory, then into the filesystem.
         images.push_back(img);
         recalledImageOutPort->prepare() = img;
         recalledImageOutPort->write();
    
         //create a filename that is imageXX.jpg

         imageId            = images.size()-1;
         imageMatch         = (double)1.0;

         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(5,0);
         out[0] = imageId;
         out[1] = imageMatch;
         out[2] = gazeAzimuth;
         out[3] = gazeElevation;
         out[4] = gazeVergence;
         imageIdOutPort->write();
            
         if (true || debug) {
            printf("EpisodicMemoryThread: gaze angles = %4.1f %4.1f %4.1f; appended imageId %d (%3.2f)\n",gazeAzimuth,gazeElevation,gazeVergence,imageId,imageMatch);
         }

         string s;
         ostringstream oss(s);
         oss << "image" << images.size()-1 << ".jpg";
         if (debug) {
            cout << "EpisodicMemoryThread:: image stored    ";
            cout << oss.str() << endl;
         }

         //write it out to the proper database

         string databasefolder = data.getDatabaseContext();
         cvCvtColor(img.getIplImage(), currImg, CV_RGB2BGR);  //opencv stores images as BGR

		 // cout << "episodicMemory: trying to save to " << (databasefolder + "/" + oss.str()).c_str() << endl;

         cvSaveImage((databasefolder + "/" + oss.str()).c_str(), currImg);
         ofstream of;

	     // cout << "episodicMemory: trying to save to " << (databasefolder + "/" + data->getDatabaseName()).c_str() << endl;

         of.open((databasefolder + "/" + data.getDatabaseName()).c_str(),ios::app);
         of << oss.str() << endl;
         of.close();
      }
	  }

      cvReleaseImage(&currImg); cvReleaseImage(&currImgH); cvReleaseImage(&currImgS); cvReleaseImage(&currImgV);
      cvReleaseHist(&currHist);
    
      // finally, generate the mosaic of all the images in the database (including an image added in this call) 

      mosaicImage.resize(imgIn->width(),imgIn->height());
      mosaicImage.zero();
      numberOfImages = images.end() - images.begin();

      for (it = images.begin(); it != images.end(); ++it)
      {
         imageNumber = it - images.begin();
         if (imageNumber == imageId) {
            border = true;
         }
         else {
            border = false;
         }
         insertThumbnailImage(&(*it), &mosaicImage, numberOfImages, imageNumber, border); // last argument is the id of the selected image
      }
      mosaicImageOutPort->prepare() = mosaicImage;
      mosaicImageOutPort->write();
    
      data.imgMutex.post();
   }
   }
}

void EpisodicMemoryThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */


}

/* Insert a thumbnail version of an original image into a mosaic image of n images at at a given position i */
/* 0 <= i <= n-1                                                                                             */

void insertThumbnailImage(ImageOf<PixelRgb> *originalImage, 
                          ImageOf<PixelRgb> *mosaicImage, 
                          int n, int i, bool border) {

   bool debug = false;
   int width;
   int height;
   int sf;
   int x, y;
   int thumbnailWidth;
   int thumbnailHeight;
   int mosaicImageX;
   int mosaicImageY;
   PixelRgb rgbPixel;


   if (debug) {
      printf("insertThumbnailImage: n = %d, i = %d\n",n,i);
   }

   sf = (int) sqrt((double)n) + 1;
   width = mosaicImage->width();
   height = mosaicImage->height();
   thumbnailWidth = width / sf;
   thumbnailHeight = height / sf;
   mosaicImageX = thumbnailWidth  * (i % sf);
   mosaicImageY = thumbnailHeight * (i / sf);

   if (debug) {
      printf("insertThumbnailImage: sf = %d; tWidth, tHeight = (%d, %d); x, y = (%d, %d)\n",
             sf,thumbnailWidth,thumbnailHeight,mosaicImageX,mosaicImageY);
   }

   for (x = 0; x < thumbnailWidth; x++) {
      for (y = 0; y < thumbnailHeight; y++) {
         rgbPixel = originalImage->safePixel(x*sf,y*sf);  
         (*mosaicImage)(x+mosaicImageX, y+mosaicImageY) = rgbPixel;
      }
   }

   if (border) {

      /* draw a border around the selected image */

      rgbPixel.r = 255;
      rgbPixel.g = 0;
      rgbPixel.b = 0;

      addRectangleOutline((*mosaicImage), rgbPixel, 
                           mosaicImageX+thumbnailWidth/2, mosaicImageY+thumbnailHeight/2,  
                           thumbnailWidth/2-1, thumbnailHeight/2-1);
   }
 
}


