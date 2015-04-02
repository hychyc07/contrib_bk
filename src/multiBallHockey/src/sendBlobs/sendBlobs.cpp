// Cristóbal Carnero Liñán <grendel.ccl@gmail.com>

#include <iostream>
//#include <iomanip>

#ifdef WIN32
#include <cv.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include "cvblob.h"


// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


using namespace cvb;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace std;


int minHueC1 = 0;
int maxHueC1 = 97;
int minSatC1 = 108;
int maxSatC1 = 245;
int morphValC1 = 1;
int pixelThC1 = 52;
int minHueC2 = 116;
int maxHueC2 = 155;
int minSatC2 = 187;
int maxSatC2 = 246;
int morphValC2 = 1;
int pixelThC2 = 10;


//initial Values
int maxValHue=179;
int iniMinHue=0;
int iniMaxHue=maxValHue;

int maxValSat=255;
int iniMinSat=0;
int iniMaxSat=maxValSat;

int maxValOpen=10;
int iniMinOpen=0;
int iniMaxOpen=0;

int maxValTh=255;
int iniMinTh=0;
int iniMaxTh=0;



// port to send out the blob info
Port outportBlobs;


void sendDumbBlobs(int x1, int y1, int x2, int y2){
  Bottle bot;  
  bool done = false;
  //if (x1 > -1){
     bot.addInt(x1);
     bot.addInt(y1);
     bot.addInt(0);
     
     //}
     //if (x2 > -1){
    bot.addInt(x2);
    bot.addInt(y2);
    bot.addInt(1);
    
    //}
    //if (done){
    outportBlobs.write(bot);
    //}
}



void getNormedGray(IplImage *gray, IplImage *ch){
 CvSize imgSize = cvGetSize(gray);
  for (unsigned int j=0; j<imgSize.height; j++){
     for (unsigned int i=0; i<imgSize.width; i++){
	CvScalar c = cvGet2D(gray, j, i);
	float curvalgray =  (float)(c.val[0]);
	c =  cvGet2D(ch, j, i);
	float chVal =  (float)(c.val[0])/255;

	float res = chVal * curvalgray;
	CvScalar resC;
	resC.val[0] = res;
	cvSet2D(gray, j, i, resC);
	
     }
  }
  
}

bool findDumbBlobs( IplImage *img, int *x, int *y){

  int retx = 0;
  int rety = 0;
  int counter = 0;
  bool done = false;
   CvSize imgSize = cvGetSize(img);

   for (unsigned int j=0; j<imgSize.height; j++){
     for (unsigned int i=0; i<imgSize.width; i++){
	CvScalar c = cvGet2D(img, j, i);
	float curval =  (float)(c.val[0]);
	//cout << curval << " ";
	if (curval == 255){
	  //cout << curval << " ";
	  retx = retx + j;
	  rety = rety + i;
	  counter++;
	  done = true;
	}
    }
     // cout << endl;
  }
  
  if (done){
    *y = (retx / counter);
    *x = (rety / counter);
    //cout << "blob pos = " << *x << ", " << *y << endl;
  }
  return done;
}


void sendBlobs(CvBlobs blobs){
 
  CvBlobs::const_iterator it=blobs.begin();
  if (it!=blobs.end()){
    cout << "first blob pos = " << it->second->centroid.x << ", " << it->second->centroid.y << endl;
      //CvScalar meanColor = cvBlobMeanColor((*it).second, labelImg, img);
    // prepare a message
        Bottle bot;  
	float minx = it->second->minx;
	float maxx = it->second->maxx;
	float miny = it->second->miny;
	float maxy = it->second->maxy;	

	float sendx = minx + (maxx - minx)/2.0;
	float sendy = miny + (maxy - miny)/2.0;	

	// bot.add("left");
        bot.add(sendx); 
	bot.add(sendy); 
        outportBlobs.write(bot);
   }

}

//sliders callback
void trackbarHandlerMinHueC1(int pos) { minHueC1 = pos; }
void trackbarHandlerMaxHueC1(int pos) { maxHueC1 = pos; }
void trackbarHandlerMinSatC1(int pos) { minSatC1 = pos; }
void trackbarHandlerMaxSatC1(int pos) { maxSatC1 = pos; }
// image processing
void trackbarHandlerOpenC1(int pos) { morphValC1 = pos; }
void trackbarHandlerThC1(int pos) { pixelThC1 = pos; }


//channel two
void trackbarHandlerMinHueC2(int pos) { minHueC2 = pos; }
void trackbarHandlerMaxHueC2(int pos) { maxHueC2 = pos; }
void trackbarHandlerMinSatC2(int pos) { minSatC2 = pos; }
void trackbarHandlerMaxSatC2(int pos) { maxSatC2 = pos; }
// image processing
void trackbarHandlerOpenC2(int pos) { morphValC2 = pos; }
void trackbarHandlerThC2(int pos) { pixelThC2 = pos; }


int main()
{

  Network yarp; // set up yarp
  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  imageLeftPort.open("/trackBlobs/left/in"); // give a name for port
  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image
  
  outportBlobs.open("/trackBlobs/out");

  cvNamedWindow("object_tracking_left", CV_WINDOW_AUTOSIZE);
  /// Mouse click updating world model

  cvNamedWindow("ChannelOne", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("Min Hue","ChannelOne", &iniMinHue, maxValHue, trackbarHandlerMinHueC1);
  cvCreateTrackbar("Max Hue","ChannelOne", &iniMaxHue, maxValHue, trackbarHandlerMaxHueC1); 
  cvCreateTrackbar("Min Sat","ChannelOne", &iniMinSat, maxValSat, trackbarHandlerMinSatC1);
  cvCreateTrackbar("Max Sat","ChannelOne", &iniMaxSat, maxValSat, trackbarHandlerMaxSatC1);
  cvCreateTrackbar("Open Val","ChannelOne", &iniMaxOpen, maxValOpen, trackbarHandlerOpenC1);
  cvCreateTrackbar("Gray Pix Thr","ChannelOne", &iniMaxTh, maxValTh, trackbarHandlerThC1);


  cvNamedWindow("ChannelTwo", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("Min Hue","ChannelTwo", &iniMinHue, maxValHue, trackbarHandlerMinHueC2);
  cvCreateTrackbar("Max Hue","ChannelTwo", &iniMaxHue, maxValHue, trackbarHandlerMaxHueC2);
  cvCreateTrackbar("Min Sat","ChannelTwo", &iniMinSat, maxValSat, trackbarHandlerMinSatC2);
  cvCreateTrackbar("Max Sat","ChannelTwo", &iniMaxSat, maxValSat, trackbarHandlerMaxSatC2);
  cvCreateTrackbar("Open Val","ChannelTwo", &iniMaxOpen, maxValOpen, trackbarHandlerOpenC2);
  cvCreateTrackbar("Gray Pix Thr","ChannelTwo", &iniMaxTh, maxValTh, trackbarHandlerThC2);

  //creating the images
  IplImage *imgOrig = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3 );
  IplImage *img = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3 );
  IplImage *imgReal = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3 );
  IplImage *imgGrayC1 = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1 );
  IplImage *imgGrayC2 = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1 );



  // blob variables  
  CvSize imgSize = cvGetSize(img);
  //IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);
  IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
  //CvTracks tracks;



  //hue separation
    IplImage *chOne = cvCreateImage(imgSize, 8, 1);
    IplImage *chTwo = cvCreateImage(imgSize, 8, 1);

    //IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

  while (1) //(cvGrabFrame(capture))
  {

    usleep(10e2);
   
    imageLeft = imageLeftPort.read(); // read an image
    cvCvtColor((IplImage*)imageLeft->getIplImage(), imgReal, CV_BGR2RGB);
    cvCvtColor((IplImage*)imageLeft->getIplImage(), imgOrig, CV_BGR2HSV);
    cvCvtColor((IplImage*)imageLeft->getIplImage(), imgGrayC1, CV_BGR2GRAY);
    cvCvtColor((IplImage*)imageLeft->getIplImage(), imgGrayC2, CV_BGR2GRAY);


    cvResize(imgOrig, img);

    //filter by hue
    cvInRangeS(img, cvScalar(minHueC1, minSatC1, 50), cvScalar(maxHueC1, maxSatC1, 200), chOne);
    cvInRangeS(img, cvScalar(minHueC2, minSatC2, 50), cvScalar(maxHueC2, maxSatC2, 200), chTwo);


    //cvConvertScale(img, frame, 1, 0);
    cvMorphologyEx(chOne, chOne, NULL, morphKernel, CV_MOP_OPEN, morphValC1);
    cvMorphologyEx(chTwo, chTwo, NULL, morphKernel, CV_MOP_OPEN, morphValC2);
    
    
    getNormedGray(imgGrayC1, chOne);
    getNormedGray(imgGrayC2, chTwo);
    cvThreshold(imgGrayC1,imgGrayC1, pixelThC1, 255, CV_THRESH_BINARY );
    cvThreshold(imgGrayC2,imgGrayC2, pixelThC2, 255, CV_THRESH_BINARY );


    int x1 = -1;
    int y1 = -1;
    int x2 = -1; 
    int y2 = -1;
    bool foundBlobs1 = findDumbBlobs(imgGrayC1, &x1, &y1);
    bool foundBlobs2 = findDumbBlobs(imgGrayC2, &x2, &y2);


    cvShowImage("ChannelOne", imgGrayC1);
    cvShowImage("ChannelTwo",imgGrayC2);


    /*   

    // Channel One
    CvBlobs blobsChOne;
    CvBlobs blobsChTwo;
    //cvLabel(chOne, labelImg, blobsChOne);
    //cvLabel(chOne, labelImg, blobsChTwo);

    cvFilterByArea(blobsChOne, 500, 10000);
    cvFilterByArea(blobsChTwo, 500, 10000);

    cvUpdateTracks(blobsChOne, tracks, 200., 5);
    //cvUpdateTracks(blobsChTwo, tracks, 200., 5);
    cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
    
    */

    sendDumbBlobs(x1,y1,x2,y2);

    if (foundBlobs1 ){
	CvSize size = cvSize( 20, 20 );
        CvPoint pt = cvPoint( x1, y1);
	cvEllipse( imgReal, pt, size, 0,0, 360, CV_RGB(0,0,255), 4,8,0 );
    }
    if (foundBlobs2 ){
      CvSize size = cvSize( 20, 20 );
      CvPoint pt = cvPoint( x2, y2);
      cvEllipse( imgReal, pt, size, 0,0, 360, CV_RGB(255,0,0), 4,8,0 );
    }
    cvShowImage("object_tracking_left", imgReal);
    //cvReleaseBlobs(blobsChOne);
    //cvReleaseBlobs(blobsChTwo);
   
    
     if ((cvWaitKey(10)&0xff)==27){
      break;
      
      

    }
    
  }


  cvReleaseImage(&chOne);
  cvReleaseImage(&chTwo); 

  //cvReleaseImage(&labelImg);
  
  outportBlobs.close();
  cvReleaseStructuringElement(&morphKernel);
  //cvReleaseImage(&frame);

  cvDestroyWindow("red_object_tracking");
  cvReleaseImage(&img);
  return 0;
}
