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

// port to send out the blob info
  Port outportIQR;
Port outportEyes2World;

// send the value and position of the biggest red blob over yarp
void sendBlobsIQR(CvBlobs blobs, const IplImage *labelImg,  const IplImage *img, int colCode){
 
  //for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
  //{
  //cout << "Blob #" << it->second->label << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
  //}

  CvBlobs::const_iterator it=blobs.begin();
  if (it!=blobs.end()){
    //cout << "first blob pos = " << it->second->centroid.x << ", " << it->second->centroid.y << endl;
      CvScalar meanColor = cvBlobMeanColor((*it).second, labelImg, img);
    // prepare a message
        Bottle bot;    
	float area = it->second->area;
	float colorness = (float)meanColor.val[0];
	
	if (colCode == 0){ // i.e. if red blob
	  colorness = 0.9;
	}
	else if (colCode == 1){ // i.e. if green blob
	  colorness = 0.6;
	}
	else if (colCode == 2){ // i.e. if blue blob
	  colorness = 0.15;
	}
	
        //area = 0.0; // default value for the moment
	bot.add(colorness); // color red
	bot.add(0.0); // area 
	
	// send only larger areas
	if ((colorness >= 0.0) && (colorness <= 1.0) && (area > 10000.0)) {
	  outportIQR.write(bot);
	  //cout << "sending: " << colorness << " " << area << endl;
	}

  }

}


void sendBlobsEyes2World(CvBlobs blobs, const IplImage *labelImg,  const IplImage *img, int colCode){
 
  CvBlobs::const_iterator it=blobs.begin();
  if (it!=blobs.end()){
    //cout << "first blob pos = " << it->second->centroid.x << ", " << it->second->centroid.y << endl;
      CvScalar meanColor = cvBlobMeanColor((*it).second, labelImg, img);
    // prepare a message
        Bottle bot;  
	float minx = it->second->minx;
	float maxx = it->second->maxx;
	float miny = it->second->miny;
	float maxy = it->second->maxy;	

	float sendx = minx + (maxx - minx)/2.0;
	float sendy = miny + (maxy - miny)/2.0;	

        bot.add("left");
        bot.add(sendx); 
	bot.add(sendy); 
       outportEyes2World.write(bot);
  }

}

void mouseClicked(int event, int x, int y, int flags , void* param  )
{

	//std::cout << "mouse clicked" << std::endl;
 	switch( event )
    	{
		case CV_EVENT_LBUTTONDOWN:
		{
			
			std::cout << "left clicked...sending simulated object" << std::endl;
			Bottle bot;    
			   bot.add("left");
			   bot.add(x); 
			   bot.add(y); 
			   outportEyes2World.write(bot);
				
		}
		break;
		case CV_EVENT_MBUTTONDOWN:
		{
			std::cout << "middle" << std::endl;
		
		}
		break;
	}
}




int main()
{

  Network yarp; // set up yarp
  BufferedPort<ImageOf<PixelRgb> > imageLeftPort; // port for reading in images
  imageLeftPort.open("/trackBlobs/left/in"); // give a name for port
  ImageOf<PixelRgb> *imageLeft = imageLeftPort.read(); // read an image

  
  outportIQR.open("/trackBlobs/left/iqr/out");
  outportEyes2World.open("/trackBlobs/left/eyesWorld/out");

  cvNamedWindow("object_tracking_left", CV_WINDOW_AUTOSIZE);
  /// Mouse click updating world model
  int mouseParam=5;

  cvSetMouseCallback("object_tracking_left", mouseClicked, &mouseParam);

  //cvNamedWindow("segmentated-red", CV_WINDOW_AUTOSIZE);
  //cvNamedWindow("segmentated-green", CV_WINDOW_AUTOSIZE);
  //cvNamedWindow("segmentated-blue", CV_WINDOW_AUTOSIZE);

  IplImage *imgOrig = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3 );
  IplImage *img = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3 );
  


  // blob variables  
  CvSize imgSize = cvGetSize(img);
  IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);
  IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
  unsigned int frameNumber = 0;
  CvTracks tracks;


  while (1) //(cvGrabFrame(capture))
  {

    usleep(10e4);
    imageLeft = imageLeftPort.read(); // read an image
    cvCvtColor((IplImage*)imageLeft->getIplImage(), imgOrig, CV_RGB2BGR);
    cvResize(imgOrig, img);

    // find blobs
    cvConvertScale(img, frame, 1, 0);
    IplImage *segmentatedRed = cvCreateImage(imgSize, 8, 1);
    IplImage *segmentatedGreen = cvCreateImage(imgSize, 8, 1);
    IplImage *segmentatedBlue = cvCreateImage(imgSize, 8, 1);
    // Detecting red pixels:
    // (This is very slow, use direct access better...)
    for (unsigned int j=0; j<imgSize.height; j++)
      for (unsigned int i=0; i<imgSize.width; i++)
      {
	CvScalar c = cvGet2D(frame, j, i);

	double b = ((double)c.val[0])/255.;
	double g = ((double)c.val[1])/255.;
	double r = ((double)c.val[2])/255.;
	unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

        unsigned char ff = 255*((g>0.2+r)&&(g>0.2+b));
        unsigned char fff = 255*((b>0.2+g)&&(b>0.2+r));    
	cvSet2D(segmentatedRed, j, i, CV_RGB(f, f, f));
	cvSet2D(segmentatedGreen, j, i, CV_RGB(ff, ff, ff));
	cvSet2D(segmentatedBlue, j, i, CV_RGB(fff, fff, fff));
      }
      cvMorphologyEx(segmentatedRed, segmentatedRed, NULL, morphKernel, CV_MOP_OPEN, 1);
      //cvShowImage("segmentated-red", segmentatedRed);

      cvMorphologyEx(segmentatedGreen, segmentatedGreen, NULL, morphKernel, CV_MOP_OPEN, 1);
      //cvShowImage("segmentated-green", segmentatedGreen);
      cvMorphologyEx(segmentatedBlue, segmentatedBlue, NULL, morphKernel, CV_MOP_OPEN, 1);
      //cvShowImage("segmentated-blue", segmentatedBlue);


    IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

    // red blobs
    CvBlobs redBlobs;
    unsigned int result = cvLabel(segmentatedRed, labelImg, redBlobs);
    cvFilterByArea(redBlobs, 200, 1000);
    cvUpdateTracks(redBlobs, tracks, 200., 5);
    cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
    sendBlobsIQR(redBlobs, labelImg, img, 0); // send the red blobs through yarp
    sendBlobsEyes2World(redBlobs, labelImg, img, 0); // send the red blobs through yarp
    
    // green blobs
    CvBlobs greenBlobs;
    unsigned int resultG = cvLabel(segmentatedGreen, labelImg, greenBlobs);
    cvFilterByArea(greenBlobs, 500, 1000000);
    cvUpdateTracks(greenBlobs, tracks, 200., 5);
    cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
    sendBlobsEyes2World(greenBlobs, labelImg, img, 1); // send the green blobs through yarp

    // blue blobs
    CvBlobs blueBlobs;
    unsigned int resultB = cvLabel(segmentatedBlue, labelImg, blueBlobs);
    cvFilterByArea(blueBlobs, 500, 1000000);
    cvUpdateTracks(blueBlobs, tracks, 200., 5);
    cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
    sendBlobsEyes2World(blueBlobs, labelImg, img, 2); // send the blue blobs through yarp

    //cvShowImage("red_object_tracking", img);
    cvShowImage("object_tracking_left", frame);
    cvReleaseBlobs(redBlobs);
    cvReleaseBlobs(greenBlobs);
    cvReleaseBlobs(blueBlobs);
    cvReleaseImage(&labelImg);
    cvReleaseImage(&segmentatedRed);
    cvReleaseImage(&segmentatedGreen);
    cvReleaseImage(&segmentatedBlue);


    if ((cvWaitKey(10)&0xff)==27){
      break;
      
    }
    frameNumber++;
    
  }

  
  outportIQR.close();
  outportEyes2World.close();
  cvReleaseStructuringElement(&morphKernel);
  cvReleaseImage(&frame);

  cvDestroyWindow("red_object_tracking");
  cvReleaseImage(&img);
  return 0;
}
