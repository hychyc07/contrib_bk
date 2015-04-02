#include <iomanip>			// io manipulator (setw, setfill)
#include <sstream>			// string stream
#include <fstream>			// ifstream, ofstream, fstream

#include <yarp/os/Time.h>
//#include <yarp/sig/all.h>

#include "iCub/MyThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

MyThread::MyThread(BufferedPort<ImageOf<PixelRgb> > *image1Port, BufferedPort<ImageOf<PixelRgb> > *image2Port, 
	   BufferedPort<ImageOf<PixelRgb>> *imageSumPort, double *img1weight, double *delay)
{
   this->image1Port		= image1Port;
   this->image2Port		= image2Port;
   this->imageSumPort	= imageSumPort;
   this->img1weight		= img1weight;
   this->delay			= delay;
}

bool MyThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */
   return true;
}

void MyThread::run(){
	ImageOf<PixelRgb> *img1 = image1Port->read();
	ImageOf<PixelRgb> *img2 = image2Port->read();
	IplImage* iplImg1, *iplImg2;
	while (!isStopping()) { // the thread continues to run until isStopping() returns true
		iplImg1 = (IplImage*) img1->getIplImage();
		iplImg2 = (IplImage*) img2->getIplImage();

		IplImage* resizedImg2 = cvCreateImage(cvGetSize(iplImg1), IPL_DEPTH_8U, 3);
		cvResize(iplImg2, resizedImg2);

		// add the 2 images
		ImageOf<PixelRgb> &sumImg = imageSumPort->prepare();
		sumImg.resize(img1->width(), img1->height());
		cvAddWeighted(iplImg1, *img1weight, resizedImg2, 1-(*img1weight), 0, (IplImage*)sumImg.getIplImage());
		imageSumPort->write();
		cvReleaseImage(&resizedImg2);		// if you didn't release the image, the used memory would increase every cycle
											// leading to the program's crash

		Time::delay(*delay);

		ImageOf<PixelRgb> *img1New = image1Port->read(false);
		ImageOf<PixelRgb> *img2New = image2Port->read(false);
		if(img1New)
			img1 = img1New;
		if(img2New)
			img2 = img2New;		
	}
}


void MyThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
}


void MyThread::log(string s, bool endLine){
	cout << "[IMAGES SUMMER THREAD " << this->getKey() << "]: " << s;
	if(endLine)
		cout << endl;
}