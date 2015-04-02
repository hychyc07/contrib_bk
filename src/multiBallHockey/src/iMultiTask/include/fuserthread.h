

#ifndef FUSERTHREAD_H
#define FUSERTHREAD_H



#include <cstdlib>
#include <qthread.h>
#include <iostream>
//#include <qvector.h>
#include <vector>
#include <string>
#include <fstream>
// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/Network.h>

// OpenCV
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/GazeControl.h>


#include "targetobject.h"
#include "dataassociator.h"
#include "targetcreationdeletion.h"
#include "targetobject.h"
#include "targetpositionsenderthread.h"
#include "definitions.h"

#include "timer.h"
#include "taskThread.h"
#include "headThread.h"
class FuserThread : public RateThread
{
public:
    //QObject* frameObject;

    //FuserThread(QObject* frameObjectt,  Frame* frameC);
    //FuserThread( );
	
  FuserThread(int period, vector<TaskThread*> taskV);
    /** main() function of the thread. */
    void run();
    /** Signal the thread to exit the next time it is woken up. */
    void stop();
	 bool threadInit();
	void threadRelease();

	vector<TaskThread*> taskVec;
	
	

    /** vector containing all the fused targets */
    vector<TargetObject*>* targetVector;

     
    TargetCreationDeletion * targCreationDeletion;	

	
	TargetPositionSenderThread *targPosSendThread;
	
    QMutex targetMutex;
    
	void updateTargetDisplay();
	
	/** delcare an image **/
	IplImage *displayImage;
	IplImage *floorImage;
	
	/** data associator **/
	DataAssociator * dataAssociator;

	// debug port 
	Port debugPort;

	// creates the display image for targets
	void createDisplay();

	// destroys the image for target display
	void destroyDisplay();

	// writes log data for analysis
	void writeLogData();
	ofstream logFile;
	
	ofstream imageTimeFile;

	Timer image_timer; // to write time of grabbing image
	/// this boolean is set if the data should be logged
	bool logData;


	// writes jpg images of the display
	void writeDisplayImage(int index);
	int imageIndex;
	/// this boolean is set if the data should be logged
	bool logImage;
	char tempfilename[256];
	IplImage *scaledImage;

	void drawValidationGate(TargetObject* target);

	void drawPrediction(TargetObject* target);

	/** add a new target object**/	
	void addTarget(int id, std::string name, double x, double y, double z, double vx,double vy, double vz, int red, int green, int blue );
	
	  /** remove target object with id**/
    	void removeTarget(int id);

	/// to synchronize the speed of the application
	Timer speed_timer;
	double frame_rate; // real frame_rate of the application
	double pre_frame_rate; // real frame_rate of the application
private:

	
    	/** Keep the thread running as long this flag is true. */
    	volatile bool run_flag;


	/// graphic display of a target
	void displayTarget(float x, float y, float angle, int gain, float sig_x, float sig_y, int rgb1, int rgb2, int rgb3, string name);

	
	//BufferedPort<Vector> inPortLeft;
 	//BufferedPort<Bottle> inPortLeft;
	//BufferedPort<Bottle> inPortRight;

	std::vector<TwoDdata> leftDataVec;
	std::vector<TwoDdata> rightDataVec;

	void estimate3DsensoriData();
	bool gazingTowardsObject;

	void transformOperational2Display(double tx, double ty, double tz, double &dx, double &dy, double &dz);
	void transformDisplay2Operational(double dx, double dy, double dz, double &tx, double &ty, double &tz);
	
	
	//BufferedPort<Bottle> gazeCommandPort;
	bool gazingConverged(); // returns true if the iCub has finished shifting gaze to attend to an object

	//BufferedPort<Bottle> eye2WorldOut; 
	BufferedPort<Bottle> inPortBlobs;

	// send 3D position data
	//BufferedPort<Bottle> sendPort;
	//BufferedPort<Bottle> sendPortFumble;

	void drawPredictedTrajectory(TargetObject* target);


	// to get hue either directly from blob detection or TPC
	BufferedPort<Bottle> inPortHue;

};


#endif // FUSERTHREAD_H

