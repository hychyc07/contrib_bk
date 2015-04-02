#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <highgui.h>
#include <cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace cv;  

class SCSPMClassifier:public RFModule
{

    Semaphore *mutex;
    RpcClient opcPort;
    Port handlerPort;
    RpcClient rpcClassifier;
    BufferedPort<ImageOf<PixelRgb> >imgInput;
    BufferedPort<ImageOf<PixelBgr> >imgSIFTInput;

    BufferedPort<ImageOf<PixelBgr> > imgOutput;
    BufferedPort<ImageOf<PixelRgb> > imgSIFTOutput;
    bool sync;
    bool doTrain;
    bool burst;

    std::vector<Bottle> trainingFeature;
    std::vector<Bottle> negativeFeature;

    string currObject;

    Port scoresInput;

    Port featureInput;
    Port featureOutput;


public:
   
    bool configure(yarp::os::ResourceFinder &rf); 
    bool interruptModule();                       
    bool close();                                
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    bool train(Bottle *locations, Bottle &reply);
    void classify(Bottle *blobs, Bottle &reply);
    bool getOPCList(Bottle &names);
    bool updateObjDatabase();
 

};

