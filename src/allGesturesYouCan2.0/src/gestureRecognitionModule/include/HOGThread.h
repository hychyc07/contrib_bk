#include <cv.h>
#include <highgui.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "DictionaryLearning.h"

#define MAXPOOLING      0
#define CONCATENATE     1

class HOGThread : public yarp::os::RateThread
{
private:

    cv::Mat disparity;
    DictionaryLearning *sparse_coder;
    yarp::sig::Vector code;
    int levels;
    int nbins;
    int pool;
    bool done;
    bool work;
    std::vector<yarp::sig::Vector> pyramidHOG;

    IplImage* initPosition;
    IplImage* frameDiff;

    void computeHOG(IplImage* image, yarp::sig::Vector &currHist, double Th=0., int aperture=-1, bool show=false);
    void computeHOGcode(yarp::sig::Vector &currHist, yarp::sig::Vector &currCode);
    void computeBoundaries(std::vector<yarp::sig::Vector> &boundaries);
    void extractBoundariesFromLevel(int i, std::vector<yarp::sig::Vector> &boundaries);

public:

    HOGThread(yarp::os::ResourceFinder &rf);
    ~HOGThread() {};

    void setImage(cv::Mat &disparity);
    bool checkDone();
    yarp::sig::Vector getCode();
    std::vector<yarp::sig::Vector> getHist();
    void setInitialPositionFrame(IplImage* initPosition);

    bool threadInit();     
    void threadRelease();
    void run(); 
 
};
