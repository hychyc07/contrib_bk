
#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/all.h>
#include "iCub/stereoVision/disparityThread.h"
#include "iCub/stereoVision/opticalFlowThread.h"
#include"DictionaryLearning.h"

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Stamp.h>

using namespace std; 
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

class SceneFlowThread : public RateThread
{
private:

    bool success;
    bool finish;
    bool work;
    DisparityThread* disp;
    OpticalFlowThread* opt;
    yarp::sig::Vector flowHist;
    yarp::sig::Vector code;

    std::vector<double> edges;
    int dimension;
    double minimum;
    double maximum;
    DictionaryLearning *sparse_coder;

    Mat optFlow;
    Mat dispOld;
    Mat dispNew;
    Mat mapperOld;
    Mat mapperNew;
    Mat QOld;
    Mat QNew;
    Mat RLOld;
    Mat RLNew;
    Mat dispFloatOld;
    Mat dispFloatNew;
    Mat HL_root;

    IplImage* initPosition;
    IplImage* frameDiff;

    int width;
    int height;
    int threshold;
    int value;
    bool initL,initR;
    bool init;

    void triangulate(Point2f &pixel,Point3f &point,Mat &Mapper, Mat &disparity, Mat &Q, Mat &RLrect);
    void buildRotTras(Mat & R, Mat & T, Mat & A);
    void printMatrix(Mat &matrix);
    void thresholdBW(IplImage *img, IplImage* res, int threshold, int x, int y, double value);
    void computeEdges();
    void generateMatIndex(double num, int &index);
    void computeFlowCode(yarp::sig::Vector& flowHist,yarp::sig::Vector& code);
    void computeFlowHistogram(const vector<Point3f>& flow3D, yarp::sig::Vector& flowHist);
    void drawFlowModule(IplImage* imgMotion);
public:

    SceneFlowThread(yarp::os::ResourceFinder &rf, DisparityThread* d, OpticalFlowThread* o);
    ~SceneFlowThread(void) {};

    bool isOpen();

    bool threadInit(void);
    void setThreshold(int threshold);
    void setValue(int value);
    void run(void); 
    void onStop(void);
    void close();
    yarp::sig::Vector getHist();
    Point3f getSceneFlowThreadPixel(int u, int v);
    IplImage* draw2DMotionField();
    int getImgWidth();
    int getImgHeight();
    void getSceneFlowThread(vector<Point3f> &flow3D);
    void threadRelease();      
    void setInitialPositionFrame(IplImage* initPosition);
    yarp::sig::Vector getCode();
    bool checkDone();
    void process();

};
