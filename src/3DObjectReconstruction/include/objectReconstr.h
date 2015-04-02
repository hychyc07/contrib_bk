#ifndef OBJECT_RECONSTRUCTION_MODULE_H
#define OBJECT_RECONSTRUCTION_MODULE_H
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/RFModule.h>

#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/data3D/minBoundBox.h>
#include <SegmentationModuleInterface.h>
#include "communicator.h"
#include "reconstructionRoutine.h"

#define ACK                     VOCAB3('a','c','k')
#define NACK                    VOCAB4('n','a','c','k')

#define STATE_WAIT              0
#define STATE_RECONSTRUCT       1
#define STATE_VISUALIZE         2

class ObjectReconstr: public yarp::os::RFModule
{
    int current_state;
    int number;
    double minVergence;
    double tableHeight;
    double middlex;
    double middley;
    bool useTable;
    bool write;
    bool closing;
    bool visualizationOn;
    bool useSegmentation;
    bool useChris;
    bool computeBB;
    int context_without_eyes_blocked;
    int context_with_eyes_blocked;
    std::string object;
    std::string outputDir;
    yarp::sig::Matrix cornerpoints;
    yarp::sig::Matrix rotmat;
    yarp::sig::Vector dim;
    yarp::sig::Vector leftTopVertex;
    yarp::sig::Vector rightDownVertex;
    yarp::sig::Vector vectxroot;
    yarp::sig::Vector vectyroot;
    yarp::sig::Vector vectzroot;

    pcl::PointXYZ centerroot;

    iCub::data3D::BoundingBox boundingBox;

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInRight;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> pointCloudPort;
    yarp::os::Port segmentationPort;
    yarp::dev::PolyDriver* gazeCtrl;
    yarp::dev::IGazeControl* igaze;
    yarp::dev::PolyDriver polyTor;
    yarp::dev::IPositionControl *posTor;
    yarp::dev::PolyDriver polyHead;
    yarp::dev::IPositionControl *posHead;

    ReconstructionRoutine recRoutine;
    SegmentationModuleInterface segmModInterface;
    Communicator communicator;

    void addPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void setTableHeight(const double tableHeight);
    bool updateCloud();
	void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second=false);

    void moveTorso(const double degrees);
    bool reconstruction();
    std::vector<yarp::sig::Pixel> getPixelList();
    std::vector<yarp::sig::Pixel> computePixelListFromBB();
    bool initProc();
    void savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void steerEyesToHome();
    void setBoundingBox(double lx, double ly, double rx, double ry);

public:

    ObjectReconstr();
    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    bool interruptModule();
    double getPeriod();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool checkPorts();
};

#endif
