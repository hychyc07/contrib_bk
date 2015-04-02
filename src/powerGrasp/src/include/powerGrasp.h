#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/Semaphore.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/principal_curvatures.h>
#include <boost/thread/thread.hpp>
#include <iCub/learningMachine/FixedRangeScaler.h>
#include <iCub/learningMachine/LSSVMLearner.h>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <visualizationThread.h>
#include <orientationThread.h>

#ifdef _WIN32
    #include "custom/dirent.h"
#endif

#ifndef __POWERGRASP_H__
#define __POWERGRASP_H__

#define similarity      0.005
#define STATE_WAIT      0
#define STATE_ESTIMATE  1
#define STATE_GRASP     2
#define RIGHT_HAND      "right"
#define LEFT_HAND       "left"
#define NO_HAND         "no_hand"
#define MODALITY_RIGHT  0
#define MODALITY_LEFT   1
#define MODALITY_TOP    2
#define MODALITY_CENTER 3
#define MODALITY_AUTO   4

class PowerGrasp: public yarp::os::RFModule
{
private:
    int current_state;
    int store_context_in_right;
    int store_context_right;
    int store_context_in_left;
    int store_context_left;
    int minClusterPoints;
    int maxClusterPoints;
    int neighborsForKSearch;
    int neighborsForNormal;
    int numberOfBestPoints;
    int winnerIndex;
    int modality;
    int current_modality;
    int nFile;
    bool useTable;
    bool fromFile;
    bool fromFileFinished;
    bool dont;
    bool grasped;
    bool visualize;
    bool straight;
    bool train;
    bool testWithLearning;
    bool readyToGrasp;
    bool rightBlocked;
    bool leftBlocked;
    bool filterCloud;
    bool graspSpecificPoint;
    bool clicked;
    bool blockRightTmp;
    bool blockLeftTmp;
    bool testWithLearningEnabled;
    bool writeCloud;
    double tableHeight;
    double minHeight;
    double percentage;
    double handSize;
    double radiusSearch;
    double max_curvature;
    double maxy;
    double maxz;
    double dimy;
    double dimz;
    float best_curvature;
    float current_curvature;
    float smoothnessThreshold;
    float curvatureThreshold;
    std::string chosenHand;
    std::string path;
    std::string filenameTrain;
    ofstream graspFileTrain;

    yarp::dev::PolyDriver robotArmRight;
    yarp::dev::PolyDriver robotArmLeft;
    yarp::dev::IEncoders *encRight,*encLeft;

    VisualizationThread *visualizationThread;
    OrientationThread* orientationThreadRight;
    OrientationThread* orientationThreadLeft;
    DataToShow data;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithPlane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr princip_curves_colors;
    pcl::PointCloud <pcl::Normal>::Ptr normals;
    std::vector<yarp::sig::Vector> colorMap;

    yarp::sig::Vector offsetR;
    yarp::sig::Vector offsetL;
    yarp::sig::Vector chosenPoint;
    yarp::sig::Vector chosenOrientation;
    yarp::sig::Vector chosenPixel;
    
    yarp::os::Port reconstructionPort;
    yarp::os::Port areRpcPort;
    yarp::os::Port areCmdPort;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> meshPort;
    yarp::os::Port rpc;

    yarp::os::Semaphore mutex;

    std::vector<double> rankScores;
    std::vector<int> rankIndices;

    iCub::data3D::BoundingBox boundingBox;

    iCub::learningmachine::FixedRangeScaler scalerIn,scalerOut;
    iCub::learningmachine::LSSVMLearner     machine;

    double min(const double a, const double b) {return (a>b)?b:a;};
    double max(const double a, const double b) {return (a<b)?b:a;};

    void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void addPlanePoints();
    void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second=false);
    void write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::string fileName);
    std::string chooseBestPointAndOrientation(int &winnerIndex, yarp::sig::Matrix &designedOrientation);
    double scoreFunction(yarp::sig::Vector &point, yarp::sig::Vector &normal, float &curvature, float &best_curvature, int current_modality);
    void insertElement(double score, int index);
    bool normalPointingOut(pcl::Normal &normal, pcl::PointXYZ &point, yarp::sig::Vector &center);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    void fromSurfaceMesh(const iCub::data3D::SurfaceMeshWithBoundingBox& msg);
    int fillClusteredCloud(std::vector<pcl::PointIndices> clusters);
    bool fillCloudFromFile();
    void askToGrasp();
    int findIndex(yarp::sig::Vector &point);
    void manageModality(int &current_modality);

public:

    PowerGrasp();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
    double getPeriod();
};

#endif

