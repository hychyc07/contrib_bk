#ifndef RECONSTRUCTION_ROUTINE_H
#define RECONSTRUCTION_ROUTINE_H
#include <cv.h>
//#include <highgui.h>

#include <yarp/os/ResourceFinder.h>

#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "iCub/stereoVision/disparityThread.h"
#include <SegmentationModuleInterface.h>

class ReconstructionRoutine
{
private:

    DisparityThread* disp;
    double tableHeight;
    int number;
    int objNumber;
    string outputDir;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudComplete;

    void triangulation(IplImage* imgL, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, std::vector<yarp::sig::Pixel>& pixelList);
    void align(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
    void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered);
    void savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

public:

    ReconstructionRoutine();
    ~ReconstructionRoutine();

    void resetClouds();
    bool reconstruct(IplImage* imgL, IplImage* imgR, std::vector<yarp::sig::Pixel>& pixelList);
    bool triangulateSinglePoint(IplImage* imgL, IplImage* imgR, yarp::sig::Vector &point2D, yarp::sig::Vector &point3D);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudComplete();
    void setTableHeight(const double tableHeight);

    void close();
    bool open(const yarp::os::Property &options);
};

#endif
