#include <string.h>
#include <sstream>
#include "visualizationThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::data3D;

VisualizationThread::VisualizationThread(DataToShow &_data) : data(_data)
{
    running=false;
}

void VisualizationThread::run() 
{
    int v1,v2;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int scale=15;
    viewer->initCameraParameters(); 
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);

    for (unsigned int i=0; i<data.goodPointsIndexes.size(); i++)
    {
        ostringstream temp;
        temp<<i;
        string s=temp.str();
        pcl::PointXYZ point(data.cloud.points.at(data.goodPointsIndexes[i]).x, data.cloud.points.at(data.goodPointsIndexes[i]).y, data.cloud.points.at(data.goodPointsIndexes[i]).z);
        viewer->addSphere (point, 0.002, 1, 1, 0, "pointScored"+s, 2);
    }

    pcl::PointXYZ origin(data.chosenPoint[0],data.chosenPoint[1],data.chosenPoint[2]);
    pcl::PointXYZ normalOrigin;
    pcl::PointXYZ normalOriginScaled;
    if (data.hand=="left")
    {
        normalOrigin.x=origin.x+data.chosenOrientation(0,2);
        normalOrigin.y=origin.y+data.chosenOrientation(1,2);
        normalOrigin.z=origin.z+data.chosenOrientation(2,2);
        normalOriginScaled.x=origin.x+(data.chosenOrientation(0,2)/scale);
        normalOriginScaled.y=origin.y+(data.chosenOrientation(1,2)/scale);
        normalOriginScaled.z=origin.z+(data.chosenOrientation(2,2)/scale);
    }
    else
    {
        normalOrigin.x=origin.x-data.chosenOrientation(0,2);
        normalOrigin.y=origin.y-data.chosenOrientation(1,2);
        normalOrigin.z=origin.z-data.chosenOrientation(2,2);
        normalOriginScaled.x=origin.x-(data.chosenOrientation(0,2)/scale);
        normalOriginScaled.y=origin.y-(data.chosenOrientation(1,2)/scale);
        normalOriginScaled.z=origin.z-(data.chosenOrientation(2,2)/scale);
    }

    viewer->addSphere (origin, 0.002, 1, 0, 0, "pointChosen", 2);
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(origin,normalOriginScaled,0,0,1,0,"pz",2);

    pcl::PointXYZ xaxis(normalOrigin.x+data.chosenOrientation(0,0),normalOrigin.y+data.chosenOrientation(1,0),normalOrigin.z+data.chosenOrientation(2,0));
    pcl::PointXYZ xaxisScaled(normalOriginScaled.x+(data.chosenOrientation(0,0)/scale),normalOriginScaled.y+(data.chosenOrientation(1,0)/scale),normalOriginScaled.z+(data.chosenOrientation(2,0)/scale));

    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(xaxisScaled,normalOriginScaled,1,0,0,0,"px",2);

    pcl::PointXYZ yaxis(normalOrigin.x+data.chosenOrientation(0,1),normalOrigin.y+data.chosenOrientation(1,1),normalOrigin.z+data.chosenOrientation(2,1));
    pcl::PointXYZ yaxisScaled(normalOriginScaled.x+(data.chosenOrientation(0,1)/scale),normalOriginScaled.y+(data.chosenOrientation(1,1)/scale),normalOriginScaled.z+(data.chosenOrientation(2,1)/scale));

    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(yaxisScaled,normalOriginScaled,0,1,0,0,"py",2);

    BoundingBox bb(data.boundingBox.getBoundingBox());
    bb.drawBoundingBox(viewer, v1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB> (data.cloud)); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusteredPtr(new pcl::PointCloud<pcl::PointXYZRGB> (data.cloud_clustered));
    pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal> (data.normals));

    viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloudPtr,normalPtr,10,0.01,"normals",v2);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> curvatures(cloud_clusteredPtr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_clusteredPtr, curvatures, "curvatures", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudPtr, rgb, "rgb", v1);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "curvatures");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgb");

    viewer->addCoordinateSystem(0.1,v2);

    viewer->resetCamera();

    running=true;

    while (!viewer->wasStopped())
    {
        if (!running)
        {
            viewer->close();
            break;
        }
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void VisualizationThread::onStop()
{
    running=false;
}
