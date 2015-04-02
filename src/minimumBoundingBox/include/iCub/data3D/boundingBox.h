#ifndef __BOUNDING_BOX_H_
#define __BOUNDING_BOX_H_

#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iCub/data3D/Box3D.h>

#include <iCub/data3D/private/helpers.h>

namespace iCub
{
namespace data3D
{
class BoundingBox
{
    iCub::data3D::Box3D boundingBox;
    yarp::sig::Vector findIndexes(const yarp::sig::Matrix &corner_i);
    yarp::sig::Matrix convertCorners();

    public:

    BoundingBox() {};
    BoundingBox(const iCub::data3D::Box3D &boundingBox);
    iCub::data3D::Box3D getBoundingBox();
    void setBoundingBox(const iCub::data3D::Box3D &boundingBox);
    std::vector<iCub::data3D::PointXYZ> getCorners();
    void setCorners(const std::vector<iCub::data3D::PointXYZ> &corners);
    yarp::sig::Matrix getOrientation();
    void setOrientation(const yarp::sig::Matrix &orientation);
    yarp::sig::Vector getDim();
    void getAxis(yarp::sig::Vector &x, yarp::sig::Vector &y, yarp::sig::Vector &z);
    yarp::sig::Vector getCenter();
    void drawBoundingBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int viewport=0);
};
}
}
#endif

