#ifndef __MIN_BOUND_BOX_H_
#define __MIN_BOUND_BOX_H_

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
#include <iCub/data3D/boundingBox.h>
#include <iCub/data3D/private/helpers.h>

namespace iCub
{
namespace data3D
{
namespace MinimumBoundingBox
{
    iCub::data3D::BoundingBox getMinimumBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}
}
}
#endif


