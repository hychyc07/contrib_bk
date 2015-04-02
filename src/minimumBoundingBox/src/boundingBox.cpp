#include <iCub/data3D/boundingBox.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::data3D;

BoundingBox::BoundingBox(const iCub::data3D::Box3D &boundingBox)
{
    this->boundingBox.corners=boundingBox.corners;
    this->boundingBox.orientation=boundingBox.orientation;
}

iCub::data3D::Box3D BoundingBox::getBoundingBox()
{
    return this->boundingBox;
}

void BoundingBox::setBoundingBox(const iCub::data3D::Box3D &boundingBox)
{
    this->boundingBox.corners=boundingBox.corners;
    this->boundingBox.orientation=boundingBox.orientation;
}

std::vector<iCub::data3D::PointXYZ> BoundingBox::getCorners()
{
    return this->boundingBox.corners;
}

void BoundingBox::setCorners(const std::vector<iCub::data3D::PointXYZ> &corners)
{
    this->boundingBox.corners=corners;
}

yarp::sig::Matrix BoundingBox::getOrientation()
{
    return this->boundingBox.orientation;
}

void BoundingBox::setOrientation(const yarp::sig::Matrix &orientation)
{
    this->boundingBox.orientation=orientation;
}

Vector BoundingBox::findIndexes(const yarp::sig::Matrix &corner_i)
{
    Vector indexes(3);
    yarp::sig::Vector point1(3); point1[0]=corner_i(0,0); point1[1]=corner_i(0,1); point1[2]=corner_i(0,2);

    int m=0;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            if (j<=i)
                continue;
            int index=-1;
            double minimum=10000;
            for (int t=1; t<corner_i.rows(); t++)
            {
                yarp::sig::Vector tmp(3); tmp[0]=corner_i(t,0); tmp[1]=corner_i(t,1); tmp[2]=corner_i(t,2);
                double value=norm(Helpers::extractSubVector(point1,i,j)-Helpers::extractSubVector(tmp,i,j));
                if (value<minimum)
                {
                    index=t;
                    minimum=value;
                }
            }
            indexes[m]=index;
            m+=1;
        }
    }
    return indexes;
}

Matrix BoundingBox::convertCorners()
{
    Matrix cornerpoints(boundingBox.corners.size(), 3);

    for (int i=0; i<boundingBox.corners.size(); i++)
    {
        cornerpoints(i,0)=boundingBox.corners[i].x;
        cornerpoints(i,1)=boundingBox.corners[i].y;
        cornerpoints(i,2)=boundingBox.corners[i].z;
    }

    Matrix corner_i=cornerpoints*boundingBox.orientation;
    return corner_i;
}

void BoundingBox::getAxis(yarp::sig::Vector &x, yarp::sig::Vector &y, yarp::sig::Vector &z)
{
    Matrix corner_i=convertCorners();
    yarp::sig::Vector indexes=findIndexes(corner_i);

    yarp::sig::Vector point1(3); point1[0]=corner_i(0,0); point1[1]=corner_i(0,1); point1[2]=corner_i(0,2);
    
    yarp::sig::Vector index0(3); index0[0]=corner_i(indexes[0],0); index0[1]=corner_i(indexes[0],1); index0[2]=corner_i(indexes[0],2);
    yarp::sig::Vector index1(3); index1[0]=corner_i(indexes[1],0); index1[1]=corner_i(indexes[1],1); index1[2]=corner_i(indexes[1],2);
    yarp::sig::Vector index2(3); index2[0]=corner_i(indexes[2],0); index2[1]=corner_i(indexes[2],1); index2[2]=corner_i(indexes[2],2);

    yarp::sig::Vector vectz=point1-index0;
    z=vectz*boundingBox.orientation.transposed();
    yarp::sig::Vector vecty=point1-index1;
    y=vecty*boundingBox.orientation.transposed();
    yarp::sig::Vector vectx=point1-index2;
    x=vectx*boundingBox.orientation.transposed();
}

Vector BoundingBox::getDim()
{
    Matrix corner_i=convertCorners();
    yarp::sig::Vector indexes=findIndexes(corner_i);

    yarp::sig::Vector point1(3); point1[0]=corner_i(0,0); point1[1]=corner_i(0,1); point1[2]=corner_i(0,2);

    Vector dim(3);
    dim[2]=max(point1[2],corner_i(indexes[0],2))-min(point1[2],corner_i(indexes[0],2));
    dim[1]=max(point1[1],corner_i(indexes[1],1))-min(point1[1],corner_i(indexes[1],1));
    dim[0]=max(point1[0],corner_i(indexes[2],0))-min(point1[0],corner_i(indexes[2],0));

    return dim;
}

Vector BoundingBox::getCenter()
{
    Matrix corner_i=convertCorners();
    yarp::sig::Vector indexes=findIndexes(corner_i);

    yarp::sig::Vector point1(3); point1[0]=corner_i(0,0); point1[1]=corner_i(0,1); point1[2]=corner_i(0,2);
    
    yarp::sig::Vector index0(3); index0[0]=corner_i(indexes[0],0); index0[1]=corner_i(indexes[0],1); index0[2]=corner_i(indexes[0],2);
    yarp::sig::Vector index1(3); index1[0]=corner_i(indexes[1],0); index1[1]=corner_i(indexes[1],1); index1[2]=corner_i(indexes[1],2);
    yarp::sig::Vector index2(3); index2[0]=corner_i(indexes[2],0); index2[1]=corner_i(indexes[2],1); index2[2]=corner_i(indexes[2],2);

    yarp::sig::Vector center(3);
    center[2]=(point1[2]+index0[2])/2;
    center[1]=(point1[1]+index1[1])/2;
    center[0]=(point1[0]+index2[0])/2;

    yarp::sig::Vector centerroot=center*boundingBox.orientation.transposed();
    return centerroot;
}

void BoundingBox::drawBoundingBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int viewport)
{
    pcl::PointXYZRGB point1;
    point1.x=boundingBox.corners[0].x;
    point1.y=boundingBox.corners[0].y;
    point1.z=boundingBox.corners[0].z;
    point1.r=255;
    point1.g=0;
    point1.b=0;
    pcl::PointXYZRGB point2;
    point2.x=boundingBox.corners[1].x;
    point2.y=boundingBox.corners[1].y;
    point2.z=boundingBox.corners[1].z;
    point2.r=255;
    point2.g=0;
    point2.b=0;
    pcl::PointXYZRGB point3;
    point3.x=boundingBox.corners[2].x;
    point3.y=boundingBox.corners[2].y;
    point3.z=boundingBox.corners[2].z;
    point3.r=255;
    point3.g=0;
    point3.b=0;
    pcl::PointXYZRGB point4;
    point4.x=boundingBox.corners[3].x;
    point4.y=boundingBox.corners[3].y;
    point4.z=boundingBox.corners[3].z;
    point4.r=255;
    point4.g=0;
    point4.b=0;
    pcl::PointXYZRGB point5;
    point5.x=boundingBox.corners[4].x;
    point5.y=boundingBox.corners[4].y;
    point5.z=boundingBox.corners[4].z;
    point5.r=255;
    point5.g=0;
    point5.b=0;
    pcl::PointXYZRGB point6;
    point6.x=boundingBox.corners[5].x;
    point6.y=boundingBox.corners[5].y;
    point6.z=boundingBox.corners[5].z;
    point6.r=255;
    point6.g=0;
    point6.b=0;
    pcl::PointXYZRGB point7;
    point7.x=boundingBox.corners[6].x;
    point7.y=boundingBox.corners[6].y;
    point7.z=boundingBox.corners[6].z;
    point7.r=255;
    point7.g=0;
    point7.b=0;
    pcl::PointXYZRGB point8;
    point8.x=boundingBox.corners[7].x;
    point8.y=boundingBox.corners[7].y;
    point8.z=boundingBox.corners[7].z;
    point8.r=255;
    point8.g=0;
    point8.b=0;

    Vector centerroot=getCenter();
    Vector vectxroot,vectyroot,vectzroot;
    getAxis(vectxroot,vectyroot,vectzroot);

    pcl::PointXYZ z1;
    z1.x=centerroot[0]+(vectzroot[0]/2);
    z1.y=centerroot[1]+(vectzroot[1]/2);
    z1.z=centerroot[2]+(vectzroot[2]/2);

    pcl::PointXYZ z2;
    z2.x=centerroot[0]+(-vectzroot[0]/2);
    z2.y=centerroot[1]+(-vectzroot[1]/2);
    z2.z=centerroot[2]+(-vectzroot[2]/2);

    pcl::PointXYZ y1;
    y1.x=centerroot[0]+(vectyroot[0]/2);
    y1.y=centerroot[1]+(vectyroot[1]/2);
    y1.z=centerroot[2]+(vectyroot[2]/2);

    pcl::PointXYZ y2;
    y2.x=centerroot[0]+(-vectyroot[0]/2);
    y2.y=centerroot[1]+(-vectyroot[1]/2);
    y2.z=centerroot[2]+(-vectyroot[2]/2);
    
    pcl::PointXYZ x1;
    x1.x=centerroot[0]+(vectxroot[0]/2);
    x1.y=centerroot[1]+(vectxroot[1]/2);
    x1.z=centerroot[2]+(vectxroot[2]/2);

    pcl::PointXYZ x2;
    x2.x=centerroot[0]+(-vectxroot[0]/2);
    x2.y=centerroot[1]+(-vectxroot[1]/2);
    x2.z=centerroot[2]+(-vectxroot[2]/2);

    pcl::PointXYZ center(centerroot[0],centerroot[1],centerroot[2]);

    if (viewport==0)
    {
        viewer->addLine<pcl::PointXYZRGB>(point1,point2,"line1");
        viewer->addLine<pcl::PointXYZRGB>(point2,point3,"line2");
        viewer->addLine<pcl::PointXYZRGB>(point3,point4,"line3");
        viewer->addLine<pcl::PointXYZRGB>(point4,point1,"line4");
        viewer->addLine<pcl::PointXYZRGB>(point5,point6,"line5");
        viewer->addLine<pcl::PointXYZRGB>(point6,point7,"line6");
        viewer->addLine<pcl::PointXYZRGB>(point7,point8,"line7");
        viewer->addLine<pcl::PointXYZRGB>(point8,point5,"line8");
        viewer->addLine<pcl::PointXYZRGB>(point1,point5,"line9");
        viewer->addLine<pcl::PointXYZRGB>(point2,point6,"line10");
        viewer->addLine<pcl::PointXYZRGB>(point3,point7,"line11");
        viewer->addLine<pcl::PointXYZRGB>(point4,point8,"line12");

        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(z1,center,1,1,1,1,"z1");
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(z2,center,1,1,1,1,"z2");
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(y1,center,1,1,1,1,"y1");
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(y2,center,1,1,1,1,"y2");
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(x1,center,1,1,1,1,"x1");
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(x2,center,1,1,1,1,"x2");
    }
    else
    {
        viewer->addLine<pcl::PointXYZRGB>(point1,point2,"line1",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point2,point3,"line2",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point3,point4,"line3",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point4,point1,"line4",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point5,point6,"line5",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point6,point7,"line6",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point7,point8,"line7",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point8,point5,"line8",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point1,point5,"line9",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point2,point6,"line10",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point3,point7,"line11",viewport);
        viewer->addLine<pcl::PointXYZRGB>(point4,point8,"line12",viewport);
        
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(z1,center,0,0,1,1,"z1",viewport);
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(z2,center,0,0,1,1,"z2",viewport);
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(y1,center,0,1,0,1,"y1",viewport);
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(y2,center,0,1,0,1,"y2",viewport);
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(x1,center,1,0,0,1,"x1",viewport);
        viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(x2,center,1,0,0,1,"x2",viewport);
    }
}


