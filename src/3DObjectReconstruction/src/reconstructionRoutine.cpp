#include "reconstructionRoutine.h"
#include <stdio.h>
#include <sstream>
using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

ReconstructionRoutine::ReconstructionRoutine() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloudComplete(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    resetClouds();
    number=0;
    objNumber=-1;
}

ReconstructionRoutine::~ReconstructionRoutine()
{
}

void ReconstructionRoutine::savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    stringstream s;
    s.str("");
    s<<outputDir + "/view" << objNumber << number;
    string filename=s.str();
    string filenameNumb=filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++)
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

    plyfile.close();

    fprintf(stdout, "Writing finished\n");
}

bool ReconstructionRoutine::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    string configFileDisparity=opt.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str();
    string cameraContext=opt.check("CameraContext",Value("cameraCalibration/conf")).asString().c_str();
    outputDir=opt.check("outputDir",Value("C:\\Lib\\iCub\\app\\3DObjectReconstruction\\conf")).asString().c_str();

    ResourceFinder cameraFinder;
    cameraFinder.setDefaultContext(cameraContext.c_str());
    cameraFinder.setDefaultConfigFile(configFileDisparity.c_str());
    int argc=0; char *argv[1];
    cameraFinder.configure("ICUB_ROOT",argc,argv);

    disp=new DisparityThread(cameraFinder, false, false, true);
    disp->start();

    //disp->setDispParameters(false,10,50,16,64,7,0,10,0);

    return true;
} 

void ReconstructionRoutine::close()
{
    disp->stop();
    delete disp;
}

void ReconstructionRoutine::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxfilter;
    voxfilter.setInputCloud (cloud_in);
    voxfilter.setLeafSize (0.01f, 0.01f, 0.01f);
    voxfilter.filter (*cloud_downsampled);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_downsampled);
    sor.setMeanK (cloud_downsampled->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReconstructionRoutine::getPointCloud()
{
    return this->cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReconstructionRoutine::getPointCloudComplete()
{
    return this->cloudComplete;
}

void ReconstructionRoutine::setTableHeight(const double tableHeight)
{
    this->tableHeight=tableHeight;
}

void ReconstructionRoutine::resetClouds()
{
    cloud->clear();
    cloudComplete->clear();
}

bool ReconstructionRoutine::triangulateSinglePoint(IplImage* imgL, IplImage* imgR, yarp::sig::Vector &point2D, yarp::sig::Vector &point3D)
{
    point3D.resize(3,0.0);
    Mat leftIm(imgL);
    Mat rightIm(imgR);
    disp->setImages(leftIm,rightIm);
    while(!disp->checkDone())
        yarp::os::Time::delay(0.1);

    Point2f pixel(point2D[0],point2D[1]);

    Point3f point;
    disp->triangulate(pixel,point);
    if (point.z==0/*||point3D.z<tableHeight*/)
        return false;

    point3D[0]=point.x;
    point3D[1]=point.y;
    point3D[2]=point.z;

    return true;
}

bool ReconstructionRoutine::reconstruct(IplImage* imgL, IplImage* imgR, std::vector<yarp::sig::Pixel>& pixelList)
{
    Mat leftIm(imgL);
    Mat rightIm(imgR);
    disp->setImages(leftIm,rightIm);
    while(!disp->checkDone())
        yarp::os::Time::delay(0.1);

    /*Mat disparity;
    disp->getDisparity(disparity);
    IplImage disparityImg=disparity;
    cvShowImage("disp",&disparityImg);
    cvWaitKey(0);*/

    if (cloudComplete->size()>0)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);
        triangulation(imgL,input,pixelList);
        align(input);
        number++;
    }
    else
    {
        triangulation(imgL,cloudComplete,pixelList);
        number=0;
        objNumber++;
        filter(cloudComplete,cloud);
        number++;
    }

    return true;
}

void ReconstructionRoutine::triangulation(IplImage* imgL, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, std::vector<yarp::sig::Pixel>& pixelList)
{
	int n=0;
    for (int i=0; i<pixelList.size(); i++)
    {
        Point2f point2D;
        point2D.x=pixelList.at(i).x;
        point2D.y=pixelList.at(i).y;

        //cvSet2D(mask,point2D.x,point2D.y,cvScalar(1,0,0,0));

        Point3f point3D;
        disp->triangulate(point2D,point3D);
        if (point3D.z==0/*||point3D.z<tableHeight*/)
            continue;

        CvScalar color=cvGet2D(imgL,point2D.y,point2D.x);

        pcl::PointXYZRGB newPoint;
        newPoint.x=point3D.x;
        newPoint.y=point3D.y;
        newPoint.z=point3D.z;

        newPoint.r=color.val[0];
        newPoint.g=color.val[1];
        newPoint.b=color.val[2];

        uint8_t r=color.val[0];
        uint8_t g=color.val[1];
        uint8_t b=color.val[2];
 
        newPoint.rgba = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        input->push_back(newPoint);
    }
}

void ReconstructionRoutine::align(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    filter(input,filtered);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.setInputCloud(filtered);
    icp.setInputTarget(cloud);
    icp.setMaximumIterations(100);
    icp.align(Final);

    for (unsigned int i=0; i<Final.width; i++)
        cloud->push_back(Final.at(i));

    Eigen::Matrix4f rotmatE=icp.getFinalTransformation();
    Matrix rotmat(4,4);

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            rotmat(i,j)=rotmatE(i,j);

    for (int i=0; i<input->size(); i++)
    {
        yarp::sig::Vector vect(4);
        vect[0]=input->at(i).x;
        vect[1]=input->at(i).y;
        vect[2]=input->at(i).z;
        vect[3]=1.0;

        yarp::sig::Vector vectNew=rotmat*vect;
        pcl::PointXYZRGB point;
        point.x=vectNew[0];
        point.y=vectNew[1];
        point.z=vectNew[2];
        point.r=input->at(i).r;
        point.g=input->at(i).g;
        point.b=input->at(i).b;

        cloudComplete->push_back(point);
    }
}

