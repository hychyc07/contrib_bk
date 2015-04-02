#include "objectReconstr.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace yarp::os;
using namespace iCub::data3D;

ObjectReconstr::ObjectReconstr()
{
    current_state=STATE_WAIT;
    gazeCtrl=NULL;
    write=false;
    visualizationOn=false;
    useSegmentation=true;
    closing=false;
    number=0;
    leftTopVertex.resize(2,0.0);
    rightDownVertex.resize(2,0.0);
    dim.resize(3,0.0);
}

bool ObjectReconstr::configure(ResourceFinder &rf)
{
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string name=rf.check("name",Value("objectReconstr")).asString().c_str();
    setName(name.c_str());
    string imL=rf.check("imL",Value("/leftPort")).asString().c_str();
    string imR=rf.check("imR",Value("/rightPort")).asString().c_str();
    tableHeight=rf.check("tableHeight",Value(-0.11)).asDouble();
    useChris=false;//rf.check("useChris");
    outputDir=rf.check("outputDir",Value("C:\\Lib\\iCub\\app\\3DObjectReconstruction\\conf")).asString().c_str();
    computeBB=true;

    middlex=-1;
    middley=-1;

    Property commOptions;
    commOptions.put("milPortName",rf.check("MilPortName",Value("/objectReconstr/mil")).asString().c_str());
    commOptions.put("opcPortName",rf.check("OpcPortName",Value("/objectReconstr/opc")).asString().c_str());

    if (useChris)
    {
        if (!communicator.open(commOptions))
        {
            fprintf(stdout, "Chris ports seem to be closed, run Chris first\n");
            return false;
        }
    }

    Property optGaze;
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/client/gaze");

    gazeCtrl=new PolyDriver(optGaze);
    if (gazeCtrl->isValid()) 
        gazeCtrl->view(igaze);
    else 
    {
        fprintf(stdout, "Gaze controller failed to open\n");
        delete gazeCtrl;
        return false;
    }
    string slash="/";

    Property optTorso;
    optTorso.put("device","remote_controlboard");   
    optTorso.put("remote",(slash+robot+"/torso").c_str());
    optTorso.put("local",(slash+name+"/torso/position").c_str());
    if (polyTor.open(optTorso))
    {
        polyTor.view(posTor);
    }
    else
    {
        delete gazeCtrl;
        fprintf(stdout, "IPositionControl failed to open\n");
        return false;
    }

    Property optHead;
    optHead.put("device","remote_controlboard");
    optHead.put("remote",(slash+robot+"/head").c_str());
    optHead.put("local",(slash+name+"/head/position").c_str());
    if (polyHead.open(optHead))
        polyHead.view(posHead);
    else
    {
        delete gazeCtrl;
        if (polyTor.isValid())
            polyTor.close();
        fprintf(stdout, "IPositionControl failed to open\n");
        return false;
    }

    imagePortInLeft.open(imL.c_str());
    imagePortInRight.open(imR.c_str());

    Network::connect((slash+robot+"/camcalib/left/out").c_str(),imL.c_str());
    Network::connect((slash+robot+"/camcalib/right/out").c_str(),imR.c_str());

    string pcOut=slash + getName().c_str() + "/mesh:o";
    pointCloudPort.open(pcOut.c_str());

    string segmIn=slash+getName().c_str()+"/segmentation:i";
    segmentationPort.open(segmIn.c_str());
    segmModInterface.yarp().attachAsClient(segmentationPort);

    Network::connect(segmIn.c_str(),"/GBSeg/conf");

    leftTopVertex[0]=0;
    leftTopVertex[1]=0;
    rightDownVertex[0]=320-1;
    rightDownVertex[1]=240-1;

    Bottle p;
    igaze->getInfo(p);
    minVergence=ceil(p.check(("min_allowed_vergence"),Value(0)).asDouble());

    initProc();

    useTable=rf.check("useTable");

    string rpcName=slash + getName().c_str() + "/rpc";
    rpc.open(rpcName.c_str());
    attach(rpc);

    //igaze->storeContext(&context_without_eyes_blocked);
    //igaze->blockEyes(5.0);
    //igaze->storeContext(&context_with_eyes_blocked);
    //igaze->restoreContext(context_without_eyes_blocked);

    Property recRoutOptions;
    recRoutOptions.put("ConfigDisparity",rf.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str());
    recRoutOptions.put("CameraContext",rf.check("CameraContext",Value("cameraCalibration/conf")).asString().c_str());
    recRoutOptions.put("outputDir",outputDir.c_str());

    if (!recRoutine.open(recRoutOptions))
    {
        fprintf(stdout, "Problem with thread, the module will be closed\n");
        close();
        return false;
    }

    return true;
}

bool ObjectReconstr::initProc()
{
    if (useChris)
    {
        double height;
        if (communicator.retrieveTableHeight(height))
            setTableHeight(height);

        if (!communicator.disableAttention())
            return false;    
    }

	steerEyesToHome();

    ImageOf< PixelRgb >* toCheckdim = imagePortInLeft.read(false);
    if (toCheckdim!=NULL)
    {
        rightDownVertex[0]=toCheckdim->width()-1;
        rightDownVertex[1]=toCheckdim->height()-1;
    }
    return true;
}

void ObjectReconstr::setTableHeight(const double tableHeight)
{
    this->tableHeight=tableHeight;
    recRoutine.setTableHeight(tableHeight);
}

bool ObjectReconstr::close()
{
    moveTorso(0.0);

    if (useChris)
        communicator.close();

    delete gazeCtrl;
    if (polyTor.isValid())
        posTor->stop();

    if (polyTor.isValid())
        polyTor.close();

    if (polyHead.isValid())
        polyHead.close();

    imagePortInLeft.close();
    imagePortInRight.close();

    rpc.close();
    pointCloudPort.close();
    segmentationPort.close();

    recRoutine.close();

    return true;
}

void ObjectReconstr::moveTorso(const double degrees)
{
    yarp::sig::Vector possT(3,0.0); 
    possT[1]=degrees;
    yarp::sig::Vector accsT(3,1e9);
    yarp::sig::Vector spdsT(3,10.0);

    posTor->setRefAccelerations(accsT.data());
    posTor->setRefSpeeds(spdsT.data());
    posTor->positionMove(possT.data());

    bool ok=false;

    while(!ok)
        posTor->checkMotionDone(&ok);
}

void ObjectReconstr::steerEyesToHome()
{
    posHead->setRefSpeed(5,30.0);
    posHead->positionMove(5,minVergence);

    bool ok=false;

    while (!ok) 
    {
        yarp::os::Time::delay(0.5);
        posHead->checkMotionDone(5,&ok);
    }
    
    fprintf(stdout, "Eyes home\n");
}

std::vector<yarp::sig::Pixel> ObjectReconstr::getPixelList()
{
    if (useChris)
    {
        communicator.retrieveBoundingBox(object,leftTopVertex,rightDownVertex);
        middlex=(leftTopVertex[0]+rightDownVertex[0])/2;
        middley=(leftTopVertex[1]+rightDownVertex[1])/2;
    }

    if (useSegmentation)
        return segmModInterface.get_component_around(yarp::sig::Pixel(middlex,middley));
    else
        return computePixelListFromBB();
}

std::vector<yarp::sig::Pixel> ObjectReconstr::computePixelListFromBB()
{
    std::vector<yarp::sig::Pixel> pixelList;
    for (int i=leftTopVertex[0]; i<rightDownVertex[0]; i++)
        for (int j=leftTopVertex[1]; j<rightDownVertex[1]; j++)
            pixelList.push_back(yarp::sig::Pixel(i,j));

    return pixelList;
}

void ObjectReconstr::savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    stringstream s;
    s.str("");
    s<<outputDir + "/3Dobject" <<number;
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

    number++;
    fprintf(stdout, "Writing finished\n");
}

bool ObjectReconstr::updateCloud()
{
    ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(true);
    ImageOf<PixelRgb> *tmpR = imagePortInRight.read(true);

    IplImage* imgL;
    IplImage* imgR;
    if(tmpL!=NULL && tmpR!=NULL)
    {
        imgL= (IplImage*) tmpL->getIplImage();
        imgR= (IplImage*) tmpR->getIplImage();
    }
    else
    {
        fprintf(stdout, "Problem with image ports occurred\n");
        return false;
    }

    std::vector<yarp::sig::Pixel> pixelList=getPixelList();

    return recRoutine.reconstruct(imgL,imgR,pixelList);
}

bool ObjectReconstr::updateModule()
{
    switch(current_state)
    {

    case STATE_WAIT:
        return true;

    case STATE_RECONSTRUCT:
        {
            if (useChris)
            {
                if (!communicator.disableAttention())
                    return false;
            }

            steerEyesToHome();

            recRoutine.resetClouds();

            if (!updateCloud())
                return false;

            /*if (!closing)
                moveTorso(20.0);
            Time::delay(1);

            if (!updateCloud())
                return false;

            if (!closing)
                moveTorso(-20.0);
            Time::delay(1);

            if (!updateCloud())
                return false;*/

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp=recRoutine.getPointCloudComplete();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			filter(tmp,cloud);

            SurfaceMeshWithBoundingBox &pointCloudOnPort=pointCloudPort.prepare();
            pointCloudOnPort.mesh.points.clear();
            pointCloudOnPort.mesh.rgbColour.clear();
            for (unsigned int i=0; i<cloud->width; i++)
            {
                pointCloudOnPort.mesh.points.push_back(PointXYZ(cloud->at(i).x,cloud->at(i).y, cloud->at(i).z));
                pointCloudOnPort.mesh.rgbColour.push_back(RGBA(cloud->at(i).rgba));
            }

            if (computeBB)
            {
                boundingBox=MinimumBoundingBox::getMinimumBoundingBox(cloud);
                pointCloudOnPort.boundingBox=boundingBox.getBoundingBox();
            }

            pointCloudPort.write();

            if (write)
                savePointsPly(cloud);

            //igaze->restoreContext(context_without_eyes_blocked);

            if (visualizationOn)
                current_state=STATE_VISUALIZE;
            else
                current_state=STATE_WAIT;

            middlex=-1;
            middley=-1;

            return true;
        }

    case STATE_VISUALIZE:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=recRoutine.getPointCloud();

            if(useTable)
                addPlanePoints(cloud);

            if (visualizationOn)
            {
                boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
                tmpViewer->setBackgroundColor (0, 0, 0);
                if (computeBB)
                {
                    boundingBox.drawBoundingBox(tmpViewer);
                }
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudComplete;
                cloudComplete=recRoutine.getPointCloudComplete();
                visualize(tmpViewer, cloudComplete);
            }
            current_state=STATE_WAIT;
        }
    }

    return true;
}

void ObjectReconstr::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (cloud_in->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

void ObjectReconstr::addPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Eigen::Vector4f minim;
    Eigen::Vector4f maxim;
    pcl::getMinMax3D(*cloud,minim,maxim);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int i=0; i<cloud->size(); i++)
    {
        pcl::PointXYZRGB point=cloud->at(i);
        if (useTable)
            point.z=tableHeight;
        else
            point.z=minim[2];
        cloud_tmp->push_back(point);
    }

    for (unsigned int i=0; i<cloud_tmp->size(); i++)
        cloud->push_back(cloud_tmp->at(i));
}


bool ObjectReconstr::interruptModule()
{
    closing=true;
    rpc.interrupt();

    imagePortInLeft.interrupt();
    imagePortInRight.interrupt();

    pointCloudPort.interrupt();
    segmentationPort.interrupt();

    return true;
}

double ObjectReconstr::getPeriod()
{
    return 0.1;
}

bool ObjectReconstr::respond(const Bottle& command, Bottle& reply) 
{
    if (command.get(0).asString()=="set")
    {
        if (command.get(1).asString()=="tableHeight")
        {
            setTableHeight(command.get(2).asDouble());
            reply.addVocab(ACK);
            reply.addString("tableHeight ");
            reply.addDouble(command.get(2).asDouble());
            return true;
        }
        else if (command.get(1).asString()=="write")
        {
            if (command.size()>2)
            {
                if (command.get(2).asString()=="true")
                {
                    reply.addVocab(ACK);
                    write=true;
                }
                else if (command.get(2).asString()=="false")
                {
                    write=false;
                    reply.addVocab(ACK);
                }
                else
                    reply.addVocab(NACK);
                return true;
            }
        }
        else if (command.get(1).asString()=="box")
        {
            if (command.size()<6)
            {
                reply.addVocab(NACK);
                reply.addString("not enough argument");
                return true;
            }
            setBoundingBox(command.get(2).asDouble(), command.get(3).asDouble(), command.get(4).asDouble(), command.get(5).asDouble());
            reply.addVocab(ACK);
            reply.addString("bounding box set");
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            reply.addString("command not recognized");
            return true;
        }
    }
    if (command.get(0).asString()=="3Drec")
    {
        if (middlex==-1 || middley==-1)
        {
            reply.addVocab(NACK);
            reply.addString("Click on the segmentation image first");
            return true;
        }
        if (command.size()>=2)
        {
            object=command.get(1).asString();
        }        

        steerEyesToHome();
        Time::delay(1.0);

        current_state=STATE_RECONSTRUCT;

        if (command.size()==3)
            if (command.get(2).asString()=="on")
                visualizationOn=true;

        reply.addVocab(ACK);
        return true;
    }
    if (command.get(0).asString()=="get")
    {
        if (command.get(1).asString()=="point" && command.size()==4)
        {
            if (middlex==-1 || middley==-1)
            {
                reply.addVocab(NACK);
                reply.addString("Click on the segmentation image first");
                return true;
            }
            if (current_state!=STATE_RECONSTRUCT)
            {
                steerEyesToHome();
                Time::delay(1.0);

                IplImage* imgL;
                IplImage* imgR;

                ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(true);
                ImageOf<PixelRgb> *tmpR = imagePortInRight.read(true);

                if(tmpL!=NULL && tmpR!=NULL)
                {
                    imgL= (IplImage*) tmpL->getIplImage();
                    imgR= (IplImage*) tmpR->getIplImage();
                }
                else
                {
                    reply.addVocab(NACK);
                    return true;
                }

                yarp::sig::Vector point2D(2);
                point2D[0]=command.get(2).asDouble();
                point2D[1]=command.get(3).asDouble();

                yarp::sig::Vector point3D(3);

                bool done=recRoutine.triangulateSinglePoint(imgL,imgR,point2D,point3D);

                if (done)
                {
                    reply.addDouble(point3D[0]);
                    reply.addDouble(point3D[1]);
                    reply.addDouble(point3D[2]);
                }
                else
                    reply.addVocab(NACK);
            }
            else
            {
                reply.addVocab(NACK);
                reply.addString("I'm still processing");
            }
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            return true;
        }
    }

    if (command.size()==2)
    {
        if (command.get(0).asInt()!=0 && command.get(1).asInt()!=0)
        {
            middlex=(double)command.get(0).asInt();
            middley=(double)command.get(1).asInt();
            reply.addVocab(ACK);
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            reply.addString("command not recognized");
            return true;
        }
    }

    reply.addVocab(NACK);
    reply.addString("command not recognized");
    return true;
}

void ObjectReconstr::setBoundingBox(double lx, double ly, double rx, double ry)
{
    leftTopVertex[0]=lx;
    leftTopVertex[1]=ly;
    rightDownVertex[0]=rx;
    rightDownVertex[1]=ry;
}

void ObjectReconstr::visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    string id=object+" Cloud";
    tmpViewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id);
    tmpViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    tmpViewer->initCameraParameters();
    while (!tmpViewer->wasStopped())
    {
        if (closing)
        {
            tmpViewer->close();
            break;
        }
        tmpViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    tmpViewer->removePointCloud(id);
}

