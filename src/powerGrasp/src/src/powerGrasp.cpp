#include "powerGrasp.h"
#ifndef __PCL_POINT_TYPE_CONVERSION__
    #include <pcl/point_types_conversion.h>
    #define __PCL_POINT_TYPE_CONVERSION__
#endif

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;
using namespace iCub::learningmachine;
using namespace iCub::data3D;
using namespace pcl::io;
using namespace pcl;

PowerGrasp::PowerGrasp() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), 
    cloudWithPlane(new pcl::PointCloud<pcl::PointXYZRGB>),
    cloudxyz(new pcl::PointCloud<pcl::PointXYZ>),
    princip_curves_colors(new pcl::PointCloud<PointXYZRGB>),
    normals (new pcl::PointCloud <pcl::Normal>)
{
    modality=MODALITY_AUTO;
    path="";
    filenameTrain="";

    chosenPoint.resize(3,0.0);
    chosenPixel.resize(2,0.0);
    chosenPoint[0]=-0.3;
    chosenPoint[2]=0.3;

    offsetR.resize(3,0.0);
    offsetL.resize(3,0.0);

    dont=false;
    grasped=false;
    train=false;
    visualize=true;
    straight=false;
    rightBlocked=false;
    leftBlocked=false;
    fromFileFinished=false;
    filterCloud=true;
    graspSpecificPoint=false;
    clicked=false;
    writeCloud=false;
    nFile=0.0;

    current_state=STATE_WAIT;
    visualizationThread=new VisualizationThread(data);
    orientationThreadRight=new OrientationThread();
    orientationThreadLeft=new OrientationThread();
}

bool PowerGrasp::configure(ResourceFinder &rf)
{
    tableHeight=rf.check("tableHeight",Value(-0.2)).asDouble();
    useTable=rf.check("useTable");
    minClusterPoints=rf.check("minClusterPoints",Value(100)).asInt();
    maxClusterPoints=rf.check("maxClusterPoints",Value(100000)).asInt();
    neighborsForNormal=rf.check("neighborsForNormal",Value(30)).asInt();
    radiusSearch=rf.check("radiusSearch",Value(0.045)).asDouble();
    neighborsForKSearch=rf.check("neighborsForKSearch",Value(20)).asInt();
    smoothnessThreshold=(float)rf.check("smoothnessThreshold",Value(7.0)).asDouble();
    curvatureThreshold=(float)rf.check("curvatureThreshold",Value(1.0)).asDouble();
    int nAngles=rf.check("nAngles",Value(50)).asInt();
    numberOfBestPoints=rf.check("numberOfBestPoints",Value(10)).asInt();
    percentage=rf.check("percentage",Value(0.2)).asDouble();
    best_curvature=(float)rf.check("curvature",Value(0.005)).asDouble();
    handSize=rf.check("handSize",Value(0.08)).asDouble();
    filenameTrain=rf.find("filenameTrain").asString().c_str();
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string name=rf.check("name",Value("powerGrasp")).asString().c_str();
    string useFile=rf.check("fromFile",Value("false")).asString().c_str();
    path=rf.find("path").asString().c_str();

    string right_arm="right_arm";
    string left_arm="left_arm";
    if (!orientationThreadRight->open(right_arm,robot,nAngles) || !orientationThreadLeft->open(left_arm,robot,nAngles))
        return false;   
    
    fromFile=(useFile=="true");
    
    if (fromFile)
    {
        path=rf.find("path").asString().c_str();
        if (path=="")
            return false;
    }

    Bottle *pR=rf.find("offsetR").asList();
    if (pR->size()>0)
    {
        for (int i=0; i<pR->size(); i++)
            offsetR[i]=pR->get(i).asDouble();
    }

    Bottle *pL=rf.find("offsetL").asList();
    if (pL->size()>0)
    {
        for (int i=0; i<pL->size(); i++)
            offsetR[i]=pL->get(i).asDouble();
    }

    //Ports opening
    rpc.open(("/"+name+"/rpc").c_str());
    attach(rpc);

    if (!meshPort.open(("/"+name+"/mesh:i").c_str()))
        return false;

    if (!areCmdPort.open(("/"+name+"/are/cmd:o").c_str()))
        return false;

    if (!areRpcPort.open(("/"+name+"/are/rpc:o").c_str()))
        return false;

    if (!reconstructionPort.open(("/"+name+"/reconstruction").c_str()))
        return false;

    //Encoders opening:
    Property optRight;
    string remoteRight="/"+robot+"/right_arm";
    optRight.put("device", "remote_controlboard");
    optRight.put("remote",remoteRight.c_str());
    optRight.put("local","/enc/localArm/right_arm");

    robotArmRight.open(optRight);

    Property optLeft;
    string remoteLeft="/"+robot+"/left_arm";
    optLeft.put("device", "remote_controlboard");
    optLeft.put("remote",remoteLeft.c_str());
    optLeft.put("local","/enc/localArm/left_arm");

    robotArmLeft.open(optLeft);

    if (!robotArmRight.isValid() || !robotArmLeft.isValid())
    {
        printf("Device not available\n");
        return false;
    }

    robotArmRight.view(encRight);
    robotArmLeft.view(encLeft);
    
    blockRightTmp=false;
    blockLeftTmp=false;

    // lssvm: default values
    scalerIn.setLowerBoundIn(0.0);
    scalerIn.setUpperBoundIn(0.03);
    scalerIn.setLowerBoundOut(0.0);
    scalerIn.setUpperBoundOut(1.0);

    scalerOut.setLowerBoundIn(0.0);
    scalerOut.setUpperBoundIn(0.6);
    scalerOut.setLowerBoundOut(0.0);
    scalerOut.setUpperBoundOut(1.0);

    machine.setDomainSize(1);
    machine.setCoDomainSize(1);
    machine.setC(20.0);
    machine.getKernel()->setGamma(100.0);

    Bottle &bMachine=rf.findGroup("lssvm");
    if (!bMachine.isNull())
    {
        if (bMachine.check("c"))
            machine.setC(bMachine.find("c").asDouble());

        if (bMachine.check("gamma"))
            machine.getKernel()->setGamma(bMachine.find("gamma").asDouble());

        if (Bottle *v=bMachine.find("in_bounds").asList())
        {
            if (v->size()>=2)
            {
                scalerIn.setLowerBoundIn(v->get(0).asDouble());
                scalerIn.setUpperBoundIn(v->get(1).asDouble());
            }
        }

        if (Bottle *v=bMachine.find("out_bounds").asList())
        {
            if (v->size()>=2)
            {
                scalerOut.setLowerBoundIn(v->get(0).asDouble());
                scalerOut.setUpperBoundIn(v->get(1).asDouble());
            }
        }

        if (Bottle *v=bMachine.find("machine").asList())
        {
            testWithLearningEnabled=machine.fromString(v->toString().c_str());
            testWithLearning=true;
            /*for (double i=0.0; i<0.2; i+=0.001)
			{
				Vector in(1,scalerIn.transform(i));
				Prediction prediction=machine.predict(in);
				Vector out=prediction.getPrediction();
				printf("Prediction %g\n", scalerOut.unTransform(out[0]));
			}*/
		}
        else
            testWithLearning=false;
    }
    else
        testWithLearning=false;

    return true;
}

bool PowerGrasp::interruptModule()
{
    reconstructionPort.interrupt();
    areCmdPort.interrupt();
    areRpcPort.interrupt();
    meshPort.interrupt();
    rpc.interrupt();
    orientationThreadRight->stop();
    orientationThreadLeft->stop();

    return true;
}

bool PowerGrasp::close()
{
    reconstructionPort.close();
    areRpcPort.close();
    areCmdPort.close();
    meshPort.close();
    rpc.close();

    if (graspFileTrain.is_open())
        graspFileTrain.close();

    if (visualizationThread->isRunning())
        visualizationThread->stop();
    delete visualizationThread;

    orientationThreadRight->close();
    orientationThreadLeft->close();

    delete orientationThreadRight;
    delete orientationThreadLeft;

    if (robotArmRight.isValid())
        robotArmRight.close();
    if (robotArmLeft.isValid())
        robotArmLeft.close();

    return true;
}

void PowerGrasp::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (cloud_in->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

void PowerGrasp::write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, string fileName)
{
    ofstream myfile;
    myfile.open (fileName.c_str());
    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << cloud_in->size() << "\n";
    myfile << "property float x\n";
    myfile << "property float y\n";
    myfile << "property float z\n";
    myfile << "property uchar diffuse_red\n";
    myfile << "property uchar diffuse_green\n";
    myfile << "property uchar diffuse_blue\n";
    myfile << "end_header\n";
    for (int i=0; i<cloud_in->size(); i++)
    {
        myfile << cloud_in->at(i).x << " " << cloud_in->at(i).y << " " << cloud_in->at(i).z << " " << cloud_in->at(i).r << " " << cloud_in->at(i).g << " " << cloud_in->at(i).b << "\n";
    }
    myfile.close();
}

void PowerGrasp::fromSurfaceMesh (const SurfaceMeshWithBoundingBox& msg)
{
    cloud->clear();
    cloudxyz->clear();

    for (size_t i = 0; i<msg.mesh.points.size(); ++i)
    {
        PointXYZRGB pointrgb;
        pointrgb.x=msg.mesh.points.at(i).x;
        pointrgb.y=msg.mesh.points.at(i).y;
        pointrgb.z=msg.mesh.points.at(i).z;
        if (i<msg.mesh.rgbColour.size())
        {
            int32_t rgb= msg.mesh.rgbColour.at(i).rgba;
            pointrgb.rgba=rgb;
            pointrgb.r = (rgb >> 16) & 0x0000ff;
            pointrgb.g = (rgb >> 8)  & 0x0000ff;
            pointrgb.b = (rgb)       & 0x0000ff;
        }
        else
            pointrgb.rgb=0;

        pcl::PointXYZ point;
        point.x=pointrgb.x;
        point.y=pointrgb.y;
        point.z=pointrgb.z;
        cloudxyz->push_back(point);
        cloud->push_back(pointrgb);
    }

    boundingBox.setBoundingBox(msg.boundingBox);
}

bool PowerGrasp::fillCloudFromFile()
{
    struct dirent *entry;
    DIR *dp;
     
    dp = opendir(path.c_str());
    if (dp == NULL) 
    {
        perror("opendir");
        return false;
    }

    while((entry = readdir(dp)))
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        else
            break;
    }

    if (entry->d_name!=NULL)
    {
        pcl::PointCloud<PointXYZRGB>::Ptr cloud_in_rgb (new pcl::PointCloud<PointXYZRGB>);

        string root=path+"/";
        string name=entry->d_name;
        string file=root+name;

        if (loadPLYFile(file.c_str(), *cloud_in_rgb) == -1) 
        {
            cout << "cannot read file \n";
            return false;
        }

        if (filterCloud)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
            filter(cloud_in_rgb,cloud_in_filtered);
            addPointCloud(cloud_in_filtered);
        }
        else
            addPointCloud(cloud_in_rgb);
    }
    closedir(dp);
    return true;
}

int PowerGrasp::findIndex(yarp::sig::Vector &point)
{
    double minNorm=100000;
    int index=-1;
    for (int i=0; i<cloud->size(); i++)
    {
        yarp::sig::Vector currentPoint(3);
        currentPoint[0]=cloud->at(i).x;
        currentPoint[1]=cloud->at(i).y;
        currentPoint[2]=cloud->at(i).z;

        if (norm(point-currentPoint)<minNorm)
        {
            index=i;
            minNorm=norm(point-currentPoint);
        }
    }
    return index;
}

bool PowerGrasp::updateModule()
{
    if ((fromFile && !fromFileFinished) || (current_state==STATE_ESTIMATE))
    {
        double totTime=Time::now();
        if (fromFile)
        {
            if (!fillCloudFromFile())
                return false;
            boundingBox=MinimumBoundingBox::getMinimumBoundingBox(cloud);
        }    
        else if (current_state==STATE_ESTIMATE)
        {
            SurfaceMeshWithBoundingBox *receivedMesh=meshPort.read(false);
            if (receivedMesh!=NULL)
            {
                fromSurfaceMesh(*receivedMesh);
            }
            else
                return true;
        }

        if (writeCloud)
        {
            stringstream ss;
            ss << nFile;
            string str = ss.str();
            string filename=path+"/cloud"+str+".ply";
            printf("Filename %s\n", filename.c_str());
            write(cloud,filename);
            nFile++;
        }

        int pointIndex;
        if (graspSpecificPoint)
            pointIndex=findIndex(chosenPoint);

        yarp::sig::Vector center=boundingBox.getCenter();

        //Normal Estimation
        double timeNormals=Time::now();
        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setRadiusSearch (radiusSearch);
        normal_estimator.setInputCloud (cloudxyz);
        normal_estimator.compute (*normals);
        printf("Time for normal estimation %g\n", Time::now()-timeNormals);

        if (graspSpecificPoint)
        {
            rankIndices.clear();
            rankScores.clear();

            yarp::sig::Vector curr_normal(3);
            curr_normal[0]=normals->at(pointIndex).normal_x;
            curr_normal[1]=normals->at(pointIndex).normal_y;
            curr_normal[2]=normals->at(pointIndex).normal_z;

            double score=scoreFunction(chosenPoint,curr_normal, normals->at(pointIndex).curvature,current_curvature,modality);
            insertElement(score,pointIndex);
        }
        else
        {
            //Clustering
            double timeClustering=Time::now();
            maxClusterPoints=(int)cloudxyz->size();
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize (minClusterPoints);
            reg.setMaxClusterSize (maxClusterPoints);
            reg.setSearchMethod (tree);
            reg.setNumberOfNeighbours (neighborsForKSearch);
            reg.setInputCloud (cloudxyz);
            reg.setInputNormals (normals);
            reg.setSmoothnessThreshold (smoothnessThreshold / 180.0 * M_PI);
            reg.setCurvatureThreshold (curvatureThreshold);

            std::vector<pcl::PointIndices> clusters;
            reg.extract (clusters);
            int totPoints=fillClusteredCloud(clusters);
            printf("Time for clustering %g\n", Time::now()-timeClustering);
            
            if (train)
            {
                double tmp=Random::uniform();
                //current_curvature=(float)fmod(tmp,max_curvature);
                printf("Chosen curvature %g\n", current_curvature);
                printf("Max curvature %g\n", max_curvature);
            }
            else
                current_curvature=best_curvature;
                
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud (cloudxyz);

            //Saving only clusters that are big enough
            std::vector<pcl::PointIndices> winningClusters;

            double timeWinningCl=Time::now();
            for (int i=0; i<clusters.size(); i++)
            {
                if (clusters.at(i).indices.size()>totPoints/(clusters.size()+3))
                    winningClusters.push_back(clusters.at(i));
            }
            printf("Time for winning clustering %g\n", Time::now()-timeWinningCl);

            rankIndices.clear();
            rankScores.clear();

            current_modality=modality;

            if (modality==MODALITY_AUTO)   
                manageModality(current_modality);

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			
			int n_good_points=0;

            //Choose best points
            int term=10;
			int cloud_perc=floor(cloud->size()/term);
            double timeBestPoints=Time::now();
            for (int i=0; i<winningClusters.size(); i++)
            {
                double numberOfPoints=percentage*((double)winningClusters.at(i).indices.size());
                double factor=(double)winningClusters.at(i).indices.size()/numberOfPoints;
                int factorI=(int)factor;
                for (int j=0; j<winningClusters.at(i).indices.size(); j++)
                {
                    int index=winningClusters.at(i).indices.at(j);
                    int n_neighbohrs=kdtree.radiusSearch(cloudxyz->at(index),radiusSearch,pointIdxRadiusSearch,pointRadiusSquaredDistance);
                    if (n_neighbohrs>cloud_perc)
                    {
						if (normalPointingOut(normals->at(index),cloudxyz->at(index),center))
						{
							n_good_points++;
							yarp::sig::Vector point(3);
							point[0]=cloudxyz->at(index).x;
							point[1]=cloudxyz->at(index).y;
							point[2]=cloudxyz->at(index).z;
							yarp::sig::Vector curr_normal(3);
							curr_normal[0]=normals->at(index).normal_x;
							curr_normal[1]=normals->at(index).normal_y;
							curr_normal[2]=normals->at(index).normal_z;
							double score=scoreFunction(point,curr_normal,normals->at(index).curvature,current_curvature,current_modality);
							insertElement(score,index);
						}
					}
                }
				if (n_good_points<20 && cloud_perc>0)
				{
					cloud_perc=floor(cloud->size()/(term+1));
					n_good_points=0;
                    term++;
                    i=i-1;
				}
            }

            printf("Final good points %d\n", n_good_points);          
            
            string smodality;
            if (current_modality==MODALITY_CENTER)
                smodality="center";
            else if (current_modality==MODALITY_LEFT)
                smodality="left";
            else if (current_modality==MODALITY_RIGHT)
                smodality="right";
            else
                smodality="top";
                
            printf("modality chosen %s\n", smodality.c_str());
			printf("Ranked curvatures ");
			for (size_t i=0; i<rankScores.size(); i++)
				printf("%g ", normals->at(rankIndices[i]).curvature);
			printf("\n");
            printf("Time for best points %g\n", Time::now()-timeBestPoints);
        }

        Matrix designedOrientation=eye(4,4);
        winnerIndex=-1;

        //Choose best point and orientation
        double timeChoosing=Time::now();
        string hand=chooseBestPointAndOrientation(winnerIndex,designedOrientation);
        printf("Time for choosing %g\n", Time::now()-timeChoosing);

        if (hand==NO_HAND)
        {
            printf("Didn't find any suitable point\n");
            current_state=STATE_WAIT;
            return true;
        }

        chosenHand=hand;

        yarp::sig::Vector offset;
        if (hand==RIGHT_HAND)
            offset=offsetR;
        else
            offset=offsetL;

        if (!graspSpecificPoint)
        {
            chosenPoint[0]=cloudxyz->at(winnerIndex).x;
            chosenPoint[1]=cloudxyz->at(winnerIndex).y;
            chosenPoint[2]=cloudxyz->at(winnerIndex).z;
        }

        printf("Chosen point: %s\n", chosenPoint.toString().c_str());
        chosenOrientation=dcm2axis(designedOrientation);
        printf("Chosen orientation: %s\n", chosenOrientation.toString().c_str());
        printf("Chosen hand: %s\n", chosenHand.c_str());
        printf("Chosen normal: %g %g %g %g\n", normals->at(winnerIndex).normal_x, normals->at(winnerIndex).normal_y, normals->at(winnerIndex).normal_z, normals->at(winnerIndex).curvature);

        if (visualize)
        {
            data.cloud=*cloud;
            if (graspSpecificPoint)
                data.cloud_clustered=*cloud;
            else
                data.cloud_clustered=*princip_curves_colors;
            data.normals=*normals;
            data.boundingBox=boundingBox;
            data.chosenPoint=chosenPoint;
            data.chosenOrientation=designedOrientation;
            data.goodPointsIndexes=rankIndices;
            data.hand=hand;

            /*graspFileTrain.open(filenameTrain.c_str(), ios_base::app); 
            for (int k=0; k<rankIndices.size(); k++)
            {
                graspFileTrain << cloudxyz->points.at(rankIndices[k]).x << " " << cloudxyz->points.at(rankIndices[k]).y << " " << cloudxyz->points.at(rankIndices[k]).z << " " << normals->at(rankIndices[k]).normal_x << " " << normals->at(rankIndices[k]).normal_y << " " << normals->at(rankIndices[k]).normal_z << "\n";
            }

            if (graspFileTrain.is_open())
                graspFileTrain.close();*/

            visualizationThread->start();
        }

        if (graspSpecificPoint)
            graspSpecificPoint=false;

        if (!fromFile)
        {
            readyToGrasp=true;
            chosenPoint+=offset;
            if (!dont && straight)
                current_state=STATE_GRASP;
            else
            {
                dont=false;                
                current_state=STATE_WAIT;
            }
        }
        else
            fromFileFinished=true;
        printf("Tot time %g\n", Time::now()-totTime);
    }

    if (current_state==STATE_GRASP)
    {
        mutex.wait();
        askToGrasp();
        mutex.post();        
        current_state=STATE_WAIT;
    }
    
    return true;
}

void PowerGrasp::manageModality(int &current_modality)
{
    yarp::sig::Vector refz(3); refz=0.0; refz[2]=1.0;
    yarp::sig::Vector refy(3); refy=0.0; refy[1]=1.0;
    yarp::sig::Vector center=boundingBox.getCenter();
    yarp::sig::Vector dim=boundingBox.getDim();
    yarp::sig::Vector x,y,z;
    boundingBox.getAxis(x,y,z);
    x=x/norm(x);
    y=y/norm(y);
    z=z/norm(z);
    
    double dotzx=fabs(dot(x,refz));
    double dotzy=fabs(dot(y,refz));
    double dotzz=fabs(dot(z,refz));
    double dotyx=fabs(dot(x,refy));
    double dotyy=fabs(dot(y,refy));
    double dotyz=fabs(dot(z,refy));
    double anglez;

    int indz=-1;
    int indx=-1;
    int indy=-1;
    if (dotzx>dotzy && dotzx>dotzz)
    {
        indz=0;
        anglez=acos(dotzx);
        if (dotyy>dotyz)
        {
            indx=2;
            indy=1;
        }
        else
        {
            indy=1;
            indx=2;
        }
    }
    else if (dotzy>dotzx && dotzy>dotzz)
    {
        indz=1;
        anglez=acos(dotzy);
        if (dotyx>dotyz)
        {
            indx=2;
            indy=0;
        }
        else
        {
            indx=0;
            indy=2;
        }
    }
    else
    {
        indz=2;
        anglez=acos(dotzz);
        if (dotyx>dotyy)
        {
            indx=1;
            indy=0;
        }
        else
        {
            indx=0;
            indy=1;
        }
    }

    yarp::sig::Vector right(3);
    right[0]=-0.3;
    right[1]=0.3;
    right[2]=0.0;
    yarp::sig::Vector left(3);
    left[0]=-0.3;
    left[1]=-0.3;
    left[2]=0.0;

    double distRight=norm(right-center);
    double distLeft=norm(left-center);

    if (distRight/distLeft>1.8)
    {
        printf("left closer\n");
        blockRightTmp=true;
    }
    else if (distLeft/distRight>1.8)
    {
        printf("right closer\n");
        blockLeftTmp=true;
    }

    if (dimz<handSize)
    {
        printf("object height\n");
        current_modality=MODALITY_TOP;
    }
    else if ((dim[indz]/dim[indy]>1.8) && (dim[indz]/dim[indx]>1.8) && (anglez<60*M_PI/180))
    {
        printf("z bigger\n");
        if (distRight/distLeft>1.3)
            current_modality=MODALITY_LEFT;
        else if (distLeft/distRight>1.3)
            current_modality=MODALITY_RIGHT;
        else
            current_modality=(int)(Random::uniform(0,1)+0.5);
    }
    else if ((dim[indx]/dim[indy]>1.5 && dim[indx]/dim[indz]>1.5) || (dim[indy]/dim[indx]>1.5 && dim[indy]/dim[indz]))
    {
        printf("center\n");
        current_modality=MODALITY_CENTER;
    }
    else 
    {
        printf("top\n");
        current_modality=MODALITY_TOP;
    }
}

void PowerGrasp::askToGrasp()
{
    Bottle cmd,reply;
    cmd.addString("grasp");
    Bottle &point=cmd.addList();
    point.addDouble(chosenPoint[0]);
    point.addDouble(chosenPoint[1]);
    point.addDouble(chosenPoint[2]);
    point.addDouble(chosenOrientation[0]);
    point.addDouble(chosenOrientation[1]);
    point.addDouble(chosenOrientation[2]);
    point.addDouble(chosenOrientation[3]);
    cmd.addString(chosenHand.c_str());

    yarp::sig::Vector dim=boundingBox.getDim();

    if (areCmdPort.getOutputCount()>0)
    {
        areCmdPort.write(cmd,reply);
        if (reply.get(0).asString()=="ack")
            grasped=true;

        if (train)
        {
            yarp::sig::Vector encoders(16);
            if (chosenHand=="right")
                encRight->getEncoders(encoders.data());
            else
                encLeft->getEncoders(encoders.data());
            graspFileTrain.open(filenameTrain.c_str(), ios_base::app);                 

            graspFileTrain << normals->at(winnerIndex).curvature << " " << encoders.toString().c_str() << " " << dim[0]*dim[1]*dim[2] << "\n";
            if (graspFileTrain.is_open())
                graspFileTrain.close();
        }
    }
    readyToGrasp=false;
}

bool PowerGrasp::respond(const Bottle& command, Bottle& reply) 
{
    string tag_0=command.get(0).asString().c_str();
    if (tag_0=="set")
    {
        if (command.size()<3)
        {
            reply.addString("nack, command not recognized");
            return true;
        }
        string tag_1=command.get(1).asString().c_str();
        if (tag_1=="visualization")
        {
            if (command.get(2).asString()=="on")
                visualize=true;
            else
                visualize=false;
            reply.addString("ack");
            return true;
        }
        if (tag_1=="train")
        {
            if (command.get(2).asString()=="on")
            {
                train=true;
                testWithLearning=false;
            }
            else
                train=false;
            reply.addString("ack");
            return true;
        }
        if (tag_1=="testWithLearning")
        {
            if (command.get(2).asString()=="on")
            {
				if (testWithLearningEnabled)
				{
					testWithLearning=true;
					train=false;
				}
				else
					printf("Failure, machine not set\n");
            }
            else
                testWithLearning=false;
            reply.addString("ack");
            return true;
        }
        if (tag_1=="offsetR")
        {
            if (command.size()<5)
            {
                reply.addString("nack, check offset size");
                return true;
            }
            else
            {
                offsetR[0]=command.get(2).asDouble();
                offsetR[1]=command.get(3).asDouble();
                offsetR[2]=command.get(4).asDouble();
                reply.addString("ack");
                return true;
            }
        }
        else if (tag_1=="offsetL")
        {
            if (command.size()<5)
            {
                reply.addString("nack, check offset size");
                return true;
            }
            else
            {
                offsetL[0]=command.get(2).asDouble();
                offsetL[1]=command.get(3).asDouble();
                offsetL[2]=command.get(4).asDouble();
                reply.addString("ack");
                return true;
            }
        }
        else if (tag_1=="modality")
        {
            if (command.get(2).asString()=="right")
                modality=MODALITY_RIGHT;
            else if (command.get(2).asString()=="left")
                modality=MODALITY_LEFT;
            else if (command.get(2).asString()=="center")
                modality=MODALITY_CENTER;
            else if (command.get(2).asString()=="top")
                modality=MODALITY_TOP;
            else
                modality=MODALITY_AUTO;
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="filter")
        {
            if (command.get(2).asString()=="true")
                filterCloud=true;
            else
                filterCloud=false;
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="write")
        {
            if (command.get(2).asString()=="on")
                writeCloud=true;
            else
                writeCloud=false;
            reply.addString("ack");
            return true;
        }
    }
    if (tag_0=="go")
    {
        if (readyToGrasp)
        {
            blockRightTmp=false;
            blockLeftTmp=false;
            current_state=STATE_GRASP;
            reply.addString("ack");
        }
        else
            reply.addString("nack");
        return true;
    }
    if (tag_0=="block")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=true;
            else
                leftBlocked=true;
            reply.addString("ack");
            return true;
        }
        else
        {
            reply.addString("nack");
        }
    }
    if (tag_0=="release")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=false;
            else
                leftBlocked=false;
            reply.addString("ack");
            return true;
        }
        else
        {
            reply.addString("nack");
            return true;
        }
    }
    if (tag_0=="isGrasped")
    {
        string r=grasped?"true":"false";
        reply.addString(r.c_str());
        return true;
    }
    if (tag_0=="dont")
    {
        dont=true;
        readyToGrasp=false;
        blockRightTmp=false;
        blockLeftTmp=false;
        reply.addString("ack");
        current_state=STATE_WAIT;
        return true;
    }
    if (tag_0=="stop")
    {
        Bottle cmd1;
        cmd1.addString("interrupt");

        if (areRpcPort.getOutputCount()>0)
            areRpcPort.write(cmd1,reply);

        grasped=false;
        readyToGrasp=false;
        blockRightTmp=false;
        blockLeftTmp=false;

        cmd1.clear();
        cmd1.addString("reinstate");

        if (areRpcPort.getOutputCount()>0)
            areRpcPort.write(cmd1,reply);

        reply.addString("ack");
        current_state=STATE_WAIT;
        return true;
    }
    if (tag_0=="grasp")
    {
        if (current_state==STATE_ESTIMATE)
        {
            reply.addString("nack");
            return true;
        }

        if (!clicked)
        {
            reply.addString("Click on the image first");
            return true;
        }

        if (visualizationThread->isRunning())
            visualizationThread->stop();

        readyToGrasp=false;
        grasped=false;
        dont=false;
        blockRightTmp=false;
        blockLeftTmp=false;
        max_curvature=0.0;

        Bottle cmd1;
        reply.clear();
        cmd1.addString("idle");

        if (areCmdPort.getOutputCount()>0)
            areCmdPort.write(cmd1,reply);

        if (train && command.size()>1)
            current_curvature=(float)command.get(1).asDouble();

        if (command.size()>1)
        {
            if (command.get(1)=="point")
            {
                Bottle cmd2;
                reply.clear();
                cmd2.addString("get");
                cmd2.addString("point");
                cmd2.addDouble(chosenPixel[0]);
                cmd2.addDouble(chosenPixel[1]);

                if (reconstructionPort.getOutputCount()>0)
                    reconstructionPort.write(cmd2,reply);

                if (reply.size()>0 && reply.get(0).asString()!="nack")
                {
                    graspSpecificPoint=true;
                    chosenPoint[0]=reply.get(0).asDouble();
                    chosenPoint[1]=reply.get(1).asDouble();
                    chosenPoint[2]=reply.get(2).asDouble();
                }
                else
                    return true;
            }
        }

        Bottle cmd2;
        reply.clear();
        cmd2.addString("3Drec");

        if (reconstructionPort.getOutputCount()>0)
            reconstructionPort.write(cmd2,reply);
                
        if (reply.size()>0 && reply.get(0).asString()=="ack")
        {
            reply.clear();
            current_state=STATE_ESTIMATE;
            if (command.size()>=2)
                straight=(command.get(1).asString()=="straight");
            reply.addString("ack");
        }
        else
            reply.addString("nack");
                
        return true;
    }
    if (command.size()==2)
    {
        if (command.get(0).asInt()!=0 && command.get(1).asInt()!=0)
        {
            Bottle cmd1;
            reply.clear();
            cmd1.addInt(command.get(0).asInt());
            cmd1.addInt(command.get(1).asInt());
            chosenPixel[0]=command.get(0).asInt();
            chosenPixel[1]=command.get(1).asInt();

            if (reconstructionPort.getOutputCount()>0)
                reconstructionPort.write(cmd1,reply);

            clicked=true;

            if ((reply.size()>0) && (reply.get(0).asString()=="ack"))
            {
                reply.clear();
                reply.addString("ack");
            }
            else
            {
                reply.clear();
                reply.addString("nack");
            }
            return true;
        }
    }
    reply.addString("nack");
    return true;
}

string PowerGrasp::chooseBestPointAndOrientation(int &winnerIndex, yarp::sig::Matrix &designedOrientation)
{
    yarp::sig::Vector center(3), eePos(3);
    yarp::sig::Vector pyRight(3), pxRight(3), pyLeft(3), pxLeft(3);
    yarp::sig::Vector pointNormal(3), normalRight(3), normalLeft(3);
    yarp::sig::Vector tmpRight(3), tmpLeft(3);
    yarp::sig::Vector xhandaxisRight(3), xhandaxisLeft(3);
    Matrix orientationRight, orientationLeft;
    Matrix tmpOrientationRight, tmpOrientationLeft;
    double rightMan,leftMan;
    double bestRightMan=0.0;
    double bestLeftMan=0.0;
    int rightIndex=-1;
    int leftIndex=-1;

    yarp::sig::Vector x(3);
    x[0]=-1.0;
    x[1]=0.0;
    x[2]=0.0;
 
    string hand=NO_HAND;

    yarp::sig::Vector biggerAxis;
    yarp::sig::Vector vx,vy,vz;
    boundingBox.getAxis(vx,vy,vz);

    double normx=norm(vx);
    double normy=norm(vy);
    double normz=norm(vz);

    if (normx>normy && normx>normz)
        biggerAxis=vx;
    else if (normy>normx && normy>normz)
        biggerAxis=vy;
    else
        biggerAxis=vz;

    biggerAxis/=norm(biggerAxis);
    
    orientationThreadRight->storeContext();
    orientationThreadLeft->storeContext();

    if (current_modality==MODALITY_LEFT)
        blockRightTmp=true;
    if (current_modality==MODALITY_RIGHT)
        blockLeftTmp=true;

    for (int i=0; i<rankScores.size(); i++)
    {
        pxRight=0.0;
        
        pointNormal[0]=normals->at(rankIndices[i]).normal_x;
        pointNormal[1]=normals->at(rankIndices[i]).normal_y;
        pointNormal[2]=normals->at(rankIndices[i]).normal_z;
        
        normalRight[0]=-pointNormal[0];
        normalRight[1]=-pointNormal[1];
        normalRight[2]=-pointNormal[2];
        
        //left hand reference frame is different with respect to the right one
        normalLeft[0]=pointNormal[0];
        normalLeft[1]=pointNormal[1];
        normalLeft[2]=pointNormal[2];

        tmpRight=0.0;
        tmpRight=cross(x,normalRight);
        pxRight=cross(normalRight,tmpRight);
        pxRight=pxRight/norm(pxRight);

        tmpLeft=0.0;
        tmpLeft=cross(x,normalLeft);
        pxLeft=cross(normalLeft,tmpLeft);
        pxLeft=pxLeft/norm(pxLeft);
        
        pyRight=cross(normalRight,pxRight);
        pyRight=pyRight/norm(pyRight);

        pyLeft=cross(normalLeft,pxLeft);
        pyLeft=pyLeft/norm(pyLeft);

        center[0]=cloudxyz->at(rankIndices[i]).x+pointNormal[0];
        center[1]=cloudxyz->at(rankIndices[i]).y+pointNormal[1];
        center[2]=cloudxyz->at(rankIndices[i]).z+pointNormal[2];
           
        eePos[0]=cloudxyz->at(rankIndices[i]).x;
        eePos[1]=cloudxyz->at(rankIndices[i]).y;
        eePos[2]=cloudxyz->at(rankIndices[i]).z;

        if (!rightBlocked && !blockRightTmp)
        {
            orientationThreadRight->setInfo(eePos,pxRight,pyRight,normalRight,center,biggerAxis);
            orientationThreadRight->resume();
        }
        if (!leftBlocked && !blockLeftTmp)
        {
            orientationThreadLeft->setInfo(eePos,pxLeft,pyLeft,normalLeft,center,biggerAxis);
            orientationThreadLeft->resume();
        }

        while (!orientationThreadRight->checkDone() || !orientationThreadLeft->checkDone())
        {
            Time::delay(0.01);
        }
        
        if (!rightBlocked && !blockRightTmp)
        {
            orientationThreadRight->getBestManipulability(rightMan,tmpOrientationRight);
            if (rightMan>bestRightMan)
            {
                bestRightMan=rightMan;
                orientationRight=tmpOrientationRight;
                rightIndex=i;
            }
        }

        if (!leftBlocked && !blockLeftTmp)
        {
            orientationThreadLeft->getBestManipulability(leftMan,tmpOrientationLeft);
            if (leftMan>bestLeftMan)
            {
                bestLeftMan=leftMan;
                orientationLeft=tmpOrientationLeft;
                leftIndex=i;
            }
        }
    }

    orientationThreadRight->restoreContext();
    orientationThreadLeft->restoreContext();

    if (rightBlocked || bestLeftMan>bestRightMan)
    {
        winnerIndex=rankIndices[leftIndex];
        designedOrientation=orientationLeft;
        hand=LEFT_HAND;
    }

    if (leftBlocked || bestRightMan>bestLeftMan)
    {
        winnerIndex=rankIndices[rightIndex];
        designedOrientation=orientationRight;
        hand=RIGHT_HAND;
    }

    return hand;
}

void PowerGrasp::addPlanePoints()
{
    Eigen::Vector4f minim;
    Eigen::Vector4f maxim;
    pcl::getMinMax3D(*cloud,minim,maxim);
    copyPointCloud(*cloud,*cloudWithPlane);

    for (int i=0; i<cloud->size(); i++)
    {
        pcl::PointXYZRGB point=cloud->at(i);
        if (useTable)
        {
            minHeight=tableHeight;
            point.z=tableHeight;
        }
        else
        {
            minHeight=minim[2];
            point.z=minim[2];
        }
        cloudWithPlane->push_back(point);
    }
}

void PowerGrasp::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    cloud->clear();
    cloudxyz->clear();

    for (size_t j = 0; j < cloud_in->points.size (); ++j)
    {
        PointXYZRGB pointrgb=cloud_in->points[j];
        pcl::PointXYZ point;
        point.x=pointrgb.x;
        point.y=pointrgb.y;
        point.z=pointrgb.z;
        cloudxyz->push_back(point);
        cloud->push_back(pointrgb);
    }
}

double PowerGrasp::getPeriod()
{
    return 0.1;
}

double PowerGrasp::scoreFunction(yarp::sig::Vector &point, yarp::sig::Vector &normal, float &curvature, float &best_curvature, int curr_modality)
{
    double score=0.0;
    
    yarp::sig::Vector center=boundingBox.getCenter();
    yarp::sig::Vector dim=boundingBox.getDim();
    yarp::sig::Vector x,y,z;
    boundingBox.getAxis(x,y,z);
    x/=norm(x); y/=norm(y); z/=norm(z);
    
    double minDim;
    if (dim[0]>dim[1])
        minDim=dim[0];
    else
        minDim=dim[1];
    if (dim[2]>minDim)
        minDim=dim[2];
    
    if (point[2]<center[2])
        return 0.0;
    
    if (train)
        return exp(-(fabs(curvature-best_curvature)/max_curvature))/2;

    double dotx=fabs(dot(normal,x));
    double doty=fabs(dot(normal,y));
    double dotz=fabs(dot(normal,z));

    /*if (dotx>=0.98 && dotx<=1.0)
    {
        printf("Good normal!\n");
        score+=1.0;
    }
    if (doty>=0.98 && doty<=1.0)
    {
        printf("Good normal!\n");
        score+=1.0;
    }
    if (dotz>=0.98 && dotz<=1.0)
    {
        printf("Good normal!\n");
        score+=1.0;
    }*/
    
    /*int count=0;
    for (int i=0; i<boundingBox.getCorners().size(); i++)
    {
        iCub::data3D::PointXYZ p=boundingBox.getCorners().at(i);
        yarp::sig::Vector tmp(3);
        tmp[0]=p.x;
        tmp[1]=p.y;
        tmp[2]=p.z;
        if (norm(tmp-point)>minDim/1.5)
            count++;
    }
    
    if (count==8)
        score+=1.0;*/

    //printf("Curvature term %g\n", exp(-((curvature-best_curvature)/max_curvature))/2);
    if (testWithLearning)
    {
        Vector in(1,scalerIn.transform(curvature));
        Prediction prediction=machine.predict(in);
        Vector out=prediction.getPrediction();
        score+=scalerOut.unTransform(out[0]);
        
        //double c=exp(-(fabs(curvature-best_curvature)/max_curvature));
        
        //printf("Curvature %g Predict %g\n", curvature, scalerOut.unTransform(out[0]));
    }
    else
    {
		//printf("Curvature %g Term %g\n", curvature, exp(-(fabs(curvature-best_curvature)/max_curvature))/2);
        score+=exp(-(fabs(curvature-best_curvature)/max_curvature))/2;
	}
    
    if (curr_modality==MODALITY_TOP)
    {
		yarp::sig::Vector tmp(3);
        tmp=center;
        tmp[2]=center[2]+(dimz/2);
        //printf("modality %g\n", exp(-(norm(point-tmp)/dimz))/2);
        return score+sqrt(fabs(point[2]-center[2])/dimz);

        //return score+=exp(-(norm(point-tmp)/dimz))/2;
    }
    else if (curr_modality==MODALITY_RIGHT)
    {
        if (point[1]-center[1]>0.0)
        {
            //printf("modality %g\n", abs(point[1]-center[1])/dimy);
            return score+(fabs(point[1]-center[1])/dimy);
        }
        else
        {
            //printf("modality 0\n");
            return score;
        }
    }
    else if (curr_modality==MODALITY_LEFT)
    {
        if (point[1]-center[1]<0.0)
        {
            //printf("modality %g\n", abs(point[1]-center[1])/dimy);
            return score+(fabs(point[1]-center[1])/dimy);
        }
        else
        {
            //printf("modality 0\n");
            return score;
        }
    }
    else
    {
		yarp::sig::Vector tmp(3);
        tmp=center;
        tmp[2]=center[2]+(dimz/2);
        //printf("modality %g\n", exp(-(norm(point-tmp)/dimz))/2);
        //return score+sqrt(fabs(point[2]-center[2])/dimz)+exp(-(fabs(point[0]-center[0])/dimz))/2;

        return score+=exp(-(norm(point-tmp)/dimz))/2;
    }
}

void PowerGrasp::insertElement(double score, int index)
{
    if (rankScores.size()==0)
    {
        rankScores.push_back(score);
        rankIndices.push_back(index);     
    }
    else if (rankScores.size()<numberOfBestPoints)
    {
        bool assigned=false;
        std::vector<int>::iterator itind=rankIndices.begin();
        for (std::vector<double>::iterator itsc = rankScores.begin(); itsc!=rankScores.end(); itsc++)
        {
            if (*itsc<score)
            {
                rankScores.insert(itsc,score);
                rankIndices.insert(itind,index);
                assigned=true;
                break;
            }
            itind++;
        }
        if (!assigned)
        {
            rankScores.push_back(score);
            rankIndices.push_back(index);
        }
    }
    else
    {
        if (rankScores[rankScores.size()-1]>score)
        {
            return;
		}
        else if (rankScores[0]<score)
        {
            std::vector<double>::iterator itsc=rankScores.begin();
            std::vector<int>::iterator itind=rankIndices.begin();
            rankScores.insert(itsc,score);
            rankIndices.insert(itind,index);
            rankScores.pop_back();
            rankIndices.pop_back();
        }
        else
        {
            std::vector<int>::iterator itind=rankIndices.begin();
            for (std::vector<double>::iterator itsc = rankScores.begin(); itsc!=rankScores.end(); itsc++)
            {
                if (*itsc<score)
                {
                    rankScores.insert(itsc,score);
                    rankIndices.insert(itind,index);
                    rankScores.pop_back();
                    rankIndices.pop_back();
                    break;
                }
                itind++;
            }
        }
    }
    /*printf("Ranked scores ");
    for (size_t i=0; i<rankScores.size(); i++)
		printf("%g ", rankScores.at(i));
	printf("\n");*/
}

bool PowerGrasp::normalPointingOut(pcl::Normal &normal, pcl::PointXYZ &point, yarp::sig::Vector &center)
{
    yarp::sig::Vector p(3);
    p[0]=point.x;
    p[1]=point.y;
    p[2]=point.z;

    yarp::sig::Vector fromCenter=(center-p)/norm(center-p);

    yarp::sig::Vector n(3);
    n[0]=normal.normal_x;
    n[1]=normal.normal_y;
    n[2]=normal.normal_z;
    n=n/norm(n);
    
    return (dot(n,fromCenter)<0);
}

int PowerGrasp::fillClusteredCloud(std::vector<pcl::PointIndices> clusters)
{
    maxy=0.0;
    maxz=0.0;
    double miny=1.0;
    double minz=1.0;
    princip_curves_colors->clear();
    colorMap.resize(clusters.size());
    for (int i=0; i<colorMap.size(); i++)
    {
        colorMap[i].resize(3);
        colorMap[i][0]=yarp::os::Random::uniform(0,255);
        colorMap[i][1]=yarp::os::Random::uniform(0,255);
        colorMap[i][2]=yarp::os::Random::uniform(0,255);
    }

    int totPoints=0;
    for (int i=0; i<clusters.size(); i++)
    {
        totPoints+=clusters.at(i).indices.size();
        for (int j=0; j<clusters[i].indices.size(); j++)
        {
            double curr_curv=normals->at(clusters[i].indices[j]).curvature;
            if (curr_curv>max_curvature)
                max_curvature=curr_curv;
            pcl::PointXYZ point=cloudxyz->at(clusters[i].indices[j]);
            PointXYZRGB pointxyzrgb;
            pointxyzrgb.x=point.x;
            pointxyzrgb.y=point.y;
            pointxyzrgb.z=point.z;
            pointxyzrgb.r=colorMap[i][0];
            pointxyzrgb.g=colorMap[i][1];
            pointxyzrgb.b=colorMap[i][2];
            princip_curves_colors->push_back(pointxyzrgb);
            if (point.y>maxy)
                maxy=point.y;
            if (point.z>maxz)
                maxz=point.z;
            if (point.y<miny)
                miny=point.y;
            if (point.z<minz)
                minz=point.z;
        }
    }
    
    printf("Max curvature %g\n", max_curvature);
    
    dimz=maxz-minz;
    dimy=maxy-miny;

    return totPoints;
}


