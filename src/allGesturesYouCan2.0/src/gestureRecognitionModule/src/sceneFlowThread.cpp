#include "sceneFlowThread.h"

SceneFlowThread::SceneFlowThread(yarp::os::ResourceFinder &rf, DisparityThread* d, OpticalFlowThread* o) : RateThread(10) 
{
    initPosition=NULL;
    frameDiff=NULL;
    string configFileDisparity=rf.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str();
    threshold=rf.check("threshold",Value(30)).asInt();
    value=rf.check("value",Value(80)).asInt();

    success=true;
    init=true;
    finish=false;
    work=false;

    dimension=rf.check("dimension",Value(5)).asInt();
    minimum=-1.00001;
    maximum=1.00001;

    computeEdges();
    string contextPath=rf.getContextPath().c_str();
    string dictionary_name=rf.check("dictionary_file",Value("dictionary_flow.ini")).asString().c_str();

    string dictionary_path=contextPath+"/"+dictionary_name;
    string dictionary_group=rf.check("dictionary_group",Value("DICTIONARY")).asString().c_str();
    sparse_coder=new DictionaryLearning(dictionary_path,dictionary_group);
    disp=d;
    opt=o;
}

void SceneFlowThread::setThreshold(int threshold)
{
    this->threshold=threshold;
}

bool SceneFlowThread::isOpen()
{
    return success;
}

yarp::sig::Vector SceneFlowThread::getCode()
{
    return this->code;
}

void SceneFlowThread::process()
{
    work=true;
    finish=false;
}

bool SceneFlowThread::checkDone()
{
    return finish;
}

void SceneFlowThread::threadRelease()
{
    if(sparse_coder!=NULL)
        delete sparse_coder;
    if (initPosition!=NULL)
        cvReleaseImage(&initPosition);
    if (frameDiff!=NULL)
        cvReleaseImage(&frameDiff);
}

void SceneFlowThread::run()
{
    if(!success)
        return;

    if (init)
    {
        disp->getDisparity(dispNew);
        IplImage disparity=dispNew;
        thresholdBW(&disparity, &disparity, threshold,0,0,value);
        initPosition=cvCreateImage(cvSize(disparity.width,disparity.height),8,1);
        frameDiff=cvCreateImage(cvSize(disparity.width,disparity.height),8,1);
        cvZero(initPosition);
        cvZero(frameDiff);
        disp->getRectMatrix(RLNew);
        disp->getMapper(mapperNew);
        disp->getQMat(QNew);
        disp->getDisparityFloat(dispFloatNew);

        init=false;
        fprintf(stdout, "Init Scene Flow Done...\n");
    }
    else if(work)
    {
        RLOld=RLNew.clone();
        QOld=QNew.clone();
        mapperOld=mapperNew.clone();
        dispOld=dispNew.clone();
        dispFloatOld=dispFloatNew.clone();

        disp->getDisparity(dispNew);
        IplImage disparity=dispNew;
        thresholdBW(&disparity, &disparity, threshold,0,0,value);
        cvAbsDiff(&disparity,initPosition,frameDiff);
        cvThreshold(frameDiff,frameDiff,10,1,CV_THRESH_BINARY);
        cvMul(frameDiff,&disparity,&disparity);
        disp->getDisparityFloat(dispFloatNew);
        disp->getRectMatrix(RLNew);
        disp->getMapper(mapperNew);
        disp->getQMat(QNew);
        opt->getOptFlow(optFlow);
        disp->getRootTransformation(HL_root,0);
        vector<Point3f> flow3D;
        getSceneFlowThread(flow3D);
        computeFlowHistogram(flow3D, flowHist);
        computeFlowCode(flowHist,code);
    }
    
    finish=true;
    this->suspend();
}

void SceneFlowThread::computeFlowCode(yarp::sig::Vector& flowHist, yarp::sig::Vector& code)
{
    sparse_coder->computeCode(flowHist,code);
}


void SceneFlowThread::triangulate(Point2f &pixel,Point3f &point,Mat &Mapper, Mat &disparity, Mat &Q, Mat &RLrect) 
{
    int u=(int) pixel.x; 
    int v=(int) pixel.y;

    // Mapping from Rectified Cameras to Original Cameras
    if(Mapper.empty()) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    }

    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1]; 

    u=cvRound(usign);
    v=cvRound(vsign);

    IplImage disp16=disparity;

    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    }
    else 
    {
        CvScalar scal= cvGet2D(&disp16,v,u);
        double dispVal=-scal.val[0]/16.0;
        float w= (float) ((float) dispVal*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
        point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
        point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
        point.z=(float) Q.at<double>(2,3);
        
     
        point.x=point.x/w;
        point.y=point.y/w;
        point.z=point.z/w;

    }
    // discard points far more than 2.5 meters or with not valid disparity (<0)
    //fprintf(stdout, "Point Before Trans: %f %f %f %f %f\n",usign, vsign,point.x,point.y,point.z);
    if(point.z>2.5 || point.z<0) 
    {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return;
    } 
    else 
    {
        Mat RLrecttemp=RLrect.t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect = Mat::eye(4, 4, CV_64F);
        buildRotTras(RLrecttemp,Tfake,Hrect);
      
        P=HL_root*Hrect*P;
        

        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
        //fprintf(stdout, "Point After Trans: %f %f %f \n",point.x,point.y,point.z);
    }
}

void SceneFlowThread::buildRotTras(Mat & R, Mat & T, Mat & A) 
{
    for(int i = 0; i < R.rows; i++)
    {
        double* Mi = A.ptr<double>(i);
        double* MRi = R.ptr<double>(i);
        for(int j = 0; j < R.cols; j++)
             Mi[j]=MRi[j];
    }
    for(int i = 0; i < T.rows; i++)
    {
        double* Mi = A.ptr<double>(i);
        double* MRi = T.ptr<double>(i);
        Mi[3]=MRi[0];
     }
}

Point3f SceneFlowThread::getSceneFlowThreadPixel(int u, int v)
{
    Point3f flowPoint;
    flowPoint.x=0.0;
    flowPoint.y=0.0;
    flowPoint.z=0.0;

    if(v>=dispFloatNew.rows || u>=dispFloatNew.cols || optFlow.empty())
    {
        return flowPoint;
    }

    int valOld=(int)dispFloatOld.ptr<uchar>(v)[u];
    int valNew=(int)dispFloatNew.ptr<uchar>(v+cvRound(optFlow.ptr<float>(v)[2*u+1]))[u+cvRound(optFlow.ptr<float>(v)[2*u])];

    if (valOld==0 || valNew==0)
    {
        return flowPoint;
    }

    Point2f point2Dold;
    point2Dold.x=(float)u;
    point2Dold.y=(float)v;
    Point3f point3Dold;
    triangulate(point2Dold,point3Dold,mapperOld,dispFloatOld,QOld,RLOld);
    if (point3Dold.x==0.0)
    {
        return flowPoint;
    }

    Point2f point2Dnew;
    point2Dnew.x=u+optFlow.ptr<float>(v)[2*u];
    point2Dnew.y=v+optFlow.ptr<float>(v)[2*u+1];
    Point3f point3Dnew;
    triangulate(point2Dnew,point3Dnew,mapperNew,dispFloatNew,QNew,RLNew);
    if (point3Dnew.x==0.0)
    {
        return flowPoint;
    }

    flowPoint.x=point3Dnew.x-point3Dold.x;
    flowPoint.y=point3Dnew.y-point3Dold.y;
    flowPoint.z=point3Dnew.z-point3Dold.z;

    return flowPoint;
}


void SceneFlowThread::getSceneFlowThread(vector<Point3f> &flow3D)
{
    flow3D.clear();
    Point3f flowPoint;
    flowPoint.x=0.0;
    flowPoint.y=0.0;
    flowPoint.z=0.0;

    if(optFlow.empty())
        return;
    
    for (int v=0; v<optFlow.rows; v++)
    {
        for(int u=0; u<optFlow.cols; u++)
        {
            int valOld=(int) dispOld.ptr<uchar>(v)[u];
            double flowy=v+cvRound(optFlow.ptr<float>(v)[2*u+1]);
            double flowx=u+cvRound(optFlow.ptr<float>(v)[2*u]);
            if(flowy<0 || flowx<0 || flowx>=optFlow.cols || flowy>=optFlow.rows)
                continue;
            
            int valNew=(int)dispNew.ptr<uchar>(v+cvRound(optFlow.ptr<float>(v)[2*u+1]))[u+cvRound(optFlow.ptr<float>(v)[2*u])];

            if (valOld==0 || valNew==0)
            {
                continue;
            }

            Point2f point2Dold;
            point2Dold.x=(float)u;
            point2Dold.y=(float)v;
            Point3f point3Dold;

            triangulate(point2Dold,point3Dold,mapperOld,dispFloatOld,QOld,RLOld);
            if (point3Dold.x==0.0)
            {
                continue;
            }

            Point2f point2Dnew;
            point2Dnew.x=u+optFlow.ptr<float>(v)[2*u];
            point2Dnew.y=v+optFlow.ptr<float>(v)[2*u+1];
            
     
            Point3f point3Dnew;

            triangulate(point2Dnew,point3Dnew,mapperNew,dispFloatNew,QNew,RLNew);
            if (point3Dnew.x==0.0)
            {
                continue;
            }
            
            //fprintf(stdout, "2dFlow: %f %f %f %f 3dPoints %f %f %f %f %f %f\n",point2Dold.x,point2Dold.y, point2Dnew.x,point2Dnew.y,point3Dold.x,point3Dold.y,point3Dold.z,point3Dnew.x,point3Dnew.y,point3Dnew.z);
            flowPoint.x=point3Dnew.x-point3Dold.x;
            flowPoint.y=point3Dnew.y-point3Dold.y;
            flowPoint.z=point3Dnew.z-point3Dold.z;

            flow3D.push_back(flowPoint);
            }
    }
}


void SceneFlowThread::drawFlowModule(IplImage* imgMotion)
{
    IplImage * module =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),32,1);
    IplImage * moduleU =cvCreateImage(cvSize(imgMotion->width,imgMotion->height),8,1);
    Mat vel[2];
    split(optFlow,vel);
    IplImage tx=(Mat)vel[0];
    IplImage ty=(Mat)vel[1];

    IplImage* velxpow=cvCloneImage(&tx);
    IplImage* velypow=cvCloneImage(&ty);

    cvPow(&tx, velxpow, 2);
    cvPow(&ty, velypow, 2);

    cvAdd(velxpow, velypow, module, NULL);
    cvPow(module, module, 0.5);
    cvNormalize(module, module, 0.0, 1.0, CV_MINMAX, NULL);
    cvZero(imgMotion);
    cvConvertScale(module,moduleU,255,0);
    cvMerge(moduleU,moduleU,moduleU,NULL,imgMotion);
    cvReleaseImage(&module);
    cvReleaseImage(&moduleU);
}


bool SceneFlowThread::threadInit()
{
    return true;
}

int SceneFlowThread::getImgWidth()
{
    return width;
}

int SceneFlowThread::getImgHeight()
{
    return height;
}

void SceneFlowThread::printMatrix(Mat &matrix) 
{
    int row=matrix.rows;
    int col =matrix.cols;
        cout << endl;
    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
        cout << endl;
}


void SceneFlowThread::generateMatIndex(double num, int &index)
{
    index=-1;
    unsigned int i;
    for (i=0; i<edges.size()-1; i++)
        if((num>=edges.at(i)) && (num<edges.at(i+1)))
            break;
    index=i;
}

void SceneFlowThread::computeEdges()
{
    edges.push_back(minimum);
    double div=(maximum-minimum)/dimension;
    double temp=minimum;
    for (int i=0; i<dimension-1; i++)
    {
        temp+=div;
        edges.push_back(temp);
    }
    edges.push_back(maximum);
}

void SceneFlowThread::computeFlowHistogram(const vector<Point3f>& flow3D,  yarp::sig::Vector& flowHist)
{
    flowHist.clear();

    if (flow3D.size()==0)
    {
        for (int i=0; i<dimension*dimension*dimension; i++)
            flowHist.push_back(0);
        return;
    }

    vector<Mat> matHistogram;
    for (int i=0; i<dimension; i++)
    {
        Mat matrix=Mat::zeros(dimension,dimension,CV_32F);
        matHistogram.push_back(matrix);
    }

    for (unsigned int i=0; i<flow3D.size(); i++)
    {
        int x,y,z;
        double n=sqrt((flow3D.at(i).x*flow3D.at(i).x)+(flow3D.at(i).y*flow3D.at(i).y)+(flow3D.at(i).z*flow3D.at(i).z));
        generateMatIndex((flow3D.at(i).x/n),x);
        generateMatIndex((flow3D.at(i).y/n),y);
        generateMatIndex((flow3D.at(i).z/n),z);
        if(x<0 || y<0 || z<0 || x>=dimension || y>=dimension || z>=dimension)
            continue;
        matHistogram.at(x).ptr<float>(y)[z]+=1;
    }
    
    double normalizer=0;
    for (unsigned int i=0; i<matHistogram.size(); i++)
    {
        for (int j=0; j<dimension; j++)
        {
            for (int k=0; k<dimension; k++)
            {
                flowHist.push_back(matHistogram.at(i).ptr<float>(j)[k]);
                normalizer=normalizer+matHistogram.at(i).ptr<float>(j)[k];
            }
        }
    }

    for (unsigned int i=0; i<flowHist.size(); i++)
    {
        if(normalizer>0)
            flowHist[i]=flowHist[i]/normalizer;
    }

    return;
}

void SceneFlowThread::thresholdBW(IplImage *img, IplImage* res, int threshold, int x, int y, double value)
{
    CvSize size =cvGetSize(img);

    CvScalar s;
    if(value==0)
    {
        if(x<=0 || y<=0 || x>size.width || y>size.height) {
            int centerW= (int) size.width/2;
            int centerH= (int) size.height/2;
            s = cvGet2D(img,centerH,centerW);
        } else {
            s = cvGet2D(img,y,x);
        }
    }
    else {
        s.val[0]=value;
        s.val[1]=value;
        s.val[2]=value;
    }

    cvThreshold(img, res, s.val[0]-threshold, 255, CV_THRESH_TOZERO);
}

yarp::sig::Vector SceneFlowThread::getHist()
{
    return this->flowHist;
}

void SceneFlowThread::setValue(int value)
{
    this->value=value;
}

void SceneFlowThread::setInitialPositionFrame(IplImage* initPosition)
{
    if (this->initPosition!=NULL)
        cvReleaseImage(&this->initPosition);
    this->initPosition=(IplImage*)cvClone(initPosition);
}
