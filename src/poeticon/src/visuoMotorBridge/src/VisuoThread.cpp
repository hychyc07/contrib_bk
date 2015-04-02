
#include <yarp/os/Time.h>

#include "VisuoThread.h"


void VisuoThread::close()
{
    outPort[LEFT].interrupt();
    outPort[RIGHT].interrupt();

    imgPort[LEFT].interrupt();
    imgPort[RIGHT].interrupt();

    mCUTPort[LEFT].interrupt();
    mCUTPort[RIGHT].interrupt();

    pftInPort.interrupt();
    pftOutPort.interrupt();

    boundMILPort.interrupt();
    cmdMILPort.interrupt();
    recMILPort.interrupt();

    cmdMSRPort.interrupt();
    recMSRPort.interrupt();

    outPort[LEFT].close();
    outPort[RIGHT].close();

    imgPort[LEFT].close();
    imgPort[RIGHT].close();

    mCUTPort[LEFT].close();
    mCUTPort[RIGHT].close();

    pftInPort.close();
    pftOutPort.close();

    boundMILPort.close();
    cmdMILPort.close();
    recMILPort.close();

    cmdMSRPort.close();
    recMSRPort.close();

    if(img[LEFT]!=NULL)
        delete img[LEFT];

    if(img[RIGHT]!=NULL)
        delete img[RIGHT];
}


bool VisuoThread::checkTracker(Vector *vec)
{
    bool track=false;

    trackBuffer.push_back(*vec);

    if(isTracking() && trackBuffer.size()>minTrackBufSize)
    {
        double speed_avg[2];
        double speed_std[2];
        speed_avg[LEFT]=speed_avg[RIGHT]=0.0;
        speed_std[LEFT]=speed_std[RIGHT]=0.0;

        for(unsigned int i=1; i<trackBuffer.size(); i++)
        {
            double speed[2];
            speed[LEFT]=(trackBuffer[i-1][0]-trackBuffer[i][0])*(trackBuffer[i-1][0]-trackBuffer[i][0])+
                        (trackBuffer[i-1][1]-trackBuffer[i][1])*(trackBuffer[i-1][1]-trackBuffer[i][1]);
            speed_avg[LEFT]+=sqrt(speed[LEFT]);
            speed_std[LEFT]+=speed[LEFT];

            speed[RIGHT]=(trackBuffer[i-1][0]-trackBuffer[i][0])*(trackBuffer[i-1][0]-trackBuffer[i][0])+
                        (trackBuffer[i-1][1]-trackBuffer[i][1])*(trackBuffer[i-1][1]-trackBuffer[i][1]);
            speed_avg[RIGHT]+=sqrt(speed[RIGHT]);
            speed_std[RIGHT]+=speed[RIGHT];
        }

        double n=1.0/(trackBuffer.size()-1);
        speed_avg[LEFT]*=n;
        speed_std[LEFT]=n*speed_std[LEFT]-speed_avg[LEFT]*speed_avg[LEFT];
        speed_avg[RIGHT]*=n;
        speed_std[RIGHT]=n*speed_std[RIGHT]-speed_avg[RIGHT]*speed_avg[RIGHT];

        track=true;

        // check that the speeds are not varying too much
        if(speed_avg[LEFT]>speedStdThresh || speed_avg[RIGHT]>speedStdThresh)
        {
            track=false;
        }

        double dist=(trackBuffer.back()[0]-trackBuffer.back()[6])*(trackBuffer.back()[0]-trackBuffer.back()[6])+
                    (trackBuffer.back()[1]-trackBuffer.back()[7])*(trackBuffer.back()[1]-trackBuffer.back()[7]);

        // avoid strabicity
        if(sqrt(dist)>stereoDistThresh)
        {
            track=false;
        }
        // check that both images are tracking similar things


        while(trackBuffer.size()>maxTrackBufSize)
            trackBuffer.pop_front();
    }

    return track;
}


void VisuoThread::startTracker(const Vector &stereo, const int &side)
{
    IplImage *ipl_img, *ipl_tpl;
    
    imgMutex.wait();
    ipl_img=img[dominant_eye]!=NULL? cvCloneImage((IplImage *) img[dominant_eye]->getIplImage()):NULL;
    imgMutex.post();

    if(ipl_img!=NULL)
    {
        int x=stereo[2*dominant_eye]-0.5*side<0?0:cvRound(stereo[2*dominant_eye]-0.5*side);
        int y=stereo[2*dominant_eye+1]-0.5*side<0?0:cvRound(stereo[2*dominant_eye+1]-0.5*side);
        int width=stereo[2*dominant_eye]+0.5*side>=ipl_img->width?ipl_img->width-x:side;
        int height=stereo[2*dominant_eye+1]+0.5*side>=ipl_img->height?ipl_img->height-y:side;

        ipl_tpl=cvCreateImage(cvSize(side,side),ipl_img->depth,ipl_img->nChannels);
        cvSetImageROI(ipl_img,cvRect(x,y,width,height));
        cvCopyImage(ipl_img,ipl_tpl);
        cvCvtColor(ipl_tpl,ipl_tpl,CV_BGR2RGB);
        //ImageOf<PixelBgr> tpl;
        if(tpl.width()>0)
        {
            IplImage *tmp_ipl=(IplImage*) tpl.getIplImage();
            //cvReleaseImage(&tmp_ipl);
        }        
        tpl.wrapIplImage(ipl_tpl);
        pftOutPort.write(tpl);

        cvReleaseImage(&ipl_img);
        cvReleaseImage(&ipl_tpl);
    }


    trackMutex.wait();
    stereoTracker.side=side;
    tracking=true;
    trackMutex.post();
}


void VisuoThread::restartTracker()
{
    pftOutPort.write(tpl);

    trackMutex.wait();
    tracking=true;
    trackMutex.post();
}



void VisuoThread::updateImages()
{
    ImageOf<PixelRgb> *iL=imgPort[LEFT].read(false);
    ImageOf<PixelRgb> *iR=imgPort[RIGHT].read(false);

    imgMutex.wait();
    if(iL!=NULL) 
    {
        if(img[LEFT]!=NULL)
            delete img[LEFT];

        img[LEFT]=new ImageOf<PixelRgb>(*iL);
        
        newImage[LEFT]=true;
    }

    if(iR!=NULL)
    {
        if(img[RIGHT]!=NULL)
            delete img[RIGHT];
        
        img[RIGHT]=new ImageOf<PixelRgb>(*iR);
        
        newImage[RIGHT]=true;
    }
    imgMutex.post();
}

void VisuoThread::updateLocationsMIL()
{
    Bottle *bLocations=recMILPort.read(false);

    if(bLocations!=NULL)// && bLocations->size()==1)
    {
        MILMutex.wait();
        /*        
        for(int cam=0; cam<2; cam++)
        {
            locations[cam].clear();
            Bottle *bCam=bLocations->get(cam).asList();
            for(int i=0; i<bCam->size(); i++)
            {
                Bottle *b=bCam->get(i).asList();
                locations[cam][b->get(0).asString().c_str()]=cvPoint(b->get(1).asInt(),b->get(2).asInt());
            }
        }
        */
        locations.clear();
        
        for(int i=0; i<bLocations->get(0).asList()->size(); i++)
        {
            Bottle *b=bLocations->get(0).asList()->get(i).asList();
            locations[b->get(0).asString().c_str()]=cvPoint(b->get(1).asInt(),b->get(2).asInt());
        }
        MILMutex.post();
    }
}


void VisuoThread::updateMotionCUT()
{
    // Detect motion
    Bottle *bMotion[2];
    bMotion[LEFT]=mCUTPort[LEFT].read(false);
    bMotion[RIGHT]=mCUTPort[RIGHT].read(false);

    double tNow=Time::now();

    bool detected=true;

    motMutex.wait();
    for(int cam=0; cam<2; cam++)
    {
        // if even the first element of the buffer is far (in time)
        // clear the whole buffer
        if(buffer[cam].size()>0 && tNow-buffer[cam].back().t>timeTol)
            buffer[cam].clear();

        // If only a single moving blob has been detected
        if (bMotion[cam] && bMotion[cam]->size()==1)
        {
            Item item;
            item.t=Time::now();
            item.size=bMotion[cam]->get(0).asList()->get(2).asDouble();
            item.p=cvPoint(bMotion[cam]->get(0).asList()->get(0).asInt(),bMotion[cam]->get(0).asList()->get(1).asInt());
            buffer[cam].push_back(item);
        }

        // Avoid time unconsistencies
        while (buffer[cam].size() && (tNow-buffer[cam].front().t)>timeTol)
            buffer[cam].pop_front();
    }
    motMutex.post();
}



void VisuoThread::updatePFTracker()
{
    Vector *trackVec=pftInPort.read(false);

    Vector stereo;
    if(trackVec!=NULL && trackVec->size()==12)
    {
        //must check if the tracker has gone mad.
        if(checkTracker(trackVec))
        {
            trackMutex.wait();
            stereoTracker.vec=*trackVec;
            trackMutex.post();

            stereo.resize(4);
            stereo[0]=stereoTracker.vec[0];
            stereo[1]=stereoTracker.vec[1];
            stereo[2]=stereoTracker.vec[6];
            stereo[3]=stereoTracker.vec[7];
            res.setStereo(stereo);
        }
        else
        {
            trackMutex.wait();
            stereoTracker.vec.clear();
            stereoTracker.side=0;
            trackMutex.post();
        }
    }



    imgMutex.wait();
    if(img[LEFT]!=NULL && img[RIGHT]!=NULL)
    {      

        Image drawImg[2];
        drawImg[LEFT]=*img[LEFT];
        drawImg[RIGHT]=*img[RIGHT];

        if(stereoTracker.vec.size()==12)
        {

            cvCircle(drawImg[LEFT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[0]),cvRound(stereoTracker.vec[1])),3,cvScalar(0,255),3);
            cvRectangle(drawImg[LEFT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[2]),cvRound(stereoTracker.vec[3])),
                                                 cvPoint(cvRound(stereoTracker.vec[4]),cvRound(stereoTracker.vec[5])),cvScalar(0,255),3);

            cvCircle(drawImg[RIGHT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[6]),cvRound(stereoTracker.vec[7])),3,cvScalar(0,255),3);
            cvRectangle(drawImg[RIGHT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[8]),cvRound(stereoTracker.vec[9])),
                                                 cvPoint(cvRound(stereoTracker.vec[10]),cvRound(stereoTracker.vec[11])),cvScalar(0,255),3);
            
            //cvCircle(drawImg[LEFT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[0]),cvRound(stereoTracker.vec[1])),stereoTracker.side,cvScalar(0,0,255),3);
            //cvCircle(drawImg[RIGHT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[6]),cvRound(stereoTracker.vec[7])),stereoTracker.side,cvScalar(0,0,255),3);


            Bottle v;
            v.clear();
            Bottle &vl=v.addList();
            vl.addInt(cvRound(stereoTracker.vec[0]));
            vl.addInt(cvRound(stereoTracker.vec[1]));
            vl.addInt(stereoTracker.side);
            Bottle &vr=v.addList();
            vr.addInt(cvRound(stereoTracker.vec[6]));
            vr.addInt(cvRound(stereoTracker.vec[7]));
            vr.addInt(stereoTracker.side);

            boundMILPort.write(v);
        }


        if(newImage[LEFT])
            outPort[LEFT].write(drawImg[LEFT]);
        
        if(newImage[RIGHT])
            outPort[RIGHT].write(drawImg[RIGHT]);

        //avoid writing multiple times the same image
        newImage[LEFT]=false;
        newImage[RIGHT]=false;
    }
    imgMutex.post();
}



VisuoThread::VisuoThread(ResourceFinder &_rf, Resource &_res)
    :RateThread(20),rf(_rf),res(_res)
{

    buffer[LEFT].clear();
    buffer[RIGHT].clear();

    //locations[LEFT].clear();
    //locations[RIGHT].clear();
    locations.clear();

    img[LEFT]=NULL;
    img[RIGHT]=NULL;

    stereoTracker.vec.clear();
    stereoTracker.side=0;
}

bool VisuoThread::threadInit()
{
    string name=rf.find("name").asString().c_str();

    Bottle bVision=rf.findGroup("vision");

    setRate(bVision.check("period",Value(20)).asInt());

    minMotionBufSize=bVision.check("minMotionBufSize",Value(10)).asInt();
    minTrackBufSize=bVision.check("minTrackBufSize",Value(1)).asInt();
    maxTrackBufSize=bVision.check("maxTrackBufSize",Value(2)).asInt();
    timeTol=bVision.check("timeTol",Value(0.5)).asDouble();
    motionStdThresh=bVision.check("motionStdThresh",Value(5.0)).asDouble();
    speedStdThresh=bVision.check("speedStdThresh",Value(700.0)).asDouble();
    stereoDistThresh=bVision.check("stereoDistThresh",Value(300.0)).asDouble();
    dominant_eye=bVision.check("dominant_eye",Value("left")).asString().c_str()=="left"?LEFT:RIGHT;

    // open ports
    outPort[LEFT].open(("/"+name+"/left/img:o").c_str());
    outPort[RIGHT].open(("/"+name+"/right/img:o").c_str());

    imgPort[LEFT].open(("/"+name+"/left/img:i").c_str());
    imgPort[RIGHT].open(("/"+name+"/right/img:i").c_str());

    mCUTPort[LEFT].open(("/"+name+"/left/blobs:i").c_str());
    mCUTPort[RIGHT].open(("/"+name+"/right/blobs:i").c_str());

    boundMILPort.open(("/"+name+"/MIL/window:o").c_str());
    cmdMILPort.open(("/"+name+"/MIL/cmd:o").c_str());
    recMILPort.open(("/"+name+"/MIL/rec:i").c_str());

    cmdMSRPort.open(("/"+name+"/MSR/cmd:o").c_str());
    recMSRPort.open(("/"+name+"/MSR/rec:i").c_str());

    pftInPort.open(("/"+name+"/tracker:i").c_str());
    pftOutPort.open(("/"+name+"/tracker:o").c_str());

    newImage[LEFT]=false;
    newImage[RIGHT]=false;

    show=false;

    return true;
}


void VisuoThread::run()
{
    updateImages();
    updatePFTracker();
    updateLocationsMIL();
    updateMotionCUT();
}


void VisuoThread::threadRelease()
{
    close();
}

Vector VisuoThread::getFixation()
{
    Vector stereo(4);
    imgMutex.wait();
    if(img[LEFT]!=NULL)
    {
        stereo[0]=stereo[2]=0.5*img[LEFT]->width();
        stereo[1]=stereo[3]=0.5*img[LEFT]->height();
    }
    imgMutex.post();


    int side=40;
    startTracker(stereo,side);

    return stereo;
}

Vector VisuoThread::getMotion()
{
    Vector stereo;

    motMutex.wait();
    // If the buffers are sufficently dense and not so small, return true.
    double size=0.0;
    if (buffer[LEFT].size()>minMotionBufSize && buffer[RIGHT].size()>minMotionBufSize)
    {
        Vector p[2];
        for (int cam=0; cam<2; cam++)
        {
            double size_cam,u,v,n;
            double u_std,v_std;
            size_cam=u=v=0.0;
            u_std=v_std=0.0;
            n=1.0/buffer[cam].size();

            for (unsigned int i=0; i<buffer[cam].size(); i++)
            {
                size_cam+=buffer[cam][i].size;
                u+=buffer[cam][i].p.x;
                v+=buffer[cam][i].p.y;
                u_std+=buffer[cam][i].p.x*buffer[cam][i].p.x;
                v_std+=buffer[cam][i].p.y*buffer[cam][i].p.y;
            }

            size_cam*=n;
            u*=n;
            v*=n;
            u_std=sqrt(n*u_std-u*u);
            v_std=sqrt(n*v_std-v*v);

            //check if the motion detected point is not wildly moving
            if (u_std<motionStdThresh && v_std<motionStdThresh)
            {
                p[cam].resize(2);
                p[cam][0]=u;
                p[cam][1]=v;
            }
            else
                break;

            size+=size_cam;
        }

        //size/=2.0;

        int side=cvRound(2*sqrt(size/3.1415)*2);

        if (p[LEFT].size()==2 && p[RIGHT].size()==2)
        {
            stereo.resize(4);
            stereo[0]=p[LEFT][0];
            stereo[1]=p[LEFT][1];
            stereo[2]=p[RIGHT][0];
            stereo[3]=p[RIGHT][1];
            
            startTracker(stereo,cvRound(side));
        }
    }
    motMutex.post();
    return stereo;
}


Vector VisuoThread::getTrack()
{
    Vector stereo;
    trackMutex.wait();
    if(stereoTracker.vec.size()==12)
    {
        stereo.resize(4);
        stereo[0]=stereoTracker.vec[0];
        stereo[1]=stereoTracker.vec[1];
        stereo[2]=stereoTracker.vec[6];
        stereo[3]=stereoTracker.vec[7];

    }
    trackMutex.post();

    //int side=60;
    //startTracker(stereo,side);
    
    return stereo;
}




// if the object is located in both cameras, return its stereo position
Vector VisuoThread::getObject(const std::string &obj_name)
{
    Vector stereo;

    MILMutex.wait();
    /*
    if (locations[LEFT].count(obj_name)>0 && locations[RIGHT].count(obj_name)>0)
    {
        stereo.resize(4);
        stereo[0]=locations[LEFT][obj_name].x;
        stereo[1]=locations[LEFT][obj_name].y;
        stereo[2]=locations[RIGHT][obj_name].x;
        stereo[3]=locations[RIGHT][obj_name].y;

        int side=30;
        startTracker(stereo,side);
    }
    */


    if(locations.count(obj_name)>0)
    {
        stereo.resize(4);
        stereo[0]=locations[obj_name].x;
        stereo[1]=locations[obj_name].y;
        stereo[2]=160.0;//locations[RIGHT][obj_name].x;
        stereo[3]=120.0;//locations[RIGHT][obj_name].y;

        //int side=30;
        //startTracker(stereo,side);
    }
    MILMutex.post();


    return stereo;
}


Bottle VisuoThread::recogMSR(string &obj_name)
{
    Bottle bDetect;
    Bottle *bMSR=recMSRPort.read(false);
    if(bMSR!=NULL)
    {
        bDetect=*bMSR;
        obj_name=bDetect.get(0).asString().c_str();
    }
    return bDetect;
}



bool VisuoThread::startLearningMIL(const std::string &obj_name)
{
    if(!isTracking())
        return false;

    Bottle command,reply;
    command.fromString(("learn " + obj_name + " template").c_str());

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}


bool VisuoThread::suspendLearningMIL()
{
    Bottle command,reply;
    command.fromString("set label 0");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}

bool VisuoThread::resumeLearningMIL()
{
    Bottle command,reply;
    command.fromString("set label template");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}

bool VisuoThread::trainMIL()
{
    Bottle command,reply;
    command.fromString("train");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}


bool VisuoThread::startLearningMSR(const string &obj_name, const int &arm)
{
    Bottle command,reply;

    //set the arm holding object
    string set_arm=(arm==LEFT)?"left":"right";

    command.fromString(("set arm " + set_arm).c_str());
    cmdMSRPort.write(command,reply);

    if(reply.toString()!=("\"set arm " + set_arm + " command received.\"").c_str())
    {  
        fprintf(stdout,"Error when setting arm!\n",reply.toString().c_str());
        return false;
    }
    command.clear();
    reply.clear();
    command.fromString(("explore " + obj_name).c_str());

    cmdMSRPort.write(command,reply);
        
    return (reply.toString()==("\"explore " + obj_name + " command received.\"").c_str());
}


bool VisuoThread::startRecogMSR(const string &obj_name, const int &arm)
{
    Bottle command,reply;

    //set the arm holding object
    string set_arm=(arm==LEFT)?"left":"right";

    command.fromString(("set arm " + set_arm).c_str());
    cmdMSRPort.write(command,reply);

    if(reply.toString()!=("\"set arm " + set_arm + " command received.\"").c_str())
        return false;

    command.clear();
    reply.clear();

    fprintf(stdout, "this is the command sent %s\n", obj_name.c_str());
    
    command.fromString( ("recog " + obj_name).c_str() );
    cmdMSRPort.write(command,reply);

    return (reply.toString()==( "\"recog "+ obj_name +" command received.\"").c_str() );
}


bool VisuoThread::suspendLearningMSR()
{
    Bottle command,reply;
    command.fromString("suspend");

    cmdMSRPort.write(command,reply);

    return (reply.toString()=="suspending \"suspend command received.\"");
}

bool VisuoThread::resumeLearningMSR()
{
    Bottle command,reply;
    command.fromString("resume");

    cmdMSRPort.write(command,reply);


    return (reply.toString()=="resuming \"resume command received.\"");
}



bool VisuoThread::stopMSR()
{
    Bottle command,reply;
    command.fromString("stop");

    cmdMSRPort.write(command,reply);

    return (reply.toString()=="\"stop command received.\"");
}




void VisuoThread::checkDoneMSR(bool &done)
{
    Bottle *bCheck=recMSRPort.read(false);

    if(bCheck!=NULL && bCheck->get(0).asInt()==1)
        done=true;
    else
        done=false;
}
