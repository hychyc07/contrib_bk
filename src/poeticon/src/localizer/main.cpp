
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <cv.h>

#include <stdio.h>
#include <string>
#include <deque>
#include <vector>
#include <map>
#include <math.h>

#define LEFT      0
#define RIGHT     1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class Blob
{
    CvPoint             center;
    int                 width;
    int                 height;

    map<string,int>     score;

public:
    Blob(Bottle *bot)
    {
        center.x=bot->get(0).asInt();
        center.y=bot->get(1).asInt();

        width=cvCeil(bot->get(2).asDouble()/2.0);
        height=cvCeil(bot->get(3).asDouble()/2.0);
    }

    void vote(const string &obj_name, CvPoint voter)
    {
        //if the object name has not yet been encountered allocate space for it
        if(score.count(obj_name)==0)
            score[obj_name]=0;

        if(abs(center.x-voter.x)<width && abs(center.y-voter.y)<height)
            score[obj_name]++;
    }

    string getBest()
    {
        string best="";
        int best_score=0;
        for(map<string,int>::iterator itr=score.begin(); itr!=score.end(); itr++)
        {
            if(best_score<(*itr).second)
            {
                best_score=(*itr).second;
                best=(*itr).first;
            }
        }

        return best;
    }

    CvPoint getCenter(){return center;}
};


class LocalizerThread: public RateThread
{
protected:
    ResourceFinder &rf;

    string  name;

    Port                            locOutPort;
    BufferedPort<Bottle>            locInPort;
    BufferedPort<Image>             imgInPort[2];
    Port                            imgOutPort[2];

    BufferedPort<Bottle>            blobsPort[2];
    BufferedPort<Image>             bkgSubPort[2];

    Semaphore                       mutex;

    bool                            verbose;

    double                          tbd_time[2];
    vector<CvPoint>                 tbd_list[2];
    vector<CvScalar>                tbd_color_list[2];

    map<string,int*>                        colors;
    map<string,deque<double> *>             margins[2];
    map<string,deque<vector<CvPoint> > *>   histories[2];

    unsigned int                    matched_max_size;

    bool                            blob_draw;

    Semaphore                       draw_mutex;

    vector<Blob>                    blobs;

    string                          current_draw_name;

    bool                            camCheck[2];

    double                          blend;

    IplImage                        *track[2];

    void close()
    {
        locOutPort.interrupt();
        locOutPort.close();

        locInPort.interrupt();
        locInPort.close();

        if(camCheck[LEFT])
        {
            imgInPort[LEFT].interrupt();
            imgInPort[LEFT].close();

            imgOutPort[LEFT].interrupt();
            imgOutPort[LEFT].close();

            blobsPort[LEFT].interrupt();
            blobsPort[LEFT].close();

            bkgSubPort[LEFT].interrupt();
            bkgSubPort[LEFT].close();

            if(track[LEFT]!=NULL)
                cvReleaseImage(&track[LEFT]);
        }

        if(camCheck[RIGHT])
        {
            imgInPort[RIGHT].interrupt();
            imgInPort[RIGHT].close();

            imgOutPort[RIGHT].interrupt();
            imgOutPort[RIGHT].close();

            blobsPort[RIGHT].interrupt();
            blobsPort[RIGHT].close();

            bkgSubPort[RIGHT].interrupt();
            bkgSubPort[RIGHT].close();

            if(track[RIGHT]!=NULL)
                cvReleaseImage(&track[RIGHT]);
        }

        for(int cam=0; cam<2; cam++)
        {
            for(map<string,deque<double> *>::iterator itr=margins[cam].begin(); itr!=margins[cam].end(); itr++)
            {
                delete (*itr).second;
            }
            
            for(map<string,deque<vector<CvPoint> > *>::iterator itr=histories[cam].begin(); itr!=histories[cam].end(); itr++)
            {
                delete (*itr).second;
            }
        }

        for(map<string,int *>::iterator itr=colors.begin(); itr!=colors.end(); itr++)
        {
            delete [] (*itr).second;
        }


    }



    void flood_fill(IplImage *img, const int &x, const int &y, const int *color, IplImage *mask, IplImage *track)
    {
        uchar *ptr=(uchar*) img->imageData + y*img->widthStep;
        uchar *msk=(uchar*) mask->imageData + y*mask->widthStep;
        uchar *kpt=(uchar*) track->imageData + y*track->widthStep;

        if(x>0 && x<img->width && y>0 && y<img->height && kpt[x]==0 && (msk[3*x]!=0 || msk[3*x+1]!=0 || msk[3*x+2]!=0))
        {
            kpt[x]=255;

            ptr[3*x]=cvFloor(blend*ptr[3*x]+(1.0-blend)*color[0]);
            ptr[3*x+1]=cvFloor(blend*ptr[3*x+1]+(1.0-blend)*color[1]);
            ptr[3*x+2]=cvFloor(blend*ptr[3*x+2]+(1.0-blend)*color[2]);

            flood_fill(img,x-1,y,color,mask,track);
            flood_fill(img,x,y-1,color,mask,track);
            flood_fill(img,x+1,y,color,mask,track);
            flood_fill(img,x,y+1,color,mask,track);
        }
    }

    int *generateColor()
    {
        int *c=new int[3];
        double h=Rand::scalar(0.0,360.0);
        double s=1.0;
        double v=200.0;//Rand::scalar(50.0,200.0);

        // convert color
        int i;
        double f, p, q, t;

        if(s == 0)
        {
            c[0] = c[1] = c[2] = int(v);
        }
        else
        {
            h /= 60;
            i = int(floor(h));
            f = h - i;
            p = v * (1 - s);
            q = v * (1 - s * f);
            t = v * (1 - s * (1 - f));

            switch(i){
            default:
              c[2] = int(v);
              c[1] = int(p);
              c[0] = int(q);
              break;
            case 0:
              c[2] = int(v);
              c[1] =int( t);
              c[0] = int(p);
              break;
            case 1:
              c[2] = int(q);
              c[1] = int(v);
              c[0] = int(p);
              break;
            case 2:
              c[2] = int(p);
              c[1] = int(v);
              c[0] = int(t);
              break;
            case 3:
              c[2] = int(p);
              c[1] = int(q);
              c[0] = int(v);
              break;
            case 4:
              c[2] = int(t);
              c[1] = int(p);
              c[0] = int(v);
              break;
            }
        }
        
        return c;
    }


    double dist(CvPoint a, CvPoint b)
    {
        return sqrt((double)(a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }



    void localize(Bottle *bClassifier, vector<Blob> &blobs, deque<vector<CvPoint> > &matchedHistory)
    {
        string obj_name=bClassifier->get(0).asString().c_str();

        vector<CvPoint> tmpMatch;
        matchedHistory.push_back(tmpMatch);
        for(int i=2; i<bClassifier->size(); i+=2)
        {
            /*
            bool insert=true;
            for(unsigned int j=0; j<matchedHistory.back().size() && insert; j++)
                if(matchedHistory.back()[j].x==b->get(i).asInt() && matchedHistory.back()[j].y==b->get(i+1).asInt())
                    insert=false;
        
            if(insert)*/
                matchedHistory.back().push_back(cvPoint(bClassifier->get(i).asInt(),bClassifier->get(i+1).asInt()));
        }

        if(matchedHistory.size()>matched_max_size)
            matchedHistory.pop_front();

        //vector<CvPoint> matched;
        for(unsigned int n=0; n<matchedHistory.size(); n++)
        {
            for(unsigned int i=0; i<matchedHistory[n].size(); i++)
            {
                /*bool insert=true;
                for(unsigned int j=0; j<matched.size(); j++)
                {
                    if(abs(matchedHistory[n].at(i).x-matched[j].x)<1 && abs(matchedHistory[n].at(i).y-matched[j].y)<1)
                        insert=false;
                }

                if(insert)
                    matched.push_back(matchedHistory[n].at(i));*/

                for(unsigned int bl=0; bl<blobs.size(); bl++)
                    blobs[bl].vote(obj_name,matchedHistory[n].at(i));
            }
        }
    }



public:
    LocalizerThread(ResourceFinder &_rf) : RateThread(20), rf(_rf)
    {}

    virtual bool threadInit()
    {
        // general part
        name=rf.check("name",Value("localizer")).asString().c_str();
        setRate(rf.check("period",Value(20)).asInt());

        string camera=rf.check("cameras",Value("left")).asString().c_str();

        verbose=rf.check("verbose");

        blend=rf.check("blend",Value(0.7)).asDouble();

        matched_max_size=rf.check("filter",Value(1)).asInt();

        locOutPort.open(("/"+name+"/loc:o").c_str());
        locInPort.open(("/"+name+"/loc:i").c_str());
        
        if(camera=="left"||camera=="both")
        {
            camCheck[LEFT]=true;
            imgInPort[LEFT].open(("/"+name+"/left/img:i").c_str());
            imgOutPort[LEFT].open(("/"+name+"/left/img:o").c_str());
            blobsPort[LEFT].open(("/"+name+"/left/blobs:i").c_str());
            bkgSubPort[LEFT].open(("/"+name+"/left/bkg:i").c_str());
        }

        if(camera=="right"||camera=="both")
        {
            camCheck[RIGHT]=true;
            imgInPort[RIGHT].open(("/"+name+"/right/img:i").c_str());
            imgOutPort[RIGHT].open(("/"+name+"/right/img:o").c_str());
            blobsPort[RIGHT].open(("/"+name+"/right/blobs:i").c_str());
            bkgSubPort[RIGHT].open(("/"+name+"/right/bkg:i").c_str());
        }

        track[LEFT]=track[RIGHT]=NULL;
        
        tbd_time[LEFT]=tbd_time[RIGHT]=0.0;
        tbd_list[LEFT].clear();
        tbd_list[RIGHT].clear();
        tbd_color_list[LEFT].clear();
        tbd_color_list[RIGHT].clear();


        blob_draw=false;
        current_draw_name="all";

        return true;
    }

    virtual void run()
    {
        double t=Time::now();

        Image *img[2];
        Image *bkg[2];
        Bottle *bBlob[2];
        if(camCheck[LEFT])
        {
            img[LEFT]=imgInPort[LEFT].read(false);
            bBlob[LEFT]=blobsPort[LEFT].read(false);
            bkg[LEFT]=bkgSubPort[LEFT].read(false);


            if(img[LEFT]!=NULL && track[LEFT]==NULL)
            {
                track[LEFT]=cvCreateImage(cvGetSize(img[LEFT]->getIplImage()),IPL_DEPTH_8U,1);
            }
        }

        if(camCheck[RIGHT])
        {
            img[RIGHT]=imgInPort[RIGHT].read(false);
            bBlob[RIGHT]=blobsPort[RIGHT].read(false);
            bkg[RIGHT]=bkgSubPort[RIGHT].read(false);

            if(img[RIGHT]!=NULL && track[RIGHT]!=NULL)
                track[RIGHT]=cvCreateImage(cvGetSize(img[RIGHT]->getIplImage()),IPL_DEPTH_8U,1);
        }


        Bottle *bLoc=locInPort.read(false);


        if(bLoc!=NULL && bLoc->size()>0 && bLoc->get(0).asList()->size()>0)
        {
            Bottle bOut;
            bOut.addList();
            bOut.addList();

            bool once=false;

            mutex.wait();
            for(int cam=0; cam<2; cam++)
            {
                if(camCheck[cam] && bBlob[cam]!=NULL && bBlob[cam]->size()>0)
                {
                    //put all background blobs in a vector
                    blobs.clear();
                    for(int bl=0; bl<bBlob[cam]->size(); bl++)
                        blobs.push_back(Blob(bBlob[cam]->get(bl).asList()));

                    for(int cl=0; cl<bLoc->get(cam).asList()->size(); cl++)
                    {
                        Bottle *bLocEye=bLoc->get(cam).asList()->get(cl).asList();

                        if(colors.count(bLocEye->get(0).asString().c_str())==0)
                            colors[bLocEye->get(0).asString().c_str()]=generateColor();

                        if(histories[cam].count(bLocEye->get(0).asString().c_str())==0)
                            histories[cam][bLocEye->get(0).asString().c_str()]=new deque<vector<CvPoint> >;

                        //get the history correspondent to the current object
                        deque<vector<CvPoint> > *matchedHistory=histories[cam][bLocEye->get(0).asString().c_str()];

                        localize(bLocEye,blobs,*matchedHistory);

                        /*

                        // margin stuff
                        if(margins[cam].count(bLocEye->get(0).asString().c_str())==0)
                            margins[cam][bLocEye->get(0).asString().c_str()]=new deque<double>;

                        deque<double> *tmp_margins=margins[cam][bLocEye->get(0).asString().c_str()];

                        tmp_margins->push_front(bLocEye->get(1).asDouble());
                        if(tmp_margins->size()>min_means)
                            tmp_margins->pop_back();

                        double mean_margin=0.0;                    
                        double std_margin=0.0;    
                        for(unsigned int i=0; i<tmp_margins->size(); i++)
                        {
                            mean_margin+=tmp_margins->at(i);
                            std_margin+=tmp_margins->at(i)*tmp_margins->at(i);
                        }
                        mean_margin/=tmp_margins->size();

                        std_margin=sqrt(std_margin/tmp_margins->size()-mean_margin*mean_margin);

                        fprintf(stdout,"%s has mean margin= %f and std margin= %f\n",bLocEye->get(0).asString().c_str(),mean_margin,std_margin);
                        */
                    }

                    if(track[cam]!=NULL)
                    {
                        //fprintf(stdout,"fatto!\n");
                        cvZero(track[cam]);
                    }
                    
                    bool first_tbd=true;
                    for(unsigned int bl=0; bl<blobs.size(); bl++)
                    {
                        Bottle *bOutEye=bOut.get(cam).asList();

                        string best=blobs[bl].getBest();

                        if(best!="" && (current_draw_name=="all" || current_draw_name==best))
                        {
                            int *tmp_color=colors[best];

                            if(img[cam]!=NULL)
                            {
                                if(bkg[cam]!=NULL && track[cam]!=NULL)
                                {
                                    flood_fill((IplImage*)img[cam]->getIplImage(),blobs[bl].getCenter().x,blobs[bl].getCenter().y,tmp_color,(IplImage*) 
                                        bkg[cam]->getIplImage(),track[cam]);
                                }
                                else
                                {
                                    //fprintf(stdout,"ma come?\n");
                                    //cvCircle(img[cam]->getIplImage(),blobs[bl].getCenter(),5,cvScalar(tmp_color[0],tmp_color[1],tmp_color[2]),3);
                                    if(first_tbd)
                                    {
                                        tbd_list[cam].clear();
                                        tbd_color_list[cam].clear();
                                        tbd_time[cam]=Time::now();
                                        first_tbd=false;
                                    }                          

                                    tbd_list[cam].push_back(blobs[bl].getCenter());   
                                    tbd_color_list[cam].push_back(cvScalar(tmp_color[0],tmp_color[1],tmp_color[2]));
                                }                            
                            
                                once=true;
                            }

                            Bottle &b=bOutEye->addList();
                            b.addString(best.c_str());
                            b.addInt(blobs[bl].getCenter().x);
                            b.addInt(blobs[bl].getCenter().y);
                        }
                    }


                    if(Time::now()-tbd_time[cam]>10.0)
                    {
                        tbd_list[cam].clear();
                        tbd_color_list[cam].clear();
                    }
                                 
                    if(img[cam]!=NULL)
                        for(unsigned int i=0; i<tbd_list[cam].size(); i++)
                            cvCircle(img[cam]->getIplImage(),tbd_list[cam][i],5,tbd_color_list[cam][i],3);
                }
            }
            mutex.post();


            if(camCheck[LEFT] && img[LEFT]!=NULL)
                imgOutPort[LEFT].write(*img[LEFT]);

            if(camCheck[RIGHT] && img[RIGHT]!=NULL)
                imgOutPort[RIGHT].write(*img[RIGHT]);

            if(once)
            {

                locOutPort.write(bOut);
            }

            if(verbose)
                fprintf(stdout,"Time: %f\n",Time::now()-t);

        }

    }

    virtual void threadRelease()
    {
        close();
    }

    void drawObj(string &obj_name)
    {
        draw_mutex.wait();        
        if(obj_name=="draw")
            blob_draw=!blob_draw;
        else
            current_draw_name=obj_name;
        draw_mutex.post();
    }

    void getObjectList(Bottle &reply)
    {
        double t0=Time::now();

        int max_size=0;
        vector<Blob> max_blobs;

        while(Time::now()-t0<0.5)
        {
            mutex.wait();
            reply.clear();

            int size=0;
            for(unsigned int i=0; i<blobs.size(); i++)
                if(blobs[i].getBest()!="")
                    size++;

            if(max_size<size)
            {
                max_size=size;
                max_blobs=blobs;
            }
            mutex.post();   

            Time::delay(0.05);        
        }


        reply.addInt(max_size);

        for(unsigned int i=0; i<max_blobs.size(); i++)
            if(max_blobs[i].getBest()!="")
                reply.addString(max_blobs[i].getBest().c_str());
    }
};


class LocalizerModule: public RFModule
{
protected:
    LocalizerThread       *thr;
    Port            rpcPort;

public:
    LocalizerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();            

        thr=new LocalizerThread(rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open("/localizer/rpc");
        attach(rpcPort);

        return true;
    }

    

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.get(0).asString()=="query")
            thr->getObjectList(reply);

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("milModule/conf");
    //rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    LocalizerModule mod;

    return mod.runModule(rf);
}



