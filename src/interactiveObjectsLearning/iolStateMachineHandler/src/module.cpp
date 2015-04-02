/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <sstream>
#include <stdio.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include "module.h"

#define RET_INVALID     -1

using namespace yarp::math;


/**********************************************************/
int Manager::processHumanCmd(const Bottle &cmd, Bottle &b)
{
    int ret=Vocab::encode(cmd.get(0).asString().c_str());
    b.clear();

    if (cmd.size()>1)
    {
        if (cmd.get(1).isList())
            b=*cmd.get(1).asList();
        else
            b=cmd.tail();
    }

    return ret;
}


/**********************************************************/
Bottle Manager::skimBlobs(const Bottle &blobs)
{
    Bottle skimmedBlobs;
    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        // skim out blobs that are too far in the cartesian space
        Vector pos;
        if (get3DPosition(cog,pos))
        {
            if ((pos[0]>skim_blobs_x_bounds[0])&&(pos[0]<skim_blobs_x_bounds[1])&&
                (pos[1]>skim_blobs_y_bounds[0])&&(pos[1]<skim_blobs_y_bounds[1]))
                skimmedBlobs.add(blobs.get(i));
        }
    }

    return skimmedBlobs;
}


/**********************************************************/
Bottle Manager::getBlobs()
{
    // grab resources
    mutexResources.wait();

    if (Bottle *pBlobs=blobExtractor.read(false))
    {
        lastBlobs=skimBlobs(*pBlobs);
        printf("Received blobs list: %s\n",lastBlobs.toString().c_str());
        
        if (lastBlobs.size()==1)
        {
            if (lastBlobs.get(0).asVocab()==Vocab::encode("empty"))
                lastBlobs.clear();
        }
    }

    // release resources
    mutexResources.post();
    
    return lastBlobs;
}


/**********************************************************/
CvPoint Manager::getBlobCOG(const Bottle &blobs, const int i)
{
    CvPoint cog=cvPoint(RET_INVALID,RET_INVALID);
    if ((i>=0) && (i<blobs.size()))
    {
        CvPoint tl,br;
        Bottle *item=blobs.get(i).asList();
        if (item==NULL)
            return cog;

        tl.x=(int)item->get(0).asDouble();
        tl.y=(int)item->get(1).asDouble();
        br.x=(int)item->get(2).asDouble();
        br.y=(int)item->get(3).asDouble();

        cog.x=(tl.x+br.x)>>1;
        cog.y=(tl.y+br.y)>>1;
    }

    return cog;
}


/**********************************************************/
bool Manager::get3DPosition(const CvPoint &point, Vector &x)
{
    if (rpcGet3D.getOutputCount()>0)
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("get"));
        cmd.addVocab(Vocab::encode("s2c"));
        Bottle &options=cmd.addList();
        options.addString(camera.c_str());
        options.addInt(point.x);
        options.addInt(point.y);
        printf("Sending get3D query: %s\n",cmd.toString().c_str());
        rpcGet3D.write(cmd,reply);
        printf("Received blob cartesian coordinates: %s\n",reply.toString().c_str());

        if (reply.size()>0)
        {
            if (Bottle *pInfo=reply.get(0).asList())
            {
                if (pInfo->size()>=3)
                {
                    x.resize(3);
                    x[0]=pInfo->get(0).asDouble();
                    x[1]=pInfo->get(1).asDouble();
                    x[2]=pInfo->get(2).asDouble();
                    return true;
                }
            }
        }
    }

    return false;
}


/**********************************************************/
void Manager::acquireImage(const bool rtlocalization)
{
    // grab resources
    mutexResources.wait();

    // wait for incoming image
    if (ImageOf<PixelBgr> *tmp=imgIn.read())
    {
        if (rtlocalization)
            imgRtLoc=*tmp;
        else
            img=*tmp;
    }
    
    // release resources
    mutexResources.post();
}


/**********************************************************/
void Manager::drawBlobs(const Bottle &blobs, const int i,
                        Bottle *scores)
{
    // grab resources
    mutexResources.wait();

    Port *port;
    if (scores==NULL)
        port=&imgOut;
    else
        port=&imgRtLocOut;

    if (port->getOutputCount()>0)
    {
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);

        ImageOf<PixelBgr> img;  // latch image
        if (scores==NULL)
            img=this->img;
        else
            img=this->imgRtLoc;

        for (int j=0; j<blobs.size(); j++)
        {
            CvPoint tl,br,txtLoc;
            Bottle *item=blobs.get(j).asList();
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();
            txtLoc.x=tl.x;
            txtLoc.y=tl.y-5;

            ostringstream tag;
            tag<<"blob_"<<j;

            if (scores!=NULL)
            {
                // find the blob name (or unknown)
                string object=db.findName(*scores,tag.str());
                tag.str("");
                tag.clear();
                tag<<object;
            }

            cvRectangle(img.getIplImage(),tl,br,(j==i)?cvScalar(0,0,255):cvScalar(0,255,0),2);
            cvPutText(img.getIplImage(),tag.str().c_str(),txtLoc,&font,cvScalar(0,255,0));
        }

        port->write(img);
    }

    // release resources
    mutexResources.post();
}


/**********************************************************/
int Manager::findClosestBlob(const Bottle &blobs, const CvPoint &loc)
{
    int ret=RET_INVALID;
    double min_d2=1e9;

    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        double dx=loc.x-cog.x;
        double dy=loc.y-cog.y;
        double d2=dx*dx+dy*dy;

        if (d2<min_d2)
        {
            min_d2=d2;
            ret=i;
        }
    }

    return ret;
}


/**********************************************************/
Bottle Manager::classify(const Bottle &blobs, const bool rtlocalization)
{
    // grab resources
    mutexResources.wait();

    if (rtlocalization)
        imgClassifier.write(imgRtLoc);
    else
        imgClassifier.write(img);

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("classify"));
    Bottle &options=cmd.addList();
    for (int i=0; i<blobs.size(); i++)
    {
        ostringstream tag;
        tag<<"blob_"<<i;
        Bottle &item=options.addList();
        item.addString(tag.str().c_str());
        item.addList()=*blobs.get(i).asList();
    }
    printf("Sending classification request: %s\n",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());

    // release resources
    mutexResources.post();

    return reply;
}


/**********************************************************/
void Manager::burst(const string &tag)
{
    if (trainBurst && (tag!=""))
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("burst"));
        cmd.addVocab(Vocab::encode(tag.c_str()));

        printf("Sending burst training request: %s\n",cmd.toString().c_str());
        rpcClassifier.write(cmd,reply);
        printf("Received reply: %s\n",reply.toString().c_str());
    }
}


/**********************************************************/
void Manager::train(const string &object, const Bottle &blobs,
                    const int i)
{
    // grab resources
    mutexResources.wait();

    imgClassifier.write(img);

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("train"));
    Bottle &options=cmd.addList().addList();
    options.addString(object.c_str());

    if (i<0)
    {
        Bottle &noBlob=options.addList();
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
    }
    else
        options.add(blobs.get(i));

    printf("Sending training request: %s\n",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());

    if (trainOnFlipped && (i>=0))
    {
        ImageOf<PixelBgr> imgFlipped=img;

        if (Bottle *item=blobs.get(i).asList())
        {
            CvPoint tl,br;
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();

            cvSetImageROI((IplImage*)imgFlipped.getIplImage(),cvRect(tl.x,tl.y,br.x-tl.x,br.y-tl.y));
            cvFlip(imgFlipped.getIplImage(),imgFlipped.getIplImage(),1);
            cvResetImageROI((IplImage*)imgFlipped.getIplImage());

            imgClassifier.write(imgFlipped);

            printf("Sending training request (for flipped image): %s\n",cmd.toString().c_str());
            rpcClassifier.write(cmd,reply);
            printf("Received reply (for flipped image): %s\n",reply.toString().c_str());
        }
    }

    // release resources
    mutexResources.post();
}


/**********************************************************/
void Manager::improve_train(const string &object, const Bottle &blobs,
                            const int i)
{
    CvPoint ref_cog=getBlobCOG(blobs,i);
    if ((ref_cog.x==RET_INVALID) || (ref_cog.y==RET_INVALID))
        return;

    double t0=Time::now();
    while (Time::now()-t0<improve_train_period)
    {
        // acquire image for training
        acquireImage();

        // grab the blobs
        Bottle blobs=getBlobs();

        // failure handling
        if (blobs.size()==0)
            continue;

        // enforce 2D consistency
        int exploredBlob=-1;
        double curMinDist=10.0;
        double curMinDist2=curMinDist*curMinDist;
        for (int i=0; i<blobs.size(); i++)
        {
            CvPoint cog=getBlobCOG(blobs,i);
            if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                continue;

            double dx=ref_cog.x-cog.x;
            double dy=ref_cog.y-cog.y;
            double dist2=dx*dx+dy*dy;
            if (dist2<curMinDist2)
            {
                exploredBlob=i;
                curMinDist2=dist2;
            }
        }

        // no candidate found => skip
        if (exploredBlob<0)
            continue;

        // train the classifier
        train(object,blobs,exploredBlob);

        // draw the blobs highlighting the explored one
        drawBlobs(blobs,exploredBlob);
    }
}


/**********************************************************/
void Manager::home(const string &part)
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("home"));
    cmdMotor.addString(part.c_str());
    rpcMotor.write(cmdMotor,replyMotor);
}


/**********************************************************/
void Manager::calibTable()
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("calib"));
    cmdMotor.addVocab(Vocab::encode("table"));
    rpcMotor.write(cmdMotor,replyMotor);
}


/**********************************************************/
bool Manager::calibKinStart(const string &object, const string &hand,
                            const int recogBlob)
{
    Bottle replyHuman;
    bool ret=false;

    // some known object has been recognized
    if (recogBlob>=0)
    {
        deque<string> param;
        param.push_back(hand);
        param.push_back("still");

        if (interruptableAction("touch",&param,object))
        {            
            Bottle cmdMotor,replyMotor;
            cmdMotor.addVocab(Vocab::encode("calib"));
            cmdMotor.addVocab(Vocab::encode("kinematics"));
            cmdMotor.addString("start");
            cmdMotor.addString(hand.c_str());
            rpcMotor.write(cmdMotor,replyMotor);

            objectToBeKinCalibrated=object;
            speaker.speak("Ok, now teach me the correct position");
            replyHuman.addString("ack");
            ret=true;
        }
        else
        {
            speaker.speak("I might be wrong");
            replyHuman.addString("nack");
        }
    }
    // no known object has been recognized in the scene
    else
    {
        ostringstream reply;
        reply<<"I am sorry, I cannot see any "<<object;
        reply<<" around. Should I try again?";
        speaker.speak(reply.str());
        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);
    return ret;
}


/**********************************************************/
void Manager::calibKinStop()
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("calib"));
    cmdMotor.addVocab(Vocab::encode("kinematics"));
    cmdMotor.addString("stop");
    cmdMotor.addString(objectToBeKinCalibrated.c_str());
    rpcMotor.write(cmdMotor,replyMotor);

    speaker.speak("Thanks for the help");
    home();
}


/**********************************************************/
void Manager::_motor(const string &cmd, const string &object)
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode(cmd.c_str()));
    cmdMotor.addString(object.c_str());
    rpcMotor.write(cmdMotor,replyMotor);

    if (cmd=="point")
    {
        cmdMotor.clear();
        cmdMotor.addVocab(Vocab::encode("home"));
        cmdMotor.addString("hands");
        rpcMotor.write(cmdMotor,replyMotor);
    }
}


/**********************************************************/
void Manager::_motor(const string &cmd, const Bottle &blobs,
                     const int i)
{
    CvPoint cog=getBlobCOG(blobs,i);
    if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
        return;

    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode(cmd.c_str()));
    Bottle &options=cmdMotor.addList();
    options.addString(camera.c_str());
    options.addInt(cog.x);
    options.addInt(cog.y);
    rpcMotor.write(cmdMotor,replyMotor);

    if (cmd=="point")
    {
        cmdMotor.clear();
        cmdMotor.addVocab(Vocab::encode("home"));
        cmdMotor.addString("hands");
        rpcMotor.write(cmdMotor,replyMotor);
    }
}


/**********************************************************/
bool Manager::interruptableAction(const string &action,
                                  deque<string> *param,
                                  const string &object)
{
    // remap "hold" into "take" without final "drop"
    string actionRemapped=action;
    if (actionRemapped=="hold")
        actionRemapped="take";

    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode(actionRemapped.c_str()));
    if (action=="drop")
        cmdMotor.addString("over");
    cmdMotor.addString(object.c_str());
    if (action=="drop")
        cmdMotor.addString("gently");

    if (param!=NULL)
        for (size_t i=0; i<param->size(); i++)
            cmdMotor.addString((*param)[i].c_str());

    actionInterrupted=false;
    enableInterrupt=true;
    rpcMotor.write(cmdMotor,replyMotor);

    // this switch might be turned on asynchronously
    // by a request received on a dedicated port
    if (actionInterrupted)
    {
        reinstateMotor();
        home();
    }
    // drop the object in the hand
    else if ((action=="take") && (replyMotor.get(0).asVocab()==Vocab::encode("ack")))
    {
        cmdMotor.clear();
        cmdMotor.addVocab(Vocab::encode("drop"));
        rpcMotor.write(cmdMotor,replyMotor);
    }

    enableInterrupt=false;
    return !actionInterrupted;
}


/**********************************************************/
void Manager::interruptMotor()
{
    if (enableInterrupt)
    {
        actionInterrupted=true;  // keep this line before the call to write
        enableInterrupt=false;
        Bottle cmdMotorStop,replyMotorStop;
        cmdMotorStop.addVocab(Vocab::encode("interrupt"));
        rpcMotorStop.write(cmdMotorStop,replyMotorStop);

        speaker.speak("Ouch!");
    }
}


/**********************************************************/
void Manager::reinstateMotor(const bool saySorry)
{        
    Bottle cmdMotorStop,replyMotorStop;
    cmdMotorStop.addVocab(Vocab::encode("reinstate"));
    rpcMotorStop.write(cmdMotorStop,replyMotorStop);

    if (saySorry)
        speaker.speak("Sorry");
}


/**********************************************************/
void Manager::point(const string &object)
{
    _motor("point",object);
}


/**********************************************************/
void Manager::look(const string &object)
{
    _motor("look",object);
}


/**********************************************************/
void Manager::point(const Bottle &blobs, const int i)
{
    _motor("point",blobs,i);
}


/**********************************************************/
void Manager::look(const Bottle &blobs, const int i)
{
    _motor("look",blobs,i);
}


/**********************************************************/
int Manager::recognize(const string &object, Bottle &blobs,
                       Classifier **ppClassifier)
{
    map<string,Classifier*>::iterator it=db.find(object);
    if (it==db.end())
    {
        // if not, create a brand new one
        db[object]=new Classifier(object,classification_threshold);
        it=db.find(object);
        printf("created classifier for %s\n",object.c_str());
    }

    // acquire image for classification/training
    acquireImage();

    // grab the blobs
    blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
        return RET_INVALID;

    // get the scores from the learning machine
    Bottle scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            return RET_INVALID;
        }
    }

    // find the best blob
    int recogBlob=db.processScores(it->second,scores);

    // draw the blobs highlighting the recognized one (if any)
    drawBlobs(blobs,recogBlob);

    // prepare output
    if (ppClassifier!=NULL)
        *ppClassifier=it->second;

    return recogBlob;
}


/**********************************************************/
int Manager::recognize(Bottle &blobs, Bottle &scores, string &object)
{
    object=OBJECT_UNKNOWN;

    // acquire image for classification/training
    acquireImage();

    // grab the blobs
    blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
        return RET_INVALID;

    // get the scores from the learning machine
    scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            return RET_INVALID;
        }
    }

    // handle the human-pointed object
    if (whatGood)
    {
        int closestBlob=findClosestBlob(blobs,whatLocation);
        drawBlobs(blobs,closestBlob);
        look(blobs,closestBlob);

        ostringstream tag;
        tag<<"blob_"<<closestBlob;
        object=db.findName(scores,tag.str());
        return closestBlob;
    }
    else
    {
        speaker.speak("Ooops! Sorry, I missed where you pointed at");
        return RET_INVALID;
    }
}


/**********************************************************/
void Manager::execName(const string &object)
{
    Bottle replyHuman;
    if (!trackStopGood)
    {
        speaker.speak("Ooops! Sorry, I missed where you pointed at");
        replyHuman.addString("nack");
        rpcHuman.reply(replyHuman);
        return;
    }

    map<string,Classifier*>::iterator it=db.find(object);
    if (it==db.end())
    {
        // if not, create a brand new one
        db[object]=new Classifier(object,classification_threshold);
        it=db.find(object);
        printf("created classifier for %s\n",object.c_str());
    }

    // acquire image for training
    acquireImage();

    // grab the blobs
    Bottle blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
    {
        speaker.speak("Ooops! Sorry, I cannot see any object");
        replyHuman.addString("nack");
        rpcHuman.reply(replyHuman);
        return;
    }

    // run the normal procedure straightaway
    Bottle scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            replyHuman.addString("nack");
            rpcHuman.reply(replyHuman);
            return;
        }
    }

    db.processScores(it->second,scores);

    // find the closest blob
    int closestBlob=findClosestBlob(blobs,trackStopLocation);

    // draw the blobs highlighting the detected one (if any)
    drawBlobs(blobs,closestBlob);

    // train
    burst("start");
    train(object,blobs,closestBlob);
    improve_train(object,blobs,closestBlob);
    burst("stop");
    ostringstream reply;
    reply<<"All right! Now I know what a "<<object;
    reply<<" is";
    speaker.speak(reply.str());
    look(blobs,closestBlob);

    replyHuman.addString("ack");
    rpcHuman.reply(replyHuman);
}


/**********************************************************/
void Manager::execForget(const string &object)
{
    Bottle cmdClassifier,replyClassifier,replyHuman;

    // grab resources
    mutexResources.wait();

    // forget the whole memory
    if (object=="all")
    {
        cmdClassifier.addVocab(Vocab::encode("forget"));
        cmdClassifier.addString("all");
        printf("Sending clearing request: %s\n",cmdClassifier.toString().c_str());
        rpcClassifier.write(cmdClassifier,replyClassifier);
        printf("Received reply: %s\n",replyClassifier.toString().c_str());

        // clear the memory too
        if (rpcMemory.getOutputCount()>0)
        {
            mutexResourcesMemory.wait();
            for (map<string,int>::iterator id=memoryIds.begin(); id!=memoryIds.end(); id++)
            {
                Bottle cmdMemory,replyMemory;
                cmdMemory.addVocab(Vocab::encode("del"));
                Bottle &bid=cmdMemory.addList().addList();
                bid.addString("id");
                bid.addInt(id->second);
                rpcMemory.write(cmdMemory,replyMemory);
            }
            memoryIds.clear();
            mutexResourcesMemory.post();
        }

        db.clear();
        speaker.speak("I have forgotten everything");
        replyHuman.addString("ack");
    }
    else    // forget specific object
    {
        ostringstream reply;
        map<string,Classifier*>::iterator it=db.find(object);
        if (it!=db.end())
        {
            cmdClassifier.addVocab(Vocab::encode("forget"));
            cmdClassifier.addString(object.c_str());
            printf("Sending clearing request: %s\n",cmdClassifier.toString().c_str());
            rpcClassifier.write(cmdClassifier,replyClassifier);
            printf("Received reply: %s\n",replyClassifier.toString().c_str());

            // remove the item from the memory too
            if (rpcMemory.getOutputCount()>0)
            {
                mutexResourcesMemory.wait();
                map<string,int>::iterator id=memoryIds.find(object);
                map<string,int>::iterator memoryIdsEnd=memoryIds.end();
                mutexResourcesMemory.post();

                if (id!=memoryIdsEnd)
                {
                    Bottle cmdMemory,replyMemory;
                    cmdMemory.addVocab(Vocab::encode("del"));
                    Bottle &bid=cmdMemory.addList().addList();
                    bid.addString("id");
                    bid.addInt(id->second);
                    rpcMemory.write(cmdMemory,replyMemory);

                    mutexResourcesMemory.wait();
                    memoryIds.erase(id);
                    mutexResourcesMemory.post();
                }
            }

            db.erase(it);
            reply<<object<<" forgotten";
            speaker.speak(reply.str());
            replyHuman.addString("ack");
        }
        else
        {
            printf("%s object is unknown\n",object.c_str());
            reply<<"I do not know any "<<object;
            speaker.speak(reply.str());
            replyHuman.addString("nack");
        }        
    }

    rpcHuman.reply(replyHuman);

    // release resources
    mutexResources.post();
}


/**********************************************************/
void Manager::execWhere(const string &object, const Bottle &blobs,
                        const int recogBlob, Classifier *pClassifier)
{
    Bottle cmdHuman,valHuman,replyHuman;

    // some known object has been recognized
    if (recogBlob>=0)
    {
        ostringstream reply;
        reply<<"I think this is the "<<object;
        speaker.speak(reply.str());
        printf("I think the %s is blob %d\n",object.c_str(),recogBlob);

        // issue a [point] and wait for action completion
        point(object);

        speaker.speak("Am I right?");

        replyHuman.addString("ack");
        replyHuman.addInt(recogBlob);
    }
    // no known object has been recognized in the scene
    else
    {
        ostringstream reply;
        reply<<"I have not found any "<<object;
        reply<<", is it so?";
        speaker.speak(reply.str());
        printf("No object recognized\n");

        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);

    // enter the human interaction mode to refine the knowledge
    bool ok=false;
    while (!ok)
    {
        replyHuman.clear();
        rpcHuman.read(cmdHuman,true);

        if (isStopping())
            return;

        int type=processHumanCmd(cmdHuman,valHuman);
        // do nothing
        if (type==Vocab::encode("skip"))
        {
            speaker.speak("Skipped");
            replyHuman.addString("ack");
            ok=true;
        }
        // good job is done
        else if (type==Vocab::encode("ack"))
        {
            // reinforce if an object is present
            if ((recogBlob>=0) && (pClassifier!=NULL))
            {
                burst("start");
                train(object,blobs,recogBlob);
                improve_train(object,blobs,recogBlob);
                burst("stop");
                pClassifier->positive();
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Cool!");
            replyHuman.addString("ack");
            ok=true;
        }
        // misrecognition
        else if (type==Vocab::encode("nack"))
        {
            // update the threshold if an object is present
            if ((recogBlob>=0) && (pClassifier!=NULL))
            {
                pClassifier->negative();
                updateClassifierInMemory(pClassifier);
            }

            // handle the human-pointed object
            CvPoint loc;
            if (pointedLoc.getLoc(loc))
            {
                int closestBlob=findClosestBlob(blobs,loc);
                burst("start");
                train(object,blobs,closestBlob);
                improve_train(object,blobs,closestBlob);
                burst("stop");
                speaker.speak("Oooh, I see");
                look(blobs,closestBlob);
            }
            else
                speaker.speak("Ooops! Sorry, I missed where you pointed at");

            replyHuman.addString("ack");
            ok=true;
        }
        else
        {
            speaker.speak("Hmmm hmmm hmmm! Try again");
            replyHuman.addString("nack");
        }

        rpcHuman.reply(replyHuman);
    }
}


/**********************************************************/
void Manager::execWhat(const Bottle &blobs, const int pointedBlob,
                       const Bottle &scores, const string &object)
{
    Bottle cmdHuman,valHuman,replyHuman;
    Bottle &_scores=const_cast<Bottle&>(scores);
    Classifier *pClassifier=NULL;

    // some known object has been recognized
    if (object!=OBJECT_UNKNOWN)
    {
        ostringstream reply;
        reply<<"I think it is the "<<object;
        speaker.speak(reply.str());
        speaker.speak("Am I right?");
        printf("I think the blob %d is the %s\n",pointedBlob,object.c_str());        

        // retrieve the corresponding classifier
        map<string,Classifier*>::iterator it=db.find(object);
        if (it!=db.end())
            pClassifier=it->second;

        replyHuman.addString("ack");
        replyHuman.addString(object.c_str());
    }
    // no known object has been recognized in the scene
    else
    {
        speaker.speak("I do not know this object");
        speaker.speak("What is it?");
        printf("No object recognized\n");
        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);

    // enter the human interaction mode to refine the knowledge
    bool ok=false;
    while (!ok)
    {
        replyHuman.clear();
        rpcHuman.read(cmdHuman,true);

        if (isStopping())
            return;

        int type=processHumanCmd(cmdHuman,valHuman);
        // do nothing
        if (type==Vocab::encode("skip"))
        {
            speaker.speak("Skipped");
            replyHuman.addString("ack");
            ok=true;
        }
        // good job is done
        else if ((object!=OBJECT_UNKNOWN) && (type==Vocab::encode("ack")))
        {
            // reinforce if an object is present
            if ((pointedBlob>=0) && (pClassifier!=NULL))
            {
                burst("start");
                train(object,blobs,pointedBlob);
                improve_train(object,blobs,pointedBlob);
                burst("stop");
                db.processScores(pClassifier,_scores);
                pClassifier->positive();
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Cool!");
            replyHuman.addString("ack");
            ok=true;
        }
        // misrecognition
        else if (type==Vocab::encode("nack"))
        {
            // update the threshold
            if ((pointedBlob>=0) && (pClassifier!=NULL))
            {
                db.processScores(pClassifier,_scores);
                pClassifier->negative();
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Sorry");
            replyHuman.addString("ack");
            ok=true;
        }
        // handle new/unrecognized/misrecognized object
        else if ((type==Vocab::encode("name")) && (valHuman.size()>0))
        {
            string objectName=valHuman.get(0).asString().c_str();

            // check whether the object is already known
            // and, if not, allocate space for it
            map<string,Classifier*>::iterator it=db.find(objectName);
            if (it==db.end())
            {
                db[objectName]=new Classifier(objectName,classification_threshold);
                it=db.find(objectName);
                speaker.speak("Oooh, I see");
                printf("created classifier for %s\n",objectName.c_str());
            }
            else
            {
                // update the threshold for the case of misrecognition
                if ((pClassifier!=NULL) && (object!=objectName) && (object!=OBJECT_UNKNOWN))
                {
                    db.processScores(pClassifier,_scores);
                    pClassifier->negative();
                    updateClassifierInMemory(pClassifier);
                }

                ostringstream reply;
                reply<<"Sorry, I should have recognized the "<<objectName;
                speaker.speak(reply.str());
            }

            // trigger the classifier
            if (pointedBlob>=0)
            {
                burst("start");
                train(objectName,blobs,pointedBlob);
                improve_train(objectName,blobs,pointedBlob);
                burst("stop");
            }

            db.processScores(it->second,_scores);

            replyHuman.addString("ack");
            ok=true;
        }
        else
        {
            speaker.speak("Hmmm hmmm hmmm! Try again");
            replyHuman.addString("nack");
        }

        rpcHuman.reply(replyHuman);
    }
}


/**********************************************************/
void Manager::execExplore(const string &object)
{
    Bottle cmdMotor,replyMotor,replyHuman;
    Vector position;

    if (get3DPositionFromMemory(object,position))
    {        
        cmdMotor.addVocab(Vocab::encode("look"));
        cmdMotor.addString(object.c_str());
        cmdMotor.addString("fixate");
        rpcMotor.write(cmdMotor,replyMotor);

        if (replyMotor.get(0).asVocab()==Vocab::encode("ack"))
        {
            ostringstream reply;
            reply<<"I will explore the "<<object;
            speaker.speak(reply.str());

            exploration.setInfo(object,position);

            burst("start");
            exploration.start();

            cmdMotor.clear();
            cmdMotor.addVocab(Vocab::encode("explore"));
            cmdMotor.addVocab(Vocab::encode("torso"));
            rpcMotor.write(cmdMotor,replyMotor);
            
            exploration.stop();
            do Time::delay(0.1);
            while (exploration.isRunning());
            burst("stop");

            home();

            cmdMotor.clear();
            cmdMotor.addVocab(Vocab::encode("idle"));
            rpcMotor.write(cmdMotor,replyMotor);
            speaker.speak("I'm done");

            replyHuman.addString("ack");
        }
        else
        {        
            speaker.speak("Sorry, something went wrong with the exploration");
            replyHuman.addString("nack");
        }
    }
    else
    {
        speaker.speak("Sorry, something went wrong with the exploration");
        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);
}


/**********************************************************/
void Manager::execInterruptableAction(const string &action,
                                      const string &object,
                                      const int recogBlob)
{
    Bottle replyHuman;

    // the object has been recognized
    if (recogBlob>=0)
    {
        ostringstream reply;
        reply<<"Ok, I will "<<action;
        if (action=="drop")
            reply<<" over ";
        reply<<" the "<<object;
        speaker.speak(reply.str());
        printf("I think the %s is blob %d\n",object.c_str(),recogBlob);

        // issue the action and wait for action completion/interruption
        if (interruptableAction(action,NULL,object))
        {
            replyHuman.addString("ack");
            replyHuman.addInt(recogBlob);
        }
        else
            replyHuman.addString("nack");
    }
    // drop straightaway what's in the hand
    else if ((action=="drop") && (object==""))
    {
        speaker.speak("Ok");

        Bottle cmdMotor,replyMotor;
        cmdMotor.addVocab(Vocab::encode("drop"));
        actionInterrupted=false;
        enableInterrupt=true;
        rpcMotor.write(cmdMotor,replyMotor);

        if (replyMotor.get(0).asVocab()==Vocab::encode("nack"))
        {
            speaker.speak("I have nothing in my hands");
            replyHuman.addString("nack");
        }
        else if (actionInterrupted)
        {
            reinstateMotor();
            home();
            replyHuman.addString("nack");
        }
        else
            replyHuman.addString("ack");

        enableInterrupt=false;
    }
    // no object has been recognized in the scene
    else
    {
        ostringstream reply;
        reply<<"I am sorry, I cannot see any "<<object;
        reply<<" around";
        speaker.speak(reply.str());

        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);
}


/**********************************************************/
void Manager::switchAttention()
{
    // skip if connection with motor interface is not in place
    if (rpcMotor.getOutputCount()>0)
    {
        mutexAttention.wait();

        // grab the blobs
        Bottle blobs=getBlobs();
        for (int i=0; i<blobs.size(); i++)
        {
            // make a guess
            int guess=(int)Rand::scalar(0.0,blobs.size());
            if (guess>=blobs.size())
                guess=blobs.size()-1;

            CvPoint cog=getBlobCOG(blobs,guess);
            if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                continue;

            look(blobs,guess);
            mutexAttention.post();
            return;
        }

        // if no good blob found go home
        home("gaze");
        mutexAttention.post();
    }
}


/**********************************************************/
void Manager::doLocalization()
{
    // acquire image for classification/training
    acquireImage(true);
    // grab the blobs
    Bottle blobs=getBlobs();
    // get the scores from the learning machine
    Bottle scores=classify(blobs,true);
    // draw the blobs
    drawBlobs(blobs,RET_INVALID,&scores);

    // data for memory update
    mutexResourcesMemory.wait();
    memoryBlobs=blobs;
    memoryScores=scores;
    mutexResourcesMemory.post();
}


/**********************************************************/
bool Manager::get3DPositionFromMemory(const string &object,
                                      Vector &position)
{
    bool ret=false;
    if (rpcMemory.getOutputCount()>0)
    {
        // grab resources
        mutexMemoryUpdate.wait();

        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(object);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        if (id!=memoryIdsEnd)
        {
            // get the relevant properties
            // [get] (("id" <num>) ("propSet" ("position_3d")))
            Bottle cmdMemory,replyMemory;
            cmdMemory.addVocab(Vocab::encode("get"));
            Bottle &content=cmdMemory.addList();
            Bottle &list_bid=content.addList();
            list_bid.addString("id");
            list_bid.addInt(id->second);
            Bottle &list_propSet=content.addList();
            list_propSet.addString("propSet");
            Bottle &list_items=list_propSet.addList();
            list_items.addString("position_3d");
            rpcMemory.write(cmdMemory,replyMemory);

            // retrieve 3D position
            if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *propField=replyMemory.get(1).asList())
                {
                    if (propField->check("position_3d"))
                    {
                        if (Bottle *pPos=propField->find("position_3d").asList())
                        {
                            if (pPos->size()>=3)
                            {
                                position.resize(3);
                                position[0]=pPos->get(0).asDouble();
                                position[1]=pPos->get(1).asDouble();
                                position[2]=pPos->get(2).asDouble();
                                ret=true;
                            }
                        }
                    }
                }
            }
        }

        // release resources
        mutexMemoryUpdate.post();
    }

    return ret;
}


/**********************************************************/
void Manager::doExploration(const string &object,
                            const Vector &position)
{
    // acquire image for training
    acquireImage();

    // grab the blobs
    Bottle blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
        return;

    // enforce 3D consistency
    int exploredBlob=-1;
    double curMinDist=0.05;
    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        Vector x;
        if (get3DPosition(cog,x))
        {
            double dist=norm(position-x);
            if (dist<curMinDist)
            {
                exploredBlob=i;
                curMinDist=dist;
            }
        }
    }

    // no candidate found => skip
    if (exploredBlob<0)
        return;

    // train the classifier
    train(object,blobs,exploredBlob);

    // draw the blobs highlighting the explored one
    drawBlobs(blobs,exploredBlob);
}


/**********************************************************/
void Manager::updateMemory()
{
    if (rpcMemory.getOutputCount()>0)
    {
        // grab resources
        mutexMemoryUpdate.wait();

        // load memory on connection event
        if (scheduleLoadMemory)
        {
            loadMemory();
            scheduleLoadMemory=false;
        }

        mutexResourcesMemory.wait();
        Bottle blobs=memoryBlobs;
        Bottle scores=memoryScores;
        mutexResourcesMemory.post();

        for (int j=0; j<blobs.size(); j++)
        {
            ostringstream tag;
            tag<<"blob_"<<j;

            // find the blob name (or unknown)
            mutexResources.wait();
            string object=db.findName(scores,tag.str());
            mutexResources.post();

            if (object!=OBJECT_UNKNOWN)
            {
                CvPoint cog=getBlobCOG(blobs,j);
                if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                    continue;

                // find 3d position
                Vector pos;
                if (get3DPosition(cog,pos))
                {
                    Bottle *item=blobs.get(j).asList();
                    if (item==NULL)
                        continue;

                    // prepare position_2d property
                    Bottle position_2d;
                    Bottle &list_2d=position_2d.addList();
                    list_2d.addString(("position_2d_"+camera).c_str());
                    Bottle &list_2d_c=list_2d.addList();
                    list_2d_c.addDouble(item->get(0).asDouble());
                    list_2d_c.addDouble(item->get(1).asDouble());
                    list_2d_c.addDouble(item->get(2).asDouble());
                    list_2d_c.addDouble(item->get(3).asDouble());

                    // prepare position_3d property
                    Bottle position_3d;
                    Bottle &list_3d=position_3d.addList();
                    list_3d.addString("position_3d");
                    Bottle &list_3d_c=list_3d.addList();
                    list_3d_c.addDouble(pos[0]);
                    list_3d_c.addDouble(pos[1]);
                    list_3d_c.addDouble(pos[2]);

                    mutexResourcesMemory.wait();
                    map<string,int>::iterator id=memoryIds.find(object);
                    map<string,int>::iterator memoryIdsEnd=memoryIds.end();
                    mutexResourcesMemory.post();

                    Bottle cmdMemory,replyMemory;
                    if (id==memoryIdsEnd)      // the object was not present => [add]
                    {
                        cmdMemory.addVocab(Vocab::encode("add"));
                        Bottle &content=cmdMemory.addList();
                        Bottle &list_entity=content.addList();
                        list_entity.addString("entity");
                        list_entity.addString("object");
                        Bottle &list_name=content.addList();
                        list_name.addString("name");
                        list_name.addString(object.c_str());
                        content.append(position_2d);
                        content.append(position_3d);
                        rpcMemory.write(cmdMemory,replyMemory);

                        if (replyMemory.size()>1)
                        {
                            // store the id for later usage
                            if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
                            {
                                if (Bottle *idField=replyMemory.get(1).asList())
                                {
                                    mutexResourcesMemory.wait();
                                    memoryIds[object]=idField->get(1).asInt();
                                    mutexResourcesMemory.post();
                                }
                                else
                                    continue;
                            }
                        }
                    }
                    else    // the object is already present => [set]
                    {
                        // prepare id property
                        Bottle bid;
                        Bottle &list_bid=bid.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id->second);

                        cmdMemory.addVocab(Vocab::encode("set"));
                        Bottle &content=cmdMemory.addList();
                        content.append(bid);
                        content.append(position_2d);
                        content.append(position_3d);
                        rpcMemory.write(cmdMemory,replyMemory);
                    }
                }
            }
        }

        // release resources
        mutexMemoryUpdate.post();
    }
}


/**********************************************************/
void Manager::updateClassifierInMemory(Classifier *pClassifier)
{
    if ((rpcMemory.getOutputCount()>0) && (pClassifier!=NULL))
    {
        string objectName=pClassifier->getName();

        // prepare classifier_thresholds property
        Bottle classifier_property;
        Bottle &list_classifier=classifier_property.addList();
        list_classifier.addString("classifier_thresholds");
        list_classifier.addList().append(pClassifier->toBottle());

        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(objectName);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        Bottle cmdMemory,replyMemory;
        if (id==memoryIdsEnd)      // the object was not present => [add]
        {
            cmdMemory.addVocab(Vocab::encode("add"));
            Bottle &content=cmdMemory.addList();
            Bottle &list_entity=content.addList();
            list_entity.addString("entity");
            list_entity.addString("object");
            Bottle &list_name=content.addList();
            list_name.addString("name");
            list_name.addString(objectName.c_str());
            content.append(classifier_property);
            rpcMemory.write(cmdMemory,replyMemory);

            if (replyMemory.size()>1)
            {
                // store the id for later usage
                if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idField=replyMemory.get(1).asList())
                    {
                        mutexResourcesMemory.wait();
                        memoryIds[objectName]=idField->get(1).asInt();
                        mutexResourcesMemory.post();
                    }
                }
            }
        }
        else    // the object is already present => [set]
        {
            // prepare id property
            Bottle bid;
            Bottle &list_bid=bid.addList();
            list_bid.addString("id");
            list_bid.addInt(id->second);

            cmdMemory.addVocab(Vocab::encode("set"));
            Bottle &content=cmdMemory.addList();
            content.append(bid);
            content.append(classifier_property);
            rpcMemory.write(cmdMemory,replyMemory);
        }
    }
}


/**********************************************************/
void Manager::updateObjCartPosInMemory(const string &object,
                                       const Bottle &blobs,
                                       const int i)
{
    if ((rpcMemory.getOutputCount()>0) && (i!=RET_INVALID) && (i<blobs.size()))
    {
        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(object);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        Bottle *item=blobs.get(i).asList();
        if ((id!=memoryIdsEnd) && (item!=NULL))
        {
            CvPoint cog=getBlobCOG(blobs,i);
            if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                return;

            Vector pos;
            if (get3DPosition(cog,pos))
            {
                Bottle cmdMemory,replyMemory;

                // prepare id property
                Bottle bid;
                Bottle &list_bid=bid.addList();
                list_bid.addString("id");
                list_bid.addInt(id->second);

                // prepare position_2d property
                Bottle position_2d;
                Bottle &list_2d=position_2d.addList();
                list_2d.addString(("position_2d_"+camera).c_str());
                Bottle &list_2d_c=list_2d.addList();
                list_2d_c.addDouble(item->get(0).asDouble());
                list_2d_c.addDouble(item->get(1).asDouble());
                list_2d_c.addDouble(item->get(2).asDouble());
                list_2d_c.addDouble(item->get(3).asDouble());

                // prepare position_3d property
                Bottle position_3d;
                Bottle &list_3d=position_3d.addList();
                list_3d.addString("position_3d");
                Bottle &list_3d_c=list_3d.addList();
                list_3d_c.addDouble(pos[0]);
                list_3d_c.addDouble(pos[1]);
                list_3d_c.addDouble(pos[2]);

                cmdMemory.addVocab(Vocab::encode("set"));
                Bottle &content=cmdMemory.addList();
                content.append(bid);
                content.append(position_2d);
                content.append(position_3d);
                rpcMemory.write(cmdMemory,replyMemory);
            }
        }
    }
}


/**********************************************************/
void Manager::loadMemory()
{
    printf("Loading memory ...\n");
    // grab resources
    mutexResourcesMemory.wait();

    // purge internal databases
    memoryIds.clear();
    db.clear();

    // ask for all the items stored in memory
    Bottle cmdMemory,replyMemory,replyMemoryProp;
    cmdMemory.addVocab(Vocab::encode("ask"));
    Bottle &content=cmdMemory.addList().addList();
    content.addString("entity");
    content.addString("==");
    content.addString("object");
    rpcMemory.write(cmdMemory,replyMemory);
    
    if (replyMemory.size()>1)
    {
        if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *idField=replyMemory.get(1).asList())
            {
                if (Bottle *idValues=idField->get(1).asList())
                {
                    // cycle over items
                    for (int i=0; i<idValues->size(); i++)
                    {
                        int id=idValues->get(i).asInt();

                        // get the relevant properties
                        // [get] (("id" <num>) ("propSet" ("name" "classifier_thresholds")))
                        cmdMemory.clear();
                        cmdMemory.addVocab(Vocab::encode("get"));
                        Bottle &content=cmdMemory.addList();
                        Bottle &list_bid=content.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id);
                        Bottle &list_propSet=content.addList();
                        list_propSet.addString("propSet");
                        Bottle &list_items=list_propSet.addList();
                        list_items.addString("name");
                        list_items.addString("classifier_thresholds");
                        rpcMemory.write(cmdMemory,replyMemoryProp);

                        // update internal databases
                        if (replyMemoryProp.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *propField=replyMemoryProp.get(1).asList())
                            {
                                if (propField->check("name"))
                                {
                                    string object=propField->find("name").asString().c_str();
                                    memoryIds[object]=id;

                                    if (propField->check("classifier_thresholds"))
                                        db[object]=new Classifier(*propField->find("classifier_thresholds").asList());
                                    else
                                        db[object]=new Classifier(object,classification_threshold);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    printf("Objects in memory: %d\n",(int)db.size());
    for (map<string,Classifier*>::iterator it=db.begin(); it!=db.end(); it++)
    {
        string object=it->first;
        string properties=it->second->toBottle().toString().c_str();
        printf("classifier for %s: memory_id=%d; properties=%s\n",
               object.c_str(),memoryIds[object],properties.c_str());
    }

    // release resources
    mutexResourcesMemory.post();
    printf("Memory loaded\n");
}


/**********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    name=rf.check("name",Value("iolStateMachineHandler")).asString().c_str();
    camera=rf.check("camera",Value("left")).asString().c_str();
    if ((camera!="left") && (camera!="right"))
        camera="left";

    imgIn.open(("/"+name+"/img:i").c_str());
    blobExtractor.open(("/"+name+"/blobs:i").c_str());
    imgOut.open(("/"+name+"/img:o").c_str());
    imgRtLocOut.open(("/"+name+"/imgLoc:o").c_str());
    imgClassifier.open(("/"+name+"/imgClassifier:o").c_str());

    rpcHuman.open(("/"+name+"/human:rpc").c_str());
    rpcClassifier.open(("/"+name+"/classify:rpc").c_str());
    rpcMotor.open(("/"+name+"/motor:rpc").c_str());
    rpcGet3D.open(("/"+name+"/get3d:rpc").c_str());
    rpcMotorStop.open(("/"+name+"/motor_stop:rpc").c_str());
    rxMotorStop.open(("/"+name+"/motor_stop:i").c_str());
    rxMotorStop.setManager(this);

    pointedLoc.open(("/"+name+"/point:i").c_str());
    speaker.open(("/"+name+"/speak:o").c_str());

    memoryReporter.setManager(this);
    rpcMemory.setReporter(memoryReporter);
    rpcMemory.open(("/"+name+"/memory:rpc").c_str());

    skim_blobs_x_bounds.resize(2);
    skim_blobs_x_bounds[0]=-0.50;
    skim_blobs_x_bounds[1]=-0.10;
    if (rf.check("skim_blobs_x_bounds"))
    {
        if (Bottle *bounds=rf.find("skim_blobs_x_bounds").asList())
        {
            if (bounds->size()>=2)
            {
                skim_blobs_x_bounds[0]=bounds->get(0).asDouble();
                skim_blobs_x_bounds[1]=bounds->get(1).asDouble();
            }
        }
    }

    skim_blobs_y_bounds.resize(2);
    skim_blobs_y_bounds[0]=-0.30;
    skim_blobs_y_bounds[1]=+0.30;
    if (rf.check("skim_blobs_y_bounds"))
    {
        if (Bottle *bounds=rf.find("skim_blobs_y_bounds").asList())
        {
            if (bounds->size()>=2)
            {
                skim_blobs_y_bounds[0]=bounds->get(0).asDouble();
                skim_blobs_y_bounds[1]=bounds->get(1).asDouble();
            }
        }
    }

    attention.setManager(this);
    attention.start();

    rtLocalization.setManager(this);
    rtLocalization.setRate(rf.check("rt_localization_period",Value(30)).asInt());
    rtLocalization.start();

    exploration.setRate(rf.check("exploration_period",Value(30)).asInt());
    exploration.setManager(this);

    memoryUpdater.setManager(this);
    memoryUpdater.setRate(rf.check("memory_update_period",Value(60)).asInt());
    memoryUpdater.start();
    
    improve_train_period=rf.check("improve_train_period",Value(0.0)).asDouble();
    trainOnFlipped=rf.check("train_flipped_images",Value("off")).asString()=="on";
    trainBurst=rf.check("train_burst_images",Value("off")).asString()=="on";
    classification_threshold=rf.check("classification_threshold",Value(0.5)).asDouble();

    img.resize(320,240);
    imgRtLoc.resize(320,240);
    img.zero();
    imgRtLoc.zero();

    Rand::init();

    scheduleLoadMemory=false;
    enableInterrupt=false;
    trackStopGood=false;
    whatGood=false;
    skipGazeHoming=false;

    objectToBeKinCalibrated="";

    return true;
}


/**********************************************************/
bool Manager::interruptModule()
{
    imgIn.interrupt();
    imgOut.interrupt();
    imgRtLocOut.interrupt();
    imgClassifier.interrupt();
    rpcHuman.interrupt();
    blobExtractor.interrupt();
    rpcClassifier.interrupt();
    rpcMotor.interrupt();
    rpcGet3D.interrupt();
    rpcMotorStop.interrupt();
    rxMotorStop.interrupt();
    pointedLoc.interrupt();
    speaker.interrupt();
    rpcMemory.interrupt();

    rtLocalization.stop();
    memoryUpdater.stop();
    attention.stop();

    return true;
}


/**********************************************************/
bool Manager::close()
{
    imgIn.close();
    imgOut.close();
    imgRtLocOut.close();
    imgClassifier.close();
    rpcHuman.close();
    blobExtractor.close();
    rpcClassifier.close();
    rpcMotor.close();
    rpcGet3D.close();
    rpcMotorStop.close();
    rxMotorStop.close();
    pointedLoc.close();
    speaker.close();
    rpcMemory.close();

    return true;
}


/**********************************************************/
bool Manager::updateModule()
{
    Bottle cmdHuman,valHuman,replyHuman;
    rpcHuman.read(cmdHuman,true);

    if (isStopping())
        return false;

    attention.suspend();

    if (!skipGazeHoming)
    {
        home("gaze");

        // this wait-state gives the memory
        // time to be updated with the 3D
        // location of the objects
        Time::delay(0.1);
    }

    skipGazeHoming=false;

    int rxCmd=processHumanCmd(cmdHuman,valHuman);
    if (rxCmd==Vocab::encode("home"))
    {
        reinstateMotor(false);
        home();
        replyHuman.addString("ack");
        rpcHuman.reply(replyHuman);
    }
    else if (rxCmd==Vocab::encode("cata"))
    {
        calibTable();
        replyHuman.addString("ack");
        rpcHuman.reply(replyHuman);
    }
    else if (rxCmd==Vocab::encode("caki"))
    {
        string type=valHuman.get(0).asString().c_str();
        if (type=="start")
        {
            Bottle blobs;
            string hand=cmdHuman.get(2).toString().c_str();
            string activeObject=cmdHuman.get(3).toString().c_str();
            
            mutexMemoryUpdate.wait();
            int recogBlob=recognize(activeObject,blobs);
            updateObjCartPosInMemory(activeObject,blobs,recogBlob);
            if (calibKinStart(activeObject,hand,recogBlob))
                return true;    // avoid resuming the attention
            else
                mutexMemoryUpdate.post();
        }
        else
        {
            calibKinStop();
            mutexMemoryUpdate.post();
            replyHuman.addString("ack");
            rpcHuman.reply(replyHuman);
        }
    }
    else if (rxCmd==Vocab::encode("track"))
    {
        Bottle cmdMotor,replyMotor;
        string type=valHuman.get(0).asString().c_str();
        if (type=="start")
        {
            cmdMotor.addVocab(Vocab::encode("track"));
            cmdMotor.addVocab(Vocab::encode("motion"));
            cmdMotor.addString("no_sacc");
            rpcMotor.write(cmdMotor,replyMotor);
            speaker.speak("Great! Show me the new toy");
            trackStopGood=false;
        }
        else
        {
            cmdMotor.addVocab(Vocab::encode("idle"));
            rpcMotor.write(cmdMotor,replyMotor);

            // avoid being distracted by the human hand
            // while it is being removed: save the last
            // pointed object
            trackStopGood=pointedLoc.getLoc(trackStopLocation);
        }

        replyHuman.addString("ack");
        rpcHuman.reply(replyHuman);
        skipGazeHoming=true;
        return true;    // avoid resuming the attention
    }
    else if (rxCmd==Vocab::encode("name"))
    {        
        string activeObject=valHuman.get(0).asString().c_str();
        execName(activeObject);
    }
    else if (rxCmd==Vocab::encode("forget"))
    {        
        string activeObject=valHuman.get(0).asString().c_str();

        mutexMemoryUpdate.wait();
        execForget(activeObject);
        mutexMemoryUpdate.post();
    }
    else if (rxCmd==Vocab::encode("where"))
    {        
        Bottle blobs;
        Classifier *pClassifier;
        string activeObject=valHuman.get(0).asString().c_str();

        mutexMemoryUpdate.wait();
        int recogBlob=recognize(activeObject,blobs,&pClassifier);
        updateObjCartPosInMemory(activeObject,blobs,recogBlob);
        execWhere(activeObject,blobs,recogBlob,pClassifier);
        mutexMemoryUpdate.post();
    }
    else if (rxCmd==Vocab::encode("what"))
    {
        // avoid being distracted by the human hand
        // while it is being removed: save the last
        // pointed object
        whatGood=pointedLoc.getLoc(whatLocation);
        Time::delay(1.0);

        Bottle blobs,scores;
        string activeObject;
        int pointedBlob=recognize(blobs,scores,activeObject);
        execWhat(blobs,pointedBlob,scores,activeObject);
    }
    else if ((rxCmd==Vocab::encode("take")) || (rxCmd==Vocab::encode("touch")) ||
             (rxCmd==Vocab::encode("push")) || (rxCmd==Vocab::encode("hold"))  ||
             (rxCmd==Vocab::encode("drop")))
    {        
        Bottle blobs;
        string activeObject="";
        int recogBlob=RET_INVALID;

        mutexMemoryUpdate.wait();
        if (valHuman.size()>0)
        {
            activeObject=valHuman.get(0).asString().c_str();
            recogBlob=recognize(activeObject,blobs);
            updateObjCartPosInMemory(activeObject,blobs,recogBlob);
        }

        string action;
        if (rxCmd==Vocab::encode("take"))
            action="take";
        else if (rxCmd==Vocab::encode("touch"))
            action="touch";
        else if (rxCmd==Vocab::encode("push"))
            action="push";
        else if (rxCmd==Vocab::encode("hold"))
            action="hold";
        else
            action="drop";

        execInterruptableAction(action,activeObject,recogBlob);
        mutexMemoryUpdate.post();
    }
    else if (rxCmd==Vocab::encode("explore"))
    {
        string activeObject=valHuman.get(0).asString().c_str();
        execExplore(activeObject);
    }
    else if (rxCmd==Vocab::encode("attention"))
    {
        string type=valHuman.get(0).asString().c_str();
        if (type=="stop")
        {
            replyHuman.addString("ack");
            rpcHuman.reply(replyHuman);
            return true;    // avoid resuming the attention
        }            
        else if (type=="start")
            replyHuman.addString("ack");
        else
            replyHuman.addString("nack");

        rpcHuman.reply(replyHuman);
    }
    else if (rxCmd==Vocab::encode("say"))
    {
        string speech=valHuman.get(0).asString().c_str();
        speaker.speak(speech);
        replyHuman.addString("ack");
        rpcHuman.reply(replyHuman);
        return true; // avoid resuming the attention
    }
    else    // manage an unknown request
    {
        speaker.speak("I don't understand what you want me to do");
        replyHuman.addString("nack");
        rpcHuman.reply(replyHuman);
    }

    attention.resume();
    return true;
}


/**********************************************************/
double Manager::getPeriod()
{
    return 0.1;
}



