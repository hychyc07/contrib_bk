/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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
#include <iostream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

//#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RET_INVALID     -1

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_EXECUTE             VOCAB4('e','x','e','c')
#define CMD_TOOLEXTEND          VOCAB4('e','x','t','d')

/**********************************************************/
bool Manager::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}

bool Manager::train(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand  )
{
    std::cout << "Training!" <<std::endl; fflush(stdout);
    this->action=action;
    this->obj=target;
    
    Vector cog;
    bool ok = opcPort.get2DPositionFromMemory(target,cog);
    if (!ok)
        return false;
    std::cout << "Obj found at" << cog[0] << ", " << cog[1] << std::endl; fflush(stdout);
    /*    
    *   segment the object
    */
    segmentAndTrack(cog[0], cog[1]);  
   // std::cout << "Segmented and tracked!" <<std::endl; fflush(stdout);
  //  Vector initPos;
    if (get3DPosition(cog,initPos))
    {
        //3d position
        fprintf(stdout,"The 3D position is: X:%lf X:%lf X:%lf\n",initPos[0], initPos[1], initPos[2]);   
//        Bottle cmdAre, reply;
//        cmdAre.addString("teach"); //or addVocab ?
//        cmdAre.addString(action.c_str());
//        cmdAre.addString("start");
        if (hand=="left")
            return dmpExecutor.teach_start(action.c_str(), iCub::LEFT);
        else
            return dmpExecutor.teach_start(action.c_str(), iCub::RIGHT);


//          cmdAre.addString("left");
//        else cmdAre.addString("right");
 
       // wbdRecalibration();

//        rpcMotorAre.write(cmdAre, reply);
//        if (reply.get(0).asVocab()==VOCAB4('n','a','c','k'))
//            return RET_INVALID;
//        return true;

    }
    else return false; 
}
bool Manager::stop_training()
{
    Bottle cmdAre, reply;
    cmdAre.addString("teach"); //or addVocab ?
    cmdAre.addString("stop");
    cmdAre.addString(action.c_str());
    rpcMotorAre.write(cmdAre, reply);
        std::cout << "Are reply:" << reply.toString()<<std::endl; fflush(stdout);
    if (reply.get(0).asVocab()==VOCAB4('n','a','c','k') || !reply.check("id") )
        return false;
    else
    {
        int id=reply.find("id").asInt();
        std::cout << "id of recorded action: "<< id <<std::endl;
        
        
        opcPort.addActionTarget(id, initPos);
        //do all the DMP processing here! 
        //return dmpLearner.estimate_DMP(id);
        if (!dmpLearner.estimate_DMP(id))
            std::cout << "WARNING! DMP encoding was not successful!!" <<std::endl;
        
        cmdAre.clear();
        cmdAre.addString("home"); //or addVocab ?
        cmdAre.addString("head");
        rpcMotorAre.write(cmdAre, reply);
        
        return true;
    }    
}
bool Manager::test(const std::string& action, const std::string& target, const std::string& tool,  const std::string& hand)
{
    //executeOnLoc(false);
    //get target position... ask Vadim how
  //  std::cout << observe_state();
    Vector targetPosition, targetPosition3D;
    bool ok = opcPort.get2DPositionFromMemory(target,targetPosition);
    get3DPosition(targetPosition,targetPosition3D);
    if (!ok)
    {
        std::cout << "Couldn't retreive object from memory!" << std::endl;
        return false;
    }
    int newId=opcPort.createActionTarget(action, targetPosition3D);
    if (!dmpLearner.generalize_DMP(newId))
    {
        std::cout << "Couldn't generalize action!" << std::endl;
        return false;
    }
    
    if (!dmpExecutor.is_running())
        dmpExecutor.run();
    ok= ok && dmpExecutor.execute_OPC(newId);
    ok= ok && dmpExecutor.waitMotionDone(getPeriod(), 0);
    dmpExecutor.stop();

    std::cout << observe_state();
    return ok;
}
std::string Manager::observe_state()
{
    return std::string("not implemented yet");
}
void Manager::go_home()
{
    goHome();
}

/**********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    name=rf.find("name").asString().c_str();
    camera=rf.find("camera").asString().c_str();
    if ((camera!="left") && (camera!="right"))
        camera="left";

    hand=rf.find("hand").asString().c_str();
    if ((hand!="left") && (hand!="right"))
        hand="left";

    //incoming
    particleFilter.open(("/"+name+"/particle:i").c_str());
    pointedLoc.open(("/"+name+"/point:i").c_str());
    blobExtractor.open(("/"+name+"/blobs:i").c_str());
   

    //outgoing
    segmentPoint.open(("/"+name+"/segmentTarget:o").c_str());      //port to send off target Points to segmentator
    iolStateMachine.open(("/"+name+"/iolState:o").c_str());    

    //rpc 
    rpcMIL.open(("/"+name+"/mil:rpc").c_str());                 //rpc client to query mil classifications
    rpcHuman.open(("/"+name+"/human:rpc").c_str());             //rpc server to interact with the italkManager
    rpcMotorAre.open(("/"+name+"/are:rpc").c_str());          //rpc server to query ARE
    opcPort.open(("/"+name+"/opc:rpc").c_str());
    rpcWBD.open(("/"+name+"/wbd:rpc").c_str());
    
    //TEST rpc thrift port
    thriftPort.open(("/"+name+"/thrift:rpc").c_str());   
    attach(thriftPort);
    
    dmpLearnerPort.open(("/"+name+"/dmpLearner:rpc").c_str());
    dmpLearner.yarp().attachAsClient(dmpLearnerPort);
    
    dmpExecutorPort.open(("/"+name+"/dmpExecutor:rpc").c_str());
    dmpExecutor.yarp().attachAsClient(dmpExecutorPort);
   
    pointGood=false;
    init=false;
    //attach(rpcHuman);
    return true;
}

/**********************************************************/
bool Manager::interruptModule()
{
    segmentPoint.interrupt();
    blobExtractor.interrupt();
    rpcHuman.interrupt();
    rpcMotorAre.interrupt();
    particleFilter.interrupt();
    pointedLoc.interrupt();
    iolStateMachine.interrupt();
    rpcMIL.interrupt();
    opcPort.interrupt();
    rpcWBD.interrupt();
    thriftPort.interrupt();
    dmpLearnerPort.interrupt();
    dmpExecutorPort.interrupt();

    return true;
}
/**********************************************************/
bool Manager::close()
{
    segmentPoint.close();
    blobExtractor.close();
    rpcHuman.close();
    rpcMotorAre.close();
    particleFilter.close();
    pointedLoc.close();
    iolStateMachine.close();
    rpcMIL.close();
    opcPort.close();
    rpcWBD.close();
    thriftPort.close();
    dmpLearnerPort.close();
    dmpExecutorPort.close();

    return true;
}
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
bool Manager::updateModule()
{
    if (isStopping())
        return false;

    init = true;//bypass for now

    while (!init)
    {   
        Time::delay(0.5);
        fprintf(stdout, "waiting for connection from iolStateMachineHandler\n");
        if (iolStateMachine.getOutputCount() > 0)
        {
            fprintf(stdout, "sending home \n");
            Bottle cmdIol, replyIol;
            cmdIol.addString("home");
            iolStateMachine.write(cmdIol,replyIol);
            fprintf(stdout,"%s\n", replyIol.toString().c_str());
            fprintf(stdout,"stopping attention...\n");
            cmdIol.clear();
            replyIol.clear();
            cmdIol.addString("attention");
            cmdIol.addString("stop");
            iolStateMachine.write(cmdIol,replyIol);
            fprintf(stdout,"%s\n", replyIol.toString().c_str());
            init = true;
            fprintf(stdout, "have successfully initialized it all\n");
        }
    }

    Bottle cmd, val, reply;
    rpcHuman.read(cmd, true);
    if (cmd != NULL)
    {
        int rxCmd=processHumanCmd(cmd,val);
        if (rxCmd==Vocab::encode("train"))
        {
            obj=cmd.get(1).asString().c_str();
            action=cmd.get(2).asString().c_str();
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            //executeOnLoc(true);
            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
        if (rxCmd==Vocab::encode("test"))
        {
            obj=cmd.get(1).asString().c_str();
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            //executeOnLoc(false);
            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
    }
    Bottle result;
    result.clear();

    return true;
}
/**********************************************************/
void Manager::wbdRecalibration()
{
    Bottle cmdWBD,replyWBD;
    cmdWBD.addInt(0);
    printf("Sending recalibration request to WBD\n");
    rpcWBD.write(cmdWBD,replyWBD);
    printf("Received reply from WBD: %s\n",replyWBD.toString().c_str());
}
/**********************************************************/
void Manager::segmentAndTrack( int x, int y )
{
    Bottle toSegment;
    toSegment.clear();
    
    toSegment.addInt(x); //(closestBlob.get(0).asInt());
    toSegment.addInt(y); //(closestBlob.get(1).asInt());
    toSegment.addInt(80);
    toSegment.addInt(80);
    fprintf(stdout, "segmenting cmd is %s\n",toSegment.toString().c_str());
    segmentPoint.write(toSegment);
    
    Bottle cmdAre, replyAre;
    cmdAre.addString("track");
    cmdAre.addString("track");
    cmdAre.addString("no_sacc");
    rpcMotorAre.write(cmdAre,replyAre);
    fprintf(stdout,"tracking started%s:\n",replyAre.toString().c_str());
}
/**********************************************************/
void Manager::goHomeArmsHead()
{
    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("home");
    cmdAre.addString("arms");
    cmdAre.addString("head");
    rpcMotorAre.write(cmdAre,replyAre);
    fprintf(stdout,"gone home %s:\n",replyAre.toString().c_str()); 
}
/**********************************************************/
void Manager::goHome()
{
    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("home");
    cmdAre.addString("all");
    rpcMotorAre.write(cmdAre,replyAre);
    fprintf(stdout,"gone home %s:\n",replyAre.toString().c_str()); 
}


/**********************************************************/
Bottle Manager::classify(const Bottle &blobs, int index)
{
    //grab resources
    mutexResources.wait();
    Bottle mils;
    mils.clear();

    Bottle gotMils;
    gotMils.clear();

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("classify"));

    Bottle &options=cmd.addList();
    ostringstream tag;
    tag<<"blob_"<<index;
    Bottle &item=options.addList();
    item.addString(tag.str().c_str());
    item.addList()=*blobs.get(index).asList();

    //printf("Sending classification request: %s\n",cmd.toString().c_str());
    rpcMIL.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());
    mutexResources.post();
    //Bottle &toReturn = gotMils.addList();
    
    if (reply!=NULL)
    {
        CvPoint cog;
        cog=getBlobCOG(blobs,index);
        Bottle &tmpLine = gotMils.addList();
        Bottle &tmpMils = tmpLine.addList();
        tmpMils.addDouble(cog.x);
        tmpMils.addDouble(cog.y);
        tmpLine.addList()=*reply.get(0).asList()->get(1).asList();
    }
    mils.clear();
    mils.addList()=gotMils;
    //release resources
    return mils;
}
/**********************************************************/
Bottle Manager::getType(const yarp::os::Bottle *scores, int index)
{
    ostringstream tag;
    tag<<"blob_"<<index;
    if (scores!=NULL)
    {
        double max_score=0.0;
        string max_label="unknown";
        if (scores->get(0).asList()->size() > 0)
        {
            Bottle *tmp_scores=scores->get(0).asList()->get(0).asList()->get(1).asList();
            
            //fprintf(stdout,"bottle is: %s \n",tmp_scores->toString().c_str());
            
            for(int s=0; s<tmp_scores->size(); s++)
            {
                if(tmp_scores->get(s).asList()->get(1).asDouble()>max_score)
                {
                    max_score=tmp_scores->get(s).asList()->get(1).asDouble();
                    max_label=tmp_scores->get(s).asList()->get(0).asString().c_str();
                }
            }
        }
        //fprintf(stdout, "the prob is %lf ", scores->get(0).asList()->get(j).asList()->get(1).asList()->get(1).asDouble());
        //fprintf(stdout,"\n");
        tag.str("");
        tag.clear();
        tag<<max_label;
        Bottle type;
        type.clear();
        type.addString(max_label.c_str());
        return type;
    }
    Bottle type;
    type.clear();
    type.addString("error");
    return type;
}
/**********************************************************/
double Manager::getPeriod()
{
    return 0.1;
}

/**********************************************************/
bool Manager::get3DPosition(Vector &point, Vector &x)
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("get"));
    cmdMotor.addVocab(Vocab::encode("s2c"));
    Bottle &options=cmdMotor.addList();
    options.addString(camera.c_str());
    options.addInt(point[0]);
    options.addInt(point[1]);
    printf("Sending motor query: %s\n",cmdMotor.toString().c_str());
    rpcMotorAre.write(cmdMotor,replyMotor);
    printf("Received blob cartesian coordinates: %s\n",replyMotor.toString().c_str());

    if (replyMotor.size()>=3)
    {   
        x.resize(3);
        x[0]=replyMotor.get(0).asDouble();
        x[1]=replyMotor.get(1).asDouble();
        x[2]=replyMotor.get(2).asDouble();
        return true;
    }
    else
        return false;
}
/**********************************************************/
Bottle Manager::getBlobs()
{
    // grab resources
    mutexResources.wait();

    if (Bottle *pBlobs=blobExtractor.read(false))
    {
        lastBlobs=*pBlobs;
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
Bottle Manager::findClosestBlob(const Bottle &blobs, const CvPoint &loc)
{
    int ret=RET_INVALID;
    double min_d2=1e9;
    Bottle pointReturn;
    pointReturn.clear();
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
    CvPoint cog=getBlobCOG( blobs, ret );
    pointReturn.addDouble(cog.x);           //cog x
    pointReturn.addDouble(cog.y);           //cog y
    pointReturn.addInt(ret);                //index of blob
    Bottle *item=blobs.get(ret).asList();
    double orient = (double)item->get(4).asDouble();
    int axe1 = (int)item->get(5).asInt();
    int axe2 = (int)item->get(6).asInt();
    pointReturn.addDouble(orient);
    pointReturn.addInt(axe1);
    pointReturn.addInt(axe2);
    return pointReturn;
}


/**********************************************************/
void Manager::quit()
{
    stopModule();
}
