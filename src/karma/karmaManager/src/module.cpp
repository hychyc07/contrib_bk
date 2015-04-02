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
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"
#include <yarp/dev/IPositionControl.h>
#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RET_INVALID     -1

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_EXECUTE             VOCAB4('e','x','e','c')
#define CMD_TOOLEXTEND          VOCAB4('e','x','t','d')

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
    rpcMotorKarma.open(("/"+name+"/karma:rpc").c_str());        //rpc server to query Karma
    rpcKarmaLearn.open(("/"+name+"/learn:rpc").c_str());
    rpcReconstruct.open(("/"+name+"/reconstruct:rpc").c_str());
    rpcGraspEstimate.open(("/"+name+"/graspestimate:rpc").c_str());
    rpcOPC.open(("/"+name+"/opc:rpc").c_str());

    Rand::init();

    randActions[0] = 0.0;
    randActions[1] = 30.0;
    randActions[2] = 150.0;
    randActions[3] = 180.0;
    randActions[4] = 225.0;
    //randActions[5] = 270.0;
    randActions[5] = 315.0;
    
    pointGood=false;
    init=false;
    
    // open cartesiancontrollerclient and gazecontrollerclient drivers
    Property optCartLeftArm("(device cartesiancontrollerclient)");
    Property optCartRightArm("(device cartesiancontrollerclient)");

    optCartLeftArm.put("remote",("/icub/cartesianController/left_arm"));
    optCartLeftArm.put("local",("/"+name+"/left_arm/cartesian").c_str());

    optCartRightArm.put("remote",("/icub/cartesianController/right_arm"));
    optCartRightArm.put("local",("/"+name+"/right_arm/cartesian").c_str());

    //drvCartLeftArm=new PolyDriver;
    if (!drvCartLeftArm.open(optCartLeftArm))
    {
        close();
        return false;
    }

    //drvCartRightArm=new PolyDriver;
    if (!drvCartRightArm.open(optCartRightArm))
    {
        close();
        return false;
    }

  
    drvCartLeftArm.view(cartArm);
    cartArm->storeContext(&startup_context_id_left);
    cartArm->restoreContext(0);
    
    drvCartRightArm.view(cartArm);
    cartArm->storeContext(&startup_context_id_right);
    cartArm->restoreContext(0);


    idleTmo = 60.0;
    toolSmall.resize(3);
    toolBig.resize(3);
    
    toolSmall[0] =  0.18;
    toolSmall[1] = -0.17;
    toolSmall[2] = -0.04;

    toolBig[0] =  0.29;
    toolBig[1] = -0.26;
    toolBig[2] = -0.05;

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
    rpcMotorKarma.interrupt();
    particleFilter.interrupt();
    pointedLoc.interrupt();
    iolStateMachine.interrupt();
    rpcMIL.interrupt();
    rpcKarmaLearn.interrupt();
    rpcReconstruct.interrupt();
    rpcGraspEstimate.interrupt();
    rpcOPC.interrupt();

    return true;
}
/**********************************************************/
bool Manager::close()
{

    cartArm->restoreContext(startup_context_id_left);
    cartArm->restoreContext(startup_context_id_right);
    
    cartArm->stopControl();
    drvCartLeftArm.close();
    drvCartRightArm.close();

    segmentPoint.close();
    blobExtractor.close();
    rpcHuman.close();
    rpcMotorAre.close();
    rpcMotorKarma.close();
    particleFilter.close();
    pointedLoc.close();
    iolStateMachine.close();
    rpcMIL.close();
    rpcKarmaLearn.close();
    rpcReconstruct.close();
    rpcGraspEstimate.close();
    rpcOPC.close();

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

    //init = true;//bypass for now

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
            fprintf(stdout, "have successfully initialize it all\n");

            executeSpeech ("ok I am ready");
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
            
            if (cmd.size() > 2)
            {
                fprintf(stdout,"with user data\n");
                userTheta=cmd.get(2).asDouble();
            }
            else 
            {
               fprintf(stdout,"no user data\n");
               userTheta = -1.0;
            }
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            executeOnLoc(true);
            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
        if (rxCmd==Vocab::encode("test"))
        {
            obj=cmd.get(1).asString().c_str();
            
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            executeOnLoc(false);
            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
        if (rxCmd==Vocab::encode("handle"))
        {
            takeMotionARE();
            reply.addString("ack");
            rpcHuman.reply(reply);
        }

        if (rxCmd==Vocab::encode("extend"))
        {
            if (cmd.size() > 2)
            {
                fprintf(stdout, "Will now use user selected arm and camera \n");
                hand   = cmd.get(1).asString().c_str();
                camera = cmd.get(2).asString().c_str();
            }
            else
            {
                fprintf(stdout, "Will now use default arm and cam \n");
            }

            lastTool.clear();
            lastTool = executeToolLearning();
            reply.addString("ack");
            rpcHuman.reply(reply);

            fprintf(stdout, "GOT TOOL at  %s, \n",lastTool.toString().c_str());
            goHomeArmsHead();
        }
        if (rxCmd==Vocab::encode("tooltip"))
        {
            Bottle toSend;
            toSend.clear();
            toSend.addDouble(lastTool.get(0).asDouble());
            toSend.addDouble(lastTool.get(1).asDouble());
            toSend.addDouble(lastTool.get(2).asDouble());
            //reply.addString("ack");
            rpcHuman.reply(toSend);

            fprintf(stdout, "Replied with  %s, \n",toSend.toString().c_str());
            //goHomeArmsHead();
        }
        if (rxCmd==Vocab::encode("grasp"))
        {
            obj=cmd.get(1).asString().c_str();
            if ( executePCLGrasp(obj) )
                reply.addString("ack");
            else 
                reply.addString("nack");
        }
        if (rxCmd==Vocab::encode("blobLoc"))
        {
            blobLoc.clear();
            blobLoc = findBlobLoc();
            if (blobLoc.size()<1)
            {
                reply.addString("nack");
                rpcHuman.reply(reply);
            }            
            else
                rpcHuman.reply(blobLoc);
        }

        if (rxCmd==Vocab::encode("objRecog"))
        {
            blobList.clear();
            obj=cmd.get(1).asString().c_str();
            blobLoc = executeBlobRecog(obj);

            if (blobLoc.size()<1)
            {
                reply.addString("nack");
                rpcHuman.reply(reply);
            }            
            else
                rpcHuman.reply(blobLoc);
        }
        if (rxCmd==Vocab::encode("close"))
        {
            int arm=cmd.get(1).asInt();
            executeCloseHand(arm);
            reply.addString("ack");
            rpcHuman.reply(reply);
        }
        if (rxCmd==Vocab::encode("give"))
        {
            int arm=cmd.get(1).asInt();
            executeGiveAction(arm);
            reply.addString("ack");
            rpcHuman.reply(reply);
        }
        if (rxCmd==Vocab::encode("class"))
        {
            reply = classifyThem();
            rpcHuman.reply(reply);
        }
    }
    Bottle result;
    result.clear();

    return true;
}

/**********************************************************/
Bottle Manager::classifyThem()
{
    Bottle cmdIol;
    Bottle replyIol;
    cmdIol.clear(), replyIol.clear();
    cmdIol.addString("class");
    iolStateMachine.write(cmdIol,replyIol);
    fprintf(stdout,"%s\n", replyIol.toString().c_str());

    return replyIol;
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
void Manager::takeMotionARE()
{
    //to fill with data for karmaMotor
    fprintf(stdout,"Will now start take motion proceedure:\n");
    Bottle cmdAre,replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("take");
    cmdAre.addString("motion");
    fprintf(stdout,"%s\n",cmdAre.toString().c_str());
    rpcMotorAre.write(cmdAre, replyAre);
    fprintf(stdout,"action is %s:\n",replyAre.toString().c_str());
}

/**********************************************************/
Bottle Manager::executeToolLearning()
{
    //to fill with data for karmaMotor
    fprintf(stdout,"Will now start the tool learn proceedure:\n");
    Bottle karmaMotor,KarmaReply;
    karmaMotor.clear();
    KarmaReply.clear();
    karmaMotor.addString("find");
    karmaMotor.addString(hand.c_str());
    karmaMotor.addString(camera.c_str());
    fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
    rpcMotorKarma.write(karmaMotor, KarmaReply);
    fprintf(stdout,"action is %s:\n",KarmaReply.toString().c_str());

    Bottle toReturn;
    toReturn.clear();
    toReturn.addDouble( KarmaReply.get(1).asDouble() );
    toReturn.addDouble( KarmaReply.get(2).asDouble() );
    toReturn.addDouble( KarmaReply.get(3).asDouble() );
    return toReturn;
}
/**********************************************************/
Bottle Manager::executeBlobRecog(const string &objName)
{
    Bottle loc;
    loc.clear();
    fprintf(stdout, "\n\n\nexecuteBlobRecog****************************************************************************\n\n" );

    Bottle blobs;
    blobs.clear();
    bool invalid    = false;
    bool isGrasped  = false;

    // grab the blobs
    blobs=getBlobs();
    // failure handling
    if (blobs.size()==0)
    {
        fprintf (stdout,"INVALID BLOB SIZE\n");   
        invalid = true;
    }
    latchTimer = 0.0;

    if ( !invalid )
    {
        Bottle classResults = classifyThem();
        
        for (int x=0; x < blobs.size(); x++)
        {
            pointLocation = getBlobCOG(blobs,x);
            fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
            Bottle closestBlob;
            mutexResources.wait();
            closestBlob=findClosestBlob(blobs,pointLocation);
            mutexResources.post();

            CvPoint cog;
            cog.x = closestBlob.get(0).asInt();
            cog.y = closestBlob.get(1).asInt();
            
            int index = 0;
            index = closestBlob.get(2).asInt();

            fprintf(stdout, "closest blob is x:%d y:%d with index %d\n\n",cog.x, cog.y, index);
        
            //classify the blob
            //Bottle mil;
            //fprintf(stdout,"ask to classify\n");
            //mil=classify(blobs, index);
            //fprintf(stdout,"done classifying\n");
            //get the type of the blob
            //Bottle type;
            //fprintf(stdout,"ask to get type\n");
            //type=getType(&mil, index);
            
            fprintf(stdout, "The type is: %s and object name requested is: %s\n\n", classResults.get(x).asString().c_str(), objName.c_str() );
            
            if (classResults.get(x).asString().c_str() == objName)
            {

                pointLocation = getBlobCOG(blobs,index);
                fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
                fprintf(stdout,"I have found the requested object %s, at: %d %d\n",objName.c_str(), pointLocation.x, pointLocation.y );
                loc.addInt (pointLocation.x);
                loc.addInt (pointLocation.y);
            }
        }
    }

    return loc;
}

/**********************************************************/
int Manager::executeToolOnLoc()
{
    fprintf(stdout, "\n\n\n****************************************************************************\n\n" );
    Bottle blobs;
    blobs.clear();
    // grab the blobs
    blobs=getBlobs();
    // failure handling
    if (blobs.size()==0)
        return RET_INVALID;

    if (blobs.size()<2)
    {
        pointLocation = getBlobCOG(blobs,0);
        fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
        pointGood = true;
    }
    else
    {
        fprintf (stdout,"I see more than two blobs\n");
        pointGood = false;
    }
    if (pointGood)
    {
        Bottle closestBlob;
        mutexResources.wait();
        closestBlob=findClosestBlob(blobs,pointLocation);
        mutexResources.post();

        CvPoint cog;
        cog.x = closestBlob.get(0).asInt();
        cog.y = closestBlob.get(1).asInt();

        /*
         *   segment the object
         */
        segmentAndTrack(cog.x, cog.y);

        int index = 0;
        index = closestBlob.get(2).asInt();
        

        fprintf(stdout, "closest blob is x:%d y:%d with index %d\n\n", cog.x, cog.y, index);
    }
    return 0;
}
/**********************************************************/
Bottle Manager::findBlobLoc()
{
    Bottle loc;
    loc.clear();
    fprintf(stdout, "\n\n\nfindBlobLoc****************************************************************************\n\n" );
    Bottle blobs;
    blobs.clear();
    bool invalid = false;
    // grab the blobs
    blobs=getBlobs();
    // failure handling
    if (blobs.size()==0)
    {        
        fprintf (stdout,"INVALID BLOB SIZE\n");   
        invalid = true;
    }

    latchTimer = 0.0;

    if (blobs.size()<2 && !invalid)
    {
        pointLocation = getBlobCOG(blobs,0);
        fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
        pointGood = true;
    }
    else
    {
        fprintf (stdout,"I either see more than two blobs or nothing at all\n");
        pointGood = false;
    }
    if (pointGood)
    {
        Bottle closestBlob;
        mutexResources.wait();
        closestBlob=findClosestBlob(blobs,pointLocation);
        mutexResources.post();

        CvPoint cog;
        cog.x = closestBlob.get(0).asInt();
        cog.y = closestBlob.get(1).asInt();

        /*
         *   segment the object
         */
        //segmentAndTrack(cog.x, cog.y);

        int index = 0;
        index = closestBlob.get(2).asInt();

        fprintf(stdout, "closest blob is x:%d y:%d with index %d\n\n", cog.x, cog.y, index);
        
        Vector initPos;
        if (get3DPosition(cog,initPos))
        {
            loc.addDouble(initPos[0]);
            loc.addDouble(initPos[1]);
            loc.addDouble(initPos[2]); 
            fprintf(stdout,"Got a 3D point at %lf %lf %lf \n", initPos[0], initPos[1],initPos[2]);
        }
    }
    return loc;
}

/**********************************************************/
bool Manager::executePCLGrasp( const string &objName )
{
    fprintf(stdout, "\n\n\nexecutePCLGrasp****************************************************************************\n\n" );
    executeSpeech ("ok, will now try to grasp the "+ objName);
    Bottle blobs;
    blobs.clear();
    bool invalid    = false;
    bool isGrasped  = false;
    CvPoint locObj; 
    Bottle result;
    result.clear();

    fprintf(stdout, "releasing head\n");
    //here put head in bind 5.0 and in idle
    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("release");
    rpcMotorAre.write(cmdAre,replyAre);
    fprintf(stdout, "the reply is: %s \n",replyAre.toString().c_str());

    Time::delay(3.0);
    // grab the blobs
    blobs=getBlobs();
    // failure handling
    if (blobs.size()==0)
    {
        invalid = true;
        return false;
    }
    latchTimer = 0.0;

    if ( !invalid )
    {    
        result = executeBlobRecog(objName);
        locObj.x = result.get(0).asInt();
        locObj.y = result.get(1).asInt();

        fprintf (stdout,"point is %d %d \n", locObj.x, locObj.y);
        pointGood = true;
    }
    else
    {
        fprintf (stdout,"I have an issue spotting the %s\n",objName.c_str());
        executeSpeech ("I can't seem to find the "+ objName);
        pointGood = false;
    }
    if (pointGood)
    {   bool close = true;
        objectPos.clear();
        if (get3DPosition(locObj,objectPos))
        {
            fprintf(stdout,"Got a 3D point at %lf %lf %lf \n", objectPos[0], objectPos[1],objectPos[2]);
            //check 3D point
            
            if (objectPos[0] < -0.47 )
            {
                executeSpeech ("I cannot safely reach for the "+ objName + ", will try to find a solution");
                fprintf(stdout, "I cannot reach for this object, will find a solution\n");
                Bottle cmdHome, cmdReply;
                cmdHome.clear();
                cmdReply.clear();
                cmdHome.addString("home");
                cmdHome.addString("all");
                rpcMotorAre.write(cmdHome,cmdReply);
                Time::delay(2.0);
                executeToolSearchOnLoc( objName );
            }
            else
            { 
                //should figure out if the 3D position is close enough to grasp or need tool to reach it
                fprintf(stdout,"Will now send the blob to graphsegmentation:\n");
                Bottle segCmd, segReply;
                segCmd.clear();
                segReply.clear();
                segCmd.addInt(locObj.x);
                segCmd.addInt(locObj.y);
                fprintf(stdout, "the cmd is: %s \n",segCmd.toString().c_str());
                rpcReconstruct.write(segCmd, segReply);
                fprintf(stdout, "the reply is: %s \n",segReply.toString().c_str());

                fprintf(stdout,"Will now ask for 3Dreconstruction:\n");
                segCmd.clear();
                segReply.clear();
                segCmd.addString("3Drec");
                segCmd.addString(objName.c_str());
                
                fprintf(stdout, "the cmd is: %s \n", segCmd.toString().c_str());
                rpcReconstruct.write(segCmd, segReply);
                fprintf(stdout, "the reply is: %s \n",segReply.toString().c_str());
                
                //should now ask if the grasp action has been accomplished
                Bottle cmd, reply;
                latchTimer=Time::now();
                
                while (!isGrasped)
                {
                    cmd.clear();
                    reply.clear();
                    cmd.addString("isGrasped");
                    fprintf(stdout, "the cmd is: %s \n", cmd.toString().c_str());
                    rpcGraspEstimate.write(cmd,reply);
                    string rep=reply.toString().c_str();
                    rep.erase (rep.begin());
                    rep.erase (rep.begin()+4);

                    if (rep == "true")
                        isGrasped = true;
                    
                    Time::delay(0.5);
                    if ((Time::now()-latchTimer)>idleTmo)
                    {
                        fprintf(stdout,"--- Timeout elapsed ---\n");
                        break;
                    }
                }

                if (isGrasped)
                {
                    fprintf(stdout, "Grasped finished\n");
                    Time::delay(3.0);
                    fprintf(stdout, "Now Releasing...\n");

                    cmd.clear();
                    reply.clear();
                    cmd.addString("release");
                    fprintf(stdout, "the cmd is: %s \n", cmd.toString().c_str());
                    rpcGraspEstimate.write(cmd,reply);
                    fprintf(stdout, "the reply is: %s \n",reply.toString().c_str());
                    Time::delay(3.0);
                }
                else
                {
                    Bottle cmd, rep;
                    cmd.clear();
                    rep.clear();
                    cmd.addString("home");
                    cmd.addString("all");
                    rpcMotorAre.write(cmd,rep);

                    Bottle cmdAre, replyAre;
                    cmdAre.clear();
                    replyAre.clear();
                    cmdAre.addString("release");
                    rpcMotorAre.write(cmdAre,replyAre);
                    fprintf(stdout, "the reply is: %s \n",replyAre.toString().c_str());
                    
                    executeSpeech ("sorry I could not figure out how to do this");
                    fprintf(stdout, "did not seem to be able to grasp correctly or has been aborted\n");
                }            
            }
        }
        fprintf(stdout, "Finished the grasping sequence \n");

        Bottle cmd, rep;
        cmd.clear();
        rep.clear();
        cmd.addString("home");
        cmd.addString("all");
        rpcMotorAre.write(cmd,rep);

    }
    return isGrasped;
}

/**********************************************************/

int Manager::executeToolSearchOnLoc( const string &objName )
{
    fprintf(stdout, "\n\n\nexecuteToolSearchOnLoc****************************************************************************\n\n" );
    Bottle blobs;
    blobs.clear();
    // grab the blobs
    blobs=getBlobs();
    // failure handling
    Bottle result;
    CvPoint objLoc;

    int objIndex = 0;

    if (blobs.size()==0)
        return RET_INVALID;

    result = executeBlobRecog(objName);
    objLoc.x = result.get(0).asInt();
    objLoc.y = result.get(1).asInt();

    blobsDetails = new blobsData[blobs.size()];

    Bottle memCmd, memRep;
    memCmd.clear(); memRep.clear();
    memCmd.addVocab(Vocab::encode("ask"));
    Bottle &tmp=memCmd.addList();
    tmp.addString ("all");
    rpcOPC.write(memCmd,memRep);

    fprintf(stdout, "THE REPLY IS %s \n",memRep.toString().c_str());

    int sizeInMem =0;

    if (memRep.get(0).asVocab()==Vocab::encode("ack"))
    {
        if (Bottle *idField = memRep.get(1).asList())
        {
            if (Bottle *idVals = idField->get(1).asList())
            {
               sizeInMem  = idVals->size();
               sizeInMem = sizeInMem - 1; //remove the table
               fprintf (stdout,"******************************************size is %d \n", sizeInMem);
            }
        }
    }

    

    bool valid = false;
    if (blobs.size() == sizeInMem)
        valid = true;
    else
        executeSpeech ("sorry Something is wrong with the objects");


    Bottle classResults = classifyThem();

    if (valid)
    {
        //figure out how many blobs are available
        for (int x=0; x < blobs.size(); x++)
        {
            
            pointLocation = getBlobCOG(blobs,x);
            fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
            fprintf (stdout,"object is %d %d \n", objLoc.x, objLoc.y);
            
            Bottle closestBlob;
            mutexResources.wait();
            closestBlob=findClosestBlob(blobs,pointLocation);
            mutexResources.post();
            
            fprintf(stdout, "checkin if objDiff x %d objDiff y %d \n",abs( objLoc.x - pointLocation.x), abs( objLoc.y - pointLocation.y));

            if ( abs( objLoc.x - pointLocation.x) < 5 &&  abs( objLoc.y - pointLocation.y) < 5)
            {
                objIndex = x;
                fprintf(stdout, "I have found the object of interest and not considering it.\n");
            }
            else
            {
                fprintf(stdout, "\n\n\n\nI AM IN SETTING UP BLOBS with blob size = %d and x= %d\n\n\n\n",blobs.size(), x);
                Bottle blobs;
                blobs.clear();
                // grab the blobs
                blobs=getBlobs();
                
                blobsDetails[x].posistion.x = (int) closestBlob.get(0).asDouble();
                blobsDetails[x].posistion.y = (int) closestBlob.get(1).asDouble();
                blobsDetails[x].index       = closestBlob.get(2).asInt();
                blobsDetails[x].lenght      = 0.0;
                blobsDetails[x].lenght      = getBlobLenght(blobs, x);
                blobsDetails[x].vdrawError  = 0.0;
                
                //Bottle mil;
                //mil.clear();
                //fprintf(stdout,"ask to classify \n ");
                //mil=classify(blobs, x);
                //fprintf(stdout,"done classifying\n");
                //Bottle type;
                //type.clear();
                //fprintf(stdout,"ask to get type\n");
                //type=getType(&mil, x);
                blobsDetails[x].name = classResults.get(x).asString().c_str();
                
                
                fprintf(stdout, "SO: name is: %s x is %d u is %d index is %d\n",blobsDetails[x].name.c_str(),blobsDetails[x].posistion.x, blobsDetails[x].posistion.y, blobsDetails[x].index);


                //should look at x y
                /*Bottle cmdAre, replyAre;
                cmdAre.clear();
                replyAre.clear();
                cmdAre.addString("look");
                Bottle &tmp=cmdAre.addList();
                tmp.addInt (blobsDetails[x].posistion.x);
                tmp.addInt (blobsDetails[x].posistion.y);
                rpcMotorAre.write(cmdAre,replyAre);
                fprintf(stdout,"looking started %s:\n",replyAre.toString().c_str());*/
            }
        }
        fprintf(stdout, "\n\n\n\nI AM OUT SETTING UP BLOBS \n\n\n\n");
        Bottle homeAfterLookCmd, homeAfterLookReply;
        homeAfterLookCmd.clear();
        homeAfterLookReply.clear();
        homeAfterLookCmd.addString("home");
        homeAfterLookCmd.addString("head");
        rpcMotorAre.write(homeAfterLookCmd,homeAfterLookReply);
        
        int     toolLenght = 1000;
        int     smallIndex = -1;
        int     bigIndex = -1;
        fprintf(stdout,"\n\n");
        for (int x=0; x < blobs.size(); x++)
        {
            if ( x!=objIndex)
            {
                fprintf(stdout,"The lenght is %lf, with index %d\n",blobsDetails[x].lenght, x);
                // figure out if using small or big tool
                if (blobsDetails[x].lenght < toolLenght )
                {
                    toolLenght=(int)blobsDetails[x].lenght;
                    smallIndex = x;
                }
            }
        }
        for (int x=0; x < blobs.size(); x++)
        {
            if ( x!=objIndex && x!=smallIndex)
                bigIndex = x;
        }

    //this needs to be changed and filled in by milClassifier

        fprintf(stdout,"The small tool is the blob index %d \n",smallIndex);
        blobsDetails[smallIndex].name = "small";

        fprintf(stdout,"The big tool is the blob index %d \n",  bigIndex);
        blobsDetails[bigIndex].name = "big";

        fprintf(stdout,"\n\n");

        Bottle small, big;

        small.clear();
        small = executeKarmaOptimize(toolSmall, blobsDetails[smallIndex].name);
        blobsDetails[smallIndex].bestAngle      = small.get(1).asDouble();
        blobsDetails[smallIndex].bestDistance   = small.get(2).asDouble();

        big.clear();
        big = executeKarmaOptimize(toolBig, blobsDetails[bigIndex].name);
        blobsDetails[bigIndex].bestAngle      = big.get(1).asDouble();
        blobsDetails[bigIndex].bestDistance   = big.get(2).asDouble();
        
        fprintf(stdout,"\n\n");

        //Attach the tool
        executeToolAttach(toolSmall);
        double virtualtmp = 0.01; 

        //setup all parameters to get the best possible configuration
        while (virtualtmp > 0.0 && virtualtmp < 0.08 )
        {
            virtualtmp = executeVirtualDraw(blobsDetails[smallIndex]);
            if (virtualtmp > 0.1)
                blobsDetails[smallIndex].bestDistance -= 0.005;
            else if (virtualtmp > 1.0)
                blobsDetails[smallIndex].bestDistance -= 0.1;
            else
                blobsDetails[smallIndex].bestDistance += 0.005;    
            
            fprintf(stdout, "the reply is: %lf \n",virtualtmp);
        }

        //do it one last time to get the correct confidence
        blobsDetails[smallIndex].vdrawError = executeVirtualDraw(blobsDetails[smallIndex]);
        fprintf (stdout, "\n\nTHE BEST ANGLE IS %lf WITH DISTANCE %lf  with confidence %lf\n\n",blobsDetails[smallIndex].bestAngle, blobsDetails[smallIndex].bestDistance, blobsDetails[smallIndex].vdrawError );

        //Attach the tool
        executeToolAttach(toolBig);
        virtualtmp = 0.01; 
        //setup all parameters to get the best possible configuration
        while (virtualtmp > 0.0 && virtualtmp < 0.08 )
        {
            virtualtmp = executeVirtualDraw(blobsDetails[bigIndex]);
            if (virtualtmp > 0.1)
                blobsDetails[bigIndex].bestDistance -= 0.005;
            else if (virtualtmp > 1.0)
                blobsDetails[bigIndex].bestDistance -= 0.1;
            else 
                blobsDetails[bigIndex].bestDistance += 0.005;

            fprintf(stdout, "the reply is: %lf \n",virtualtmp);
        }

        //do it one last time to get the correct confidence
        blobsDetails[bigIndex].vdrawError = executeVirtualDraw(blobsDetails[bigIndex]);
        fprintf (stdout, "\n\nTHE BEST ANGLE IS %lf WITH DISTANCE %lf and confidence %lf\n\n",blobsDetails[bigIndex].bestAngle, blobsDetails[bigIndex].bestDistance, blobsDetails[bigIndex].vdrawError );

        int whichArm = 0;
        Bottle cmdHome, cmdReply;
        cmdHome.clear();
        cmdReply.clear();
        cmdHome.addString("home");
        cmdHome.addString("head");
        rpcMotorAre.write(cmdHome,cmdReply);
        
        //once blobs and vdraw has been determined compare them
        double  bestChoice = 1000.0;
        int     bestIndex = -1;
        double  bestDistance = -1;
        for (int x=0; x < blobs.size(); x++) 
        {
           if (blobsDetails[x].vdrawError < bestChoice && blobsDetails[x].bestDistance > bestDistance && x != objIndex)
            //if ( blobsDetails[x].bestDistance > bestDistance && x != objIndex)
            {
                bestDistance = blobsDetails[x].bestDistance;
                //bestChoice = blobsDetails[x].vdrawError;
                bestIndex = x;
            }
        }

        //HERE DO MAXIMUM DISTANCE CHECK... -0.40 -0.33
        string tmpObjName = blobsDetails[bestIndex].name.c_str();
        executeSpeech ("can you give me the " + tmpObjName + "please?");
        
        //objectPos[0] has the oject position -0.43 is the maximum distance

        //blobsDetails[bestIndex].bestDistance = blobsDetails[bestIndex].bestDistance - 0.05;
        //figure out best distance fro subsequent grasp
                
        double max = -0.35;
        blobsDetails[bestIndex].bestDistance = fabs(objectPos[0]) - fabs( max );
        
        fprintf(stdout, "*********************************************************************************\n");
        fprintf(stdout, "THE MAX DISTANCE I WILL DO IS %lf\n,",blobsDetails[bestIndex].bestDistance);
        fprintf(stdout, "*********************************************************************************\n");

        //blobsDetails[bestIndex].bestDistance = blobsDetails[bestIndex].bestDistance - 0.01;

        Bottle cmdAre, replyAre;
        cmdAre.clear();
        replyAre.clear();
        cmdAre.addString("point");
        Bottle &tmp=cmdAre.addList();
        tmp.addInt (blobsDetails[bestIndex].posistion.x);
        tmp.addInt (blobsDetails[bestIndex].posistion.y);
        
        bool sendAction = true;
        if (blobsDetails[bestIndex].posistion.x > 0 && blobsDetails[bestIndex].posistion.x < 160 )
        {
            whichArm = 0;
            cmdAre.addString("left");
        }
        else if (blobsDetails[bestIndex].posistion.x > 160 && blobsDetails[bestIndex].posistion.x < 320 )
        {
            whichArm = 1;
            cmdAre.addString("right");
        }    
        else
        {
            executeSpeech ("oh my...I seemed to got confused...sorry");
            fprintf(stdout, "something is wrong with the action:\n");
            sendAction = false;
        }
            
        fprintf(stdout, "the cmd is: %s \n",cmdAre.toString().c_str());
        if (sendAction)
        {
            rpcMotorAre.write(cmdAre,replyAre);
            fprintf(stdout, "the reply is: %s \n",replyAre.toString().c_str());
        }
        

        if (sendAction)
        {
            executeGiveAction(whichArm);
            Time::delay(5.0);
        }   
        if (sendAction)
        {
            executeCloseHand(whichArm);
            Time::delay(5.0);
        }

        Bottle homeCmd, homeReply;
        homeCmd.clear();
        homeReply.clear();
        homeCmd.addString("home");
        homeCmd.addString("arms");
        homeCmd.addString("head");
        rpcMotorAre.write(homeCmd,homeReply);

        if (tmpObjName == "small")
            executeToolAttach(toolSmall);
        else
            executeToolAttach(toolBig);

        if (sendAction)
            executeToolDrawNear(blobsDetails[bestIndex]);

        Bottle homeToolcmd, homeToolrep;
        homeToolcmd.clear();
        homeToolrep.clear();
        homeToolcmd.addString("home");
        homeToolcmd.addString("arms");
        homeToolcmd.addString("head");
        rpcMotorAre.write(homeToolcmd,homeToolrep);

        executeSpeech ("Ok, thank you");
        
        if (sendAction)    
            executeDropAway(whichArm);
        
        if (sendAction)
            executePCLGrasp( objName );
    }

    return 0;
}
/**********************************************************/
bool Manager::executeDropAway(int ARM)
{
    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("drop");
    cmdAre.addString("away");
    if (ARM == LEFTARM)
        cmdAre.addString("left");
    else
        cmdAre.addString("right");

    rpcMotorAre.write(cmdAre,replyAre);
    
    printf("done deploying\n"); 
    return true;
}
/**********************************************************/
int Manager::executeToolAttach(const Vector &tool)
{
    fprintf(stdout,"Will now send to karmaMotor:\n");
    Bottle karmaMotor,KarmaReply;
    karmaMotor.addString("tool");
    karmaMotor.addString("remove");
    rpcMotorKarma.write(karmaMotor, KarmaReply);

    karmaMotor.clear();KarmaReply.clear();
    karmaMotor.addString("tool");
    karmaMotor.addString("attach");
    karmaMotor.addString("right");
    karmaMotor.addDouble(tool[0]);
    karmaMotor.addDouble(tool[1]);
    karmaMotor.addDouble(tool[2]);
    fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
    rpcMotorKarma.write(karmaMotor, KarmaReply);
    fprintf(stdout,"reply is %s:\n",KarmaReply.toString().c_str());
    return 1;
}
/**********************************************************/
Bottle Manager::executeKarmaOptimize(const Vector &tool, const string &name)
{
    Bottle cmdLearn, cmdReply;
    cmdLearn.clear();
    cmdReply.clear();
    cmdLearn.addString("optimize");
    cmdLearn.addString(name.c_str());
    fprintf(stdout, "the cmd is: %s \n",cmdLearn.toString().c_str());
    rpcKarmaLearn.write(cmdLearn, cmdReply);
    fprintf(stdout, "the reply is: %s \n",cmdReply.toString().c_str()); 

    double optAng = cmdReply.get(1).asDouble();
    double optDisp = cmdReply.get(2).asDouble();
            
    fprintf(stdout, "the optimum angle is %lf with optimum disp %lf\n ",optAng , optDisp);
    return cmdReply;
}
/**********************************************************/
double Manager::executeToolDrawNear(blobsData &blobsDetails)
{
    double result = 0.0;
    //for (int tools = 0; tools < blobs.size(); tools++)
    fprintf(stdout,"Will now send to karmaMotor using %s as a tool:\n", blobsDetails.name.c_str());
    Bottle karmaMotor,KarmaReply;
    karmaMotor.addString("draw");
    karmaMotor.addDouble(objectPos[0]);
    karmaMotor.addDouble(objectPos[1]);
    karmaMotor.addDouble(objectPos[2]);
    karmaMotor.addDouble(blobsDetails.bestAngle);
    karmaMotor.addDouble(0.12); //10 cm 
    karmaMotor.addDouble(blobsDetails.bestDistance);
    fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
    rpcMotorKarma.write(karmaMotor, KarmaReply);
    fprintf(stdout,"vdraw is %s:\n",KarmaReply.toString().c_str());
    result = KarmaReply.get(1).asDouble();
    return result;
}
/**********************************************************/
double Manager::executeVirtualDraw(blobsData &blobsDetails)
{
    double result = 0.0;
    //for (int tools = 0; tools < blobs.size(); tools++)
    fprintf(stdout,"Will now send to karmaMotor:\n");
    Bottle karmaMotor,KarmaReply;
    karmaMotor.addString("vdraw");
    karmaMotor.addDouble(objectPos[0]);
    karmaMotor.addDouble(objectPos[1]);
    karmaMotor.addDouble(objectPos[2]);
    karmaMotor.addDouble(blobsDetails.bestAngle);
    karmaMotor.addDouble(0.1); //10 cm 
    karmaMotor.addDouble(blobsDetails.bestDistance);
   

    fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
    rpcMotorKarma.write(karmaMotor, KarmaReply);
    fprintf(stdout,"vdraw is %s:\n",KarmaReply.toString().c_str());
    result = KarmaReply.get(1).asDouble();
    return result;
}
/**********************************************************/
bool Manager::executeGiveAction(int ARM)
{

    yarp::dev::ICartesianControl *icart=cartArm;
    Vector dof;
    string type;

    if (ARM==LEFTARM)
    {
        fprintf(stdout,"IN LEFT HAND\n"); 
        drvCartLeftArm.view(icart);
        icart->storeContext(&startup_context_id_left);
        icart->restoreContext(0);
     }
     else
     {
        fprintf(stdout,"IN RIGHT HAND\n"); 
        drvCartRightArm.view(icart);
        icart->storeContext(&startup_context_id_right);
        icart->restoreContext(0);
     }

    Vector x,o;
    x.resize(3); o.resize(4);

    if ( ARM == LEFTARM )
    {
        x[0] = -0.271048; x[1] = -0.303519; x[2] =  0.224249;
        o[0] =  0.418304; o[1] = -0.891147; o[2] =  0.175724; o[3] =  2.854429;
        fprintf(stdout,"--- GIVING WITH LEFT\n");
    }
    else if ( ARM == RIGHTARM )
    {
        x[0] = -0.212088; x[1] = 0.349181; x[2] =  0.209662;
        o[0] = 0.079137; o[1] = -0.215934; o[2] =  0.973196; o[3] =  2.358149;
        fprintf(stdout,"--- GIVING WITH RIGHT\n");
    }
    else
    {
        fprintf(stdout,"ERROR can't seem to find which arm to use\n");
    }

    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("look");
    cmdAre.addString("hand");
    if (ARM == LEFTARM)
        cmdAre.addString("left");
    else
        cmdAre.addString("right");

    rpcMotorAre.write(cmdAre,replyAre);

    Vector handPos,handOrient;
    
    icart->goToPoseSync(x,o);
    icart->getPose(handPos, handOrient);

    printf("Hand pos: (%f, %f, %f)\n", handPos[0], handPos[1], handPos[2]);
    printf("Hand orient: (%f, %f, %f, %f)\n", handOrient[0], handOrient[1], handOrient[2], handOrient[3]);
    return true;
}
/**********************************************************/
bool Manager::executeCloseHand (int ARM)
{

    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("close");
    if (ARM == LEFTARM)
        cmdAre.addString("left");
    else
        cmdAre.addString("right");

    rpcMotorAre.write(cmdAre,replyAre);
    
    printf("done closing\n"); 
    return true;
}

/**********************************************************/
bool Manager::executeSpeech (const string &speech)
{
    Bottle cmdIol;
    Bottle replyIol;
    cmdIol.clear(), replyIol.clear();
    cmdIol.addString("say");
    cmdIol.addString(speech.c_str());
    iolStateMachine.write(cmdIol,replyIol);
    fprintf(stdout,"%s\n", replyIol.toString().c_str());
    return true;
}

/**********************************************************/
int Manager::executeOnLoc(bool shouldTrain)
{
    fprintf(stdout, "\n\n\n****************************************************************************\n\n" );
    Bottle blobs;
    blobs.clear();
    // grab the blobs
    blobs=getBlobs();
    // failure handling

    if (blobs.size()==0)
        return RET_INVALID;

    if (blobs.size()<2)
    {
        pointLocation = getBlobCOG(blobs,0);
        fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
        pointGood = true;
    }
    else
    {
        fprintf (stdout,"I see more than two blobs\n");
        pointGood = false;
    }
    if (pointGood)
    {
        Bottle closestBlob;
        mutexResources.wait();
        closestBlob=findClosestBlob(blobs,pointLocation);
        mutexResources.post();
        
        CvPoint cog;
        cog.x = closestBlob.get(0).asInt();
        cog.y = closestBlob.get(1).asInt();
        
        /*
        *   segment the object
        */
        segmentAndTrack(cog.x, cog.y);
        
        int index = 0;
        index = closestBlob.get(2).asInt();

        fprintf(stdout, "closest blob is x:%d y:%d with index %d\n\n",cog.x, cog.y, index);
        
        //classify the blob
        Bottle mil;
        mil=classify(blobs, index);
        //get the type of the blob
        Bottle type;
        type=getType(&mil, index);

        double actionOrient = 0.0;
        int guessAction=(int)Rand::scalar(0,randActions.size());
        fprintf(stdout, "the guess action is %d chosen from size %d\n", guessAction, (int)randActions.size());

        double orient = 0.0;
        orient = closestBlob.get(3).asDouble();
        
        if (!shouldTrain)
        {
            fprintf(stdout,"SHOULD BE IN TEST MODE!\n");
            Bottle cmdLearn, cmdReply;
            cmdLearn.clear();
            cmdReply.clear();
            cmdLearn.addString("optimize");
            cmdLearn.addString(obj.c_str());
            /*Bottle &angles = cmdLearn.addList();
            for (int i = 0; i< randActions.size(); i++)
            {
                angles.addDouble(randActions[i]);
            }*/
            fprintf(stdout, "the cmd is: %s \n",cmdLearn.toString().c_str());
            rpcKarmaLearn.write(cmdLearn, cmdReply);
            fprintf(stdout, "the reply is: %s \n",cmdReply.toString().c_str()); 

            double optAng = cmdReply.get(1).asDouble();
            double optDisp = cmdReply.get(2).asDouble();
            
            fprintf(stdout, "the optimum angle is %lf with optimum disp %lf\n ",optAng , optDisp);
            
            Vector initPos;
            actionOrient = 0.0;
            actionOrient = optAng + orient;
            
            if (actionOrient > 360.0)
                actionOrient -= 360.0;
                
            if (actionOrient > 45.0 && actionOrient < 135.0)
                actionOrient += 180.0;
            
            fprintf(stdout, "\n\nthe FINAL angle is %lf \n\n",actionOrient);
        }
        else
        {
            fprintf(stdout,"SHOULD BE IN TRAIN MODE!\n");
            if (userTheta >= 0.0)
                actionOrient = userTheta;
            else
                actionOrient = Rand::scalar(-210.0,30.0);//randActions[guessAction];
        }
        
        Vector initPos;
        double finalOrient;
        if (get3DPosition(cog,initPos))
        {
            Bottle results;
            double offset = 0.0;
            results = getOffset(closestBlob, actionOrient, initPos);
            offset = results.get(0).asDouble();
            finalOrient = results.get(1).asDouble();
            
            fprintf(stdout,"Will now send to karmaMotor:\n");
            Bottle karmaMotor,KarmaReply;
            karmaMotor.addString("push");
            karmaMotor.addDouble(initPos[0]);
            karmaMotor.addDouble(initPos[1]);
            karmaMotor.addDouble(initPos[2] + 0.05);
            karmaMotor.addDouble(actionOrient);
            karmaMotor.addDouble( offset );// + 0.06 );

            fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
            rpcMotorKarma.write(karmaMotor, KarmaReply);
            fprintf(stdout,"action is %s:\n",KarmaReply.toString().c_str());

            CvPoint finalPoint;
            if (particleFilter.getTraker(finalPoint))
            {
                while(finalPoint.x <1 && finalPoint.x >320 && finalPoint.y <1 && finalPoint.y >240)
                {
                    fprintf(stdout, "\n\n\ndid not get a correct final point from particle filter..retrying...\n\n\n");
                    particleFilter.getTraker(finalPoint);
                }   
                Vector finalPos;
                if (get3DPosition(finalPoint,finalPos))
                {
                    fprintf(stdout,"The Final 3Dpos is %lf %lf %lf \n",finalPos[0], finalPos[1], finalPos[2]);
                   
                    //original orientation 
                    //send it all
                    double disp = 0.0;
                    disp = norm(finalPos - initPos);
                    
                    if (shouldTrain)
                    {
                        Bottle cmdLearn, replyLearn;
                        cmdLearn.clear();
                        replyLearn.clear();
                        cmdLearn.addString("train");
                        cmdLearn.addString(obj.c_str());//type.toString().c_str());
                        
                        double wrappedAng = 0.0;
                        wrappedAng = wrapAng ( finalOrient );
                        
                        cmdLearn.addDouble( wrappedAng );
                        cmdLearn.addDouble( disp );
                        
                        fprintf(stdout, "the cmd is: %s \n",cmdLearn.toString().c_str());
                        rpcKarmaLearn.write(cmdLearn, replyLearn);
                        fprintf(stdout, "the reply is: %s \n",replyLearn.toString().c_str());
                    }
                }
            }
            goHome();
        }
    }
    return 0;
}
/**********************************************************/
double Manager::getBlobLenght(const Bottle &blobs, const int i)
{

    double dist = 0;
    if ((i>=0) && (i<blobs.size()))
    {
        CvPoint tl,br;
        Bottle *item=blobs.get(i).asList();
        if (item==NULL)
            return dist;

        tl.x=(int)item->get(0).asDouble();
        tl.y=(int)item->get(1).asDouble();
        br.x=(int)item->get(2).asDouble();
        br.y=(int)item->get(3).asDouble();

        
        int dx = abs(tl.x - br.x);
        int dy = abs(tl.y - br.y);
        
        fprintf(stdout, "%d %d \n", dx, dy);

        dist = sqrt ( (double)((dx*dx) + (dy*dy)) );

    }
    return dist;
}

/**********************************************************/
double Manager::wrapAng ( const double ang )
{
    if ((ang<0.0) || (ang>=360.0))
    {
        double theta=(M_PI/180.0)*ang;
        theta=(180.0/M_PI)*atan2(sin(theta),cos(theta));
        if (theta<0.0)
            theta+=360.0;
        return theta;
    }
    else
        return ang;
}
/**********************************************************/
Bottle Manager::getOffset( Bottle &closestBlob, double actionOrient, Vector &initPos )
{
    double orient = 0.0;
    orient = closestBlob.get(3).asDouble();
    
    int axe1 = 0;
    int axe2 = 0;
    CvPoint cog;
    cog.x = closestBlob.get(0).asInt();
    cog.y = closestBlob.get(1).asInt();
    
    axe1 = closestBlob.get(4).asInt();
    axe2 = closestBlob.get(5).asInt();
    double finalOrient=actionOrient;
    fprintf(stdout ,"INITIAL orientation is: %lf \n", orient);
    if (abs(axe1-axe2) < 5)
    {
        fprintf(stdout,"seem a round object to me...sending theta2\n");
    }
    else
    {
        fprintf(stdout,"sending theta2 - theta1 <-> axis diff is %d\n", abs(axe1-axe2));
        finalOrient -= orient;
    }

    CvPoint pxls;
    Vector offPos;
    fprintf(stdout,"The 3Dpos is %lf %lf %lf \n",initPos[0], initPos[1], initPos[2]);
    
    double alpha = finalOrient * M_PI/180; //radians
    pxls.x = int(cog.x + (axe1 * 1.2 ) * cos(alpha));
    pxls.y = int(cog.y - (axe2 * 1.2 ) * sin(alpha));
    
    get3DPosition(pxls,offPos);
    fprintf(stdout,"The 3Dpos off point is  %lf %lf %lf \n",offPos[0], offPos[1], offPos[2]);
    double offset= norm (initPos-offPos);
    fprintf(stdout,"The offset is %lf \n",offset);
    
    Bottle ret;
    ret.addDouble(offset);
    ret.addDouble(finalOrient);
    return ret;
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

    printf("Sending classification request: %s\n",cmd.toString().c_str());
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
            
            fprintf(stdout,"bottle is: %s \n",tmp_scores->toString().c_str());
            
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
bool Manager::get3DPosition(const CvPoint &point, Vector &x)
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("get"));
    cmdMotor.addVocab(Vocab::encode("s2c"));
    Bottle &options=cmdMotor.addList();
    options.addString(camera.c_str());
    options.addInt(point.x);
    options.addInt(point.y);
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
    int axe1 = item->get(5).asInt();
    int axe2 = item->get(6).asInt();
    pointReturn.addDouble(orient);
    pointReturn.addInt(axe1);
    pointReturn.addInt(axe2);
    return pointReturn;
}
/**********************************************************/
