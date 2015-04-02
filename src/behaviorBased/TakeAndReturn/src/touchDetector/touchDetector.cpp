// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email:  ali.paikan@iit.it
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

#include "touchDetector.h"
#include <iCub/iKin/iKinFwd.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
//using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;

#define     FOREARM_LEFT    2
#define     FOREARM_RIGHT   5
#define     UPARM_LEFT      3
#define     UPARM_RIGHT     6
//#define     FOREARM_LEFT    2
//#define     FOREARM_LEFT    2


void TouchDetector::loop()
{
    
    
    skinContactList *skinContacts  = inputPort.read(false);
    
    if(skinContacts)
    {
        if(detectContact(skinContacts)) // READ A CONTACT ON THE SKIN
        {            
            printf("XYZ: %.2f, %.2f %.2f\n", cntctPosWRF[0], cntctPosWRF[1], cntctPosWRF[2]);

            Bottle &target=targetPort.prepare();
            target.clear();

            if(cntctPosWRF[0] > 0 )
                cntctPosWRF[0] = -0.10;
            target.addDouble(cntctPosWRF[0]);
            target.addDouble(cntctPosWRF[1]);
            target.addDouble(cntctPosWRF[2]);

            Bottle &cmd=faceExpPort.prepare();
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("all"));
            cmd.addVocab(Vocab::encode(faceExpression.c_str()));           
            
            targetPort.write();                
            faceExpPort.write();
        }            
    }
    

    /*
    if(posEye)
    {
        if((posEye->size()>6) && (posEye->get(6).asDouble()==1.0) )
        {
            Vector fp(4);
            fp[0]=posEye->get(0).asDouble();
            fp[1]=posEye->get(1).asDouble();
            fp[2]=posEye->get(2).asDouble();
            fp[3]=1.0;
            Vector x,o;
            if (eye == "left")
                iGaze->getLeftEyePose(x,o);
            else
                iGaze->getRightEyePose(x,o);

            Matrix T = axis2dcm(o);
            T(0,3)=x[0];
            T(1,3)=x[1];
            T(2,3)=x[2];
            Vector posRoot = T*fp;

            //printf("dbg 0:%s\n",T.toString(3,3).c_str());            

            Bottle &target=targetPort.prepare();
            target.clear();

            target.addDouble(posRoot[0]+x_offset);
            if(leftArmEnabled)
                target.addDouble(posRoot[1]-fabs(y_offset));
            else
                target.addDouble(posRoot[1]+fabs(y_offset));            
            target.addDouble(posRoot[2]+z_offset);

            if(leftArmEnabled)
            {
                for(int i=0; i<4; i++)
                  target.addDouble(LEFT_ROT_POS[i]);
            }
            else
            {
                for(int i=0; i<4; i++)
                  target.addDouble(RIGHT_ROT_POS[i]);
            }

            Bottle &cmd=faceExpPort.prepare();
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("all"));
            cmd.addVocab(Vocab::encode(faceExpression.c_str()));

            if((posRoot[0] <= MIN_X_RANGE)  && (posRoot[0] >= MAX_X_RANGE) && 
              (posRoot[2] >= MIN_Z_RANGE)  && (posRoot[2] <= MAX_Z_RANGE) )  
            {
                targetPort.write();                
                faceExpPort.write();
            }                
        }
    }
    */
}

bool TouchDetector::open(yarp::os::ResourceFinder &rf)
{    
    string name = rf.check("name",Value("touchDetector")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    faceExpression = rf.check("expression", Value("cun")).asString().c_str();

    /*
    string robotName = rf.check("robot",Value("icub")).asString().c_str();
    string partName = rf.check("part",Value("right_arm")).asString().c_str();
    string local = string("/touchDetector/board/")+partName;
    string remote = string("/")+robotName+string("/")+partName;

    printf("%s\n", local.c_str());
    printf("%s\n", remote.c_str());

    Property opt;
    opt.put("device", "remote_controlboard");
    opt.put("local", local.c_str());   //local port names
    opt.put("remote", remote.c_str());         //where we connect to
    // create a device
    if(!ddR.open(opt))
        return false;
    */

    

    Property OptR;
    //OptR.put("robot",  robot.c_str());
    //OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name+"/posCtrl/right_arm").c_str());


    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name+"/posCtrl/left_arm").c_str());

    Property OptT;
    OptT.put("robot",  robot.c_str());
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote",("/"+robot+"/torso").c_str());
    OptT.put("local", ("/"+name+"/posCtrl/torso").c_str());


    if (!ddR.open(OptR))
    {
        printf("%s : %d\n", __FILE__, __LINE__);
        printf("ERROR: could not open right_arm PolyDriver!\n");
        return false;
    }

    // open the view
    bool ok = 1;
    if (ddR.isValid())
        ok = ok && ddR.view(iencsR);

    if (!ok)
    {
        printf("\nERROR: Problems acquiring right_arm interfaces!!!!\n");
        return false;
    }

    int jntsR;
    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    if (!ddL.open(OptL))
    {
        printf("ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }

    // open the view
    ok = 1;
    if (ddL.isValid())
        ok = ok && ddL.view(iencsL);

    if (!ok)
    {
        printf("\nERROR: Problems acquiring left_arm interfaces!!!!\n");
        return false;
    }

    int jntsL;
    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    // TORSO
    if (!ddT.open(OptT))
    {
        printf("ERROR: could not open torso PolyDriver!\n");
        return false;
    }

    // open the view
    ok = 1;
    if (ddT.isValid())
        ok = ok && ddT.view(iencsT);

    if (!ok)
    {
        printf("\nERROR: Problems acquiring torso interfaces!!!!\n");
        return false;
    }

    int jntsT;
    iencsT->getAxes(&jntsT);
    encsT = new Vector(jntsT,0.0);

    bool ret=true; 
    ret = targetPort.open("/touchDetector/gazeXd");    
    ret &= inputPort.open("/touchDetector/contatcs/in");    
    ret &= faceExpPort.open("/touchDetector/face:rpc"); 
    return ret;
}

bool TouchDetector::close()
{

    targetPort.close();   
    //inputPort.close();
    faceExpPort.close();

    ddR.close();
    ddL.close();
    ddT.close();

    delete encsL; 
    delete encsR; 
    delete encsT; 

    return true;
}

bool TouchDetector::interrupt()
{
    //inputPort.interrupt();
    return true;
}


bool TouchDetector::detectContact(skinContactList *_sCL)
{
    // Reset variables:
    //cntctPosLink.resize(3,0.0);
    cntctPosWRF.resize(3,0.0);
    //cntctNormDir.resize(3,0.0);
    //cntctPressure = -1;
    //cntctLinkNum = -1;
    cntctArm = "";


    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++) {
        int skinPart = it -> getSkinPart(); // Retrieve the skinPart of the skinContact
        //printf("%s\n", it->toString().c_str());
        if(skinPart == FOREARM_LEFT || skinPart == FOREARM_RIGHT || skinPart == UPARM_LEFT || skinPart == UPARM_RIGHT)
        {
            // Store the skinContact for eventual future use
            cntctSkin = *it;
            if(skinPart == FOREARM_LEFT || skinPart == UPARM_LEFT)
            {
                cntctArm = "left";
            }
            else if(skinPart == FOREARM_RIGHT || skinPart == UPARM_RIGHT)
            {
                cntctArm = "right";
            }
            cntctPosWRF = locateContact(cntctSkin);
            return true;
        }
    }
    //printf("\n\n---\n\n");
    return false;
}

Vector TouchDetector::locateContact(skinContact &sc)
{
    Vector result(4,0.0);
    Matrix Twl = eye(4);
    iCubArm *lmb = new iCubArm(cntctArm.c_str());
    lmb -> releaseLink(0);
    lmb -> releaseLink(1);
    lmb -> releaseLink(2);

    iencsL->getEncoders(encsL->data());
    iencsT->getEncoders(encsT->data());
    Vector q(10,0.0);
    q.setSubvector(3,encsL->subVector(0,6));
    q[0] = (*encsT)[2];
    q[1] = (*encsT)[1];
    q[2] = (*encsT)[0];

    lmb -> setAng(q*CTRL_DEG2RAD);

    Twl = lmb -> getH(sc.getLinkNumber()+3, true);
    Vector posLink = sc.getGeoCenter();
    posLink.push_back(1);
    result = Twl * posLink;
    result.pop_back();

    delete lmb;
    return result;
}

