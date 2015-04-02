/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

/** 
\defgroup karmaMotor Motor Part of the KARMA Experiment
 
@ingroup icub_karma  
 
Motor Control Module that allows the robot to push/draw the 
object and explore a tool. 

\section intro_sec Description 
This module aims to control the robot hands in order to properly
execute the push and the draw actions of an object within the 
KARMA experiment to then learn the corresponding affordance. \n 
It also enable the tool exploration. 
 
\section lib_sec Libraries 
- YARP libraries. 
- icubmod library.

\section parameters_sec Parameters 
--robot \e robot
- Select the robot to connect to.

--name \e name
- Select the stem-name of the module used to open up ports. 
  By default \e name is <i>karmaMotor</i>. 
 
\section portsa_sec Ports Accessed
Assume that iCubInterface (with ICartesianControl interface
implemented) is running. 
 
\section portsc_sec Ports Created 
- \e /karmaMotor/rpc receives the information to execute the 
  motor action as a Bottle. It manages the following commands:
  -# <b>Push</b>: <i>[push] cx cy cz theta radius</i>. \n
  The coordinates <i>(cx,cy,cz)</i> represent in meters the
  position of the object's centroid to be pushed; <i>theta</i>,
  given in degrees, and <i>radius</i>, specified in meters,
  account for the point from which push the object, that is
  located onto the circle centered in <i>(cx,cy,cz)</i> and
  contained in the x-y plane. \n
  The reply <i>[ack]</i> is returned as soon as the push is
  accomplished.
  -# <b>Draw</b>: <i>[draw] cx cy cz theta radius dist</i>. \n
  The coordinates <i>(cx,cy,cz)</i> represent in meters the
  position of the object's centroid to be drawn closer;
  <i>theta</i>, given in degrees, and <i>radius</i>, specified
  in meters, account for the point from which draw the object,
  that is located onto the circle centered in <i>(cx,cy,cz)</i>
  and contained in the x-y plane. The parameter <i>dist</i>
  specifies the length in meters of the draw action. \n
  The reply <i>[ack]</i> is returned as soon as the draw is
  accomplished.
  -# <b>Virtual draw</b>: <i>[vdraw] cx cy cz theta radius
   dist</i>. \n Simulate the draw without performing any
   movement in order to test the quality of the action. \n
   The reply <i>[ack] val</i> is returned at the end of the
   simulation, where <i>val</i> accounts for the quality of the
   action: the lower it is the better the action is.
  -# <b>Tool-attach</b>: <i>[tool] [attach] arm x y z</i>. \n
  Attach a tool to the given arm whose dimensions are specified
  in the frame attached to the hand. The subsequent push action
  will make use of this tool.
  -# <b>Tool-get</b>: <i>[tool] [get]</i>. \n
  Retrieve tool information as <i>[ack] arm x y z</i>.
  -# <b>Tool-remove</b>: <i>[tool] [remove]</i>. \n
  Remove the attached tool.
  -# <b>Find</b>: <i>[find] arm eye</i>. \n
  An exploration is performed which aims at finding the tool
  dimension. It is possible to select the arm for executing the
  movement as well as the eye from which the motion is observed.
  The reply <i>[ack] x y z</i> returns the tool's dimensions
  with respect to reference frame attached to the robot hand.
 
- \e /karmaMotor/vision:i receives the information about the 
  pixel corresponding to the tool tip during the tool
  exploration phase.
 
- \e /karmaMotor/finder:rpc communicates with the module in 
  charge of solving for the tool's dimensions.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <string>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;


/************************************************************************/
class KarmaMotor: public RFModule
{
protected:
    PolyDriver driverG;
    PolyDriver driverL;
    PolyDriver driverR;
    PolyDriver driverHL;
    PolyDriver driverHR;

    IGazeControl      *iGaze;
    ICartesianControl *iCartCtrlL;
    ICartesianControl *iCartCtrlR;
    ICartesianControl *iCartCtrl;

    string pushHand;
    Matrix toolFrame;

    string handUsed;
    bool interrupting;
    double flipHand;

    BufferedPort<Bottle> visionPort;
    RpcClient            finderPort;
    RpcServer            rpcPort;

    /************************************************************************/
    double dist(const Matrix &M)
    {
        double ret=0.0;
        for (int r=0; r<M.rows(); r++)
            for (int c=0; c<M.cols(); c++)
                ret+=M(r,c)*M(r,c);

        return sqrt(ret);
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            int cmd=command.get(0).asVocab();
            switch (cmd)
            {
                //-----------------
                case VOCAB4('p','u','s','h'):
                {
                    Bottle payload=command.tail();
                    if (payload.size()>=5)
                    {
                        Vector c(3);
                        double theta;
                        double radius;

                        c[0]=payload.get(0).asDouble();
                        c[1]=payload.get(1).asDouble();
                        c[2]=payload.get(2).asDouble();
                        theta=payload.get(3).asDouble();
                        radius=payload.get(4).asDouble();

                        push(c,theta,radius,pushHand,toolFrame);
                        reply.addVocab(ack);
                        return true;
                    }

                    break;
                }

                //-----------------
                case VOCAB4('d','r','a','w'):
                case VOCAB4('v','d','r','a'):
                {
                    Bottle payload=command.tail();
                    if (payload.size()>=6)
                    {
                        Vector c(3);
                        double theta;
                        double radius;
                        double dist;

                        c[0]=payload.get(0).asDouble();
                        c[1]=payload.get(1).asDouble();
                        c[2]=payload.get(2).asDouble();
                        theta=payload.get(3).asDouble();
                        radius=payload.get(4).asDouble();
                        dist=payload.get(5).asDouble();

                        double res=draw(cmd==VOCAB4('v','d','r','a'),c,theta,
                                        radius,dist,pushHand,toolFrame);

                        reply.addVocab(ack);
                        if (cmd==VOCAB4('v','d','r','a'))
                            reply.addDouble(res);
                        return true;
                    }

                    break;
                }

                //-----------------
                case VOCAB4('f','i','n','d'):
                {
                    Bottle payload=command.tail();
                    if (payload.size()>=2)
                    {
                        string arm=payload.get(0).asString().c_str();
                        string eye=payload.get(1).asString().c_str();
                        Bottle solution;

                        if (findToolTip(arm,eye,solution))
                        {
                            reply.addVocab(ack);
                            reply.append(solution.tail());
                        }
                        else
                            reply.addVocab(nack);

                        return true;
                    }

                    break;
                }

                //-----------------
                case VOCAB4('t','o','o','l'):
                {
                    if (command.size()>1)
                    {
                        Bottle subcommand=command.tail();
                        int tag=subcommand.get(0).asVocab();
                        if (tag==Vocab::encode("attach"))
                        {
                            Bottle payload=subcommand.tail();
                            if (payload.size()>=4)
                            {
                                pushHand=payload.get(0).asString().c_str();

                                Vector point(4);
                                point[0]=payload.get(1).asDouble();
                                point[1]=payload.get(2).asDouble();
                                point[2]=payload.get(3).asDouble();
                                point[3]=1.0;

                                Vector r(4,0.0);
                                r[2]=-1.0;
                                r[3]=atan2(-point[1],point[0]);
                                toolFrame=axis2dcm(r);
                                toolFrame.setCol(3,point);

                                reply.addVocab(ack);
                                return true;
                            }
                        }
                        else if (tag==Vocab::encode("get"))
                        {
                            reply.addVocab(ack);
                            reply.addString(pushHand.c_str());
                            reply.addDouble(toolFrame(0,3));
                            reply.addDouble(toolFrame(1,3));
                            reply.addDouble(toolFrame(2,3));
                            return true;
                        }
                        else if (tag==Vocab::encode("remove"))
                        {
                            pushHand="selectable";
                            toolFrame=eye(4,4);

                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    /************************************************************************/
    void push(const Vector &c, const double theta, const double radius,
              const string &armType="selectable", const Matrix &frame=eye(4,4))
    {
        // wrt root frame: frame centered at c with x-axis pointing rightward,
        // y-axis pointing forward and z-axis pointing upward
        Matrix H0(4,4); H0.zero();
        H0(1,0)=1.0;
        H0(0,1)=-1.0;
        H0(2,2)=1.0;
        H0(0,3)=c[0]; H0(1,3)=c[1]; H0(2,3)=c[2]; H0(3,3)=1.0;

        double theta_rad=CTRL_DEG2RAD*theta;
        double _c=cos(theta_rad);
        double _s=sin(theta_rad);
        double _theta=CTRL_RAD2DEG*atan2(_s,_c);    // to have theta in [-180.0,180.0]
        double epsilon=0.05;

        // wrt H0 frame: frame centered at R*[_c,_s] with z-axis pointing inward
        // and x-axis tangential
        Matrix H1(4,4); H1.zero();
        H1(0,0)=-_s;       H1(1,0)=_c;
        H1(2,1)=-1.0;
        H1(0,2)=-_c;       H1(1,2)=-_s;
        H1(0,3)=radius*_c; H1(1,3)=radius*_s; H1(3,3)=1.0;

        // wrt H0 frame: frame centered at R*[_c,_s] with z-axis pointing outward
        // and x-axis tangential
        Matrix H2(4,4); H2.zero();
        H2(0,0)=_s;        H2(1,0)=-_c;
        H2(2,1)=-1.0;
        H2(0,2)=_c;        H2(1,2)=_s;
        H2(0,3)=radius*_c; H2(1,3)=radius*_s; H2(3,3)=1.0;

        // matrices that serve to account for pushing with the back of the hand
        Matrix H1eps=H1; Matrix H2eps=H2;
        H1eps(0,3)+=epsilon*_c; H1eps(1,3)+=epsilon*_s;
        H2eps(0,3)+=epsilon*_c; H2eps(1,3)+=epsilon*_s;
        
        // go back into root frame and apply tool (if any)
        Matrix invFrame=SE3inv(frame);
        H1=H0*H1*invFrame;
        H2=H0*H2*invFrame;
        H1eps=H0*H1eps*invFrame;
        H2eps=H0*H2eps*invFrame;
        
        Vector xd1=H1.getCol(3).subVector(0,2);
        Vector od1=dcm2axis(H1);

        Vector xd2=H2.getCol(3).subVector(0,2);
        Vector od2=dcm2axis(H2);

        Vector xd1eps=H1eps.getCol(3).subVector(0,2);
        Vector od1eps=dcm2axis(H1eps);

        Vector xd2eps=H2eps.getCol(3).subVector(0,2);
        Vector od2eps=dcm2axis(H2eps);

        printf("identified locations...\n");
        printf("xd1=(%s) od1=(%s)\n",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        printf("xd2=(%s) od2=(%s)\n",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // choose the arm
        if (armType=="selectable")
        {
            if (xd1[1]>=0.0)
                iCartCtrl=iCartCtrlR;
            else
                iCartCtrl=iCartCtrlL;
        }
        else if (armType=="left")
            iCartCtrl=iCartCtrlL;
        else
            iCartCtrl=iCartCtrlR;

        // deal with the arm context
        int context;
        iCartCtrl->storeContext(&context);

        Bottle options;
        Bottle &straightOpt=options.addList();
        straightOpt.addString("straightness");
        straightOpt.addDouble(10.0);
        iCartCtrl->tweakSet(options);

        Vector dof;
        iCartCtrl->getDOF(dof);
        
        dof=1.0; dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);

        Vector xdhat1,odhat1,xdhat2,odhat2;
        Vector dummy;

        // try out different poses
        iCartCtrl->askForPose(xd1,od1,xdhat1,odhat1,dummy);
        iCartCtrl->askForPose(xd2,od2,xdhat2,odhat2,dummy);

        Matrix Hhat1=axis2dcm(odhat1); Hhat1(0,3)=xdhat1[0]; Hhat1(1,3)=xdhat1[1]; Hhat1(2,3)=xdhat1[2];
        Matrix Hhat2=axis2dcm(odhat2); Hhat2(0,3)=xdhat2[0]; Hhat2(1,3)=xdhat2[1]; Hhat2(2,3)=xdhat2[2];

        double d1=dist(H1-Hhat1);
        double d2=dist(H2-Hhat2);

        printf("solutions...\n");
        printf("#1: xdhat1=(%s) odhat1=(%s); e=%.3f\n",xdhat1.toString(3,3).c_str(),odhat1.toString(3,3).c_str(),d1);
        printf("#2: xdhat2=(%s) odhat2=(%s); e=%.3f\n",xdhat2.toString(3,3).c_str(),odhat2.toString(3,3).c_str(),d2);
        printf("selection: ");

        // compare solutions and choose the best
        Vector *xd,*od;
        if (fabs(_theta-90.0)<45.0)
        {
            printf("(detected singularity) ");
            if (iCartCtrl==iCartCtrlR)
            {
                xd=&xd1;
                od=&od1;
            }
            else
            {
                xd=&xd2;
                od=&od2;
            }
        }
        else if (fabs(_theta+90.0)<45.0)
        {
            printf("(detected singularity) ");
            if (iCartCtrl==iCartCtrlR)
            {
                xd=&xd2;
                od=&od2;
            }
            else
            {
                xd=&xd1;
                od=&od1;
            }
        }
        else if (d1<d2)
        {
            xd=&xd1;
            od=&od1;
        }
        else
        {
            xd=&xd2;
            od=&od2;
        }

        if (xd==&xd1)
            printf("#1 ");
        else
            printf("#2 ");

        if ((iCartCtrl==iCartCtrlR) && (_theta<0.0) && (xd==&xd2))
        {
            printf("(increased radius)");
            xd=&xd2eps;
            od=&od2eps;
        }
        else if ((iCartCtrl==iCartCtrlL) && (_theta<0.0) && (xd==&xd1))
        {
            printf("(increased radius)");
            xd=&xd1eps;
            od=&od1eps;
        }

        printf(": xd=(%s); od=(%s)\n",xd->toString(3,3).c_str(),od->toString(3,3).c_str());

        // execute the movement
        Vector offs(3,0.0); offs[2]=0.1;
        if (!interrupting)
        {
            Vector x=*xd+offs;

            printf("moving to: x=(%s); o=(%s)\n",x.toString(3,3).c_str(),od->toString(3,3).c_str());
            iCartCtrl->goToPoseSync(x,*od,1.0);
            iCartCtrl->waitMotionDone(0.1,4.0);
        }

        if (!interrupting)
        {
            printf("moving to: x=(%s); o=(%s)\n",xd->toString(3,3).c_str(),od->toString(3,3).c_str());
            iCartCtrl->goToPoseSync(*xd,*od,1.0);
            iCartCtrl->waitMotionDone(0.1,4.0);
        }

        double rmin,rmax,tmin,tmax;
        if (((fabs(theta)<10.0) || (fabs(theta-180.0)<10.0)))
        {
            rmin=0.04; rmax=0.18;
            tmin=0.40; tmax=0.60;
        }
        else
        {
            rmin=0.04; rmax=0.18;
            tmin=0.50; tmax=0.80;
        }

        // safe guard for using the tool
        if (armType!="selectable")
        {
            tmin*=1.3;
            tmax*=1.3;
        }

        double trajTime=tmin+((tmax-tmin)/(rmax-rmin))*(radius-rmin);
        trajTime=std::max(std::min(tmax,trajTime),tmin);

        if (!interrupting)
        {
            Matrix H=axis2dcm(*od);
            Vector center=c; center.push_back(1.0);
            H.setCol(3,center);
            Vector x=-1.0*frame.getCol(3); x[3]=1.0;
            x=H*x; x.pop_back();

            printf("moving to: x=(%s); o=(%s)\n",x.toString(3,3).c_str(),od->toString(3,3).c_str());
            iCartCtrl->goToPoseSync(x,*od,trajTime);
            iCartCtrl->waitMotionDone(0.1,3.0);
        }

        if (!interrupting)
        {
            printf("moving to: x=(%s); o=(%s)\n",xd->toString(3,3).c_str(),od->toString(3,3).c_str());
            iCartCtrl->goToPoseSync(*xd,*od,1.0);
            iCartCtrl->waitMotionDone(0.1,2.0);
        }
        
        iCartCtrl->restoreContext(context);
        iCartCtrl->deleteContext(context);
    }

    /************************************************************************/
    double draw(bool simulation, const Vector &c, const double theta, const double radius,
                const double dist, const string &armType, const Matrix &frame=eye(4,4))
    {
        // c0 is the projection of c on the sagittal plane
        Vector c_sag=c;
        c_sag[1]=0.0;

        // wrt root frame: frame centered at c_sag with x-axis pointing rightward,
        // y-axis pointing forward and z-axis pointing upward
        Matrix H0(4,4); H0.zero();
        H0(1,0)=1.0;
        H0(0,1)=-1.0;
        H0(2,2)=1.0;
        H0(0,3)=c_sag[0]; H0(1,3)=c_sag[1]; H0(2,3)=c_sag[2]; H0(3,3)=1.0;

        double theta_rad=CTRL_DEG2RAD*theta;
        double _c=cos(theta_rad);
        double _s=sin(theta_rad);

        // wrt H0 frame: frame translated in R*[_c,_s]
        Matrix H1=eye(4,4);
        H1(0,3)=radius*_c; H1(1,3)=radius*_s;

        // wrt H1 frame: frame translated in [0,-dist]
        Matrix H2=eye(4,4);
        H2(1,3)=-dist;

        // go back into root frame
        H2=H0*H1*H2;
        H1=H0*H1;

        // apply final axes
        Matrix R(3,3);
        if (iCartCtrl==iCartCtrlR)
        {
            R(0,0)=-1.0;
            R(2,1)=-1.0;
            R(1,2)=-1.0;
        }
        else
        {
            R(0,0)=-1.0;
            R(2,1)=-1.0;
            R(1,2)=-1.0;
        }

        H1.setSubmatrix(R,0,0);
        H2.setSubmatrix(R,0,0);

        Vector xd1=H1.getCol(3).subVector(0,2);
        Vector od1=dcm2axis(H1);

        Vector xd2=H2.getCol(3).subVector(0,2);
        Vector od2=dcm2axis(H2);

        printf("identified locations on the sagittal plane...\n");
        printf("xd1=(%s) od1=(%s)\n",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        printf("xd2=(%s) od2=(%s)\n",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // choose the arm
        if (armType=="selectable")
        {
            if (xd1[1]>=0.0)
                iCartCtrl=iCartCtrlR;
            else
                iCartCtrl=iCartCtrlL;
        }
        else if (armType=="left")
            iCartCtrl=iCartCtrlL;
        else
            iCartCtrl=iCartCtrlR;

        // recover the original place: do translation and rotation
        if (c[1]!=0.0)
        {
            Vector r(4,0.0);
            r[2]=-1.0;
            r[3]=atan2(c[1],fabs(c[0]));
            Matrix H=axis2dcm(r);

            H(0,3)=H1(0,3);
            H(1,3)=H1(1,3)+c[1];
            H(2,3)=H1(2,3);
            H1(0,3)=H1(1,3)=H1(2,3)=0.0;
            H1=H*H1;

            H(0,3)=H2(0,3);
            H(1,3)=H2(1,3)+c[1];
            H(2,3)=H2(2,3);
            H2(0,3)=H2(1,3)=H2(2,3)=0.0;
            H2=H*H2;

            xd1=H1.getCol(3).subVector(0,2);
            od1=dcm2axis(H1);

            xd2=H2.getCol(3).subVector(0,2);
            od2=dcm2axis(H2);
        }

        printf("in-place locations...\n");
        printf("xd1=(%s) od1=(%s)\n",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        printf("xd2=(%s) od2=(%s)\n",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // apply tool (if any)
        Matrix invFrame=SE3inv(frame);
        H1=H1*invFrame;
        H2=H2*invFrame;

        xd1=H1.getCol(3).subVector(0,2);
        od1=dcm2axis(H1);

        xd2=H2.getCol(3).subVector(0,2);
        od2=dcm2axis(H2);

        printf("apply tool (if any)...\n");
        printf("xd1=(%s) od1=(%s)\n",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
        printf("xd2=(%s) od2=(%s)\n",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());

        // deal with the arm context
        int context;
        iCartCtrl->storeContext(&context);

        Bottle options;
        Bottle &straightOpt=options.addList();
        straightOpt.addString("straightness");
        straightOpt.addDouble(30.0);
        iCartCtrl->tweakSet(options);

        Vector dof;
        iCartCtrl->getDOF(dof);

        dof=1.0; dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);
        
        double res=0.0;

        // simulate the movements
        if (simulation)
        {
            Vector xdhat1,odhat1,xdhat2,odhat2,qdhat;
            iCartCtrl->askForPose(xd1,od1,xdhat1,odhat1,qdhat);
            iCartCtrl->askForPose(qdhat,xd2,od2,xdhat2,odhat2,qdhat);

            double e_x1=norm(xd1-xdhat1);
            double e_o1=norm(od1-odhat1);
            printf("testing x=(%s); o=(%s) => xhat=(%s); ohat=(%s) ... |e_x|=%g; |e_o|=%g\n",
                   xd1.toString(3,3).c_str(),od1.toString(3,3).c_str(),
                   xdhat1.toString(3,3).c_str(),odhat1.toString(3,3).c_str(),
                   e_x1,e_o1);

            double e_x2=norm(xd2-xdhat2);
            double e_o2=norm(od2-odhat2);
            printf("testing x=(%s); o=(%s) => xhat=(%s); ohat=(%s) ... |e_x|=%g; |e_o|=%g\n",
                   xd2.toString(3,3).c_str(),od2.toString(3,3).c_str(),
                   xdhat2.toString(3,3).c_str(),odhat2.toString(3,3).c_str(),
                   e_x2,e_o2);

            double nearness_penalty=(norm(xdhat2)<0.15?10.0:0.0);
            printf("nearness penalty=%g\n",nearness_penalty);
            res=e_x1+e_o1+e_x2+e_o2+nearness_penalty;
            printf("final quality=%g\n",res);
        }
        // execute the movements
        else
        {
            Vector offs(3,0.0); offs[2]=0.05;
            if (!interrupting)
            {
                Vector x=xd1+offs;

                printf("moving to: x=(%s); o=(%s)\n",x.toString(3,3).c_str(),od1.toString(3,3).c_str());
                iCartCtrl->goToPoseSync(x,od1,2.0);
                iCartCtrl->waitMotionDone(0.1,5.0);
            }

            if (!interrupting)
            {
                printf("moving to: x=(%s); o=(%s)\n",xd1.toString(3,3).c_str(),od1.toString(3,3).c_str());
                iCartCtrl->goToPoseSync(xd1,od1,1.5);
                iCartCtrl->waitMotionDone(0.1,5.0);
            }

            if (!interrupting)
            {
                printf("moving to: x=(%s); o=(%s)\n",xd2.toString(3,3).c_str(),od2.toString(3,3).c_str());
                iCartCtrl->goToPoseSync(xd2,od2,3.5);
                iCartCtrl->waitMotionDone(0.1,5.0);
            }
        }

        iCartCtrl->restoreContext(context);
        iCartCtrl->deleteContext(context);

        return res;
    }

    /************************************************************************/
    void shakeHand()
    {
        IEncoders        *ienc;
        IVelocityControl *ivel;

        if (handUsed=="left")
        {
            driverHL.view(ienc);
            driverHL.view(ivel);
        }
        else
        {
            driverHR.view(ienc);
            driverHR.view(ivel);
        }

        double pos;
        ienc->getEncoder(4,&pos);

        double e=flipHand-pos;
        if (fabs(e)<1.0)
        {
            flipHand=-flipHand;
            e=flipHand-pos;
        }

        ivel->velocityMove(4,70.0*sign(e));
    }

    /************************************************************************/
    void stopHand(const string &hand)
    {
        IVelocityControl *ivel;
        if (hand=="left")
            driverHL.view(ivel);
        else
            driverHR.view(ivel);

        ivel->stop(4);
    }

    /************************************************************************/
    void moveTool(const string &arm, const string &eye, const Vector &xd, const Vector &od,
                  const Vector &xOffset, const int maxItems)
    {
        iGaze->restoreContext(0);
        
        if (!interrupting)
        {
            iGaze->setTrackingMode(true);
            iGaze->lookAtFixationPoint(xd+xOffset);
            iCartCtrl->goToPoseSync(xd,od,1.0);
            iCartCtrl->waitMotionDone(0.1);
        }

        iGaze->setSaccadesStatus(false);
        iGaze->setNeckTrajTime(1.5);
        iGaze->setEyesTrajTime(0.7);
        handUsed=arm;   // this triggers the hand shaking

        // gaze robustly at the tool tip
        Vector pxCum(2,0.0);
        int cnt=0; bool done=false;
        double t0=Time::now();
        while (!interrupting && !done)
        {
            double t1=Time::now();
            if (Bottle *target=visionPort.read(false))
            {
                if (target->size()>=2)
                {
                    Vector px(2);
                    px[0]=target->get(0).asDouble();
                    px[1]=target->get(1).asDouble()+50.0;
                    iGaze->lookAtMonoPixel(eye=="left"?0:1,px);

                    pxCum+=px;
                    cnt++;
                }
            }

            if (t1-t0>=3.0)
            {
                if (cnt>20)
                    done=fabs(pxCum[1]/cnt-120)<30.0;

                pxCum=0.0;
                cnt=0;
                t0=t1;
            }

            Time::delay(0.02);
        }

        iGaze->setNeckTrajTime(2.5);
        iGaze->setEyesTrajTime(1.5);

        // gather sufficient information
        Bottle command,reply;
        command.addVocab(Vocab::encode("enable"));
        finderPort.write(command,reply);

        command.clear();
        command.addVocab(Vocab::encode("num"));
        finderPort.write(command,reply);
        int curItems=reply.get(1).asInt();

        int nItems=0;
        while (!interrupting && (nItems<curItems+maxItems))
        {
            finderPort.write(command,reply);
            nItems=reply.get(1).asInt();

            if (Bottle *target=visionPort.read(false))
            {
                if (target->size()>=2)
                {
                    Vector px(2);
                    px[0]=target->get(0).asDouble();
                    px[1]=target->get(1).asDouble()+50.0;
                    iGaze->lookAtMonoPixel(eye=="left"?0:1,px);
                }
            }

            Time::delay(0.1);
        }

        command.clear();
        command.addVocab(Vocab::encode("disable"));
        finderPort.write(command,reply);

        handUsed="null";
        stopHand(arm);
    }

    /************************************************************************/
    bool findToolTip(const string &arm, const string &eye, Bottle &reply)
    {
        if (arm=="left")
            iCartCtrl=iCartCtrlL;
        else if (arm=="right")
            iCartCtrl=iCartCtrlR;
        else
            return false;

        int context_arm,context_gaze;
        iCartCtrl->storeContext(&context_arm);
        iGaze->storeContext(&context_gaze);

        Vector dof;
        iCartCtrl->getDOF(dof);
        dof=1.0; dof[0]=dof[1]=0.0;
        iCartCtrl->setDOF(dof,dof);

        Bottle command;
        command.addVocab(Vocab::encode("clear"));
        finderPort.write(command,reply);

        // solving
        command.clear();
        command.addVocab(Vocab::encode("select"));
        command.addString(arm.c_str());
        command.addString(eye.c_str());
        finderPort.write(command,reply);

        Matrix R(4,4);
        R(0,0)=-1.0;
        R(2,1)=-1.0;
        R(1,2)=-1.0;
        R(3,3)=+1.0;
        Vector r(4,0.0); r[0]=-1.0;
        Vector xd(3),od;
        Vector offset(3,0.0); offset[2]=0.1;

        // point 1
        r[3]=0.0;
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.35;
        moveTool(arm,eye,xd,od,offset,25);

        // point 2
        r[3]=CTRL_DEG2RAD*(arm=="left"?30.0:-30.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[1]=(arm=="left")?-0.15:0.15;
        offset[1]=(arm=="left")?0.1:-0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // point 3
        r[3]=CTRL_DEG2RAD*(arm=="left"?20.0:-20.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[2]=0.15;
        offset[1]=(arm=="left")?0.2:-0.2;
        offset[2]=0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // point 4
        r[3]=CTRL_DEG2RAD*(arm=="left"?10.0:-10.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.3;
        xd[1]=0.0;
        xd[2]=-0.05;
        moveTool(arm,eye,xd,od,offset,25);

        // point 5
        r[3]=CTRL_DEG2RAD*(arm=="left"?45.0:-45.0);
        od=dcm2axis(axis2dcm(r)*R);
        xd[0]=-0.35;
        xd[1]=(arm=="left")?-0.05:0.05;
        xd[2]=0.1;
        offset[1]=(arm=="left")?0.1:-0.1;
        moveTool(arm,eye,xd,od,offset,25);

        // solving
        command.clear();
        command.addVocab(Vocab::encode("find"));
        finderPort.write(command,reply);

        iCartCtrl->restoreContext(context_arm);
        iCartCtrl->deleteContext(context_arm);

        iGaze->restoreContext(context_gaze);
        iGaze->deleteContext(context_gaze);

        return true;
    }

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("karmaMotor")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();

        Property optionG("(device gazecontrollerclient)");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local",("/"+name+"/gaze_ctrl").c_str());

        Property optionL("(device cartesiancontrollerclient)");
        optionL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
        optionL.put("local",("/"+name+"/cart_ctrl/left_arm").c_str());

        Property optionR("(device cartesiancontrollerclient)");
        optionR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
        optionR.put("local",("/"+name+"/cart_ctrl/right_arm").c_str());

        Property optionHL("(device remote_controlboard)");
        optionHL.put("remote",("/"+robot+"/left_arm").c_str());
        optionHL.put("local",("/"+name+"/hand_ctrl/left_arm").c_str());

        Property optionHR("(device remote_controlboard)");
        optionHR.put("remote",("/"+robot+"/right_arm").c_str());
        optionHR.put("local",("/"+name+"/hand_ctrl/right_arm").c_str());

        if (!driverG.open(optionG))
            return false;

        if (!driverL.open(optionL))
        {
            driverG.close();
            return false;
        }

        if (!driverR.open(optionR))
        {
            driverG.close();
            driverL.close();
            return false;
        }

        if (!driverHL.open(optionHL))
        {
            driverG.close();
            driverL.close();
            driverR.close();
            return false;
        }

        if (!driverHR.open(optionHR))
        {
            driverG.close();
            driverL.close();
            driverR.close();
            driverHL.close();
            return false;
        }

        driverG.view(iGaze);
        driverL.view(iCartCtrlL);
        driverR.view(iCartCtrlR);

        visionPort.open(("/"+name+"/vision:i").c_str());
        finderPort.open(("/"+name+"/finder:rpc").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        interrupting=false;
        handUsed="null";
        flipHand=8.0;

        pushHand="selectable";
        toolFrame=eye(4,4);

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        interrupting=true;

        iGaze->stopControl();
        iCartCtrlL->stopControl();
        iCartCtrlR->stopControl();

        stopHand("left");
        stopHand("right");

        return true;
    }

    /************************************************************************/
    bool close()
    {
        driverG.close();
        driverL.close();
        driverR.close();
        driverHL.close();
        driverHR.close();
        visionPort.close();
        finderPort.close();
        rpcPort.close();
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.02;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (!interrupting && (handUsed!="null"))
            shakeHand();

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    KarmaMotor karmaMotor;
    return karmaMotor.runModule(rf);
}



