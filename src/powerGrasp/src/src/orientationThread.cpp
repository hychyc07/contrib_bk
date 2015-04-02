#include "orientationThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

OrientationThread::OrientationThread() : RateThread(20) 
{
    iCtrl=NULL;
    done=true;
    work=false;
}

bool OrientationThread::open(string &hand, string &robot, int &nAngles)
{
    this->nAngles=nAngles;
    this->hand=hand;

    //Opening useful devices
    Property optCtrl;
    optCtrl.put("device","cartesiancontrollerclient");
    optCtrl.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optCtrl.put("local",("/orientation/"+hand+"/cartesianRight").c_str());

    if (!dCtrl.open(optCtrl))
    {
        fprintf(stdout, "%s Cartesian Interface is not open\n", hand.c_str());
        return false;
    }

    dCtrl.view(iCtrl);
    iCtrl->storeContext(&context_in);
    yarp::sig::Vector dof;
    iCtrl->getDOF(dof);
    yarp::sig::Vector newDof=dof;
    newDof[0]=1.0;
    newDof[2]=1.0;

    iCtrl->setDOF(newDof,dof);
    iCtrl->setLimits(7,-70.0,70.0);
    iCtrl->setTrajTime(1.5);
    iCtrl->storeContext(&context);
    iCtrl->restoreContext(context_in);

    Property optArm;
    Property optTorso;

    string remoteArmName="/"+robot+"/"+hand;
    optArm.put("device", "remote_controlboard");
    optArm.put("remote",remoteArmName.c_str());
    optArm.put("local",("/localArm/"+hand).c_str());

    string remoteTorsoName="/"+robot+"/torso";
    optTorso.put("device", "remote_controlboard");
    optTorso.put("remote",remoteTorsoName.c_str());
    optTorso.put("local",("/localTorso/"+hand).c_str());

    robotTorso.open(optTorso);
    robotArm.open(optArm);

    if (!robotTorso.isValid() || !robotArm.isValid())
    {
        fprintf(stdout, "Device not available\n");
        return false;
    }

    robotArm.view(limArm);
    robotTorso.view(limTorso);

    if (hand=="right_arm")
        arm=new iCubArm("right");
    else
        arm=new iCubArm("left");

    chain=arm->asChain();

    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    deque<IControlLimits*> lim;
    lim.push_back(limTorso);
    lim.push_back(limArm);
    arm->alignJointsBounds(lim);

    arm->setAllConstraints(false);

    thetaMin.resize(10,0.0);
    thetaMax.resize(10,0.0);
    for (unsigned int i=0; i< chain->getDOF(); i++)
    {
       thetaMin[i]=(*chain)(i).getMin();
       thetaMax[i]=(*chain)(i).getMax();
    }

    getSampledAngles(angles, nAngles);
    q.resize(10);
    ones.resize(q.size()); ones=1.0;
    q0.resize(10);
    xdhat.resize(3); 
    odhat.resize(4);
    orientation=eye(4,4);

    this->start();
    return true;
}

void OrientationThread::setInfo(yarp::sig::Vector &eePos, yarp::sig::Vector &px, yarp::sig::Vector &py, yarp::sig::Vector &pointNormal, yarp::sig::Vector &center, yarp::sig::Vector &biggerAxis)
{
    this->eePos=eePos;
    this->px=px;
    this->py=py;
    this->pointNormal=pointNormal;
    this->center=center;
    this->biggerAxis=biggerAxis;
    done=false;
    work=true;
}

void OrientationThread::getSampledAngles(yarp::sig::Vector &angles, int nAngles)
{
    angles.resize(nAngles);
    int factor=(int)360/nAngles;
    int num=0;
    for (int i=0; i<nAngles; i++)
    {
        angles[i]=num;
        num+=factor;
    }
}

void OrientationThread::storeContext()
{
    iCtrl->deleteContext(current_context);
    iCtrl->storeContext(&current_context);
    iCtrl->restoreContext(context);
}

void OrientationThread::restoreContext()
{
    iCtrl->restoreContext(current_context);
}

void OrientationThread::run()
{
    manipulability=0.0;
    designedOrientation=eye(4,4);
    result=0.0;
    limits=0.0;
    man=0.0;
    /*if (work)
    {
        if (pointNormal[1]>0.0)
        {
			printf("bad point normal\n");
            work=false;
            done=true;
        }
    }*/
    if (work)
    {
        bool firstAngle=true;

        orientation(0,3)=center[0];
        orientation(1,3)=center[1];
        orientation(2,3)=center[2];
        orientation(0,2)=pointNormal[0];
        orientation(1,2)=pointNormal[1];
        orientation(2,2)=pointNormal[2];

        for (int j=0; j<nAngles; j++)
        {
            orientation(0,0)=px[0]*cos(angles[j])-py[0]*sin(angles[j]);
            orientation(1,0)=px[1]*cos(angles[j])-py[1]*sin(angles[j]);
            orientation(2,0)=px[2]*cos(angles[j])-py[2]*sin(angles[j]);
            orientation(0,1)=px[0]*sin(angles[j])+py[0]*cos(angles[j]);
            orientation(1,1)=px[1]*sin(angles[j])+py[1]*cos(angles[j]);
            orientation(2,1)=px[2]*sin(angles[j])+py[2]*cos(angles[j]);

            yarp::sig::Vector x(3); x[0]=orientation(0,0); x[1]=orientation(1,0); x[2]=orientation(2,0);
            if (dot(x,biggerAxis)>0.6 && dot(x,biggerAxis)<1.0)
                continue;
            if (dot(x,biggerAxis)<-0.6 && dot(x,biggerAxis)>-1.0)
                continue;

            if (hand=="right_arm")
            {
                if (orientation(0,0)>0.0 || orientation(1,2)>0.2 || orientation(2,0)<0.1 || orientation(1,0)<0.1)
                    continue;
            }
            else
            {
                if (orientation(0,0)>0.0 || orientation(1,2)>0.2 || orientation(2,0)<0.1 || orientation(1,0)>0.1)
                    continue;
            }

            od=dcm2axis(orientation);

            q=0.0;       
            man=0.0;

            if (firstAngle)
            {
                iCtrl->askForPose(eePos,od,xdhat,odhat,q);
                firstAngle=false;
                q0=q;
            }
            else
                iCtrl->askForPose(q0,eePos,od,xdhat,odhat,q);
            
            result=dot(q,ones);

            if (result==0.0)
            {
                manipulability=0.0;
                break;
            }

            q=q*M_PI/180.0;
            arm->setAng(q);
            Jacobian=arm->GeoJacobian();
            mulJac=Jacobian*(Jacobian.transposed());

            man=sqrt(det(mulJac));
        
            limits=0.0;
            for (unsigned int k=0; k<thetaMin.size(); k++)
                limits+=(q[k]-thetaMin[k])*(thetaMax[k]-q[k])/((thetaMax[k]-thetaMin[k])*(thetaMax[k]-thetaMin[k]));
        
            man+=(1-exp(-limits));

            if (man>manipulability)
            {
                manipulability=man;
                designedOrientation=orientation;
            }
        }
        done=true;
        work=false;
    }
    this->suspend();
}

void OrientationThread::getBestManipulability(double &manipulability, yarp::sig::Matrix &orientation)
{
    manipulability=this->manipulability;
    orientation=this->designedOrientation;
}

bool OrientationThread::checkDone()
{
    return done;
}

bool OrientationThread::normalDirection(string &hand, yarp::sig::Vector &normal)
{
    if (normal[1]<0 && hand=="left_arm")
        return true;
    if (normal[1]>0 && hand=="right_arm")
        return true;

    return false;
}

void OrientationThread::close()
{
    if (dCtrl.isValid())
    {
        iCtrl->restoreContext(context_in);
        dCtrl.close();
    }
    if (robotArm.isValid())
        robotArm.close();
    if (robotTorso.isValid())
        robotTorso.close();
}

void OrientationThread::threadRelease() 
{

}

