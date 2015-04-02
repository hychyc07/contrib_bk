/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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

#include <iCub/controller.h>


/************************************************************************/
Controller::Controller(PolyDriver *_drvTorso, PolyDriver *_drvArm, exchangeData *_commData,
                       const string &_localName, const string &partName, double _execTime,
                       unsigned int _ctrlTorso, unsigned int _ctrlPose, unsigned int _period) :
                       RateThread(_period),   drvTorso(_drvTorso),   drvArm(_drvArm),
                       commData(_commData),   localName(_localName), execTime(_execTime),
                       ctrlTorso(_ctrlTorso), ctrlPose(_ctrlPose),   period(_period),
                       Ts(_period/1000.0)
{
    Robotable=drvTorso&&drvArm;

    // Instantiate iCub arm
    arm=createArm(partName,ctrlTorso>0);

    // Get the chain object attached to the arm
    chain=arm->asChain();

    if (Robotable)
    {
        // create interfaces
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvTorso->view(velTorso);
        ok&=drvArm->view(limArm);
        ok&=drvArm->view(encArm);
        ok&=drvArm->view(velArm);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        // read number of joints
        encTorso->getAxes(&nJointsTorso);
        encArm->getAxes(&nJointsArm);
        nJoints=ctrlTorso ? nJointsTorso+nJointsArm : nJointsArm;
            
        // joints bounds alignment
        deque<IControlLimits*> lim;
        lim.push_back(limTorso);
        lim.push_back(limArm);
        arm->alignJointsBounds(lim);

        // exclude acceleration constraints by fixing
        // thresholds at high values
        if (ctrlTorso)
        {
            Vector a_robTorso(nJointsTorso); a_robTorso=1e9;
            velTorso->setRefAccelerations(a_robTorso.data());
        }

        Vector a_robArm(nJointsArm); a_robArm=1e9;
        velArm->setRefAccelerations(a_robArm.data());
    }
    else
    {
        nJointsTorso=3;
        nJointsArm  =7;
        nJoints=chain->getDOF();
    }

    // re-adjust torso bounds
    switch (ctrlTorso)
    {
    case 1: // only yaw enabled
        (*chain)[0].setMin(0.0);
        (*chain)[0].setMax(0.0);

    case 2: // yaw and pitch enabled
        (*chain)[1].setMin(0.0);
        (*chain)[1].setMax(0.0);
    }

    fb.resize(nJoints,0.0);
    vOld.resize(nJoints,0.0);
}


/************************************************************************/
void Controller::stopLimbsVel()
{
    if (Robotable)
    {
        // this timeout prevents the stop() from
        // being overwritten by the last velocityMove()
        // which travels on a different connection.
        Time::delay(2*Ts);

        velArm->stop();

        if (ctrlTorso)
            velTorso->stop();
    }
}


/************************************************************************/
bool Controller::threadInit()
{
    if (Robotable)
    {
        // arm's starting position
        getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0);
        chain->setAng(fb);
    }

    // Instantiate controller
    ctrl=new MultiRefMinJerkCtrl(*chain,ctrlPose,Ts);

    // Set the task execution time
    execTime=ctrl->set_execTime(execTime,true);

    port_x=new BufferedPort<Vector>;
    string n1=localName+"/x:o";
    port_x->open(n1.c_str());

    port_q=new BufferedPort<Vector>;
    string n2=localName+"/q:o";
    port_q->open(n2.c_str());

    port_v=new BufferedPort<Vector>;
    string n3=localName+"/v:o";
    port_v->open(n3.c_str());

    cout << "Starting Controller at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void Controller::afterStart(bool s)
{
    if (s)
        cout << "Controller started successfully" << endl;
    else
        cout << "Controller did not start" << endl;
}


/************************************************************************/
void Controller::run()
{
    Vector xd,qd;
    
    // get the current target pose (both xd and qd are required)
    commData->getDesired(xd,qd);

    // Introduce the feedback within the control computation.
    if (Robotable)
    {
        if (!getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0))
        {
            cout << endl;
            cout << "Communication timeout detected!" << endl;
            cout << endl;

            suspend();

            return;
        }

        ctrl->set_q(fb);
    }

    // control the arm and dump all available information at rate of 1/100th
    Vector q=CTRL_RAD2DEG*ctrl->iterate(xd,qd,0x0064ffff);
    Vector v=CTRL_RAD2DEG*ctrl->get_qdot();

    // send velocities to the robot
    if (Robotable && !(v==vOld))
    {
        if (ctrlTorso)
        {
            // filled in reversed order
            Vector v_robTorso(nJointsTorso);
            for (int i=0; i<nJointsTorso; i++)
                v_robTorso[nJointsTorso-i-1]=v[i];

            velTorso->velocityMove(v_robTorso.data());
        }

        Vector v_robArm(nJointsArm); v_robArm=0.0;
        unsigned int i1=ctrlTorso ? nJointsTorso : 0;
        unsigned int i2=chain->getDOF();

        for (unsigned int i=i1; i<i2; i++)
            v_robArm[i-i1]=v[i];

        velArm->velocityMove(v_robArm.data());

        vOld=v;
    }

    // send x,q,qdot through YARP ports
    Vector &q1=port_q->prepare();
    Vector &v1=port_v->prepare();
    Vector &x =port_x->prepare();

    if (ctrlTorso)
    {
        q1=q;
        v1=v;
    }
    else
    {
        q1.resize(chain->getN());
        v1.resize(chain->getN(),0.0);

        for (int i=0; i<nJointsTorso; i++)
            q1[i]=CTRL_RAD2DEG*chain->getAng(i);

        for (unsigned int i=nJointsTorso; i<chain->getN(); i++)
        {    
            q1[i]=q[i-nJointsTorso];
            v1[i]=v[i-nJointsTorso];
        }
    }

    x=ctrl->get_x();

    port_x->write();
    port_q->write();
    port_v->write();

    commData->set_q(chain->getAng());    
}


/************************************************************************/
void Controller::threadRelease()
{
    stopLimbsVel();

    port_x->interrupt();
    port_q->interrupt();
    port_v->interrupt();

    port_x->close();
    port_q->close();
    port_v->close();

    delete port_x;
    delete port_q;
    delete port_v;
    delete ctrl;
    delete arm;
}


/************************************************************************/
void Controller::suspend()
{
    stopLimbsVel();

    cout << endl;
    cout << "Controller has been suspended!" << endl;
    cout << endl;

    RateThread::suspend();
}


/************************************************************************/
void Controller::resume()
{
    if (Robotable)
    {    
        getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0);
        ctrl->restart(fb);
    }

    cout << endl;
    cout << "Controller has been resumed!" << endl;
    cout << endl;

    RateThread::resume();
}



