#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>    

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "comBalancing.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

#define FOR_PLOTS_ONLY
#define TH0_LEGS

comBalancingThread::comBalancingThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_leftLeg, string _robot_name, string _local_name, string _wbs_name, bool display, bool ankles_sens, bool _springs, bool _torso, bool _verbose, double Kp_zmp_x, double Kd_zmp_x, double Kp_zmp_y, double Kd_zmp_y, double Kp_theta, double Kd_theta, double Kp_phi, double Kd_phi) : RateThread(_rate), dd_torso(_ddTor), dd_rightLeg(_dd_rightLeg),dd_leftLeg(_dd_leftLeg), robot_name(_robot_name), local_name(_local_name), wbsName(_wbs_name), springs(_springs), torso(_torso), verbose(_verbose), Kp_zmp_x(Kp_zmp_x), Kd_zmp_x(Kd_zmp_x), Kp_zmp_y(Kp_zmp_y), Kd_zmp_y(Kd_zmp_y), Kp_theta(Kp_theta), Kd_theta(Kd_theta), Kp_phi(Kp_phi), Kd_phi(Kd_phi)
{   
    //------------------ INTERFACE INITIALIZATION ----------------------------------
    printf("rate: %d\n",_rate);
    Ipos_TO     = 0;
    Ipos_RL     = 0;
    Ipos_LL     = 0;
    Ictrl_TO    = 0;
    Ictrl_LL    = 0;
    Ictrl_RL    = 0;
    Ivel_TO     = 0;
    Ivel_LL     = 0;
    Ivel_RL     = 0;
    Ienc_TO     = 0;
    Ienc_LL     = 0;
    Ienc_RL     = 0;
    Ipid_TO     = 0;
    Ipid_LL     = 0;
    Ipid_RL     = 0;

    Opt_display = display;
    Opt_ankles_sens = ankles_sens; 

    zmp_xy_vel_estimator = new iCub::ctrl::AWLinEstimator(1,0.0);
    
    //---------------------------- PORTS ---------------------------------------------
    EEWRightLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
    EEWRightLeg->open(string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str());
    Network::connect(string(wbsName+"/right_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str(),"tcp",false);

    EEWLeftLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
    EEWLeftLeg->open(string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str());
    Network::connect(string(wbsName+"/left_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str(),"tcp",false);

    EEWRightAnkle = new BufferedPort<Vector>;
    EEWRightAnkle->open(string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
    Network::connect(string(wbsName+"/right_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());

    EEWLeftAnkle = new BufferedPort<Vector>;
    EEWLeftAnkle->open(string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
    Network::connect(string(wbsName+"/left_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());

    objPort = new BufferedPort<Vector>;
    objPort->open(string("/"+local_name+"/DSPzmp:o").c_str());
    Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/myiCubGui/objects").c_str());

    objPort2 = new BufferedPort<Vector>;
    objPort2->open(string("/"+local_name+"/DSPzmp2iCubGui:o").c_str());

    //system output
    desired_zmp = new BufferedPort<Vector>;
    desired_zmp->open(string("/"+local_name+"/desired_zmp:o").c_str());

    //Connecting output of BalancerModule's zmp TO velocityObserver input port to get derivatives.
    Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/zmpVel/pos:i").c_str());

    //COM Jacobian 
    COM_Jacob_port = new BufferedPort<Matrix>;
    COM_Jacob_port->open(string("/"+local_name+"/COM_Jacob_port:i").c_str());
    Network::connect(string(wbsName+"/com_jacobian:o").c_str(),string("/"+local_name+"/COM_Jacob_port:i").c_str());

    port_ft_foot_left = new BufferedPort<Vector>;
    port_ft_foot_left->open(string("/"+local_name+"/left_foot/FT:i").c_str());
    Network::connect(string("/"+robot_name+"/left_foot/analog:o").c_str(), string("/"+local_name+"/left_foot/FT:i").c_str(), "tcp", false);

    port_ft_foot_right = new BufferedPort<Vector>;
    port_ft_foot_right->open(string("/"+local_name+"/right_foot/FT:i").c_str());
    Network::connect(string("/"+robot_name+"/right_foot/analog:o").c_str(), string("/"+local_name+"/right_foot/FT:i").c_str(),"tcp",false);

    COM_ref_port = new BufferedPort<Vector>;
    COM_ref_port->open(string("/"+local_name+"/com_ref:o").c_str());

    ankle_angle = new BufferedPort<Vector>;
    ankle_angle->open(string("/"+local_name+"/commanded_ankle_ang:o").c_str());

    Right_Leg = new iCubLegDyn("right");
    Left_Leg = new iCubLegDyn("left");
}

bool comBalancingThread::threadInit()
{   
F_ext_LL = 0;
F_ext_RL = 0;

    //POLIDRIVERS AND INTERFACES

    // Torso Interface
    dd_torso->view(Ienc_TO);
    dd_torso->view(Ictrl_TO);
    dd_torso->view(Ivel_TO);
    dd_torso->view(Ipid_TO);
    dd_torso->view(Ipos_TO);
    if((!dd_torso) || (!Ienc_TO) || (!Ictrl_TO) || (!Ivel_TO) || (!Ipid_TO) || (!Ipos_TO))
    {
        printf("ERROR acquiring torso interfaces\n");
        return false;
    }

    //Right leg interfaces
    dd_rightLeg->view(Ienc_RL);
    dd_rightLeg->view(Ictrl_RL);
    dd_rightLeg->view(Ivel_RL);
    dd_rightLeg->view(Ipid_RL);
    dd_rightLeg->view(Ipos_RL);
    if((!dd_rightLeg) || (!Ienc_RL) || (!Ictrl_RL) || (!Ivel_RL) || (!Ipid_RL) || (!Ipos_RL))
    {
        printf("ERROR acquiring right leg interfaces\n");
        return false;
    }

    
    //Left leg interfaces
    dd_leftLeg->view(Ienc_LL);
    dd_leftLeg->view(Ictrl_LL);
    dd_leftLeg->view(Ivel_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipos_LL);
    if((!dd_leftLeg) || (!Ienc_LL) || (!Ictrl_LL) || (!Ivel_LL) || (!Ipid_LL) || (!Ipos_LL))
    {
        printf("ERROR acquiring left leg interfaces\n");
        return false;
    }
     
     //FOR Sending commands to the TORSO
    Ienc_TO->getAxes(&nj); 
    command.resize(nj);
    command[0] = 0.0;
    command[1] = 0.0;
    command[2] = 0.0;
    
    
    //FOR sending commands to the legs
    Ienc_LL->getAxes(&nj); //Legs have 6 joints from 0 to 5.
    command_LL.resize(nj);
    command_LL = 0;
    command_LL[0] = 0.0;

    Ienc_RL->getAxes(&nj);
    command_RL.resize(nj);
    command_RL = 0;
    command_RL[0] = 1.0;
    
    //Setting Reference Accelerations
    setRefAcc(Ienc_TO, Ivel_TO);
    setRefAcc(Ienc_RL, Ivel_RL);
    setRefAcc(Ienc_LL, Ivel_LL);
    

    //INITIALIZING thread variables.
    rot_f = zeros(3,3);
    
        // rot_f(0,0) = rot_f(1,1) = -1;
        // rot_f(2,2) = 1;
        rot_f(1,1) = -1;
        rot_f(2,0) = rot_f(0,2) = 1;
    
    //INITIALIZING zmp vector
    zmp_xy = zeros(2);    

    //INITIALIZING zmpXd desired
    zmp_des.resize(2);
    zmp_des[0] = -0.1;
    zmp_des[1] = 0.0;

    zmp_xy_vel.resize(2);
    zmp_xy_vel[0] = 0.0;
    zmp_xy_vel[1] = 0.0;

// #ifdef STIFF_JNTS
//     Kp_zmp_x = 0.40; Kp_zmp_y = 0;
//     Kd_zmp_x = 0.005; Kd_zmp_y = 0;

// //  Kp_theta = 80.0;  Kp_phi   = 0.0;
// //  Kd_theta =  1.8;  Kd_phi   = 0.0;
    
//     Kp_theta = 0.0;  Kp_phi   = 0.0;
//     Kd_theta = 0.0;  Kd_phi   = 0.0;
// #else
//     Kp_zmp_x = 0.30; Kp_zmp_y = 0;      //Good values with spings 0.45 is original value with 4 springs
//     Kd_zmp_x = 0.08; Kd_zmp_y = 0;      //Good values with springs 0.018

//     Kp_theta = 80.0;  Kp_phi   = 0.0;   //Good values with springs 
//     Kd_theta =  1.0;  Kd_phi   = 0.0;   //Good values with springs
// #endif
  
    //FOR SAFETY
    if(Kd_zmp_x>0.025){
        Kd_zmp_x=0.025;
    }
    if(Kp_zmp_x>0.9){
        Kp_zmp_x=0.9;
    }
    if(Kp_theta>100.0){
        Kp_theta=100;
    }
    if(Kd_theta>2.5){
        Kd_theta=2.5;
    }

    Ienc_RL->getAxes(&nj); 
    encs_r.resize(nj);

    Ienc_LL->getAxes(&nj);
    encs_l.resize(nj);

    Jac_FR.resize(6,32);
    Jac_FR.zero();

    Hright.resize(4,4);
    Hright.zero(); 
    Hright(0,0) = Hright(1,2) = Hright(3,3) = 1;
    Hright(2,1) = -1; Hright(1,3) = 0.0681; Hright(2,3) = -0.1199;
    
    Hleft.resize(4,4);
    Hleft.zero(); 
    Hleft(0,0) = Hleft(1,2) = Hleft(3,3) = 1;
    Hleft(2,1) = -1; Hleft(1,3) = -0.0681; Hleft(2,3) = -0.1199;

  /*  #ifdef FOR_PLOTS_ONLY 
        Vector* tmp;
        tmp = port_ft_foot_right->read(true);
        Offset_Rfoot = *tmp;
        tmp =  port_ft_foot_left->read(true);
        Offset_Lfoot = *tmp;
    #endif */
	
    #ifdef FOR_PLOTS_ONLY 	
	Vector tmp(6);
	tmp[0]=18.129639; tmp[1]= -31.828339; tmp[2]=15.287476; tmp[3]=-0.272552 ; tmp[4]=0.672455 ; tmp[5]=-0.066376;
	Offset_Rfoot = tmp;
	tmp[0]=59.664429; tmp[1]=23.460388; tmp[2]=21.802551; tmp[3]=0.045166; tmp[4]=0.010712; tmp[5]=0.083221;
	Offset_Lfoot = tmp;
    #endif
    
    // Setting initial configuration of entire robot.
    Vector tmp2;
    tmp2.resize(nj);
    for(int i=0; i<nj;i++){
        tmp2[i] = 10.0;
    }
    
    Ipos_RL->setRefSpeeds(tmp2.data());
    Ipos_LL->setRefSpeeds(tmp2.data());
        
    bool ok;
    ok = Ipos_RL->positionMove(command_RL.data());
    if(ok)
        printf("Right Leg set to initial position...\n");
    else
        return false;
        
    ok = Ipos_LL->positionMove(command_LL.data());
    if(ok)
        printf("Left Leg set to initial position...\n");
    else
        return false;
    
    ok = Ipos_TO->positionMove(command.data());
    if(ok)
        printf("Torso set to initial position...\n");
    else
        return false;

    string key;
    printf("Enter to continue ... \n");
    cin.ignore();

    return true;
}

void comBalancingThread::setRefAcc(IEncoders* iencs, IVelocityControl* ivel)
{
    Vector tmp;
    iencs->getAxes(&nj);

    tmp.resize(nj);
    int i;
    for (i=0;i<=nj;i++){
        tmp[i]=800;
    }
    ivel->setRefAccelerations(tmp.data());
}

void comBalancingThread::run()
{    
        //updating Time Stamp
        static Stamp timeStamp;
        timeStamp.update();
        
        //***************************** Reading F/T measurements and encoders ****************

        #ifdef FOR_PLOTS_ONLY 
                Vector* F_ext_RLt = port_ft_foot_right->read(true);
                Vector* F_ext_LLt = port_ft_foot_left->read(true);

                if (F_ext_LL==0) F_ext_LL= new Vector(6,0.0);
                if (F_ext_RL==0) F_ext_RL = new Vector(6,0.0);

                (*F_ext_LL) = -1.0*((*F_ext_LLt) - (Offset_Lfoot));
                (*F_ext_RL) = -1.0*((*F_ext_RLt) - (Offset_Rfoot));

                //Transformation from ankle to F/T sensor
                Matrix foot_hn(4,4); foot_hn.zero();
                foot_hn(0,2)=1; foot_hn(0,3)=-7.75;
                foot_hn(1,1)=-1;
                foot_hn(2,0)=-1;
                foot_hn(3,3)=1;           

                Vector tmp1, tmp2;
                tmp1 = (*F_ext_RL).subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
                tmp2 = (*F_ext_RL).subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;

                for (int i=0; i<3; i++) (*F_ext_RL)[i] = tmp1[i];
                for (int i=3; i<6; i++) (*F_ext_RL)[i] = tmp2[i-3];

                tmp1 = (*F_ext_LL).subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
                tmp2 = (*F_ext_LL).subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;

                for (int i=0; i<3; i++) (*F_ext_LL)[i] = tmp1[i];
                for (int i=3; i<6; i++) (*F_ext_LL)[i] = tmp2[i-3];

        #else
                if(Opt_ankles_sens)
                {
                    F_ext_RL = EEWRightAnkle->read(true);
                    F_ext_LL = EEWLeftAnkle->read(true);

                }
                else{
                    F_ext_RL = EEWRightLeg->read(true);
                    F_ext_LL = EEWLeftLeg->read(true);
                }   
        #endif      


        Ienc_RL->getEncoders(encs_r.data());
        Ienc_LL->getEncoders(encs_l.data());


        // check if the robot is not in contact with the ground
        static bool on_ground = true;
        if ((*F_ext_LL)[0] > 50 &&
            (*F_ext_RL)[0] > 50  )
            {
              on_ground = true;
            }
        else
            {
              on_ground = false;
              for (int jj=0; jj<6; jj++) (*F_ext_LL)[jj] =  (*F_ext_RL)[jj] = 1e-20;
              (*F_ext_LL)[0] =  (*F_ext_RL)[0] = 1e+20;
            }

        //***************************** Computing ZMP ****************************************

        computeZMP(&zmp_xy, F_ext_LL, F_ext_RL, encs_l, encs_r);
        
        if(verbose){
        fprintf(stderr, "ZMP coordinates: %f %f       %d \n",zmp_xy[0],zmp_xy[1],(int)(torso));
        fprintf(stderr, "ZMP desired: %f %f\n",zmp_des[0], zmp_des[1]);
        }

        //*********************** SENDING ZMP COORDINATES TO GuiBalancer *********************
        if (objPort->getOutputCount() > 0)
        {
            objPort->prepare() = zmp_xy;
            objPort->setEnvelope(timeStamp);
            objPort->write();
        }
        
        //********************** SENDING ZMP COORDS TO BE PLOT BY ICUBGUI *****************************
        if (objPort2->getOutputCount() > 0)
        {
            Matrix Trans_lastRot(4,4); Trans_lastRot.zero();
            Trans_lastRot.setSubmatrix(rot_f,0,0);
            Trans_lastRot(3,3) = 1;
            Trans_lastRot(1,3) = -separation(encs_r,encs_l)/2;
            Vector zmp_xy_trans(4); zmp_xy_trans.zero();
            zmp_xy_trans.setSubvector(0,zmp_xy);
            zmp_xy_trans(3)=1;
            zmp_xy_trans = Hright*(Right_Leg->getH())*SE3inv(Trans_lastRot)*zmp_xy_trans;
            objPort2->prepare() = zmp_xy_trans.subVector(0,1);
            objPort2->write();
        }
        //*********************** Reading ZMP derivative **********************************************
        
        iCub::ctrl::AWPolyElement el;
        el.data=zmp_xy;
        el.time=Time::now();
        zmp_xy_vel = zmp_xy_vel_estimator->estimate(el);

        //*********************** Obtaining Transformation Matrix from Root to WRF ********************
        
        Matrix RLeg_RotTrans(4,4); RLeg_RotTrans.zero();
        RLeg_RotTrans = Hright * Right_Leg->getH();

        Matrix lastRotTrans(4,4); 
        lastRotTrans.zero(); 
        lastRotTrans(0,2) = lastRotTrans(2,0) = lastRotTrans(3,3) = 1; 
        lastRotTrans(1,1) = -1;
        lastRotTrans(1,3) = -1*RLeg_RotTrans(1,3);

        Matrix H_r_f(4,4); H_r_f.zero();
        H_r_f = RLeg_RotTrans * lastRotTrans;
        Matrix H_f_r = SE3inv(H_r_f);
        
        //********************** SUPPORT POLYGON DEFINITION ****************************
/*        double bound_fw =  0.10;
        double bound_bw = -0.05;
        double security_offset = 0.02;

            //zmp support polygon
        if((zmp_xy[0]>bound_fw) || (zmp_xy[0]<bound_bw)){
            if(zmp_xy[0]<bound_bw){
                zmp_des[0] = bound_bw + security_offset;
            }
            else{
                if(zmp_xy[0]>bound_fw){
                   zmp_des[0] = bound_fw - security_offset;
                }
            }  
        }*/
        //********************** CONTROL BLOCK 1 **************************************
        double delta_zmp [2];
        delta_zmp [0] = zmp_xy[0] - zmp_des[0];
        delta_zmp [1] = zmp_xy[1] - zmp_des[1];
        double delta_zmp_vel [2];
        delta_zmp_vel[0] = zmp_xy_vel[0] - 0.0;
        delta_zmp_vel[1] = zmp_xy_vel[1] - 0.0;
        double delta_com [2];
        double delta_theta;
        double delta_phi;
        delta_com [0] = + Kp_zmp_x * delta_zmp[0] - Kd_zmp_x * delta_zmp_vel[0];
        delta_com [1] = + Kp_zmp_y * delta_zmp[1] - Kd_zmp_y * delta_zmp_vel[1];
        delta_theta   = - Kp_theta * delta_zmp[0] - Kd_theta * delta_zmp_vel[0];
        delta_phi     = - Kp_phi   * delta_zmp[1] - Kd_phi   * delta_zmp_vel[1];
        
        double der_part =  Kd_zmp_x * delta_zmp_vel[0];
        if(verbose)
        fprintf(stderr, "\n Kd*delta_zmp_vel[0] =  %f , %f \n", der_part, zmp_xy_vel[0]); 

        //********************** CONTROL BLOCK 2 **************************************
        double COM_des[2];
        double phi_des;
        double theta_des;
        double COM_ref[2];
        double phi_ref;
        double theta_ref;
        //From initial standing position
        COM_des[0] = 0.0;
        COM_des[1] = 0.0;
        theta_des  = 0;
        phi_des   = 0;
        COM_ref[0] = delta_com[0] + COM_des[0];
        COM_ref[1] = delta_com[1] + COM_des[1];
        theta_ref  = delta_theta  + theta_des;
        phi_ref    = delta_phi    + phi_des;

        Vector COM_r(2); COM_r[0] = COM_ref[0]; COM_r[1] = COM_ref[1];
        COM_ref_port->prepare() = COM_r;
        COM_ref_port->setEnvelope(timeStamp);
        COM_ref_port->write();

        //********************** CONTROL BLOCK 3 **************************************
        double q_torso_theta        = 0.0;
        double q_torso_phi          = 0.0;
        double q_left_ankle_theta   = 0.0;
        double q_left_ankle_phi     = 0.0;
        double q_right_ankle_theta  = 0.0;
        double q_right_ankle_phi    = 0.0;
        
        double q0_right_leg         = 0.0;
        double q0_left_leg          = 0.0;

        double length_leg = 0.47;       //empirically taken this value from robot standing. COM z coord. from w.r.f.
        if(verbose)
        fprintf(stderr, "COM_ref = %+6.6f ",COM_ref[0]);
        
        q_right_ankle_theta = asin(-COM_ref[0]/length_leg)*CTRL_RAD2DEG;

        //q_right_ankle_phi   = -asin(-COM_ref[1]/length_leg)*CTRL_RAD2DEG;
        q_right_ankle_phi = 0.0;
        q_left_ankle_phi  = 0.0;
        
        if(!torso){
            q_torso_theta = 2.0;
        }            
        else{
            q_torso_theta = theta_ref;       
        }
        

        //********************** SEND COMMANDS   **************************************
            // SATURATORS
            //torso forward/backward
            double limit = 20;
            if (q_torso_theta>limit)  q_torso_theta=limit;
            if (q_torso_theta<-limit) q_torso_theta=-limit;
            //torso right/left
            if (q_torso_phi>limit)  q_torso_phi=limit;
            if (q_torso_phi<-limit) q_torso_phi=-limit;

            //ankle backward/forward
            if (q_right_ankle_theta>limit)  q_right_ankle_theta=limit;
            if (q_right_ankle_theta<-limit) q_right_ankle_theta=-limit; 

            //ankle right/left
            if (q_right_ankle_phi>10)  q_right_ankle_phi=10;
            if (q_right_ankle_phi<-5) q_right_ankle_phi=-5;
            
        //copying movement        
        q_left_ankle_theta = q_right_ankle_theta;
        //q_left_ankle_phi = q_right_ankle_phi;  //WE NEED TO CALIBRATE AGAIN THIS JOINT BECAUSE -2 DEG FOR RIGHT LEG ARE ACTUALLY 0 FOR THE LEFT ONE.
        
        if(verbose){
        fprintf(stderr, "q theta to be sent: %+6.6f ", q_right_ankle_theta);
        fprintf(stderr, "q phi   to be sent: %+6.6f ", q_right_ankle_phi);
        }

        Vector ankle_ang(2); ankle_ang.zero();
        ankle_ang[0]=q_right_ankle_theta;
        ankle_angle->prepare()=ankle_ang;
        ankle_angle->setEnvelope(timeStamp);
        ankle_angle->write();

        if(verbose)
        fprintf(stderr, "q torso to be sent: %+6.6f\n", q_torso_theta);

        #ifdef TH0_LEGS
            q0_right_leg = theta_ref;
            q0_left_leg = q0_right_leg;
        #endif
        
        if(!Opt_display)
        {
            if (on_ground)
            {
                if(torso){
                    Ipid_TO->setReference(2,q_torso_theta);
                    // Ipid_TO->setReference(1,q_torso_phi);
                }

            Ipid_RL->setReference(4,q_right_ankle_theta);
            Ipid_LL->setReference(4,q_left_ankle_theta);
            
            Ipid_RL->setReference(0,q0_right_leg);
            Ipid_LL->setReference(0,q0_left_leg);
            
            //open legs version
            Ipid_RL->setReference(5,q_right_ankle_phi);
            Ipid_LL->setReference(5,q_left_ankle_phi);
            
            //closed leg version
            //Ipid_RL->setReference(5,q_right_ankle_phi);
            //Ipid_RL->setReference(1,-q_right_ankle_phi);
            //Ipid_LL->setReference(5,-q_left_ankle_phi);
            //Ipid_LL->setReference(1,q_left_ankle_phi);
            }
        }
}


void comBalancingThread::computeZMP(Vector* zmp_xy, Vector *F_ext_LL, Vector *F_ext_RL, Vector encs_l, Vector encs_r)
{
    //rotation of the measured force
    (*F_ext_RL).setSubvector(0,rot_f*((*F_ext_RL).subVector(0,2))); 
    (*F_ext_LL).setSubvector(0,rot_f*((*F_ext_LL).subVector(0,2)));
    (*F_ext_RL).setSubvector(3,rot_f*((*F_ext_RL).subVector(3,5)));
    (*F_ext_LL).setSubvector(3,rot_f*((*F_ext_LL).subVector(3,5)));

    // CoP Right Leg
    yRL =  (*F_ext_RL)[3]/(*F_ext_RL)[2];
    xRL = -(*F_ext_RL)[4]/(*F_ext_RL)[2];

    double sep = separation(encs_r,encs_l);
    if(verbose)
    fprintf(stderr, "\n FEET SEPARATION: %f\n", sep);

    // yLL =  (*F_ext_LL)[3]/(*F_ext_LL)[2] - separation(encs_r, encs_l)/2; 
    yLL =  (*F_ext_LL)[3]/(*F_ext_LL)[2];
    xLL = -(*F_ext_LL)[4]/(*F_ext_LL)[2];

    xDS =   ((*F_ext_RL)[2]*xRL + (*F_ext_LL)[2]*xLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
    yDS =  -((*F_ext_RL)[2]*yRL + (*F_ext_LL)[2]*yLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
               
    (*zmp_xy)[0] = xDS;
    (*zmp_xy)[1] = yDS; 
}

double comBalancingThread::separation(Vector encs_r, Vector encs_l){

    PoseRightLeg = Right_Leg->EndEffPose(CTRL_DEG2RAD*encs_r,false);
    PoseRightLeg.resize(3);
    PoseRightLeg.push_back(1);
    PoseRightLeg = Hright*PoseRightLeg;

    PoseLeftLeg = Left_Leg->EndEffPose(CTRL_DEG2RAD*encs_l,false);
    PoseLeftLeg.resize(3);
    PoseLeftLeg.push_back(1);
    PoseLeftLeg = Hleft*PoseLeftLeg;

    double sep;
    sep = fabs(PoseLeftLeg[1]) + fabs(PoseRightLeg[1]);
    return sep;
}

void comBalancingThread::threadRelease()
{    /* Should delete dynamically created data-structures*/

    fprintf(stderr, "Closing EEWRightLeg port \n");
    closePort(EEWRightLeg);

    fprintf(stderr, "Closing EEWLeftLeg port \n");
    closePort(EEWLeftLeg);

    fprintf(stderr, "Closing EEWRightAnkle port\n");
    closePort(EEWRightAnkle);

    fprintf(stderr, "Closing EEWLeftAnkle port\n");
    closePort(EEWLeftAnkle);

    fprintf(stderr, "Closing Object Port \n");
    closePort(objPort);

    fprintf(stderr, "Closing Object2 Port \n");
    closePort(objPort2);

    fprintf(stderr, "Closing desired_zmp port\n");
    closePort(desired_zmp);

    fprintf(stderr, "Closing COM_Jacob_port\n");
    closePort(COM_Jacob_port);
    
    fprintf(stderr, "Closing COM_ref_port\n");
    closePort(COM_ref_port);

    delete Right_Leg;
    delete Left_Leg;
    delete zmp_xy_vel_estimator;
}

void comBalancingThread::closePort(Contactable *_port)
{
    if(_port)
    {
        _port->interrupt();
        _port->close();

        delete _port;
        _port = 0;
    }
}
