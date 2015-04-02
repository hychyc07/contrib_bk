#ifndef COM_BALANCING
#define COM_BALANCING

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>


#include <iostream>
#include <iomanip>
#include <string.h>
#include <queue>
#include <list>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;


class comBalancingThread: public RateThread
{
public:

    //****************** POLYDRIVERS AND INTERFACES ******************
    PolyDriver *dd_torso;
    PolyDriver *dd_rightLeg;
    PolyDriver *dd_leftLeg;
    
    IPositionControl *Ipos_TO;
    IPositionControl *Ipos_RL;
    IPositionControl *Ipos_LL;
    IControlMode     *Ictrl_TO;
    IControlMode     *Ictrl_LL;
    IControlMode     *Ictrl_RL;
    IVelocityControl *Ivel_TO;
    IVelocityControl *Ivel_LL;
    IVelocityControl *Ivel_RL;
    IEncoders        *Ienc_TO;
    IEncoders        *Ienc_LL;
    IEncoders        *Ienc_RL;
    IPidControl      *Ipid_TO;
    IPidControl      *Ipid_LL;
    IPidControl      *Ipid_RL;
    //****************************************************************

    int nj;

    Vector command;
    Vector command_RL;
    Vector command_LL;
    Vector zmp_xy;
    Vector zmp_xy_vel;
    Vector zmp_des;
    iCub::ctrl::AWLinEstimator      *zmp_xy_vel_estimator;

private:
    string robot_name;
    string local_name;
    string wbsName;
    bool springs;
    bool torso;
    bool verbose;

    //PORTS
    //input ports
    BufferedPort<Vector> *EEWRightLeg;        //EE Wrench Right Leg
    BufferedPort<Vector> *EEWLeftLeg;         //EE Wrench Left Leg
    BufferedPort<Vector> *EEWRightAnkle;      //EE Wrench Right Ankle Sensor
    BufferedPort<Vector> *EEWLeftAnkle;       //EE Wrench Left Ankle Sensor

    BufferedPort<Vector> *objPort;
    BufferedPort<Vector> *objPort2;
    BufferedPort<Matrix> *EEPRightLeg;        //EE Pose Right Leg
    BufferedPort<Matrix> *EEPLeftLeg;         //EE Pose Left Leg
    BufferedPort<Vector> *desired_zmp;        //varying set point.
    BufferedPort<Matrix> *COM_Jacob_port;
    BufferedPort<Vector> *ankle_angle;        //Commanded ankle angle
    BufferedPort<Vector> *COM_ref_port;       //COM_ref 
    BufferedPort<Vector> *port_ft_foot_left;  //Left foot f/t sensor reading
    BufferedPort<Vector> *port_ft_foot_right; //Right foot f/t sensor reading

    double *angle;

    Matrix rot_f;

    //controller gains 
    double Kp_zmp_x, Kp_zmp_y;
    double Kd_zmp_x, Kd_zmp_y;
    double Kp_theta, Kp_phi;
    double Kd_theta, Kd_phi;

    //compute ZMP variables
    double xLL, yLL, xDS, yDS, yRL, xRL;
    Vector *F_ext_RL;
    Vector *F_ext_LL;

    //For plots only
    Vector *F_ext_rf;
    Vector *F_ext_lf;

    //display
    bool Opt_display;

    //Left and Right leg pose.
    Vector PoseLeftLeg;
    Vector PoseRightLeg;

    // To read from Inertial sensor. 
    Vector *inrtl_reading;
    // To read HEAD, RIGHT AND LEFT poses
    Vector *head_pose;

    //Right and left leg encoders
    Vector encs_r;
    Vector encs_l;

    Matrix Jac_FR;             //Jacobian matrix for right FOOT from ROOT.
    iCubLegDyn *Right_Leg;
    iCubLegDyn *Left_Leg;

    Matrix Hright;
    Matrix Hleft;

    bool Opt_ankles_sens;
    
    Stamp zmp_time;

    Vector Offset_Lfoot, Offset_Rfoot;
    

public:
    comBalancingThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_lefLeg,\
                       string _robot_name, string _local_name, string _wbs_name, bool display,\
                       bool ankles_sens, bool springs, bool torso, bool verbose, double Kp_zmp_x, double Kd_zmp_x,\
                       double Kp_zmp_y, double Kd_zmp_y, double Kp_theta, double Kd_theta, double Kp_phi, double Kd_phi);
    void setRefAcc(IEncoders* iencs, IVelocityControl* ivel);
    bool threadInit();
    void run();
    void threadRelease();
    void computeZMP(Vector* zmp_xy, Vector *F_ext_LL, Vector *F_ext_RL, Vector encs_l, Vector encs_r);
    double separation(Vector encs_r, Vector encs_l);
    void closePort(Contactable *_port);
};

#endif
