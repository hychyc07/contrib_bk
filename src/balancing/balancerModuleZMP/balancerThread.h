#ifndef BALANCER_THREAD
#define BALANCER_THREAD

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>


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
using namespace iCub::skinDynLib;
using namespace std;


class balanceThread: public RateThread
{
public:
	IPositionControl *Ipos;
	IVelocityControl *Ivel;
	IControlMode *ICtrl;
	IControlMode *ICtrl_LL;
	IControlMode *ICtrl_RL;
	IVelocityControl *Ivel_LL;
	IVelocityControl *Ivel_RL;
	// ICartesianControl *LLCart;

	int count;
	Vector command;
	Vector command_RL;
	Vector command_LL;
	Vector zmp_xy;
   	int nj;

   	bool torso;
   	bool knees;
   	bool hip;


private:
	string robot_name;
	string local_name;
	string head_type;

	//POLYDRIVERS ******************
	PolyDriver *ddTor;

	//Polydrivers LEGS
	PolyDriver *dd_rightLeg;
	PolyDriver *dd_leftLeg;

	// PolyDriver *clientCartCtrlLL;

	//INTERFACES *******************
	IEncoders *iencs_torso;

	//Legs Interfaces
	IEncoders *iencs_rightLeg;
	IEncoders *iencs_leftLeg;

	//PORTS
	//input ports
	BufferedPort<Vector> *EEWRightLeg; //EE Wrench Right Leg
	BufferedPort<Vector> *EEWLeftLeg;  //EE Wrench Left Leg
	BufferedPort<Vector> *EEWRightAnkle;	//EE Wrench Right Ankle Sensor
	BufferedPort<Vector> *EEWLeftAnkle;		//EE Wrench Left Ankle Sensor
	BufferedPort<Vector> *objPort;
	BufferedPort<Vector> *objPort2;
	BufferedPort<Matrix> *EEPRightLeg; //EE Pose Right Leg
	BufferedPort<Matrix> *EEPLeftLeg;  //EE Pose Left Leg
	BufferedPort<Matrix> *HeadPose;      //HEAD Pose
	BufferedPort<Vector> *InertialVec;   //Inertial data from Inertial Sensor port.
	BufferedPort<Vector> *zmpVelin;    	 //First derivative of zmp position
	BufferedPort<Vector> *torso_ctrl_sig; //velocity sent to torso
	BufferedPort<Vector> *knee_ctrl_sig; //velocity sent to knee
   	BufferedPort<Vector> *desired_zmp;   //varying set point.
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_contacts; //for skin events
    BufferedPort<Vector> *contact_sig;
    BufferedPort<Matrix> *COM_Jacob_port;
    BufferedPort<Vector> *COM_vel_foot_port;
    BufferedPort<Vector> *full_COM_port;
    BufferedPort<Vector> *all_velocities;
	BufferedPort<Vector> *COM_foot_port;
	BufferedPort<Vector> *Hip_from_foot; //Hip cartesian position from foot reference frame

	BufferedPort<Vector> *force_out_port_right;
	BufferedPort<Vector> *force_out_port_left;

   	double *angle;

   	Matrix rot_f;

   	//desired ZMPx
   	double zmpXd;

   	//zmp velocity vector and PD gains.
   	Vector *zmpVel;
   	double Kp_T, Kd_T;		//Torso gains.
   	double Kd_K, Kp_K;		//Knees gains.
   	double Kp_h, Kd_h;		//hip gains.


   	//compute ZMP variables
   	double xLL, yLL, xDS, yDS, separation, yRL, xRL;
   	Vector *F_ext_RL;
   	Vector *F_ext_LL;

   	//display
   	bool Opt_display;

   	//Left and Right leg pose.
    Vector PoseLeftLeg;
    Vector PoseRightLeg;

    //Initial Left and Right Leg Matrix
	// Matrix *PoseLeftLegIni;
	// Matrix *PoseRightLegIni;

	// To read from Inertial sensor. 
	Vector *inrtl_reading;
	// To read HEAD, RIGHT AND LEFT poses
	Vector *head_pose;

	//Counter for the velocityObserver
	double *sample_count;

	//Right and left leg encoders
	Vector encs_r;
	Vector encs_l;

	Vector zmpd;
	Vector contacto; 

	bool switch_ctrl;

	Matrix Jac_FR; //Jacobian matrix for right FOOT from ROOT.
	iCubLegDyn *Right_Leg;
	iCubLegDyn *Left_Leg;

	// Matrix *COM_Jacob_mat;

	Matrix Hright;
	Matrix Hleft;

	bool Opt_ankles_sens;

	//output ports
	//These ports should be declared as BufferedPort<Bottle/Vector/matrix> *port_name; depending on the need.
public:
	balanceThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_lefLeg, string _robot_name, string _local_name, string head_type, bool display, bool ankles_sens);
	void setRefAcc(IEncoders* iencs, IVelocityControl* ivel);
	bool threadInit();
	void run();
	void threadRelease();
	void areFeetFlat();
	void VelocityRotoTrans();
	void computeZMP(Vector* zmp_xy);
	void closePort(Contactable *_port);
	void printMatrix(string s, const Matrix &m);
	// float *eye(int N)
};

#endif
