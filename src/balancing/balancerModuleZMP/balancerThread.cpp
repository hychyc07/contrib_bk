#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>	
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "balancerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

/*The thread is a subclass of the yarp::os::RateThread class. It basically executes de algorithm mean to be run 
by the module. The algorithm's variables and specific thread parameters and ports go in the private data members
and four methods need to be overridden. MyThread::MyThread() which is the constructor, bool threadInit() which 
initializes variables and returns true if successful; void run() which runs the main algorithm; void threadRelease()
which closes and shuts down the thread. The arguments of the thread object instantiation in the configure() method
should be the addresses of (pointers to) the module parameters and ports in the myModule object. In this way, the
thread's parameters and port variables are just references to the original module parameters and ports that were 
initialized in the configure method of the myModule object.*/


balanceThread::balanceThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_leftLeg, string _robot_name, string _local_name, string head_type, bool display, bool ankle_sens) : RateThread(_rate), ddTor(_ddTor), dd_rightLeg(_dd_rightLeg),dd_leftLeg(_dd_leftLeg), robot_name(_robot_name), local_name(_local_name) 
{   /*This is the constructor of the thread. In order to communicate with the module, module parameters must be 
	passed in this fashion. The variables in the balanceThread class which represent the thread's parameters and ports
	should be pointer types and the constructor will initialize them.*/

	//IF I CREATE NEW PORTS I NEED TO DO THAT HERE AND DECLARE THEM IN THE HEADER FILE
	//CONNECTION OF THESE PORTS WITH EXISTING ONES SHOULD ALSO BE DONE HERE. FOR EXAMPLE WITH THE wholeBodyDynamics!!!
	//
	//------------------ INTERFACE INITIALIZATION ---------------------------

	iencs_torso = 0;
	iencs_leftLeg = iencs_rightLeg = 0;
	Opt_display = display;
	Opt_ankles_sens = ankle_sens;

	torso = true;
	knees = false;
	hip = false;
	
	sample_count = new double;
	//--------------- PORTS -----------------------------------------------
	EEWRightLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
	EEWRightLeg->open(string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/right_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str(),"tcp",false);

	EEWLeftLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
	EEWLeftLeg->open(string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/left_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str(),"tcp",false);

	EEWRightAnkle = new BufferedPort<Vector>;
	EEWRightAnkle->open(string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/right_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());

	EEWLeftAnkle = new BufferedPort<Vector>;
	EEWLeftAnkle->open(string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/left_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());

	//Mainly for debugging, this port outstreams the force/torque measured at the end-effector with LEGS and ANKLES sensors.
	force_out_port_right = new BufferedPort<Vector>;
	force_out_port_right->open(string("/"+local_name+"/wrench_trans_right:o").c_str());

	force_out_port_left = new BufferedPort<Vector>;
	force_out_port_left->open(string("/"+local_name+"/wrench_trans_left:o").c_str());

	objPort = new BufferedPort<Vector>;
	objPort->open(string("/"+local_name+"/DSPzmp:o").c_str());
	Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/myiCubGui/objects").c_str());

	objPort2 = new BufferedPort<Vector>;
	objPort2->open(string("/"+local_name+"/DSPzmp2iCubGui:o").c_str());


	InertialVec = new BufferedPort<Vector>;
	InertialVec->open(string("/"+local_name+"/InertialVec:i").c_str());
	Network::connect(string("/"+robot_name+"/inertial").c_str(), string("/"+local_name+"/InertialVec:i").c_str(),"tcp",false);

	//Connecting output of BalancerModule's zmp TO velocityObserver input port to get derivatives.
	Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/zmpVel/pos:i").c_str());

	//Creating/connecting port zmpVelin:i to read in the zmp derivative computed by the velocityObserver module and sent out through the /zmpVel/vel:o port
	zmpVelin = new BufferedPort<Vector>;
	zmpVelin->open(string("/"+local_name+"/zmpVelin:i").c_str());
	Network::connect(string("/zmpVel/vel:o").c_str(), string("/"+local_name+"/zmpVelin:i").c_str());

	//Control signal (Sent velocity)
	torso_ctrl_sig = new BufferedPort<Vector>;
	torso_ctrl_sig->open(string("/"+local_name+"/torso_ctrl_sig:o").c_str());

	//knee control signal
	knee_ctrl_sig = new BufferedPort<Vector>;
	knee_ctrl_sig->open(string("/"+local_name+"/knee_ctrl_sig:o").c_str());

	//system output
	desired_zmp = new BufferedPort<Vector>;
	desired_zmp->open(string("/"+local_name+"/desired_zmp:o").c_str());

	//skin contacts port INPUT PORT
    port_skin_contacts = new BufferedPort<skinContactList>;
	port_skin_contacts->open(string("/"+local_name+"/skinEvent:i").c_str());
	Network::connect(string("/skinManager/skin_events:o").c_str(), string("/"+local_name+"/skinEvent:i").c_str());

	//Contact signal
	contact_sig = new BufferedPort<Vector>;
	contact_sig->open(string("/"+local_name+"/contact_sig:o").c_str());

	//COM Jacobian 
	COM_Jacob_port = new BufferedPort<Matrix>;
	COM_Jacob_port->open(string("/"+local_name+"/COM_Jacob_port:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/com_jacobian:o").c_str(),string("/"+local_name+"/COM_Jacob_port:i").c_str());

	//COM Velocity from foot
	COM_vel_foot_port = new BufferedPort<Vector>;
	COM_vel_foot_port->open(string("/"+local_name+"/COM_vel_foot_port:o").c_str());

	//Whole Body COM
	full_COM_port = new BufferedPort<Vector>;
	full_COM_port->open(string("/"+local_name+"/full_COM_port:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/com:o").c_str(),string("/"+local_name+"/full_COM_port:i").c_str());

	all_velocities = new BufferedPort<Vector>;
	all_velocities->open(string("/"+local_name+"/all_velocities:i").c_str());
	Network::connect(string("/mywholeBodyDynamics/all_velocities:o").c_str(), string("/"+local_name+"/all_velocities:i").c_str());

	COM_foot_port = new BufferedPort<Vector>;
	COM_foot_port->open(string("/"+local_name+"/com_foot:o").c_str());	
	Network::connect(string("/"+local_name+"/com_foot:o").c_str(), string("/comVel/pos:i").c_str());

	Hip_from_foot = new BufferedPort<Vector>;
	Hip_from_foot->open(string("/"+local_name+"/hip_pos_foot:o").c_str());

	Right_Leg = new iCubLegDyn("right");
	Left_Leg = new iCubLegDyn("left");



}

bool balanceThread::threadInit()
{ 	/*threadInit() returns true if the initialization was successful, otherwise it should return false.
	This is	significant because if it returns false the thread will not subsequently be run.
	The thread executes this function when it starts and before "run". This is a 
	good place to perform initialization tasks that need to be done by the thread 
	itself (device drivers initialization, memory allocation etc). If the function 
	returns false the thread quits and never calls "run". */

	//CHECKING INTERFACES

	// Torso Interface
		ddTor->view(iencs_torso);
		ddTor->view(ICtrl);
		ddTor->view(Ivel);

		if((!ddTor) || (!iencs_torso) || (!ICtrl) || (!Ivel))
			return false;

	//Right and Left leg interfaces
		dd_rightLeg->view(iencs_rightLeg);
		dd_rightLeg->view(ICtrl_RL);
		dd_rightLeg->view(Ivel_RL);

		dd_leftLeg->view(iencs_leftLeg);
		dd_leftLeg->view(ICtrl_LL);
		dd_leftLeg->view(Ivel_LL);

		if((!dd_rightLeg) || (!iencs_rightLeg) || (!ICtrl_RL) || (!Ivel_RL) || (!dd_leftLeg) || (!iencs_leftLeg) || (!ICtrl_LL) || (!Ivel_LL))
			return false;		

	//Setting Reference Accelerations
	setRefAcc(iencs_torso, Ivel);
	setRefAcc(iencs_rightLeg,Ivel_RL);
	setRefAcc(iencs_leftLeg, Ivel_LL);
 	
 	//FOR Sending commands to the TORSO
	iencs_torso->getAxes(&nj); 
	command.resize(nj);
	command[0] = 0;
	command[1] = 0;
	command[2] = 0;
	
	
	//FOR sending commands to the legs
	iencs_leftLeg->getAxes(&nj); //Legs have 6 joints from 0 to 5. Knee is NUMBER 3
	command_LL.resize(nj);
	command_LL = 0;

	iencs_rightLeg->getAxes(&nj);
	command_RL.resize(nj);
	command_RL = 0;

	//INITIALIZING counter - for debugging only
	count = 0;

	//INITIALIZING thread variables.
	rot_f = zeros(3,3);
	
		// rot_f(0,0) = rot_f(1,1) = -1;
		// rot_f(2,2) = 1;
		rot_f(1,1) = -1;
		rot_f(2,0) = rot_f(0,2) = 1;
	


	//INITIALIZING zmp vector
	zmp_xy = zeros(2);	

	//INITIALIZING zmpXd desired
	zmpXd = 0.03;

	//GETTING INITIAL RIGHT FOOT ORIENTATION TO BE USED AS GLOBAL REFERENCE FRAME OF THE ROBOT, ASSUMING
	//THAT THE ROBOT IS NEVER MOVING AWAY FROM ITS INITIAL POSITION. 
    // PoseLeftLegIni = EEPLeftLeg->read(); //This is returning Left/Right end effector pose w.r.t. ROOT reference frame
    // PoseRightLegIni = EEPRightLeg->read();

    Kp_T = 15;  //proportional gain for torso.
	Kd_T = 45;	//derivative gain for torso.

	//Ration 1:5
	Kp_K = 2;	//proportional gain for knees.
	Kd_K = 10;	//derivative gain for knees.

	Kp_h = 0.5;	//proportional gain for hip.
	Kd_h = 0.5; //derivative gain for hip.
	
	switch_ctrl = false;
	(*sample_count) = 0;

	zmpd.resize(1);
	zmpd[0]=0;

  	contacto.resize(1);
  	contacto[0]=0;


	iencs_rightLeg->getAxes(&nj); 
	encs_r.resize(nj);

	iencs_leftLeg->getAxes(&nj);
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
	return true;

}

void balanceThread::setRefAcc(IEncoders* iencs, IVelocityControl* ivel)
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

void balanceThread::run()
{ 	/*The run() method is where the algorithm is implemented and will run continuously until some stopping condition
	is met. The thread only runs once so that you don't have to check isStopping() to see if the thread should end.*/

	// if(!Opt_display){

		//********** Computing ZMP ******************************************

		computeZMP(&zmp_xy);
		
		// fprintf(stderr, "TIME: Computing ZMP: %f \n",end_time-start_time);
		fprintf(stderr, "ZMP coordinates: %f %f       %d \n",zmp_xy[0],zmp_xy[1],(int)(torso));
		fprintf(stderr, "ZMP desired%f\n",zmpXd);

		
		//*********************** SENDING ZMP COORDINATES TO GuiBalancer ****************************
			objPort->prepare() = zmp_xy;
			objPort->write();
			// fprintf(stderr, "TIME: Sending ZMP coords to GuiBalancer: %f\n", end_time-start_time);

		//********************** SENDING ZMP COORDS TO BE PLOT BY ICUBGUI *****************************
		 //   	iencs_rightLeg->getEncoders(encs_r.data());
			// PoseRightLeg = Right_Leg->EndEffPose(CTRL_DEG2RAD*encs_r,false);
			// PoseRightLeg.resize(4);
			// PoseRightLeg(3)=1;
			Matrix Trans_lastRot(4,4); Trans_lastRot.zero();
			Trans_lastRot.setSubmatrix(rot_f,0,0);
			Trans_lastRot(3,3) = 1;
			// PoseRightLeg = Hright*PoseRightLeg*SE3inv(Trans_lastRot);
			// Hright*(Right_Leg->getH())*SE3inv(Trans_lastRot);

			Vector zmp_xy_trans(4); zmp_xy_trans.zero();
			zmp_xy_trans.setSubvector(0,zmp_xy);
			zmp_xy_trans(3)=1;
			zmp_xy_trans = Hright*(Right_Leg->getH())*SE3inv(Trans_lastRot)*zmp_xy_trans;
			fprintf(stderr, "zmp_xy after transformation: %s\n", zmp_xy_trans.toString().c_str());
			objPort2->prepare() = zmp_xy_trans.subVector(0,1);
			objPort2->write();

		//*********************** Reading ZMP derivative **********************************************
			if((*sample_count)>=2)
		    zmpVel = zmpVelin->read(true);

			// fprintf(stderr, "TIME: Reading ZMP derivative: %f\n", end_time-start_time);

		/////////////////// CONTROLLERS /////////////////////////////////////////////////////////////////////////////////////////
		
		//Control Law and setting commanded velocity


		//*********************** Obtaining Transformation Matrix from Root to Foot ********************

		Matrix rot_last(3,3); rot_last.zero(); rot_last(0,2) = rot_last(2,0) = 1; rot_last(1,1) = -1;
		Matrix lastRotTrans(4,4); lastRotTrans.zero();
		lastRotTrans.setSubmatrix(rot_last,0,0);
		lastRotTrans(3,3) = 1;

		Matrix H_r_f(4,4); H_r_f.zero();
		H_r_f = Hright * Right_Leg->getH() * lastRotTrans;
		Matrix H_f_r = SE3inv(H_r_f);
		fprintf(stderr, "\n Right foot H_f_r Matrix: \n %s\n", H_f_r.toString().c_str());

		Hip_from_foot->prepare() = H_f_r.getCol(3);
		Hip_from_foot->write();

		//******** COM seen from foot reference frame **************************************************

		Vector *COM_read = full_COM_port->read();
		fprintf(stderr, "WB COM from foot %s\n", (*COM_read).toString().c_str());

		//********************** CONTACT WITH HUMAN DETECTION ******************************************
		skinContactList *skinContacts  = port_skin_contacts->read(false);
		static double start_time = 0;
		bool contact_detected=false;
 		if(skinContacts)
        {
            for(skinContactList::iterator it=skinContacts->begin(); it!=skinContacts->end(); it++){
                if(it->getBodyPart() == RIGHT_ARM)
                	contact_detected=true;
            }
        }

        if(contact_detected) //if CONTACT happened.
         {
        	start_time = yarp::os::Time::now();
        	fprintf(stderr, "Switching to Torso Control Mode\n");

         	torso=true;
        	knees=false;

        	contacto[0]=1;
        }
        else{
        	double end_time = yarp::os::Time::now();
        	if((end_time - start_time)>=5.0 && torso==true){
        		fprintf(stderr, "Switching back to previous control strategy\n");
        		torso=false;
        		knees=true;
        		contacto[0]=0;
        	}
        }
			
			contact_sig->prepare() = contacto;
			contact_sig->write();


		//TORSO CONTROL MODE
		if(torso && (*sample_count)>=2){

			if((zmp_xy[0]>0.05) || (zmp_xy[0]<-0.01)){

				if(zmp_xy[0]<0){
					zmpXd = 0;
				}
				else{
					if(zmp_xy[0]>0){
					zmpXd = 0.03;
					}
				}
				zmpd[0] = zmpXd;  
				switch_ctrl = true;
			}
				
			if(switch_ctrl){ 
				double error_zmp = zmp_xy[0]-zmpXd;
				command[2]=-Kp_T*(error_zmp) + Kd_T*((*zmpVel)[0]);

				if(error_zmp<0.005){
					switch_ctrl = false;
					zmpd[0]=0;
					command[2]=0;
				}
			}

			torso_ctrl_sig->prepare() = command;
			torso_ctrl_sig->write();

			desired_zmp->prepare() = zmpd;
			desired_zmp->write();
		
		}

		//KNEES CONTROL MODE
		if(knees && (*sample_count)>=2){
			if((zmp_xy[0]>0.05) || (zmp_xy[0]<-0.02)){

				if(zmp_xy[0]<0){
					zmpXd = 0;
				}
				else{
					if(zmp_xy[0]>0){
					zmpXd = 0.03;
					}
				}
				zmpd[0] = zmpXd; 
				switch_ctrl = true;
			}
				
			if(switch_ctrl){
				double error_zmp = zmp_xy[0]-zmpXd;
				command_RL[3]=-Kp_K*(error_zmp) + Kd_K*((*zmpVel)[0]);
				command_LL[3]= command_RL[3];

				if(error_zmp<0.005){
					switch_ctrl = false;
					zmpd[0]=0;
					command_RL[3]=0;
					command_LL[3]=0;
				}
			}

			knee_ctrl_sig->prepare() = command_RL;
			knee_ctrl_sig->write();

			desired_zmp->prepare() = zmpd;
			desired_zmp->write();
		}

		// if(hip && (*sample_count)>=2){
		// 	iencs_rightLeg->getAxes(&nj); 
		// 	encs_r.resize(nj);
		// 	iencs_leftLeg->getAxes(&nj); 
		// 	encs_l.resize(nj);

		// 	iencs_rightLeg->getEncoders(encs_r.data());
		// 	iencs_leftLeg->getEncoders(encs_l.data());
		// 	command_LL[0]=-Kp_h*(zmp_xy[0]-zmpXd) + Kd_h*((*zmpVel)[0]);
		// 	command_RL[0]=-Kp_h*(encs_r[0]-encs_l[0]);
		// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Applying limits. With 10 it works better than with 15. 
		// start_time = yarp::os::Time::now();

		// if(command[2] || command_RL[3] || command_LL[3] || command_RL[0] || command_LL[3] > 10){
		// 	if(torso)
		// 		command[2] = 10;
		// 	else{
		// 		if(knees){
		// 			command_RL[3]=10;
		// 			command_LL[3]=10;
		// 		}
		// 		else{
		// 			if(hip){
		// 				command_RL[0]=10;
		// 				command_LL[0]=10;
		// 			}
		// 		}
		// 	}
		// }

		// if(command[2] || command_RL[3] || command_LL[3] || command_RL[0] || command_LL[3] < -10){
		// 	if(torso)
		// 		command[2] = -10;
		// 	else{
		// 		if(knees){
		// 			command_RL[3]=-10;
		// 			command_LL[3]=-10;
		// 		}
		// 		else{
		// 			if(hip){
		// 				command_RL[0]=-10;
		// 				command_LL[0]=-10;
		// 			}
		// 		}
		// 	}
		// }

		if(torso){
			if(command[2]>10)
			{
				command[2]=10;
			}
			else
			{
				if(command[2]<-10)
				{
					command[2]=-10;
				}			
			}
		}

		if(knees){
			//LIMITING RIGHT LEG SPEED
			if(command_RL[3]>10)
			{
				command_RL[3]=10;
			}
			else
			{
				if(command_RL[3]<-10)
				{
					command_RL[3]=-10;
				}			
			}

			//LIMITING LEF LEG SPEED
			if(command_LL[3]>10)
			{
				command_LL[3]=10;
			}
			else
			{
				if(command_LL[3]<-10)
				{
					command_LL[3]=-10;
				}			
			}
		}

		// Sending velocity commands
		if(!Opt_display)
		{
			if(torso){
		 	Ivel->velocityMove(command.data());
     	 	}

		 	if(knees){
		 	Ivel_RL->velocityMove(command_RL.data());
			Ivel_LL->velocityMove(command_LL.data());
			}
		}

		//ARE FEET FLAT after sending velocity commands?
		// areFeetFlat();

		(*sample_count)=(*sample_count+1);
	// }
	
	// VelocityRotoTrans();


}


void balanceThread::computeZMP(Vector* zmp_xy)
{

	if(Opt_ankles_sens)
	{
		fprintf(stderr, "READING F/T FROM ANKLES SENSORS\n");
		F_ext_RL = EEWRightAnkle->read(true);
		F_ext_LL = EEWLeftAnkle->read(true);
	}
	else{
		fprintf(stderr, "READING F/T FROM LEGS SENSORS\n");
	    F_ext_RL = EEWRightLeg->read(true);
	    F_ext_LL = EEWLeftLeg->read(true);
	}

    (*F_ext_RL).setSubvector(0,rot_f*((*F_ext_RL).subVector(0,2))); //rotation of the measured force
    (*F_ext_LL).setSubvector(0,rot_f*((*F_ext_LL).subVector(0,2)));
    (*F_ext_RL).setSubvector(3,rot_f*((*F_ext_RL).subVector(3,5)));
    (*F_ext_LL).setSubvector(3,rot_f*((*F_ext_LL).subVector(3,5)));
    
    force_out_port_right->prepare() = (*F_ext_RL);
    force_out_port_right->write(); //FOR DEBUGGING PROPER TRANSLATION OF F/T MEASUREMENTS

	force_out_port_left->prepare() = (*F_ext_LL);
    force_out_port_left->write();


    //CoP Right Leg
    yRL =  (*F_ext_RL)[3]/(*F_ext_RL)[2];
    xRL = -(*F_ext_RL)[4]/(*F_ext_RL)[2];
    
    //CoP left Leg
    // The separation between the two feet must be computed with the feet kinematics.It should be the 
    // separation between the origin of the right foot and the origin of the left foot.

    //reading feet separation.
   	iencs_rightLeg->getEncoders(encs_r.data());

	PoseRightLeg = Right_Leg->EndEffPose(CTRL_DEG2RAD*encs_r,false);
	PoseRightLeg.resize(3);
	PoseRightLeg.push_back(1);
	PoseRightLeg = Hright*PoseRightLeg;
   	fprintf(stderr, "Right Leg POSE: %s \n",PoseRightLeg.toString().c_str());

   	iencs_leftLeg->getEncoders(encs_l.data());
   	PoseLeftLeg = Left_Leg->EndEffPose(CTRL_DEG2RAD*encs_l,false);
   	PoseLeftLeg.resize(3);
   	PoseLeftLeg.push_back(1);
   	PoseLeftLeg = Hleft*PoseLeftLeg;
   	fprintf(stderr, "Left Leg POSE: %s \n", PoseLeftLeg.toString().c_str());

	separation = fabs(PoseLeftLeg[1]) + fabs(PoseRightLeg[1]);

	fprintf(stderr, "y coordinate left leg wrt root: %f \n",fabs(PoseLeftLeg[1]) );
	fprintf(stderr, "Feet separation: %f \n",separation);

    yLL =  (*F_ext_LL)[3]/(*F_ext_LL)[2]-separation; //including y offset of separation between feet.
    xLL = -(*F_ext_LL)[4]/(*F_ext_LL)[2];

    xDS =   ((*F_ext_RL)[2]*xRL + (*F_ext_LL)[2]*xLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
    yDS =  -((*F_ext_RL)[2]*yRL + (*F_ext_LL)[2]*yLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
               
	(*zmp_xy)[0] = xDS;
	(*zmp_xy)[1] = yDS; 
}

void balanceThread::VelocityRotoTrans()
{
  	double start_time = yarp::os::Time::now();
  	fprintf(stderr, "I must get HERE\n");
	
	Matrix *COM_Jacob_mat = COM_Jacob_port->read(); //COM Jacobian seen from ROOT (Jac_CR)
  	
  	iencs_rightLeg->getEncoders(encs_r.data());

	Jac_FR.setSubmatrix(Right_Leg->GeoJacobian(CTRL_DEG2RAD*encs_r),0,5); //Jacobian from FOOT to ROOT (Jac_FR)
	Matrix H_rl = Hright*Right_Leg->getH(CTRL_DEG2RAD*encs_r); //Rototranslation matrix from FOOT to ROOT.


	//FOOT REFERENCED NEW JACOBIAN
	// fprintf(stderr, "COM_Jacob_mat:\n %s\n",(*COM_Jacob_mat));
	// fprintf(stderr, "COM_Jacob_mat %d\n", COM_Jacob_mat);
	Matrix Jac_cf =  -1*Jac_FR + adjointInv(H_rl,0)*(*COM_Jacob_mat); //*(*COM_Jacob_mat);
	// printMatrix("V_cf",V_cf);
	// fprintf(stderr, "V_cf: %s \n",V_cf.toString().c_str());


	/*FOR DEBUGGING ONYL: In order to verify that the COM velocity from foot reference frame is equal to the velocity computed
	computed with the jacobian, I will take the COM w.r.t. to ROOT and refer it to the right foot. After which I can derivate
	and compare with V_cf. This can be seen as validation of Jacobian-based COM velocity. */
	
	//COM wrt FOOT sent to a port
	Vector *full_COM_read = full_COM_port->read();
	Vector full_COM = (*full_COM_read).subVector(0,2);
	full_COM.push_back(1);
	// fprintf(stderr, "full_COM: %s\n",full_COM.toString().c_str() );

	Vector COM_foot = SE3inv(H_rl,0)*full_COM;
	fprintf(stderr, "COM_foot%s\n", COM_foot.toString().c_str());

	//COM w.r.t FOOT 
	COM_foot_port->prepare() = COM_foot;
	COM_foot_port->write();	 

	
	Vector *full_dq = all_velocities->read();

	//Sending COM w.r.t FOOT speed.
	printf("BEFORE\n");
	Matrix factor(6,6); factor.zero();
	COM_vel_foot_port->prepare() = Jac_cf*(*full_dq);
	COM_vel_foot_port->write();
	printf("AFTER\n");

  	double end_time = yarp::os::Time::now();
  	fprintf(stderr, "Time velocityRotoTrans: %f\n",end_time-start_time );

}

void balanceThread::areFeetFlat()
{ /*This method allows to know if feet are still flat on the floor or they have started to lift up. TO BE IMPLEMENTED*/

	//Read inertial sensor vector from the InertialVec port and extract the gravity vector w.r.t. ROOT. 
	inrtl_reading = InertialVec->read();
	Vector gravity_vec = (*inrtl_reading).subVector(3,5);
	

	//This is returning Left/Right end effector pose w.r.t. ROOT reference frame
   	iencs_rightLeg->getEncoders(encs_r.data());
   	PoseRightLeg = Right_Leg->getH0() * Right_Leg->EndEffPose(CTRL_DEG2RAD*encs_r);

   	iencs_leftLeg->getEncoders(encs_l.data());
   	PoseLeftLeg = Left_Leg->EndEffPose(CTRL_DEG2RAD*encs_l);

	//Check if both vectors are parallel, if not then send a WARNING message saying that feet have been lifted from the ground. 
    Vector cross_LL = cross(gravity_vec,PoseLeftLeg.subVector(3,5));
    Vector cross_RL = cross(gravity_vec,PoseRightLeg.subVector(3,5));

    // fprintf(stderr, "CROSS PRODUCT LEFT LEG: %s\n",cross_LL.toString().c_str());
    // fprintf(stderr, "CROSS PRODUCT RIGHT LEG: %s\n",cross_RL.toString().c_str());
    if(norm(cross_LL)>0.00001 || norm(cross_RL)>0.00001)
    {	
    	fprintf(stderr, "WARNING! Feet have been lifted from the ground\n");
    	fprintf(stderr, "Lifting Index Right Leg: %f    Lifting Index Left Leg: %f\n",norm(cross_RL),norm(cross_LL));
	}
}
void balanceThread::threadRelease()
{	/* Should delete dynamically created data-structures*/

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

	fprintf(stderr, "Closing Inertial input port\n");
	closePort(InertialVec);

	fprintf(stderr, "Closing zmpVel port\n");
	closePort(zmpVelin);

	fprintf(stderr, "Closing torso_ctrl_sig port\n");
	closePort(torso_ctrl_sig);

	fprintf(stderr, "Closing knee_ctrl_sig port\n");
	closePort(knee_ctrl_sig);

	fprintf(stderr, "Closing desired_zmp port\n");
	closePort(desired_zmp);

	fprintf(stderr, "Closing skin events port\n");
	closePort(port_skin_contacts);
	
	// Ivel->stop();
	// Ipos->stop();
	// Ivel_LL->stop();
	// Ivel_RL->stop();

	fprintf(stderr, "Closing full_COM_port\n");
	closePort(full_COM_port);

	fprintf(stderr, "Closing COM_Jacob_port\n");
	closePort(COM_Jacob_port);

	fprintf(stderr, "Closing COM_vel_foot_port\n");
	closePort(COM_vel_foot_port);

	fprintf(stderr, "Closing contact_sig port\n");
	closePort(contact_sig);

	fprintf(stderr, "Closing all_velocities\n");
	closePort(all_velocities);
	
	fprintf(stderr, "Closing COM_foot_port\n"); 
	closePort(COM_foot_port);

	delete sample_count;
	delete Right_Leg;
	delete Left_Leg;
}

void balanceThread::closePort(Contactable *_port)
{
	if(_port)
	{
		_port->interrupt();
		_port->close();

		delete _port;
		_port = 0;
	}
}

void balanceThread::printMatrix(string s, const Matrix &m)
{
         cout<<s<<endl;
         for(int i=0;i<m.rows();i++)
         {
                 for(int j=0;j<m.cols();j++)
                         cout<<setw(15)<<m(i,j);
                 cout<<endl;
         }
 }
