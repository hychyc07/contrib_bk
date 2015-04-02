#include "darwin/PMP.h"
extern "C" {
    #include <gsl/gsl_eigen.h>
}

/**
* Constructor.
*/

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace iCub::iKin;

using namespace Darwin::pmp;

PassiveMotionParadigm::PassiveMotionParadigm(Property *opt_right, Property *opt_left, Property *opt_bimanual, Property *opt_tbg)
{
	// right Arm joints + torso
	q_rightArm.resize(10,0.0);
	qdot_rightArm.resize(10,0.0);
	q_ref_right.resize(10,0.0);
	q_0R.resize(10,0.0);
	q_homeR.resize(10,0.0);

	// left Arm Joints + torso
	q_leftArm.resize(10,0.0);
	qdot_leftArm.resize(10,0.0);
	q_ref_left.resize(10,0.0);
	q_0L.resize(10,0.0);
	q_homeL.resize(10,0.0);

	// Stiffness matrices
	K_right.resize(3,3);
	K_left.resize(3,3);
	K_biman.resize(3,3);
	Kint_right.resize(10,10);
	Kint_left.resize(10,10);
	K_right.zero();
	K_left.zero();
	K_biman.zero();
	Kint_right.zero();
	Kint_left.zero();

	// Admittance matrices
	A_rightArm.resize(10,10);
	A_leftArm.resize(10,10);
	A_biman.resize(10,10);
	A_torso.resize(3,3);
	A_rightArm.eye();
	A_leftArm.eye();
	A_torso.eye();
	A_biman.eye();

	// Virtual Forces matrices
	F_right.resize(3,0.0);
	F_left.resize(3,0.0);

	// Position vectors initialization
	x_0R.resize(3,0.0);
	x_0L.resize(3,0.0);
	x_tgR.resize(3,0.0);
	x_tgL.resize(3,0.0);

	// iKin arms initialization
	Rarm = new iCubArm("right");
	Larm = new iCubArm("left");

	// Tool end point homogeneous coordinates in EE space:
	Tool_R.resize(4,0.0);
	Tool_L.resize(4,0.0);
	Tool_R(3) = 1;
	Tool_L(3) = 1;
	useTool_R = false;
	useTool_L = false;

	// Others
	ref_right_active = 0;
	ref_left_active = 0;
	scale = 1000;

	//set properties
	this->opt_right    = opt_right;
	this->opt_left     = opt_left;
	this->opt_bimanual = opt_bimanual;
	this->opt_tbg      = opt_tbg;

	// set tbg's parameters
	if (opt_tbg->check("T_init"))
		tbg.setT_init(opt_tbg->find("T_init").asDouble());
	if (opt_tbg->check("T_dur"))
		tbg.setT_dur(opt_tbg->find("T_dur").asDouble());
	if (opt_tbg->check("SlopeRamp"))
		tbg.setSlopeRamp(opt_tbg->find("SlopeRamp").asDouble());
	if (opt_tbg->check("alpha"))
		tbg.setAlpha(opt_tbg->find("alpha").asDouble());

	if(!setActiveChain("bimanual"))
		printf("Error: no chain initialized\n");

	printf("--> PMP object created. Default active chain is bimanual.\n");

	//save data
		tg.open("tg.txt");
		pos.open("pos.txt");
		Force.open("F.txt");
		joints.open("q.txt");
		jvel.open("qdot.txt");
		eigV.open("ev.txt");
		eigL.open("el.txt");
		G.open("gamma.txt");
		proj.open("v.txt");
		mom.open("RPY.txt");

	// compensation
		tg_old.resize(3,0.0);
		dv_old = 0.0;
}

bool PassiveMotionParadigm::setActiveChain (const string &_side)
{
	side = _side;
	if(!initializeChain(side))
	{
		printf("Error: PMP chain selection failed\n");
		return false;
	}
	printf("--> PMP %s chain selected as active\n", side.c_str());
	return true;
}

bool PassiveMotionParadigm::initializeChain(string side)
{
	Vector temp(10);

	if (side == "right")
	{
		if(opt_right->check("qr_initial"))
		{
			setFromProperty(opt_right, "qr_initial", q_0R);
			q_homeR = q_0R;
			q_0R = q_0R*CTRL_DEG2RAD;
			q_rightArm = q_0R;
		}
		if(opt_right->check("A_rightArm"))
			setFromProperty(opt_right, "A_rightArm", A_rightArm);
		if(opt_right->check("Kr_virt"))
			setFromProperty(opt_right, "Kr_virt", K_right);
		if(opt_right->check("Kr_int"))
			setFromProperty(opt_right, "Kr_int", Kint_right);		
		if(opt_right->check("q_ref_right"))
		{
			setFromProperty(opt_right,"q_ref_right", temp);
			set_q_ref_right(temp);
		}	
		
		initIkinChain("right");		

		// update EE initial position values
		x_0R = get_EEPose("right");
		cout << "Initial EE position:" << endl;
		cout << "right: " << x_0R.toString() << endl;
		cout << "left: " << x_0L.toString() << endl;

		return true;
	}
	if (side == "left" )
	{
		if(opt_left->check("ql_initial"))
		{
			setFromProperty(opt_left, "ql_initial", q_0L);
			q_homeL = q_0L;
			q_0L = q_0L*CTRL_DEG2RAD;
			q_leftArm = q_0L;
		}
		if(opt_left->check("A_leftArm"))
			setFromProperty(opt_left, "A_leftArm", A_leftArm);
		if(opt_left->check("Kl_virt"))
			setFromProperty(opt_left, "Kl_virt", K_left);
		if(opt_left->check("Kl_int"))
			setFromProperty(opt_left, "Kl_int", Kint_left);
		if(opt_left->check("q_ref_left"))
		{
			setFromProperty(opt_left,"q_ref_left", temp);
			set_q_ref_left(temp);
		}

		initIkinChain("left");

		// update EE initial position values
		x_0L = get_EEPose("left");
		cout << "Initial EE position:" << endl;
		cout << "right: " << x_0R.toString() << endl;
		cout << "left: " << x_0L.toString() << endl;

		return true;
	}
	if (side == "bimanual")
	{
		if(opt_right->check("qr_initial"))
		{
			setFromProperty(opt_right, "qr_initial", q_0R);
			q_homeR = q_0R;
			q_0R = q_0R*CTRL_DEG2RAD;
			q_rightArm = q_0R;
		}
		if(opt_left->check("ql_initial"))
		{
			setFromProperty(opt_left, "ql_initial", q_0L);
			q_homeL = q_0L;
			q_0L = q_0L*CTRL_DEG2RAD;
			q_leftArm = q_0L;
			
		}

		if(opt_right->check("Kr_int"))
			setFromProperty(opt_right, "Kr_int", Kint_right);
		if(opt_left->check("Kl_int"))
			setFromProperty(opt_left, "Kl_int", Kint_left);

		if(opt_right->check("q_ref_right"))
		{
			setFromProperty(opt_right,"q_ref_right", temp);
			set_q_ref_right(temp);
		}
		if(opt_left->check("q_ref_left"))
		{
			setFromProperty(opt_left,"q_ref_left", temp);
			set_q_ref_left(temp);
		}

		if(opt_bimanual->check("A_bimanual"))
		{
			setFromProperty(opt_bimanual, "A_bimanual", A_biman);
			set_A_torso(A_biman.submatrix(0,2,0,2));
		}

		if(opt_bimanual->check("K_virt_bimanual"))
			setFromProperty(opt_bimanual, "K_virt_bimanual", K_biman);

		initIkinChain("bimanual");

		// update EE initial position values
		x_0R = get_EEPose("right");
		x_0L = get_EEPose("left");

		cout << "Initial EE position:" << endl;
		cout << "right: " << x_0R.toString() << endl;
		cout << "left: " << x_0L.toString() << endl;

		return true;
	}

	return false;
} 


bool PassiveMotionParadigm::initIkinChain(string side)
{
	if(side == "right")
	{
		// init active ikin chain:
		Rchain = Rarm->asChain();
		
		for (int i=0; i < (int)Rchain->getN(); i++)
			Rchain->releaseLink(i);

		Rchain->setAng(q_rightArm);
		return true;
	}

	if(side == "left")
	{
		// init active ikin chain:
		Lchain = Larm->asChain();

		for (int i=0; i < (int)Lchain->getN(); i++)
			Lchain->releaseLink(i);

		Lchain->setAng(q_leftArm);
		return true;
	}

	if(side == "bimanual")
	{
		// init active ikin chain:
		Rchain = Rarm->asChain();
		Lchain = Larm->asChain();

		for (int i=0; i < (int)Rchain->getN(); i++)
		{
			Rchain->releaseLink(i);
			Lchain->releaseLink(i);
		}

/*
		double q[10] = {-1, 0, 12, -29,31,8,34,73,0,0};
		Vector qq(10,q);

		Rchain->setAng(qq*CTRL_DEG2RAD);
		
		cout << "kinematics: " << getPose("right").toString() << endl;
*/
		Rchain->setAng(q_rightArm);
		Lchain->setAng(q_leftArm);

		return true;
	}

	return false;
} 

static bool ReadTarget()
{/*
	// conversione da robot a simulatore
	Matrix Trs(4,4);
	Trs.zero();
	Trs(0,1) = -1;   // da modificare con i nuovi valori
	Trs(1,2) = 1;
	Trs(1,3) = 0.5976;
	Trs(2,0) = -1;
	Trs(2,3) = -0.026;
	Trs(3,3) = 1;
	Matrix Tsr(4,4);
	Tsr = luinv(Trs);	

	Bottle *tg = inPort.read(false);

	if(!tg)
		return false;

	if(tg->size() != 6)
	{
		printf("ERROR: wrong coordinates number\n");
		printf("Values have to match x y z for right target and x y z for left one\n");
		printf("If you want to disable one side of the chain just send 0 0 0 to it\n");

		return false;
	}
	else
	{
	//tg = Tsr*tg;
		// parse input: set kinematic chain

		x_tgR(0) = tg->get(0).asDouble();
		x_tgR(1) = tg->get(1).asDouble();
		x_tgR(2) = tg->get(2).asDouble();

		x_tgL(0) = tg->get(3).asDouble();
		x_tgL(1) = tg->get(4).asDouble();
		x_tgL(2) = tg->get(5).asDouble();

		cout << "target tot: " << x_tgR.toString() << x_tgL.toString() << endl;

		if (x_tgR(0) == 0.0 && x_tgR(1) == 0.0 && x_tgR(2) == 0.0)
		{
			// one arm reaching
			K(0,0) = 0.26;
			K(1,1) = 0.26;
			K(2,2) = 0.22;
			A_torso = A_torso*0.00001*0.1;
			A_rightArm = A_rightArm*0.0007;
			//A_rightArm(0,0) = 0.0001;//0.0001;
			A_rightArm(1,1) = 0.0;
			A_rightArm(2,2) = 0.0002;// 0.0009;
			A_rightArm(3,3) = 0.0009;
			A_rightArm(4,4) = 0.0001;
			A_rightArm(9,9) = 0.0001;
			A_rightArm(8,8) = 0.001;
			A_rightArm(7,7) = 0.0009;
			A_rightArm(6,6) = 0.0015;
			//A_rightArm(1,1) = 0.001; // for best results
			A_leftArm = A_rightArm;
			A_torso(0,0) = A_rightArm(0,0);
			A_torso(1,1) = A_rightArm(1,1);
			A_torso(2,2) = A_rightArm(2,2);

			side = "left";
		}

		else if (x_tgL(0) == 0.0 && x_tgL(1) == 0.0 && x_tgL(2) == 0.0)
		{
			// one arm reaching
			K(0,0) = 0.26;
			K(1,1) = 0.26;
			K(2,2) = 0.22;
			//A_torso = A_torso*0.00001*0.1;
			A_rightArm = A_rightArm*0.0007;
			//A_rightArm(0,0) = 0.0001;//0.0001;
			A_rightArm(1,1) = 0.0;
			A_rightArm(2,2) = 0.0002;// 0.0009;
			A_rightArm(3,3) = 0.0009;
			A_rightArm(4,4) = 0.0001;
			A_rightArm(9,9) = 0.0001;
			A_rightArm(8,8) = 0.001;
			A_rightArm(7,7) = 0.0009;
			A_rightArm(6,6) = 0.0015;
			//A_rightArm(1,1) = 0.001; // for best results
			A_leftArm = A_rightArm;
			A_torso(0,0) = A_rightArm(0,0);
			A_torso(1,1) = A_rightArm(1,1);
			A_torso(2,2) = A_rightArm(2,2);

			side = "right";
		}

		else
		{
			side = "bimanual";

			K(0,0) = 0.15;
			K(1,1) = 0.15;
			K(2,2) = 0.15;

			A_rightArm.eye();
			A_rightArm = A_rightArm*0.0006;
			A_rightArm(0,0) = 0.00002;//0.0001;
			A_rightArm(1,1) = 0.0;
			A_rightArm(2,2) = 0.000002;// 0.0009;
			A_rightArm(3,3) = 0.0008;
			A_rightArm(4,4) = 0.0008;
			A_rightArm(6,6) = 0.0009;
			//A_rightArm(9,9) = 0.0001;
			//A_rightArm(8,8) = 0.001;
			//A_rightArm(7,7) = 0.0008;
			
			//A_rightArm(1,1) = 0.001; // for best results
			A_leftArm = A_rightArm;
			A_torso(0,0) = A_rightArm(0,0);
			A_torso(1,1) = A_rightArm(1,1);
			A_torso(2,2) = A_rightArm(2,2);
*/
/*
			// bimanual reaching
			A_rightArm = A_rightArm*0.00002;
			A_rightArm(0,0) = 0.0002;//0.0001;
			A_rightArm(1,1) = 0.0;
			A_rightArm(2,2) = 0.0002;// 0.0009;
			A_rightArm(3,3) = 0.0008;
			A_rightArm(4,4) = 0.0007;
			A_rightArm(6,6) = 0.001;
			//A_rightArm(9,9) = 0.0001;
			//A_rightArm(8,8) = 0.001;
			//A_rightArm(7,7) = 0.0008;
			
			//A_rightArm(1,1) = 0.001; // for best results
			A_leftArm = A_rightArm;
			A_torso(0,0) = A_rightArm(0,0);
			A_torso(1,1) = A_rightArm(1,1);
			A_torso(2,2) = A_rightArm(2,2);
*/
//		}
/*
		K(0,0) = 0.26;
			K(1,1) = 0.26;
			K(2,2) = 0.22;
			A_torso = A_torso*0.00001*0.1;
			A_rightArm = A_rightArm*0.0007;
			A_rightArm(0,0) = 0.0001;//0.0001;
			A_rightArm(1,1) = 0.0;
			A_rightArm(2,2) = 0.0002;// 0.0009;
			A_rightArm(3,3) = 0.0009;
			A_rightArm(4,4) = 0.0001;
			A_rightArm(9,9) = 0.0001;
			A_rightArm(8,8) = 0.001;
			A_rightArm(7,7) = 0.0009;
			A_rightArm(6,6) = 0.0015;
			//A_rightArm(1,1) = 0.001; // for best results
			A_leftArm = A_rightArm;
			A_torso(0,0) = A_rightArm(0,0);
			A_torso(1,1) = A_rightArm(1,1);
			A_torso(2,2) = A_rightArm(2,2);
*/
/*
		//cout << side << endl;

		// update current EE position:

		return true;
	}

	//double xr[4] = {-100,-450,250,1};
	//double xl[4] = {100,-450,250,1};
*/
}


/**
 * Destructor.
*/
PassiveMotionParadigm::~PassiveMotionParadigm()
{
	tg.close();
	pos.close();
	Force.close();
	joints.close();
	jvel.close();
	eigV.close();
	eigL.close();
	G.close();
	proj.close();
	mom.close();
}

// ------- get and set methods--------------------

Vector PassiveMotionParadigm::get_q_rightArm()
{
	return q_rightArm;
}

Vector PassiveMotionParadigm::get_q_leftArm()
{
	return q_leftArm;
}

Matrix PassiveMotionParadigm::get_A_rightArm()
{
	return A_rightArm;
}

Matrix PassiveMotionParadigm::get_A_leftArm()
{
	return A_leftArm;
}

Matrix PassiveMotionParadigm::get_A_biman()
{
	return A_biman;
}

Matrix PassiveMotionParadigm::get_A_torso()
{
	return A_torso;
}

Vector PassiveMotionParadigm::get_F_right()
{
	return F_right;
}

Vector PassiveMotionParadigm::get_F_left()
{
	return F_left;
}

Matrix PassiveMotionParadigm::get_K_right()
{
	return K_right;
}

Matrix PassiveMotionParadigm::get_K_left()
{
	return K_left;
}

Matrix PassiveMotionParadigm::get_K_biman()
{
	return K_biman;
}

Matrix PassiveMotionParadigm::get_Kint_right()
{
	return Kint_right;
}

Matrix PassiveMotionParadigm::get_Kint_left()
{
	return Kint_left;
}

Vector PassiveMotionParadigm::get_q_ref_right()
{
	return q_ref_right;
}

Vector PassiveMotionParadigm::get_q_ref_left()
{
	return q_ref_left;
}

string PassiveMotionParadigm::getActiveChain()
{
	return side;
}
/*End get Methods*/
void   PassiveMotionParadigm::set_q_rightArm (const Vector &_q_rightArm)
{
	q_rightArm = _q_rightArm;
	Rchain->setAng(q_rightArm);

	// check if any tool is connected: if so, use the tool end-point as starting point for computation:
	x_0R = get_EEPose("right");
	//x_0R = getPose("right");
}

void   PassiveMotionParadigm::set_q_leftArm  (const Vector &_q_leftArm)
{
	q_leftArm = _q_leftArm;
	Lchain->setAng(q_leftArm);

	// check if any tool is connected: if so, use the tool end-point as starting point for computation:
	x_0L = get_EEPose("left");
	//x_0L = getPose("left");
}

void   PassiveMotionParadigm::set_A_rightArm (const Matrix &_A_rightArm)
{
	A_rightArm = _A_rightArm;
}

void   PassiveMotionParadigm::set_A_rightArm (const Vector &_A_rightArm)
{
	setEyeMatrixFromVector(_A_rightArm,A_rightArm);
}

void   PassiveMotionParadigm::set_A_leftArm  (const Matrix &_A_leftArm)
{
	A_leftArm = _A_leftArm;
}

void   PassiveMotionParadigm::set_A_leftArm  (const Vector &_A_leftArm)
{
	setEyeMatrixFromVector(_A_leftArm, A_leftArm);
}

void   PassiveMotionParadigm::set_A_biman    (const Matrix &_A_biman)
{
	A_biman = _A_biman;
	// Update values in A_torso
	A_torso(0,0) = A_biman(0,0);
	A_torso(1,1) = A_biman(1,1);
	A_torso(2,2) = A_biman(2,2);
}

void   PassiveMotionParadigm::set_A_biman    (const Vector &_A_biman)
{
	setEyeMatrixFromVector(_A_biman, A_biman);
	// Update values in A_torso
	A_torso(0,0) = A_biman(0,0);
	A_torso(1,1) = A_biman(1,1);
	A_torso(2,2) = A_biman(2,2);
}

void   PassiveMotionParadigm::set_A_torso    (const Matrix &_A_torso)
{
	A_torso = _A_torso;
	// Copy values in other matrices
	A_rightArm(0,0) = A_torso(0,0);
	A_rightArm(1,1) = A_torso(1,1);
	A_rightArm(2,2) = A_torso(2,2);
	A_leftArm(0,0)  = A_torso(0,0);
	A_leftArm(1,1)  = A_torso(1,1);
	A_leftArm(2,2)  = A_torso(2,2);
	A_biman(0,0)    = A_torso(0,0);
	A_biman(1,1)    = A_torso(1,1);
	A_biman(2,2)    = A_torso(2,2);
	
}

void   PassiveMotionParadigm::set_A_torso    (const Vector &_A_torso)
{
	setEyeMatrixFromVector(_A_torso, A_torso);
	// Copy values in other matrices
	A_rightArm(0,0) = A_torso(0,0);
	A_rightArm(1,1) = A_torso(1,1);
	A_rightArm(2,2) = A_torso(2,2);
	A_leftArm(0,0)  = A_torso(0,0);
	A_leftArm(1,1)  = A_torso(1,1);
	A_leftArm(2,2)  = A_torso(2,2);
	A_biman(0,0)    = A_torso(0,0);
	A_biman(1,1)    = A_torso(1,1);
	A_biman(2,2)    = A_torso(2,2);
	
}

void   PassiveMotionParadigm::set_F_right    (const Vector &_F_right)
{
	F_right = _F_right;
}

void   PassiveMotionParadigm::set_F_left     (const Vector &_F_left)
{
	F_left = _F_left;
}

void   PassiveMotionParadigm::set_K_right    (const Matrix &_K_right)
{
	K_right = _K_right;
}

void   PassiveMotionParadigm::set_K_right    (const Vector &_K_right)
{
	setEyeMatrixFromVector(_K_right, K_right);
}

void   PassiveMotionParadigm::set_K_left     (const Matrix &_K_left)
{
	K_left = _K_left;
}

void   PassiveMotionParadigm::set_K_left     (const Vector &_K_left)
{
	setEyeMatrixFromVector(_K_left, K_left);
}

void   PassiveMotionParadigm::set_K_biman    (const Matrix &_K_biman)
{
	K_biman = _K_biman;
}

void   PassiveMotionParadigm::set_K_biman    (const Vector &_K_biman)
{
	setEyeMatrixFromVector(_K_biman, K_biman);
}

void   PassiveMotionParadigm::set_Kint_right (const Matrix &_Kint_right)
{
	Kint_right = _Kint_right;
}

void   PassiveMotionParadigm::set_Kint_right (const Vector &_Kint_right)
{
	setEyeMatrixFromVector(_Kint_right, Kint_right);
}

void   PassiveMotionParadigm::set_Kint_left  (const Matrix &_Kint_left)
{
	Kint_left = _Kint_left;
}

void   PassiveMotionParadigm::set_Kint_left  (const Vector &_Kint_left)
{
	setEyeMatrixFromVector(_Kint_left, Kint_left);
}

void   PassiveMotionParadigm::set_q_ref_right(const Vector &_q_ref_right)
{
	q_ref_right = _q_ref_right;
	ref_right_active = 1;
}

void   PassiveMotionParadigm::set_q_ref_left (const Vector &_q_ref_left)
{
	q_ref_left = _q_ref_left;
	ref_left_active = 1;
}


void   PassiveMotionParadigm::set_Tool  (const yarp::sig::Vector &_ToolEE, const string &_side, bool connect)
{
	if (side == "right")
	{
		for (int i=0; i<3; i++)
			Tool_R(i) = _ToolEE(i);
		connect? useTool_R = true:false;
		cout << side << " tool active: " << std::boolalpha << useTool_R << endl;
	}

	if (side == "left")
	{
		for (int i=0; i<3; i++)
			Tool_L(i) = _ToolEE(i);
		connect? useTool_L = true:false;
		cout << side << " tool active: " << std::boolalpha << useTool_L << endl;
	}

}

/*End Set methods*/
// ---------- PMP methods ---------------------------------------------------- */
void PassiveMotionParadigm::get_Eigens(const Matrix &system, Matrix & eigVec, Vector & eigVal)
{
	gsl_matrix * H = (gsl_matrix *) (system.getGslMatrix());
	 
	gsl_vector_complex *evalC = gsl_vector_complex_alloc (system.rows());
    gsl_matrix_complex *evecC = gsl_matrix_complex_alloc (system.rows(), system.rows());
     
    gsl_eigen_nonsymmv_workspace * w = gsl_eigen_nonsymmv_alloc (3);
    gsl_eigen_nonsymmv (H, evalC, evecC, w);
    gsl_eigen_nonsymmv_free (w);
     
	int r =    gsl_eigen_nonsymmv_sort (evalC, evecC, GSL_EIGEN_SORT_ABS_DESC); 

	// get real parts of vectors and matrices of first eigenVector and value:
	gsl_vector_view e_img = gsl_vector_complex_real(evalC);
			
	gsl_vector_view eval = gsl_vector_complex_real (evalC);

	gsl_vector_complex_view evecC_0 = gsl_matrix_complex_column(evecC, 0);
	gsl_vector_view evec_0 = gsl_vector_complex_real (&evecC_0.vector);
	

	// save eigenvectors plus eigenvalues:
     
       for (unsigned int i = 0; i < 3; i++)
       {
		  gsl_vector_complex_view evecC_i = gsl_matrix_complex_column(evecC, i);
		  gsl_vector_view evec_i = gsl_vector_complex_real (&evecC_i.vector);
          eigVal(i) = gsl_vector_get (&eval.vector, i);
         
		  eigVec(0,i) = gsl_vector_get(&evec_i.vector,0);
		  eigVec(1,i) = gsl_vector_get(&evec_i.vector,1);
		  eigVec(2,i) = gsl_vector_get(&evec_i.vector,2);
	   }
	   eigV << eigVec.toString() << endl;
	   eigL << eigVal.toString() << endl;

	return;	
}

Vector PassiveMotionParadigm::distort_delta(const Vector &delta, const Vector &delta_id, const Vector &delta_re, const Matrix &H, const double & Gamma)
{	
	Matrix eigVec(3,3);
	Vector eigVal(3);
	get_Eigens(H, eigVec, eigVal);

	if (delta_id(0) == 0 && delta_id(1) == 0 && delta_id(2) == 0) return delta;
	if (delta(0)	== 0 && delta(1)	== 0 && delta(2)	== 0) return delta;

	// compute projection of delta along the eigenvectors:
	// pick out the one for whom the projection is bigger
	// NO:(and has the minimal projection along the normal to delta and the virtual trajectory displacement)

	Vector dx_m(3);
	//Vector dx_n(3);
	//double n_max = 0.9;
	double n_delta = norm(delta);
	//Vector n = cross(delta_id,delta)/(n_delta*norm(delta_id));
	
		int dx_max = 0;
		//dx_m(0) = fabs(dot(delta/n_delta,eigVec.getCol(0)));
		//dx_n(0) = fabs(dot(n,eigVec.getCol(0)));

		for (unsigned int i=1; i<3; i++)
		{
			dx_m(i) = fabs(dot(delta/n_delta,eigVec.getCol(i)));
			//dx_n(i) = fabs(dot(n,eigVec.getCol(i)));
			//if (dx_m(i) > dx_m(dx_max) && dx_n(i) < n_max)	dx_max = i;
			if (dx_m(i) > dx_m(dx_max))	dx_max = i;
		}

	// for the selected one, choose the right sense
		Vector v(3);
		double v_m;

		if ( norm( delta + dot(delta,eigVec.getCol(dx_max))*eigVec.getCol(dx_max) )	  > 
			 norm( delta - dot(delta,eigVec.getCol(dx_max))*eigVec.getCol(dx_max) )
			)	
			v = eigVec.getCol(dx_max);
		else
			v = -1*eigVec.getCol(dx_max);		
		
		v_m = dot(delta, v);

	// gain control law:
	// nonlinear gain profile to compensate low initial force intensity: adapt the stiffness
	// g prop lambda_max/lambda_i
		double l_max = max(eigVal(0),max(eigVal(1),eigVal(2)));
		double grate = (( l_max - eigVal(dx_max) )/l_max);
		double Tg = 0.01;
		double Kg = 1;

		//double g =  1/(Tg+tbg.getSlopeRamp())*(Tg*grate_old + grate*tbg.getSlopeRamp()*Kg);
		double g = grate;
		grate_old = g;

		//cout << g << endl;

		double Ti = 0.0001;
		double Ki = 50;
/*
		g = Ti/(Ti+tbg.getSlopeRamp())*(g_old + g*tbg.getSlopeRamp());
		//g *= Ki/Ti;
		g_old = g;
*/
	
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double gam;
		gam = 1/(Ti+tbg.getSlopeRamp())*(Ti*g_old + 1/(Gamma+0.0001)*tbg.getSlopeRamp()*Ki);
		if (g_old!= 0.0 && gam > g_old)
		{
			Ki= 70;
			gam = 1/(Ti+tbg.getSlopeRamp())*(Ti*g_old + 1/(2*g_turn-Gamma)*tbg.getSlopeRamp()*Ki);
			cout << 1;
			//return g*delta;
		}
		else	g_turn = Gamma;
	
		//g+= gam;
		g=gam;
		proj << g << endl;
		//G << Gamma << endl;
		g_old = g;

	// angle control law:
	// enlarge the projection of delta along the eigenvector (v):
	// compute filtered angle according to the filter: Kr/(1+sT)
	// using Kr = 1; T = 0.1; period = 1 sec, step = 0.001 sec
	// using the finite difference equivalent for the system: y(-1) = 0
	// x(k) = T/(T+step)*(x(k-1) - step*u(k))
	// y(k) = -Kr/T*x(k);
	// x(k-1) = y(k-1)*T/Kr;
	// controlling the single components of v:
		double T = 0.01;//0.00001;//0.008;//0.01;
		double Kr = 10;//2*1.25;//2*0.1;

		Vector dv(3);
		dv(0)= 1/(T+tbg.getSlopeRamp())*(dv_old(0)*T + v_m*v(0)*tbg.getSlopeRamp()*Kr);
		dv(1)= 1/(T+tbg.getSlopeRamp())*(dv_old(1)*T + v_m*v(1)*tbg.getSlopeRamp()*Kr);
		dv(2)= 1/(T+tbg.getSlopeRamp())*(dv_old(2)*T + v_m*v(2)*tbg.getSlopeRamp()*Kr);
		dv_old = dv;
		//dv = Kr/T*dv;

		//cout << dv.toString() << endl;

/*		//controllong the module of v
		double T = 0.008;
		double Kr = 2*1.25;

		double dv = T/(T+tbg.getSlopeRamp())*(dv_old + v_m*tbg.getSlopeRamp());
		dv_old = dv;
		dv = Kr/T*dv;
	*/

		//proj << n.toString() << endl;
		//proj << g << endl;

	// compute the new distorted force direction
		//return delta + g*dv*v;

		G << dv.toString() << endl;

		return g*(delta + dv);
}

Matrix PassiveMotionParadigm::align_K(const Vector &delta, const Matrix &K)
{
	// re-orient principal axes of L along (x_tg-x) direction:
	Matrix Krot = K;
	Matrix V(3,3);
	Vector x_axis(3);
	x_axis(0) = 1;
	x_axis(1) = x_axis(2) = 0;
	
	// define new matrix eigenvectors:
	if ( delta(0) == 0 && delta(1) == 0 && delta(2) == 0) return Krot;
	Vector V1 = delta/norm(delta);
	Vector V2 = cross(x_axis,V1);
	V.setCol(0,V1);
	V.setCol(1,V2);
	V.setCol(2, cross(V1,V2) );

	// compute new stiffness matrix according to eigendecomposition law:
	Krot = V*K*V.transposed();
	
	return Krot;
}

Vector PassiveMotionParadigm::calculate_F(const Vector &delta, const Matrix &K)
{
	Vector F = align_K(delta,K)*delta;
	return F;
}

Vector PassiveMotionParadigm::calculate_F(Vector x_tg, Vector x, Matrix K)
{
	Vector F(3);
	F = K*(x_tg-x);
	return F;
}

Vector PassiveMotionParadigm::calculate_Tau_int(string side)
{
	Vector Tau_int;
	Tau_int.resize(10);
	Tau_int.zero();

	// Attractive forces to q_ref joint angles values
	if (side.compare("right") == 0 && ref_right_active == 1)
		Tau_int = Kint_right*(q_ref_right - q_rightArm);

	if (side.compare("left") == 0 && ref_left_active == 1)
		Tau_int = Kint_left*(q_ref_left - q_leftArm);

	return Tau_int;
}

void PassiveMotionParadigm::calculate_qdot(Vector &F_r,Vector &F_l, string s)
 {	
	 Vector Tau_ext(10);
	 Vector Tau_int(10);
	 Vector q_sum_node(3);

	 Tau_ext.zero();
	 Tau_int.zero();

	 string side;

	 if(s!="null")	side = s;
	 else side = this->side;
	
	 if (side.compare("right") == 0)
	 {
		 Tau_ext = (get_Jacobian("right").transposed())*F_r;
		 Tau_int = calculate_Tau_int(side);
		 qdot_rightArm = tbg.calculateGamma(time) * A_rightArm * (Tau_ext + Tau_int);
	 }

	 if (side.compare("left") == 0)
	 {
		 Tau_ext = (get_Jacobian("left").transposed())*F_l;
		 Tau_int = calculate_Tau_int(side);
		 qdot_leftArm = tbg.calculateGamma(time) * A_leftArm * (Tau_ext + Tau_int);
	 }

	 if (side.compare("bimanual") == 0)
	 {
		 Vector Tau_extSum(10);
		 Vector Tau_intSum(10);
		 Tau_ext = (get_Jacobian("right").transposed())*F_r;
		 Tau_int = calculate_Tau_int("right");
		 qdot_rightArm = tbg.calculateGamma(time) * A_biman * (Tau_ext + Tau_int);

		 Tau_extSum = Tau_ext;
		 Tau_intSum = Tau_int;
		 
		 Tau_ext = (get_Jacobian("left").transposed())*F_l;
		 Tau_int = calculate_Tau_int("left");
		 qdot_leftArm = tbg.calculateGamma(time) * A_biman * (Tau_ext + Tau_int);

		 Tau_extSum = Tau_extSum + Tau_ext;
		 Tau_intSum = Tau_extSum + Tau_int;
		
		 q_sum_node = tbg.calculateGamma(time) * A_torso * (Tau_extSum.subVector(0,2) + Tau_intSum.subVector(0,2));
		 for ( int i=0; i<3; i++)
		 {
		 	qdot_rightArm(i) = q_sum_node(i);
		 	qdot_leftArm(i)  = q_sum_node(i);
		 }
	 }
 };


Vector PassiveMotionParadigm::getPose(const string & side)
{
	// HP che i valori di angoli siano già stati settati nelle varie chain con chain->setAng(q)
	Vector pose3D(3);
	Matrix T(4,4);

	if(side == "right")
		T = Rchain->getH();

	if(side == "left")
		T = Lchain->getH();

	pose3D = (T.getCol(3)).subVector(0,2);

	return pose3D;
}

Matrix PassiveMotionParadigm::get_Jacobian(string side)
{
	// HP che i valori di angoli siano già stati settati nelle varie chain con chain->setAng(q)
	Matrix J;
	int DOF;

	if(side == "right")
	{
		DOF = Rchain->getDOF();
		J.resize(6,DOF);
		J = Rchain->GeoJacobian();
	}

	if(side == "left")
	{
		DOF = Lchain->getDOF();
		J.resize(6,DOF);
		J = Lchain->GeoJacobian();
	}

	return J.submatrix(0,2,0,DOF-1); // positional Jacobian
}

/* Main Function */
bool PassiveMotionParadigm::run(Vector par, const Vector *xR_target, const Vector *xL_target)
{
	Vector x_right(3);
	Vector x_left(3);
	Vector F_right(3);
	Vector F_left(3);	

	x_right = x_0R;
	x_left  = x_0L;

	//cout << "tool R: " << std::boolalpha << useTool_R << endl;
	//cout << x_right.toString() << endl;

	//Runner initialization
	if(xR_target == NULL)	x_tgR = x_0R;
	else					x_tgR = *xR_target;

	if(xL_target == NULL)	x_tgL = x_0L;
	else					x_tgL = *xL_target;

	time = (unsigned int)par(0);

	if (time == 0)
	{
		tg_old = x_tgR;
		g_old = 0.0;
		grate_old = 0.0;
		g_turn = 0.0;
		dv_old.resize(3,0.0);
		x_old = x_right;
		//rpy_tg = rpy_old = get_RPYAng((Schain->getH()*Tool).submatrix(0,2,0,2));
		//dv_old = 0.0;
	}

	if (x_right == x_tgR) return true;

	if(side.compare("right") == 0)
	{	
		// get joint velocity compatible with the external force field:
		Matrix J = get_Jacobian("right");
		Matrix T = J*A_rightArm*J.transposed();
		//F_right = calculate_F(distort_delta(x_tgR-x_right,  x_tgR-tg_old, x_right-x_old, T, tbg.calculateGamma(time)),K_right);

		F_right = calculate_F(x_tgR, x_right, K_biman);
		calculate_qdot(F_right, F_left);
		q_rightArm = q_rightArm + qdot_rightArm * tbg.getSlopeRamp();
	
		// update torso joint angles of the other chain
		q_leftArm(0) = q_rightArm(0);
		q_leftArm(1) = q_rightArm(1);
		q_leftArm(2) = q_rightArm(2);
	}

	if(side.compare("left") == 0)
	{
		F_left = calculate_F(x_tgL,x_left,K_left);
		calculate_qdot(F_right, F_left);		
		q_leftArm = q_leftArm + qdot_leftArm * tbg.getSlopeRamp();

		// update torso joint angles of the other chain
		q_rightArm(0) = q_leftArm(0);
		q_rightArm(1) = q_leftArm(1);
		q_rightArm(2) = q_leftArm(2);
	}

	if(side.compare("bimanual") == 0)
	{
		F_right = calculate_F(x_tgR, x_right, K_biman);
		F_left  = calculate_F(x_tgL, x_left, K_biman);
		calculate_qdot(F_right, F_left);

		q_rightArm = q_rightArm + qdot_rightArm * tbg.getSlopeRamp();
		q_leftArm  = q_leftArm  + qdot_leftArm  * tbg.getSlopeRamp();
	}

	// update joint angles
	Rchain->setAng(q_rightArm);
	Lchain->setAng(q_leftArm);

	Vector rtemp = Rchain->getAng();
	Vector ltemp = Lchain->getAng();
	for (unsigned int i=0; i< Rchain->getN(); i++)
	{
		if ( rtemp(i)!=q_rightArm(i) )	{q_rightArm(i) = rtemp(i);}//cout << "qr cambia" << endl;}
		//else							cout << "non cambia" << endl;
		if ( ltemp(i)!=q_leftArm(i) )	{q_leftArm(i)  = ltemp(i);}//cout << "ql cambia" << endl;}
	}

	// update EE position
	x_right = get_EEPose("right");
	x_left  = get_EEPose("left");

	// update initial position with current EE position
	x_0R = x_right;
	x_0L = x_left;

		tg_old = x_tgR;
		x_old = x_right;
	
		pos << x_0R.toString() << endl;
		tg << x_tgR.toString() << endl;
		Force << F_right.toString() << endl;
		joints << q_rightArm.toString() << endl;

	// Update final end-effector position
	//cout << "\n\nFinal end effector cartesian coordinates: " << endl;
	//cout << "Right Hand: " << x_right.toString() << endl;
	//cout << "Left Hand: " << x_left.toString() << endl;
	//cout << "\nFinal joint angles: " << endl;
	//cout << "Right chain: \n" << (get_q_rightArm()*180.0/M_PI).toString() << endl;
	//cout << "Left chain: \n" << (get_q_leftArm()*180.0/M_PI).toString() << endl;

	return true;

};

void PassiveMotionParadigm::connectTool(Vector & target, const string & _side)
{
	Vector EE_inRoot = get_ToolPose(_side);

	target = target - EE_inRoot;

	return;
}

void PassiveMotionParadigm::use_Tool	(const string &_side)
{
	if(_side == "right") 
	{
		useTool_R = true;
		cout << "Tool right: " << Tool_R.toString() << endl;
	}

	if(_side == "left")  useTool_L = true;

	cout << "--> tool connected" << endl;
}
void PassiveMotionParadigm::leave_Tool	(const string &_side)
{
	if(_side == "right") useTool_R = false;
	if(_side == "left")  useTool_L = false;

	cout << "--> tool released" << endl;
}

void PassiveMotionParadigm::delete_Tool	(const string &_side)
{
	if(_side == "right") {useTool_R = false; Tool_R.zero(); Tool_R(3) = 1;}
	if(_side == "left")  {useTool_L = false; Tool_L.zero(); Tool_L(3) = 1;}

	cout << "--> tool deleted" << endl;
}

Vector PassiveMotionParadigm::get_ToolPose	(const string &_side)
{
	Vector EE_inRoot(3);

	if(_side == "right")
		EE_inRoot = (Rchain->getH()*Tool_R).subVector(0,2);
	
	if(_side == "left")
		EE_inRoot = (Lchain->getH()*Tool_L).subVector(0,2);

	return EE_inRoot;

}

Vector PassiveMotionParadigm::get_EEPose	 (const string &_side)
{
	Vector EE(3);

	if (_side == "right")	useTool_R ? EE = get_ToolPose(_side) : EE = getPose(_side);
	if (_side == "left")	useTool_L ? EE = get_ToolPose(_side) : EE = getPose(_side);

	return EE;

}

// ---------- PMP auxiliary methods ------------------------------------------
Vector PassiveMotionParadigm::pmp2Sim(Vector xp)
{
	Matrix Rot(3,3);
	Rot.zero();
	Rot(0,0) = 1;
	Rot(1,2) = 1;
	Rot(2,1) = -1;
	Vector xs(3);
	xs = Rot*xp;
	xs(1) = xs(1) + 597.6;
	xs(2) = xs(2) - 26.0;

	return xs;
};

Vector PassiveMotionParadigm::Sim2pmp(Vector xs)
{
	Matrix Rot(3,3);
	Rot.zero();
	Rot(0,0) = 1;
	Rot(1,2) = -1;
	Rot(2,1) = 1;
	Vector xp(3);
	Vector trans(3);
	trans(0) = 0;
	trans(1) = 597.6;
	trans(2) = -26.0;
	xp = Rot*xs -Rot*trans;

	return xp;
};


bool PassiveMotionParadigm::setFromProperty(Property *options, string key, Vector &v)
{
	Bottle *bot = ( options->findGroup(key.c_str())).find(key.c_str() ).asList();

	if ( bot->size()!= v.size())
	{	
		printf("Option %s size not compatible\n", key.c_str());
		return false;
	}

	for(unsigned int i=0; i<v.size(); i++)
	{
		v(i) = bot->get(i).asDouble();
		//cout << v(i) << endl;
	}
	return true;
}
bool PassiveMotionParadigm::setFromProperty(Property *options, string key, Matrix &m)
{
	Bottle *bot = ( options->findGroup(key.c_str())).find(key.c_str() ).asList();

	if ( bot->size()!= m.rows())
	{	
		printf("Option %s size not compatible\n", key.c_str());
		return false;
	}

	for(int i=0; i<bot->size(); i++)
	{
		m(i,i) = bot->get(i).asDouble();
		//cout << m(i) << endl;
	}
	return true;
}
bool PassiveMotionParadigm::setEyeMatrixFromVector(Vector v, Matrix &m)
{
	if ( v.size()!= m.rows())
	{	
		printf("Error: Size not compatible\n");
		return false;
	}

	m.eye();
	for(unsigned int i=0; i<v.size(); i++)
		m(i,i) = v(i);

	return true;
}
/*
void PassiveMotionParadigm::MessagePass()
 {
	BufferedPort<Bottle> out;
	BufferedPort<Bottle> outr;
	BufferedPort<Bottle> outl;

    // Name the ports
    out.open("/DARWIN/torso");
    outr.open("/DARWIN/right");
	outl.open("/DARWIN/left");
    //
    // Connect the ports so that anything written from /out arrives to /in
    Network::connect("/DARWIN/right","/icubSim/right_arm/rpc:i");
    Network::connect("/DARWIN/left","/icubSim/left_arm/rpc:i");
	Network::connect("/DARWIN/torso","/icubSim/torso/rpc:i");

	Bottle& outBot= out.prepare();
	Bottle& outBotr= out.prepare();
	Bottle& outBotl= out.prepare();
	//Bottle& listBot ;
	//Bottle& listBotr ;
	//Bottle& listBotl ;

for (int i = 0; i < 3;i++)
{
	outBot = out.prepare();	
    outBot.clear();

	outBot.addString("set"); // put "set" command in the bottle
    outBot.addString("pos");
	Bottle& listBot = outBot.addList();
	
	listBot.addInt(i);
	listBot.addDouble(q_rightArm(i)*CTRL_RAD2DEG);

	cout << "Q_r(i): " << q_rightArm(i)*CTRL_RAD2DEG <<endl;
	//outBot1.addString(")");
   // printf("\n\n Writing bottle 2 (%s)\n",outBot3.toString().c_str());
    out.write();                       // Now send i 
}
cout << "right arm joint angles degrees: " <<endl;
for (int i = 3; i < 10; i++)
{
	outBotr = outr.prepare();
	outBotr.clear();

	outBotr.addString("set"); // put "set" command in the bottle
    outBotr.addString("pos");
    
	Bottle& listBotr = outBotr.addList();

	listBotr.addInt(i-3);
	listBotr.addDouble(q_rightArm(i)*CTRL_RAD2DEG);


	cout << i-3 << " " << q_rightArm(i)*CTRL_RAD2DEG << endl;
	
	outr.write();
}
};
*/
/*
 void PassiveMotionParadigm::MessageDevDriver(bool Activate)
 {    
    Property options;
	//options.fromConfigFile("PMP_rightArm.ini");

    options.put("device", "remote_controlboard");
    options.put("local", "/DARWIN/right");			//local port names
    options.put("remote", "/icubSim/right_arm");     //where we connect to

    // create a device
    PolyDriver robotArmDevice(options);
    if (!robotArmDevice.isValid()) {
        printf("Device not available. Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
    }

	Property optionsT;

    optionsT.put("device", "remote_controlboard");
    optionsT.put("local", "/DARWIN/torso");   //local port names
    optionsT.put("remote", "/icubSim/torso");         //where we connect to

    // create a device
    PolyDriver robotTruncDevice(optionsT);
    if (!robotTruncDevice.isValid()) {
        printf("Device not available. Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
    }

    IPositionControl *posR;
	IPositionControl *posT;

    IEncoders		 *encsR;
	IEncoders		 *encsT;

    bool ok = robotArmDevice.view(posR);
    ok = ok && robotArmDevice.view(encsR);

	bool okT;
    okT = robotTruncDevice.view(posT);
    okT = okT && robotTruncDevice.view(encsT);

	
    if (!ok) {
        printf("Problems acquiring RIGHT ARM interfaces\n");
    }

	if (!okT) {
        printf("Problems acquiring Torso interfaces\n");
        //return 0;
    }

	Vector encodersR, commandR, velR, accR;
    int njR=0;
    posR->getAxes(&njR);
	encodersR.resize(njR);
	velR.resize(njR);
	accR.resize(njR);
    commandR.resize(njR);
	commandR.zero();
	//printf("Joints %d \n", nj);

	Vector encodersT, commandT, velT, accT;
	int njT=0;
    posT->getAxes(&njT);
	//printf("Joints Torso %d \n", njT);
    encodersT.resize(njT);
    velT.resize(njT);
	accT.resize(njT);
    commandT.resize(njT);
	commandT.zero();

	// set reference acceleration used to generate velocity profile [deg/s]
	// and initialize new position
    for (int i = 0; i < njR; i++) {
         accR[i] = 60.0;
		 velR[i] = 30.0;
		 commandR(i) = q_rightArm(i+3)*CTRL_RAD2DEG;
    }
    posR->setRefAccelerations(accR.data());
	posR->setRefSpeeds(velR.data());

	for (int i = 0; i < njT; i++) {
         accT[i] = 60.0;
		 velT[i] = 30.0;
    }
	commandT(0) = -q_rightArm(2)*CTRL_RAD2DEG;
	commandT(1) = q_rightArm(1)*CTRL_RAD2DEG;
	commandT(2) = q_rightArm(0)*CTRL_RAD2DEG;

    posT->setRefAccelerations(accT.data());
	posT->setRefSpeeds(velT.data());

	// move to the set position (position control) and check 
	bool checkR = posR->positionMove(commandR.data());
	bool checkT = posT->positionMove(commandT.data());

	bool doneR=false;
	bool doneT=false;

	while(!doneR & !doneT)
	{
		posR->checkMotionDone(&doneR);
		posT->checkMotionDone(&doneT);
		Time::delay(0.1);
	}

    robotArmDevice.close();
	robotTruncDevice.close();

 };
*/
/*
void PassiveMotionParadigm::Sim_reference(char final_ref)
{
	Matrix T0w(4,4);
	T0w.zero();
	T0w(0,1) = -1;
	T0w(1,2) = 1;
	T0w(1,3) = 0.5976;
	T0w(2,0) = -1;
	T0w(2,3) = -0.026;
	T0w(3,3) = 1;

	Matrix Tw0(4,4);
	Tw0 = luinv(T0w);

	if(final_ref == 's')
	{
		TRo_r = Tw0*TRo_r;
	}
	if(final_ref == 'r')
	{
		TRo_r = T0w*TRo_r;
	}
};*/
