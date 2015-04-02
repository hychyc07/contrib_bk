#include "darwin/PMP.h"

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

	//tgx.open("MovingTargetR.txt");
	//posx.open("MovingPosR.txt");
	//tgx1.open("MovingTargetL.txt");
	//posx1.open("MovingPosL.txt");
	//Fr.open("MovingForceR.txt");
	//Fl.open("MovingForceL.txt");
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
	//delete Rchain;
	//delete Lchain;
/*
	cout << "deleting chains" << endl;
	//Rchain->clear();
	Lchain->clear();
	cout << "deleting arms" << endl;
	delete Rarm;
	delete Larm;

	tgx.close();
	posx.close();
	tgx1.close();
	posx1.close();
	Fl.close();
	Fr.close();
	*/
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

/*yarp::sig::Matrix PassiveMotionParadigm::get_CompMatrix(Vector F, string side)
{
	Matrix Rot(3,3);
	Rot.eye();

	Vector Zero(3);
	Zero.zero();

	if (F == Zero) return Rot;

	Vector F_N(3);
	F_N.zero();

	Vector q_init(10), q_temp(10);
	Vector xdot_N(3), x_curr(3);

	// save current cartesian position and chain to be used
		if(side == "right") {x_curr = x_0R; q_init = q_rightArm;}
		else				{x_curr = x_0L; q_init = q_leftArm;}

	// compute the ouput velocity without compensation for an unitary input force:
		F_N = F/norm(F);
	
		if (side == "right") 
		{
			calculate_qdot(F_N,Zero,side);
			q_temp = q_rightArm + qdot_rightArm * tbg.getSlopeRamp();
			Rchain->setAng(q_temp);
			xdot_N = get_Jacobian(side)*qdot_rightArm;
		}
		else
		{
			calculate_qdot(Zero,F_N,side);
			q_temp = q_leftArm + qdot_leftArm * tbg.getSlopeRamp();
			Lchain->setAng(q_temp);
			xdot_N = get_Jacobian(side)*qdot_leftArm;
		}
		//cout << "x_dot " << xdot_N.toString() << endl;

		if (xdot_N == Zero) return Rot;
	// compensate for orientation distortion:
		// 1-compute the rotation axis versors: v = force x vel


		xdot_N = xdot_N/norm(xdot_N);
		Vector v = cross(F_N,xdot_N);
		
		// 2-compute rotation angle between force and velocity respect to the rotation axis
		double cos_theta = dot(F_N,xdot_N);
		double tg_thetaHalf = sqrt( (1-cos_theta)/(1+cos_theta) );

		//cout << "cos theta " << cos_theta << endl;
		//cout << "tg theta/2 " << tg_thetaHalf << endl;

		// 3-Apply Rodrigues formula to obtain a 3D rotation matrix
		Vector rho = v*tg_thetaHalf;
		Rot(0,0) = 1 + rho(0)*rho(0) - rho(1)*rho(1) - rho(2)*rho(2);
		Rot(1,1) = 1 + rho(1)*rho(1) - rho(0)*rho(0) - rho(2)*rho(2);
		Rot(2,2) = 1 + rho(2)*rho(2) - rho(0)*rho(0) - rho(1)*rho(1);
		Rot(0,1) = 2*( rho(0)*rho(1) + rho(2) );
		Rot(1,0) = 2*( rho(0)*rho(1) - rho(2) );
		Rot(0,2) = 2*( rho(0)*rho(2) - rho(1) );
		Rot(2,0) = 2*( rho(0)*rho(2) + rho(1) );
		Rot(1,2) = 2*( rho(1)*rho(2) + rho(0) );
		Rot(2,1) = 2*( rho(1)*rho(2) - rho(0) );

		Rot = Rot/(1+dot(rho,rho));	

	// compensate for the loop gain: gain = deltaZ/norm(X_dot);
	// Z = curvilinear abscissa on the 3D trajectory, 
	// deltaZ is the step along the trajectory given an unitary input step
		if (side == "right") 
		{
			q_rightArm = q_init;
			calculate_qdot(Rot*F_N, Zero,side);
			q_temp = q_rightArm + qdot_rightArm * tbg.getSlopeRamp();
			Rchain->setAng(q_temp);
		}
		else
		{
			q_leftArm = q_init;
			calculate_qdot(Zero,Rot*F_N,side);
			q_temp = q_leftArm + qdot_leftArm * tbg.getSlopeRamp();
			Lchain->setAng(q_temp);
		}

		Vector x_next = getPose(side);
		double deltaZ = norm( (x_next-x_curr) );	
		double gain = deltaZ/norm(xdot_N);

	// revert all values to the original ones
		if (side == "right")	{q_rightArm = q_init; Rchain->setAng(q_init); }
		else					{q_leftArm  = q_init; Lchain->setAng(q_init); }

	return Rot*gain;
}*/

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
		 qdot_rightArm = tbg.calculateGamma((double)time) * A_rightArm * (Tau_ext + Tau_int);
	 }

	 if (side.compare("left") == 0)
	 {
		 Tau_ext = (get_Jacobian("left").transposed())*F_l;
		 Tau_int = calculate_Tau_int(side);
		 qdot_leftArm = tbg.calculateGamma((double)time) * A_leftArm * (Tau_ext + Tau_int);
	 }

	 if (side.compare("bimanual") == 0)
	 {
		 Vector Tau_extSum(10);
		 Vector Tau_intSum(10);
		 Tau_ext = (get_Jacobian("right").transposed())*F_r;
		 Tau_int = calculate_Tau_int("right");
		 qdot_rightArm = tbg.calculateGamma((double)time) * A_biman * (Tau_ext + Tau_int);

		 Tau_extSum = Tau_ext;
		 Tau_intSum = Tau_int;
		 
		 Tau_ext = (get_Jacobian("left").transposed())*F_l;
		 Tau_int = calculate_Tau_int("left");
		 qdot_leftArm = tbg.calculateGamma((double)time) * A_biman * (Tau_ext + Tau_int);

		 Tau_extSum = Tau_extSum + Tau_ext;
		 Tau_intSum = Tau_extSum + Tau_int;
		
		 q_sum_node = tbg.calculateGamma((double)time) * A_torso * (Tau_extSum.subVector(0,2) + Tau_intSum.subVector(0,2));
		 for ( int i=0; i<3; i++)
		 {
		 	qdot_rightArm(i) = q_sum_node(i);
		 	qdot_leftArm(i)  = q_sum_node(i);
		 }
	 }
 };

/*
Matrix PassiveMotionParadigm::compute_T0n1(string side)
{
	// no fingers
	iCubArm arm(side);
	iKinChain *chain;
	chain = arm.asChain();

	for (int i=0; i < (int)chain->getDOF(); i++)
	{
		if (chain->isLinkBlocked(i))
			chain->releaseLink(i);
	}

	Vector q;
	if (side.compare("right") == 0)
		q = q_rightArm;
	if (side.compare("left") == 0)
		q = q_leftArm;

	//cout <<" q: "<< q.toString() <<endl;
	chain->setAng(q);
	return chain->getH(q);
};
*/

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

	//cout << "target passed: " << xR_target->toString() << endl;
	//cout <<"X right: " << x_right.toString() << endl;
	//cout <<"X tg: " << x_tgR.toString() << endl;
	//cout <<"X0 : " << x_0R.toString() << endl;

	//if (useTool_R)	connectTool(x_tgR,"right");
	//if (useTool_L)	connectTool(x_tgR,"left");

	//cout << "\n\nRight EE initial position: " << x_right.toString() << endl;
	//cout << "\n\nLeft  EE initial position: " << x_left.toString() << endl;
	//cout << "Initial Joint Angles: " <<endl;
	//cout << "right: " << endl;
	//cout << (q_rightArm*CTRL_RAD2DEG).toString() <<endl;
	//cout << "left: " << endl;
	//cout << (q_leftArm*CTRL_RAD2DEG).toString() <<endl;

	//tgx << x_tgR.toString() << endl;
	//posx << x_right.toString() << endl;
	//tgx1 << x_tgL.toString() << endl;
	//posx1 << x_left.toString() << endl;

	// x_tg is set from outside
	if(side.compare("right") == 0)
	{	
		F_right = calculate_F(x_tgR, x_right, K_right);
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

	//cout << x_left.toString() << endl;
	//tgx << x_tgR.toString() << endl;
	//posx << x_right.toString() << endl;
	//tgx1 << q_rightArm.toString() << endl;
	//tgx1 << x_tgL.toString() << endl;
	//posx1 << x_left.toString() << endl;
	//tgx1 << q_leftArm.toString() << endl;
	//Fr << F_right.toString() << endl;
//	Fr << Fnew.toString() << endl;
	//Fl << F_left.toString() << endl;

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

	for(int i=0; i<v.size(); i++)
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
	for(int i=0; i<v.size(); i++)
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
