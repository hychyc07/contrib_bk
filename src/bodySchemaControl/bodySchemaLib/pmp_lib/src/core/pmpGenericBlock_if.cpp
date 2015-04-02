#include <pmp_lib/core/tbg.h>
#include <pmp_lib/core/pmpGenericBlock_if.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;

using namespace iCub::pmplib;
using namespace iCub::pmplib::core;


void iCubFinger::allocate(const std::string &_type)
{
	iKinLimb::allocate(_type);

    Matrix H0(4,4);
    H0.zero();
    H0.eye();
    setH0(H0);

    if (getType()=="ring")
	{
        pushLink(new iKinLink( 0.019, 0.0, 0.0, 0.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
		blockLink(0,90.0*CTRL_DEG2RAD);
	}
    else // index
	{
		pushLink(new iKinLink( 0.019, 0.0, 0.0, 0.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
		blockLink(0,-90.0*CTRL_DEG2RAD);
	}

	blockLink(0,90.0*CTRL_DEG2RAD);
}

iCubFinger::iCubFinger()
{
	allocate("ring");
}
iCubFinger::iCubFinger(const std::string &_type)
{
	allocate(_type);
}
iCubFinger::iCubFinger(const iCubFinger &finger)
{
	clone(finger);
}

/* iCubTorso definitions */

iCubTorso::iCubTorso()
{
	allocate("right");
}

iCubTorso::iCubTorso(const std::string &_type)
{
	allocate(_type);
}

iCubTorso::iCubTorso(const iCub::pmplib::core::iCubTorso &torso)
{
	clone(torso);
}

void iCubTorso::allocate(const std::string &_type)
{
	iKinLimb::allocate(_type);

    Matrix H0(4,4);
    H0.zero();
    H0(0,1)=-1.0;
    H0(1,2)=-1.0;
    H0(2,0)=1.0;
    H0(3,3)=1.0;
    setH0(H0);

	 pushLink(new iKinLink(   0.032,      0.0,  M_PI/2.0,        0.0,  -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
     pushLink(new iKinLink(     0.0,  -0.0055,  M_PI/2.0,  -M_PI/2.0,  -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));

    if (getType()=="right")
        pushLink(new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));

    else
        pushLink(new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));  
}


/* iCubArmNoTorso definitions */
iCubArmNoTorso::iCubArmNoTorso()
{
    allocate("right");
}


iCubArmNoTorso::iCubArmNoTorso(const string &_type)
{
    allocate(_type);
}


iCubArmNoTorso::iCubArmNoTorso(const iCubArmNoTorso &arm)
{
    clone(arm);
}


void iCubArmNoTorso::allocate(const string &_type)
{
    iKinLimb::allocate(_type);

    Matrix H0(4,4);
    H0.eye();
    setH0(H0);

    if (getType()=="right")
    {
        pushLink(new iKinLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
        pushLink(new iKinLink(    -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    }
    else
    {
        pushLink(new iKinLink(       0.0,  0.10774, -M_PI/2.0,            M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    -0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,   0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0625,   -0.016,       0.0,                 0.0, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
const double pmpBlock::PALM_LITTLE [] = {0,  38.7352, 0};
const double pmpBlock::PALM_INDEX  [] = {0, -19.5646, 0};
pmpBlock::~pmpBlock()
{
/*	tg.close();
	pos.close();
	Force.close();
	joints.close();
	jvel.close();
	eigV.close();
	eigL.close();
	G.close();
	proj.close();
	mom.close();
*/
}



pmpBlock::pmpBlock()
{
	// chain fake initialization:
		chain = new iKinChain();
}

pmpBlock::pmpBlock(const string &block_type) : pmpGenericBlock(), blockType(block_type)
{
	// allocate block's chain: initialize vectors and matrices that do depend on robot's DOF:
		allocateChain();
}

pmpBlock::pmpBlock(const pmpBlock &block) : blockType(block.blockType)
{
	//cout << "copy constructor called" << endl;
	this->allocateChain();	
	*this = block;
}
pmpBlock& pmpBlock::operator=(const pmpBlock &block)
{
	if (this != &block) // protect against self-copy
	{
		this->F = block.F;
		this->K = block.K;
		this->Km = block.Km;
		this->x_0 = block.x_0;
		this->x_tg = block.x_tg;
		this->refPose = block.refPose;

		// Anisotropy Compensation
		this->tg_old = block.tg_old;
		this->dv_old = block.dv_old;
		this->rpy_old = block.rpy_old;
		this->rpy_tg = block.rpy_tg;

		// Others
		this->ref_active = block.ref_active;
		this->scale = block.scale;

	//	allocate block's chain: initialize vectors and matrices that do depend on robot's DOF:
		//this->chain = new iKinChain(*(block.chain));
		*this->chain = *(block.chain);
		this->blockType = block.blockType;

		this->Kint = block.Kint;
		this->A = block.A;
		this->q = block.q;
		this->q_ref = block.q_ref;
		this->q_0 = block.q_0;
		this->q_home = block.q_home;

	// copy tbg & options:
		this->tbg = block.tbg;
		this->opt_chain = block.opt_chain;
		this->opt_tbg = block.opt_tbg;
	}

	return *this;
}


void pmpBlock::allocate(const std::string &block_type)
{
	//cout << "allocate called" << endl;
	if (block_type == "right_arm")
	{
		Limb = new iCubArmNoTorso("right");
	}
	else if (block_type == "left_arm")
	{
		Limb =  new iCubArmNoTorso("left");
	}
	else if (block_type == "torso_right" ||block_type == "torso")
	{
		Limb = new iCubTorso("right");
	}
	else if (block_type == "torso_left")
	{
		Limb = new iCubTorso("left");
	}

	//this->default_chain = new iKinChain(*Limb->asChain());
	this->chain = new iKinChain(*Limb->asChain());
	//chain = default_chain;

	int DOF = chain->getDOF();
	Kint.resize(DOF,DOF);
	A.resize(DOF,DOF);
	q.resize(DOF,0.0);
	q_ref.resize(DOF,0.0);
	q_0.resize(DOF,0.0);
	q_home.resize(DOF,0.0);
}

void pmpBlock::allocateChain()  // TO DO: funzione con enumerativi!
{
	//cout << "allocate called" << endl;
	if (blockType == "right_arm")
	{
		Limb = new iCubArmNoTorso("right");
	}
	else if (blockType == "left_arm")
	{
		Limb =  new iCubArmNoTorso("left");
	}
	else if (blockType == "torso_right" ||blockType == "torso")
	{
		Limb = new iCubTorso("right");
	}
	else if (blockType == "torso_left")
	{
		Limb = new iCubTorso("left");
	}
	else if (blockType == "ring")
	{
		Limb = new iCubFinger("ring");
	}
	else if (blockType == "index")
	{
		Limb = new iCubFinger("index");
	}
	else 
	{
		cout << "Error, chain not recognized" << endl;
		return;
	}

	this->chain = new iKinChain(*Limb->asChain());
	this->allocateVariableSizeParams(this->chain->getDOF());
}

bool pmpBlock::initializeBlock(Property * _opt_chain)
{		
	if (_opt_chain!=NULL)	this->opt_chain = _opt_chain;

	if (chain->getDOF() == 0)
	{
		if (chain->getN()>0)
		{
			cout << "warning: block " << blockType << " has 0 DOF" << endl;
		}
		else
		{
			cout << "error: block " << blockType << " has no link" << endl;
			return false;
		}
	}
	else // allocate variables whose size is dof-dependent
	{

		if(opt_chain->check("q_initial"))
		{
			setAngFromProperty(opt_chain, "q_initial", q_0);
			q_home = q = q_0;
			chain->setAng(q_home);
		}
		if(opt_chain->check("q_ref"))				
		{	
			Vector temp(chain->getDOF(),0.0);

			setAngFromProperty(opt_chain,"q_ref", temp);
			set_q_ref(temp);
		}
		if(opt_chain->check("pose_ref"))				
		{	
			Vector a(2,0.0);
			//setAngFromProperty(options,"pose_ref", a);
			//refPose = get_RPYAng(a(0),a(1));
			setAngFromProperty(opt_chain,"pose_ref", refPose);
			cout << refPose.toString() << endl;
		}

		if(opt_chain->check("Admittance"))			setFromProperty(opt_chain, "Admittance", A);
		if(opt_chain->check("JointStiffness"))		setFromProperty(opt_chain, "JointStiffness", Kint);
	}

	// allocate all other params that are not size-dependent
	if(opt_chain->check("VirtualStiffness"))
	{
		Matrix Ktot(6,6);
		setFromProperty(opt_chain, "VirtualStiffness", Ktot);
		K = Ktot.submatrix(0,2,0,2);
		Km = Ktot.submatrix(3,5,3,5);
	}

	// update EE initial position values
	chain->setAllConstraints(true);
	chain->setAng(q_home);
	x_0 = get_EEposition();
	//cout << "Initial EE position:  " << x_0.toString() << endl;
	//cout << "Km: " << Km.toString() << endl;

	return true;
}


bool pmpBlock::initializeBlock(Bottle * _opt_chain)
{		
	this->opt_chain = new Property(_opt_chain->toString().c_str());
	Property *p = NULL;
	initializeBlock(p);

	/*
	if(opt_chain->check("q_initial"))
	{
		setAngFromProperty(opt_chain, "q_initial", q_0);
		q_home = q = q_0;
		chain->setAng(q_home);
	}
	if(opt_chain->check("q_ref"))				
	{	
		Vector temp(chain->getDOF(),0.0);
		setAngFromProperty(opt_chain,"q_ref", temp);
		set_q_ref(temp);
	}
	if(opt_chain->check("pose_ref"))				
	{	
		Vector a(2,0.0);
		//setAngFromProperty(options,"pose_ref", a);
		//refPose = get_RPYAng(a(0),a(1));
		setAngFromProperty(opt_chain,"pose_ref", refPose);
		cout << refPose.toString() << endl;
	}

	if(opt_chain->check("Admittance"))			setFromProperty(opt_chain, "Admittance", A);
	if(opt_chain->check("JointStiffness"))		setFromProperty(opt_chain, "JointStiffness", Kint);
	if(opt_chain->check("VirtualStiffness"))
	{
		Matrix Ktot(6,6);
		setFromProperty(opt_chain, "VirtualStiffness", Ktot);
		K = Ktot.submatrix(0,2,0,2);
		Km = Ktot.submatrix(3,5,3,5);
	}

	// update EE initial position values
	chain->setAllConstraints(true);
	chain->setAng(q_home);
	x_0 = get_EEposition();
	cout << "Initial EE position:  " << x_0.toString() << endl;
	//cout << "q_initial: " << (chain->getAng()*CTRL_RAD2DEG).toString() << endl;
	//cout << "q_0: " << q_0.toString() << endl;
*/
	return true;
}

bool pmpBlock::initializeTbg(Property * _opt_tbg)
{
	this->opt_tbg = _opt_tbg;

	// initialize internal tbg:		
	if (opt_tbg->check("T_init"))
		tbg.setT_init(opt_tbg->find("T_init").asDouble());
	if (opt_tbg->check("T_dur"))
		tbg.setT_dur(opt_tbg->find("T_dur").asDouble());
	if (opt_tbg->check("SlopeRamp"))
		tbg.setSlopeRamp(opt_tbg->find("SlopeRamp").asDouble());
	if (opt_tbg->check("alpha"))
		tbg.setAlpha(opt_tbg->find("alpha").asDouble());

	return true;
}
bool pmpBlock::initializeTbg(Bottle * _opt_tbg)
{
	this->opt_tbg = new Property(_opt_tbg->toString().c_str());

	// initialize internal tbg:		
	if (opt_tbg->check("T_init"))
		tbg.setT_init(opt_tbg->find("T_init").asDouble());
	if (opt_tbg->check("T_dur"))
		tbg.setT_dur(opt_tbg->find("T_dur").asDouble());
	if (opt_tbg->check("SlopeRamp"))
		tbg.setSlopeRamp(opt_tbg->find("SlopeRamp").asDouble());
	if (opt_tbg->check("alpha"))
		tbg.setAlpha(opt_tbg->find("alpha").asDouble());

	return true;
}

bool pmpBlock::initialize(Property * opt_block, Property * opt_tbg)
{
	if ( !initializeBlock(opt_block) )
	{
		cout << "--> Error: " << blockType << " initialization FAILED" << endl;
		return false;
	}

	if ( !initializeTbg(opt_tbg) )
	{
		cout << "--> Error: " << blockType << " internal tbg initialization FAILED" << endl;
		return false;
	}

	cout << "--> " << blockType << " initialization DONE" << endl;
	return true;
}



// ------------------ get and set methods--------------------

void pmpBlock::set (const string & param, const Vector & v)
{
	switch((int)string2pmp_param(param))
	{
	case iCub::pmplib::core::q:			pmpBlock::set_q(v);			break;
	case iCub::pmplib::core::q_ref:		pmpBlock::set_q_ref(v);		break;
	case iCub::pmplib::core::A:			pmpBlock::set_A(v);			break;
	case iCub::pmplib::core::K:			pmpBlock::set_K(v);			break;
	case iCub::pmplib::core::K_int:		pmpBlock::set_Kint(v);		break;
	default:		cout << "throw an error" << endl;
	}
}

void pmpBlock::set (const string & param, const Matrix & m)
{
	switch((int)string2pmp_param(param))
	{
	case iCub::pmplib::core::A:			pmpBlock::set_A(m);			break;
	case iCub::pmplib::core::K:			pmpBlock::set_K(m);			break;
	case iCub::pmplib::core::K_int:		pmpBlock::set_Kint(m);		break;
	default:		cout << "throw an error" << endl;
	}
}

yarp::sig::Vector pmpBlock::getVector (const string & param)
{
	switch((int)string2pmp_param(param))
	{
	case iCub::pmplib::core::q:			return pmpBlock::get_q();
	case iCub::pmplib::core::q_ref:		return pmpBlock::get_q_ref();
	default:		cout << "throw an error" << endl;
	}

	Vector v(1);
	return v;
}

yarp::sig::Matrix pmpBlock::getMatrix (const string & param)
{
	switch(string2pmp_param(param))
	{
	case iCub::pmplib::core::A:			return pmpBlock::get_A();
	case iCub::pmplib::core::K:			return pmpBlock::get_K();
	case iCub::pmplib::core::K_int:		return pmpBlock::get_Kint();
	default:		cout << "throw an error" << endl;
	}

	Matrix m(1,1);
	return m;
}

Vector pmpBlock::get_q()
{
	return q;
}

Matrix& pmpBlock::get_A()
{
	return A;
}

Matrix& pmpBlock::get_K()
{
	return K;
}
Matrix& pmpBlock::get_Km()
{
	return Km;
}
Matrix pmpBlock::get_Kint()
{
	return Kint;
}

Vector pmpBlock::get_q_ref()
{
	return q_ref;
}

double pmpBlock::get_DOF()
{
	return chain->getDOF();
}


double pmpBlock::get_N()
{
	return chain->getN();
}


iKinChain pmpBlock::get_chain()
{
	return *(this->chain);
}

iKinChain *pmpBlock::get_defaultChain()
{
	//return this->default_chain;
	return this->chain;
}

void pmpBlock::set_chain(iKinChain * _c)
{
	this->chain = _c;
}

void   pmpBlock::set_q (const Vector &_q)
{
	q = _q;
	chain->setAng(q);

	// check if any tool is connected: if so, use the tool end-point as starting point for computation:
	x_0 = get_EEposition();
	//x_0R = getPose("right");
}

void   pmpBlock::set_A (const Matrix &_A)
{
	A = _A;
}

void   pmpBlock::set_A (const Vector &_A)
{
	setEyeMatrixFromVector(_A,A);
}

void   pmpBlock::set_K    (const Matrix &_K)
{
	K = _K;
}

void   pmpBlock::set_K    (const Vector &_K)
{
	setEyeMatrixFromVector(_K, K);
}

void   pmpBlock::set_Km   (const Matrix &_K)
{
	Km = _K;
}

void   pmpBlock::set_Km   (const Vector &_K)
{
	setEyeMatrixFromVector(_K, Km);
}

void   pmpBlock::set_Kint (const Matrix &_Kint)
{
	Kint = _Kint;
}

void   pmpBlock::set_Kint (const Vector &_Kint)
{
	setEyeMatrixFromVector(_Kint, Kint);
}

void   pmpBlock::set_q_ref(const Vector &_q_ref)
{
	q_ref = _q_ref;
	ref_active = 1;
}


Vector pmpBlock::get_EEposition()
{
	// HP che i valori di angoli siano già stati settati nelle varie chain con chain->setAng(q)
	return chain->EndEffPosition();
}

yarp::sig::Vector pmpBlock::get_EEpose_asRPY()
{
	if (get_DOF() > 0)
		return chain->EndEffPose(false);
	else
	{
		Vector Zero(1,0.0);
		return Zero;
	}
}
yarp::sig::Matrix pmpBlock::get_EEpose_asRot()
{
	if (get_DOF() > 0)
		return chain->getH().submatrix(0,2,0,2);
	else
	{
		Matrix Zero(1,1);
		return Zero;
	}
}


Vector pmpBlock::get_EulAng (const double &Rotz,const double &Rotx)
{
	Matrix Rx(3,3);
	Matrix Rz(3,3);
	Rx.zero();
	Rz.zero();
	Rx(0,0) = 1;
	Rx(2,2) = Rx(1,1) = cos(Rotx);
	Rx(1,2) = sin(Rotx);
	Rx(2,1) = -sin(Rotx);
	Rz(2,2) = 1;
	Rz(0,0) = Rz(1,1) = cos(Rotz);
	Rz(0,1) = sin(Rotz);
	Rz(1,0) = -sin(Rotz);

	Matrix R = Rz*Rx;
	Vector EulAng(3);
	EulAng(0) = atan2(-R(2,1),R(2,2));
    EulAng(1) = asin(R(2,0));
    EulAng(2) = atan2(-R(1,0),R(0,0));

	return EulAng;
}

Vector pmpBlock::get_RPYAng (const double &Rotz,const double &Rotx)
{
	Matrix Rx(3,3);
	Matrix Rz(3,3);
	Rx.zero();
	Rz.zero();
	Rx(0,0) = 1;
	Rx(2,2) = Rx(1,1) = cos(Rotx);
	Rx(1,2) = -sin(Rotx);
	Rx(2,1) = sin(Rotx);
	Rz(2,2) = 1;
	Rz(0,0) = Rz(1,1) = cos(Rotz);
	Rz(0,1) = -sin(Rotz);
	Rz(1,0) = sin(Rotz);

	Matrix R = Rz*Rx;
	Vector RPYAng(3);
	RPYAng(0) = atan2( -R(1,0),R(0,0) );
    RPYAng(1) = atan2( -R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)) );
    RPYAng(2) = atan2( -R(2,1),R(2,2) );

	return RPYAng;
}
Vector pmpBlock::get_RPYAng (const Matrix &R)
{
	Vector RPYAng(3);
	double cbeta = sqrt( R(0,0)*R(0,0) + R(1,0)*R(1,0) );
	double sbeta = -R(2,0);
	double eps = 0.0001;

	if ( fabs(cbeta) < eps && sbeta > 0)
	{
		RPYAng(0) = 0;
		RPYAng(1) = M_PI/2;
		RPYAng(2) = atan2( R(0,1),R(1,1) );
	}
	else if ( fabs(cbeta) < eps && sbeta < 0)
	{
		RPYAng(0) = 0;
		RPYAng(1) = -M_PI/2;
		RPYAng(2) = atan2( -R(0,1),R(1,1) );
	}
	else
	{
		RPYAng(0) = atan2( R(1,0), R(0,0) );
		RPYAng(1) = atan2( sbeta, cbeta );
		RPYAng(2) = atan2( R(2,1),R(2,2) );
	}

	return RPYAng;
}
void pmpBlock::get_nextRPY(unsigned int time)
{
	rpy_tg = rpy_tg + (refPose-rpy_tg)*tbg.calculateGamma(time)*tbg.getSlopeRamp();
}


bool pmpBlock::setEEorientation (const string &_side, const Vector & RPY)
{
	Matrix R_root2palm = RPY2Rot(RPY);

	Matrix R_root2EE(3,3);
	R_root2EE = chain->getH().submatrix(0,2,0,2);

	Matrix R_palm2EE = R_root2palm.transposed() * R_root2EE;
	Vector EEpoint1(3,PALM_INDEX);
	Vector EEpoint2(3,PALM_LITTLE);

	palmPoints.setRow(0, R_palm2EE * EEpoint1);
	palmPoints.setRow(1, R_palm2EE * EEpoint2);

	cout << palmPoints.toString() << endl;
	useOrientation = true;

	return true;
}

Vector pmpBlock::Rot2RPY (const Matrix &R)
{
	Vector RPYAng(3);
	double cbeta = sqrt( R(0,0)*R(0,0) + R(1,0)*R(1,0) );
	double sbeta = -R(2,0);
	double eps = 0.0001;

	if ( fabs(cbeta) < eps && sbeta > 0)
	{
		RPYAng(0) = 0;
		RPYAng(1) = M_PI/2;
		RPYAng(2) = atan2( R(0,1),R(1,1) );
	}
	else if ( fabs(cbeta) < eps && sbeta < 0)
	{
		RPYAng(0) = 0;
		RPYAng(1) = -M_PI/2;
		RPYAng(2) = atan2( -R(0,1),R(1,1) );
	}
	else
	{
		RPYAng(0) = atan2( R(1,0), R(0,0) );
		RPYAng(1) = atan2( sbeta, cbeta );
		RPYAng(2) = atan2( R(2,1),R(2,2) );
	}

	return RPYAng;
}

Matrix pmpBlock::RPY2Rot (const Vector &RPY_deg)
{
	Matrix R(3,3);
	//R.eye();

	double cr = cos(RPY_deg(0)*CTRL_DEG2RAD);
	double sr = sin(RPY_deg(0)*CTRL_DEG2RAD);
	double cp = cos(RPY_deg(1)*CTRL_DEG2RAD);
	double sp = sin(RPY_deg(1)*CTRL_DEG2RAD);
	double cy = cos(RPY_deg(2)*CTRL_DEG2RAD);
	double sy = sin(RPY_deg(2)*CTRL_DEG2RAD);

	R(0,0) = cr*cp;
	R(1,0) = sr*cp;
	R(2,0) = -sp;

	R(0,1) = cr*sp*sy - sr*cy;
	R(1,1) = sr*sp*sy + cr*cy;
	R(2,1) = cp*sy;

	R(0,2) = cr*sp*cy + sr*sy;
	R(1,2) = sr*sp*cy - cr*sy;
	R(2,2) = cp*cy;

	return R;
}



bool pmpBlock::setAngFromProperty(Property *options, string key, Vector &v)
{
	Bottle *bot = ( options->findGroup(key.c_str())).find(key.c_str() ).asList();

	if ( bot->size()!= v.size())
	{	
		printf("Option %s size not compatible\n", key.c_str());
		return false;
	}

	for(unsigned int i=0; i<v.size(); i++)
	{
		v(i) = bot->get(i).asDouble()*CTRL_DEG2RAD;
		//cout << v(i) << endl;
	}
	return true;
}
bool pmpBlock::setFromProperty(Property *options, string key, Vector &v)
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
bool pmpBlock::setFromProperty(Property *options, string key, Matrix &m)
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
bool pmpBlock::setAngFromProperty(Bottle *options, string key, Vector &v)
{
	Bottle *bot = ( options->findGroup(key.c_str())).find(key.c_str() ).asList();

	if ( bot->size()!= v.size())
	{	
		printf("Option %s size not compatible\n", key.c_str());
		return false;
	}

	for(unsigned int i=0; i<v.size(); i++)
	{
		v(i) = bot->get(i).asDouble()*CTRL_DEG2RAD;
		//cout << v(i) << endl;
	}
	return true;
}
bool pmpBlock::setFromProperty(Bottle *options, string key, Vector &v)
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
bool pmpBlock::setFromProperty(Bottle *options, string key, Matrix &m)
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
bool pmpBlock::setEyeMatrixFromVector(Vector v, Matrix &m)
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
