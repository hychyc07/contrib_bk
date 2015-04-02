#include <pmp_lib/core/VTGS.h>
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::pmplib::core;

VirtualTrajectoryGenerator::VirtualTrajectoryGenerator(const Property *options)
{
	weights.resize(3,3);
	weights.zero();
	weights(0,0) = 1; // default is straight trajectory;
	weights(0,1) = 1; // default is straight trajectory;
	weights(0,2) = 1; // default is straight trajectory;
	//weights(1) = 0;
	x_virtual.resize(3,0.0);
	x_tg.resize(3,3);
	x_tg.zero();
	tgPose.resize(3,3);
	virtualPose.resize(3,3);
	initPose.resize(3,3);
	axisAngle.resize(4,0.0);
	dRot.resize(3,3);
	theta = 0;
	dRot.eye();

	this->options = new Property(*options);

	if(!initialize())
		printf("Initialization error\n");

	//traj.open("VirtualTraj.txt");
}

VirtualTrajectoryGenerator::VirtualTrajectoryGenerator(const VirtualTrajectoryGenerator &v)
{
	this->x_virtual.resize(3,0.0);
	this->x_tg.resize(3,3);
	this->tgPose.resize(3,3);
	this->virtualPose.resize(3,3);
	this->axisAngle.resize(4,0.0);
	this->dRot.resize(3,3);
	this->weights.resize(3,3);
	this->options = new Property(*v.options);
	this->weights = v.weights;
	this->x_tg = v.x_tg;
	this->x_virtual = v.x_virtual;
	this->tgPose = v.tgPose;
	this->virtualPose = v.virtualPose;
	this->initPose = v.initPose;
	this->axisAngle = v.axisAngle;
	this->dRot = v.dRot;
	this->theta = v.theta;
	this->x_virtual = v.x_virtual;
	this->tbg1 = v.tbg1;
	this->tbg2 = v.tbg2;
	this->hasTarget = v.hasTarget;
	this->hasPose = v.hasPose;
	this->hasInitialPose = v.hasInitialPose;
	this->numTargetPoints = v.numTargetPoints;
}

VirtualTrajectoryGenerator::~VirtualTrajectoryGenerator()
{
}

// Initialization of VTGS object (weights, tbg1, tbg2) from class property
bool VirtualTrajectoryGenerator::initialize()
{
	// set tbg initial parameters value
	// set weights vector size and value

	hasTarget = false;
	hasPose = false;
	hasInitialPose = false;
	numTargetPoints = 0;

	if (options->check("weights"))
	{	
		if(!setFromProperty("weights",weights))	
			return false;
	}
	//else
		//printf("--> Weights matrix initialized to default (1,1,1;0,0,0)\n");
	
	// set tbg1's parameters
	if (options->check("T_init1"))
		tbg1.setT_init(options->find("T_init1").asDouble());
	if (options->check("T_dur1"))
		tbg1.setT_dur(options->find("T_dur1").asDouble());
	if (options->check("SlopeRamp1"))
		tbg1.setSlopeRamp(options->find("SlopeRamp1").asDouble());
	if (options->check("alpha1"))
		tbg1.setAlpha(options->find("alpha1").asDouble());

	// set tbg2's parameters
	if (options->check("T_init2"))
		tbg2.setT_init(options->find("T_init2").asDouble());
	if (options->check("T_dur2"))
		tbg2.setT_dur(options->find("T_dur2").asDouble());
	if (options->check("SlopeRamp2"))
		tbg2.setSlopeRamp(options->find("SlopeRamp2").asDouble());
	if (options->check("alpha2"))
		tbg2.setAlpha(options->find("alpha2").asDouble());

	return true;
}

bool VirtualTrajectoryGenerator::update(const Property &opt)
{
	// set tbg initial parameters value
	// set weights vector size and value
	options->clear();
	*options = opt;

	hasTarget = false;
	numTargetPoints = 0;

	if (options->check("weights"))
	{	
		if(!setFromProperty("weights",weights))	
			return false;
	}
	//else
		//printf("--> Weights matrix initialized to default (1,1,1;0,0,0)\n");
	
	// set tbg1's parameters
	if (options->check("T_init1"))
		tbg1.setT_init(options->find("T_init1").asDouble());
	if (options->check("T_dur1"))
		tbg1.setT_dur(options->find("T_dur1").asDouble());
	if (options->check("SlopeRamp1"))
		tbg1.setSlopeRamp(options->find("SlopeRamp1").asDouble());
	if (options->check("alpha1"))
		tbg1.setAlpha(options->find("alpha1").asDouble());

	// set tbg2's parameters
	if (options->check("T_init2"))
		tbg2.setT_init(options->find("T_init2").asDouble());
	if (options->check("T_dur2"))
		tbg2.setT_dur(options->find("T_dur2").asDouble());
	if (options->check("SlopeRamp2"))
		tbg2.setSlopeRamp(options->find("SlopeRamp2").asDouble());
	if (options->check("alpha2"))
		tbg2.setAlpha(options->find("alpha2").asDouble());

	return true;
}


// compute the new time sample of virtual trajectory, given as a 3D point x_virtual
void VirtualTrajectoryGenerator::getNextVirtualPosition(Vector &x)
{
	// Find new target point on the trajectory (x_virtual) considering it as a straight path (weights = 1,0).

	/*
	double firstPointStep  = weights(0)*tbg1.calculateGamma(time)*tbg1.getSlopeRamp();
	double secondPointStep = weights(1)*tbg2.calculateGamma(time)*tbg2.getSlopeRamp();
	x_virtual = x_virtual + (x_tg.getRow(0)-x_virtual)*firstPointStep + (x_tg.getRow(1)-x_virtual)*secondPointStep;
	*/

	double firstPointStep  = tbg1.calculateGamma(time,true)*tbg1.getSlopeRamp();
	double secondPointStep = tbg2.calculateGamma(time,true)*tbg2.getSlopeRamp();
	x_virtual = x_virtual + weights.getRow(0)*(x_tg.getRow(0)-x_virtual)*firstPointStep + 
							weights.getRow(1)*(x_tg.getRow(1)-x_virtual)*secondPointStep;

	x = x_virtual;
	
	//traj <<x_virtual.toString()<< endl; 
}

Vector VirtualTrajectoryGenerator::getNextVirtualPosition(const int &iter)
{
	double firstPointStep  = tbg1.calculateGamma(iter,true)*tbg1.getSlopeRamp();
	double secondPointStep = tbg2.calculateGamma(iter,true)*tbg2.getSlopeRamp();
	x_virtual = x_virtual + weights.getRow(0)*(x_tg.getRow(0)-x_virtual)*firstPointStep + 
							weights.getRow(1)*(x_tg.getRow(1)-x_virtual)*secondPointStep;

	return x_virtual;
	
	//traj <<x_virtual.toString()<< endl; 
}

Matrix VirtualTrajectoryGenerator::getNextVirtualPose(const int &iter)
{
	double Step  = tbg1.calculateGamma(iter,true)*tbg1.getSlopeRamp();
	if(Step==0.0) return virtualPose;
	double dtheta = (axisAngle(3)-theta)*Step;
	theta += dtheta;
	
	Matrix I(3,3);
	I.eye();
	Vector v(4,axisAngle.data());
	v(3) = theta;
	Matrix dR = iCub::ctrl::axis2dcm(v).submatrix(0,2,0,2);

	virtualPose = initPose*dR;
	//virtualPose = (I+dtheta*dRot)*virtualPose;

	//cout << dtheta << " " << axisAngle(3) << endl;
	//cout << virtualPose.toString() << endl << endl;
	//cout << dRot.toString() << endl;
	return virtualPose;
	//traj <<x_virtual.toString()<< endl; 
}

Vector VirtualTrajectoryGenerator::getAxisAngle(const Matrix &initPose, const Matrix &tgPose)
{
	Matrix relativeR = initPose.transposed()*tgPose;
	axisAngle = iCub::ctrl::dcm2axis(relativeR);

	dRot.zero();
	dRot(0,1) = axisAngle(2);
	dRot(1,0) = -axisAngle(2);
	dRot(0,2) = -axisAngle(1);
	dRot(2,0) = axisAngle(1);
	dRot(2,1) = axisAngle(0);
	dRot(1,2) = -axisAngle(0);

	return axisAngle;
}

// set a new triplet of critical points for trajectory formation
// x,y,z are on rows, points on cols.
bool VirtualTrajectoryGenerator::setTarget(const Matrix &tg)
{
	x_tg.zero();

	int i = 0;
	for (i=0; i<tg.rows(); i++ )
		x_tg.setRow(i, tg.getRow(i));
	
	numTargetPoints = i;
	hasTarget = true;

	return true;
}

// set the new target vector shifting the second and the third value one position right
// and inserting the new value at the end of the vector. x,y,z are on rows, points are on cols.
bool VirtualTrajectoryGenerator::setTarget(const Vector &tg, bool back = true)
{
	if(tg.size() != x_tg.rows())
		return false;

	Matrix temp;
	temp = x_tg;

	if (back)
	{
		// write new value in tail
		x_tg.setRow(0,temp.getRow(1));
		x_tg.setRow(1,temp.getRow(2));
		x_tg.setRow(2,tg);
	}
	else
	{
		// write new value in head
		x_tg.setRow(0,tg);
		x_tg.setRow(1,temp.getRow(0));
		x_tg.setRow(2,temp.getRow(1));
	}

	//numTargetPoints++;

	hasTarget = true;
	return true;
}


bool VirtualTrajectoryGenerator::setTargetPose(const Matrix &tgPose)
{
	if( tgPose.rows()*(tgPose.cols()) != this->tgPose.rows()*(this->tgPose.cols()) )
		return false;

	this->tgPose = tgPose;

	// compute total angular displacement in axis/angle notation:
	if (hasInitialPose) axisAngle = getAxisAngle(initPose,tgPose);

	hasPose = true;
	return true;
}
// set the trajectory starting point
bool VirtualTrajectoryGenerator::setStartingPoint(const Vector &x0)
{
	x_virtual = x0;

	//cout << "VTGS starting point: " << x_virtual.toString() << endl;
	return true;
}

bool VirtualTrajectoryGenerator::setStartingPose(const Matrix &iniPose)
{
	virtualPose = initPose = iniPose;
	theta = 0.0;

	// compute total angular displacement in axis/angle notation:
	if (hasPose) axisAngle = getAxisAngle(virtualPose,tgPose);

	//cout << "VTGS starting pose: " << virtualPose.toString() << endl;
	return true;
}
// Initialize tbg parameters
bool VirtualTrajectoryGenerator::setTbg(TimeBaseGenerator &tbg, 
										const double &T_init, 
										const double &T_dur, 
										const double &SlopeRamp,
										const double &alpha)
{
	tbg.setT_init(T_init);
	tbg.setT_dur(T_dur);
	tbg.setSlopeRamp(SlopeRamp);
	tbg.setAlpha(alpha);

	return true;
}

// Initialize weights Matrix
bool VirtualTrajectoryGenerator::setWeights(const Matrix &w)
{
	if(w.rows() != weights.rows() || w.cols() != weights.cols())
		return false;

	weights = w;
	return true;
}

// Initialize weights as a FIFO
bool VirtualTrajectoryGenerator::setWeights(const Vector &w)
{
	if(w.size() != weights.cols())
		return false;

	Matrix temp;
	temp = weights;

	// write new value in tail
	weights.setRow(0,temp.getRow(1));
	weights.setRow(1,temp.getRow(2));
	weights.setRow(2,w);
	
	return true;
	
}

Matrix VirtualTrajectoryGenerator::getTarget()
{
	return x_tg;
}
/*
Vector VirtualTrajectoryGenerator::getTarget()
{
	return x_tg.getRow(0);
}
*/
Matrix VirtualTrajectoryGenerator::getWeights()
{
	return weights;
}

void VirtualTrajectoryGenerator::run(unsigned int iter, Vector &x_target)
{
	// write the next virtual position in pos vector
	runSem.wait();
	Vector par;

	Matrix xtg(3,3);

if(iter == 0)
{
	//-289.6071  133.9407  261.2717
	//x_virtual(0) = -289.6071;
	//x_virtual(1) = 133.9407;
	//x_virtual(2) = 261.2717;
	//xtg.zero();
	//xtg(0,0) = -300;
	//xtg(0,1) = -200;
	//xtg(0,2) = 300;

	//weights(0) = 1;
	//weights(1) = 0;
	
	//x_tg = xtg;
}
	time = iter;
	
	getNextVirtualPosition(x_target);

	//if(iter == 1003)
		//traj.close();

	runSem.post();
}

bool VirtualTrajectoryGenerator::Bottle2Vector(Bottle *bot, Vector &v)
{
	if(bot->size() != v.size())
	{
		printf("Option size not compatible\n");
		return false;
	}
	for (int i=0; i<bot->size(); i++)
		v(i) = bot->get(i).asDouble();
	return true;
}

bool VirtualTrajectoryGenerator::setFromProperty(string key, Vector &v)
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

bool VirtualTrajectoryGenerator::setFromProperty(string key, Matrix &m)
{
	Bottle *bot = ( options->findGroup(key.c_str())).find(key.c_str() ).asList();

	if ( bot->size()!= (m.rows()+m.cols()))
	{	
		printf("Option %s size not compatible\n", key.c_str());
		return false;
	}

	int k=0;
	for(int i=0; i<m.rows(); i++)
		for(int j=0; i<m.cols(); i++)
		{
			m(i,j) = bot->get(k).asDouble();
			k++;
			//cout << v(i) << endl;
		}
	return true;
}

int VirtualTrajectoryGenerator::maxIter()
{
	int iter1 = (int)(tbg1.getT_dur() + tbg1.getT_init())/tbg1.getSlopeRamp()+1;
	int iter2 = (int)(tbg2.getT_dur() + tbg2.getT_init())/tbg2.getSlopeRamp()+1;

	if (iter1 >= iter2)	return iter1;
	else				return iter2;
}


static bool getListFromProperty(Property &prop, const string &opt, Vector &list)
{
	if(!prop.check(opt.c_str()))
	{
		printf("Option not found\n");
		return false;
	}

	Bottle *val = prop.find(opt.c_str()).asList();

	if(val->size() != list.size())
	{
		printf("Option size not compatible\n");
		return false;
	}

	return true;

}
