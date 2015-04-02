#include "darwin/VTGS.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace Darwin::pmp;

VirtualTrajectoryGenerator::VirtualTrajectoryGenerator(Property *options)
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

	this->options = options;

	if(!initialize())
		printf("Initialization error\n");

	//traj.open("VirtualTraj.txt");
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
	numTargetPoints = 0;

	if (options->check("weights"))
	{	
		if(!setFromProperty("weights",weights))	
			return false;
	}
	else
		printf("--> Weights matrix initialized to default (1,1,1;0,0,0)\n");
	
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

// set a new triplet of critical points for trajectory formation
// x,y,z are on rows, points on cols.
bool VirtualTrajectoryGenerator::setTarget(Matrix tg)
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
bool VirtualTrajectoryGenerator::setTarget(Vector tg)
{
	if(tg.size() != x_tg.rows())
		return false;

	Matrix temp;
	temp = x_tg;

	// write new value in tail
	x_tg.setRow(0,temp.getRow(1));
	x_tg.setRow(1,temp.getRow(2));
	x_tg.setRow(2,tg);

	//numTargetPoints++;

	hasTarget = true;
	return true;
}

// set the trajectory starting point
bool VirtualTrajectoryGenerator::setStartingPoint(Vector x0)
{
	x_virtual = x0;

	cout << "VTGS starting point: " << x_virtual.toString() << endl;
	return true;
}
// Initialize tbg parameters
bool VirtualTrajectoryGenerator::setTbg(TimeBaseGenerator &tbg, double T_init, double T_dur, double SlopeRamp, double alpha)
{
	tbg.setT_init(T_init);
	tbg.setT_dur(T_dur);
	tbg.setSlopeRamp(SlopeRamp);
	tbg.setAlpha(alpha);

	return true;
}

// Initialize weights Matrix
bool VirtualTrajectoryGenerator::setWeights(Matrix w)
{
	if(w.rows() != weights.rows() || w.cols() != weights.cols())
		return false;

	weights = w;
	return true;
}

// Initialize weights as a FIFO
bool VirtualTrajectoryGenerator::setWeights(Vector w)
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

	for(int i=0; i<v.size(); i++)
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
