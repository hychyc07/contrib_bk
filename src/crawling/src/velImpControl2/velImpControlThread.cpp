// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Note: this is a modified version of the basic velocity control module to combine it with impedance control
#include "velImpControlThread.h"
#include <string.h>
#include <string>
#include <math.h>

#include <iostream>

#define SWING 1
#define STANCE 0
#define INIT -1

//switching between swing and stance (according to the direction of the shoulder/hip pitch joints)
#define SWITCH 1


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

const double MAX_SPEED=110;
const double AVERAGE=50;
const double ACCELERATIONS=1000000;
const double MAX_GAIN = 10.0;

//switch between stiffness
const double V_SWITCH = 0.0; // velocity where to switch (absolute value)
const double EPS_HYST = 0.5; //hysteresis parameter for the switch

const int VELOCITY_INDEX_OFFSET=1000;



velImpControlThread::velImpControlThread(int rate):
				yarp::os::RateThread(rate)
{
	control_rate = rate;
	first_command = 0;
	impedance_enabled = true;
	
}

velImpControlThread::~velImpControlThread()
{}

void velImpControlThread::compute_stiffness(yarp::sig::Vector stiff_in, yarp::sig::Vector damp_in, yarp::sig::Vector& stiff_out, yarp::sig::Vector& damp_out)
{
	const double GAIN = 4.077683537e+00;
	static double s_xv[2][20]={0,0,0,0,0,0,0,0,0,0,0,0};
	static double s_yv[2][20]={0,0,0,0,0,0,0,0,0,0,0,0};
	static double d_xv[2][20]={0,0,0,0,0,0,0,0,0,0,0,0};
	static double d_yv[2][20]={0,0,0,0,0,0,0,0,0,0,0,0};

	for (int j=0; j<nJoints; j++)
    {
		s_xv[0][j] = s_xv[1][j]; 
        s_xv[1][j] = stiff_in[j] / GAIN;
        s_yv[0][j] = s_yv[1][j]; 
        s_yv[1][j] =   (s_xv[0][j] + s_xv[1][j]) + (  0.5095254495 * s_yv[0][j]);
        stiff_out[j] = s_yv[1][j];
	}

	for (int j=0; j<nJoints; j++)
	{
		d_xv[0][j] = d_xv[1][j]; 
        d_xv[1][j] = damp_in[j] / GAIN;
        d_yv[0][j] = d_yv[1][j]; 
        d_yv[1][j] =   (d_xv[0][j] + d_xv[1][j]) + (  0.5095254495 * d_yv[0][j]);
        damp_out[j] = d_yv[1][j];
	}
}

void velImpControlThread::run()
{
	double t_start = yarp::os::Time::now();


	if (getIterations()>100)
	{
		fprintf(stderr, "Thread ran %d times, est period %lf[ms], used %lf[ms]\n",
				getIterations(),
				getEstPeriod(),
				getEstUsed());
		resetStat();
	}
	_mutex.wait();

	////getting new commands from the fast command port
	//this command receives also feedforward velocities
	yarp::os::Bottle *bot = command_port.read(false);
	if(bot!=NULL)
	{
        nb_void_loops = 0;
		//fprintf(stderr, "\n Receiving command: \t");
		int size = bot->size()/2;
		for(int i=0;i<size;i++)
		{	
			int ind = bot->get(2*i).asInt();
			if(ind < VELOCITY_INDEX_OFFSET) 
			{//this is a position command
				targets(ind) = bot->get(2*i+1).asDouble();
			} 
			else 
			{//this is a velocity command
				ffVelocities(ind - VELOCITY_INDEX_OFFSET) = bot->get(2*i+1).asDouble();
			}
			//fprintf(stderr, "for joint *%d, received %f, \t", ind, targets(ind));
		}
		first_command++;
	} else {
        nb_void_loops++;
        if(nb_void_loops > 5) {
            ffVelocities = 0.0;
        }
    }

#if SWITCH    
    switchImp(ffVelocities[0]); //change stiffness according to shoulder/hips velocities
	compute_stiffness(requestedStiff, requestedDamp, currStiff, currDamp);
	//printf("0: %+3.5f %+3.5f %+3.5f %+3.5f *** ",requestedStiff[0], requestedDamp[0], currStiff[0], currDamp[0]);
	//printf("1: %+3.5f %+3.5f %+3.5f %+3.5f *** ",requestedStiff[1], requestedDamp[1], currStiff[1], currDamp[1]);
	//printf("2: %+3.5f %+3.5f %+3.5f %+3.5f *** ",requestedStiff[2], requestedDamp[2], currStiff[2], currDamp[2]);
	//printf("\n");
	if (impedance_enabled==true)
	{
		for(int i=0; i< nJoints; i++) iimp->setImpedance(i,  currStiff[i], currDamp[i]);
	}
#endif
	Bottle stiffness_output;
	Bottle damping_output;
	Bottle velocity_output;
    for(int i=0; i< nJoints; i++) 
	{
		stiffness_output.addDouble(currStiff[i]);
		damping_output.addDouble(currDamp[i]);
		velocity_output.addDouble(command[i]);
	}
	Stamp stmp;
	stmp = itime->getLastInputStamp();
	if (stmp.isValid())
	{
		stiffness_port.setEnvelope(stmp);
		damping_port.setEnvelope(stmp);
		velocity_port.setEnvelope(stmp);
	}
	else
	{
		stmp=Stamp(-1,0.0);
		stiffness_port.setEnvelope(stmp);
		damping_port.setEnvelope(stmp);
		velocity_port.setEnvelope(stmp);
	}
    velocity_port.write(velocity_output);
	stiffness_port.write(stiffness_output);
	damping_port.write(damping_output);

	//getting commands from the slow port
	yarp::sig::Vector *vec = command_port2.read(false);
	if (vec!=0)
	{
		targets=*vec;
		first_command++;
	}

	static int count=0;
	count++;

    // normale by randaz
	ienc->getEncoders(encoders.data());
    
    // versione che prende direttam la refernce del pid
    for(int i=0; i<nJoints; i++)
    {
    		if(impContr[i]==1)
    		{
    			ipid->getReference(i, encoders_ref.data()+i);
                printf("%d :  %f vs %f \n",i,encoders(i),encoders_ref(i));
    		}
    }

	//ienc->getEncoderSpeeds(encoders_speed.data());
/*	fprintf(stderr, "printing to file \n");
//#if 0
	 for(int i=0;i<nJoints;i++)
	 {
		printf("%f ",encoders(i));
		//fprintf(currentSpeedFile,"%f ",encoders(i));
	 }
	//fprintf(currentSpeedFile,"%f\n",t_start-time_watch);
	 printf("\n");
	 for(int i=0;i<nJoints;i++)
	 {
		printf("%f ",encoders_ref(i));
	 }
	 printf("\n");

	 for(int i=0;i<nJoints;i++)
	 {
		printf("%d :  %f vs %f \n",i,encoders(i),encoders_ref(i));
	 }
	 printf("\n");*/

//#endif

	Kd=0.0;

	for(int k=0;k<nJoints;k++)
	{
		double current_err = targets(k)-encoders_ref(k);
		error_d(k) = (current_err - error(k))/((double)control_rate)*1000.0;
		error(k) = current_err;

		//we calculate the command Adding the ffVelocities
		command(k) = Kp(k)*error(k) + Kd(k)*error_d(k) + ffVelocities(k);
	}
	

	//    std::cout << command.toString() << std::endl;

	limitSpeed(command);

	if (suspended)
		command=0;

#if 0
	for(int i=0;i<nJoints;i++)
		fprintf(targetSpeedFile,"%f ",command(i));
	fprintf(targetSpeedFile,"%f\n",t_start-time_watch);
#endif

	if(first_command) {
		int trials = 0;
		while(!ivel->velocityMove(command.data())){
			trials++;
			fprintf(stderr,"velcontrol ERROR>> velocity move sent false\n");
			if(trials>10) {
				fprintf(stderr, "velcontrol ERROR>> tried 10 times to velocityMove, halting...\n");
				this->halt();
				break;
			}
		}
	}
	_mutex.post();

}

bool velImpControlThread::threadInit()
{
	suspended=false;
	ienc->getEncoders(encoders.data());
    ipid->getReferences(encoders_ref.data());
	//ienc->getEncoderSpeeds(encoders_speed.data());
	targets=encoders;
	ffVelocities = 0;
	count = 0;
	time_watch = Time::now();
	time_loop = 0.0;
	first_command=0;

	return true;
}

void velImpControlThread::threadRelease()
{
	for(int k=0;k<nJoints;k++)
	{
		command(k)=0;
		ivel->velocityMove(k, command[k]);
        setGain(k,0);
        setVel(k,0);
        ictrl->setPositionMode(k); //we set the position mode to be sure to have high stiffness
	}
	
	command_port.close();
	command_port2.close();
	stiffness_port.close();
	damping_port.close();
	velocity_port.close();

	fclose(stiffFile);
#if 0
	fclose(targetSpeedFile);
	fclose(currentSpeedFile);

#endif
}

bool velImpControlThread::init(PolyDriver *d, ConstString partName, ConstString robotName, bool _impedance_enabled)
{
	impedance_enabled = _impedance_enabled;
	char tmp[255];
	
	limbName = partName;

	yarp::os::Time::turboBoost();

    nb_void_loops = 0;
    
	///opening port for fast transfer of position command
	sprintf(tmp,"/%s/vc/%s/fastCommand", robotName.c_str(), partName.c_str());
	fprintf(stderr,"opening port for part %s\n",tmp);
	command_port.open(tmp);
	

	std::string tmp2;
	tmp2="/";
	tmp2+=robotName.c_str();
	tmp2+="/vc/";
	tmp2+=partName.c_str();

	command_port2.open(std::string(tmp2+"/command").c_str());
	stiffness_port.open(std::string(tmp2+"/stiffness:o").c_str());
	damping_port.open(std::string(tmp2+"/damping:o").c_str());
	velocity_port.open(std::string(tmp2+"/velocity:o").c_str());

	if (d==0)
		return false;

	driver=d;

	driver->view(ivel);
	driver->view(ienc);
    driver->view(ipid);
	driver->view(ictrl);
	driver->view(iimp);
	driver->view(itime);
	

	if ( (ivel==0)||(ienc==0) || (iimp==0)||(ictrl==0)||(ipid==0))
	{
		printf("velContr::init >> failed to open a device\n"); 
		return false;
	}

	ivel->getAxes(&nJoints);

	fprintf(stderr,"controlling %d DOFs\n",nJoints);

	Vector accs;
	accs.resize(nJoints);
	ivel->getRefAccelerations(accs.data());
	accs=ACCELERATIONS;
	ivel->setRefAccelerations(accs.data());
	
	
	for(int i=0;i<nJoints;i++)
	{
		ictrl->setPositionMode(i);	//we set the position mode to be sure to have high stiffness
	}
	
	for(int i=0;i<nJoints;i++)
	{
		if(impContr[i]==1)
		{
			requestedStiff[i]=currStiff[i]=stanceStiff[i];
			requestedDamp[i]=currDamp[i]=stanceDamp[i];
			ictrl->setImpedancePositionMode(i);	//we set the position mode to be sure to have high stiffness
			iimp->setImpedance(i, requestedStiff[i], requestedDamp[i]);
		}
	}


	encoders.resize(nJoints);
    encoders_ref.resize(nJoints);
	encoders_speed.resize(nJoints);
	command.resize(nJoints);
	targets.resize(nJoints);
	ffVelocities.resize(nJoints);
	command=0;
	targets=0;
	ffVelocities = 0;
	Kp.resize(nJoints);
	Kp=0;
	Kd.resize(nJoints);
	Kd=0;
	error.resize(nJoints);
	error=0;
	error_d.resize(nJoints);
	error_d=0;
	state = INIT;

	maxVel.resize(nJoints);
	maxVel = 0.0;
	

	
#if 0 
	sprintf(tmp,"%s_target_speed.dat",partName.c_str());
	targetSpeedFile = fopen(tmp,"w");
	sprintf(tmp,"%s_current_speed.dat",partName.c_str());
	currentSpeedFile = fopen(tmp,"w");
#endif

	char file_name[255];
    sprintf(file_name, "%s_impedance.dat", partName.c_str());
	stiffFile = fopen(file_name, "w");

	return true;
}

void velImpControlThread::halt()
{
	suspended=true;
	for(int k=0;k<nJoints;k++)
	{
		command(k)=0;
		ivel->velocityMove(k, command[k]);
	}
	fprintf(stderr, "Suspended\n");
	targets=encoders;
	ffVelocities = 0;
}

void velImpControlThread::go()
{
	suspended=false;
	fprintf(stderr, "Run\n");
	targets=encoders;
	ffVelocities = 0;
}

void velImpControlThread::setRef(int i, double pos)
{
	fprintf(stderr, "Setting new target %d to %lf\n", i, pos);

	_mutex.wait();
	targets(i)=pos;
	ffVelocities(i)=0;
	first_command++;
	_mutex.post();
}

void velImpControlThread::setVel(int i, double vel)
{
	_mutex.wait();

	if((vel >= 0.0) && (vel < MAX_SPEED) && (i>=0) && (i<nJoints))
	{
		maxVel(i) = vel;
		fprintf(stderr,"setting max vel of joint %d to %f\n",i,maxVel(i));
	}
	else
		fprintf(stderr,"impossible to set max vel higher than %f\n", MAX_SPEED);

	_mutex.post();
}

void velImpControlThread::setGain(int i, double gain)
{
	_mutex.wait();

	if (gain>MAX_GAIN)
		gain=MAX_GAIN;

	if((gain >= 0.0) && (i>=0) && (i<nJoints))
	{
		Kp(i) = gain;
		fprintf(stderr,"setting gain for joint %d to %lf\n", i, Kp(i));
	}
	else
		fprintf(stderr,"cannot set gain of joint %d\n",i);

	_mutex.post();
}


void velImpControlThread::limitSpeed(Vector &v)
{
	for(int k=0; k<nJoints;k++)
	{
		if(command(k)!=command(k))//check not Nan
		{
			command(k)=0.0;
			fprintf(stderr,"WARNING::Receiving NaN values\n");
		}
		if (fabs(command(k))>maxVel(k))
		{
			if (command(k)>0)
				command(k)=maxVel(k);
			else
				command(k)=-maxVel(k);
		}
	}
}

void velImpControlThread::switchImp(double vel)
{
	//we change the stiffness depending on whether the limb is in swing or stance
	//arm shoulder pitch: negative velocity => swing, positive velocity => stance
	//leg hip pitch: positive velocity => swing, negative velocity => stance
	
	if(limbName=="left_arm" || limbName=="right_arm")
	{
		if(state == STANCE || state ==INIT)
		{
			if(vel < -V_SWITCH - EPS_HYST )
			{
				state = SWING;
				for(int i=0; i< nJoints; i++)
				{
					if(impContr[i]==1)
					{
						requestedStiff[i]=swingStiff[i];
						requestedDamp[i] =swingDamp[i];
						//iimp->setImpedance(i,  requestedStiff[i], requestedDamp[i]);
					}
				}
			}
		}
		
		if(state == SWING || state ==INIT)
		{
			if(vel > -V_SWITCH + EPS_HYST )
			{
				state = STANCE;
			    for(int i=0; i< nJoints; i++)
				{
					if(impContr[i]==1)
					{
						requestedStiff[i]=stanceStiff[i];
						requestedDamp[i] =stanceDamp[i];
						//iimp->setImpedance(i, requestedStiff[i], requestedDamp[i]);
					}
				}
			}
		}
	}
		
	if(limbName == "left_leg" || limbName == "right_leg")
	{
		if(state == STANCE || state ==INIT)
		{
			if(vel > V_SWITCH + EPS_HYST )
			{
				state = SWING;
				for(int i=0; i< nJoints; i++)
				{
					if(impContr[i]==1)
					{
						requestedStiff[i]=swingStiff[i];
						requestedDamp[i] =swingDamp[i];
						//iimp->setImpedance(i,  requestedStiff[i], requestedDamp[i]);
					}
				}
			}
		}
		
		if(state == SWING || state ==INIT)
		{
			if(vel < V_SWITCH -EPS_HYST )
			{
				state = STANCE;
				for(int i=0; i< nJoints; i++)
				{
					if(impContr[i]==1)
					{
						requestedStiff[i]=stanceStiff[i];
						requestedDamp[i] =stanceDamp[i];
						//iimp->setImpedance(i, requestedStiff[i], requestedDamp[i]);
					}
				}
			}
		}	
	}	
	
	//fprintf(stiffFile, "%f %d %f \n",vel , state, Time::now());
}
	
	    
