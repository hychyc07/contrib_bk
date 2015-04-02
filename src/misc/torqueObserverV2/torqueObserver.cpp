/* 
 * Copyright (C) <2010> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Matteo Fumagalli and Serena Ivaldi
 * email:   matteo.fumagalli@iit.it and serena.ivaldi@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/



#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "torqueObserver.h"
#include "armDynamics.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;
using namespace iCub::ctrl;
using namespace iCub::iDyn;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//			SENS TO  TORQUES         [RateThread]
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FILE *datas = fopen("ftMethodDiff.txt","w+");




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector SensToTorques::updateVelocity(const Vector &v)
{
	estElement.data=v;
	estElement.time=Time::now();
	return linEst->estimate(estElement);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector SensToTorques::updateAcceleration(const Vector &a)
{
	estElement.data=a;
	estElement.time=Time::now();
	return quadEst->estimate(estElement);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SensToTorques::SensToTorques(int _rate, PolyDriver *_dd, string _type, string _name)
:RateThread(_rate), dd(_dd)
{
	// attach ports
	
	type = _type;
    string fwdSlash = "/";
    string port = fwdSlash+_name;
    port += (fwdSlash+_type).c_str();
	// input port:  /moduleName/robotPart/FT:i
	port_FT = new BufferedPort<Vector>;	  
	port_Wrench = new BufferedPort<Vector>;	  
	port_estim_ext_Wrench = new BufferedPort<Vector>;	  
	port_Torques_limb = new BufferedPort<Bottle>;	
	port_external_Torques = new BufferedPort<Vector>;
	port_model_Torques = new BufferedPort<Vector>;	
	port_joint_Torques = new BufferedPort<Vector>;	
	port_projected_Wrench = new BufferedPort<Vector>;	
#ifdef HAS_EXT_FTSENSOR
	port_external_wrench = new BufferedPort<Vector>;
	port_jacobian_Torques = new BufferedPort<Vector>;	
#pragma message("messaggio")

#endif
	
	// output ports: /moduleName/robotPart/torques:o  /moduleName/robotPart/FTendeff:o
	port_FT->open((port+"/FT:i").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/FT:i").c_str()<<endl;
	port_Wrench->open((port+"/wrench:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/wrench:o").c_str()<<endl;
	port_estim_ext_Wrench->open((port+"/extEstimWrench:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/extEstimWrench:o").c_str()<<endl;
	port_Torques_limb->open((port+"/limbTorques:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/limbTorques:o").c_str()<<endl;
	port_external_Torques->open((port+"/externalTorques:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/externalTorques:o").c_str()<<endl;
	port_model_Torques->open((port+"/modelTorques:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/modelTorques:o").c_str()<<endl;
	port_joint_Torques->open((port+"/jointTorques:i").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/jointTorques:i").c_str()<<endl;
	port_projected_Wrench->open((port+"/projectedWrench:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/projectedWrench:o").c_str()<<endl;
	
#ifdef HAS_EXT_FTSENSOR
	port_external_wrench->open((port+"/externalWrench:i").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/externalWrench:i").c_str()<<endl;
	port_jacobian_Torques->open((port+"/jacobianTorques:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/jacobianTorques:o").c_str()<<endl;
#endif
	// initialize estimators of velocity and acceleration
	// they are necessary to avoid noise in this components
	linEst =new AWLinEstimator(16,1.0);
	quadEst=new AWQuadEstimator(25,1.0);

	// Checking device
	dd->view(iencs);
	// check leg type
	if((_type == "left_arm") || (_type == "right_arm"))
	{
		if(_type == "left_arm")
		{
			limb = new iCubArmDynV2("left");
			limbInv = new iCubArmDynV2("left");
		}
		else
		{
			limb = new iCubArmDynV2("right");
			limbInv = new iCubArmDynV2("right"); //@@@ it was left. maybe a bug here?
		}
		sens = new iDynSensorArmV2(dynamic_cast<iCubArmDynV2 *>(limb),DYNAMIC,VERBOSE);
		sensInv = new iDynInvSensorArmV2(dynamic_cast<iCubArmDynV2 *>(limbInv),DYNAMIC,VERBOSE);
	}
	else
	{
		if(_type == "left_leg")
		{
			limb = new iCubLegDyn("left");
			limbInv = new iCubLegDyn("left");
		}
		else
		{
			limb = new iCubLegDyn("right");
			limbInv = new iCubLegDyn("right");
		}
		sens = new iDynSensorLeg(dynamic_cast<iCubLegDyn *>(limb),DYNAMIC,VERBOSE);
		sensInv = new iDynInvSensorLeg(dynamic_cast<iCubLegDyn *>(limbInv),DYNAMIC,VERBOSE);
	}
	
	// the sensor solver for the leg
	int jnt1=0;

    iencs->getAxes(&jnt1);
    encoders.resize(jnt1);
	encoders.zero();


	// init all variables
	// first the unused ones
	// joints var

	fprintf(stderr,"\nJNT 1 : %d\n",jnt1);
	int jnt=jnt1;
	fprintf(stderr,"\nTotal JNT number: %d\n",jnt);

    q.resize(4,0.0);
    dq.resize(4,0.0);

    d2q.resize(4,0.0);

	/// the joints torques
	Tau.resize(4);
	externalTau.resize(4);
	/// the joints torques on limb
	limbTau.resize(4,0.0);

	q.zero(); dq.zero(); d2q.zero();
	// init Newt-Eul
	w0.resize(3);dw0.resize(3);d2p0.resize(3);Fend.resize(3);Mend.resize(3);
	w0.zero(); dw0.zero(); d2p0.zero();Fend.zero();Mend.zero();
	d2p0(1)=-9.81;
	// forces moments torques
	FTsensor.resize(6); FTsensor.zero();
	FTendeff.resize(6); FTendeff.zero();
	FTexternal.resize(6); FTexternal.zero();
	sensorOffset.resize(6); sensorOffset.zero();
	externalSensorOffset.resize(6); externalSensorOffset.zero();

	//set joints pos/vel/acc
	limb->setAng(q);
	limb->setDAng(dq);
	limb->setD2Ang(d2q);
	limb->prepareNewtonEuler(DYNAMIC);
	limb->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	
	limbInv->setAng(q);
	limbInv->setDAng(dq);
	limbInv->setD2Ang(d2q);
	limbInv->prepareNewtonEuler(DYNAMIC);
	limbInv->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);

	//other
	FTmeasure = NULL;
	FTprojected.resize(6,0.0);
	realJntTqs.resize(4,0.0);
//#pragma message("Your Message Here")

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool SensToTorques::threadInit()
{		 
	Time::delay(2.0);
	calibrateOffset(200);
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::run()
{	  
	
	// read encoders values, update limb model with pos/vel/acc
	// read FT sensor measurements
#ifdef TORQUE_PROJECTION
	static int hasConnection = 0;
	if(hasConnection<=0 && Network::exists("/armV2Sniffer/jointCalibratedTorques:o"))
	{
		Network::connect("/armV2Sniffer/jointCalibratedTorques:o","/tqObs/right_arm/jointTorques:i");
		hasConnection++;
	}
#endif
	readAndUpdate(false);
  	
	// estimate sensor FT
	//limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	//sensInv->computeSensorForceMoment();
	//Vector sensEstim = -1.0*sensInv->getSensorForceMoment();

	//compute torques
	sens->computeFromSensorNewtonEuler(-1.0*(FTsensor  - sensorOffset));
	//fprintf(stderr,".");
	
	limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	// compute on sensor
	sensInv->computeSensorForceMoment();
	Vector FTInv = -1.0*sensInv->getSensorForceMoment();
	//get torques and FTendeff
	Tau = sens->getTorques();
	Vector TauInv = sensInv->getTorques();
	//Vector Tinv = limbInv->getTorques();
	FTendeff = -1.0*sens->getForceMomentEndEff();
		
	if((type == "left_arm") || (type == "right_arm"))
	{
		for(unsigned int i=0;i<limbInv->getN();i++)
			limbTau(i) = Tau(i);
	} 
	Bottle a;
	if((type == "left_arm") || (type == "right_arm"))
		a.addInt(1);
	for(int i=0;i<limbTau.length();i++)
		a.addDouble(limbTau(i));

	port_Torques_limb->prepare() = a;
	port_external_Torques->prepare() = Tau-TauInv;
	port_model_Torques->prepare() = TauInv;
	port_Wrench->prepare() = FTInv;
	port_Wrench->write();
	port_estim_ext_Wrench->prepare() = FTendeff;
	port_estim_ext_Wrench->write();

	//send data
	port_Torques_limb->write();
	port_external_Torques->write();
	port_model_Torques->write();
	

#ifdef HAS_EXT_FTSENSOR
	Matrix T(6,6);T.zero();
	Matrix Rend=limbInv->getH();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			T(i,j)=Rend(i,j);
			T(i+3,j+3)=Rend(i,j);
		}
	}
	//fprintf(stderr,"%s\n",externalSensorOffset.toString().c_str());
	externalTau=limbInv->GeoJacobian().transposed()*T*(FTexternal - externalSensorOffset);
	port_jacobian_Torques->prepare()=externalTau;
	port_jacobian_Torques->write();
#endif
	
	port_jacobian_Torques->prepare()=externalTau;
	port_jacobian_Torques->write();


#ifdef TORQUE_PROJECTION
	jntTorques = port_joint_Torques->read(false);
	if((jntTorques!=0))
	{
		realJntTqs = *jntTorques;
		Matrix A=limbInv->GeoJacobian().transposed()*T;
		Matrix Ap=pinv(A.transposed()).transposed();
		FTprojected=Ap*realJntTqs;
	}
	else
	{
		cerr<<"ERR: port_external_wrench->read(waitMeasure) unable to read"<<endl;
	}
#endif
	port_projected_Wrench->prepare() = FTprojected;
	port_projected_Wrench->write();

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::threadRelease()
{
	fclose(datas);
	cerr<<"SensToTorques: deleting dynamic structures ..";
	if(sens)		{delete sens;		sens = NULL; cout<<"sens ";}
	if(sensInv)		{delete sensInv;	sensInv = NULL; cout<<"sensInv ";}
	if(linEst)		{delete linEst;		linEst = NULL; cout<<"linEst ";}
	if(quadEst)		{delete quadEst;	quadEst = NULL; cout<<"quadEst ";}
	//if(FTmeasure)	{delete FTmeasure;	FTmeasure = NULL; cout<<"FTmeasure "; }
	if(limb)		{delete limb;		limb=NULL; cout<<"limb ";}
	if(limbInv)		{delete limbInv;	limbInv=NULL; cout<<"limbInv ";}
	if(port_FT)		{delete port_FT; port_FT = NULL; cout<<"port_FT ";}	  
	if(port_Wrench)	{delete port_Wrench; port_Wrench = NULL; cout<<"port_Wrench ";}	  
	if(port_Torques_limb){delete port_Torques_limb; port_Torques_limb = NULL; cout<<"port_Torques_limb ";}
	if(port_external_Torques){delete port_external_Torques; port_external_Torques = NULL; cout<<"port_Torques_limb ";}	
	if(port_model_Torques){delete port_model_Torques; port_model_Torques = NULL; cout<<"port_Torques_limb ";}	  
	cout<<" .. done "<<endl;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::calibrateOffset(const unsigned int Ntrials)
{
	cout<<"SensToTorques: starting sensor offset calibration .."<<endl;

	sensorOffset.zero();
	// N trials to get a more accurate estimation
	for(unsigned int i=0; i<Ntrials; i++)
	{
		//read joints and ft sensor
		readAndUpdate(true);
		fprintf(stdout,"qsize: %d\t encs: %.1lf %.1lf %.1lf %.1lf\n",q.length(),q(0),q(1),q(2),q(3));
		// compute forces/moments/torques along the chain
		limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
		// compute on sensor
		sensInv->computeSensorForceMoment();
		// get an estimate of the force/moment in the sensor
		sensorOffset =  sensorOffset + FTsensor - (-1.0*sensInv->getSensorForceMoment());
#ifdef HAS_EXT_FTSENSOR
		externalSensorOffset = 	externalSensorOffset + FTexternal;
#endif
		 
	}

	sensorOffset = sensorOffset * (1.0/(double)(Ntrials));
	externalSensorOffset = externalSensorOffset * (1.0/(double)(Ntrials));

	cout<<"SensToTorques: sensor offset calibration completed"<<endl
		<<"               offset forces: "
		<<sensorOffset[0]<<"; "<<sensorOffset[1]<<"; "<<sensorOffset[2]<<"; "<<endl
		<<"               offset moment: "
		<<sensorOffset[3]<<"; "<<sensorOffset[4]<<"; "<<sensorOffset[5]<<"; "<<endl;

	Time::delay(2.0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::readAndUpdate(bool waitMeasure)
{
	unsigned int i;
	//read encoders values
	
	iencs->getEncoders(encoders.data());
	for( i=0;i<limbInv->getN();i++ )
		q(i) = encoders(i);
	
	//estimate velocity and accelerations
	dq = updateVelocity(q);
	d2q = updateAcceleration(q);

	// update joints
	limb->setAng(M_PI/180.0 * q);
	limb->setDAng(M_PI/180.0 * dq);
	limb->setD2Ang(M_PI/180.0 * d2q);

	limbInv->setAng(M_PI/180.0 * q);
	limbInv->setDAng(M_PI/180.0 * dq);
	limbInv->setD2Ang(M_PI/180.0 * d2q);

	//read FT sensor measurements
	FTmeasure = port_FT->read(waitMeasure);
	
#ifdef HAS_EXT_FTSENSOR
	FText = port_external_wrench->read(waitMeasure);
	if((FText!=0))
	{
		FTexternal = *FText;
	}
	else
	{
		cerr<<"ERR: port_external_wrench->read(waitMeasure) unable to read"<<endl;
	}
#endif
	if((FTmeasure!=0))
	{
		FTsensor = *FTmeasure;
	}
	else
	{
		cerr<<"ERR: port_FT->read(waitMeasure) unable to read"<<endl;
	}

	
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	  

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//			TORQUE OBSERVER            [RFModule]
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TorqueObserver::TorqueObserver()
{
	sens2torques= NULL;
	dd		= NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::checkDriver(PolyDriver *_dd)
{
	// check driver
    if(!_dd || !(_dd->isValid()))
	{
		cerr<<"TorqueObserver: error: could not instantiate the device driver. Closing."<<endl;
		return false;
	}
	//check encoders
	IEncoders *encs;
	if(!_dd->view(encs))
	{
		cerr<<"TorqueObserver: error: could not view encoders properly. Closing."<<endl;
		return false;
	}
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::configure(ResourceFinder &rf)
{
	string moduleName;	// tqObs
	string robotName;	// icub
	string robotPart;	// left_arm, right_leg
	string limb;		// leg, arm
	string type;		// right, left
	int rate;			// rate

	// getting parameters from RF: name, robot, part, rate
	if(rf.check("name")) 		moduleName = rf.find("name").asString();
	else 		moduleName = "tqObs";
	if (rf.check("robot"))		robotName = rf.find("robot").asString().c_str();
    else		robotName = "icub";
	if (rf.check("rate"))		rate = rf.find("rate").asInt();
    else		rate = 10;
	if (rf.check("part"))		robotPart = rf.find("part").asString().c_str();
    else
	{
		cerr<<"TorqueObserver: error in configuration: missing robot part in configuration parameters. Impossible to proceed."<<endl;
		Time::delay(3.0);
        return false;
	}

	//summary of config param 
	cout<<"TorqueObserver: module = "<<moduleName<<endl
		<<"                robot   = "<<robotName<<endl
		<<"                part   = "<<robotPart<<endl
		<<"                rate   = "<<rate<<" ms"<<endl;
	
	// now create the devices
	//device driver of the limb
	cout<<"TorqueObserver: creating "<<robotPart<<" polyDriver"<<endl;
	OptionsLimb.put("robot",robotName.c_str());
	OptionsLimb.put("part",robotPart.c_str());
	OptionsLimb.put("device","remote_controlboard");
	OptionsLimb.put("local",("/"+robotName+"/"+robotPart+"/client").c_str());
	OptionsLimb.put("remote",("/"+robotName+"/"+robotPart).c_str());
	dd = new PolyDriver(OptionsLimb);
	if(!checkDriver(dd)) 
	{
		cerr<<"TorqueObserver: error: unable to create /"<<robotName<<"/"<<robotPart<<" device driver...quitting"<<endl;
		return false;
	}


	// note: the arm needs the torso
	
	sens2torques = new SensToTorques(rate, dd, robotPart, moduleName);
	
	Time::delay(2.0);
	Network::connect("/icubV2/wrench_right_arm/analog:o","/tqObs/right_arm/externalWrench:i");
	//now the thread can start
	sens2torques->start();
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double TorqueObserver::getPeriod()	
{ 
	return 1; 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::updateModule() 
{ 
	return true; 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::close()
{
	cout<<"TorqueObserver is closing: "<<endl;
	if (sens2torques) 
	{
		sens2torques->stop();
		//delete sens2torques; 
		//sens2torques=NULL;
	}

	if (dd) {delete dd; dd=NULL;}

	Time::delay(1.0);

	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::interruptModule()
{
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::respond(const Bottle& command, Bottle& reply)
{
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

