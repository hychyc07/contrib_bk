
#include "cartesianImpedance.h"
#include "stdio.h"

using namespace std;

#define SEND_COMMANDS
//#define IMPEDANCE_JOINT
#define IMPEDANCE_CARTESIAN

const double THRESHOLD = 3;

void cartesianImpedance::loop()
{
	int jnts = 0;
	pos->getAxes(&jnts);

	encs->getEncoders(encoders_deg.data());

	encoders_deg = encoders_deg.subVector(0,6);

	encoders_rad = (M_PI/180)*encoders_deg;

	gcomp_bottle = gcomp.read(false);


	if(!gcomp_bottle)
	{
		if(verbose)
			printf("No data on Gcomp port...\n");
		return;
	}

	for(int i=0;i<jnts; i++)
		homeTorques(i) = gcomp_bottle->get(i).asDouble();


#ifdef IMPEDANCE_JOINT
	Kj.resize(jnts, jnts);
	Kj.zero();
	Kj(0,0)= -0.1;     Kj(1,1)= -0.1;
	Kj(2,2)= -0.1;     Kj(3,3)= -0.05;
	cmdTorques = homeTorques + Kj * (encoders_deg - home_deg);
#endif

#ifdef IMPEDANCE_CARTESIAN

	eigvec_mat_t = eigvec_mat.transposed();
	//Filling eigval matrix
	eigval_mat(0,0) = -stiff_1; eigval_mat(1,1) = -stiff_2; eigval_mat(2,2) = -stiff_3;

	//Building Kj....
	Matrix Kj_transl = eigvec_mat * eigval_mat * eigvec_mat_t;
	Kj.zero();
	for( int i=0; i<3; i++ ){
		for( int j=0; j<3; j++ ){
			Kj(i,j) = Kj_transl(i,j);
		}
	}

	jac = limb->GeoJacobian(encoders_rad);
	for(int i=0;i<7;i++)
		if(blockedIndex[i] ==1)
			jac.setCol(i,zero_vec);

	jac_t = jac.transposed();
//	multiplier = jac_t*Kj*jac;
	multiplier = jac_t*Kj;

	if(traj)
	{
		init_cart_pos(2) += sign_traj*TRAJ_STEP;
		if(init_cart_pos(2) > TRAJ_UP_ZLIM)
			sign_traj = -1;
		if(init_cart_pos(2) < TRAJ_LOW_ZLIM)
			sign_traj = 1;

	}


//	cmdTorques = homeTorques + multiplier*(encoders_rad - home_rad);

	// Fix this if want to use orientation impendance
	cmdTorques = homeTorques + multiplier*(limb->EndEffPose(encoders_rad).subVector(0,5) - init_cart_pos);

	//	fprintf(outFile, "%s\n",multiplier.toString().c_str());
#endif

#ifdef SEND_COMMANDS

	for(int i = 0; i < numberControlledJoints; i++)
		if (fabs(cmdTorques.data()[i] - homeTorques.data()[i]) < THRESHOLD)
			itrq->setRefTorque(i, cmdTorques.data()[i]);
		else
		{
			fprintf(stderr, "WARNING: saturating the current threshold!\n");
		}

	if(verbose)
		fprintf(stderr, "Trying to send trqs: %s \n", (cmdTorques.subVector(0,numberControlledJoints-1) - homeTorques.subVector(0,numberControlledJoints-1)).toString().c_str());

#else

	for(int i = 0; i < numberControlledJoints; i++)
		if (! (fabs(cmdTorques.data()[i] - homeTorques.data()[i]) < THRESHOLD))
			fprintf(stderr, "WARNING: saturating the current threshold on %d!\n", i);

	if(verbose)
		fprintf(stderr, "Trying to send trqs: %s \n", (cmdTorques.subVector(0,numberControlledJoints-1) - homeTorques.subVector(0,numberControlledJoints-1)).toString().c_str());

#endif

}

bool cartesianImpedance::open()
{
	std::string remotePorts="/";
	remotePorts+=robotname;
	remotePorts+="/"+controlled_part;

	std::string localPorts="/cartesianImpedance/"+controlled_part;

	Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPorts.c_str());   //local port names
	options.put("remote", remotePorts.c_str());         //where we connect to


	// create a device
	robotDevice.open(options);
	if (!robotDevice.isValid()) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		return 0;
	}

	bool ok;
	ok = robotDevice.view(pos);
	ok = ok && robotDevice.view(encs);
	ok = ok && robotDevice.view(ictrl);
	ok = ok && robotDevice.view(iimp);
	ok = ok && robotDevice.view(itrq);

	if (!ok) {
		printf("Problems acquiring interfaces\n");
		return false;
	}

	int axes;
	pos->getAxes(&axes);
	home_deg.resize(axes);
	home_rad.resize(axes);
	while(!encs->getEncoders(home_deg.data()))
	{
		fprintf(stderr, ".");
		Time::delay(0.1);
	}

	home_rad = (M_PI/180)*home_deg;


	gcomp.open((localPorts+"/gcomp").c_str());

	//remove this when running with App manager
	//	Network::connect(gcomp_port.c_str(),(localPorts+"/gcomp").c_str());

#ifdef SEND_COMMANDS
	pos->positionMove(home_deg.data());
#else
	printf( "Will move to: %s \n", home_deg.toString().c_str());
#endif
	//switching joints to torque control
	homeTorques.resize(axes);
	itrq->getTorques(homeTorques.data());

#ifdef SEND_COMMANDS
	for(int i = 0; i < numberControlledJoints; i++)
		ictrl->setTorqueMode(i);

	for(int i=0;i<7;i++)
		if(blockedIndex[i] ==1)
			ictrl->setPositionMode(i);		//for fixing shoulder joint due to inaccurate gravity comp

	for(int i = 0; i < numberControlledJoints; i++)
		itrq->setRefTorque(i, homeTorques.data()[i]);
	fprintf(stderr, "These will be home trqs: %s \n", homeTorques.subVector(0,3).toString().c_str());

#else
	printf("These will be home trqs: %s \n", homeTorques.subVector(0,3).toString().c_str());
#endif

#ifdef IMPEDANCE_CARTESIAN
	Kj.resize(6,6);
#endif

#ifdef IMPEDANCE_JOINT
	Kj.resize(axes, axes);
#endif

	cmdTorques.resize(axes);


	// Initialization - eigen decomposition of Cartesian Kj(:3,:3)
	eigval_mat.resize(3,3); eigval_mat.zero();
	eigvec_mat.resize(3,3); eigvec_mat.zero();
	eigvec_mat_t.resize(3,3);

	Vector axis(3);
	axis.zero();
	axis(0) =1;
	eigvec_mat.setCol(0,axis);
	axis(0) = 0; axis(1) = 1;
	eigvec_mat.setCol(1,axis);
	axis(1) = 0; axis(2) = 1;
	eigvec_mat.setCol(2,axis);

	init_cart_pos = limb->EndEffPose(home_rad).subVector(0,5);

	return true;
}

bool cartesianImpedance::close()
{

#ifdef SEND_COMMANDS
	for(int i = 0; i < numberControlledJoints; i++)
		ictrl->setPositionMode(i);

	pos->positionMove(home_deg.data());
#else
	printf("Will move to: %s \n", home_deg.toString().c_str());
#endif
	robotDevice.close();

	gcomp.close();
	return true;
}

bool cartesianImpedance::interrupt()
{
	return true;
}


int cartesianImpedance::checkOrtho(){
	/*
		Checks orthogonality of eigen vectors eig_X
		if not, returns false and orthogonalizes the user basis
	 */
	Vector eig_1, eig_2, eig_3;
	eig_1.resize(3); eig_2.resize(3); eig_3.resize(3);
	for( int i=0; i<3; i++ ){
		eig_1(i) = eigvec_mat(i,0);
		eig_2(i) = eigvec_mat(i,1);
		eig_3(i) = eigvec_mat(i,2);
	}
	double _ok  = 0.;
	_ok += fabs(dot(eig_1, eig_2));
	_ok += fabs(dot(eig_2, eig_3));
	_ok += fabs(dot(eig_1, eig_3));
	if( _ok < 3*(1 - cos(GSCHMIDT_THRESHOLD_DEG*M_PI/180)) && _ok > 1e-10){
		// Gram-Schmidt
		eig_2 = eig_2 - proj( eig_2, eig_1 );
		eig_3 = eig_3 - proj( eig_3, eig_1 ) - proj( eig_3, eig_2 );

		eigvec_mat.setCol(0,eig_1);
		eigvec_mat.setCol(1,eig_2);
		eigvec_mat.setCol(2,eig_3);
		return 1;
	}
	else if(_ok<1e-10)
		return 0;
	else
		return 2;
}

Vector cartesianImpedance::proj( Vector v, Vector in_u ){
	/*
		Returns Euclidean projection of vector v over vector in_u
	 */
	Vector result; result.resize(3); result.zero();
	double u_2 = dot(in_u, in_u);
	if( u_2 == 0. )
		return result;
	result = dot(v, in_u) / u_2 * in_u;
	return result;
}
