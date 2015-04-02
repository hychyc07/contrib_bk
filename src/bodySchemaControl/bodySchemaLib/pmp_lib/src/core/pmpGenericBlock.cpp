#include <pmp_lib/core/tbg.h>
#include <pmp_lib/core/pmpGenericBlock.h>
#include <iCub/ctrl/math.h>

extern "C" {
    #include <gsl/gsl_eigen.h>
}

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace iCub::pmplib::core;

void pmpGenericBlock::allocateVariableSizeParams(int _DOF)
{
	if (_DOF == 0) return;

	Kint.resize(_DOF,_DOF);
	A.resize(_DOF,_DOF);
	q.resize(_DOF,0.0);
	q_ref.resize(_DOF,0.0);
	q_0.resize(_DOF,0.0);
	q_home.resize(_DOF,0.0);
	qdot.resize(_DOF,0.0);

	// private:
	halfJointRange.resize(_DOF,0.0);

	int dof = 0;
	for (unsigned int i=0; i<chain->getN(); i++)
	{
		if (!chain->isLinkBlocked(i))
		{
			double min = (*chain)(0).getMin();
			double max = (*chain)(0).getMax();
			halfJointRange(dof) = 0.5*abs((max - min));
			dof ++;
		}
	}
}


pmpGenericBlock::pmpGenericBlock() : chain(0), N(0)
{
	// initialize vectors and matrices that do not depend on robot's DOF:		
		F.resize(6,0.0);		// Cartesian Virtual Forces vector
		K.resize(3,3);			// Cartesian Virtual Stiffness matrix
		Km.resize(3,3);			// Cartesian Virtual Stiffness matrix for momentum
		x_0.resize(3,0.0);	    // Initial position vector
		x.resize(3,0.0);		// Final position vector
		refPose.resize(3,0.0);  // EE reference pose

		// Anisotropy Compensation
		tg_old.resize(3,0.0);
		dv_old = 0.0;
		rpy_old.resize(3,0.0);
		rpy_tg.resize(3,0.0);

		// Others
		ref_active = 0;
		scale = 1000;
}

pmpGenericBlock::pmpGenericBlock(const pmpGenericBlock &block): F(block.F),
																K(block.K),
																Km(block.Km),
																x_0(block.x_0),
																x(block.x),
																refPose(block.refPose),
																tg_old(block.tg_old),
																rpy_old(block.rpy_old),
																rpy_tg(block.rpy_tg),
																comp(block.comp),
																chain(0), N(0)

{
	dv_old = block.dv_old;
	ref_active = block.ref_active;
	scale = block.scale;
}

pmpGenericBlock & pmpGenericBlock::operator=(const pmpGenericBlock &block)
{
	if(this != &block)
	{
		this->F = block.F;
		this->K = block.K;
		this->x_0 = block.x_0;
		this->x = block.x;

		this->refPose = block.refPose;
		this->tg_old = block.tg_old;
		this->rpy_old = block.rpy_old;
		this->rpy_tg = block.rpy_tg;
		this->comp = block.comp;
		this->scale = block.scale;

		this->tbg = block.tbg;
		this->opt_chain = block.opt_chain;
		this->opt_tbg = block.opt_tbg;

		if (block.N != 0)
		{
			if (this->N == 0) delete chain;
			this->N = block.N;
			this->chain = new iKinChain(*block.chain);
			allocateVariableSizeParams(this->chain->getDOF());

			this->Kint = block.Kint;
			this->A = block.A;
			this->q = block.q;
			this->q_ref = block.q_ref;
			this->q_0 = block.q_0;
			this->q_home = block.q_home;
			this->qdot = block.qdot;

	// private:
			this->halfJointRange = block.halfJointRange;
		}
	}

	return *this;
}

Matrix pmpGenericBlock::align_K(const Vector &delta, const Matrix &K)
{
	// re-orient principal axes of L along (x_tg-x) direction:
	Matrix Krot = K;
	Matrix V(3,3);
	Vector x_axis(3,0.0);
	x_axis(0) = 1;
	//x_axis(1) = x_axis(2) = 0;
	
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

Vector pmpGenericBlock::calculate_F(const Vector &x_tg, const Vector &x, const Matrix &K)
{
	
	return ( align_K((x_tg-x),K)*(x_tg-x) );

	//G << (x_tg-x).toString() << endl;
}

Vector pmpGenericBlock::calculate_F(const Vector &delta, const Matrix &K)
{
	Vector F = align_K(delta,K)*delta;
	return F;
}

Vector pmpGenericBlock::calculate_M(const Vector &XYZ, const Matrix &K)
{
	Vector M(3,0.0);
	return M;
}

// in questa formulazione TGpose è la posa finale. Calcolo ogni step di posa in questa funzione
Vector pmpGenericBlock::calculate_M(const Matrix &TGpose, const Matrix &K, const int &iter)
{
	Vector M(3,0.0);

	Matrix EEpose(3,3);
	Matrix EE2TGpose(3,3);
	Matrix dPose(3,3);

// CURRENT ITERATION:
	// compute the rotation matrix between current EE pose and TG pose
	// then convert it into axis-angle notation [ax,ay,az,theta] in the EE reference frame
	EEpose = chain->getH().submatrix(0,2,0,2);
	EE2TGpose = EEpose.transposed() * TGpose;
	Vector axis = dcm2axis(EE2TGpose);
	Vector omega = axis(3)*axis.subVector(0,2);

	// get the virtual momentum along axis considering Dtheta as elastic drive
	// the momentum will be parallel to the axis.
	//M = align_K(axis.subVector(0,2),K)*EEpose*omega;
	M = K(0,0)*EEpose*omega;
	
	//cout << "dtheta: " << axis(3)*CTRL_RAD2DEG << endl;
	return M;

		/*
// COMPUTE TRACKING ERROR:
	// get the next differential angular step at current iteration (iter) using internal tbg
	// and compute the infinitesimal rotation matrix relative to EE frame of reference:
	// dR = (I-S(axis[0:2])*Dtheta_old

	diffPose.eye();
	diffPose(0,1) = axis(2);
	diffPose(1,0) = -axis(2);
	diffPose(0,2) = -axis(1);
	diffPose(2,0) = axis(1);
	diffPose(2,1) = axis(0);
	diffPose(1,2) = -axis(0);

	// get the final ee incremental pose in root reference frame:
	// EEpose(t+1) = dR(t) * EEpose(t);
	*/
}
Vector pmpGenericBlock::calculate_omega  (const Matrix &TGpose)
{
	Matrix EEpose(3,3);
	Matrix EE2TGpose(3,3);
	Matrix dPose(3,3);

// CURRENT ITERATION:
	// compute the rotation matrix between current EE pose and TG pose
	// then convert it into axis-angle notation [ax,ay,az,theta] in the EE reference frame
	EEpose = chain->getH().submatrix(0,2,0,2);
	EE2TGpose = EEpose.transposed() * TGpose;
	Vector axis = dcm2axis(EE2TGpose);
	Vector omega = axis(3)*axis.subVector(0,2);

//	cout << "dtheta: " << axis(3)*CTRL_RAD2DEG << endl;

	// return velocity wrt global axes
	return EEpose*omega;
}

Vector pmpGenericBlock::calculate_M(const Vector &omega, const Matrix &K, const int &iter)
{
	Vector M(3,0.0);
	M = K(0,0)*omega;
	
	return M;
}

Vector pmpGenericBlock::calculate_Tau_int()
{
	// non-linear attractive force field toward the mid-range of the joint
	/*double min = (*chain)(0).getMin();
	double max = (*chain)(0).getMax();
	double qref = min + 0.5*(abs(max) + abs(min));*/
	
	Vector Tau_int (chain->getDOF(),0.0);
	
	Vector qdelta = (q_ref-q);
	double max = 1.0/(exp(1.5)-1.0);
	
	
	for (unsigned int i=0; i<qdelta.size(); i++)
	{
		//cout << "i " << i << "perc  " << qdelta(i)/halfJointRange(i) << endl;
		Tau_int(i) =  (q_ref(i) >= q(i)) ?
				 Kint(i,i)*( exp( 1*pow(qdelta(i)/halfJointRange(i),2) + 0.5*pow(qdelta(i)/halfJointRange(i),4) ) - 1)*max:
				-1*Kint(i,i)*( exp( 1*pow(qdelta(i)/halfJointRange(i),4) + 0.5*pow(qdelta(i)/halfJointRange(i),4) ) - 1)*max;
	}

	//Kint(i,i)*( exp( 1*pow(qdelta(i)/halfJointRange(i),2) + 0.5*pow(qdelta(i)/halfJointRange(i),4) ) - 1)*max:

/*	cout << "qref - q: " << endl;
	cout << qdelta.toString() << endl;
	cout << "Tau int: " << endl;
	cout << Tau_int.toString() << endl;
*/		
	//Tau_int(i) = Kint(i,i)*( 0.25*pow(qdelta(i),4) + pow(qdelta(i),6) ) - Kint(i,i);
	
	// Linear Attractive forces to q_ref joint angles values
	//Tau_int = Kint*(q_ref - q);
	//Tau_int = Kint*(q_ref - q);

	return Tau_int;	
}

Vector pmpGenericBlock::calculate_Tau(const Vector &F)
 {	
	 Vector Tau_ext(chain->getDOF(),0.0);

	 double sum = 0;
	 for (unsigned int i=0; i<F.size(); i++) sum+=F(i);
	 //cout << sum << " ";
	 if (sum==0) return Tau_ext;
	
	 if (F.size() == 3)	Tau_ext = (get_Jacobian(true).transposed())*F;
	 else				Tau_ext = (get_Jacobian().transposed())*F;

	 return (Tau_ext + calculate_Tau_int());
 }

/*void pmpGenericBlock::calculate_qdot(const Vector &F)
 {	
	 Vector Tau_ext(chain->getDOF(),0.0);
	
	 if (F.size() == 3)	Tau_ext = (get_Jacobian(true).transposed())*F;
	 else				Tau_ext = (get_Jacobian().transposed())*F;

	 Tau_int = calculate_Tau_int();
	 qdot	 = tbg.calculateGamma(time) * A * (Tau_ext + Tau_int);	
 };
*/

Matrix pmpGenericBlock::get_Jacobian(bool pos)
{
	// HP che i valori di angoli siano già stati settati nelle varie chain con chain->setAng(q)
	
	int DOF = chain->getDOF();
	Matrix J = chain->GeoJacobian();

	if(pos) return J.submatrix(0,2,0,DOF-1);
	else	return J;
	//return J.submatrix(0,2,0,DOF-1); // positional Jacobian
}

Matrix pmpGenericBlock::get_RotJacobian()
{
	// HP che i valori di angoli siano già stati settati nelle varie chain con chain->setAng(q)
	
	int DOF = chain->getDOF();
	Matrix J = chain->GeoJacobian();

	return J.submatrix(3,5,0,DOF-1);
	//return J.submatrix(0,2,0,DOF-1); // positional Jacobian
}

bool pmpGenericBlock::run(const Vector &par, const Vector &x_tg)
{
	//Runner initialization: x_tg is set from outside
	Vector x(3);	
	x = x_0;	
	//x_tg = x_target;

	time = (unsigned int)par(0);
	// update joint pose trajectory
	

	if (x == x_tg) return true;
 
	// get joint velocity compatible with the external force field:
		Matrix J = get_Jacobian(true);
		Matrix T = J*A*J.transposed();

		//F = calculate_F(distort_delta(x_tg-x, x_tg-tg_old, T, tbg.calculateGamma(time)),K);
		F.setSubvector( 0, calculate_F(comp.distort_delta(x_tg-x, x_tg-tg_old, x-x_old, T, tbg.calculateGamma(time)),K) );
		F.setSubvector( 3, calculate_M(rpy_tg, Km) );

		calculate_qdot(F,time);
		q += qdot * tbg.getSlopeRamp();
	
	// update robot joint angles
		chain->setAng(q);
		q = chain->getAng();

	// update initial EE position with current EE position
		//x_0 = get_EEposition();
		tg_old = x_tg;
		x_old = x;

		//pos << x_0.toString() << endl;
		//tg << x_tg.toString() << endl;
		//Force << F.toString() << endl;
		//joints << q.toString() << endl;

/*
	Vector temp = Schain->getAng();
	for (unsigned int i=0; i< Schain->getN(); i++)
	{
		if ( temp(i)!=q_rightArm(i) )	{q(i) = rtemp(i);}//cout << "q cambia" << endl;}
		//else							cout << "q non cambia" << endl;		
	}
*/

	return true;

}


// -------------------------------------------------------------------------------------------------------------------------------

/* Compensator definitions */

Compensator::Compensator()
{
	eigVec.resize(3,3); 
	eigVal.resize(3,0.0);
	Ki = 0.1;//40; //10; //40
	Ti = 0.001;//0.01; //0.0001
	Kr = 20;
	Tr = 0.05;

	//G.open("gamma.txt");
	//proj.open("v.txt");
	//eigV.open("ev.txt");
	//eigL.open("el.txt");
}

Compensator::Compensator (const Compensator &c): eigVec(c.eigVec),
												 eigVal(c.eigVal)
{
	this->Ki = c.Ki;
	this->Ti = c.Ti;
	this->Kr = c.Kr;
	this->Tr = c.Tr;

	//G.open("gamma.txt");
	//proj.open("v.txt");
	//eigV.open("ev.txt");
	//eigL.open("el.txt");
}

Compensator & Compensator::operator=(const Compensator &c)
{
	if (this != &c)
	{
		this->Ki = c.Ki;
		this->Ti = c.Ti;
		this->Kr = c.Kr;
		this->Tr = c.Tr;
		this->eigVec = c.eigVec;
		this->eigVal = c.eigVal;
	}
	return *this;
}

Compensator::~Compensator()
{
	//G.close();
	//proj.close();
	//eigV.close();
	//eigL.close();
}

void Compensator::init(const double &_L, const double &_DeltaT)
{
	this->L = _L;
	this->dt = _DeltaT;
	delta_old = 0.0;
	omega_old.resize(3,0.0);
	dv_old.resize(3,0.0);
	g_turn = 0.0;
	grate_old = 0.0;
	g_old = 0;
	dvel_old = 0.0;
	err_old = 0.0;
	eigVec.zero();
	eigVal.zero();
	G.open("Ff.txt");
}

void Compensator::get_Eigens(const yarp::sig::Matrix &system, yarp::sig::Matrix &eigVec, yarp::sig::Vector &eigVal)
{
	gsl_matrix * H = (gsl_matrix *) (system.getGslMatrix());
	 
	gsl_vector_complex *evalC = gsl_vector_complex_alloc (system.rows());
    gsl_matrix_complex *evecC = gsl_matrix_complex_alloc (system.rows(), system.rows());
     
    gsl_eigen_nonsymmv_workspace * w = gsl_eigen_nonsymmv_alloc (3);
    gsl_eigen_nonsymmv (H, evalC, evecC, w);
    gsl_eigen_nonsymmv_free (w);
     
	int r =  gsl_eigen_nonsymmv_sort (evalC, evecC, GSL_EIGEN_SORT_ABS_DESC); 

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
	   //eigV << eigVec.toString() << endl;
	   //eigL << eigVal.toString() << endl;

	return;	
}

Vector Compensator::distort_delta(const yarp::sig::Vector &delta, const yarp::sig::Vector &delta_id, const yarp::sig::Vector &delta_re,
								  const yarp::sig::Matrix &H, const double &Gamma)
{	
	get_Eigens(H, eigVec, eigVal);

	if (delta_id(0) == 0 && delta_id(1) == 0 && delta_id(2) == 0) return delta;
	if (delta(0)	== 0 && delta(1)	== 0 && delta(2)	== 0) return delta;

	// check that the tracking error is less than one mm.
	// if the error is too big, then zero the virtual force field.
	Vector dx_m(3,0.0);	
	double n_delta = norm(delta);

	if(fabs(delta_old-n_delta/L) > .001)
	{
		//cout << delta_old-n_delta/L << endl;
		return dx_m;
	}
	delta_old = n_delta/L;
	
	// compute projection of delta along the eigenvectors:
	// pick out the one for whom the projection is bigger
		int dx_max = 0;
		for (unsigned int i=1; i<3; i++)
		{
			dx_m(i) = fabs(dot(delta/n_delta,eigVec.getCol(i)));
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
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double g = 1/(Ti+dt)*(Ti*g_old + 1/(Gamma+0.0001)*dt*Ki);

		if (g_old!= 0.0 && g_turn >= Gamma)
			g = 1/(Ti+dt)*(Ti*g_old + 1/(2*g_turn-Gamma + 0.0001)*dt*Ki);
		else
			g_turn = Gamma;

		//proj << Gamma/g << endl;
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
		Vector dv(3);
		dv(0)= 1/(Tr+dt)*(dv_old(0)*Tr + (delta(0)+v_m*v(0))*dt*Kr);
		dv(1)= 1/(Tr+dt)*(dv_old(1)*Tr + (delta(1)+v_m*v(1))*dt*Kr);
		dv(2)= 1/(Tr+dt)*(dv_old(2)*Tr + (delta(2)+v_m*v(2))*dt*Kr);
		dv_old = dv;

	// compute the new distorted force direction
		//return delta + g*dv*v;

		//G << delta.toString() << "\t" << dv.toString() << endl;

		//return g*(delta + dv);
		return g*dv;
}

Vector Compensator::distort_delta(const yarp::sig::Vector &delta, const yarp::sig::Vector &delta_id, const yarp::sig::Vector &delta_re, 
								  const yarp::sig::Matrix &H, const double &Gamma, double &gain)
{	
	if (Gamma == 0)
	{
		gain = 0.0;
		return delta;
	}

	// gain control law:
	// nonlinear gain profile to compensate low initial force intensity: adapt the stiffness
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double g = 1/(Ti+dt)*(Ti*g_old + 1/(Gamma+0.00001)*dt*Ki);

		if (g_old!= 0.0 && g_turn >= Gamma)
			g = 1/(Ti+dt)*(Ti*g_old + 1/(2*g_turn-Gamma + 0.00001)*dt*Ki);
		else
			g_turn = Gamma;

		//proj << Gamma/g << endl;
		G << g << " " << Gamma << endl;
		gain = g_old = g;
		gain *=100;

	if ( delta_id(0) == 0 && delta_id(1) == 0 && delta_id(2) == 0 && L!=0)   return delta;
	if ( delta(0)	 == 0 && delta(1)	 == 0 && delta(2)	 == 0)			 return delta;
/*	if ( delta_id(0) == 0 && delta_id(1) == 0 && delta_id(2) == 0 && L!=0)   return delta;
	if ( delta(0)	 == 0 && delta(1)	 == 0 && delta(2)	 == 0)
	{
		Vector eps(3,1e-6);
		if (L == 0)
		{
			delta = eps;
			return g*eps;
		}
		else return delta;
	}
*/	
	Vector dx_m(3,0.0);
	double n_delta = norm(delta);

	// check if there is too much discrepancy btw target velocity and robot's one:
	//if (delta_re(0) != 0 && delta_re(1) != 0 && delta_re(2) != 0)
	{
		double ierr = 1/(3*dt)*(2*dt*err_old + fabs(delta_old-n_delta/L)*dt);
		err_old = ierr;
		
		//cout << ierr << endl;//" ";

		if( ierr > .05)
		{
			gain = 0.0;
			err_old = 10;
			Vector Zero(3,0.0);
			return Zero;
		}
		//cout << norm(delta_re)/norm(delta_id) <<  " " << fabs(delta_old-n_delta/L)<<endl;
	}

#if 0	
	// check that the tracking error is less than one mm.
	// if the error is too big, then zero the virtual force field.
	if(fabs(delta_old-n_delta/L) > .001)
	{
		//cout << delta_old-n_delta/L << endl;
		return dx_m;
	}
#endif
	
	// compute projection of delta along the eigenvectors:
	// pick out the one for whom the projection is bigger
		get_Eigens(H, eigVec, eigVal);

		int dx_max = 0;
		for (unsigned int i=1; i<3; i++)
		{
			dx_m(i) = fabs(dot(delta/n_delta,eigVec.getCol(i)));
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
#if 0
	// gain control law:
	// nonlinear gain profile to compensate low initial force intensity: adapt the stiffness
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double g = 1/(Ti+dt)*(Ti*g_old + 1/(Gamma+0.0001)*dt*Ki);

		if (g_old!= 0.0 && g_turn >= Gamma)
			g = 1/(Ti+dt)*(Ti*g_old + 1/(2*g_turn-Gamma + 0.0001)*dt*Ki);
		else
			g_turn = Gamma;

		//proj << Gamma/g << endl;
		g_old = g;
#endif
	// angle control law:
	// enlarge the projection of delta along the eigenvector (v):
	// compute filtered angle according to the filter: Kr/(1+sT)
	// using Kr = 1; T = 0.1; period = 1 sec, step = 0.001 sec
	// using the finite difference equivalent for the system: y(-1) = 0
	// x(k) = T/(T+step)*(x(k-1) - step*u(k))
	// y(k) = -Kr/T*x(k);
	// x(k-1) = y(k-1)*T/Kr;
	// controlling the single components of v:
		Vector dv(3);
		dv(0)= 1/(Tr+dt)*(dv_old(0)*Tr + (delta(0)+v_m*v(0))*dt*Kr);
		dv(1)= 1/(Tr+dt)*(dv_old(1)*Tr + (delta(1)+v_m*v(1))*dt*Kr);
		dv(2)= 1/(Tr+dt)*(dv_old(2)*Tr + (delta(2)+v_m*v(2))*dt*Kr);
		dv_old = dv;

	// compute the new distorted force direction
		//return delta + g*dv*v;

		//G << delta.toString() << "\t" << dv.toString() << endl;
		//cout << delta.toString() << "\t" << dv.toString() << endl;
		
		//return g*(delta + dv);
		//return g*dv;
		return dv;
}

// only gain compensation here:
Vector Compensator::distort_delta(const yarp::sig::Vector &delta, const double &Gamma)
{	
	// gain control law:
	// nonlinear gain profile to compensate low initial force intensity: adapt the stiffness
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double g = 1/(Ti+dt)*(Ti*g_old + 1/(Gamma+0.0001)*dt*Ki);

		if (g_old!= 0.0 && g_turn >= Gamma)
			g = 1/(Ti+dt)*(Ti*g_old + 1/(2*g_turn-Gamma + 0.0001)*dt*Ki);
		else
			g_turn = Gamma;

		//proj << Gamma/g << endl;
		g_old = g;

		return g*(delta);
}

Vector Compensator::distort_delta(const yarp::sig::Vector &delta, const double &Gamma, double &gain)
{	
	// gain control law:
	// nonlinear gain profile to compensate low initial force intensity: adapt the stiffness
	// check that g is a decreasing function: this may not happen in the very last iterations
	// due to Gamma finite approximation
		double g = 1/(Ti+dt)*(Ti*g_old + 1/(Gamma+0.0001)*dt*Ki);

		if (g_old!= 0.0 && g_turn >= Gamma)
			g = 1/(Ti+dt)*(Ti*g_old + 1/(2*g_turn-Gamma + 0.0001)*dt*Ki);
		else
			g_turn = Gamma;

		//proj << Gamma/g << endl;
		gain = g_old = g;

		return g*(delta);
}

Vector Compensator::distort_axis(const yarp::sig::Vector &omega,
								 const yarp::sig::Matrix &H)
{	
	get_Eigens(H, eigVec, eigVal);

	if (omega(0)==0 && omega(1)==0 && omega(2)==0) return omega;

	// check that the tracking error is less than one mm.
	// if the error is too big, then zero the virtual force field.
	Vector dth_m(3,0.0);
	
	// compute projection of omega along the eigenvectors:
	// pick out the one for whom the projection is bigger
		int dth_max = 0;
		for (unsigned int i=1; i<3; i++)
		{
			dth_m(i) = fabs(dot(omega,eigVec.getCol(i)));
			if (dth_m(i) > dth_m(dth_max))	dth_max = i;
		}

	// for the selected one, choose the right sense
		Vector v(3);
		double v_m;

		if ( norm( omega + dot(omega,eigVec.getCol(dth_max))*eigVec.getCol(dth_max) )	  > 
			 norm( omega - dot(omega,eigVec.getCol(dth_max))*eigVec.getCol(dth_max) )
			)	
			v = eigVec.getCol(dth_max);
		else
			v = -1*eigVec.getCol(dth_max);		
		
		v_m = dot(omega, v);

	// angle control law:
	// enlarge the projection of omega along the eigenvector (v):
	// compute filtered velocity according to the filter: Kr/(1+sT)
	// using Kr = 1; T = 0.1; period = 1 sec, step = 0.001 sec
	// using the finite difference equivalent for the system: y(-1) = 0
	// x(k) = T/(T+step)*(x(k-1) - step*u(k))
	// y(k) = -Kr/T*x(k);
	// x(k-1) = y(k-1)*T/Kr;
	// controlling the single components of v:
		Vector dv(3);
		dv(0)= 1/(Tr+dt)*(omega_old(0)*Tr + (omega(0)+v_m*v(0))*dt*Kr);
		dv(1)= 1/(Tr+dt)*(omega_old(1)*Tr + (omega(1)+v_m*v(1))*dt*Kr);
		dv(2)= 1/(Tr+dt)*(omega_old(2)*Tr + (omega(2)+v_m*v(2))*dt*Kr);
		double n_dv = norm(dv);
		omega_old = dv/n_dv;

	// compute the new distorted force direction
		//return delta + g*dv*v;

		G << omega.toString() << "\t" << dv.toString() << endl;
		
		//return g*(delta + dv);
		return dv;
}
