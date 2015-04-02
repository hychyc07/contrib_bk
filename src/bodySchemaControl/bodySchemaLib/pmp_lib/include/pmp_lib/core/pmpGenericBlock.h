/** \file pmp_core.h
/*	\brief PMP model core classes.
*/

#ifndef PMP_GENERICBLOCK_H
#define PMP_GENERICBLOCK_H

#include <iCub/iKin/iKinFwd.h>
#include <pmp_lib/core/tbg.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace iCub::iKin;


namespace iCub{
namespace pmplib{
namespace core{

/**
* @class Compensator implements the intrinsic kinematic distortion compensation algorithm
* along a trajectory.
*/
	class Compensator
	{
		public:
			Compensator();
			~Compensator();
			Compensator (const Compensator &c);
			Compensator &operator=(const Compensator &c);
			
			void init(const double & _L, const double & _DeltaT);

			yarp::sig::Vector distort_delta	(const Vector &delta, const Vector &delta_id,  const Vector &delta_re, const Matrix &H, const double &Gamma);
			yarp::sig::Vector distort_delta	(const Vector &delta, const Vector &delta_id,  const Vector &delta_re, const Matrix &H, const double &Gamma, double &gain);
			yarp::sig::Vector distort_delta (const yarp::sig::Vector &delta, const double &Gamma);
			yarp::sig::Vector distort_delta (const yarp::sig::Vector &delta, const double &Gamma, double &gain);

			yarp::sig::Vector distort_axis	(const yarp::sig::Vector &omega, const Matrix &H);

		//private:
			double L;
			double Theta;
			double dt;
			double delta_old;
			Vector dv_old;
			Vector omega_old;
			double g_old;
			double g_turn;
			double grate_old;
			double dvel_old;
			double err_old;
			double Ki,Kr;
			double Ti,Tr;
			Matrix eigVec;
			Vector eigVal;

			void get_Eigens(const Matrix &system, Matrix & eigVec, Vector & eigVal);

			ofstream proj, G, eigV, eigL;
	};


/**
* @class pmpGenericBlock defines a single block for a modular PMP-based network
* block can be: right arm, left arm, torso, legs...
*/
	class pmpGenericBlock
	{
		public:

			//class variables:
				//yarp::sig::Vector x;
				yarp::sig::Vector q_0;
				yarp::sig::Vector q_home;
				TimeBaseGenerator tbg;    // privata o pubblica? idem per sopra!

				/// \brief class copy constructor
				/** This constructor creates a pmp block with a kinematic chain specified by block parameter
				/* \param block is an existing pmpGenericBlock instance.
				*/
				pmpGenericBlock(const pmpGenericBlock &block);

				/// \brief class constructor
				/** This constructor creates an empty block with a fake kinematic chain with 0 DOF
				/*  To allocate a new chain the abstract method allocate() has to be overwritten
				*/
				pmpGenericBlock();

				/// \brief class destructor				
				~pmpGenericBlock(){};

				///// \brief copy operator
				/** This operator initializes block parameters from an existing one
				/* Block kinematics and parameters which size is dependent on DOF has to be initialized
				/* separately by the user calling allocateChain() and allocateVariableSizeParams()
				*/
				pmpGenericBlock& operator=(const pmpGenericBlock &block);

				/** \brief This method allocates the kinematic chain (abstract)
				/* 
				*/
				virtual void allocateChain() = 0;

				/** \brief This method allocates all kinematic chain size dependent variables
				/* 
				*/
				void allocateVariableSizeParams(int _DOF);

				/** \brief This method initializes all pmp variables using a yarp::os::Property object
				/* \param _opt_chain specifies all the core pmp parameters
				*/
				bool initializeBlock(Property *_opt_chain);

				/** \brief This method initializes all internal tbg variables using a yarp::os::Property object
				/* \param _opt_tbg specifies all the core internal tbg parameters
				*/
				bool initializeTbg(Property *_opt_tbg);

				/** \brief Main method of the block
				/* Every time run is called, for a given target point, one iteration of pmp is computed.
				/* \param par is a Vector containing parameters to be passed to the method, like the iteration number
				/* and the trajectory lenght.
				/* \param x_target is the Vector containing the target 3D point in extrinsic coordinates
				*/
				bool run(const Vector &par, const Vector &x_target);
			
		private:
				yarp::sig::Vector halfJointRange;

		public:
		//protected:
				
				iKinChain  * chain;
				int N;
				yarp::sig::Vector q;
				yarp::sig::Vector q_ref;
				yarp::sig::Vector qdot;
				yarp::sig::Vector refPose;

				yarp::sig::Matrix K;
				yarp::sig::Matrix Km;
				yarp::sig::Matrix Kint;

				yarp::sig::Matrix A; // Admittance matrices
				yarp::sig::Vector F;

				yarp::sig::Vector x_0;
				yarp::sig::Vector x;
			
				int ref_active;

				unsigned int time; //iteration number
				int scale; // conversion factor mm -> m

				Property *opt_chain, *opt_tbg;

			// compensation
				Vector tg_old;
				Vector x_old,dv_old;
				Vector rpy_old;
				Vector rpy_tg;
				Compensator comp;
				double dtheta_old;

			// class methods				
				yarp::sig::Matrix align_K		   (const Vector & deltax, const Matrix &K);
				yarp::sig::Vector calculate_F      (const Vector &delta, const Matrix &K);
				yarp::sig::Vector calculate_F	   (const Vector &x_tg, const Vector &x, const Matrix &K);
				yarp::sig::Vector calculate_M      (const Matrix &Rref, const Matrix &K, const int &time);
				yarp::sig::Vector calculate_M	   (const Vector &omega, const Matrix &K, const int &iter);
				yarp::sig::Vector calculate_M      (const Vector &XYZ, const Matrix &K);
				yarp::sig::Vector calculate_omega  (const Matrix &Rref);
				inline Vector calculate_qdot(const Vector &scaledTau, const int &time)
				{	
				  // scaled tau = A*tau;
				  return tbg.calculateGamma(time) * scaledTau;
	 
				  //non scaled tau
				  //return tbg.calculateGamma(time) * A * Tau;
				};
				yarp::sig::Vector calculate_Tau	   (const Vector &F);
				yarp::sig::Vector calculate_Tau_int();
				yarp::sig::Matrix get_Jacobian	   (bool positional = false);
				yarp::sig::Matrix get_RotJacobian	();

	};
}
}
}

#endif
