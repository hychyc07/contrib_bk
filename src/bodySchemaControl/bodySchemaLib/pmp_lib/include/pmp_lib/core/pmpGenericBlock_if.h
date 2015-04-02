#ifndef PMP_GENERIC_IF_H
#define PMP_GENERIC_IF_H

#include <iCub/iKin/iKinFwd.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <pmp_lib/core/pmpGenericBlock.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace iCub::iKin;


namespace iCub{
namespace pmplib{
namespace core{

	typedef enum {q, q_ref, A, K, K_int, ukn} pmp_param;
	static const string pmp_param_list[] = {"q", "q_ref", "A", "K", "K_int"};
	static const unsigned int NUM_PMP_PARAM = 5;
	static pmp_param string2pmp_param(const string & s)
	{
		for(int i=0; i<NUM_PMP_PARAM; i++)
			if (s == pmp_param_list[i])	 return (pmp_param)i;

		return ukn;
	};
	static string	  pmp_param2string(const pmp_param & p)
	{
		return pmp_param_list[(int)p];

	};

	class iCubFinger : public iKinLimb
	{
	protected:
		virtual void allocate(const std::string &_type);
	public:
		iCubFinger();
		iCubFinger(const std::string &_type);
		iCubFinger(const iCubFinger &finger);
	};

	class iCubTorso : public iKinLimb
	{
	protected:
		virtual void allocate(const std::string &_type);

	public:
		/**
		* Default constructor. 
		*/
		iCubTorso();

		/**
		* Constructor. 
		* @param _type is a string to discriminate between "left" and 
		*              "right" arm connected torso
		*/
		iCubTorso(const std::string &_type);

		/**
		* Creates a new Arm from an already existing Arm object.
		* @param arm is the Arm to be copied.
		*/
		iCubTorso(const iCubTorso &torso);

		/**
		* Alignes the Torso joints bounds with current values set aboard 
		* the iCub. 
		* @param lim is the ordered list of control interfaces that 
		*            allows to access the Torso limits.
		* @return true/false on success/failure. 
		*/
		//virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
	};



	class iCubArmNoTorso : public iKinLimb
	{
	protected:
		virtual void allocate(const std::string &_type);

	public:
		/**
		* Default constructor. 
		*/
		iCubArmNoTorso();

		/**
		* Constructor. 
		* @param _type is a string to discriminate between "left" and 
		*              "right" arm
		*/
		iCubArmNoTorso(const std::string &_type);

		/**
		* Creates a new Arm from an already existing Arm object.
		* @param arm is the Arm to be copied.
		*/
		iCubArmNoTorso(const iCubArmNoTorso &arm);

		/**
		* Alignes the Arm joints bounds with current values set aboard 
		* the iCub. 
		* @param lim is the ordered list of control interfaces that 
		*            allows to access the Torso and the Arm limits.
		* @return true/false on success/failure. 
		*/
		//virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
	};
	class pmpBlock : public pmpGenericBlock
	{
		public:

			//class variables:
				yarp::sig::Vector x_tg;
				string blockType;
				

			// class methods
				/// \brief class constructor
				/** This constructor creates a pmp block with a kinematic chain specified by block_type parameter
				/* \param block_type defines the kinematic chain by a name string
				/*  All known block types will be stored in a factory file.
				*/
				pmpBlock(const string &block_type);

				/// \brief class copy constructor
				/** This constructor creates a pmp block with a kinematic chain specified by block parameter
				/* \param block is an existing pmpBlock instance.
				*/
				pmpBlock(const pmpBlock &block);

				/// \brief class constructor
				/** This constructor creates an empty block with a fake kinematic chain with 0 DOF
				/*  To allocate a new chain call allocate() member function, specifyng the chain type
				*/
				pmpBlock();

				/// \brief class destructor				
				~pmpBlock();

				/// \brief copy operator 
				/** This operator initializes the block from an existing one
				*/
				pmpBlock& operator=(const pmpBlock &block);

				/** \brief This method allocates all kinematic chain dependent variables
				/* @param block_type defines a kinematic chain type defined in chain factory file
				*/
				void allocate(const string &block_type);

				/** \brief This method initializes all pmp variables and internal tbg using a yarp::os::Property object
				/* \param opt_chain specifies all the core pmp parameters
				/* \param opt_tbg spacifies all the block internal tbg object parameters
				*/
				bool initialize(Property *opt_chain, Property *opt_tbg);

				/** \brief This method initializes all pmp variables using a yarp::os::Property object
				/* \param _opt_chain specifies all the core pmp parameters
				*/
				bool initializeBlock(Property *_opt_chain);
				bool initializeBlock(Bottle *_opt_chain);

				/** \brief This method initializes all internal tbg variables using a yarp::os::Property object
				/* \param _opt_tbg specifies all the core internal tbg parameters
				*/
				bool initializeTbg(Property *_opt_tbg);
				bool initializeTbg(Bottle *_opt_tbg);

				/** \brief Main method of the block
				/* Every time run is called, for a given target point, one iteration of pmp is computed.
				/* \param par is a Vector containing parameters to be passed to the method, like the iteration number
				/* and the trajectory lenght.
				/* \param x_target is the Vector containing the target 3D point in extrinsic coordinates
				*/
				bool run(const Vector &par, const Vector &x_target);
		
			// get methods
				yarp::sig::Vector get_q();
				yarp::sig::Vector get_q_ref();
				yarp::sig::Matrix &get_A();
				yarp::sig::Matrix &get_K();
				yarp::sig::Matrix &get_Km();
				yarp::sig::Matrix get_Kint();
				yarp::sig::Vector get_EEposition();
				yarp::sig::Vector get_EEpose_asRPY();
				yarp::sig::Matrix get_EEpose_asRot();

				/// \brief This method gives the mumber of block DOF
				/// \return the number of active degrees of freedom in the block kinematic chain
				double get_DOF();

				/// \brief This method gives the mumber of block potential DOF
				/// \return the number of all potential degrees of freedom in the block kinematic chain
				double get_N();

				/// \brief This method gives the block kinematic chain
				/// \return the block kinematic chain
				iKinChain get_chain();
				iKinChain *get_defaultChain();

				/// \brief This method assigns the block a new kinematic chain (TODO)
				//void set_chain(const iKinChain & _c);
				void set_chain(iKinChain * _c);
	
			// set methods
				void set (const string & param, const Vector & v);
				void set (const string & param, const Matrix & m);
				yarp::sig::Vector getVector (const string & param);
				yarp::sig::Matrix getMatrix (const string & param);

				void set_q     (const yarp::sig::Vector &_q);
				void set_A     (const yarp::sig::Matrix &_A);
				void set_A     (const yarp::sig::Vector &_A);
				void set_K     (const yarp::sig::Matrix &_K);
				void set_K     (const yarp::sig::Vector &_K);
				void set_Km    (const yarp::sig::Matrix &_K);
				void set_Km    (const yarp::sig::Vector &_K);
				void set_Kint  (const yarp::sig::Matrix &_Kint);
				void set_Kint  (const yarp::sig::Vector &_Kint);
				void set_q_ref (const yarp::sig::Vector &_q_ref);
			
		//private: OK
		public: // for testing
			// class variables
				iKinLimb * Limb;
				void allocateChain();
			
			// orientation
				static const double PALM_LITTLE [];
				static const double PALM_INDEX [];
				Matrix palmPoints;
				bool useOrientation;

			// class methods				
				yarp::sig::Vector get_EulAng (const double &Rotz,const double &Rotx);
				yarp::sig::Vector get_RPYAng (const double &Rotz,const double &Rotx);
				yarp::sig::Vector get_RPYAng (const Matrix &R);
				void get_nextRPY			 (unsigned int time);
				static yarp::sig::Matrix RPY2Rot	 (const Vector &RPY_deg);
				static yarp::sig::Vector Rot2RPY	 (const Matrix &R);
				bool setEEorientation		 (const string &_side, const Vector & RPY);

				bool setAngFromProperty(Property *options, string key, Vector &v);
				bool setFromProperty(Property *options, string key, Vector &v);
				bool setFromProperty(Property *options, string key, Matrix &m);
				bool setAngFromProperty(Bottle *options, string key, Vector &v);
				bool setFromProperty(Bottle *options, string key, Vector &v);
				bool setFromProperty(Bottle *options, string key, Matrix &m);
				bool setEyeMatrixFromVector(Vector v, Matrix &m);
	};

}
}
}
#endif

