#ifndef PMP_RUNNER_H
#define PMP_RUNNER_H
//#define BOOST_THREAD_USE_LIB

//#include <boost/thread/thread.hpp>
#include <pmp_lib/groupThread.h>

#include "pmp_network.h"
#include <pmp_lib/core/pmpGenericBlock.h>
#include <pmp_lib/core/VTGS.h>

using namespace lemon;
using namespace lemon::concepts;
using namespace iCub::iKin;
using namespace iCub::pmplib;
using namespace iCub::pmplib::core;
using namespace iCub::pmp_network;
//using namespace boost;

namespace iCub{
namespace pmp_runner {

	struct fileWriter
	{
		string name;
		ofstream x_file;
		ofstream q_file;
		ofstream tg_file;
		ofstream F_file;
		ofstream T_file;
		ofstream segment_file;
		ofstream pose_file;
		ofstream eigs_file;
		ofstream link_file;

		void open(string _name);
		void close();
		void write(const Vector &x, const Vector &q, const Vector &tg);
		void writeF(const Vector &F, const Vector &T);		
		void writeSeg(const Vector &p1, const Vector &p2);
		void writeSeg(const Vector &p1, const Vector &p2,const Vector &p3, const Vector &p4);
		void writePose(const Vector &p, const Vector &mp);
		void writeEigs(const Matrix &eV);
		void writeLink(const Matrix &pi);
	};

	struct DependencyTraits
	{
		int cont;
		vector<int> pathId;
		DependencyTraits(){cont=0;};
	};

	struct JointTraits
	{
		Vector data;
		vector<int> NodeId;
		void push( int n_id, const Vector &vel);
		void push( int n_id, const int &length);
		Vector subvector(int n_id);
		void setSubvector(int n_id, const Vector & v);
	};


	// classe che contiene i valori cinematici di un path
	struct pathChain : public iKinChain
	{
		vector<int> NodeId;
		
		pathChain(const iKinChain &ch):iKinChain(ch){};
		pathChain():iKinChain(){};
		pathChain(const pathChain &pc):iKinChain(pc){};
		
		using iKinChain::operator=;		
	
		void pushChain(iKinChain &ch, const int &nodeid, bool root = false);
		void pushChain(iKinChain *ch, const int &nodeid, bool root = false);

		Matrix getRelativeH(const int & nodeId);
		Matrix getRelativeH(const int & nodeId, const int &jointNo);

		void getRelativeChain(const int &nodeId,iKinChain & ch);
		void splitChain(const int &nodeId, iKinChain & chHead, iKinChain &chTail);

		Vector getRelativeAng(const int &nodeId);
		Bottle writeRelativeAng(const int &nodeId, bool DEG = true);

		//void pushChain(iKinChain &ch, bool root = false);
		//void pushChain(iKinChain *ch, bool root = false);
	};

	// classe che contiene i parametri di rilassamento di un path
	struct pmpChain
	{
		Vector A;
		Matrix K;
		Matrix Km;
		deque<Vector> Kint;
		deque<Vector> qref;

	public:	

		pmpChain(){};
		pmpChain(const pmpChain &pc);
		pmpChain &operator=(const pmpChain &pc);
		void push_A(Matrix & _A);
		void push_Kint(const Matrix & _Kint);
		void push_Kint(const Vector & _Kint);
		void push_qref(const Vector & _qref);
		void set_K(const Matrix &_K);
		void set_Km(const Matrix &_K);
	};

	// classe che definisce le caratteristiche di un nodo target
	// raccoglie tutti i parametri per far girare l'algoritmo su 1 path
	// class for running a pmp algorithm with only one target assigned to the network
	// the network is usually a subnetwork with nodes linked in sequence from root to target
	// when neither loops nor divergent/convergent branches are present
	template <typename DGR>
	class PathTraits : public pmpGenericBlock
	{
	public:
		template <class T> class propagatorThread;
		template <class T> class updaterThread;

		yarp::sig::Vector tg;
		yarp::sig::Matrix pose;
		//boost::thread * pathThread;
		propagatorThread< PathTraits<DGR> > * propagator;
		updaterThread< PathTraits<DGR> > * updater;

		typedef typename DGR::Digraph Digraph;
		//typedef typename DGR::template NodeMap<Vector> NodeVectorMap;
		typedef typename Digraph::template NodeMap<node_block> NodeBlockMap;
		typedef typename Digraph::template ArcMap<arc_block> ArcBlockMap;
		typedef typename DGR::Node Node;
		typedef typename DGR::Arc Arc;
		typedef typename DGR::OutArcIt OutArcIt;

	private:

		Node targetNode;
		NodeBlockMap * _nmap;
		ArcBlockMap * _amap;
		
		lemon::Path<DGR> path;
		pathChain	pChain;
		pmpChain	pmp;
		Compensator comp;
		vtgsBlock  vtgs;
		string _vtgsMode;
		Bottle _vtgs_opt;

		bool inactive;
		bool initialized;
		bool allocated;
		bool compensated;
		bool vtgsAssigned;
		bool poseAssigned;

		fileWriter fwriter;

	public:
		unsigned int pathId;
		vector<int> nodeIds;
		JointTraits qT;
		JointTraits torqueT;
		Vector Fv;
		//Vector Mv;
		Vector xtg;
		
		PathTraits();
		PathTraits(const PathTraits & pt);
		PathTraits &operator = (const PathTraits & pt);
		~PathTraits(){};
		

		// compute direct kinematics for the current target node (when the node is coincident with the root)
		void init(const DGR &g, NodeBlockMap & Nmap, ArcBlockMap & Amap, Node n, const int &_pathId);
		// compute direct kinematics for the current path to target
		// joint angles values are read from the node parameters, not from arcs.
		void init(const DGR &g, NodeBlockMap & Nmap, ArcBlockMap & Amap, lemon::Path<DGR> p, const int &_pathId);
		// create path chain
		void allocateChain();
		bool setTarget(const yarp::sig::Vector &v, bool compensate = true, const string &vtgsMode = "default");
		bool setTarget(const yarp::sig::Vector &v, const yarp::sig::Matrix &R, bool compensate = true, const string &vtgsMode = "default");
		bool setTarget(const yarp::sig::Matrix &R, bool compensate = true, const string &vtgsMode = "default");
		void assignVtgs(const Bottle &vtgs_opt);

		// get targetNode id:
		int getTargetNodeID();		
		lemon::Path<DGR> getPath();

		inline bool finished();
		inline bool isActive();
		void launchPropagation();
		void launchUpdate();
		void run(){};

		//friend class Network_runner;
	//protected:
		void updatePathState(const DGR &g, typename DGR::template NodeMap<DependencyTraits> &dmap);
		Bottle writePathState();
		void writeNodesStateOnPort(const DGR &g);

				
	private:
		// feedforward pmp transfer fcn: compute chain joint virtual torques
		inline void propagateRelaxation();
		// feedback pmp transfer fcn: compute compatible joint angles and end-effector position
		inline void updateState();
		void activateTargetNode( bool compensate, bool keepPosition = false );
		void writeLinksPosition();

	public:
		template <class T>
		class propagatorThread : public jointThread
		{
		private:
			bool *disabled;
			T *refClass;

		public:
			propagatorThread(T* _refClass) : jointThread(), disabled(0), refClass(_refClass){};
			void setDisabler(bool *disflag){disabled = disflag;};
			virtual inline bool runnable()
			{
				if(!*disabled) refClass->propagateRelaxation();
				return false;
			};
		};

		template <class T>
		class updaterThread : public jointThread
		{
		private:
			bool *disabled;
			T *refClass;

		public:
			updaterThread(T* _refClass) : jointThread(), disabled(0), refClass(_refClass){};
			void setDisabler(bool *disflag){disabled = disflag;};
			virtual inline bool runnable()
			{
				if(!*disabled) refClass->updateState();
				return false;
			};
		};
	};


	///Default traits class of Dfs class.
	///\tparam GR Digraph type.
	template <class TR>
	struct PmpRunnerTraits
	{
		typedef typename TR::Digraph Digraph; // così funziona anche per i sottografi
		typedef typename TR::template NodeMap<DependencyTraits> NodeDepMap;
		typedef typename Digraph::template NodeMap<node_block> NodeBlockMap;
		typedef typename Digraph::template ArcMap<arc_block> ArcBlockMap;
		typedef typename Digraph::template NodeMap<bool> NodeBoolMap;
		typedef typename Digraph::template ArcMap<bool> ArcBoolMap;

		static NodeBlockMap *createNodeBlockMap(const Digraph &g){return new NodeBlockMap(g);};
		static NodeDepMap *createNodeDepMap(const TR &g){return new NodeDepMap(g);};
		static ArcBlockMap *createArcBlockMap(const Digraph &g){return new ArcBlockMap(g);};
		static NodeBoolMap *createNodeBoolMap(const Digraph &g){return new NodeBoolMap(g);};
		static ArcBoolMap *createArcBoolMap(const Digraph &g){return new ArcBoolMap(g);};
	};


	// class for running pmp algorithm on a digraph.
	// it takes the digraph as constructor parameter and iterates using a multi-threaded
	// approach to compute the virtual force propagation and subsequent relaxation of the network
	// given a vector of node external targets.
	// DGR is the type of directed graph (e.g. <ListDigraph> of <SubDigraph<g,n,a>>)

	// si prende un sottografo della rete attiva in fase di inizializzazione +
	// mappa dei target associati ai nodi attivi map<node_id,double*> con TargetMap() +
	// mappa dei predecessori per ogni nodo dal visitor per calcolo dei path

	///Default traits class of pmpNetwork_runner class.
	template <typename DGR = ListDigraph,
			  typename TR = PmpRunnerTraits<DGR> >
	class Network_runner : public yarp::os::Thread
	{
	public:

		///The type of the digraph the algorithm runs on.
		typedef typename TR::Digraph Digraph;
		typedef TR Traits;

		typedef typename TR::NodeDepMap NodeDepMap;
		typedef typename TR::NodeBlockMap NodeBlockMap;
		typedef typename TR::ArcBlockMap ArcBlockMap;

		typedef typename Digraph::Node Node;
		typedef typename Digraph::Arc Arc;
		typedef typename DGR::NodeIt NodeIt;
		typedef typename DGR::ArcIt ArcIt;

		bool initialized;
		bool dependent;
		bool dynamicCreation;

	private:
		const DGR *_g;

		NodeBlockMap *_nBlocks;
		ArcBlockMap  *_aBlocks;

		// mappe che contengono per ogni nodo che appartiene a più target path
		// gli id dei path e il numero di input concorrenti
		NodeDepMap * dependency;

		// visitor del grafo
		Network_visitor<DGR> * _visitor;

		int pathId;
		std::map< int,PathTraits<DGR> > tgMap;
		vector<Node> dependencyList;

		//boost::thread_group * network;
		groupThread * networkPropagator;
		groupThread * networkUpdater;
		Semaphore secure;
        volatile bool canRun;
		int iter;
		double start_time;
		//boost::chrono::system_clock::time_point start_time;

	public:
		// constructor
		Network_runner();

		// disable dynamic creation of path- TODO
		// if is enabled, once a target is assigned a new path chain is created and
		// automatically deleted when the algorithm has finished
		void useDynamicCreation(const bool &ok=true){ dynamicCreation = ok;};

		// assign a graph to the runner and initialize all the internal maps.
		// it takes a (sub)digraph, the property maps (node_block+arc_block maps)
		// and the visitor class of the network that can be assigned from outside.
		// if no visitor is specified, the runner creates its own one.
		void init(DGR *g, Node root,
				  NodeBlockMap &Nblocks,
				  ArcBlockMap &Ablocks,
				  Network_visitor<DGR> *visitor = NULL);

		void init(DGR &g, Node root,
				  NodeBlockMap &Nblocks,
				  ArcBlockMap &Ablocks,
				  Network_visitor<DGR> *visitor = NULL);
		
		// assign a target to the network. It can be a position+pose, only position
		// but pose is not constrained, or only pose (target position is kept).
		// return value is false if the node is not reachable
		// @param PathId is overwritten with the id for retrieving the path chain.
		// its value is -1 in case an error occurred.
		// NB: All the path are dynamically created.
		bool setTarget(	int & PathId,
						const yarp::sig::Vector & tg,
						int node_id, const Bottle &vtgs_opt, bool compensate = true);
		bool setTarget(	int & PathId,
						const yarp::sig::Vector & tg, const yarp::sig::Matrix & pose,
						int node_id, const Bottle &vtgs_opt, bool compensate = true);
		bool setTarget(	int & PathId,
						const yarp::sig::Matrix & pose,
						int node_id, const Bottle &vtgs_opt, bool compensate = true);
		
		// get 4x4 ortonormal matrix of the target node relative to the root
		pathChain getPathChain(const int &node_id);
		Matrix getTargetState(const int &nodeId);
		Matrix getTargetState(const int &nodeId, const int &link);

		// method to retrieve a specific path to target PMP.
		bool getPathTraitsFromID(const int &PathId, PathTraits<DGR> &PT);
		bool getPathTraits(const int &NodeID, PathTraits<DGR> &PT);
		bool loadPathTraits(const int & NodeID, const PathTraits<DGR> &PT);

		// run the algortihm on the current state
		bool go();
		// check if the algoritm is running
		bool isWaiting();
		// clear the target map
		bool clear();

	private:
		// compute internal virtual torque field for each path
		inline void propagateRelaxation();
		// if a node is present in multiple paths, sum up the computed
		// joint virtual torques computed by each path and reassign the total torque to each.
		inline void updatePathsTorques();
		// compute inverse model iteration to get a new kinematic state for the active paths
		inline bool updatePathsState();
		// update nodes and connections internal state. Reset Dependency map.
		// Deals only with nodes and arcs of each path (subnetwork).
		// So the other nodes/arcs in the network are not affected
		void updateNetworkState();

		// run the algorithm
		bool threadInit();
		void run();
		void threadRelease();
		inline bool assignThreadGroup();
		inline void freeThreadGroup();

		// port communication
		inline void setNodesWriterId(Node &n, const int &id);
		void updateDependencies(lemon::Path<DGR> &p, const int &Id);
		
	};

}
}


using namespace iCub::pmp_runner;
// template definitions:

// ----------------------------- PATH_TRAITS --------------------------------------	
	
		template <class DGR>
		PathTraits<DGR>::PathTraits(): pmpGenericBlock(), _nmap(0), _amap(0), allocated(0)
		{
			tg.resize(3,0.0);
			xtg.resize(3,0.0);
			pose.resize(3,3);
			Fv.resize(6,0.0);//3,0.0);
			
			initialized = false;
			compensated = false;
			allocated = false;
			vtgsAssigned = false;
			poseAssigned = false;
			inactive = true;

			/*propagator = new propagatorThread< PathTraits<DGR> >(this);
			updater = new updaterThread< PathTraits<DGR> >(this);
			propagator->setDisabler(&inactive);
			updater->setDisabler(&inactive);
			*/
		}

		template <class DGR>
		PathTraits<DGR>::PathTraits(const PathTraits & pt):	pmpGenericBlock(pt),
											pChain(pt.pChain),
											pmp(pt.pmp),
											vtgs(pt.vtgs),
											Fv(pt.Fv)
											
		{
			this->path = pt.path;
			this->pathId = pt.pathId;
			this->tg = pt.tg;
			this->pose = pt.pose;
			this->_nmap = pt._nmap;
			this->_amap = pt._amap;
			this->targetNode = pt.targetNode;
			this->q = pt.q;
			this->torqueT = pt.torqueT;
			this->compensated = pt.compensated;
			this->initialized = pt.initialized;
			this->inactive = pt.inactive;
			this->poseAssigned = pt.poseAssigned;
			this->_vtgsMode = pt._vtgsMode;
			this->_vtgs_opt = pt._vtgs_opt;

			if (pt.allocated) this->activateTargetNode(compensated);
			else			  allocated = false;
			
			if (pt.vtgsAssigned)this->vtgsAssigned = true;
			else				this->vtgsAssigned = false;

			if(this->initialized)
			{
				propagator = new propagatorThread < PathTraits<DGR> >(this);
				updater = new updaterThread < PathTraits<DGR> >(this);
				propagator->setDisabler(&this->inactive);
				updater->setDisabler(&this->inactive);
			}

			//allocate();
			time = 0;
		}

		template <class DGR>
		PathTraits<DGR> &PathTraits<DGR>::operator = (const PathTraits & pt)
		{
			if (this != &pt)
			{
				this->path = pt.path;
				this->pChain = pt.pChain;
				this->pmp = pt.pmp;
				this->pathId = pt.pathId;
				this->tg = pt.tg;
				this->pose = pt.pose;
				this->_nmap = pt._nmap;
				this->targetNode = pt.targetNode;
				this->q = pt.q;
				this->torqueT = pt.torqueT;
				this->compensated = pt.compensated;
				this->initialized = pt.initialized;
				this->inactive = pt.inactive;
				this->vtgs = pt.vtgs;
				this->comp = pt.comp;
				this->Fv = pt.Fv;
				//this->Mv = pt.Mv;

				if (pt.allocated) this->activateTargetNode(compensated);
				else			 allocated = false;
				if (vtgsAssigned) this->vtgsAssigned = true;
				else			  this->vtgsAssigned = false;

				//allocate();
				time = 0;

				if(this->initialized)
				{
					propagator = new propagatorThread< PathTraits<DGR> >(this);
					updater = new updaterThread< PathTraits<DGR> >(this);
					propagator->setDisabler(&this->inactive);
					updater->setDisabler(&this->inactive);
				}
			}

			return *this;
		}
		template <class DGR>
		void PathTraits<DGR>::init(const DGR &g, NodeBlockMap & Nmap, ArcBlockMap & Amap, Node n, const int &_pathId)
		{
			_nmap = &Nmap;
			_amap = &Amap;
			pathId = _pathId;

			targetNode = n;
			nodeIds.push_back(g.id(n));
			//if (!Nmap[n].pwriter.isOpen()) Nmap[n].pwriter.openPort();
		
			// set pmp kinematics and internal parameters
			pChain.pushChain(Nmap[n].NodeDefaultChain(),g.id(n));
			pmp.set_K(Nmap[n].NodeForceStiffness());
			pmp.set_Km(Nmap[n].NodeTorqueStiffness());

			if (Nmap[n].NodeDefaultChain().getDOF() > 0)
			{
				pmp.push_A(Nmap[n].NodeAdmittance());
				torqueT.push(g.id(n),Nmap[n].NodeDefaultChain().getDOF());
				qT.push(g.id(n),Nmap[n].NodeChain()->getAng());
				internalConstraints c(Nmap[n].get_internalConstraints());
				pmp.push_Kint(c.Kint);
				pmp.push_qref(c.qref);
			}

			x = pChain.EndEffPosition(qT.data);
			tbg = Nmap[n].tbg();

			cout << "EE initial " << pathId << ": " << x.toString() << endl;

			tg_old = x;
			x_old = x;

			fwriter.open(Nmap[n].name());
			initialized = true;
			inactive = true;

			// allocate and initialize parameters of pmp generic block
			allocateChain();

			propagator = new propagatorThread< PathTraits<DGR> >(this);
			updater = new updaterThread< PathTraits<DGR> >(this);
			propagator->setDisabler(&inactive);
			updater->setDisabler(&inactive);
		}
		template <class DGR>
		void PathTraits<DGR>::init(const DGR &g, NodeBlockMap & Nmap, ArcBlockMap & Amap, lemon::Path<DGR> p, const int &_pathId)
		{
			_nmap = &Nmap;
			_amap = &Amap;
			path = p;
			pathId = _pathId;
			
			Arc a;
			Node n;
			int i;

			for (i=0; i<path.length(); i++)
			{
				a = path.nth(i);
				n = g.source(path.nth(i));
				nodeIds.push_back(g.id(n));
				//cout << Nmap[n].name() << endl;

				if (i==0) pChain.pushChain(Amap[a].getKine(),g.id(n),true);
				else	  pChain.pushChain(Amap[a].getKine(),g.id(n));
				pmp.push_A(Amap[a].getA());//, Amap[a].getKine());			TODO!

				torqueT.push(g.id(n),Amap[a].getKine().getDOF());
				//qT.push(g.id(n),Amap[a].getKine().getAng());				//TODO! choose the best one
				qT.push(g.id(n),Nmap[n].NodeDefaultChain().getAng());
				internalConstraints c(Nmap[n].get_internalConstraints());
				pmp.push_Kint(c.Kint);
				pmp.push_qref(c.qref);

				//if (!Nmap[n].pwriter.isOpen()) Nmap[n].pwriter.openPort();
			}
			
			n = targetNode = g.target(path.nth(i-1));
			nodeIds.push_back(g.id(n));

			pChain.pushChain(Nmap[n].NodeDefaultChain(),g.id(n));
			pmp.set_K(Nmap[n].NodeForceStiffness());
			pmp.set_Km(Nmap[n].NodeTorqueStiffness());
			
			if (Nmap[n].NodeDefaultChain().getDOF() > 0)
			{
				pmp.push_A(Nmap[n].NodeAdmittance());
				torqueT.push(g.id(n),Nmap[n].NodeDefaultChain().getDOF());
				qT.push(g.id(n),Nmap[n].NodeChain()->getAng());
				internalConstraints c(Nmap[n].get_internalConstraints());
				pmp.push_Kint(c.Kint);
				pmp.push_qref(c.qref);
			}

			//if (!Nmap[n].pwriter.isOpen()) Nmap[n].pwriter.openPort();

			x = pChain.EndEffPosition(qT.data);
			tbg = Nmap[n].tbg();

			cout << "EE initial " << pathId << ": " << x.toString() << endl;
			
			tg_old = x;
			x_old = x;

			initialized = true;
			inactive = true;

			// allocate and initialize parameters of pmp generic block
			allocateChain();

			propagator = new propagatorThread< PathTraits<DGR> >(this);
			updater = new updaterThread< PathTraits<DGR> >(this);
			propagator->setDisabler(&inactive);
			updater->setDisabler(&inactive);

			//cout << "path State: " << endl;
			//cout << writePathState().toString() << endl;
		}

		template <class DGR>
		bool PathTraits<DGR>::setTarget(const yarp::sig::Vector &v, bool compensate, const string &vtgsMode)
		{
			if (!initialized) {cout << "not initialized " << endl; return false;}
			if (!vtgsAssigned){cout << "no vtgs assigned " << endl; return false;}
			tg = v;
			_vtgsMode = vtgsMode;
			activateTargetNode(compensate);
			
			return true;
		}
		template <class DGR>
		bool PathTraits<DGR>::setTarget(const yarp::sig::Vector &v, const yarp::sig::Matrix &R, bool compensate, const string &vtgsMode)
		{
			if (!initialized) {cout << "not initialized " << endl; return false;}
			if (!vtgsAssigned){cout << "no vtgs assigned " << endl; return false;}
			tg = v;
			pose = R;
			poseAssigned = true;
			_vtgsMode = vtgsMode;
			activateTargetNode(compensate);
			//path.back()
			//Nmap[n].set_target(v,"default");

			return true;
		}
		template <class DGR>
		bool PathTraits<DGR>::setTarget(const yarp::sig::Matrix &R, bool compensate, const string &vtgsMode)
		{
			if (!initialized) {cout << "not initialized " << endl; return false;}
			if (!vtgsAssigned){cout << "no vtgs assigned " << endl; return false;}
			
			pose = R;
			poseAssigned = true;
			_vtgsMode = vtgsMode;
			activateTargetNode(compensate,true);

			return true;
		}
		template <class DGR>
		void PathTraits<DGR>::assignVtgs(const Bottle &vtgs_opt)
		{
			Property pr(vtgs_opt.toString().c_str());
			if (!vtgsAssigned)
			{
				_vtgs_opt = vtgs_opt;
				vtgsAssigned = true;			
				vtgs.init(&pr);
			}
			else
			{
				if (_vtgs_opt!=_vtgs_opt)
				{
					vtgs.update(pr);
					_vtgs_opt.clear();
					_vtgs_opt = vtgs_opt;
				}
			}
		}	
		template <class DGR>
		int PathTraits<DGR>::getTargetNodeID()
		{
			return this->nodeIds.back();			
		}

		template <class DGR>
		lemon::Path<DGR> PathTraits<DGR>::getPath()
		{
			return this->path;			
		}


		template <class DGR>
		void PathTraits<DGR>::propagateRelaxation()
		{ 
			if (finished()) return;
		
			// calcola nextVirtualposition
			if(poseAssigned)
				vtgs.stepAll(xtg,pose);	
			else
				xtg = vtgs.step();

			// calcola forza (con eventuale compensazione) e il momento
				Matrix J = this->get_Jacobian(true);
				//Matrix Jr = this->get_RotJacobian();
				double gain=0;

				if (compensated)
				  Fv.setSubvector(0,calculate_F(comp.distort_delta(xtg-x, xtg-tg_old, x-x_old, J*A*J.transposed(), (tbg.calculateGamma(vtgs.iter())*tbg.getT_dur()), gain),K) );
					//Fv.setSubvector(0,calculate_F(comp.distort_delta(xtg-x, xtg-tg_old, x-x_old, J*A*J.transposed(), tbg.calculateGamma(vtgs.iter())),K) );
				else
					Fv.setSubvector(0, calculate_F(comp.distort_delta(xtg-x,tbg.calculateGamma(vtgs.iter()),gain),K) );
					//Fv.setSubvector(0,calculate_F(comp.distort_delta(xtg-x,tbg.calculateGamma(vtgs.iter())),K) );

				if(poseAssigned)
				{
					Vector omega = calculate_omega(pose);
					//cout << endl << "omega " << omega.toString() << endl;
					Fv.setSubvector(3,calculate_M(omega,Km,vtgs.iter()) );	

					//Vector p = comp.distort_axis(omega,Jr*A*Jr.transposed());
					//fwriter.writePose(omega,omega);
					//Fv.setSubvector(3,gain*calculate_M(pose,Km,vtgs.iter()) );
				}
				
				//cout << Fv.toString() << endl;
			// calcola torque interni
				torqueT.data = A*calculate_Tau(gain*Fv);//.subVector(0,2));
				//torqueT.data = A*gain*calculate_Tau(Fv);
		}

		template <class DGR>
		void PathTraits<DGR>::updateState()
		{
			if (finished())
			{
				cout << "EE " << this->pathId << " " << x.toString() << endl;
				cout << "q: " << (this->qT.data*CTRL_RAD2DEG).toString() << endl << endl;
				cout << "pose: " << this->chain->getH().submatrix(0,2,0,2).toString() << endl;

				fwriter.close();
				//inactive = true;
				return;
			}

			// calcola velocità
				qdot = calculate_qdot(torqueT.data,vtgs.iter());
				//double sum = 0;
				//for (unsigned int i=0; i<F.size(); i++) sum+=qdot(i);
				//cout << "qsum: "<< sum << endl;
				
			// calcola angoli ai giunti e cinematica diretta
				qT.data += qdot * tbg.getSlopeRamp();
			// aggiorna tutte le variabili interne della catena (non i singoli nodi!)
				tg_old = xtg;
				x_old = x;

				x = chain->EndEffPosition(qT.data);
				Vector p = chain->EndEffPose();
				//if(pathId == 0)	cout << x.toString() << endl;

				qT.data = chain->getAng();
			
				fwriter.write(x,qT.data,xtg);
				fwriter.writeF(Fv,torqueT.data);
				fwriter.writePose(iCub::ctrl::dcm2axis(pose).subVector(0,2),p.subVector(3,5));
				writeLinksPosition();

				//cout << Fv.toString() << endl;
				//fwriter.writePose(p);

				// segment along y
				Matrix sR(4,4);
				sR.eye();
				sR(1,3) = -0.02;
				Vector p1=(chain->getH()*sR).getCol(3).subVector(0,2);
				sR(1,3) = 0.02;
				Vector p2=(chain->getH()*sR).getCol(3).subVector(0,2);

				// segment along x
				sR(1,3) = 0;
				sR(0,3) = 0.06;
				Vector p3=(chain->getH()*sR).getCol(3).subVector(0,2);
				sR(0,3) = -0.02;
				Vector p4=(chain->getH()*sR).getCol(3).subVector(0,2);
				fwriter.writeSeg(p1,p2,p3,p4);
				
				//fwriter.writeSeg(p1,p2);
				//fwriter.writeF(Fv.subVector(0,2),torqueT.data);
		}

/*		template <class DGR>
		inline bool PathTraits<DGR>::propagatorThread::runnable()
		{
			if (!inactive)	propagateRelaxation();
			return true;
		}

		template <class DGR>
		inline bool PathTraits<DGR>::updaterThread::runnable()
		{
			if (!inactive)	updateState();
			return true;
		}
*/
		template <class DGR>
		bool PathTraits<DGR>::finished()
		{
			bool done = vtgs.hasFinished();
			if (done) 
			{
				inactive = true;
			}
			return done;
		}
		template <class DGR>
		bool PathTraits<DGR>::isActive()
		{
			return !inactive;
		}
		template <class DGR>
		void PathTraits<DGR>::updatePathState(const DGR &g, typename DGR::template NodeMap<DependencyTraits> &dmap)
		{
			Arc a; 
			Node n;
			for (int i=0; i<path.length(); i++)
			{
				a = path.nth(i);
				n = g.source(path.nth(i));
		
				Vector _q = qT.subvector(g.id(n));
				if (_q.size()!= 0)
				{
					if (dmap[n].cont >= 1)
					{
						dmap[n].cont = 0;					
						(*_nmap)[n].updateState(_q);
					}
					(*_amap)[a].updateState(_q);
				}
			}
			Vector _q = qT.subvector(g.id(targetNode));
			if (_q.size() != 0)
				(*_nmap)[targetNode].updateState(qT.subvector(g.id(targetNode)));

            cout << "EE " << this->pathId << " " << x.toString() << endl;
            cout << "q: " << (this->qT.data*CTRL_RAD2DEG).toString() << endl << endl;
            cout << "pose: " << this->chain->getH().submatrix(0,2,0,2).toString() << endl;

            return;
		}

		template <class DGR>
		Bottle PathTraits<DGR>::writePathState()
		{
			Bottle AngList;
			
			for (unsigned int i=0; i<this->nodeIds.size(); i++)
			{			
				Bottle &link = AngList.addList();
				link.addInt(nodeIds[i]);
				Bottle &ang = link.addList();
				ang.append(pChain.writeRelativeAng(nodeIds[i]));
			}
			return AngList;
		}
		template <class DGR>
		void PathTraits<DGR>::writeNodesStateOnPort(const DGR &g)
		{			
			for (unsigned int i=0; i<this->nodeIds.size(); i++)
			{	
				Bottle ang = pChain.writeRelativeAng(nodeIds[i]);
				(*_nmap)[g.nodeFromId(nodeIds[i])].write(ang, pathId);				
			}
		}

		template <class DGR>
		void PathTraits<DGR>::launchPropagation()
		{
			if (this->inactive) return;
			//*pathThread = boost::thread(&Darwin::pmp::PathTraits<DGR>::propagateRelaxation, this);
		}
		template <class DGR>
		void PathTraits<DGR>::launchUpdate()
		{
			if (this->inactive) return;
			//*pathThread = boost::thread(&Darwin::pmp::PathTraits<DGR>::updateState, this);
		}

		template <class DGR>
		void PathTraits<DGR>::allocateChain()
		{
			if(!initialized) return;
			if (allocated || N>0) delete chain;
			this->chain = new iKinChain(pChain);

			N = chain->getN();

			allocateVariableSizeParams(chain->getDOF());
			chain->setAllConstraints(true);

			q = qT.data;
			A.diagonal(pmp.A);
			K = pmp.K;
			Km = pmp.Km;

			Vector ktemp(chain->getDOF());
			int n=0;
			for(unsigned int i=0; i<pmp.Kint.size(); i++)
			{				
				//cout << "it: " << i << " " << pmp.Kint[i].toString() << endl;
				ktemp.setSubvector(n,pmp.Kint[i]);
				q_ref.setSubvector(n,pmp.qref[i]);
				n += pmp.qref[i].size();
			}

			Kint.diagonal(ktemp);
			allocated = true;
		}

		template <class DGR>
		void PathTraits<DGR>::activateTargetNode( bool compensate, bool keepPosition )
		{			
			if (keepPosition) tg = chain->EndEffPosition(); // = x

			// Set vtgs' inital conditions and mode
			if (poseAssigned)
			{
				vtgs.setTarget(tg,&pose);
				Matrix iniPose = chain->getH().submatrix(0,2,0,2);
				vtgs.setInitialState(x,&iniPose);
				cout << "init pose: " << this->chain->getH().submatrix(0,2,0,2).toString() << endl;
				cout << "fin pose: " <<pose.toString() << endl;
			}
			else
			{
				vtgs.setTarget(tg);			
				vtgs.setInitialState(x);
			}
			vtgs.setMode(_vtgsMode);
			vtgs.reset();

			// initialize compensator:
			if (compensate)
			{
				comp.init(norm( tg - x ),vtgs.TimeStep());
				compensated = compensate ? true : false;
				cout << "compensator active " << std::boolalpha << compensated << endl;
			}

			inactive = false;
			fwriter.open((*_nmap)[targetNode].name());
			/*
			cout << "chain angles" << endl;
			for (unsigned int i=0; i<this->nodeIds.size(); i++)
				cout << nodeIds[i] << " " << (pChain.getRelativeAng(nodeIds[i])*CTRL_RAD2DEG).toString()<< endl;
			*/
		}

		template <class DGR>
		void PathTraits<DGR>::writeLinksPosition()
		{
			Matrix Lp(chain->getN(),3);
			for (unsigned int i =0; i<chain->getN(); i++)
				Lp.setRow(i,chain->getH(i,true).subcol(0,3,3));
			
			fwriter.writeLink(Lp);


		}


// ----------------------------- NETWORK_RUNNER --------------------------------------	
	
		template <typename DGR, typename TR>
		Network_runner<DGR,TR>::Network_runner():_g(0),
						_nBlocks(0),_aBlocks(0),
						dependency(0),
						pathId(0),
						iter(0)
		{ 
			networkPropagator = new groupThread();
			networkUpdater	  = new groupThread();

			initialized = false;
			dependent = false;
			dynamicCreation = true;
			canRun = false;
		}
	
		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::init(  DGR *g, Node root,
						    NodeBlockMap &Nblocks,
						    ArcBlockMap &Ablocks,
						    Network_visitor<DGR> *visitor)
		{
			_g = g;
			_nBlocks = &Nblocks;
			_aBlocks = &Ablocks;

			// initialize visitor algorithm
			if (!visitor)
			{
				_visitor = new Network_visitor<DGR>(*_g);
				_visitor->set_root(root);
			}
			else
				_visitor = visitor;
			_visitor->run();

			// initialize dependency map:
			dependency = Traits::createNodeDepMap(*_g);
			for (NodeIt it(*_g); it != INVALID; ++it)
				(*dependency)[it].cont = 0;

			// initialize pathMap parameters:
			tgMap.clear();
			pathId = 0;

			initialized = true;
		}		
		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::init(  DGR &g, Node root,
						    NodeBlockMap &Nblocks,
						    ArcBlockMap &Ablocks,
						    Network_visitor<DGR> *visitor)
		{
			_g = &g;
			_nBlocks = &Nblocks;
			_aBlocks = &Ablocks;

			// initialize visitor algorithm
			if (!visitor)
			{
				_visitor = new Network_visitor<DGR>(*_g);
				_visitor->set_root(root);
			}
			else
				_visitor = visitor;
			_visitor->run();

			// initialize dependency map:
			dependency = Traits::createNodeDepMap(*_g);
			for (NodeIt it(*_g); it != INVALID; ++it)
				(*dependency)[it].cont = 0;

			// initialize pathMap parameters:
			tgMap.clear();
			pathId = 0;

			initialized = true;
		}		
		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::updateDependencies( lemon::Path<DGR> &p, const int &Id)
		{
			// update graph dependency map
			Node n;
			Arc a;

			if (p.length() > 0)
			{
				n = _g->source(p.nth(0));

				if ((*dependency)[n].cont == 0)
					setNodesWriterId(n,Id);
				(*dependency)[n].cont ++;
				(*dependency)[n].pathId.push_back(Id);
				
				for (int i=0; i<p.length(); i++)
				{
					a = p.nth(i);
					n = _g->target(a);
					if ((*dependency)[n].cont == 0)
						setNodesWriterId(n,Id);
					(*dependency)[n].cont ++;
					(*dependency)[n].pathId.push_back(Id);
				}
			}
			else
			{
				n = _visitor->Root();
				if ((*dependency)[n].cont == 0)
					setNodesWriterId(n,Id);
				(*dependency)[n].cont ++;
				(*dependency)[n].pathId.push_back(Id);
			}

			//update dependencies list:
			dependencyList.clear();
			dependent = false;

			for (NodeIt Nit(*_g); Nit!=INVALID; ++Nit)
			{	if ((*dependency)[Nit].cont > 1)
				{
					dependencyList.push_back(Nit);
					//cout << _g->id(dependencyList.back()) << endl;
					dependent = true;
				}
			}

		}
		
		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::setTarget(	int & PathId,
							const yarp::sig::Vector & tg, int node_id, 
							const Bottle &vtgs_opt, bool compensate) // tg size is checked by other modules
		{	
			PathId = -1;
			if (!initialized) {cout << "object not initialized" << endl;return false;}
			if (tg.size() != 3) return false;
			if (node_id < 0) return false;
			
			lemon::Path<DGR> p;
			
			// check if is required to create a new path
			int pID = -1;
			if (!dynamicCreation)
			{
				// is required to assign a new target to an existing path:
				// check that the path is existent:
				int n=0;
				for (typename map< int,PathTraits<DGR> >::iterator it = tgMap.begin(); it != tgMap.end(); ++it)
					if (it->second.getTargetNodeID() == node_id)
					{
						pID = it->first;
						break;
					}
			}

			// the path is not existing: create a new path and assign the target
			secure.wait();
			if (pID == -1)
			{			
				// compute path to the target node and mark the mapped nodes
				p = _visitor->path(_g->nodeFromId(node_id));
			
				if(p.length() > 0)
				{				
					// save target traits for the given target node
					// id is the path id.
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,p,pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(tg,compensate);

					PathId = pathId;
					pathId++;
				}
				// target chain is root chain:
				else if(node_id == _g->id(_visitor->Root()))
				{			
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,_visitor->Root(),pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(tg,compensate);
					
					PathId = pathId;
					pathId++;
				}
				else
				{
					secure.post();
					cout << _g->id(_visitor->Root()) << endl;
					cout << node_id << endl;
					cout << "node can't be reached" << endl;
					return false;
				}

				updateDependencies(p, pathId-1);
				networkPropagator->add_thread(tgMap[pathId-1].propagator);
				networkUpdater->add_thread(tgMap[pathId-1].updater);
			}
			// assign the new target to the existing path:
			else
			{
				tgMap[pID].assignVtgs(vtgs_opt);
				tgMap[pID].setTarget(tg,compensate);
				p = tgMap[pID].getPath();

				updateDependencies(p, pID);
				networkPropagator->add_thread(tgMap[pID].propagator);
				networkUpdater->add_thread(tgMap[pID].updater);
			}

			secure.post();

/*
			// update graph dependency map
			if (p.length() > 0)
			{
				if ((*dependency)[_g->source(p.nth(0))].cont == 0)
					setNodesWriterId(Nit,pathId-1);
				(*dependency)[_g->source(p.nth(0))].cont ++;
				(*dependency)[_g->source(p.nth(0))].pathId.push_back(pathId);
				Arc a;
				for (int i=0; i<p.length(); i++)
				{
					a = p.nth(i);
					if ((*dependency)[_g->target(a)].cont == 0)
						setNodesWriterId(Nit,pathId-1);
					(*dependency)[_g->target(a)].cont ++;
					(*dependency)[_g->target(a)].pathId.push_back(pathId);
				}
			}
			else
			{
				if ((*dependency)[_visitor->Root()].cont == 0)
					setNodesWriterId(Nit,pathId-1);

				(*dependency)[_visitor->Root()].cont ++;
				(*dependency)[_visitor->Root()].pathId.push_back(pathId);
			}

			//update dependencies list:
			dependencyList.clear();
			dependent = false;

			for (NodeIt Nit(*_g); Nit!=INVALID; ++Nit)
			{	
				cout << "Node id: " << _g->id(Nit) << " cont: " << (*dependency)[Nit].cont << endl;
				if ((*dependency)[Nit].cont > 1)
				{
					dependencyList.push_back(Nit);
					dependent = true;
				}
			}
			 */
			return true;

		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::setTarget(	int & PathId,
							const yarp::sig::Vector & tg, const yarp::sig::Matrix & pose,
							int node_id, const Bottle &vtgs_opt, bool compensate) // tg size is checked by other modules
		{	
			PathId = -1;
			if (!initialized) {cout << "object not initialized" << endl;return false;}
			if (tg.size() != 3) return false;
			if (pose.rows()*pose.cols() != 9) return false;
			if (node_id < 0) return false;

			lemon::Path<DGR> p;
			
			// check if is required to create a new path
			int pID = -1;
			if (!dynamicCreation)
			{
				// is required to assign a new target to an existing path:
				// check that the path is existent:
				int n=0;
				for (typename map< int,PathTraits<DGR> >::iterator it = tgMap.begin(); it != tgMap.end(); ++it)
					if (it->second.getTargetNodeID() == node_id)
					{
						pID = it->first;
						break;
					}
			}

			// the path is not existing: create a new path and assign the target
			secure.wait();
			if (pID == -1)
			{			
				// compute path to the target node and mark the mapped nodes
				p = _visitor->path(_g->nodeFromId(node_id));
			
				if(p.length() > 0)
				{				
					// save target traits for the given target node
					// id is the path id.
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,p,pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(tg,pose,compensate);

					PathId = pathId;
					pathId++;
				}
				// target chain is root chain:
				else if(node_id == _g->id(_visitor->Root()))
				{			
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,_visitor->Root(),pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(tg,pose,compensate);
					
					PathId = pathId;
					pathId++;
				}
				else
				{
					secure.post();
					cout << _g->id(_visitor->Root()) << endl;
					cout << node_id << endl;
					cout << "node can't be reached" << endl;
					return false;
				}
				updateDependencies(p, pathId-1);

				networkPropagator->add_thread(tgMap[pathId-1].propagator);
				networkUpdater->add_thread(tgMap[pathId-1].updater);
			}
			// assign the new target to the existing path:
			else
			{
				tgMap[pID].assignVtgs(vtgs_opt);
				tgMap[pID].setTarget(tg,pose,compensate);
				p = tgMap[pID].getPath();
				updateDependencies(p, pID);

				networkPropagator->add_thread(tgMap[pID].propagator);
				networkUpdater->add_thread(tgMap[pID].updater);
			}
				
			secure.post();
			return true;
		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::setTarget(	int & PathId,
							const yarp::sig::Matrix & pose,
							int node_id,
							const Bottle &vtgs_opt,
							bool compensate) // tg size is checked by other modules
		{	
			cout << "set Target called" << endl;
			PathId = -1;
			if (!initialized) {cout << "object not initialized" << endl;return false;}
			if (pose.rows()*pose.cols() != 9) return false;
			if (node_id < 0) return false;

			lemon::Path<DGR> p;
			
			// check if is required to create a new path
			int pID = -1;
			if (!dynamicCreation)
			{
				// is required to assign a new target to an existing path:
				// check that the path is existent:
				int n=0;
				for (typename map< int,PathTraits<DGR> >::iterator it = tgMap.begin(); it != tgMap.end(); ++it)
					if (it->second.getTargetNodeID() == node_id)
					{
						pID = it->first;
						break;
					}
			}

			// the path is not existing: create a new path and assign the target
			secure.wait();
			if (pID == -1)
			{			
				// compute path to the target node and mark the mapped nodes
				p = _visitor->path(_g->nodeFromId(node_id));
			
				if(p.length() > 0)
				{				
					// save target traits for the given target node
					// id is the path id.
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,p,pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(pose,compensate);

					PathId = pathId;
					pathId++;
				}
				// target chain is root chain:
				else if(node_id == _g->id(_visitor->Root()))
				{			
					tgMap[pathId].init(*_g,*_nBlocks,*_aBlocks,_visitor->Root(),pathId);
					tgMap[pathId].assignVtgs(vtgs_opt);
					tgMap[pathId].setTarget(pose,compensate);
					
					PathId = pathId;
					pathId++;
				}
				else
				{
					secure.post();
					cout << _g->id(_visitor->Root()) << endl;
					cout << node_id << endl;
					cout << "node can't be reached" << endl;
					return false;
				}
				updateDependencies(p, pathId-1);

				networkPropagator->add_thread(tgMap[pathId-1].propagator);
				networkUpdater->add_thread(tgMap[pathId-1].updater);
			}
			// assign the new target to the existing path:
			else
			{
				tgMap[pID].assignVtgs(vtgs_opt);
				tgMap[pID].setTarget(pose,compensate);
				p = tgMap[pID].getPath();
				updateDependencies(p, pID);

				networkPropagator->add_thread(tgMap[pID].propagator);
				networkUpdater->add_thread(tgMap[pID].updater);
			}

			cout << "set Target finished" << endl;

			secure.post();
			return true;
		}

		template <typename DGR, typename TR>
		inline void Network_runner<DGR,TR>::setNodesWriterId(Node &n, const int &id)
		{
			(*_nBlocks)[n].openPort();
			(*_nBlocks)[n].setWriterId(id);

			//cout << "writer id : " << (*_nBlocks)[n].pwriter.getWriterId() << " id: " << id;
			cout << endl;
		}
		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::getPathTraitsFromID(const int &PathId, PathTraits<DGR> & PT)
		{
			// check that an existing path is required
			if (tgMap.count(PathId) == 0) return false;

			PT = tgMap[PathId];
			return true;

		}


		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::getPathTraits(const int &node_id, PathTraits<DGR> &PT)
		{
			if (node_id < 0) return false;

			// compute path to the target node and mark the mapped nodes
			lemon::Path<DGR> p = _visitor->path(_g->nodeFromId(node_id));
		
			if(p.length() > 0)
			{
				int id = -1;
				PT.init(*_g,*_nBlocks,*_aBlocks,p,id);
				PT.allocateChain();
				return true;
			}
			else
			{
				cout << _g->id(_visitor->Root()) << endl;
				cout << node_id << endl;
				cout << "node can't be reached" << endl;
				return false;
			}
		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::loadPathTraits(const int & NodeID, const PathTraits<DGR> &PT)
		{		
			if (NodeID < 0) return false;

			tgMap[NodeID] = PT;
			return true;
		}

		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::propagateRelaxation()
		{
			//for (unsigned int i=0; i<tgMap.size(); i++)
			//	tgMap[i].launchPropagation();			
				
			networkPropagator->activate();
			networkPropagator->join_all();

			for (unsigned int i=0; i<tgMap.size(); i++)
				if (tgMap[i].isActive())	tgMap[i].writeNodesStateOnPort(*this->_g);
		}	
		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::updatePathsTorques()
		{
			for (typename vector<Node>::iterator it=dependencyList.begin(); it!=dependencyList.end(); it++)
			{
				vector<int> ids = (*dependency)[*it].pathId;
				Vector Ntorques = tgMap[ids[0]].torqueT.subvector(_g->id(*it));

				for (unsigned int j=1; j<ids.size(); j++)	Ntorques += tgMap[ids[j]].torqueT.subvector(_g->id(*it));
				for (unsigned int j=0; j<ids.size(); j++)	tgMap[ids[j]].torqueT.setSubvector(_g->id(*it),Ntorques);
			}
			//cout << "update torques " << endl;
			
		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::updatePathsState()
		{
			//for (unsigned int i=0; i<tgMap.size(); i++)
			//	tgMap[i].launchUpdate();

			networkUpdater->activate();
			networkUpdater->join_all();

			bool finished = true;
			for (unsigned int i=0; i<tgMap.size(); i++)
			{	if (tgMap[i].isActive())
					finished = (finished && tgMap[i].finished());
			}

			return finished;
		}

		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::updateNetworkState()
		{			
			// call updateState for each path
            cout <<"Status: update network state"<<endl;
			for (typename map< int,PathTraits<DGR> >::iterator it=tgMap.begin(); it!=tgMap.end(); ++it)
			{
				//if (it->second.isActive())
					it->second.updatePathState(*_g,*dependency);
					Time::delay(0.5);
			}

			double final_time = Time::now();
			std::cout << "took " << final_time-start_time << " seconds\n";
	
			freeThreadGroup();
		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::go()
		{
			return assignThreadGroup();
		}
		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::isWaiting()
		{
			return !canRun;
		}
		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::clear()
		{
			if (canRun) return false;
			
			secure.wait();
				tgMap.clear();
			secure.post();
			return true;
		}
		template <typename DGR, typename TR>
		inline bool Network_runner<DGR,TR>::assignThreadGroup()
		{
			// check that some path exist:
			if(tgMap.empty())
			{
				cout << "Error: no target assigned" << endl;
				return false;
			}

			/*
			// istantiate a thread for each active path
			//network = new boost::thread_group();
			for (map< int,PathTraits<DGR> >::iterator it=tgMap.begin(); it!=tgMap.end(); ++it)
			{
				if (it->second.isActive())
				{			
					it->second.pathThread = new boost::thread();
					network->add_thread(it->second.pathThread);
				}
			}
			*/

			// check that the group has at least one active path:
			if (networkPropagator->size() > 0 && networkUpdater->size() > 0)
			{
				secure.wait();
					iter = 0;
                    cout << "Status: starting algorithm" << endl;
					start_time = Time::now();
					canRun = true;
				secure.post();

				return true;
			}
			else
			{
				cout << "Error: no active paths" << endl;
				//delete network;

				return false;
			}
		}

		template <typename DGR, typename TR>
		inline void Network_runner<DGR,TR>::freeThreadGroup()
		{
			canRun = false;
            cout << "Status: algorithm stopped" << endl;

			for (NodeIt it(*_g); it != INVALID; ++it)
				(*dependency)[it].cont = 0;

			if( dynamicCreation )
			{
				networkPropagator->interrupt_all();
				networkUpdater->interrupt_all();
				
				tgMap.clear();
				pathId = 0;
			}
		}

		template <typename DGR, typename TR>
		bool Network_runner<DGR,TR>::threadInit()
		{
			if (!initialized) return false;

			cout << "threadInit" << endl;

			// just to be sure:
			iter = 0;
			canRun = false;

			return true;
		}

		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::run()
		{	
			while (!isStopping())// && i<=500)
			{	
				if (canRun)
				{	
                    //cout <<"run"<< endl;
					secure.wait();
						propagateRelaxation();
						updatePathsTorques();
						bool done = updatePathsState();
						if (done)	updateNetworkState();
						else		iter++;
					secure.post();
				}
			}
		}

		template <typename DGR, typename TR>
		void Network_runner<DGR,TR>::threadRelease()
		{
			if (networkPropagator != NULL)  delete networkPropagator;
			if (networkUpdater != NULL)		delete networkUpdater;
			tgMap.clear();
		}

		template <typename DGR, typename TR>
		pathChain Network_runner<DGR,TR>::getPathChain(const int &node_id)
		{		
			Arc a;
			Node n;
			int i=0;
			JointTraits qT;
			pathChain ch;

			lemon::Path<DGR> p = _visitor->path(_g->nodeFromId(node_id));
			cout << "path lenght: " << p.length() << endl;
			for (i=0; i<p.length(); i++)
			{
				a = p.nth(i);	
				n = _g->source(a);

				if (i==0) ch.pushChain((*_aBlocks)[a].getKine(),_g->id(n),true);
				else	  ch.pushChain((*_aBlocks)[a].getKine(),_g->id(n));
				
				qT.push(_g->id(n),(*_nBlocks)[n].NodeChain()->getAng());
			}
			
			n = _g->target(p.nth(i-1));
			ch.pushChain((*_nBlocks)[n].NodeDefaultChain(),_g->id(n));
			
			if ((*_nBlocks)[n].NodeDefaultChain().getDOF() > 0)
				qT.push(_g->id(n),(*_nBlocks)[n].NodeChain()->getAng());

			ch.setAng(qT.data);

			cout << "qT: " << endl << qT.data.toString() << endl;
			cout << "chain ang" << endl << ch.getAng().toString() << endl;
			
			return ch;
		};


#endif
