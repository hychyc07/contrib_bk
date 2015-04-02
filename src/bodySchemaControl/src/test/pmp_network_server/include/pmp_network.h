#ifndef PMP_NETWORK_H
#define PMP_NETWORK_H

#include <lemon/list_graph.h>
#include <lemon/concepts/digraph.h>
#include <lemon/adaptors.h>
#include <lemon/dfs.h>
#include <lemon/graph_to_eps.h>

//#include <lemon/path.h>

#include <pmp_lib/core/pmpGenericBlock_if.h>
#include <pmp_lib/pmp_utils.h>

#include <vector>
#include <map>

using namespace lemon;
using namespace lemon::concepts;
using namespace iCub::iKin;
using namespace iCub::pmplib;
using namespace iCub::pmplib::core;


namespace iCub{
namespace pmp_network {

	struct internalConstraints
	{
		Vector qref;
		Vector Kint;
		//Matrix K_int;

		internalConstraints(const Vector &_qref, const Vector &_Kint) : qref(_qref), Kint(_Kint) {};
		internalConstraints(const internalConstraints & ic) : qref(ic.qref), Kint(ic.Kint){};
		internalConstraints(){};
	};

	/// \class node_block defines the properties of a pmp network node
	class node_block
	{
	private:
		bool _locked;
		pmpBlock *pmp;
		PmpPortWriter pwriter;

	public:

		// serve un costruttore con parametri di default per usare la classe come tipo nelle mappe:
		node_block(const string & name = "invalid"):blockName(name) {_locked = false;};

		//creare operatore copia solo per pmpBlock
		//node_block(const node_block &){};
		~node_block(){closePort();};

		// assign block constituents: pmp and vtgs objects
		//void assign(const pmpBlock & pmp, const vtgsBlock & vtgs);
		void allocate(const pmpBlock & _pmp)
		{
			pmp = new pmpBlock(_pmp);
			pwriter.init(blockName);
		};

		void allocate(const string & block_type)
		{
			pmp = new pmpBlock(block_type);
			pwriter.init(block_type);
		};

		inline void openPort()
		{
			if (!pwriter.isOpen())	pwriter.openPort();
		};

		inline void closePort()
		{
			if (pwriter.isOpen())	pwriter.closePort();
		};

		inline void write(PortWriter &writer, const int _writerId)
		{
			if (pwriter.isOpen())	pwriter.write(writer,_writerId);
		};

		inline void setWriterId(const int &id)
		{
			pwriter.setWriterId(id);
		};

		// returns the name of the block. It has to be unique in the network
		string name(){return this->blockName;};

		// get/set pmp internal constraints (K_int, q_ref)
		void set_internalConstraints(const Vector &qref, const Vector &K_int)
		{
			pmp->set_q_ref(qref);
			pmp->set_Kint(K_int);
		};
		internalConstraints get_internalConstraints()
		{
			Matrix Ki = pmp->get_Kint();
			Vector Kii(Ki.rows());
			for (unsigned int i=0; i<Kii.size(); i++) Kii(i) = Ki(i,i);
			//cout << "kii: " << Kii.toString() << endl;
			internalConstraints c(pmp->get_q_ref(),Kii);

			/*cout << "qref pmp: " << pmp->get_q_ref().toString() << endl;
			cout << "Kint pmp: " << pmp->get_Kint().toString() << endl;
			cout << "qref c: " << c.qref.toString() << endl;
			cout << "Kint c: " << c.Kint.toString() << endl;*/
			return c;
		};

		
		// set state parameters: q, x0
		void updateState(const Vector &_q)
		{
			pmp->set_q(_q);
		};

		Vector getEEPosition()
		{
			return pmp->get_EEposition();
		};

		Matrix getEEPose()
		{
			return pmp->get_EEpose_asRot();
		};

		
		// interface information:
		iKinChain &NodeDefaultChain()
		{
		//	cout << "q0: " << this->pmp->q_0.toString() << endl;
		//	cout << "qchain: " << this->pmp->chain->getAng().toString() << endl;
		//	cout << "qreturn: " << this->pmp->get_defaultChain()->getAng().toString() << endl;
			return *pmp->get_defaultChain();
		};

		iKinChain *NodeChain()
		{
		//	cout << "q0: " << this->pmp->q_0.toString() << endl;
		//	cout << "qchain: " << this->pmp->chain->getAng().toString() << endl;
		//	cout << "qreturn: " << this->pmp->get_defaultChain()->getAng().toString() << endl;
			return pmp->get_defaultChain();
		};

		Matrix &NodeAdmittance()
		{
			return pmp->get_A();
		};
		Matrix &NodeForceStiffness()
		{
			return pmp->get_K();
		};
		Matrix &NodeTorqueStiffness()
		{
			return pmp->get_Km();
		};

		TimeBaseGenerator tbg()
		{
			return pmp->tbg;
		};

		// lock the kinematic chain: no joint angles update allowed
		void lock(){_locked = true;};
		// lock the kinematic chain: no joint angles update allowed
		void unlock(){ _locked = false;};
		// returns true if the kinematic chain is blocked
		bool locked(){return _locked;};

		// if a target is assigned, this method first activates the vtgsBlock that creates an
		// internal target for the pmpBlock, and then it executes a pmp iteration to update the
		// block state.
		// if there is no internal target for the block, it only computes the block contribution 
		// to the network relaxation process.
		void run()
		{ 
			// if (vtgs_active)
			//		vtgs-> getNextVirtualTarget()
			//		pmp-> run()
			// else
			//		propagate_relaxation()
		};

	private:	
		string blockName;

	};	
	/// \class arc_block defines the properties of a pmp network arc
	class arc_block
	{
	private:
		iKinChain interface_chain;
		Matrix	  interface_A;

	public:
		arc_block(){};
		~arc_block(){};

		arc_block(const string & type)
		{
			pmpBlock b(type);
			interface_chain = b.get_chain();
			interface_A.resize(interface_chain.getN(), interface_chain.getN());
		};

		arc_block(const arc_block & b)
		{
			interface_chain = b.interface_chain;
			interface_A.resize(interface_chain.getN(), interface_chain.getN());
			this->interface_A = b.interface_A;
		};

		void setKine(iKinChain &ch)
		{
			interface_chain = ch;
			
			for (unsigned int i=0; i<ch.getN(); i++)
			{
				if (ch.isLinkBlocked(i)) interface_chain.blockLink(i,ch.getAng(i));
				else					 interface_chain.setAng(i,ch.getAng(i));
			}
		};

		void setA(const Matrix &m)
		{
			interface_A.resize(m.rows(),m.cols());
			interface_A = m;
		};

		void updateState(const Vector &_q)
		{
			interface_chain.setAng(_q);
		};

		iKinChain &getKine(){return interface_chain;};
		iKinChain &getInterfaceChain() const {return const_cast<iKinChain&>(interface_chain);};
		Matrix &getA(){return interface_A;};	
	};


	
	// utility class for building a name map for a pmp network graph
	template <typename DGR>
	class nameMap_utility
	{
	public:
		//typedef typename DGR::template NodeMap<node_block> NodeBlockMap;

		nameMap_utility(const DGR &_g): g(_g){};
		//nameMap_utility(const DGR &_g, const NodeBlockMap &_Nmap): g(_g),Nmap(_Nmap){};

	private:
		DGR &g;
/*		NodeBlockMap & Nmap;

	protected:
		std::map<string, int> nameMap;

	public:
		int nodeIdFromName(std::string name)
		{
			if (nameMap.count(name) == 0)	return -99;
			else							return nameMap[name];
		};
		inline bool addNodeName(const int & id, const string &name)
		{
			nameMap[b.name()] = g.id(n);
			return true;
		};
		void removeNodeName(const string & name)
		{
			int i = nameMap[name];
			nameMap.erase(name);
		};

		// create a new name map for the nodeMap of the network. Before adding the new elements,
		// it always clears the map. Not safe for duplicates: if two nodes with the same name exist,
		// the second one is not inserted in the map and the method returns false.
		bool fill_NameMap()
		{
			bool error = false;
			if(!nameMap.empty())	nameMap.clear();

			pair<string,int> in;
			pair<map<string,int>::iterator,bool> ret;
			
			for (DGR::NodeIt it_n(g); it_n!=INVALID; ++it_n)	
			{
				in.first = Nmap[it_n].name();
				in.second = g.id(it_n);
				ret = nameMap.insert(in);
				if (!ret.second)	error = false;

				// for debug:
				cout << Nmap[it_n].name() << " ";
			}
			cout << endl;

			return error;
		};
*/	};

	// template class for building a view of a directed graph:
	// APPUNTI: con questa classe creo delle viste del grafo originale. Per ogni classe 
	// sonno definite delle bool map che verranno aggiornate a seconda della vista che si vuole
	// ottenere.

	template <typename DGR=ListDigraph>
	class Network_visitor
	{

	public:
		typedef typename lemon::Dfs<DGR>::Digraph G;
		typedef typename G::Node Node;
		typedef typename lemon::Dfs<DGR>::PredMap		PredMap;
		typedef typename lemon::Dfs<DGR>::DistMap		DistMap;
		typedef typename lemon::Dfs<DGR>::ReachedMap	ReachedMap;

	private:
		lemon::Dfs<DGR>	visitor;
		PredMap			predM;
		DistMap			distM;
		ReachedMap		reachedM;
		Node			root;

		inline void runner()
		{
			cout << "runner" << endl;
			visitor
				.predMap(predM)
				.distMap(distM)
				.reachedMap(reachedM);

			visitor.init();
			visitor.addSource(root);
			visitor.start();
		};

	public:
		// cinstructor
		Network_visitor(const G &g) :
						visitor(g),predM(g),distM(g),reachedM(g)
						{root = lemon::INVALID;};

		void set_root(Node _root){root = _root;};
		Node Root(){return root;};

		// get the path from the network root to a target node.
		// the algorithm has to be run before calling this method
		lemon::Path<DGR> path(Node target)
		{
			return visitor.path(target);
		};

		//run dfs algorithm on all the graph
		void run()
		{
			cout << "run visitor nel visitor" << endl;
			runner();
		};

		// run dfs algorithm only to find paths to target nodes and write them in a map
		void run(const vector<Node> & target_list, map< Node, lemon::Path<DGR> > & m)
		{
			runner();
            typename vector<Node>::iterator it;
			for (it = target_list.begin(); it != target_list.end(); ++it)
				m[*it] = visitor.path(*it);
		};

		// run dfs algorithm to get a path to a target node
		void run(const Node & target, lemon::Path<DGR> & p)
		{
			runner();
			p = visitor.path(target);
		};

	};

	// class to create a view of a digraph (= subgraph)
	template <typename DGR=ListDigraph>
	class Network_view //: public nameMap_utility<DGR>
	{
	public:
		typedef typename DGR::template NodeMap<bool> NodeBoolMap;
		typedef typename DGR::template ArcMap<bool> ArcBoolMap;

		typedef typename DGR::Node Node;
		typedef typename DGR::Arc Arc;

		typedef typename DGR::NodeIt NodeIt;
		typedef typename DGR::ArcIt ArcIt;
		typedef typename DGR::OutArcIt OutArcIt;
		typedef typename DGR::InArcIt InArcIt;		

		typedef typename DGR::Digraph Digraph;
		typedef SubDigraph<DGR,NodeBoolMap, ArcBoolMap> Subgraph;

	private:
		DGR *g;
		Node root;

		NodeBoolMap	  bNodes;
		ArcBoolMap	  bArches;
		SubDigraph<DGR,NodeBoolMap,ArcBoolMap> netView;

	public:
		Network_visitor<Subgraph> visitor;

		// constructor
		Network_view(DGR &_g, Node _root) :
					g(&_g), root(_root),
					bNodes((*g)),bArches((*g)),netView(*g,bNodes,bArches),
					visitor(netView)
		{	
			visitor.set_root(root);
			// enable all nodes and arcs:
			for (NodeIt Nit(*g); Nit!=INVALID; ++Nit)	bNodes[Nit] = true;
			for (ArcIt Ait(*g); Ait!=INVALID; ++Ait)	bArches[Ait] = true;
			
		};//{fill_NameMap()};
		Network_view(DGR &_g, Node _root, NodeBoolMap boolN, ArcBoolMap boolA) :
					g(&_g), root(_root),
					bNodes((*g)),bArches((*g)),netView(*g,bNodes,bArches),
					visitor(netView)
		{	
			visitor.set_root(root);
			// enable all nodes and arcs:
			for (NodeIt Nit(*g); Nit!=INVALID; ++Nit)	bNodes[Nit] = boolN[Nit];
			for (ArcIt Ait(*g); Ait!=INVALID; ++Ait)	bArches[Ait] = boolA[Ait];

		};//{fill_NameMap()};

		// return the subdigraph
		SubDigraph<DGR,NodeBoolMap,ArcBoolMap> &view()
		{			
			return netView;
		};

		// return a pointer to the subdigraph
		SubDigraph<DGR,NodeBoolMap,ArcBoolMap> * pview()
		{			
			return &netView;
		};

		DGR graph()
		{			
			return *g;
		};

		// return root node
		Node Root(){return root;};

		const NodeBoolMap& NodeMap()
		{
			return bNodes;
		};
		const ArcBoolMap& ArcMap()
		{
			return bArches;
		};

		// enable a target node
		inline void enable (const Node & n)
		{
			netView.enable(n);
		};

		// enable an arc and its target node
		bool enable (const Node & s, const Node & t)
		{			
			Arc a = lemon::findArc(*g,s,t);
			if ((*g).valid(a))
			{
				enable(a);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		}

		inline void enable (const Arc & a)
		{
			netView.enable(a);
			netView.enable((*g).target(a));
		};

		void disable(const Node & n, bool cascade = true)
		{
			// disable the node and all its incoming connections
			// disable a in cascade all the outgoing connections and the target nodes
			// that depends only on the disabled node (no other incoming connections)
			if (cascade)
			{
				disable_inArches(n);
				disable_outArches(n);
			}
			netView.disable(n);
		};

		bool disable(const Node & s, const Node & t, bool cascade = true)
		{			
			Arc a = lemon::findArc(*g,s,t);
			if ((*g).valid(a))
			{
				disable(a, cascade);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		};

		void disable(const Arc & a, bool cascade)
		{
			// disable the arc and its target node if no other input arc is present for that node
			if (cascade)	disable_tgNode(a);
			netView.disable(a);			
		};

	private:
		inline void disable_inArches(const Node &n)
		{
			cout << "dis in arches of node " << g->id(n) << endl;
			for (InArcIt it(*g,n); it != INVALID; ++it)
			{
				netView.disable(it);
				cout << "status is " << std::boolalpha << netView.status(it) << endl;
			}
		};

		// disable all outgoing connections from a node.
		// if the target nodes receive only this arc as input, then disable the node in cascade
		void disable_outArches(const Node &n)
		{
			cout << "dis out arches of node " << g->id(n) << endl;
			for (OutArcIt it(*g,n); it != INVALID; ++it)
			{
				// disable the target node if this is the only input arc 
				Node nt = g->target(it);
				if(lemon::countInArcs(*g,nt) == 1)	 disable_tgNode(it);

				// disable current arc 
				netView.disable(it);
				cout << "status is " << std::boolalpha << netView.status(it) << endl;
			}
		};

		// disable the target node of an arch and its outgoing connections
		void disable_tgNode(const Arc &a)
		{
			Node n = (*g).target(a);
			cout << "dis tg node " << g->id(n) << endl;
			disable_outArches(n);
			netView.disable(n);			
		};

	};

	// class for network maps handling: it contains a node map and an arc map.
	// It also have a protected map, nameMap, only accessible for a PMP_network object
	class network_map
	{		
	public:
		ListDigraph g;
		ListDigraph::NodeMap<node_block>  nodes;
		ListDigraph::ArcMap<arc_block>    arcs;

		// root node, the one that contains the kinematic transformation to the root reference system
		// it must always be present to run the network algorithm, and it is the source of the dfs
		// search algorithm. It cannot be disabled, only blocked.
		ListDigraph::Node root;

		// nodes to wich a target has been assigned will receive a true value
		// path from the source to these nodes will be computed to run the algorithm
		// ListDigraph::NodeMap<bool>  target_nodes;
		

	private:
		// lookUp to quickly find connections between assigned nodes
		// Hp is that there exists only one arc between two nodes
		//DynArcLookUp<ListDigraph> lookUp;

	public:
		// visiting algorithm
		Network_visitor<ListDigraph> visitor;

		// create maps out of graph g
		network_map():	nodes(g), arcs(g),
						//target_nodes(g),
						visitor(g)
						//lookUp(g)
		{
			this->root = INVALID;
			visitor.set_root(this->root);
		};

		bool has_root()
		{
			return g.valid(root);
		};

	protected:
		// adds a node to the network allocating block from an existing one:
		void push_node(const pmpBlock & b, const string & name, bool IS_ROOT = false)
		{			
			node_block block(name);
			block.allocate(b);

			ListDigraph::Node N = g.addNode();
			if (IS_ROOT) root = N;

			// add node to internal maps
			nodes[N] = block;	
			this->addNodeName(N, block);
			//nodes[N].openPort();
			//this->enable(N);

			//cout << nodes[N].NodeDefaultChain().getAng().toString() << endl;

		};

		// adds a node to the network allocatinag block from type name:
		void push_node(const string & block_type, const string & name, bool IS_ROOT = false)
		{			
			node_block block(name);
			block.allocate(block_type);
		
			ListDigraph::Node N = g.addNode();
			if (IS_ROOT)
			{
				root = N;
				visitor.set_root(root);
			}

			// add node to internal maps
			nodes[N] = block;
			this->addNodeName(N, block);
			//nodes[N].openPort();
			//this->enable(N);
		};

		void pop_node(const string & name) // to be moved under protected
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[name]);
			//nodes[n].closePort();
			if (n != INVALID)	g.erase(n);			
		};

		void pop_node(ListDigraph::Node n)
		{
			this->removeNodeName(n);
			//nodes[n].closePort();
			g.erase(n);
		};

		// adds a new arc to the network
		void push_arc(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);

			arc_block b;
			b.setKine(nodes[g.source(a)].NodeDefaultChain());
			iKinChain * nodeChain = nodes[g.source(a)].NodeChain();

			for (unsigned int i=0; i<nodeChain->getN(); i++)
			{
				if (nodeChain->isLinkBlocked(i)) b.getInterfaceChain().blockLink(i,nodeChain->getAng(i));
				else							 b.getInterfaceChain().setAng(i,nodeChain->getAng(i));
			}
			b.setA(nodes[g.source(a)].NodeAdmittance());
			arcs[a] = b;
			//cout <<nodes[g.source(a)].NodeDefaultChain().getAng().toString() << endl;

			//netView.enable(a);

			cout << "set default values for connection from " << source << " to " << target << endl;
		};

		// assumes that the block has a valid admittance matrix and kinematic chain
		void push_arc(const string & source, const string & target, arc_block & block)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);

			/*
			if (block.getKine().getN()==0)
			{
				cout << "no valid kinematic chain provided: copying source node" << endl;
				block.setKine(nodes[g.source(a)].NodeDefaultChain());
			}

			// check that the block has a valid admittance matrix:
			// if not, copy the one of the source node:
			Matrix A = block.getA();
			if( (A.rows()+A.cols()) == 0)
			{
				cout << "no valid admittance provided: copying source node" << endl;
				block.setA(nodes[g.source(a)].NodeAdmittance());
			}
			*/
			arcs[a] = block;

			//netView.enable(a);
		};
		// to be used in case only admittance matrix is changed respect to the source node
		void push_arc(const string & source, const string & target, const Matrix &_A)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);

			arc_block b;
			b.setKine(nodes[g.source(a)].NodeDefaultChain());
			iKinChain * nodeChain = nodes[g.source(a)].NodeChain();

			for (unsigned int i=0; i<nodeChain->getN(); i++)
			{
				if (nodeChain->isLinkBlocked(i)) b.getInterfaceChain().blockLink(i,nodeChain->getAng(i));
				else							 b.getInterfaceChain().setAng(i,nodeChain->getAng(i));
			}

			b.setA(_A);
			arcs[a] = b;
			//cout <<nodes[g.source(a)].NodeDefaultChain().getAng().toString() << endl;

			//netView.enable(a);

			cout << "set default values for connection from " << source << " to " << target << endl;
		};

		// to be used in case only kinematics is changed respect to the source node
		void push_arc(const string & source, const string & target, const string &kineType)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);

			arc_block b(kineType);
			b.setA(nodes[g.source(a)].NodeAdmittance());
			iKinChain * nodeChain = nodes[g.source(a)].NodeChain();

			for (unsigned int i=0; i<nodeChain->getN(); i++)
			{
				if (nodeChain->isLinkBlocked(i)) b.getInterfaceChain().blockLink(i,nodeChain->getAng(i));
				else							 b.getInterfaceChain().setAng(i,nodeChain->getAng(i));
			}
			arcs[a] = b;
			//cout <<nodes[g.source(a)].NodeDefaultChain().getAng().toString() << endl;

			//netView.enable(a);

			cout << "set default values for connection from " << source << " to " << target << endl;
		};

		void push_arc(const ListDigraph::Node & source, const ListDigraph::Node & target, const arc_block & block)
		{
			ListDigraph::Arc a = g.addArc(source,target);
			arcs[a] = block;

			//netView.enable(a);
		};

		void pop_arc(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))	g.erase(a);
		};
		void pop_arc(const ListDigraph::Node & source, const ListDigraph::Node & target)
		{
			ListDigraph::Arc a = lemon::findArc(g,source,target);
			if (g.valid(a))	g.erase(a);
		};

	
		// add a node both to NodeMap and nameMap
		inline bool addNodeName(const ListDigraph::Node & n, node_block b)
		{
			nodes[n] = b;
			nameMap[b.name()] = g.id(n);

			return true;
		};

		// add a node both to NodeMap and nameMap
		void removeNodeName(const ListDigraph::Node & n)
		{
			nameMap.erase(nodes[n].name());
		};

		void removeNodeName(const string & name)
		{
			int i = nameMap[name];
			nameMap.erase(name);
		};

		// create a new name map for the nodeMap of the network. Before adding the new elements,
		// it always clears the map. Not safe for duplicates: if two nodes with the same name exist,
		// the second one is not inserted in the map and the method returns false.
		bool fill_NameMap()
		{
			bool error = false;
			if(!nameMap.empty())	nameMap.clear();

			pair<string,int> in;
			pair<map<string,int>::iterator,bool> ret;
			
			for (ListDigraph::NodeIt it_n(g); it_n!=INVALID; ++it_n)	
			{
				in.first = nodes[it_n].name();
				in.second = g.id(it_n);
				ret = nameMap.insert(in);
				if (!ret.second)	error = false;
			}

			return error;
		};

	public:		
		// questa viene creata in modo automatico!: accessibile solo se questa classe è dichiarata
		// membro di una da una pmp_network
		std::map<string, int> nameMap;

		int nodeIdFromName(const std::string &name)
		{
			//cout << "name" << " " << name << " " << nameMap.count(name) << endl;
			if (nameMap.count(name) == 0)	return -99;
			else							return nameMap[name];
		};

		ListDigraph::Node nodeFromName(const std::string &name)
		{
			if (nameMap.count(name) == 0)	return INVALID;
			else							return g.nodeFromId(nameMap[name]);
		};

		void printNameMap()
		{
			map<string,int>::iterator it;
			for (it = nameMap.begin(); it!=nameMap.end(); it++)
				cout << it->first << " " << it->second << endl;
		};

		bool block_node(const string & node_name)
		{
			if (nameMap.count(node_name) == 0) return false;
			nodes[g.nodeFromId(nameMap[node_name])].lock();
			return true;
		};

		bool release_node(const string & node_name)
		{
			if (nameMap.count(node_name) == 0) return false;
			nodes[g.nodeFromId(nameMap[node_name])].unlock();
			return true;
		};

/*		// enable a target node
		void enable (const string & node)
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[node]);
			enable(n);
		}
		inline void enable (const ListDigraph::Node & n)
		{
			netView.enable(n);
			//active_nodes[n] = true;
		};

		// enable an arc and its target node
		bool enable (const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))
			{
				enable(a);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		}
		inline void enable (const ListDigraph::Arc & a)
		{
			netView.enable(a);
			netView.enable(g.target(a));
			//active_arches[a] = true;
		};

		void disable (const string & node)
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[node]);
			disable(n);
		}
		void disable(const ListDigraph::Node & n)
		{
			// disattivo il nodo e le connessioni in ingresso.
			// disattivo a catena le conenssioni in uscita e i nodi successivi
			// dipendenti solo dal nodo cancellato
			netView.disable(n);
			disable_inArches(n);
			disable_outArches(n);
			//active_nodes[n] = false;
		};

		bool disable(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))
			{
				disable(a);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		};

		void disable(const ListDigraph::Arc & a)
		{
			// disattivo l'arco e il nodo target se non ha altri archi di input
			netView.disable(a);
			disable_tgNode(a);
			//active_arches[a] = false;
		};


	private:
		inline void disable_inArches(const ListDigraph::Node &n)
		{
			for (ListDigraph::InArcIt it(g,n); it != INVALID; ++it)
				netView.disable(it);
		};

		// disable all outgoing connections from a node.
		// if the target nodes receive only this arc as input, then disable the node in cascade
		void disable_outArches(const ListDigraph::Node &n)
		{
			for (ListDigraph::OutArcIt it(g,n); it != INVALID; ++it)
			{
				// disable current arc 
				netView.disable(it);
				// disable the target node if this is the only input arc 
				if(lemon::countInArcs(g,n) == 1)	 disable_tgNode(it);
			}
		};

		// disable the target node of an arch and its outgoing connections
		void disable_tgNode(const ListDigraph::Arc &a)
		{
			ListDigraph::Node n = g.target(a);
			netView.disable(n);
			disable_outArches(n);
		};

*/
	public:		

/*		
		void test()
		{
			
			// ATTENZIONE: queste due forme sonno equivalenti!
			//g.addNode();
			//netView.addNode();

			// ATTENZIONE: non vado al primo indice dei nodi
			//ListDigraph::Node N;
			//netView.first(N);

			// ATTENZIONE: non vado al successivo, decrementa!!
			//neView.next(N);
			
			// controllo cancellazione:
			ListDigraph::Node N1 = g.nodeFromId(0);
			string nome = nodes[N1].name();
			cout << "nome " << nome << " id " << nameMap[nome] << endl;
			pop_node(N1);
			cout << " esiste nome: " << nameMap.count(nome)<< endl;
			

			// iterazione per filtrare i nodi disattivati!
			for (int j = 0; j <= netView.maxNodeId(); ++j)
			{
				ListDigraph::Node N = netView.nodeFromId(j);				
				cout << "valid " << g.valid(N) << endl; // va: if(g.valid(node))
				
				//if (netView.nodeFromId(j) == INVALID) cout << "invalid j: " << j << endl; // non va
				if (netView.status(netView.nodeFromId(j))) 
				{
					if (g.nodeFromId(j) != INVALID)
						std::cout << nodes[g.nodeFromId(j)].name() << " " << j << endl;
				}
			}
		};
*/

	/*	void test_dfs()
		{	
			run_dfs();
			run_dfs(true);
			lemon::Path<ListDigraph> p = path(g.nodeFromId(nameMap["destro"]));
			cout << "Path to destro: lenght "<< p.length() << endl;
			p = path(g.nodeFromId(nameMap["destro1"]));
			cout << "Path to destro1: lenght "<< p.length() << endl;
			p = path(g.nodeFromId(nameMap["sinistro"]));
			cout << "Path to sinistro: lenght "<< p.length() << endl;
			cout << endl;

			cout << "FILTERED VERSIONS: " << endl;
			ListDigraph::Node n;
			if(reachedM_f[n=g.nodeFromId(nameMap["destro"])])
			{
				p = path(g.nodeFromId(nameMap["destro"]),true);
				cout << "Path to destro: lenght "<< p.length() << endl;
			}
			else
				cout << "destro is disabled" << endl;
		
			if(reachedM_f[n=g.nodeFromId(nameMap["destro1"])])
			{
				p = path(g.nodeFromId(nameMap["destro1"]),true);
				cout << "Path to destro1: lenght "<< p.length() << endl;
			}
			else
				cout << "destro1 is disabled" << endl;

			if(reachedM_f[n=g.nodeFromId(nameMap["sinistro"])])
			{
				p = path(g.nodeFromId(nameMap["sinistro"]),true);
				cout << "Path to sinistro: lenght "<< p.length() << endl;
			}
			else
				cout << "sinistro is disabled" << endl;
		};

*/
		void test_view()
		{
			cout << "run visitor" << endl;
			cout << "test id root: " << g.id(this->root) << " " << g.id(visitor.Root()) << endl;
			visitor.run();
			Network_view<ListDigraph> NV(g,root);
			
			NV.disable(g.nodeFromId(nameMap["destro"]));
			//NV.disable(g.nodeFromId(nameMap["sinistro"]));

			for (ListDigraph::NodeIt i(g); i!=INVALID; ++i)	
			{
				//cout << "status " << std::boolalpha << NV.view().status(i) << endl;
				//if(NV.view().status(i))		std::cout << nodes[i].name() << " " << g.id(i) << " ";
			}
			cout << endl;

			Network_view<Network_view<>::Subgraph> SNV(NV.view(), NV.Root());//, NV.NodeMap(), NV.ArcMap());
	/*		for (ListDigraph::NodeIt i(g); i!=INVALID; ++i)	
			{
				cout << "status " << std::boolalpha << SNV.view().status(i) << endl;
				//if(SNV.view().status(i))		std::cout << nodes[i].name() << " " << g.id(i) << " ";
			}
	*/
			for (Network_view<Network_view<>::Subgraph>::NodeIt i(SNV.graph()); i!=INVALID; ++i)	
			{
				cout << "status " << std::boolalpha << SNV.view().status(i) << endl;
				//if(SNV.view().status(i))		std::cout << nodes[i].name() << " " << g.id(i) << " ";
			}
			cout << endl;

			//NV.view();

			//SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>::Adaptor::NodeMap<node_block> Nm(netView);
			//Network_view<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>,
						//ListDigraph> view(netView,nodes,arcs);
			//Network_view<> view(g,nodes,arcs);
		};

	friend class PMP_network;

	} ;

	

	// creo una classe di proprietà da associare alla mappa di archi:
	// dovrà contenere tra le altre la cinematica di interfaccia
	// in più ci sarà un parametro che indica se la connessione è attiva (risente del campo di F virtuale)
	// oppure è passiva (il blocco a monte è rigido, i link sono bloccati).
	// Tutto in PMP_core
	// class for representing and manipulating a pmp-based network
	// the network is represented as a Simple Graph (CHECK!!!!!! TODO:
	// Nessun arco connette lo stesso nodo. Non esistono archi paralleli tra 2 nodi (ok))
	class PMP_network
	{
	public:
		//lemon::ListDigraph net;	// to be moved to private
		network_map net;

		PMP_network(){cout << "network costruttore" << endl;};

		ListDigraph *graph()
		{
			return &net.g;
		};

		network_map *map()
		{
			return &net;
		};

		// add a node:
		void push_node(const pmpBlock & b, const string & name, bool IS_ROOT = false)			{net.push_node(b,name, IS_ROOT);};
		void push_node(const string & block_type, const string & name, bool IS_ROOT = false)	{net.push_node(block_type,name, IS_ROOT);}

		// delete a node:
		void pop_node(const string &name){net.pop_node(name);};
		void pop_arc(const string &source, const string &target){net.pop_arc(source, target);};

		// connect two nodes: if no block is passed, it initializes the connection to the source node values
		bool connect(const string & source_name, const string & target_name, arc_block & block) // public usage -> usa la nameMap
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99) 
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
			{
				net.push_arc(source_name, target_name, block);
				return true;
			}
		};

		bool connect(const string & source_name, const string & target_name, const string &kineType) // public usage -> usa la nameMap
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99) 
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
			{
				net.push_arc(source_name, target_name, kineType);
				return true;
			}
		};

		bool connect(const string & source_name, const string & target_name,const Matrix &A) // public usage -> usa la nameMap
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99) 
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
			{
				net.push_arc(source_name, target_name, A);
				return true;
			}
		};
		bool connect(const string & source_name, const string & target_name)
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99) 
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
			{
				net.push_arc(source_name, target_name);
				return true;
			}
		};

		bool disconnect(const string & source_name, const string & target_name)
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99) 
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
			{
				net.pop_arc(source_name, target_name);
				return true;
			}
		};
		
/*		// enable a node or a connection:
		bool enable_node(const string & node_name)
		{
			if (net.nodeIdFromName(node_name) == -99 )
			{
				cout << "non existing node" << endl;
				return false;
			}
			else
			{
				net.enable(node_name);
				return true;
			}
		};

		bool enable_connection(const string & source_name, const string & target_name)
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99)
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
				return net.enable(source_name, target_name);
		};

		bool disable_node(const string & node_name)
		{
			if (net.nodeIdFromName(node_name) == -99 )
			{
				cout << "non existing node" << endl;
				return false;
			}
			else
			{
				net.disable(node_name);
				return false;
			}
		};

		bool disable_connection(const string & source_name, const string & target_name)
		{
			if (net.nodeIdFromName(source_name) == -99 || net.nodeIdFromName(target_name) == -99)
			{
				cout << "non existing nodes" << endl;
				return false;
			}
			else
				return net.disable(source_name, target_name);
		}
*/
		// block/release a node:
		bool block_node(const string & node_name)	{return net.block_node(node_name);};
		bool release_node(const string & node_name)	{return net.release_node(node_name);};

		// aggiungo funzioni di interfaccia:
		// def target, vincoli interni e pose EE, parametri movimento e nodi
		// funzione run() della rete PMP
		int run(const bool & overt)
		{
			cout << "run network" << endl;
			if (!net.has_root()) {cout << "no root specified" << endl; return -1;} // No root error.
			return 0;
		};

		// per aggiornare una mappa di rete ogni volta che la rete viene modificata
		// si può usare una allArcLookUp per controllare se nuove connessioni sono state fatte
		// probabilmente non serve.. più importante la submap!!!
		void refreshMaps();

		// ridà una rete che è un sottoinsieme della rete di partenza
		// le mappe della rete sono aggiornate per la sottorete.
		// guarda FilterNodes/arcs
		int nodeId(string name)
		{
			return net.nodeIdFromName(name);
		};

		PMP_network subnetwork(vector<string> names_list);

		// ridà una sottovista della rete: sotto mappe!

		// valutare quali parametri di input dare: indirizzo del nodo?
		// questo metodo deve aggiornare una lista di nodi a cui è stato assegnato un target
		// che non hanno
		void set_Target(const string& node_Name);
		void set_Target(const lemon::ListDigraph::NodeIt & node_it);
		void set_Target(const int & node_id);
		
	private:
		//lemon::ListDigraph net;	
		void test();
	};


	template <typename DGR=ListDigraph>
	class Network_drawer
	{
		DGR * _g;
		map<string,int> _names;

		typedef dim2::Point<int> Point;
		typedef typename DGR::template NodeMap<int> NodeIntMap;
		typedef typename DGR::template NodeMap<string> NodeStringMap;
		typedef typename DGR::template NodeMap<Point> NodePointMap;
		typedef typename DGR::NodeIt NodeIt;
	public:
		Network_drawer(){};

		void init(DGR * g,const map<string,int> &nameMap){_g = g;_names = nameMap;};
		void draw(const string &fileName)
		{
			NodeIntMap color(*_g);
			NodeIntMap shapes(*_g);
			NodeStringMap names(*_g);
			NodePointMap coord(*_g);

			int pin, pfin;
			pin = pfin = 10;
			int i=1;
			for (NodeIt it(*_g); it!= INVALID; ++it)
			{
				//color[it] = i;
				color[it] = _g->id(it);
				shapes[it] = 0;
				
				if (i%2)	{coord[it] = Point(pin,pfin+15); pfin+=15;}
				else		{coord[it] = Point(pin-15,pfin); pin+=15;}
				for (map<string,int>::iterator mit=_names.begin(); mit!=_names.end(); ++mit)
				{
					if(mit->second == _g->id(it)) names[it] = mit->first;
				}
				
				i++;
			}

			Palette palette;
			string name = fileName + ".eps";
			graphToEps(*_g,name).scaleToA4().
			title("PMP network graph").
			copyright("(C) 2012 Dalia De Santis").
			coords(coord).nodeShapes(shapes).
			absoluteNodeSizes().absoluteArcWidths().
			nodeScale(5).
			nodeColors(composeMap(palette,color)).
			distantColorNodeTexts().
			//negateY().
			edgeWidthScale(.5).
			nodeTexts(names).nodeTextSize(2).
			drawArrows().arrowWidth(2).arrowLength(2).
			run();
		};
	};
	
/*

class PMP_network : public lemon::ListDigraph
	{
	public:
		// aggiungo funzioni di interfaccia:
		// def target, vincoli interni e pose EE, parametri movimento e nodi
		// funzione run() della rete PMP
		int run(const bool & overt);

		// valutare quali parametri di input dare: indirizzo del nodo?
		// questo metodo deve aggiornare una lista di nodi a cui è stato assegnato un target
		// che non hanno
		void set_Target();
		
		
	private:
		void test();
	};
	class PMP_block
	{
	public:
		string s;
		vector<int> v_int;
		PMP_block(string _s, vector<int> _v):s(_s),v_int(_v){};
		PMP_block(){};

	};

	class VTGS_block
	{
	public:
		string s;
		vector<int> v_int;
		
		VTGS_block(){};

	};
*/
	

/*	class network_view
	{
	private:
		
	public:
		ListDigraph g;
		ListDigraph::NodeMap<node_block>  nodes;
		ListDigraph::ArcMap<arc_block>    arcs;

		// create an invalid graph with maps
		network_view(): g(),nodes(g),arcs(g){};

		// create a graph copying maps from existing ones
		network_view(const ListDigraph & _g, 
					const ListDigraph::NodeMap<node_block>  & _nm,
					const ListDigraph::ArcMap<arc_block> & _am) 
					: g(), nodes(g), arcs(g)
		{		
			DigraphCopy<ListDigraph,ListDigraph> dg_copy(_g,g);
			
			// create nodes and arcs refernces: is it necessary?
			ListDigraph::NodeMap<ListDigraph::Node> nmap(g);
			ListDigraph::ArcMap<ListDigraph::Arc> amap(g);
			dg_copy.nodeRef(nmap);
			dg_copy.arcRef(amap);

			// copy maps:
			dg_copy.nodeMap(_nm,nodes);
			dg_copy.arcMap (_am, arcs);
			
			dg_copy.run();
		};

		// create a graph from an existing one
		network_view(const ListDigraph & _g): g(), nodes(g), arcs(g)
		{
			DigraphCopy<ListDigraph,ListDigraph> dg_copy(_g,g);
			// create nodes and arcs refernces: is it necessary?
			ListDigraph::NodeMap<ListDigraph::Node> nmap(g);
			ListDigraph::ArcMap<ListDigraph::Arc> amap(g);
			dg_copy.nodeRef(nmap);
			dg_copy.arcRef(amap);

			dg_copy.run();
		}

		// assign a graph:
		void assign(const ListDigraph &_g)
		{
			DigraphCopy<ListDigraph,ListDigraph> dg_copy(_g,g);

			// create nodes and arcs references: is it necessary?
			ListDigraph::NodeMap<ListDigraph::Node> nmap(_g);
			ListDigraph::ArcMap<ListDigraph::Arc> amap(_g);
			dg_copy.nodeRef(nmap);
			dg_copy.arcRef(amap);

			dg_copy.run();
		}

	protected:
		// questa viene creata in modo automatico!: accessibile solo se questa classe è dichiarata
		// membro di una da una pmp_network
		std::map<string, int> nameMap;

		ListDigraph::Node & nodeIdFromName(std::string name); //TODO

		// create a new name map for the nodeMap of the network. Before adding the new elements,
		// it always clears the map. Not safe for duplicates: if two nodes with the same name exist,
		// the second one is not inserted in the map and the method returns false.
		bool fill_NameMap()
		{
			bool error = false;
			if(!nameMap.empty())	nameMap.clear();

			pair<string,int> in;
			pair<map<string,int>::iterator,bool> ret;
			
			for (ListDigraph::NodeIt it_n(g); it_n!=INVALID; ++it_n)	
			{
				in.first = nodes[it_n].name();
				in.second = g.id(it_n);
				ret = nameMap.insert(in);
				if (!ret.second)	error = false;
			}

			return error;
		};

	friend class PMP_network;

	} ;
*/

}
}

#endif

/*
// class for network maps handling: it contains a node map and an arc map.
	// It also have a protected map, nameMap, only accessible for a PMP_network object
	class network_map
	{		
	public:
		ListDigraph g;
		ListDigraph::NodeMap<node_block>  nodes;
		ListDigraph::ArcMap<arc_block>    arcs;

		// root node, the one that contains the kinematic transformation to the root reference system
		// it must always be present to run the network algorithm, and it is the source of the dfs
		// search algorithm. It cannot be disabled, only blocked.
		ListDigraph::Node root;

		// nodes to wich a target has been assigned will receive a true value
		// path from the source to these nodes will be computed to run the algorithm
		ListDigraph::NodeMap<bool>  target_nodes;
		
	//private:

		// Maps for creating network views:
		ListDigraph::NodeMap<bool>  active_nodes;
		ListDigraph::ArcMap<bool>   active_arches;
		SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>> netView;

	private:
		// dfs visitor algorithm and maps to compute the shortest (and unique for pmp) path between two nodes.
		lemon::Dfs<ListDigraph>				visitor;
		lemon::Dfs<ListDigraph>::PredMap	predM;
		lemon::Dfs<ListDigraph>::DistMap	distM;
		lemon::Dfs<ListDigraph>::ReachedMap	reachedM;

		// dfs visitor for the network view graph
		lemon::Dfs<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>>			  visitor_f;
		lemon::Dfs<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>>::PredMap	  predM_f;
		lemon::Dfs<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>>::DistMap	  distM_f;
		lemon::Dfs<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>>::ReachedMap  reachedM_f;

		// lookUp to quickly find connections between assigned nodes
		// Hp is that there exists only one arc between two nodes
		//DynArcLookUp<ListDigraph> lookUp;

	public:
		// create maps out of graph g
		network_map():	nodes(g), arcs(g),
						active_nodes(g), active_arches(g),
						target_nodes(g),
						netView(g,active_nodes,active_arches),
						visitor(g),predM(g),distM(g),reachedM(g),
						visitor_f(netView),predM_f(netView),distM_f(netView),reachedM_f(netView)
						//lookUp(g)
		{};

		bool has_root()
		{
			return g.valid(root);
		};

		// adds a node to the network allocating block from an existing one:
		void push_node(const pmpBlock & b, const string & name, bool IS_ROOT = false)
		{			
			node_block block(name);
			block.allocate(b);

			ListDigraph::Node N = g.addNode();
			if (IS_ROOT) root = N;

			// add node to internal maps
			nodes[N] = block;	
			this->addNodeName(N, block);
			this->enable(N);

		};

		// adds a node to the network allocatinag block from type name:
		void push_node(const string & block_type, const string & name, bool IS_ROOT = false)
		{			
			node_block block(name);
			block.allocate(block_type);
		
			ListDigraph::Node N = g.addNode();
			if (IS_ROOT) root = N;

			// add node to internal maps
			nodes[N] = block;
			this->addNodeName(N, block);
			this->enable(N);
		};

		void pop_node(const string & name) // to be moved under protected
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[name]);
			if (n != INVALID)	g.erase(n);
		};

		void pop_node(ListDigraph::Node n)
		{
			this->removeNodeName(n);
			g.erase(n);
		};

		void push_arc(const string & source, const string & target, const arc_block & block)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);
			arcs[a] = block;

			netView.enable(a);
		};

		void push_arc(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);

			if (g.valid(a))
			{
				cout << "a connection is already existing between " << source << " and " << target << endl;
				return;
			}

			a = g.addArc(s,t);

			arc_block b;
			b.setKine(nodes[g.source(a)].NodeDefaultChain());
			b.setA(nodes[g.source(a)].NodeAdmittance());
			arcs[a] = b;

			netView.enable(a);

			cout << "set default values for connection from " << source << " to " << target << endl;
		};

		void push_arc(const ListDigraph::Node & source, const ListDigraph::Node & target, const arc_block & block)
		{
			ListDigraph::Arc a = g.addArc(source,target);
			arcs[a] = block;

			netView.enable(a);
		};

		void pop_arc(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))	g.erase(a);
		};
		void pop_arc(const ListDigraph::Node & source, const ListDigraph::Node & target)
		{
			ListDigraph::Arc a = lemon::findArc(g,source,target);
			if (g.valid(a))	g.erase(a);
		};
	protected:
		// questa viene creata in modo automatico!: accessibile solo se questa classe è dichiarata
		// membro di una da una pmp_network
		std::map<string, int> nameMap;
		int nodeIdFromName(std::string name)
		{
			if (nameMap.count(name) == 0)	return -99;
			else							return nameMap[name];
		};
		// add a node both to NodeMap and nameMap
		inline bool addNodeName(const ListDigraph::Node & n, node_block b)
		{
			nodes[n] = b;
			nameMap[b.name()] = g.id(n);

			return true;
		};

		// add a node both to NodeMap and nameMap
		void removeNodeName(const ListDigraph::Node & n)
		{
			nameMap.erase(nodes[n].name());
		};

		void removeNodeName(const string & name)
		{
			int i = nameMap[name];
			nameMap.erase(name);
		};

		// create a new name map for the nodeMap of the network. Before adding the new elements,
		// it always clears the map. Not safe for duplicates: if two nodes with the same name exist,
		// the second one is not inserted in the map and the method returns false.
		bool fill_NameMap()
		{
			bool error = false;
			if(!nameMap.empty())	nameMap.clear();

			pair<string,int> in;
			pair<map<string,int>::iterator,bool> ret;
			
			for (ListDigraph::NodeIt it_n(g); it_n!=INVALID; ++it_n)	
			{
				in.first = nodes[it_n].name();
				in.second = g.id(it_n);
				ret = nameMap.insert(in);
				if (!ret.second)	error = false;
			}

			return error;
		};

	public:
		bool block_node(const string & node_name)
		{
			if (nameMap.count(node_name) == 0) return false;
			nodes[g.nodeFromId(nameMap[node_name])].lock();
			return true;
		};

		bool release_node(const string & node_name)
		{
			if (nameMap.count(node_name) == 0) return false;
			nodes[g.nodeFromId(nameMap[node_name])].unlock();
			return true;
		};

		// enable a target node
		void enable (const string & node)
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[node]);
			enable(n);
		}
		inline void enable (const ListDigraph::Node & n)
		{
			netView.enable(n);
			//active_nodes[n] = true;
		};

		// enable an arc and its target node
		bool enable (const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))
			{
				enable(a);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		}
		inline void enable (const ListDigraph::Arc & a)
		{
			netView.enable(a);
			netView.enable(g.target(a));
			//active_arches[a] = true;
		};

		void disable (const string & node)
		{
			ListDigraph::Node n = g.nodeFromId(nameMap[node]);
			disable(n);
		}
		void disable(const ListDigraph::Node & n)
		{
			// disattivo il nodo e le connessioni in ingresso.
			// disattivo a catena le conenssioni in uscita e i nodi successivi
			// dipendenti solo dal nodo cancellato
			netView.disable(n);
			disable_inArches(n);
			disable_outArches(n);
			//active_nodes[n] = false;
		};

		bool disable(const string & source, const string & target)
		{
			ListDigraph::Node s = g.nodeFromId(nameMap[source]);
			ListDigraph::Node t = g.nodeFromId(nameMap[target]);
			
			ListDigraph::Arc a = lemon::findArc(g,s,t);
			if (g.valid(a))
			{
				disable(a);
				return true;
			}
			else
			{
				cout << "non existing arc" << endl;
				return false;
			}
		};

		void disable(const ListDigraph::Arc & a)
		{
			// disattivo l'arco e il nodo target se non ha altri archi di input
			netView.disable(a);
			disable_tgNode(a);
			//active_arches[a] = false;
		};


	private:
		inline void disable_inArches(const ListDigraph::Node &n)
		{
			for (ListDigraph::InArcIt it(g,n); it != INVALID; ++it)
				netView.disable(it);
		};

		// disable all outgoing connections from a node.
		// if the target nodes receive only this arc as input, then disable the node in cascade
		void disable_outArches(const ListDigraph::Node &n)
		{
			for (ListDigraph::OutArcIt it(g,n); it != INVALID; ++it)
			{
				// disable current arc 
				netView.disable(it);
				// disable the target node if this is the only input arc 
				if(lemon::countInArcs(g,n) == 1)	 disable_tgNode(it);
			}
		};

		// disable the target node of an arch and its outgoing connections
		void disable_tgNode(const ListDigraph::Arc &a)
		{
			ListDigraph::Node n = g.target(a);
			netView.disable(n);
			disable_outArches(n);
		};

		public:		
		// get the path from the network root to a target node.
		// the algorithm has to be run before calling this method
		lemon::Path<ListDigraph> path(ListDigraph::Node target, bool filter=false)
		{
			if (filter)			return visitor_f.path(target);
			else				return visitor.path(target);
		};

		// run dfs on all graph nodes starting from root node - does not work
		inline void dfs_runner()
		{
			visitor
				.predMap(predM)
				.distMap(distM)
				.reachedMap(reachedM);

			visitor.init();
			visitor.addSource(root);
			visitor.start();
		};

		inline void dfs_filt_runner()
		{
			visitor_f
				.predMap(predM_f)
				.distMap(distM_f)
				.reachedMap(reachedM_f);

			visitor_f.init();
			visitor_f.addSource(root);
			visitor_f.start();
		};

		void run_dfs(bool filter=false)
		{
			if (filter) dfs_filt_runner();
			else		dfs_runner();
		};

		void run_dfs(vector<ListDigraph::Node> & target_list, map<ListDigraph::Node,lemon::Path<ListDigraph>> & m, bool filter)
		{
			if (filter)
			{
				dfs_filt_runner();
				for (vector<ListDigraph::Node>::iterator it = target_list.begin(); it != target_list.end(); ++it)
					m[*it] = visitor_f.path(*it);
			}
			else
			{
				dfs_runner();
				for (vector<ListDigraph::Node>::iterator it = target_list.begin(); it != target_list.end(); ++it)
					m[*it] = visitor.path(*it);
			}
		};

		void run_dfs(const ListDigraph::Node & target, lemon::Path<ListDigraph> & p, bool filter)
		{
			if (filter)
			{
				dfs_filt_runner();
				p = visitor_f.path(target);
			}
			else
			{
				dfs_runner();
				p = visitor.path(target);
			}
		}

		
		// run dfs visit algorithm on all the filtered graph starting from root node
		void run_dfs_filt()
		{
			visitor
				.predMap(predM)
				.distMap(distM)
				.reachedMap(reachedM);
				//.nodeFilt(active_nodes)
				//.arcFilt(active_arches);

			visitor.init();
			visitor.addSource(root);
			visitor.start();

			cout << "Path to destro: lenght "<< visitor.path(g.nodeFromId(nameMap["destro1"])).length() << endl;
		};


		// run dfs visit algorithm to get the path to a target node
		bool run_dfs_filt(const ListDigraph::Node & target, lemon::Path<ListDigraph> & p)
		{
			ListDigraph::Arc a;
			ListDigraph::Node n;

			visitor
				.predMap(predM)
				.distMap(distM)
				.reachedMap(reachedM);
				//.nodeFilt(active_nodes)
				//.arcFilt(active_arches);

			visitor.init();
			visitor.addSource(root);
			visitor.start(target);

			// if node is reachable from the assigned source, 
			// then write the path to it:
			if (reachedM[target])
			{
				p = visitor.path(target);
				return true;
			}
			else
				return false;
		};

		// run dfs visit algorithm to get the path to a list of target nodes
		void run_dfs_filt(vector<ListDigraph::Node> & target_list, map<ListDigraph::Node,lemon::Path<ListDigraph>> & m)
		{
			visitor
				.predMap(predM)
				.distMap(distM)
				.reachedMap(reachedM);
				//.nodeFilt(active_nodes)
				//.arcFilt(active_arches);

			visitor.init();
			visitor.addSource(root);
			visitor.start();

			for (vector<ListDigraph::Node>::iterator it = target_list.begin(); it != target_list.end(); ++it)
				m[*it] = visitor.path(*it);

			return;
		};

		
		void test()
		{
			
			// ATTENZIONE: queste due forme sonno equivalenti!
			//g.addNode();
			//netView.addNode();

			// ATTENZIONE: non vado al primo indice dei nodi
			//ListDigraph::Node N;
			//netView.first(N);

			// ATTENZIONE: non vado al successivo, decrementa!!
			//neView.next(N);
			
			// controllo cancellazione:
			ListDigraph::Node N1 = g.nodeFromId(0);
			string nome = nodes[N1].name();
			cout << "nome " << nome << " id " << nameMap[nome] << endl;
			pop_node(N1);
			cout << " esiste nome: " << nameMap.count(nome)<< endl;
			

			// iterazione per filtrare i nodi disattivati!
			for (int j = 0; j <= netView.maxNodeId(); ++j)
			{
				ListDigraph::Node N = netView.nodeFromId(j);				
				cout << "valid " << g.valid(N) << endl; // va: if(g.valid(node))
				
				//if (netView.nodeFromId(j) == INVALID) cout << "invalid j: " << j << endl; // non va
				if (netView.status(netView.nodeFromId(j))) 
				{
					if (g.nodeFromId(j) != INVALID)
						std::cout << nodes[g.nodeFromId(j)].name() << " " << j << endl;
				}
			}
		};


		void test_dfs()
		{	
			run_dfs();
			run_dfs(true);
			lemon::Path<ListDigraph> p = path(g.nodeFromId(nameMap["destro"]));
			cout << "Path to destro: lenght "<< p.length() << endl;
			p = path(g.nodeFromId(nameMap["destro1"]));
			cout << "Path to destro1: lenght "<< p.length() << endl;
			p = path(g.nodeFromId(nameMap["sinistro"]));
			cout << "Path to sinistro: lenght "<< p.length() << endl;
			cout << endl;

			cout << "FILTERED VERSIONS: " << endl;
			ListDigraph::Node n;
			if(reachedM_f[n=g.nodeFromId(nameMap["destro"])])
			{
				p = path(g.nodeFromId(nameMap["destro"]),true);
				cout << "Path to destro: lenght "<< p.length() << endl;
			}
			else
				cout << "destro is disabled" << endl;
		
			if(reachedM_f[n=g.nodeFromId(nameMap["destro1"])])
			{
				p = path(g.nodeFromId(nameMap["destro1"]),true);
				cout << "Path to destro1: lenght "<< p.length() << endl;
			}
			else
				cout << "destro1 is disabled" << endl;

			if(reachedM_f[n=g.nodeFromId(nameMap["sinistro"])])
			{
				p = path(g.nodeFromId(nameMap["sinistro"]),true);
				cout << "Path to sinistro: lenght "<< p.length() << endl;
			}
			else
				cout << "sinistro is disabled" << endl;
		};


		void test_view()
		{
			Network_view<> NV(g);
			
			NV.netView->disable(g.nodeFromId(nameMap["destro1"]));
			NV.netView->disable(g.nodeFromId(nameMap["sinistro"]));

			NV.view();
			//NV.view();

			//SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>::Adaptor::NodeMap<node_block> Nm(netView);
			//Network_view<SubDigraph<ListDigraph,ListDigraph::NodeMap<bool>,ListDigraph::ArcMap<bool>>,
						//ListDigraph> view(netView,nodes,arcs);
			//Network_view<> view(g,nodes,arcs);
		};
	friend class PMP_network;

	} ;
*/

