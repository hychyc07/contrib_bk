#ifndef PMP_RUNNER_CPP
#define PMP_RUNNER_CPP

#include "pmp_runner.h"

using namespace lemon;
using namespace lemon::concepts;
using namespace iCub::iKin;
//using namespace boost;
using namespace iCub::pmp_runner;

// ----------------------------- FILE_WRITER -------------------------------------
		void fileWriter::open(string _name)
		{
			this->name = _name;
			x_file.open((name + "_x.txt").c_str());
			q_file.open((name + "_q.txt").c_str());
			tg_file.open((name + "_tg.txt").c_str());
			F_file.open((name + "_F.txt").c_str());
			T_file.open((name + "_T.txt").c_str());
			segment_file.open((name + "_segm.txt").c_str());
			pose_file.open((name + "_pose.txt").c_str());
			eigs_file.open((name + "_eigs.txt").c_str());
			link_file.open((name + "_links.txt").c_str());
		}
		void fileWriter::close()
		{
			x_file.close();
			q_file.close();
			tg_file.close();
			F_file.close();
			T_file.close();
			segment_file.close();
			pose_file.close();
			eigs_file.close();
			link_file.close();
		}
		void fileWriter::write(const Vector &x, const Vector &q, const Vector &tg)
		{
			x_file << x.toString() << endl;
			q_file << (q*CTRL_RAD2DEG).toString() << endl;
			tg_file << tg.toString() << endl;
		}
		void fileWriter::writeF(const Vector &F, const Vector &T)
		{
			F_file << F.toString() << endl;
			T_file << T.toString() << endl;
		}
		void fileWriter::writeSeg(const Vector &p1, const Vector &p2)
		{
			segment_file << p1.toString() << " " << p2.toString() << endl;
		}

		void fileWriter::writeSeg(const Vector &p1, const Vector &p2,const Vector &p3, const Vector &p4)
		{
			segment_file << p1.toString() << " " << p2.toString()<<" ";
			segment_file << p3.toString() << " " << p4.toString()<< endl;
		}

		void fileWriter::writePose(const Vector &p, const Vector &mp)
		{
			pose_file << p.toString() << " " << mp.toString() << endl;
		}
		void fileWriter::writeEigs(const Matrix &eV)
		{
			eigs_file << eV.toString() << endl;
		}
		void fileWriter::writeLink(const Matrix &pi)
		{
			for (int i=0; i<pi.rows(); i++)
				link_file << pi.getRow(i).toString() << "\t";
	
			link_file << endl;
		}


// ----------------------------- JOINT_TRAITS-------------------------------------
		void JointTraits::push( int n_id, const Vector &vel)
		{
			for (unsigned int i=0; i<vel.size(); i++)
			{
				data.push_back(vel(i));
				NodeId.push_back(n_id);
			}
		}

		void JointTraits::push( int n_id, const int &length)
		{
			for (int i=0; i<length; i++)
			{
				data.push_back(0.0);
				NodeId.push_back(n_id);
			}
		}

		Vector JointTraits::subvector(int n_id)
		{
			Vector vel;
			int in = -1;
			int n = 0;
			for(unsigned int i=0; i<data.size(); i++)
			{
				if(NodeId[i] == n_id)
				{
					if(in==-1) in=i;
					else n++;
				}
			}
			return data.subVector(in,in+n);
		}

		void JointTraits::setSubvector(int n_id, const Vector & v)
		{
			for(unsigned int i=0; i<data.size(); i++)
			{
				if(NodeId[i] == n_id)
				{
					data.setSubvector(i,v);
					break;
				}
			}
		}

// ----------------------------- PATH_CHAIN --------------------------------------
	
		void pathChain::pushChain(iKinChain &ch, const int &nodeid, bool root)
		{
			if (root)	this->H0 = ch.getH0();
			int nlink = this->getN();
			for(unsigned int i=0; i<ch.getN(); i++)
			{
				this->pushLink(ch[i]);
				if (ch.isLinkBlocked(i)) this->blockLink(i+nlink,ch.getAng(i));
				else					 this->setAng(i+nlink,ch.getAng(i));
				NodeId.push_back(nodeid);
			}
		};

		void pathChain::pushChain(iKinChain *ch, const int &nodeid, bool root)
		{
			if (root)	this->H0 = ch->getH0();
			
			int nlink = this->getN();
			for(unsigned int i=0; i<ch->getN(); i++)
			{
				this->pushLink((*ch)[i]);
				if (ch->isLinkBlocked(i)) {this->blockLink(i+nlink,ch->getAng(i));}
				else					 {cout << this->setAng(i+nlink,ch->getAng(i)) << endl;}
				NodeId.push_back(nodeid);
				//cout << "angolo " << i << " " << ch->getAng(i) <<  " " << this->getAng(i) << endl;
			}
			cout << endl;
			//cout << "originali: " << ch->getAng().toString() << endl;
			//cout << "inseriti: " << this->getAng().toString() << endl;
		}
/*		void pathChain::pushChain(iKinChain &ch, bool root)
		{
			if (root)	this->H0 = ch.getH0();
			int nlink = this->getN();
			for(unsigned int i=0; i<ch.getN(); i++)
			{
				this->pushLink(ch[i]);
				if (ch.isLinkBlocked(i)) this->blockLink(i+nlink,ch.getAng(i));
				else					 this->setAng(i+nlink,ch.getAng(i));
			}
			//cout << endl;
			//cout << "originali: " << ch.getAng().toString() << endl;
			//cout << "inseriti: " << this->getAng().toString() << endl;
		}

		void pathChain::pushChain(iKinChain *ch, bool root)
		{
			if (root)	this->H0 = ch->getH0();
			
			int nlink = this->getN();
			for(unsigned int i=0; i<ch->getN(); i++)
			{
				this->pushLink((*ch)[i]);
				if (ch->isLinkBlocked(i)) {this->blockLink(i+nlink,ch->getAng(i));}
				else					 {cout << this->setAng(i+nlink,ch->getAng(i)) << endl;}
				//cout << "angolo " << i << " " << ch->getAng(i) <<  " " << this->getAng(i) << endl;
			}
			cout << endl;
			//cout << "originali: " << ch->getAng().toString() << endl;
			//cout << "inseriti: " << this->getAng().toString() << endl;
		}
*/
		Matrix pathChain::getRelativeH(const int & nodeId)
		{
			int n = 0;
			int nlink = 0;
			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if(*it == nodeId)	n++;
				else if (n == 0)	nlink++;
				else				break;
			}

			//cout << "nlink: " << nlink+n << endl;
			return getH(nlink+n-1,true);
		}
		Matrix pathChain::getRelativeH(const int & nodeId, const int &jointNo)
		{
			int n = 0;
			int nlink = 0;
			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if(*it == nodeId && n<=jointNo)	n++;
				else if (n == 0)	nlink++;
				else				break;
			}

			cout << "nlink: " << nlink+n << endl;
			return getH(nlink+n-1,true);
		}

		void pathChain::getRelativeChain(const int &nodeId,iKinChain & ch)
		{
			ch.setH0(this->getH0());
			int i = 0;
			int n = 0;

			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if (*it == nodeId || n == 0)
				{
					ch.pushLink((*this)[i]);
					if (this->isLinkBlocked(i)) {ch.blockLink(i,this->getAng(i));}
					if (*it == nodeId)	n++;
					i++;
				}
				else
				{
					cout << "nlink: " << i-1 << endl;
					return;
				}
			}
		}

		void pathChain::splitChain(const int &nodeId, iKinChain & chHead, iKinChain &chTail)
		{
			chHead.setH0(this->getH0());
			chTail.setH0(this->getH0().eye());
			int i = 0;
			int n = 0;

			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if (*it == nodeId || n == 0)
				{
					chHead.pushLink((*this)[i]);
					if (this->isLinkBlocked(i)) {chHead.blockLink(i,this->getAng(i));}
					if (*it == nodeId)	n++;
				}
				else
				{
					chTail.pushLink((*this)[i]);
					if (this->isLinkBlocked(i)) {chTail.blockLink(i-chHead.getN(),this->getAng(i));}
				}
				i++;
			}
		}

		Vector pathChain::getRelativeAng(const int &nodeId)
		{
			int i = 0;
			int n = 0;
			Vector ang;

			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if (*it == nodeId || n == 0)
				{
					if(*it == nodeId)
					{
						ang.push_back(this->getAng(i+n));
						n++;
					}
					else i++;					
				}
				else break;
			}
			
			return ang;			
		}
		Bottle pathChain::writeRelativeAng(const int &nodeId, bool DEG)
		{
			int i = 0;
			int n = 0;
			Bottle ang;

			for ( vector<int>::iterator it = NodeId.begin(); it != NodeId.end(); it++)
			{
				if (*it == nodeId || n == 0)
				{
					if(*it == nodeId)
					{
						if (DEG) ang.addDouble(this->getAng(i+n)*CTRL_RAD2DEG);
						else	 ang.addDouble(this->getAng(i+n));
						n++;
					}
					else i++;					
				}
				else break;
			}
			
			return ang;			
		}
// ----------------------------- PMP_CHAIN ---------------------------------------
	// classe che contiene i parametri di rilassamento di un path
	
		pmpChain::pmpChain(const pmpChain &pc)
		{
			A = pc.A;
			K = pc.K;
			Km = pc.Km;
			Kint = pc.Kint;
			qref = pc.qref;
		}
		pmpChain &pmpChain::operator=(const pmpChain &pc)
		{
			if (this != &pc)
			{
				A = pc.A;
				K = pc.K;
				Km = pc.Km;
				Kint = pc.Kint;
				qref = pc.qref;
			}
			return *this;
		}
		void pmpChain::push_A(Matrix & _A)
		{
			if (_A.rows() == 0) return;
			for(int i=0;i<_A.rows(); i++) A.push_back(_A(i,i));
		}

		void pmpChain::push_Kint(const Matrix & _Kint)
		{
			if (_Kint.rows() == 0) return;
			Vector v;
			for(int i=0;i<_Kint.rows(); i++) v.push_back(_Kint(i,i));
			Kint.push_back(v);
		}

		void pmpChain::push_Kint(const Vector & _Kint)
		{
			if (_Kint.size() == 0) return;
			Kint.push_back(_Kint);
		}

		void pmpChain::push_qref(const Vector & _qref)
		{
			if (_qref.size() == 0) return;
			qref.push_back(_qref);
		}
		void pmpChain::set_K(const Matrix &_K)
		{
			K.resize(3,3);
			K = _K;
		}
		void pmpChain::set_Km(const Matrix &_K)
		{
			Km.resize(3,3);
			Km = _K;
		}

#endif