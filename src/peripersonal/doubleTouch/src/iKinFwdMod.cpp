/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <stdio.h>
#include <sstream>

#include <gsl/gsl_math.h>

#include "iKinFwdMod.h"

#define IKINIPOPT_SHOULDER_MAXABDUCTION     (100.0*CTRL_DEG2RAD)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/************************************************************************/
iKinLinkFather::iKinLinkFather(double _A, double _D, double _Alpha, double _Offset,
                   double _Min, double _Max): zeros1x1(zeros(1,1)), zeros1(zeros(1))
{
    A     =_A;
    D     =_D;
    Alpha =_Alpha;
    Offset=_Offset;

    Min=_Min;
    Max=_Max;
    Ang=Min;

    c_alpha=cos(Alpha);
    s_alpha=sin(Alpha);

    blocked    =false;
    cumulative =false;
    constrained=true;
    verbose    =0;

    H.resize(4,4);
    H.zero();
    DnH =H;
    cumH=H;
    cumH.eye();

    H(2,1)=s_alpha;
    H(2,2)=c_alpha;
    H(2,3)=D;
    H(3,3)=1.0;
}


/************************************************************************/
void iKinLinkFather::clone(const iKinLinkFather &l)
{
    A     =l.A;
    D     =l.D;
    Alpha =l.Alpha;
    Offset=l.Offset;

    c_alpha=l.c_alpha;
    s_alpha=l.s_alpha;

    Ang=l.Ang;
    Min=l.Min;
    Max=l.Max;

    blocked    =l.blocked;
    cumulative =l.cumulative;
    constrained=l.constrained;
    verbose    =l.verbose;

    H   =l.H;
    cumH=l.cumH;
    DnH =l.DnH;
}


/************************************************************************/
iKinLinkFather::iKinLinkFather(const iKinLinkFather &l)
{
    clone(l);
}


/************************************************************************/
iKinLinkFather &iKinLinkFather::operator=(const iKinLinkFather &l)
{
    clone(l);

    return *this;
}


/************************************************************************/
void iKinLinkFather::setMin(const double _Min)
{
    Min=_Min;

    if (Ang<Min)
        Ang=Min;
}


/************************************************************************/
void iKinLinkFather::setMax(const double _Max)
{
    Max=_Max;

    if (Ang>Max)
        Ang=Max;
}


/************************************************************************/
void iKinLinkFather::setD(const double _D)
{
    H(2,3)=D=_D;
}


/************************************************************************/
void iKinLinkFather::setAlpha(const double _Alpha)
{
    Alpha=_Alpha;

    H(2,2)=c_alpha=cos(Alpha);
    H(2,1)=s_alpha=sin(Alpha);
}


/************************************************************************/
double iKinLinkFather::setAng(double _Ang)
{
    if (!blocked)
    {
        if (constrained)
            Ang=(_Ang<Min) ? Min : ((_Ang>Max) ? Max : _Ang);
        else
            Ang=_Ang;
    }
    else if (verbose)
        fprintf(stderr,"Attempt to set joint angle to %g while blocked\n",_Ang);

    return Ang;
}


/************************************************************************/
Matrix iKinLinkFather::getH(bool c_override)
{

    if (cumulative && !c_override)
        return cumH*H;
    else
        return H;
}

/************************************************************************/
Matrix iKinLinkFather::getH(double _Ang, bool c_override)
{
    setAng(_Ang);

    return getH(c_override);
}


/************************************************************************/
Matrix iKinLinkFather::getDnH(unsigned int n, bool c_override)
{
    if (n==0)
        return getH(c_override);
    else
    {
        double theta=Ang+Offset;
        double c_theta=cos(theta);
        double s_theta=sin(theta);

        int    C=(n>>1)&1 ? -1 : 1;

        if (n&1)
        {
            DnH(0,0)=-C*s_theta;
            DnH(0,1)=-C*c_theta*c_alpha;
            DnH(0,2)=C*c_theta*s_alpha;
            DnH(0,3)=-C*s_theta*A;
    
            DnH(1,0)=C*c_theta;
            DnH(1,1)=-C*s_theta*c_alpha;
            DnH(1,2)=C*s_theta*s_alpha;
            DnH(1,3)=C*c_theta*A;
        }
        else
        {
            DnH(0,0)=C*c_theta;
            DnH(0,1)=-C*s_theta*c_alpha;
            DnH(0,2)=C*s_theta*s_alpha;
            DnH(0,3)=C*c_theta*A;

            DnH(1,0)=C*s_theta;
            DnH(1,1)=C*c_theta*c_alpha;
            DnH(1,2)=-C*c_theta*s_alpha;
            DnH(1,3)=C*s_theta*A;
        }

        if (cumulative && !c_override)
            return cumH*DnH;
        else
            return DnH;
    }
}


/************************************************************************/
void iKinLinkFather::addCumH(const Matrix &_cumH)
{
    cumulative=true;
    cumH=_cumH;
}

/************************************************************************/
/************************************************************************/

Matrix iKinDirectLink::getH(bool c_override)
{
    double theta=Ang+Offset;
    double c_th=cos(theta);
    double s_th=sin(theta);

    H(0,0)= c_th;
    H(0,1)=-s_th*c_alpha;
    H(0,2)= s_th*s_alpha;
    H(0,3)= c_th*A;

    H(1,0)= s_th;
    H(1,1)= c_th*c_alpha;
    H(1,2)=-c_th*s_alpha;
    H(1,3)= s_th*A;

    if (cumulative && !c_override)
        return cumH*H;
    else
        return H;
}

/************************************************************************/
/************************************************************************/

Matrix iKinInvertedLink::getH(bool c_override)
{
    double theta=Ang+Offset;

    double c_th=cos(theta);
    double s_th=sin(theta);

    H(0,0)=  c_th;
    H(0,1)= -s_th;
    H(0,2)=     0;
    H(0,3)=     A;

    H(1,0)= c_alpha*s_th;
    H(1,1)= c_alpha*c_th;
    H(1,2)=     -s_alpha;
    H(1,3)=   -s_alpha*D;

    H(2,0)= s_alpha*s_th;
    H(2,1)= s_alpha*c_th;
    H(2,2)=      c_alpha;
    H(2,3)=    c_alpha*D;

    if (cumulative && !c_override) 
        return cumH*H;
    else
        return H;
}

/************************************************************************/
Matrix iKinInvertedLink::getDnH(unsigned int n, bool c_override)
{
    if (n==0)
        return getH(c_override);
    else
    {
        double theta=Ang+Offset;
        double c_th=cos(theta);
        double s_th=sin(theta);

        int    C=(n>>1)&1 ? -1 : 1;

        if (n&1)
        {
            DnH(0,0)= -C*s_th;
            DnH(0,1)= -C*c_th;
            DnH(0,2)=       0;
            DnH(0,3)=       0;
    
            DnH(1,0)= C*c_th*c_alpha;
            DnH(1,1)=-C*s_th*c_alpha;
            DnH(1,2)=              0;
            DnH(1,3)=              0;

            DnH(2,0)= C*c_th*s_alpha;
            DnH(2,1)=-C*s_th*s_alpha;
            DnH(2,2)=              0;
            DnH(2,3)=              0;
        }
        else
        {
            DnH(0,0)=  C*c_th;
            DnH(0,1)= -C*s_th;
            DnH(0,2)=       0;
            DnH(0,3)=       0;

            DnH(1,0)= C*c_alpha*s_th;
            DnH(1,1)= C*c_alpha*c_th;
            DnH(1,2)=              0;
            DnH(1,3)=              0;

            DnH(2,0)= C*s_alpha*s_th;
            DnH(2,1)= C*s_alpha*c_th;
            DnH(2,2)=              0;
            DnH(2,3)=              0;
        }

        if (cumulative && !c_override)
            return cumH*DnH;
        else
            return DnH;
    }
}

/************************************************************************/
/************************************************************************/

iKinFixedRTLink::iKinFixedRTLink(Matrix _H) : iKinLinkFather(0,0,0,0,0,0)
{
    Ang=0;
    blocked = true;
    H.zero();
    H=_H;
};

/************************************************************************/
Matrix iKinFixedRTLink::getH(bool c_override)
{
    if (cumulative && !c_override)
        return cumH*H;
    else
        return H;
}

/************************************************************************/
Matrix iKinFixedRTLink::getDnH(unsigned int n, bool c_override)
{
    if (cumulative && !c_override)
        return cumH*DnH;
    else
        return DnH;
}

/************************************************************************/
bool iKinFixedRTLink::setH(Matrix _H)
{
    H = _H;
    return true;
}

/************************************************************************/
/************************************************************************/
iKinChainMod::iKinChainMod()
{
    N=DOF=verbose=0;
    H0=HN=eye(4,4);
}


/************************************************************************/
void iKinChainMod::clone(const iKinChainMod &c)
{
    N         =c.N;
    DOF       =c.DOF;
    H0        =c.H0;
    HN        =c.HN;
    curr_q    =c.curr_q;    
    verbose   =c.verbose;
    hess_J    =c.hess_J;
    hess_Jlnk =c.hess_Jlnk;

    allList.assign(c.allList.begin(),c.allList.end());
    quickList.assign(c.quickList.begin(),c.quickList.end());
    hash.assign(c.hash.begin(),c.hash.end());
    hash_dof.assign(c.hash_dof.begin(),c.hash_dof.end());
}


/************************************************************************/
iKinChainMod::iKinChainMod(const iKinChainMod &c)
{
    clone(c);
}


/************************************************************************/
iKinChainMod &iKinChainMod::operator=(const iKinChainMod &c)
{
    clone(c);

    return *this;
}


/************************************************************************/
bool iKinChainMod::addLink(const unsigned int i, iKinLinkFather &l)
{
    if (i<=N)
    {
        allList.insert(allList.begin()+i,&l);
        N=allList.size();

        build();

        return true;
    }
    else
    {
        if (verbose)
            fprintf(stderr,"addLink() failed due to out of range index: %d>%d\n",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChainMod::rmLink(const unsigned int i)
{
    if (i<N)
    {
        allList.erase(allList.begin()+i);
        N=allList.size();

        build();

        return true;
    }
    else
    {
        if (verbose)
            fprintf(stderr,"rmLink() failed due to out of range index: %d>=%d\n",i,N);

        return false;
    }
}


/************************************************************************/
void iKinChainMod::pushLink(iKinLinkFather &l)
{
    allList.push_back(&l);
    N=allList.size();

    build();
}


/************************************************************************/
void iKinChainMod::clear()
{
    allList.clear();
    quickList.clear();
    hash.clear();
    hash_dof.clear();

    N=DOF=0;
}


/************************************************************************/
iKinChainMod &iKinChainMod::operator<<(iKinLinkFather &l)
{
    pushLink(l);

    return *this;
}


/************************************************************************/
void iKinChainMod::popLink()
{
    allList.pop_back();
    N=allList.size();

    build();
}


/************************************************************************/
iKinChainMod &iKinChainMod::operator--(int)
{
    popLink();

    return *this;
}


/************************************************************************/
bool iKinChainMod::blockLink(const unsigned int i, double Ang)
{
    if (i<N)
    {
        allList[i]->block(Ang);
        build();

        return true;
    }
    else
    {
        if (verbose)
            fprintf(stderr,"blockLink() failed due to out of range index: %d>=%d\n",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChainMod::setBlockingValue(const unsigned int i, double Ang)
{
    if (i<N)
    {
        if (allList[i]->isBlocked() && (Ang!=allList[i]->getAng()))
        {
            allList[i]->blocked=false; // remove the block temporarly
            allList[i]->block(Ang);    // update the blocked link

            // update the cumulative link which follows in the chain
            if (i<N-1)
            {
                Matrix H=eye(4,4);
                int j;

                for (j=i-1; j>=0; j--)
                    if (!allList[j]->isBlocked())
                        break;
                
                for (++j; j<=(int)i; j++)
                    H*=allList[j]->getH(true);
    
                for (; j<(int)N && !allList[j]->isCumulative(); j++)
                    H*=allList[j]->getH(true);
    
                allList[j]->addCumH(H);
            } 

            return true;
        }
        else
        {
            if (verbose)
                fprintf(stderr,"setBlockingValue() failed since the %dth link was not already blocked\n",i);

            return false;
        }
    }
    else
    {
        if (verbose)
            fprintf(stderr,"setBlockingValue() failed due to out of range index: %d>=%d\n",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChainMod::releaseLink(const unsigned int i)
{
    if (i<N)
    {
        allList[i]->release();
        build();

        return true;
    }
    else
    {    
        if (verbose)
            fprintf(stderr,"releaseLink() failed due to out of range index: %d>=%d\n",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChainMod::isLinkBlocked(const unsigned int i)
{
    if (i<N)
        return allList[i]->isBlocked();
    else
    {    
        if (verbose)
            fprintf(stderr,"isLinkBlocked() failed due to out of range index: %d>=%d\n",i,N);

        return false;
    }
}


/************************************************************************/
void iKinChainMod::setAllConstraints(bool _constrained)
{
    for (unsigned int i=0; i<N; i++)
        allList[i]->setConstraint(_constrained);
}


/************************************************************************/
void iKinChainMod::setAllLinkVerbosity(unsigned int _verbose)
{
    for (unsigned int i=0; i<N; i++)
        allList[i]->setVerbosity(_verbose);
}


/************************************************************************/
void iKinChainMod::build()
{
    quickList.clear();
    hash.clear();
    hash_dof.clear();
    DOF=0;

    Matrix H=eye(4,4);
    bool cumulOn=false;

    for (unsigned int i=0; i<N; i++)
    {
        allList[i]->rmCumH();

        if (allList[i]->isBlocked())
        {
            if (i==N-1)
            {    
                allList[i]->addCumH(H);
                quickList.push_back(allList[i]);
            }
            else
            {
                H*=allList[i]->getH();
                cumulOn=true;
            }
        }
        else
        {
            if (cumulOn) {
                allList[i]->addCumH(H);
            }

            DOF++;
            quickList.push_back(allList[i]);
            hash_dof.push_back(quickList.size()-1);
            hash.push_back(i);

            H.eye();
            cumulOn=false;
        }
    }

    if (DOF>0)
        curr_q.resize(DOF,0);
}


/************************************************************************/
bool iKinChainMod::setH0(const Matrix &_H0)
{
    if ((_H0.rows()==4) && (_H0.cols()==4))
    {
        H0=_H0;
        return true;
    }
    else
    {
        if (verbose)
            fprintf(stderr,"Attempt to reference a wrong matrix H0 (not 4x4)\n");

        return false;
    }
}


/************************************************************************/
bool iKinChainMod::setHN(const Matrix &_HN)
{
    if ((_HN.rows()==4) && (_HN.cols()==4))
    {
        HN=_HN;
        return true;
    }
    else
    {
        if (verbose)
            fprintf(stderr,"Attempt to reference a wrong matrix HN (not 4x4)\n");

        return false;
    }
}


/************************************************************************/
Vector iKinChainMod::setAng(const Vector &q)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"setAng() failed since DOF==0\n");

        return Vector(0);
    }

    size_t sz=std::min(q.length(),(size_t)DOF);
    for (size_t i=0; i<sz; i++)
        curr_q[i]=quickList[hash_dof[i]]->setAng(q[i]);

    return curr_q;
}


/************************************************************************/
Vector iKinChainMod::getAng()
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"getAng() failed since DOF==0\n");

        return Vector(0);
    }

    for (unsigned int i=0; i<DOF; i++)
        curr_q[i]=quickList[hash_dof[i]]->getAng();

    return curr_q;
}


/************************************************************************/
double iKinChainMod::setAng(const unsigned int i, double _Ang)
{
    double res=0.0;

    if (i<N)
    {
        if (allList[i]->isBlocked())
        {
            setBlockingValue(i,_Ang);
            res=allList[i]->getAng();
        }
        else
            res=allList[i]->setAng(_Ang);
    }
    else if (verbose)
        fprintf(stderr,"setAng() failed due to out of range index: %d>=%d\n",i,N);

    return res;
}


/************************************************************************/
double iKinChainMod::getAng(const unsigned int i)
{
    double res=0.0;

    if (i<N)
        res=allList[i]->getAng();
    else if (verbose)
        fprintf(stderr,"getAng() failed due to out of range index: %d>=%d\n",i,N);

    return res;
}

/************************************************************************/
Matrix iKinChainMod::getH(const unsigned int i, const bool allLink)
{
    Matrix H=H0;
    unsigned int _i,n;
    deque<iKinLinkFather*> *l;
    bool cumulHN=false;
    bool c_override;

    if (allLink)
    {
        n=N;
        l=&allList;
        c_override=true;

        _i=i;
        if (_i>=N-1)
            cumulHN=true;
    }
    else
    {
        n=DOF;
        l=&quickList;
        c_override=false;

        if (i==DOF)
            _i=quickList.size();
        else
            _i=i;

        if (hash[_i]>=N-1)
            cumulHN=true;
    }

    if (i<n)
    {
        for (unsigned int j=0; j<=_i; j++)
            H*=((*l)[j]->getH(c_override));

        if (cumulHN)
            H*=HN;
    }
    else if (verbose)
        fprintf(stderr,"getH() failed due to out of range index: %d>=%d\n",i,n);

    return H;
}


/************************************************************************/
Matrix iKinChainMod::getH()
{
    // may be different from DOF since one blocked link may lie
    // at the end of the chain.
    unsigned int n=quickList.size();
    Matrix H=H0;

    for (unsigned int i=0; i<n; i++)
        H*=quickList[i]->getH();

    return H*HN;
}


/************************************************************************/
Matrix iKinChainMod::getH(const Vector &q)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"getH() failed since DOF==0\n");
    
        return Matrix(0,0);
    }

    setAng(q);
    return getH();
}


/************************************************************************/
Vector iKinChainMod::Pose(const unsigned int i, const bool axisRep)
{
    Matrix H=getH(i,true);
    Vector v;

    if (i<N)
    {
        if (axisRep)
        {
            v.resize(7);
            Vector r=dcm2axis(H,verbose);
            v[0]=H(0,3);
            v[1]=H(1,3);
            v[2]=H(2,3);
            v[3]=r[0];
            v[4]=r[1];
            v[5]=r[2];
            v[6]=r[3];
        }
        else
        {
            v.resize(6);
            Vector r=RotAng(H);
            v[0]=H(0,3);
            v[1]=H(1,3);
            v[2]=H(2,3);
            v[3]=r[0];
            v[4]=r[1];
            v[5]=r[2];
        }
    }
    else if (verbose)
        fprintf(stderr,"Pose() failed due to out of range index: %d>=%d\n",i,N);

    return v;
}


/************************************************************************/
Vector iKinChainMod::Position(const unsigned int i)
{
    if (i>=N)
    {
        if (verbose)
            fprintf(stderr,"Position() failed due to out of range index: %d>=%d\n",i,N);

        return Vector(0);
    }

    return getH(i,true).subcol(0,3,3);
}


/************************************************************************/
Vector iKinChainMod::EndEffPose(const bool axisRep)
{
    Matrix H=getH();
    Vector v;

    if (axisRep)
    {
        v.resize(7);
        Vector r=dcm2axis(H,verbose);
        v[0]=H(0,3);
        v[1]=H(1,3);
        v[2]=H(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
        v[6]=r[3];
    }
    else
    {
        v.resize(6);
        Vector r=RotAng(H);
        v[0]=H(0,3);
        v[1]=H(1,3);
        v[2]=H(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
    }

    return v;
}


/************************************************************************/
Vector iKinChainMod::EndEffPose(const Vector &q, const bool axisRep)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"EndEffPose() failed since DOF==0\n");
    
        return Vector(0);
    }

    setAng(q);
    return EndEffPose(axisRep);
}


/************************************************************************/
Vector iKinChainMod::EndEffPosition()
{
    return getH().subcol(0,3,3);
}


/************************************************************************/
Vector iKinChainMod::EndEffPosition(const Vector &q)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"EndEffPosition() failed since DOF==0\n");
    
        return Vector(0);
    }

    setAng(q);
    return EndEffPosition();
}


/************************************************************************/
Matrix iKinChainMod::AnaJacobian(const unsigned int i, unsigned int col)
{
    if (i>=N)
    {
        if (verbose)
            fprintf(stderr,"AnaJacobian() failed due to out of range index: %d>=%d\n",i,N);

        return Matrix(0,0);
    }

    col=col>3 ? 3 : col;

    Matrix J(6,i+1);
    Matrix H,dH,_H;
    Vector dr;

    for (unsigned int j=0; j<=i; j++)
    {
        H=dH=H0;

        for (unsigned int k=0; k<=i; k++)
        {
            _H=allList[k]->getH(true);
            H*=_H;

            if (j==k)
                dH*=allList[k]->getDnH(1,true);
            else
                dH*=_H;
        }

        if (i>=N-1)
        {
            H*=HN;
            dH*=HN;
        }

        dr=dRotAng(H,dH);

        J(0,j)=dH(0,col);
        J(1,j)=dH(1,col);
        J(2,j)=dH(2,col);
        J(3,j)=dr[0];
        J(4,j)=dr[1];
        J(5,j)=dr[2];
    }

    return J;
}


/************************************************************************/
Matrix iKinChainMod::AnaJacobian(unsigned int col)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"AnaJacobian() failed since DOF==0\n");

        return Matrix(0,0);
    }

    col=col>3 ? 3 : col;

    // may be different from DOF since one blocked link may lie
    // at the end of the chain.
    unsigned int n=quickList.size();
    Matrix J(6,DOF);
    Matrix H,dH,_H;
    Vector dr;

    for (unsigned int i=0; i<DOF; i++)
    {
        H=dH=H0;

        for (unsigned int j=0; j<n; j++)
        {
            _H=quickList[j]->getH();
            H*=_H;

            if (hash_dof[i]==j)
                dH*=quickList[j]->getDnH();
            else
                dH*=_H;
        }

        H*=HN;
        dH*=HN;
        dr=dRotAng(H,dH);

        J(0,i)=dH(0,col);
        J(1,i)=dH(1,col);
        J(2,i)=dH(2,col);
        J(3,i)=dr[0];
        J(4,i)=dr[1];
        J(5,i)=dr[2];
    }

    return J;
}


/************************************************************************/
Matrix iKinChainMod::AnaJacobian(const Vector &q, unsigned int col)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"AnaJacobian() failed since DOF==0\n");
    
        return Matrix(0,0);
    }

    setAng(q);
    return AnaJacobian(col);
}


/************************************************************************/
Matrix iKinChainMod::GeoJacobian(const unsigned int i)
{
    if (i>=N)
    {
        if (verbose)
            fprintf(stderr,"GeoJacobian() failed due to out of range index: %d>=%d\n",i,N);

        return Matrix(0,0);
    }

    Matrix J(6,i+1);
    Matrix PN,Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int j=0; j<=i; j++)
        intH.push_back(intH[j]*allList[j]->getH(true));

    PN=intH[i+1];
    if (i>=N-1)
        PN=PN*HN;

    for (unsigned int j=0; j<=i; j++)
    {
        Z=intH[j];
        w=cross(Z,2,PN-Z,3,verbose);

        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }

    return J;
}


/************************************************************************/
Matrix iKinChainMod::GeoJacobian()
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"GeoJacobian() failed since DOF==0\n");

        return Matrix(0,0);
    }

    Matrix J(6,DOF);
    Matrix PN,Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int i=0; i<N; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    PN=intH[N]*HN;

    for (unsigned int i=0; i<DOF; i++)
    {
        unsigned int j=hash[i];
        if(iKinInvertedLink* ikil = dynamic_cast<iKinInvertedLink*>(allList[i])) //Type of the object is iKinInvertedLink
        {
            Z=intH[j+1];
        }
        else
            Z=intH[j];

        w=cross(Z,2,PN-Z,3,verbose);

        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }

    return J;
}


/************************************************************************/
Matrix iKinChainMod::GeoJacobian(const Vector &q)
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"GeoJacobian() failed since DOF==0\n");
    
        return Matrix(0,0);
    }

    setAng(q);
    return GeoJacobian();
}


/************************************************************************/
Vector iKinChainMod::Hessian_ij(const unsigned int i, const unsigned int j)
{
    prepareForHessian();
    return fastHessian_ij(i,j);
}


/************************************************************************/
void iKinChainMod::prepareForHessian()
{
    if (DOF==0)
    {
        if (verbose)
            fprintf(stderr,"prepareForHessian() failed since DOF==0\n");

        return;
    }

    hess_J=GeoJacobian();
}


/************************************************************************/
Vector iKinChainMod::fastHessian_ij(const unsigned int i, const unsigned int j)
{
    if ((i>=DOF) || (j>=DOF))
    {
        if (verbose)
            fprintf(stderr,"fastHessian_ij() failed due to out of range index: %d>=%d || %d>=%d\n",i,DOF,j,DOF);

        return Vector(0);
    }

    // ref. E.D. Pohl, H. Lipkin, "A New Method of Robotic Motion Control Near Singularities",
    // Advanced Robotics, 1991
    Vector h(6,0.0);
    if(i<j)
    {
        //h.setSubvector(0,cross(hess_Jo,i,hess_Jl,j));
        h[0] = hess_J(4,i)*hess_J(2,j) - hess_J(5,i)*hess_J(1,j);
        h[1] = hess_J(5,i)*hess_J(0,j) - hess_J(3,i)*hess_J(2,j);
        h[2] = hess_J(3,i)*hess_J(1,j) - hess_J(4,i)*hess_J(0,j);
        //h.setSubvector(3,cross(hess_Jo,i,hess_Jo,j));
        h(3) = hess_J(4,i)*hess_J(5,j)-hess_J(5,i)*hess_J(4,j);
        h(4) = hess_J(5,i)*hess_J(3,j)-hess_J(3,i)*hess_J(5,j);
        h(5) = hess_J(3,i)*hess_J(4,j)-hess_J(4,i)*hess_J(3,j);
    }
    else
    {
        //h.setSubvector(0, cross(Jo,j,Jl,i));
        h[0] = hess_J(4,j)*hess_J(2,i) - hess_J(5,j)*hess_J(1,i);
        h[1] = hess_J(5,j)*hess_J(0,i) - hess_J(3,j)*hess_J(2,i);
        h[2] = hess_J(3,j)*hess_J(1,i) - hess_J(4,j)*hess_J(0,i);
        h[3]=h[4]=h[5]=0.0;
    }

    return h;
}


/************************************************************************/
Vector iKinChainMod::Hessian_ij(const unsigned int lnk, const unsigned int i,
                             const unsigned int j)
{
    prepareForHessian(lnk);
    return fastHessian_ij(lnk,i,j);
}


/************************************************************************/
void iKinChainMod::prepareForHessian(const unsigned int lnk)
{
    if (lnk>=N)
    {
        if (verbose)
            fprintf(stderr,"prepareForHessian() failed due to out of range index: %d>=%d\n",lnk,N);

        return;
    }

    hess_Jlnk=GeoJacobian(lnk);
}


/************************************************************************/
Vector iKinChainMod::fastHessian_ij(const unsigned int lnk, const unsigned int i,
                                 const unsigned int j)
{
    if ((i>=lnk) || (j>=lnk))
    {
        if (verbose)
            fprintf(stderr,"fastHessian_ij() failed due to out of range index: %d>=%d || %d>=%d\n",i,lnk,j,lnk);

        return Vector(0);
    }

    Vector h(6,0.0);
    if (i<j)
    {
        //h.setSubvector(0,cross(hess_Jlnko,i,hess_Jlnkl,j));
        h[0] = hess_Jlnk(4,i)*hess_Jlnk(2,j) - hess_Jlnk(5,i)*hess_Jlnk(1,j);
        h[1] = hess_Jlnk(5,i)*hess_Jlnk(0,j) - hess_Jlnk(3,i)*hess_Jlnk(2,j);
        h[2] = hess_Jlnk(3,i)*hess_Jlnk(1,j) - hess_Jlnk(4,i)*hess_Jlnk(0,j);
        //h.setSubvector(3,cross(hess_Jlnko,i,hess_Jlnko,j));
        h[3] = hess_Jlnk(4,i)*hess_Jlnk(5,j) - hess_Jlnk(5,i)*hess_Jlnk(4,j);
        h[4] = hess_Jlnk(5,i)*hess_Jlnk(3,j) - hess_Jlnk(3,i)*hess_Jlnk(5,j);
        h[5] = hess_Jlnk(3,i)*hess_Jlnk(4,j) - hess_Jlnk(4,i)*hess_Jlnk(3,j);
    }
    else
    {
        //h.setSubvector(0,cross(hess_Jlnko,j,hess_Jlnkl,i));
        h[0] = hess_Jlnk(4,j)*hess_Jlnk(2,i) - hess_Jlnk(5,j)*hess_Jlnk(1,i);
        h[1] = hess_Jlnk(5,j)*hess_Jlnk(0,i) - hess_Jlnk(3,j)*hess_Jlnk(2,i);
        h[2] = hess_Jlnk(3,j)*hess_Jlnk(1,i) - hess_Jlnk(4,j)*hess_Jlnk(0,i);
        h[3]=h[4]=h[5]=0.0;
    }

    return h;
}


/************************************************************************/
Matrix iKinChainMod::DJacobian(const Vector &dq)
{
    Matrix J = GeoJacobian();
    Matrix dJ(6,DOF);
    dJ.zero();
    double dqj, dqi, a, b, c;
    for (unsigned int i=0; i<DOF; i++)  // i: col
    {
        for (unsigned int j=0; j<=i; j++)  // j: row
        {
            dqj = dq(j);
            
            a = J(4,j)*J(2,i) - J(5,j)*J(1,i);
            b = J(5,j)*J(0,i) - J(3,j)*J(2,i);
            c = J(3,j)*J(1,i) - J(4,j)*J(0,i);
            dJ(0,i) += dqj*a;
            dJ(1,i) += dqj*b;
            dJ(2,i) += dqj*c;
            dJ(3,i) += dqj*(J(4,j)*J(5,i)-J(5,j)*J(4,i));
            dJ(4,i) += dqj*(J(5,j)*J(3,i)-J(3,j)*J(5,i));
            dJ(5,i) += dqj*(J(3,j)*J(4,i)-J(4,j)*J(3,i));
            
            if(i!=j)
            {
                dqi = dq(i);
                dJ(0,j) += dqi*a;
                dJ(1,j) += dqi*b;
                dJ(2,j) += dqi*c;
            }
        }
    }
    return dJ;

    /* OLD IMPLEMENTATION (SLOWER, BUT CLEARER)
    prepareForHessian();
    Vector tmp(6,0.0);
    for (unsigned int i=0; i<DOF; i++)
    {
        for (unsigned int j=0; j<DOF; j++)
            tmp+=fastHessian_ij(j,i)*dq(j);

        dJ.setCol(i,tmp);
        tmp.zero();
    }*/
}


/************************************************************************/
Matrix iKinChainMod::DJacobian(const unsigned int lnk, const Vector &dq)
{
    Matrix J = GeoJacobian(lnk);
    Matrix dJ(6,lnk-1);
    dJ.zero();
    double dqj, dqi, a, b, c;
    for (unsigned int i=0; i<lnk; i++)  // i: col
    {
        for (unsigned int j=0; j<=i; j++)  // j: row
        {
            dqj = dq(j);
            
            a = J(4,j)*J(2,i) - J(5,j)*J(1,i);
            b = J(5,j)*J(0,i) - J(3,j)*J(2,i);
            c = J(3,j)*J(1,i) - J(4,j)*J(0,i);
            dJ(0,i) += dqj*a;
            dJ(1,i) += dqj*b;
            dJ(2,i) += dqj*c;
            dJ(3,i) += dqj*(J(4,j)*J(5,i)-J(5,j)*J(4,i));
            dJ(4,i) += dqj*(J(5,j)*J(3,i)-J(3,j)*J(5,i));
            dJ(5,i) += dqj*(J(3,j)*J(4,i)-J(4,j)*J(3,i));
            
            if(i!=j)
            {
                dqi = dq(i);
                dJ(0,j) += dqi*a;
                dJ(1,j) += dqi*b;
                dJ(2,j) += dqi*c;
            }
        }
    }
    return dJ;

    // OLD IMPLEMENTATION (SLOWER, BUT CLEARER)
    /*prepareForHessian(lnk);
    Vector tmp(6,0.0);
    for (unsigned int i=0; i<lnk; i++)
    {
        for (unsigned int j=0; j<lnk; j++)
            tmp+=fastHessian_ij(lnk,j,i)*dq(j);

        dJ.setCol(i,tmp);
        tmp.zero();
    }*/
}


/************************************************************************/
iKinChainMod::~iKinChainMod()
{
    dispose();
}


/************************************************************************/
void iKinChainMod::dispose()
{
    allList.clear();
    quickList.clear();
}

/************************************************************************/
/************************************************************************/
iKinLimbMod::iKinLimbMod()
{
    allocate("right");
}


/************************************************************************/
iKinLimbMod::iKinLimbMod(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iKinLimbMod::iKinLimbMod(const iKinLimbMod &limb)
{
    clone(limb);
}


/************************************************************************/
iKinLimbMod::iKinLimbMod(const Property &options)
{
    fromLinksProperties(options);
}


/************************************************************************/
void iKinLimbMod::pushLink(iKinLinkFather *pl)
{
    linkList.push_back(pl);
    pushLink(*pl);
}


/************************************************************************/
void iKinLimbMod::getMatrixFromProperties(Property &options, const string &tag, Matrix &H)
{
    if (Bottle *bH=options.find(tag.c_str()).asList())
    {
        int i=0;
        int j=0;

        H.zero();
        for (int cnt=0; (cnt<bH->size()) && (cnt<H.rows()*H.cols()); cnt++)
        {    
            H(i,j)=bH->get(cnt).asDouble();
            if (++j>=H.cols())
            {
                i++;
                j=0;
            }
        }
    }
}


/************************************************************************/
bool iKinLimbMod::fromLinksProperties(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    dispose();    

    type=opt.check("type",Value("right")).asString().c_str();

    getMatrixFromProperties(opt,"H0",H0);
    getMatrixFromProperties(opt,"HN",HN);

    int numLinks=opt.check("numLinks",Value(0)).asInt();
    if (numLinks==0)
    {
        fprintf(stderr,"Error: invalid number of links specified!\n");

        type="right";
        H0.eye();
        HN.eye();

        return false;
    }

    for (int i=0; i<numLinks; i++)
    {
        ostringstream link;
        link<<"link_"<<i;

        Bottle &bLink=opt.findGroup(link.str().c_str());
        if (bLink.isNull())
        {
            fprintf(stderr,"Error: %s is missing!\n",link.str().c_str());

            type="right";
            H0.eye();
            dispose();

            return false;
        }

        double A=bLink.check("A",Value(0.0)).asDouble();
        double D=bLink.check("D",Value(0.0)).asDouble();
        double alpha=CTRL_DEG2RAD*bLink.check("alpha",Value(0.0)).asDouble();
        double offset=CTRL_DEG2RAD*bLink.check("offset",Value(0.0)).asDouble();
        double min=CTRL_DEG2RAD*bLink.check("min",Value(-180.0)).asDouble();
        double max=CTRL_DEG2RAD*bLink.check("max",Value(180.0)).asDouble();

        pushLink(new iKinLinkFather(A,D,alpha,offset,min,max));

        if (bLink.check("blocked"))
            blockLink(i,CTRL_DEG2RAD*bLink.find("blocked").asDouble());
    }

    return configured=true;
}


/************************************************************************/
iKinLimbMod &iKinLimbMod::operator=(const iKinLimbMod &limb)
{
    dispose();
    clone(limb);

    return *this;
}


/************************************************************************/
iKinLimbMod::~iKinLimbMod()
{
    dispose();
}

/************************************************************************/
void iKinLimbMod::allocate(const string &_type)
{
    type=_type;
    configured=true;
}

/************************************************************************/
void iKinLimbMod::clone(const iKinLimbMod &limb)
{
    type=limb.type;
    H0=limb.H0;
    HN=limb.HN;

    for (unsigned int i=0; i<limb.getN(); i++)
        pushLink(new iKinLinkFather(*(limb.linkList[i])));

    configured=limb.configured;
}

/************************************************************************/
void iKinLimbMod::dispose()
{
    for (unsigned int i=0; i<linkList.size(); i++)
        if (linkList[i]!=NULL)
            delete linkList[i];

    linkList.clear();
    iKinChainMod::dispose();

    configured=false;
}



/************************************************************************/
/************************************************************************/
void iCubCustomLimb::allocate(const string &_type)
{
    iKinLimbMod::allocate(_type);

    Matrix H0 = eye(4);
    setH0(H0);

    pushLink(new iKinInvertedLink(       0.0,  -0.1373, -M_PI/2.0,            M_PI/2.0,  -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iKinInvertedLink(     0.015,      0.0, -M_PI/2.0,                 0.0, -106.0*CTRL_DEG2RAD,  -5.5*CTRL_DEG2RAD));
    pushLink(new iKinInvertedLink(    -0.015, -0.15228,  M_PI/2.0,  -75.0*CTRL_DEG2RAD,  -90.0*CTRL_DEG2RAD,  37.0*CTRL_DEG2RAD));
    pushLink(new iKinInvertedLink(       0.0,      0.0, -M_PI/2.0,            M_PI/2.0, -160.8*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iKinInvertedLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0,   -5.0*CTRL_DEG2RAD,  95.5*CTRL_DEG2RAD));

    Matrix L2R(4,4);
    L2R.zero();
    L2R.eye();

    L2R(0,0) =  -0.866;    L2R(0,2) =    0.5;    L2R(0,3) = -0.0031303;
    L2R(1,1) =      -1;   
    L2R(2,0) =     0.5;    L2R(2,2) =  0.866;    L2R(2,3) = -0.0116823;

    pushLink(new iKinFixedRTLink(L2R));

    // iKin bounds
    pushLink(new iKinDirectLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(       0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,                0.0, 160.8*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(    -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(     0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(       0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
    pushLink(new iKinDirectLink(    0.0625,    0.016,       0.0,                M_PI, -20.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
}

/************************************************************************/
Vector iCubCustomLimb::setAng(const Vector &q)
{
    Vector nq = iKinLimbMod::setAng(q);
    return nq;
}

/************************************************************************/
Vector iCubCustomLimb::setAng(const Vector &ql, const Vector &qr)
{
    Vector q(getDOF(),0.0);
    
    q[4]  = -ql[0];
    q[3]  = -ql[1];
    q[2]  = -ql[2];
    q[1]  = -ql[3];
    q[0]  = -ql[4];
   
    q[5]  = qr[0];
    q[6]  = qr[1];
    q[7]  = qr[2];
    q[8]  = qr[3];
    q[9]  = qr[4];
    q[10] = qr[5];
    q[11] = qr[6];

    Vector nq = iKinLimbMod::setAng(q);
    return nq;
}

/************************************************************************/
bool iCubCustomLimb::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limLeft =*lim[0];
    IControlLimits &limRight=*lim[1];

    unsigned int iLeft;
    unsigned int iRight;
    double min, max;

    for (iLeft=0; iLeft<5; iLeft++)
    {   
        if (!limLeft.getLimits(iLeft,&min,&max))
            return false;
        // printf("old lims %g %g \t", (*this)[4-iLeft].getMin()*CTRL_RAD2DEG, (*this)[4-iLeft].getMax()*CTRL_RAD2DEG);
        (*this)[4-iLeft].setMin(-CTRL_DEG2RAD*max);
        (*this)[4-iLeft].setMax(-CTRL_DEG2RAD*min);
        // printf("new lims %g %g \n", (*this)[4-iLeft].getMin()*CTRL_RAD2DEG, (*this)[4-iLeft].getMax()*CTRL_RAD2DEG);
    }

    for (iRight=0; iRight<getN()-iLeft-1; iRight++)
    {   
        if (!limRight.getLimits(iRight,&min,&max))
            return false;

        // printf("old lims %g %g \t", (*this)[iLeft+1+iRight].getMin()*CTRL_RAD2DEG, (*this)[iLeft+1+iRight].getMax()*CTRL_RAD2DEG);
        (*this)[iLeft+1+iRight].setMin(CTRL_DEG2RAD*min);
        (*this)[iLeft+1+iRight].setMax(CTRL_DEG2RAD*max);
        // printf("old lims %g %g \n", (*this)[iLeft+1+iRight].getMin()*CTRL_RAD2DEG, (*this)[iLeft+1+iRight].getMax()*CTRL_RAD2DEG);
    }

    return true;
}



/************************************************************************/
/************************************************************************/
void iCubShoulderConstrMod::clone(const iKinLinIneqConstr *obj)
{
    iKinLinIneqConstr::clone(obj);

    const iCubShoulderConstrMod *ptr=static_cast<const iCubShoulderConstrMod*>(obj);

    shou_m=ptr->shou_m;
    shou_n=ptr->shou_n;
    elb_m=ptr->elb_m;
    elb_n=ptr->elb_n;
    shoulder=ptr->shoulder;
    
    chain=ptr->chain;
}

/************************************************************************/
void iCubShoulderConstrMod::appendMatrixRow(yarp::sig::Matrix &dest,
                                         const yarp::sig::Vector &row)
{
    yarp::sig::Matrix tmp;

    // if dest is already filled with something
    if (dest.rows())
    {   
        // exit if lengths do not match     
        if (row.length()!=dest.cols())
            return;

        tmp.resize(dest.rows()+1,dest.cols());

        // copy the content of dest in temp
        for (int i=0; i<dest.rows(); i++)
            for (int j=0; j<dest.cols(); j++)
                tmp(i,j)=dest(i,j);

        // reassign dest
        dest=tmp;
    }
    else
        dest.resize(1,row.length());

    // append the last row
    for (int i=0; i<dest.cols(); i++)
        dest(dest.rows()-1,i)=row[i];
}

/************************************************************************/
void iCubShoulderConstrMod::appendVectorValue(yarp::sig::Vector &dest, double val)
{
    yarp::sig::Vector tmp(dest.length()+1);
    for (size_t i=0; i<dest.length(); i++)
        tmp[i]=dest[i];

    dest=tmp;
    dest[dest.length()-1]=val;
}

/************************************************************************/
iCubShoulderConstrMod::iCubShoulderConstrMod(iKinChainMod *_chain, char _shoulder) : iKinLinIneqConstr(), chain(_chain), shoulder(_shoulder)
{   
    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    joint1_0= 28.0*CTRL_DEG2RAD;
    joint1_1= 23.0*CTRL_DEG2RAD;
    joint2_0=-37.0*CTRL_DEG2RAD;
    joint2_1= 80.0*CTRL_DEG2RAD;
    shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    shou_n=joint1_0-shou_m*joint2_0;

    double joint3_0, joint3_1;
    double joint4_0, joint4_1;
    joint3_0= 85.0*CTRL_DEG2RAD;
    joint3_1=105.0*CTRL_DEG2RAD;
    joint4_0= 90.0*CTRL_DEG2RAD;
    joint4_1= 40.0*CTRL_DEG2RAD;
    elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
    elb_n=joint4_0-elb_m*joint3_0;
    printf("elb_n: %g\telb_m: %g\n",elb_n,elb_m);
    // printf("*******\nshou_m: %g \t shou_n: %g\telb_m: %g \t elb_n: %g\n*******\n", shou_m*CTRL_RAD2DEG,shou_n*CTRL_RAD2DEG,elb_m*CTRL_RAD2DEG,elb_n*CTRL_RAD2DEG);

    update(NULL);
}

/************************************************************************/
void iCubShoulderConstrMod::update(void*)
{
    // optimization won't use LinIneqConstr by default
    setActive(false);

    yarp::sig::Matrix _C;
    yarp::sig::Vector _lB;
    yarp::sig::Vector _uB;

    yarp::sig::Vector row(chain->getDOF());

    int sh=6; //first joint of the shoulder

    if (shoulder == 'r')
    {   
        // if shoulder's axes are controlled, constraint them
        if (!(*chain)[sh].isBlocked() && !(*chain)[sh+1].isBlocked() &&
            !(*chain)[sh+2].isBlocked())
        {
            // compute offset to shoulder's axes
            // given the blocked/release status of
            // previous link
            int offs=0;
            for (int i=0; i<sh; i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // constraints on the cables length
            row.zero();
            row[offs]=1.71; row[offs+1]=-1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-347.00*CTRL_DEG2RAD);
            appendVectorValue(_uB,upperBoundInf);

            row.zero();
            row[offs]=1.71; row[offs+1]=-1.71; row[offs+2]=-1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-366.57*CTRL_DEG2RAD);
            appendVectorValue(_uB,112.42*CTRL_DEG2RAD);

            row.zero();
            row[offs+1]=0.8; row[offs+2]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-66.600*CTRL_DEG2RAD);
            appendVectorValue(_uB,213.30*CTRL_DEG2RAD);

            // constraints to prevent arm from touching torso
            row.zero();
            row[offs+1]=1.0; row[offs+2]=-shou_m;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,shou_n/*-20*CTRL_DEG2RAD*/);
            appendVectorValue(_uB,upperBoundInf);

            // constraints to limit shoulder abduction
            row.zero();
            row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,IKINIPOPT_SHOULDER_MAXABDUCTION);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }

        // if elbow and pronosupination axes are controlled, constraint them
        if (!(*chain)[sh+3].isBlocked() && !(*chain)[sh+3+1].isBlocked())
        {
            // compute offset to elbow's axis
            // given the blocked/release status of
            // previous link
            int offs=0;
            for (int i=0; i<(sh+3); i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // constraints to prevent the forearm from hitting the arm
            row.zero();
            row[offs]=-elb_m; row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,elb_n);

            row.zero();
            row[offs]=elb_m; row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-elb_n);
            appendVectorValue(_uB,upperBoundInf);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }
    }
    else if (shoulder == 'l')
    {   
        sh=2;
        // if shoulder's axes are controlled, constraint them
        if (!(*chain)[sh].isBlocked() && !(*chain)[sh+1].isBlocked() &&
            !(*chain)[sh+2].isBlocked())
        {
            // compute offset to shoulder's axes
            // given the blocked/release status of
            // previous link
            int offs=0;
            for (int i=0; i<sh; i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // constraints on the cables length
            row.zero();
            row[offs+2]=-1.71; row[offs+1]=1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-347.00*CTRL_DEG2RAD);
            appendVectorValue(_uB,upperBoundInf);

            row.zero();
            row[offs+2]=-1.71; row[offs+1]=1.71; row[offs]=1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-366.57*CTRL_DEG2RAD);
            appendVectorValue(_uB,112.42*CTRL_DEG2RAD);

            row.zero();
            row[offs+1]=-1.0; row[offs]=-1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-66.600*CTRL_DEG2RAD);
            appendVectorValue(_uB,213.30*CTRL_DEG2RAD);

            // // constraints to prevent arm from touching torso
            row.zero();
            row[offs+1]=-0.8; row[offs]=shou_m;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,shou_n-5.0*CTRL_DEG2RAD);
            appendVectorValue(_uB,upperBoundInf);

            // // constraints to limit shoulder abduction
            row.zero();
            row[offs+1]=-1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,IKINIPOPT_SHOULDER_MAXABDUCTION);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }
// 43.648918	-79.067511	-21.170343	-16.012210	 71.242122	-64.689977	 13.869851	 74.091853	 95.926386	-47.842270	-59.591317	 20.991342
// -71.242122   16.012210   21.170343
        // if elbow and pronosupination axes are controlled, constraint them
        if (!(*chain)[sh].isBlocked() && !(*chain)[sh+1].isBlocked())
        {
            // compute offset to elbow's axis
            // given the blocked/release status of
            // previous link
            int offs=0;
            // for (int i=0; i<(sh+3); i++)
            //     if (!(*chain)[i].isBlocked())
            //         offs++;

            // constraints to prevent the forearm from hitting the arm
            row.zero();
            row[offs+1]=elb_m; row[offs]=-1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,elb_n);

            row.zero();
            row[offs+1]=-elb_m; row[offs]=-1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-elb_n);
            appendVectorValue(_uB,upperBoundInf);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }
    }
}

/************************************************************************/