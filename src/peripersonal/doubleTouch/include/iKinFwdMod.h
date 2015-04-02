/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
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

/**
 * \defgroup iKinMod iKinMod
 *  
 * Classes for forward-inverse kinematics of serial-links chains
 * of revolute joints and iCub limbs with the classical as well as the
 * modified DH convention
 *  
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \section dep_sec Dependencies 
 * - ctrlLib 
 * - IPOPT: see the <a
 *   href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>.
 *  
 * \author Alessandro Roncone - Ugo Pattacini 
 *  
 * \defgroup iKinFwdMod iKinFwdMod
 *  
 * @ingroup iKinMod 
 *
 * Classes for forward kinematics of serial-links chains 
 * and iCub limbs with the classical as well as the
 * modified DH convention
 *
 * Date: first release 30/06/2013
 *
 * \author Alessandro Roncone - Ugo Pattacini 
 *  
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */ 

#ifndef __IKINFWDMOD_H__
#define __IKINFWDMOD_H__

#include <string>
#include <deque>

#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinSlv.h>

#include <iCub/ctrl/math.h>


using namespace iCub::iKin;

/**
* \ingroup iKinFwdMod
*
* A Base class for defining a Link
*  
*/
class iKinLinkFather
{
protected:
    double       A;
    double       D;
    double       Alpha;
    double       Offset;
    double       c_alpha;
    double       s_alpha;
    double       Min;
    double       Max;
    double       Ang;
    bool         blocked;
    bool         cumulative;
    bool         constrained;
    unsigned int verbose;

    yarp::sig::Matrix H;
    yarp::sig::Matrix cumH;
    yarp::sig::Matrix DnH;

    const yarp::sig::Matrix zeros1x1;
    const yarp::sig::Vector zeros1;

    friend class iKinChainMod;

    // Default constructor: not implemented.
    iKinLinkFather();

    virtual void clone(const iKinLinkFather &l);    
    bool         isCumulative()     { return cumulative;          }
    void         block()            { blocked=true;               }
    void         block(double _Ang) { setAng(_Ang); blocked=true; }
    void         release()          { blocked=false;              }
    void         rmCumH()           { cumulative=false;           }
    void         addCumH(const yarp::sig::Matrix &_cumH);

public:
    /**
    * Constructor. 
    * @param _A is the Link length.
    * @param _D is the Link offset. 
    * @param _Alpha is the Link twist. 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by
    *             default).
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by
    *             default).
    */
    iKinLinkFather(double _A, double _D, double _Alpha, double _Offset,
             double _Min=-M_PI, double _Max=M_PI);

    /**
    * Creates a new Link from an already existing Link object.
    * @param l is the Link to be copied.
    */
    iKinLinkFather(const iKinLinkFather &l);

    /**
    * Copies a Link object into the current one.
    * @param l is a reference to an object of type iKinLinkFather.
    * @return a reference to the current object.
    */
    iKinLinkFather &operator=(const iKinLinkFather &l);

    /**
    * Sets the constraint status.
    * @param _constrained if true the joint angle cannot be set out 
    *                     of its limits (initialized as true).
    */
    void setConstraint(bool _constrained) { constrained=_constrained; }

    /**
    * Returns the constraint status.
    * @return current constraint status.
    */
    bool getConstraint() const { return constrained; }

    /**
    * Sets Link verbosity level.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Link verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() const { return verbose; }

    /**
    * Returns the Link blocking status.
    * @return true if link is blocked.
    */
    bool isBlocked() const { return blocked; }

    /**
    * Returns the Link length A.
    * @return Link length A.
    */
    double getA() const { return A; }

    /**
    * Sets the Link length A. 
    * @param new Link length _A. 
    */
    void setA(const double _A) { A=_A; }

    /**
    * Returns the Link offset D.
    * @return Link offset D.
    */
    double getD() const { return D; }

    /**
    * Sets the Link offset D. 
    * @param new Link offset _D. 
    */
    void setD(const double _D);

    /**
    * Returns the Link twist Alpha.
    * @return Link twist Alpha.
    */
    double getAlpha() const { return Alpha; }

    /**
    * Sets the Link twist Alpha. 
    * @param new Link twist _Alpha. 
    */
    void setAlpha(const double _Alpha);

    /**
    * Returns the joint angle offset.
    * @return joint angle offset.
    */
    double getOffset() const { return Offset; }

    /**
    * Sets the joint angle offset. 
    * @param new joint angle offset _Offset. 
    */
    void setOffset(const double _Offset) { Offset=_Offset; }

    /**
    * Returns the joint angle lower bound.
    * @return joint angle lower bound.
    */
    double getMin() const { return Min; }

    /**
    * Sets the joint angle lower bound.
    * @param _Min is the joint angle lower bound in [-pi,pi].
    */
    void setMin(const double _Min);

    /**
    * Returns the joint angle upper bound.
    * @return joint angle upper bound.
    */
    double getMax() const { return Max; }

    /**
    * Sets the joint angle higher bound.
    * @param _Max is the joint angle higher bound in [-pi,pi].
    */
    void setMax(const double _Max);

    /**
    * Returns the current joint angle value.
    * @return current joint angle value.
    */
    double getAng() const { return Ang; }

    /**
    * Sets the joint angle value. 
    * @param _Ang new value for joint angle.
    * @return actual joint angle value (constraints are evaluated).
    */
    double setAng(double _Ang);

    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    virtual yarp::sig::Matrix getH(bool c_override=false);

    /**
    * Same as getH() with specification of new joint angle position. 
    * @see getH()
    * @param _Ang is the new joint angle position. 
    * @param c_override.
    * @return a reference to H.
    */
    virtual yarp::sig::Matrix getH(double _Ang, bool c_override=false);

    /**
    * Computes the derivative of order n of the homogeneous 
    * transformation matrix H with respect to the joint angle. 
    * @param n is the order of the derivative (1 by default)
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain (false by
    *                   default).
    * @return a reference to derivative of order n.
    */
    virtual yarp::sig::Matrix getDnH(unsigned int n=1, bool c_override=false);

    /**
    * Set the homogeneous transformation.
    * It does nothing unless the link is FIXED RT LINK
    */
    virtual bool setH(yarp::sig::Matrix _H) { return false; }

    /**
    * Default destructor. 
    */
    virtual ~iKinLinkFather() { }
};

/**
* \ingroup iKinFwdMod
*
* A Derived class for defining a Link with standard 
* Denavit-Hartenberg convention. 
*  
* \note This class implements revolute joints only, as they are 
*       the unique type of joints used in humanoid robotics.
*/
/****************************************************************/
class iKinDirectLink : public iKinLinkFather
{
public:
    /**
    * Constructor. 
    */
    iKinDirectLink(double _A, double _D, double _Alpha, double _Offset,
                   double _Min=-M_PI, double _Max=M_PI) : iKinLinkFather(_A,_D,
                   _Alpha,_Offset,_Min,_Max) { };
    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    yarp::sig::Matrix getH(bool c_override=false);
};

/**
* \ingroup iKinFwdMod
*
* A Derived class for defining a Link with inverted
* Denavit-Hartenberg convention. 
*  
* \note This class implements revolute joints only, as they are 
*       the unique type of joints used in humanoid robotics.
*/
/****************************************************************/
class iKinInvertedLink : public iKinLinkFather
{
public:

    /**
    * Constructor. 
    */
    iKinInvertedLink(double _A, double _D, double _Alpha, double _Offset,
                     double _Min=-M_PI, double _Max=M_PI) : iKinLinkFather(_A,_D,
                     _Alpha,_Offset,_Min,_Max) { };

    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    yarp::sig::Matrix getH(bool c_override=false);
    /**
    * Computes the derivative of order n of the homogeneous 
    * transformation matrix H with respect to the joint angle. 
    * @param n is the order of the derivative (1 by default)
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain (false by
    *                   default).
    * @return a reference to derivative of order n.
    */
    yarp::sig::Matrix getDnH(unsigned int n=1, bool c_override=false);
};

/**
* \ingroup iKinFwdMod
*
* A Derived class for defining a Link through a fixed transform matrix
*  
*/
/****************************************************************/
class iKinFixedRTLink : public iKinLinkFather
{
public:

    /**
    * Constructor. 
    * @param _H is the roto-translation matrix.
    *           it's the matrix returned by getH();
    */
    iKinFixedRTLink(yarp::sig::Matrix _H);

    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    yarp::sig::Matrix getH(bool c_override=false);

    /**
    * Set the homogeneous transformation matrix H of the Link.
    * @param _H the new transformation matrix to be implemented.
    */
    bool setH(yarp::sig::Matrix _H);
    
    /**
    * Computes the derivative of order n of the homogeneous
    * transformation matrix H with respect to the joint angle.
    * @param n is the order of the derivative (1 by default)
    * @param c_override if true avoid accumulating the computation
    *                   of previous links in the chain (false by
    *                   default).
    * @return a reference to derivative of order n.
    */
    yarp::sig::Matrix getDnH(unsigned int n=1, bool c_override=false);

    /**
    * Default destructor.
    */
    virtual ~iKinFixedRTLink() { }
};


/**
* \ingroup iKinFwdMod
*
* A Base class for defining a Serial Link Chain.
*/
class iKinChainMod : public iKinChain
{
protected:
    std::deque<iKinLinkFather*> allList;
    std::deque<iKinLinkFather*> quickList;

    virtual void clone(const iKinChainMod &c);
    virtual void build();
    virtual void dispose();

public:
    /**
    * Default constructor. 
    */
    iKinChainMod();

    /**
    * Creates a new Chain from an already existing Chain object.
    * @param c is the Chain to be copied.
    */
    iKinChainMod(const iKinChainMod &c);

    /**
    * Copies a Chain object into the current one.
    * @param c is a reference to an object of type iKinChainMod.
    * @return a reference to the current object.
    */
    iKinChainMod &operator=(const iKinChainMod &c);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added.
    * @return a reference to the current object.
    */
    iKinChainMod &operator<<(iKinLinkFather &l);

    /**
    * Removes a Link from the bottom of the Chain.
    * @return a reference to the current object.
    */
    iKinChainMod &operator--(int);

    /**
    * Returns a reference to the ith Link of the Chain.
    * @param i is the Link number within the Chain.
    * @return a reference to the ith Link object.
    */
    iKinLinkFather &operator[](const unsigned int i) { return *allList[i]; }

    /**
    * Returns a reference to the ith Link of the Chain considering 
    * only those Links related to DOF.
    * @param i is the Link number within the Chain (in DOF order)
    * @return a reference to the ith Link object.
    */
    iKinLinkFather &operator()(const unsigned int i) { return *quickList[hash_dof[i]]; }

    /**
    * Adds a Link at the position ith within the Chain. 
    * @param i is the ith position where the Link is to be added.
    * @param l is the Link to be added.
    * @return true if successful (e.g. param i is in range).
    */
    bool addLink(const unsigned int i, iKinLinkFather &l);

    /**
    * Removes the ith Link from the Chain. 
    * @param i is the ith position from which the Link is to be 
    *          removed.
    * @return true if successful (e.g. param i is in range).
    */
    bool rmLink(const unsigned int i);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added. 
    * @see operator<<()
    */
    void pushLink(iKinLinkFather &l);

    /**
    * Removes all Links.
    */
    void clear();

    /**
    * Removes a Link from the bottom of the Chain. 
    * @see operator--()
    */
    void popLink();

    /**
    * Blocks the ith Link at the a certain value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link number. 
    * @param Ang is the value of joint angle to which the Link is 
    *            blocked.
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i, double Ang);

    /**
    * Blocks the ith Link at the current value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link number. 
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i) { return blockLink(i,getAng(i)); }

    /**
    * Changes the value of the ith blocked Link. Avoid the overhead 
    * required for DOFs handling. 
    * @param i is the Link number. 
    * @param Ang is the new value of joint angle to which the Link 
    *            is blocked.
    * @return true if successful (e.g. param i is in range and the 
    *         ith Link was already blocked).
    */
    bool setBlockingValue(const unsigned int i, double Ang);

    /**
    * Releases the ith Link. 
    * Chain DOF augmented by one.
    * @param i is the Link number. 
    * @return true if successful (e.g. param i is in range).
    */
    bool releaseLink(const unsigned int i);

    /**
    * Queries whether the ith Link is blocked.
    * @param i is the Link number. 
    * @return true if blocked && (param i is in range).
    */
    bool isLinkBlocked(const unsigned int i);

    /**
    * Sets the constraint status of all chain links.
    * @param _constrained is the new constraint status. 
    */
    void setAllConstraints(bool _constrained);

    /**
    * Sets the constraint status of ith link.
    * @param _constrained is the new constraint status. 
    */
    void setConstraint(unsigned int i, bool _constrained) { allList[i]->setConstraint(_constrained); }

    /**
    * Returns the constraint status of ith link. 
    * @return current constraint status.
    */
    bool getConstraint(unsigned int i) { return allList[i]->getConstraint(); }

    /**
    * Sets the verbosity level of all Links belonging to the Chain.
    * @param _verbose is the verbosity level. 
    */
    void setAllLinkVerbosity(unsigned int _verbose);

    /**
    * Sets the verbosity level of the Chain.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Chain verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() const { return verbose; }

    /**
    * Returns the number of Links belonging to the Chain.
    * @return number of Links.
    */
    unsigned int getN() const { return N; }

    /**
    * Returns the current number of Chain's DOF. 
    * DOF=N-M, where N=# of Links, M=# of blocked Links. 
    * @return number of DOF.
    */
    unsigned int getDOF() const { return DOF; }

    /**
    * Returns H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @return H0
    */
    yarp::sig::Matrix getH0() const { return H0; }

    /**
    * Sets H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @param H0 
    * @return true if succeed, false otherwise. 
    */
    bool setH0(const yarp::sig::Matrix &_H0);

    /**
    * Returns HN, the rigid roto-translation matrix from the Nth
    * frame to the end-effector. 
    * @return HN
    */
    yarp::sig::Matrix getHN() const { return HN; }

    /**
    * Sets HN, the rigid roto-translation matrix from the Nth frame 
    * to the end-effector. 
    * @param HN 
    * @return true if succeed, false otherwise. 
    */
    bool setHN(const yarp::sig::Matrix &_HN);

    /**
    * Sets the free joint angles to values of q[i].
    * @param q is a vector containing values for DOF.
    * @return the actual DOF values (angles constraints are 
    *         evaluated).
    */
    yarp::sig::Vector setAng(const yarp::sig::Vector &q);

    /**
    * Returns the current free joint angles values.
    * @return the actual DOF values.
    */
    yarp::sig::Vector getAng();

    /**
    * Sets the ith joint angle. 
    * @param i is the Link number. 
    * @param Ang the new angle's value. 
    * @return current ith joint angle (angle constraint is 
    *         evaluated).
    */
    double setAng(const unsigned int i, double _Ang);

    /**
    * Returns the current angle of ith joint. 
    * @param i is the Link number.  
    * @return current ith joint angle.
    */
    double getAng(const unsigned int i);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the ith frame in Denavit-Hartenberg 
    * notation. The second parameter if true enables the spannig 
    * over the full set of links, i.e. the blocked links as well. 
    * @param i is the Link number. 
    * @param allLink if true enables the spanning over the full set 
    *                of links (false by default).
    * @return Hi 
    */
    yarp::sig::Matrix getH(const unsigned int i, const bool allLink=false);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the end-effector frame in 
    * Denavit-Hartenberg notation (HN is taken into account). 
    * @return H(N-1)*HN
    */
    yarp::sig::Matrix getH();

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the end-effector frame in 
    * Denavit-Hartenberg notation (HN is taken into account). 
    * @param q is the vector of new DOF values.  
    * @return H(N-1)*HN
    */
    yarp::sig::Matrix getH(const yarp::sig::Vector &q);

    /**
    * Returns the coordinates of ith Link. Two notations are
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param i is the Link number. 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the ith Link Pose.
    */
    yarp::sig::Vector Pose(const unsigned int i, const bool axisRep=true);

    /**
    * Returns the 3D position coordinates of ith Link. 
    * @param i is the Link number. 
    * @return the ith Link Position.
    */
    yarp::sig::Vector Position(const unsigned int i);

    /**
    * Returns the coordinates of end-effector. Two notations are 
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the end-effector pose.
    */
    yarp::sig::Vector EndEffPose(const bool axisRep=true);

    /**
    * Returns the coordinates of end-effector computed in q. Two
    * notations are provided: the first with Euler Angles (XYZ 
    * form=>6x1 output vector) and second with axis/angle 
    * representation (default=>7x1 output vector). 
    * @param q is the vector of new DOF values. 
    * @param axisRep if true returns the axis/angle notation.  
    * @return the end-effector pose.
    */
    yarp::sig::Vector EndEffPose(const yarp::sig::Vector &q, const bool axisRep=true);

    /**
    * Returns the 3D coordinates of end-effector position.
    * @return the end-effector position.
    */
    yarp::sig::Vector EndEffPosition();

    /**
    * Returns the 3D coordinates of end-effector position computed in q.
    * @param q is the vector of new DOF values. 
    * @return the end-effector position.
    */
    yarp::sig::Vector EndEffPosition(const yarp::sig::Vector &q);

    /**
    * Returns the analitical Jacobian of the ith link.
    * @param i is the Link number. 
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(const unsigned int i, unsigned int col);

    /**
    * Returns the analitical Jacobian of the end-effector.
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(unsigned int col=3);

    /**
    * Returns the analitical Jacobian of the end-effector computed 
    * in q. 
    * @param q is the vector of new DOF values. 
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(const yarp::sig::Vector &q, unsigned int col=3);

    /**
    * Returns the geometric Jacobian of the ith link. 
    * @param i is the Link number.
    * @return the 6x(i-1) geometric Jacobian matrix.
    * @note All the links are considered.
    */
    yarp::sig::Matrix GeoJacobian(const unsigned int i);

    /**
    * Returns the geometric Jacobian of the end-effector.
    * @return the 6xDOF geometric Jacobian matrix.
    * @note The blocked links are not considered.
    */
    yarp::sig::Matrix GeoJacobian();

    /**
    * Returns the geometric Jacobian of the end-effector computed in
    * q. 
    * @param q is the vector of new DOF values. 
    * @return the geometric Jacobian.
    * @note The blocked links are not considered.
    */
    yarp::sig::Matrix GeoJacobian(const yarp::sig::Vector &q);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the DOF couple. 
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector Hessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Prepares computation for a successive call to 
    * fastHessian_ij(). 
    * @see fastHessian_ij()
    */
    void prepareForHessian();

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the DOF couple. 
    * <i>Fast Version</i>: to be used in conjunction with 
    * prepareForHessian(). 
    * @note It is advisable to use this version when successive 
    * computations with different indexes values are needed. 
    * @see prepareForHessian()
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector fastHessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the couple of 
    * links. 
    * @param lnk is the Link number up to which consider the 
    *            computation.
    * @param i is the index of the first link. 
    * @param j is the index of the second link.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector Hessian_ij(const unsigned int lnk, const unsigned int i,
                                 const unsigned int j);

    /**
    * Prepares computation for a successive call to 
    * fastHessian_ij() (link version). 
    * @see fastHessian_ij(lnk,...) 
    * @param lnk is the Link number up to which consider the 
    *            computation.
    */
    void prepareForHessian(const unsigned int lnk);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the couple of 
    * links. 
    * <i>Fast Version</i>: to be used in conjunction with 
    * prepareForHessian(lnk). 
    * @note It is advisable to use this version when successive 
    * computations with different indexes values are needed. 
    * @see prepareForHessian() 
    * @param lnk is the Link number up to which consider the 
    *            computation. 
    * @param i is the index of the first link. 
    * @param j is the index of the second link.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector fastHessian_ij(const unsigned int lnk, const unsigned int i,
                                     const unsigned int j);

    /**
    * Compute the time derivative of the geometric Jacobian.
    * @param dq the joint velocities.
    * @return the 6xDOF matrix \f$ 
    *         \partial{^2}F\left(q\right)/\partial t \partial q.
    *                 \f$
    */
    yarp::sig::Matrix DJacobian(const yarp::sig::Vector &dq);

    /**
    * Compute the time derivative of the geometric Jacobian
    * (link version).
    * @param lnk is the Link number up to which consider the 
    *            computation. 
    * @param dq the (lnk-1)x1 joint velocity vector.
    * @return the 6x(lnk-1) matrix \f$ 
    *         \partial{^2}F\left(q\right)/\partial t \partial q.
    *                 \f$
    */
    yarp::sig::Matrix DJacobian(const unsigned int lnk, const yarp::sig::Vector &dq);

    /**
    * Returns a pointer to the ChainMod seen as Chain object. 
    * Useful to to operate with the default iKin library.
    * @return a pointer to a Chain object with the same Links of
    *         ChainMod.
    */
    iKinChain *asChain() { return static_cast<iKinChain*>(this); }

    /**
    * Destructor. 
    */
    virtual ~iKinChainMod();
};

/**
* \ingroup iKinFwdMod
*
* A class for defining generic Limb.
*/
class iKinLimbMod : public iKinChainMod
{
protected:
    std::deque<iKinLinkFather*> linkList;
    std::string           type;
    bool                  configured;

    virtual void getMatrixFromProperties(yarp::os::Property &options,
                                         const std::string &tag, yarp::sig::Matrix &H);
    virtual void allocate(const std::string &_type);
    virtual void clone(const iKinLimbMod &limb);
    virtual void dispose();

    // make the following methods protected in order to prevent user from changing
    // too easily the internal structure of the chain;
    // to get access anyway to these hidden methods, user can rely on asChain()
    iKinChainMod &operator=(const iKinChainMod &c)              { return iKinChainMod::operator=(c);  }
    iKinChainMod &operator<<(iKinLinkFather &l)                    { return iKinChainMod::operator<<(l); }
    iKinChainMod &operator--(int)                            { return iKinChainMod::operator--(0); }
    iKinLinkFather  &operator[](const unsigned int i)           { return iKinChainMod::operator[](i); }
    iKinLinkFather  &operator()(const unsigned int i)           { return iKinChainMod::operator()(i); }
    bool       addLink(const unsigned int i, iKinLinkFather &l) { return iKinChainMod::addLink(i,l);  }
    bool       rmLink(const unsigned int i)               { return iKinChainMod::rmLink(i);     }
    void       pushLink(iKinLinkFather &l)                      { iKinChainMod::pushLink(l);          }
    void       clear()                                    { iKinChainMod::clear();              }
    void       popLink()                                  { iKinChainMod::popLink();            }
    void       pushLink(iKinLinkFather *pl);

public:
    /**
    * Default constructor. 
    */
    iKinLimbMod();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" limb
    */
    iKinLimbMod(const std::string &_type);

    /**
    * Creates a new Limb from an already existing Limb object.
    * @param limb is the Limb to be copied.
    */
    iKinLimbMod(const iKinLimbMod &limb);

    /**
    * Creates a new Limb from a list of properties wherein links 
    * parameters are specified. 
    * @param options is the list of links properties. 
    * @see fromLinksProperties()
    */
    iKinLimbMod(const yarp::os::Property &options);

    /**
    * Initializes the Limb from a list of properties wherein links 
    * parameters are specified. 
    * @param options is the list of links properties. 
    *  
    * @note Available options are: 
    *  
    * \b type <string>: specifies the limb handedness [left/right] 
    *    (default=right).
    *  
    * \b H0 <list of 4x4 doubles (per rows)>: specifies the rigid 
    *    roto-translation matrix from the root reference frame to
    *    the 0th frame (default=eye(4,4)).
    *  
    * \b HN <list of 4x4 doubles (per rows)>: specifies the rigid 
    *    roto-translation matrix from the Nth frame to the
    *    end-effector (default=eye(4,4)).
    *  
    * \b numLinks <int>: specifies the expected number of links.
    *  
    * \b A <double>: specifies the link length [m] (default=0.0).
    *  
    * \b D <double>: specifies the link offset [m] (default=0.0).
    *  
    * \b alpha <double>: specifies the link twist [deg] 
    *    (default=0.0).
    *  
    * \b offset <double>: specifies the joint angle offset [deg] 
    *    (default=0.0).
    *  
    * \b min <double>: specifies the joint angle lower bound [deg]
    *    (default=0.0).
    *  
    * \b max <double>: specifies the joint angle upper bound [deg] 
    *    (default=0.0).
    *  
    * \b blocked <double>: blocks the link at the specified value 
    *    [deg] (default=released).
    *  
    * @note The list should look like as the following: 
    *  
    * @code 
    * type right 
    * H0 (1.0 2.0 3.0 ...) 
    * HN (1.0 2.0 3.0 ...) 
    * numLinks 4 
    * link_0 (option1 value1) (option2 value2) ... 
    * link_1 (option1 value1) ... 
    * ... 
    * @endcode 
    */
    bool fromLinksProperties(const yarp::os::Property &options);

    /**
    * Checks if the limb has been properly configured.
    * @return true iff correctly coinfigured.
    */
    bool isValid() const { return configured; }

    /**
    * Copies a Limb object into the current one.
    * @param limb is a reference to an object of type iKinLimbMod.
    * @return a reference to the current object.
    */
    iKinLimbMod &operator=(const iKinLimbMod &limb);

    /**
    * Returns a pointer to the Limb seen as ChainMod object. 
    * Useful to to operate on the Links of Limb.
    * @return a pointer to a Chain object with the same Links of 
    *         Limb.
    */
    iKinChainMod *asChainMod() { return static_cast<iKinChainMod*>(this); }

    /**
    * Returns the Limb type as a string. 
    * @return the type as a string {"left", "right"}.
    */
    std::string getType() const { return type; }

    /**
    * Alignes the Limb joints bounds with current values set aboard 
    * the robot. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Limb limits.
    * @return true/false on success/failure. 
    *  
    * @note This method is empty in iKinLimbMod because it's 
    * limb-specific: see the implementations for iCubLimbs. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*>&) { notImplemented(verbose); return true; }

    /**
    * Destructor. 
    */
    virtual ~iKinLimbMod();
};

/**
* \ingroup iKinFwdMod
*
* A class for defining the iCub Custom Limb for doubleTouch. \n 
*/
class iCubCustomLimb : public iKinLimbMod
{
protected:
    virtual void allocate(const std::string &_type);
    virtual void clone(const iKinLimbMod &limb)
    {
        iKinLimbMod::clone(limb);
    }

public:
    /**
    * Default constructor. 
    */
    iCubCustomLimb()
    {
        allocate("test");
    }

    /**
    * Constructor. 
    * @param _type is a string to discriminate between the chain's
    *              type which can be either "right", "test" or "upper_body"
    */
    iCubCustomLimb(const std::string &_type)
    {
        allocate(_type);
    }

    /**
    * Creates a new limb from an already existing object.
    * @param sensor is the object to be copied.
    */
    iCubCustomLimb(const iCubCustomLimb &cust_limb)
    {
        clone(cust_limb);
    }

    /**
    * Returns a pointer to the Limb seen as Chain object.
    * Useful to to operate on the Links of Limb.
    * @return a pointer to a Chain object with the same Links of 
    *         Limb.
    */
    iKinChainMod* asChain()
    {
        return iKinLimbMod::asChainMod();
    }

    /**
    * Returns the Limb type as a string. 
    * @return the type as a string {"right", "test", "upper_body"}.
    */
    std::string getType() const
    {
        return iKinLimbMod::getType();
    }
    
    /**
    * Alignes the customLimb joints bounds with current values
    * set aboard the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Torso and the Arm limits.
    * @return true/false on success/failure. 
    */
    bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);

    /**
    * Sets the joint angles.
    * @param q is the joint Vector (straightforward).
    */
    yarp::sig::Vector setAng(const yarp::sig::Vector &q);

    /**
    * Sets the joint angles from the encoder values.
    * Hence, the encoders for the left (right) arm has to be reversed.
    * @param qr is the joint Vector of the left arm.
    * @param ql is the joint Vector of the right arm.
    */
    yarp::sig::Vector setAng(const yarp::sig::Vector &ql, const yarp::sig::Vector &qr);
};


/**
* \ingroup iKinIpOpt
*
* Class for dealing with iCub shoulder's constraints due to the 
* cables lenght.
*/
class iCubShoulderConstrMod : public iKinLinIneqConstr
{
protected:    
    double        shou_m, shou_n;
    double        elb_m,  elb_n;
    iKinChainMod *chain;
    char          shoulder;

    void clone(const iKinLinIneqConstr *obj);
    void appendMatrixRow(yarp::sig::Matrix &dest, const yarp::sig::Vector &row);
    void appendVectorValue(yarp::sig::Vector &dest, double val);

public:
    /**
    * Constructor. 
    * @param chain    the iKinChainMod object.
    * @param shoulder right or left shoulder (the left
    *                 shoulder has to be reversed).
    */
    iCubShoulderConstrMod(iKinChainMod *_chain, char _shoulder);

    void update(void*);
};

#endif



