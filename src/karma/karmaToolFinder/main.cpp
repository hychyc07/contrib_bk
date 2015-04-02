/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
\defgroup karmaToolFinder Tool Solver Part of the KARMA Experiment
 
@ingroup icub_karma  
 
This module finds the tool dimension employing nonlinear 
optimization. 

\section intro_sec Description 
Through an active exploration of the tool held in the robot hand 
a set of relevant data is collected, including hand position in 
3D space, eye's frame and pixels of the tool tip as acquired by 
vision algorithms. A nonlinear optimization is then exploited to 
find out the best estimate of the tool dimensions with respect 
to the hand reference frame. 
 
\section lib_sec Libraries 
- YARP libraries. 
- icubmod library. 
- IPOPT library. 
- OpenCV library.  

\section parameters_sec Parameters 
--robot \e robot
- Select the robot to connect to.

--name \e name
- Select the stem-name of the module used to open up ports. 
  By default \e name is <i>karmaToolFinder</i>. 
 
--arm \e type
- Select the default arm ("left" or "right") used for the data 
  acquisition.
 
--eye \e type
- Select the default eye ("left" or "right") used for the data 
  acquisition.
 
\section portsa_sec Ports Accessed
Assume that iCubInterface (with ICartesianControl interface
implemented) is running. 
 
\section portsc_sec Ports Created 
- \e /karmaToolFinder/rpc receives the information to manage the 
  data acquisition and optimization phase:
  -# <b>Enable</b>: <i>[enable]</i>. \n
  Start the data acquisition phase.
  -# <b>Disable</b>: <i>[disable]</i>. \n
  Terminate the data acquisition phase.
  -# <b>Num</b>: <i>[num]</i>. \n
  Retrieve the current number of input-output pairs used for the
  optimization. The reply is <i>[ack] num</i>.
  -# <b>Clear</b>: <i>[clear]</i>. \n
  Clear the current content of input-output pairs database.
  -# <b>Select</b>: <i>[select] arm eye</i>. \n
  Select the robot sources in terms of arm and eye used during
  the data acquisition.
  -# <b>Find</b>: <i>[find]</i>. \n
  Execute the optimization over the current database of
  input-output pairs. The reply is <i>[ack] x y z</i> including
  the tool dimensions given wrt hand reference frame.
  -# <b>Show</b>: <i>[show] x y z</i>. \n
  Enable the visualization of a tool with the dimensions
  specified by the user. The reply is <i>[ack]</i> or
  <i>[nack]</i>.

- \e /karmaToolFinder/in receives the position of the tool tip
   in the image plane.
 
 - \e /karmaToolFinder/img:i receives images from the camera.
 
 - \e /karmaToolFinder/img:o streams out images with
   superimposed information on the tool.
 
 - \e /karmaToolFinder/log:o streams out a complete set of data
   used during the acquisition.

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <algorithm>
#include <string>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cv.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;



/****************************************************************/
class FindToolTipNLP : public Ipopt::TNLP
{
protected:
    const deque<Vector> &p;
    const deque<Matrix> &H;

    Vector min;
    Vector max;
    Vector x0;
    Vector x;

public:
    /****************************************************************/
    FindToolTipNLP(const deque<Vector> &_p,
                   const deque<Matrix> &_H,
                   const Vector &_min, const Vector &_max) :
                   p(_p), H(_H)
    {
        min=_min;
        max=_max;
        x0=0.5*(min+max);
    }

    /****************************************************************/
    void set_x0(const Vector &x0)
    {
        size_t len=std::min(this->x0.length(),x0.length());
        for (size_t i=0; i<len; i++)
            this->x0[i]=x0[i];
    }

    /****************************************************************/
    Vector get_result() const
    {
        return x;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=3;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=min[i];
            x_u[i]=max[i];
        }

        return true;
    }
    
    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=x0[i];

        return true;
    }
    
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        obj_value=0.0;
        if (p.size()>0)
        {
            Vector _x(4);
            _x[0]=x[0];
            _x[1]=x[1];
            _x[2]=x[2];
            _x[3]=1.0;

            for (size_t i=0; i<p.size(); i++)
            {
                Vector pi=H[i]*_x;
                pi=pi/pi[2];
                pi.pop_back();

                obj_value+=0.5*norm2(p[i]-pi);
            }

            obj_value/=p.size();
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        Vector _x(4);
        _x[0]=x[0];
        _x[1]=x[1];
        _x[2]=x[2];
        _x[3]=1.0;

        grad_f[0]=grad_f[1]=grad_f[2]=0.0;
        if (p.size()>0)
        {
            for (size_t i=0; i<p.size(); i++)
            {
                Vector pi=H[i]*_x;
                pi=pi/pi[2];
                pi.pop_back();

                Vector d=p[i]-pi;

                double u_num=dot(H[i].getRow(0),_x);
                double v_num=dot(H[i].getRow(1),_x);

                double lambda=dot(H[i].getRow(2),_x);
                double lambda2=lambda*lambda;

                Vector dp_dx1(2);
                dp_dx1[0]=(H[i](0,0)*lambda-H[i](2,0)*u_num)/lambda2;
                dp_dx1[1]=(H[i](1,0)*lambda-H[i](2,0)*v_num)/lambda2;

                Vector dp_dx2(2);
                dp_dx2[0]=(H[i](0,1)*lambda-H[i](2,1)*u_num)/lambda2;
                dp_dx2[1]=(H[i](1,1)*lambda-H[i](2,1)*v_num)/lambda2;

                Vector dp_dx3(2);
                dp_dx3[0]=(H[i](0,2)*lambda-H[i](2,2)*u_num)/lambda2;
                dp_dx3[1]=(H[i](1,2)*lambda-H[i](2,2)*v_num)/lambda2;
                
                grad_f[0]-=dot(d,dp_dx1);
                grad_f[1]-=dot(d,dp_dx2);
                grad_f[2]-=dot(d,dp_dx3);
            }

            for (Ipopt::Index i=0; i<n; i++)
                grad_f[i]/=p.size();
        }        

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }


    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }
    

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        this->x.resize(n);
        for (Ipopt::Index i=0; i<n; i++)
            this->x[i]=x[i];
    }
};



/****************************************************************/
class FindToolTip
{
protected:
    Vector min;
    Vector max;
    Vector x0;

    deque<Vector> p;
    deque<Matrix> H;

    /****************************************************************/
    double evalError(const Vector &x)
    {
        double error=0.0;
        if (p.size()>0)
        {
            Vector _x=x;
            if (_x.length()<4)
                _x.push_back(1.0);

            for (size_t i=0; i<p.size(); i++)
            {
                Vector pi=H[i]*_x;
                pi=pi/pi[2];
                pi.pop_back();

                error+=norm(p[i]-pi);
            }

            error/=p.size();
        }

        return error;
    }

public:
    /****************************************************************/
    FindToolTip()
    {
        min.resize(3); max.resize(3);
        min[0]=-1.0;   max[0]=1.0;
        min[1]=-1.0;   max[1]=1.0;
        min[2]=-1.0;   max[2]=1.0;

        x0=0.5*(min+max);
    }

    /****************************************************************/
    void setBounds(const Vector &min, const Vector &max)
    {
        size_t len_min=std::min(this->min.length(),min.length());
        size_t len_max=std::min(this->max.length(),max.length());

        for (size_t i=0; i<len_min; i++)
            this->min[i]=min[i];

        for (size_t i=0; i<len_max; i++)
            this->max[i]=max[i];
    }

    /****************************************************************/
    bool addItem(const Vector &pi, const Matrix &Hi)
    {
        if ((pi.length()>=2) && (Hi.rows()>=3) && (Hi.cols()>=4))
        {
            Vector _pi=pi.subVector(0,1);
            Matrix _Hi=Hi.submatrix(0,2,0,3);

            p.push_back(_pi);
            H.push_back(_Hi);

            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    void clearItems()
    {
        p.clear();
        H.clear();
    }

    /****************************************************************/
    size_t getNumItems() const
    {
        return p.size();
    }

    /****************************************************************/
    bool setInitialGuess(const Vector &x0)
    {
        size_t len=std::min(x0.length(),this->x0.length());
        for (size_t i=0; i<len; i++)
            this->x0[i]=x0[i];

        return true;
    }

    /****************************************************************/
    bool solve(Vector &x, double &error)
    {
        if (p.size()>0)
        {
            Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
            app->Options()->SetNumericValue("tol",1e-8);
            app->Options()->SetNumericValue("acceptable_tol",1e-8);
            app->Options()->SetIntegerValue("acceptable_iter",10);
            app->Options()->SetStringValue("mu_strategy","adaptive");
            app->Options()->SetIntegerValue("max_iter",300);
            app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
            app->Options()->SetStringValue("hessian_approximation","limited-memory");
            app->Options()->SetIntegerValue("print_level",0);
            app->Options()->SetStringValue("derivative_test","none");
            app->Initialize();

            Ipopt::SmartPtr<FindToolTipNLP> nlp=new FindToolTipNLP(p,H,min,max);

            nlp->set_x0(x0);
            Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

            x=nlp->get_result();
            error=evalError(x);

            return (status==Ipopt::Solve_Succeeded);
        }
        else
            return false;
    }
};



/************************************************************************/
class FinderModule: public RFModule, public PortReader
{
protected:
    PolyDriver         drvArmL;
    PolyDriver         drvArmR;
    PolyDriver         drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    RpcServer          rpcPort;
    Matrix             Prj;
    Semaphore          mutex;
    FindToolTip        solver;
    Vector             solution;
    string             arm;
    string             eye;
    bool               enabled;

    BufferedPort<ImageOf<PixelBgr> > imgInPort;
    Port                             imgOutPort;
    Port                             dataInPort;
    Port                             logPort;

    /************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle data;
        data.read(connection);
        if ((data.size()>=2) && enabled)
        {
            Vector xa,oa;
            iarm->getPose(xa,oa);

            Vector xe,oe;
            if (eye=="left")
                igaze->getLeftEyePose(xe,oe);
            else
                igaze->getRightEyePose(xe,oe);

            Matrix Ha=axis2dcm(oa);
            xa.push_back(1.0);
            Ha.setCol(3,xa);

            Matrix He=axis2dcm(oe);
            xe.push_back(1.0);
            He.setCol(3,xe);

            Matrix H=Prj*SE3inv(He)*Ha;
            Vector p(2);
            p[0]=data.get(0).asDouble();
            p[1]=data.get(1).asDouble();

            if (logPort.getOutputCount()>0)
            {
                Vector log=p;
                for (int i=0; i<H.rows(); i++)
                    log=cat(log,H.getRow(i));
                for (int i=0; i<Prj.rows(); i++)
                    log=cat(log,Prj.getRow(i));
                for (int i=0; i<Ha.rows(); i++)
                    log=cat(log,Ha.getRow(i));
                for (int i=0; i<He.rows(); i++)
                    log=cat(log,He.getRow(i));

                logPort.write(log);
            }

            mutex.wait();
            solver.addItem(p,H);
            mutex.post();
        }

        return true;
    }

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        string name=rf.check("name",Value("karmaToolFinder")).asString().c_str();
        arm=rf.check("arm",Value("right")).asString().c_str();
        eye=rf.check("eye",Value("left")).asString().c_str();

        if ((arm!="left") && (arm!="right"))
        {
            printf("Invalid arm requested!\n");
            return false;
        }

        if ((eye!="left") && (eye!="right"))
        {
            printf("Invalid eye requested!\n");
            return false;
        }

        Property optionArmL("(device cartesiancontrollerclient)");
        optionArmL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
        optionArmL.put("local",("/"+name+"/left_arm").c_str());
        if (!drvArmL.open(optionArmL))
        {
            printf("Cartesian left_arm controller not available!\n");
            terminate();
            return false;
        }

        Property optionArmR("(device cartesiancontrollerclient)");
        optionArmR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
        optionArmR.put("local",("/"+name+"/right_arm").c_str());
        if (!drvArmR.open(optionArmR))
        {
            printf("Cartesian right_arm controller not available!\n");
            terminate();
            return false;
        }

        if (arm=="left")
            drvArmL.view(iarm);
        else
            drvArmR.view(iarm);

        Property optionGaze("(device gazecontrollerclient)");
        optionGaze.put("remote","/iKinGazeCtrl");
        optionGaze.put("local",("/"+name+"/gaze").c_str());
        if (drvGaze.open(optionGaze))
            drvGaze.view(igaze);
        else
        {
            printf("Gaze controller not available!\n");
            terminate();
            return false;
        }

        Bottle info;
        igaze->getInfo(info);
        if (Bottle *pB=info.find(("camera_intrinsics_"+eye).c_str()).asList())
        {
            int cnt=0;
            Prj.resize(3,4);
            for (int r=0; r<Prj.rows(); r++)
                for (int c=0; c<Prj.cols(); c++)
                    Prj(r,c)=pB->get(cnt++).asDouble();
        }
        else
        {
            printf("Camera intrinsic parameters not available!\n");
            terminate();
            return false;
        }

        imgInPort.open(("/"+name+"/img:i").c_str());
        imgOutPort.open(("/"+name+"/img:o").c_str());
        dataInPort.open(("/"+name+"/in").c_str());
        logPort.open(("/"+name+"/log:o").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        Vector min(3),max(3);
        min[0]=-1.0; max[0]=1.0;
        min[1]=-1.0; max[1]=1.0;
        min[2]=-1.0; max[2]=1.0;
        solver.setBounds(min,max);
        solution.resize(3,0.0);

        enabled=false;
        dataInPort.setReader(*this);

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB4('c','l','e','a'):
                {
                    mutex.wait();
                    solver.clearItems();
                    solution=0.0;
                    mutex.post();

                    reply.addVocab(ack);
                    return true;
                }
                
                //-----------------
                case VOCAB4('s','e','l','e'):
                {
                    if (command.size()>=3)
                    {
                        string arm=command.get(1).asString().c_str();
                        string eye=command.get(2).asString().c_str();

                        if ((arm=="left") || (arm=="right"))
                            this->arm=arm;

                        if ((eye=="left") || (eye=="right"))
                            this->eye=eye;

                        if (this->arm=="left")
                            drvArmL.view(iarm);
                        else
                            drvArmR.view(iarm);

                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB3('n','u','m'):
                {
                    reply.addVocab(ack);

                    mutex.wait();
                    reply.addInt((int)solver.getNumItems());
                    mutex.post();

                    return true;
                }

                //-----------------
                case VOCAB4('f','i','n','d'):
                {
                    double error;

                    mutex.wait();
                    bool ok=solver.solve(solution,error);
                    mutex.post();

                    if (ok)
                    {
                        reply.addVocab(ack);
                        for (size_t i=0; i<solution.length(); i++)
                            reply.addDouble(solution[i]);
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB4('s','h','o','w'):
                {
                    if (command.size()>=4)
                    {
                        solution[0]=command.get(1).asDouble();
                        solution[1]=command.get(2).asDouble();
                        solution[2]=command.get(3).asDouble();

                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB4('e','n','a','b'):
                {
                    enabled=true;
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('d','i','s','a'):
                {
                    enabled=false;
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.03;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (imgOutPort.getOutputCount()>0)
        {
            if (ImageOf<PixelBgr> *pImgBgrIn=imgInPort.read(false))
            {
                Vector xa,oa;
                iarm->getPose(xa,oa);

                Matrix Ha=axis2dcm(oa);
                xa.push_back(1.0);
                Ha.setCol(3,xa);

                Vector v(4,0.0); v[3]=1.0;
                Vector c=Ha*v;

                v=0.0; v[0]=0.05; v[3]=1.0;
                Vector x=Ha*v;

                v=0.0; v[1]=0.05; v[3]=1.0;
                Vector y=Ha*v;

                v=0.0; v[2]=0.05; v[3]=1.0;
                Vector z=Ha*v;

                v=solution; v.push_back(1.0);
                Vector t=Ha*v;

                Vector pc,px,py,pz,pt;
                int camSel=(eye=="left")?0:1;
                igaze->get2DPixel(camSel,c,pc);
                igaze->get2DPixel(camSel,x,px);
                igaze->get2DPixel(camSel,y,py);
                igaze->get2DPixel(camSel,z,pz);
                igaze->get2DPixel(camSel,t,pt);

                CvPoint point_c=cvPoint((int)pc[0],(int)pc[1]);
                CvPoint point_x=cvPoint((int)px[0],(int)px[1]);
                CvPoint point_y=cvPoint((int)py[0],(int)py[1]);
                CvPoint point_z=cvPoint((int)pz[0],(int)pz[1]);
                CvPoint point_t=cvPoint((int)pt[0],(int)pt[1]);

                cvCircle(pImgBgrIn->getIplImage(),point_c,4,cvScalar(0,255,0),4);
                cvCircle(pImgBgrIn->getIplImage(),point_t,4,cvScalar(255,0,0),4);

                cvLine(pImgBgrIn->getIplImage(),point_c,point_x,cvScalar(0,0,255),2);
                cvLine(pImgBgrIn->getIplImage(),point_c,point_y,cvScalar(0,255,0),2);
                cvLine(pImgBgrIn->getIplImage(),point_c,point_z,cvScalar(255,0,0),2);
                cvLine(pImgBgrIn->getIplImage(),point_c,point_t,cvScalar(255,255,255),2);

                imgOutPort.write(*pImgBgrIn);
            }
        }

        return true;
    }

    /************************************************************************/
    void terminate()
    {

        imgInPort.close();
        imgOutPort.close();
        dataInPort.close();     // close prior to shutting down motor-interfaces
        logPort.close();
        rpcPort.close();

        if (drvArmL.isValid())
            drvArmL.close();

        if (drvArmR.isValid())
            drvArmR.close();

        if (drvGaze.isValid())
            drvGaze.close();
    }

    /************************************************************************/
    bool close()
    {
        terminate();
        return true;
    }
};



/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    FinderModule mod;
    return mod.runModule(rf);
}


