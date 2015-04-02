#include "iCubDblTchSlv.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <iCub/iKin/iKinFwd.h>

using namespace iCub::iKin;

/****************************************************************/
/****************************************************************/
iCubDoubleTouch_Problem::iCubDoubleTouch_Problem(string _type) : limb(_type), index("right_index")
    {   
        nJoints = limb.getDOF();
        nVars   = limb.getDOF();

        if (_type == "customLimb")
        {
            RLIC = new iCubShoulderConstrMod(limb.asChainMod(),'r'); // right shoulder constraints
            LLIC = new iCubShoulderConstrMod(limb.asChainMod(),'l'); // left  shoulder constraints
        }
        else {
            RLIC = NULL;
            LLIC = NULL;
        }
    }

/****************************************************************/
/****************************************************************/
iCubDoubleTouch_Variables::iCubDoubleTouch_Variables(int dim)
{
    joints.resize(dim,0.0);
    ee.resize(3,0.0);

    H   = eye(4);
    H_0 = eye(4);

    z_hat.resize(4,0.0);
    x_hat.resize(4,0.0);

    dot = 0;
}

/************************************************************************/
void iCubDoubleTouch_Variables::clone(const iCubDoubleTouch_Variables &v)
{
    ee     = v.ee;
    joints = v.joints;
    H      = v.H;
    H_0    = v.H_0;
    z_hat  = v.z_hat;
    x_hat  = v.x_hat;
    dot    = v.dot;
}

/************************************************************************/
iCubDoubleTouch_Variables &iCubDoubleTouch_Variables::operator=(const iCubDoubleTouch_Variables &v)
{
    clone(v);
    return *this;
}

/************************************************************************/
void iCubDoubleTouch_Variables::print()
{
    printf("joints [deg]  = (%s)\n",(CTRL_RAD2DEG*joints).toString().c_str());
    printf("EE   = (%s)\t",ee.toString().c_str());
    printf("dot product   = (%g)\t",dot);
    printf("obj function  = (%g)\t",1.0 + 1 * dot);
    printf("0.5*norm2(ee) = (%f)\n",0.5*norm2(ee));
}

/****************************************************************/
/****************************************************************/
class iCubDoubleTouch_NLP : public Ipopt::TNLP
{
protected:
    iCubDoubleTouch_Problem   &problem;
    iCubDoubleTouch_Variables  guess;
    iCubDoubleTouch_Variables  solution;

    iKinChainMod      *chain;
    iKinLinIneqConstr &rLIC;
    iKinLinIneqConstr &lLIC;

    unsigned int dim;

    yarp::sig::Vector  xd;

    // current end-eff H, pos, orien:
    yarp::sig::Matrix  H;
    yarp::sig::Matrix  H_0;
    yarp::sig::Vector  pos;
    yarp::sig::Vector  ori;

    yarp::sig::Vector  qd;
    yarp::sig::Vector  q0;
    yarp::sig::Vector  q;

    yarp::sig::Matrix J_xyz;
    yarp::sig::Matrix J1;

    yarp::sig::Vector z3rd;
    yarp::sig::Vector x1st;

    yarp::sig::Vector z_hat;
    yarp::sig::Vector x_hat;

    yarp::sig::Vector rlinC;
    yarp::sig::Vector llinC;

    bool   firstGo;

public:
    /****************************************************************/
    void computeQuantities(const Ipopt::Number *x)
    {
        Vector new_q(dim);
        for (Ipopt::Index i=0; i<(int)dim; i++)
            new_q[i]=x[i];

        if (!(q==new_q) || firstGo)
        {
            firstGo=false;
            
            q=chain->setAng(new_q);

            H=chain->getH();
            ori=dcm2axis(H);
            pos[0]=H(0,3);
            pos[1]=H(1,3);
            pos[2]=H(2,3);

            H_0=chain->getH0();

            z_hat = H   * z3rd;
            z_hat = z_hat / norm2(z_hat);
            x_hat = x1st;
            x_hat = x_hat / norm2(x_hat);

            J1   = chain->GeoJacobian();
            submatrix(J1,J_xyz,0,2,0,dim-1);
        }

        if (rLIC.isActive())
            rlinC=rLIC.getC()*q;

        if (lLIC.isActive())
            llinC=lLIC.getC()*q;
    }

    /****************************************************************/
    iCubDoubleTouch_NLP(iCubDoubleTouch_Problem &p, int _dim) : problem(p), guess(_dim), solution(_dim), rLIC(*p.getRLIC()), lLIC(*p.getLLIC())
    {
        chain = problem.limb.asChain();

        dim=chain->getDOF();

        xd.resize(3,0.0);

        ori.resize(4,0.0);
        pos.resize(3,0.0);

        q0.resize(dim,0.0);
        q.resize(dim,0.0);
        qd.resize(dim,0.0);

        J_xyz.resize(3,dim);  J_xyz.zero();
        J1.resize(6,dim);     J1.zero();

        H.resize(4,4);        H.zero();
        H_0.resize(4,4);      H_0.zero();

        z3rd.resize(4,0.0);     z3rd[2] = 1.0;
        x1st.resize(4,0.0);     x1st[0] = 1.0;

        z_hat.resize(4,0.0);
        x_hat.resize(4,0.0);

        firstGo=true;
    }

    /****************************************************************/
    yarp::sig::Vector get_qd() { return qd; }

    /****************************************************************/
    virtual void setInitialGuess(const iCubDoubleTouch_Variables &g)
    {
        guess=g;
        q0=guess.joints;
    }

    /****************************************************************/
    virtual iCubDoubleTouch_Variables getSolution() const
    {
        return solution;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n = dim;
        m=1;
        nnz_jac_g = dim;

        if (rLIC.isActive())
        {
            int lenLower=rLIC.getlB().length();
            int lenUpper=rLIC.getuB().length();

            if (lenLower && (lenLower==lenUpper) && (rLIC.getC().cols()==dim))
            {
                m+=lenLower;
                nnz_jac_g+=lenLower*dim;
            }
            else
                rLIC.setActive(false);
        }
        
        if (lLIC.isActive())
        {
            int lenLower=lLIC.getlB().length();
            int lenUpper=lLIC.getuB().length();

            if (lenLower && (lenLower==lenUpper) && (lLIC.getC().cols()==dim))
            {
                m+=lenLower;
                nnz_jac_g+=lenLower*dim;
            }
            else
                lLIC.setActive(false);
        }

        nnz_h_lag = (dim*(dim+1))>>1;
        index_style=TNLP::C_STYLE;
        
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=(*chain)(i).getMin();
            x_u[i]=(*chain)(i).getMax();
        }

        int lenLower=rLIC.getlB().length();
        Ipopt::Index offs=1;

        for (Ipopt::Index i=0; i<m; i++)
        {
            if (i < offs)
            {
                g_l[i]=0;
                g_u[i]=1e-6; // This seems to me a big margin, isn't it?
            }
            else if (i < lenLower+offs)
            {
                g_l[i]=rLIC.getlB()[i-offs];
                g_u[i]=rLIC.getuB()[i-offs];
            }
            else
            {
                g_l[i]=lLIC.getlB()[i-lenLower-offs];
                g_u[i]=lLIC.getuB()[i-lenLower-offs];
            }
        }

        return true;
    }
    
    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=q0[i];

        return true;
    }
    
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x);
        obj_value = 1.0 + 1 * dot(z_hat,x_hat);

        return true;
    }

    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x);

        Matrix AJ   = chain -> AnaJacobian(2).removeRows(4,2);
        AJ.setRow(3,Vector(AJ.cols(), 0.0));

        Matrix AJ_0 = chain -> AnaJacobian(0,2).removeRows(4,2);
        AJ_0.setRow(3,Vector(AJ_0.cols(), 0.0));

        grad_f[0] = -dot(AJ_0.getCol(0),z_hat) + dot(AJ.getCol(0),x_hat);

        for (Ipopt::Index i=1; i<n; i++)
            grad_f[i]=+dot(AJ.getCol(i),x_hat);

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x);

        Ipopt::Index offs=1;
        int lenLower=rLIC.getlB().length();

        for (Ipopt::Index i=0; i<m; i++)
        {
            if (i < offs)
                g[i] = 0.5*norm2(pos);

            else if (i<lenLower+offs)
                g[i]=rlinC[i-offs];

            else
                g[i]=llinC[i-lenLower-offs];
        }
                
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (m)
        {
            if (!values)
            {
                Ipopt::Index idx=0;
        
                for (Ipopt::Index row=0; row<m; row++)
                {
                    for (Ipopt::Index col=0; col<n; col++)
                    {
                        iRow[idx]=row;
                        jCol[idx]=col;
                        idx++;
                    }
                }
            }
            else
            {   
                computeQuantities(x);
            
                yarp::sig::Vector grad=(J_xyz.transposed()*pos);
                int lenLower=rLIC.getlB().length();

                Ipopt::Index idx =0;
                Ipopt::Index offs=1;

                for (Ipopt::Index row=0; row<m; row++)
                {
                    for (Ipopt::Index col=0; col<n; col++)
                    {    
                        if (row < offs)
                            values[idx]=grad[idx];

                        else if (row<lenLower+offs)
                            values[idx]=rLIC.getC()(row-offs,col);

                        else
                            values[idx]=lLIC.getC()(row-lenLower-offs,col);

                        idx++;
                    }
                }
            }
        }

        return true;
    }

    /****************************************************************/
    // bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
    //             Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
    //             bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
    //             Ipopt::Index *jCol, Ipopt::Number *values)
    // {
    //     return true;
    // }

    /****************************************************************/
    bool get_scaling_parameters(Ipopt::Number &obj_scaling, bool &use_x_scaling,
                                Ipopt::Index n, Ipopt::Number *x_scaling,
                                bool &use_g_scaling, Ipopt::Index m,
                                Ipopt::Number *g_scaling)
    {
        obj_scaling=1;

        use_x_scaling=false;
        // x_scaling[0]=1.0;
        // for (int j=1; j<11; j++)
        //     x_scaling[j]=10.0;

        use_g_scaling=true;
        for (int i=0; i<m; i++)
            g_scaling[i]=1.0;

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
        computeQuantities(x);
        // joints
        for (Ipopt::Index i=0; i<dim; i++) { solution.joints[i]=x[i]; }

        Vector ee(3,0.0);
        ee[0]=H(0,3);
        ee[1]=H(1,3);
        ee[2]=H(2,3);
        solution.ee    = ee;
        solution.H     = H;
        solution.H_0   = H_0;
        solution.z_hat = z_hat;
        solution.x_hat = x_hat;
        solution.dot   = dot(z_hat,x_hat);
    }
};

/****************************************************************/
/****************************************************************/

bool iCubDoubleTouch_Solver::solve(iCubDoubleTouch_Variables &solution)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-8);
    app->Options()->SetNumericValue("acceptable_tol",1e-4);
    app->Options()->SetIntegerValue("acceptable_iter",6);
    app->Options()->SetIntegerValue("max_iter",400);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetStringValue("nlp_scaling_method","user-scaling");
    // app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    // app->Options()->SetStringValue("nlp_scaling_method","none"); 
    // #####################################
    // app->Options()->SetStringValue("jacobian_approximation","finite-difference-values");
    // #####################################
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetIntegerValue("print_level",0);
    // app->Options()->SetIntegerValue("print_level",0);
    app->Options()->SetStringValue("derivative_test","none");
    // app->Options()->SetStringValue("derivative_test","first-order");
    // app->Options()->SetStringValue("derivative_test_print_all","yes");
    app->Options()->SetNumericValue("derivative_test_perturbation",1e-4);

    app->Initialize();

    Ipopt::SmartPtr<iCubDoubleTouch_NLP> nlp = new iCubDoubleTouch_NLP(problem,problem.nJoints);

    nlp->setInitialGuess(guess);

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));

    solution=nlp->getSolution();

    return (status==Ipopt::Solve_Succeeded);
}

// empty line to make gcc happy