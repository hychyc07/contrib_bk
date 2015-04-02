/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "iCub/learningMachine/GPRoptimization.h" // optimization library definition
#include "iCub/learningMachine/GPR.h"

#ifdef GPR_USE_IPOPT

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define CAST_IPOPTAPP(x)        (static_cast<IpoptApplication*>(x))
#define GPRIPOPT_DEFAULT_TRANSTOL      (1e-6)
#define GPRIPOPT_DEFAULT_LWBOUNDINF    (-1e9)
#define GPRIPOPT_DEFAULT_UPBOUNDINF    (+1e9)

using namespace Ipopt;
using namespace yarp::math;

namespace iCub {
namespace learningmachine {
    
    
class GPR_NLP : public TNLP
{
private:
    // Copy constructor: not implemented.
    GPR_NLP(const GPR_NLP&);
    // Assignment operator: not implemented.
    GPR_NLP &operator=(const GPR_NLP&);

protected:
    GPRLearner &gpr;
    yarp::sig::Matrix &samplesX; //training input points
    yarp::sig::Matrix &samplesY; // training targets

    unsigned int dim;
    double __obj_scaling;
    double __x_scaling;
    double __g_scaling;
    double lowerBoundInf;
    double upperBoundInf;
    double tol;

    //GPRIterateCallback *callback;
    
    bool   firstGo;

    /************************************************************************/
    double negL(const Number *x)
    {
       
        int nSamples=this->samplesX.rows();
       // int inputDim=this->samplesX.cols();
        // assert(dim==2*(inputDim+1));
        int nMeanHyperpar=gpr.getMeanPtr()->getNumberOfHyperparameters();
        int nCovHyperpar=gpr.getCovariancePtr()->getNumberOfHyperparameters();
        yarp::sig::Vector hypmean(nMeanHyperpar);
        yarp::sig::Vector hypcov(nCovHyperpar);
        Index i=0;
        for (; i<nMeanHyperpar; ++i)
        {
            hypmean[i]=x[i];
        }  
        for (; i<nCovHyperpar+nMeanHyperpar; ++i)
        {
            hypcov[i]=x[i+nMeanHyperpar];
        }
        double hyp_lik=x[nMeanHyperpar+nCovHyperpar]; //last parameter is sigmaNoise
        gpr.setSigma(hyp_lik);
        
        gpr.getCovariancePtr()->setHyperparameters(hypcov);
        yarp::sig::Matrix covariance=gpr.getCovariancePtr()->calculateCovariance(samplesX)/hyp_lik;
        
           
        //add sigma*I (add I because we pre-divided covariance)
        covariance+=yarp::math::eye(covariance.rows());
        
        gpr.getMeanPtr()->setHyperparameters(hypmean);
        yarp::sig::Vector mean=gpr.getMeanPtr()->calculateMean(samplesX);
        
         
        yarp::sig::Matrix Kinv=yarp::math::pinv(covariance);
        
        double data_fit=0;
        for (int dimCt=0; dimCt<this->samplesY.cols(); dimCt++)
        {
            yarp::sig::Vector b=samplesY.getCol(dimCt)-mean;
            data_fit+=(0.5/hyp_lik)*(b*(Kinv*b))(0);
            
        }
        
        double complexity_penalty=det(covariance)/2;
        
        double normalization_constant=nSamples*log(2*M_PI*hyp_lik)/2;
        
        return data_fit+complexity_penalty+normalization_constant;
    };
    
    yarp::sig::Vector dNegL(const Number *x)
    {
        yarp::sig::Vector gradient=zeros(dim);
        int nSamples=this->samplesX.rows();
        //int inputDim=this->samplesX.cols();
        // assert(dim==2*(inputDim+1));
        int nMeanHyperpar=gpr.getMeanPtr()->getNumberOfHyperparameters();
        int nCovHyperpar=gpr.getCovariancePtr()->getNumberOfHyperparameters();
        yarp::sig::Vector hypmean(nMeanHyperpar);
        yarp::sig::Vector hypcov(nCovHyperpar);
        Index i=0;
        for (; i<nMeanHyperpar; ++i)
        {
            hypmean[i]=x[i];
        }  
        for (; i<nCovHyperpar+nMeanHyperpar; ++i)
        {
            hypcov[i]=x[i+nMeanHyperpar];
        }
        double hyp_lik=x[nMeanHyperpar+nCovHyperpar]; //last parameter is sigmaNoise
        gpr.setSigma(hyp_lik);
        
        gpr.getCovariancePtr()->setHyperparameters(hypcov);
        yarp::sig::Matrix covariance=gpr.getCovariancePtr()->calculateCovariance(samplesX)/hyp_lik;
        
           
        //add sigma*I (add I because we pre-divided covariance)
        covariance+=yarp::math::eye(covariance.rows());
        
        gpr.getMeanPtr()->setHyperparameters(hypmean);
        yarp::sig::Vector mean=gpr.getMeanPtr()->calculateMean(samplesX);
        
         
        yarp::sig::Matrix Kinv=yarp::math::pinv(covariance);
        
        // DERIVATIVES
        for (int dimCt=0; dimCt<this->samplesY.cols(); dimCt++)
        {
            yarp::sig::Vector b=samplesY.getCol(dimCt)-mean;
            yarp::sig::Vector alpha=Kinv*b; 
            yarp::sig::Matrix Q= Kinv - outerProduct(alpha, alpha);
            
            double traceQ=0;
            for (int ind=0; ind<Q.rows(); ++ind)
                traceQ=+ Q(ind, ind);
            double dHyplik=hyp_lik*traceQ;
            gradient(dim-1)+=dHyplik;
            
            // MEAN HYPERPARAMETER DERIVATIVE
            //     for i = 1:numel(hyp.mean), 
            //       dnlZ.mean(i) = -feval(mean{:}, hyp.mean, x, i)'*alpha;
            for (int meanCt=0; meanCt<nCovHyperpar; ++meanCt)
            {
                
                yarp::sig::Vector dMean = gpr.getMeanPtr()->calculateMeanDerivative(samplesX, meanCt);
                gradient(meanCt)+=dot(dMean, alpha);
            }
            
            
            // COVARIANCE HYPERPARAMETER DERIVATIVE
            //     for i = 1:numel(hyp.cov)
            //       dnlZ.cov(i) = sum(sum(Q.*feval(cov{:}, hyp.cov, x, [], i)))/2;
            //     end
            for (int covCt=0; covCt<nCovHyperpar; ++covCt)
            {
                yarp::sig::Matrix dCov = gpr.getCovariancePtr()->getDerivative(samplesX, covCt);
                for (int i1=0; i1<dCov.rows(); ++i1)  //check beforehand for matrices' size?
                    for(int i2=0; i2<dCov.cols(); ++i2)
                        gradient(covCt+nMeanHyperpar)+=Q(i1, i2)*dCov(i1, i2);                
                
            }                 
        }   
		return gradient;
    };
    

public:
    /************************************************************************/
    GPR_NLP(GPRLearner&c, yarp::sig::Matrix trainingX, yarp::sig::Matrix trainingY, bool *_exhalt=NULL) :
             gpr(c), samplesX(trainingX), samplesY(trainingY)
    {
        dim=gpr.getMeanPtr()->getNumberOfHyperparameters()+gpr.getCovariancePtr()->getNumberOfHyperparameters()+1;

        firstGo=true;

        __obj_scaling=1.0;
        __x_scaling  =1.0;
        __g_scaling  =1.0;

        lowerBoundInf=GPRIPOPT_DEFAULT_LWBOUNDINF;
        upperBoundInf=GPRIPOPT_DEFAULT_UPBOUNDINF;
        tol=GPRIPOPT_DEFAULT_TRANSTOL;

      //  callback=NULL;
    }

    /************************************************************************/
  //  void set_callback(GPRIterateCallback *_callback) { callback=_callback; }

    /************************************************************************/
    void set_scaling(double _obj_scaling, double _x_scaling, double _g_scaling)
    {
        __obj_scaling=_obj_scaling;
        __x_scaling  =_x_scaling;
        __g_scaling  =_g_scaling;
    }

    /************************************************************************/
    void set_bound_inf(double lower, double upper)
    {
        lowerBoundInf=lower;
        upperBoundInf=upper;
    }

    /************************************************************************/
    void set_translational_tol(double tol) { this->tol=tol; }

    /************************************************************************/
    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                      IndexStyleEnum& index_style)
    {
        n=dim;
        m=0;
        nnz_jac_g=dim;

        nnz_h_lag=(dim*(dim+1))>>1;
        index_style=TNLP::C_STYLE;
        
        return true;
    }

    /************************************************************************/
    bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l,
                         Number* g_u)
    {
        int nMeanHyperpar=gpr.getMeanPtr()->getNumberOfHyperparameters();
        int nCovHyperpar=gpr.getCovariancePtr()->getNumberOfHyperparameters();
        assert(n==(nMeanHyperpar+nCovHyperpar+1));
        assert(m == 0);

        for (Index i=0; i<n; i++)
        {
            x_l[i]=GPRIPOPT_DEFAULT_LWBOUNDINF;
            x_u[i]=GPRIPOPT_DEFAULT_UPBOUNDINF;
        }
        // COMMENTED OUT BECAUSE WE HAVE NO CONSTRAINTS
//         Index offs=0;   
// 
//         for (Index i=0; i<m; i++)
//         {
//             if (i==0)
//             {
//                 g_l[0]=lowerBoundInf;
//                 g_u[0]=translationalTol;        
//                 offs=1;
//             }
//             else
//             {
//                 g_l[i]=LIC.getlB()[i-offs];
//                 g_u[i]=LIC.getuB()[i-offs];
//             }
//         }

        return true;
    }
    
    /************************************************************************/
    bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                            Number* z_L, Number* z_U, Index m, bool init_lambda,
                            Number* lambda)
    {
        
        assert(init_z == false);
        assert(init_lambda == false);
        int nMeanHyperpar=gpr.getMeanPtr()->getNumberOfHyperparameters();
        int nCovHyperpar=gpr.getCovariancePtr()->getNumberOfHyperparameters();
        assert(n==(nMeanHyperpar+nCovHyperpar+1));
        yarp::sig::Vector hypcov = gpr.getCovariancePtr()->getHyperparameters();
        yarp::sig::Vector hypmean = gpr.getMeanPtr()->getHyperparameters();
        
        Index i=0;
        for (; i<nMeanHyperpar; ++i)
        {
            x[i]=hypmean[i];
        }  
        for (; i<nCovHyperpar+nMeanHyperpar; ++i)
        {
            x[i+nMeanHyperpar]=hypcov[i];
        }
        x[nMeanHyperpar+nCovHyperpar]=gpr.getSigma(); //last parameter is sigmaNoise
        
        return true;
    }
    
    /************************************************************************/
    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
    {
        obj_value=negL(x);

        return true;
    }
    
    /************************************************************************/
    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
    {
        //ATTN maybe add some checks?
        yarp::sig::Vector grad=dNegL(x);
         for (Index i=0; i<n; i++)
             grad_f[i]=grad[i];
        return true;
    }
    
    /************************************************************************/
    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
    {
        return false;
    }
    
    /************************************************************************/
    bool eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac,
                    Index* iRow, Index *jCol, Number* values)
    {
        return false;
    }
    
    /************************************************************************/
    bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                Index m, const Number* lambda, bool new_lambda,
                Index nele_hess, Index* iRow, Index* jCol, Number* values)
    {
        return false;
    }

    /************************************************************************/
    bool intermediate_callback(AlgorithmMode mode, Index iter, Number obj_value,
                               Number inf_pr, Number inf_du, Number mu, Number d_norm,
                               Number regularization_size, Number alpha_du, Number alpha_pr,
                               Index ls_trials, const IpoptData* ip_data,
                               IpoptCalculatedQuantities* ip_cq)
    {
//         if (callback!=NULL)
//             callback->exec(xd,q);
// 
//         if (exhalt!=NULL)
//             return !(*exhalt);
//         else
            return true;
    }

    /************************************************************************/
    bool get_scaling_parameters(Number& obj_scaling, bool& use_x_scaling, Index n,
                                Number* x_scaling, bool& use_g_scaling, Index m,
                                Number* g_scaling)
    {
        obj_scaling=__obj_scaling;

        for (Index i=0; i<n; i++)
            x_scaling[i]=__x_scaling;

        for (Index j=0; j<m; j++)
            g_scaling[j]=__g_scaling;

        use_x_scaling=use_g_scaling=true;

        return true;
    }

    /************************************************************************/
    void finalize_solution(SolverReturn status, Index n, const Number* x,
                           const Number* z_L, const Number* z_U, Index m,
                           const Number* g, const Number* lambda, Number obj_value,
                           const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
    {
        int nSamples=this->samplesX.rows();
        // int inputDim=this->samplesX.cols();
        // assert(dim==2*(inputDim+1));
        int nMeanHyperpar=gpr.getMeanPtr()->getNumberOfHyperparameters();
        int nCovHyperpar=gpr.getCovariancePtr()->getNumberOfHyperparameters();
        yarp::sig::Vector hypmean(nMeanHyperpar);
        yarp::sig::Vector hypcov(nCovHyperpar);
        Index i=0;
        for (; i<nMeanHyperpar; ++i)
        {
            hypmean[i]=x[i];
        }  
        for (; i<nCovHyperpar+nMeanHyperpar; ++i)
        {
            hypcov[i]=x[i+nMeanHyperpar];
        }
        double hyp_lik=x[nMeanHyperpar+nCovHyperpar]; //last parameter is sigmaNoise
        gpr.setSigma(hyp_lik);
        
    }

    /************************************************************************/
    virtual ~GPR_NLP() { }
};

void GPRLearner::optimize(const double tol,const int max_iter, const unsigned int verbose, bool useHessian)
{
    IpoptApplication *App=new IpoptApplication();

    CAST_IPOPTAPP(App)->Options()->SetNumericValue("tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetNumericValue("acceptable_tol",tol);
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("acceptable_iter",10);
    CAST_IPOPTAPP(App)->Options()->SetStringValue("mu_strategy","adaptive");
    CAST_IPOPTAPP(App)->Options()->SetIntegerValue("print_level",verbose);

  //  getBoundsInf(lowerBoundInf,upperBoundInf);

   // translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

    if (max_iter>0)
        CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",max_iter);
//     else
//         CAST_IPOPTAPP(App)->Options()->SetIntegerValue("max_iter",(Index)upperBoundInf);

    if (!useHessian)
        CAST_IPOPTAPP(App)->Options()->SetStringValue("hessian_approximation","limited-memory");

    CAST_IPOPTAPP(App)->Initialize();
    
    SmartPtr<GPR_NLP> nlp=new GPR_NLP(*this, trainingInputs, trainingTargets);

   // nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
   // nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    nlp->set_translational_tol(tol);
  //  nlp->set_callback(iterate);

    ApplicationReturnStatus status=CAST_IPOPTAPP(App)->OptimizeTNLP(GetRawPtr(nlp));

}

} // learningmachine
} // iCub
#else
#include <iostream>
namespace iCub {
namespace learningmachine {
void GPRLearner::optimize(const double tol, const int max_iter, const unsigned int verbose, bool useHessian)
{
    std::cout <<"optimization without IPOPT not implemented yet..."<<std::endl;
    
}

} // learningmachine
} // iCub

#endif



