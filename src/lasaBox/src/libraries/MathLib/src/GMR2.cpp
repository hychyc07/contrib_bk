#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include "float.h"

#include "GMR2.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif


bool GaussianMixture2::loadParams(const char fileName[])
{
    std::ifstream fich(fileName);
    if (!fich.is_open())
        return false;
    fich >> mDim >> mNbState;
    mPriors.Resize(mNbState);
    for (int s = 0; s < mNbState; s++)
        fich >> mPriors[s];
  
    mMu.Resize(mNbState,mDim);
    for (int i = 0; i < mNbState; i++){
        for (int j = 0;j<mDim;j++)
            fich >> mMu(i,j);
    }

    if(mSigma!=NULL) delete [] mSigma;
    mSigma = new Matrix[mNbState];
    for (int s = 0; s < mNbState; s++){
        mSigma[s].Resize(mDim,mDim);
        for (int i = 0; i < mDim; i++){
            for (int j = 0;j<mDim;j++)
                fich >> mSigma[s](i,j);
        }
    }
    fich.close();
    return true;
}

void GaussianMixture2::debug(void)
{
    /*display on std output info about current parameters */
    std::cout << "Nb state : "<< mNbState <<std::endl;
    std::cout << "Dimensions : "<< mDim  <<std::endl;
    std::cout << "Priors : ";
    for(int i = 0;i<mNbState;i++) {
        std::cout << mPriors[i] <<" ";
    }
    std::cout << std::endl;
    std::cout << "Means :";
    mMu.Print();
    std::cout << "Covariance Matrices :"<<endl;
    for(int i =0;i<mNbState;i++) {
        mSigma[i].Print();
    }
}


double GaussianMixture2::PdfState(const Vector &Vin, const Vector &Components,int state,const Matrix * weights) 
{
  /* Compute the probability density function at vector Vin, 
     (given along the dimensions Components), for a given state */ 
    if((state<0)||(state>=mNbState))
        cout << "pdf FOFF"<<endl;

    for(int i=0;i<Components.Size();i++){
        if(Components.At(i)>=mDim)
            cout << "pdf FOFF"<<endl;
    }

    int dim = Components.Size();

    Vector muRow;
    Vector mu;
    Matrix sigma,invSigma;    
    double det;
    double p;
    
    mMu.GetRow(state,muRow);
    muRow.GetSubVector(Components,mu);    
    mSigma[state].GetMatrixSpace(Components,Components,sigma);

    sigma.InverseSymmetric(invSigma,&det);
    if(sigma.IsInverseOk()){
        if(det == 0){
            std::cout << "Warning: null determinant" << std::endl;
            for(unsigned int k=0;k<Components.Size();k++){
                sigma(k,k) += 1e-6;
            }
            sigma.InverseSymmetric(invSigma,&det);
        }
        Vector dmu;
        Vector muTmp;
        Vin.Sub(mu,dmu);
        if(weights!=NULL){
            Vector tmp = dmu;
            dmu = (*weights) * tmp;
        }


        invSigma.Mult(dmu,muTmp);
        p = dmu.Dot(muTmp);
        p = exp(-0.5*p);//sqrt(pow(2*3.14159,dim)*(fabs(det)+DBL_MIN));
        if(p < DBL_MIN) return DBL_MIN;
        else return p;
    }else{
        std::cout << "Error inverting sigma" << std::endl;
        return 0;
    }
}

#define COVMODE2


double GaussianMixture2::doRegression(const Vector & in,                        
                                        const Vector & inComponents,
                                        const Vector & outComponents,
                                        Vector& out,                        
                                        Matrix& outSigma,
                                        const Matrix * inWeights
                                        ){
    int outDim = outComponents.Size();
    int inDim  = inComponents.Size();


    for(int i=0;i<inDim;i++){
        if(inComponents.At(i)>=mDim)
            cout << "doReg FOFF"<<endl;
    }
    for(int i=0;i<outDim;i++){
        if(outComponents.At(i)>=mDim)
            cout << "doReg FOFF"<<endl;
    }

    double retVal = 0.0;

    Vector Pxi(mNbState);
    if(inDim>0){
        double norm=0.0;
        for(int s=0;s<mNbState;s++){
            double p_i = PdfState(in,inComponents,s,inWeights);
            //double p_i = mPriors[s]* PdfState(in,inComponents,s);
            Pxi(s) = p_i;
            norm+= p_i;
        }
        retVal = Pxi.Sum();
        if(norm>1e-30){
            Pxi *= (1.0/norm);
        }
    }else{
        Pxi = 1.0/(double(mNbState));
    }
    //Pxi.Print();
    //Pxi.Normalize();


    Matrix subMuOut;
    Matrix subMuIn;
    mMu.GetColumnSpace(outComponents,   subMuOut);
    mMu.GetColumnSpace(inComponents,    subMuIn);


    out.Resize(outDim,false);
    out.Zero();

    outSigma.Resize(outDim,outDim,false);
    outSigma.Zero();

    //mPriors.Print();
    for(int s=0;s<mNbState;s++){
        Vector cMuOut;
        subMuOut.GetRow(s,cMuOut);
        // cy = mu_o

        Vector dMuIn;
        if(inDim>0){
            subMuIn.GetRow(s,dMuIn);
            dMuIn.SMinus();
            dMuIn += in;
            if(inWeights!=NULL){
                Vector tmpIn = dMuIn;
                dMuIn = (*inWeights) * tmpIn;
            }
        }
        // x-mu
            
        Matrix sigmaInIn;
        Matrix sigmaOutIn;
        if(inDim>0){
            mSigma[s].GetMatrixSpace(inComponents,inComponents,sigmaInIn);
            sigmaInIn.SInverseSymmetric();
            mSigma[s].GetMatrixSpace(outComponents,inComponents,sigmaOutIn);
        }
        Matrix sigmaOutOut;
        mSigma[s].GetMatrixSpace(outComponents,outComponents,sigmaOutOut);
        // (S_ii)^-1 S_oi S_oo


        if(inDim>0){
            Vector dMuInTmp;
            Vector dMuOutTmp;
            sigmaInIn.Mult(dMuIn,dMuInTmp);
            sigmaOutIn.Mult(dMuInTmp,dMuOutTmp);
            cMuOut += dMuOutTmp;
            //cy = mu_o + S_oi(S_ii)^-1(x-mu_i)
        }
        Matrix cSigmaOut;
        Matrix cSigmaOutTmp;
        if(inDim>0){
            sigmaOutIn.Mult(sigmaInIn,cSigmaOutTmp);
            cSigmaOutTmp.MultTranspose2(sigmaOutIn,cSigmaOut);
            cSigmaOut.SMinus();
            cSigmaOut += sigmaOutOut;

            //cSy =  S_oo - (S_oi (S_ii)^-1 S_io)
        }else{
            cSigmaOut = sigmaOutOut;        
        }


#ifdef COVMODE2
            Matrix cSigmaOut2;
            cMuOut.MultTranspose(cMuOut,cSigmaOut2);
            cSigmaOut += cSigmaOut2;
            // cSy = cy*cy' + cSy
            cSigmaOut.ScaleAddTo(Pxi(s),outSigma);
            // Sy = Sy + p * cSy
#else
            cSigmaOut.ScaleAddTo(Pxi(s)*Pxi(s),outSigma);            
            // Sy = Sy + p*p * cSy
#endif

        cMuOut.ScaleAddTo(Pxi(s),out);
        // y = y + p * cy
    }
#ifdef COVMODE2
    Matrix sigmaOut2;
    out.MultTranspose(out,sigmaOut2);
    outSigma -= sigmaOut2;
//    Sigma_y(:,:,i) = sum(Sigma_y_tmp2,3) - y(:,i)*y(:,i)';
#endif

    return retVal;
}


double GaussianMixture2::GetPx(const Vector & in,                        
           const Vector & inComponents,
            const Matrix * inWeights
           ){
    
    double norm=0.0;
    for(int s=0;s<mNbState;s++){
        double p = PdfState(in,inComponents,s,inWeights);
        norm+= p;
    }           
    return norm;
}

double GaussianMixture2::GetGradX(const Vector & in,                        
              const Vector & inComponents,
              Vector& out,
              const Matrix * inWeights,
              Vector *noExpOut
              ){
              
    int inDim  = inComponents.Size();
    out.Resize(inDim,false);
    out.Zero();
              
    Matrix subMuIn;
    mMu.GetColumnSpace(inComponents,    subMuIn);

    if(noExpOut!=NULL){
        noExpOut->Resize(inDim,false);
        noExpOut->Zero();    
    }

    double norm=0.0;

    for(int s=0;s<mNbState;s++){
        double noExp;
        double p = PdfState(in,inComponents,s,inWeights);
        norm+= p;


        Matrix sigmaInIn;

        mSigma[s].GetMatrixSpace(inComponents,inComponents,sigmaInIn);
        sigmaInIn.SInverseSymmetric();
        
        Vector dMuIn;
        subMuIn.GetRow(s,dMuIn);
        dMuIn.SMinus();
        dMuIn += in;
        if(inWeights!=NULL){
            Vector tmpIn = dMuIn;
            dMuIn = (*inWeights) * tmpIn;
        }

        Vector g;
        sigmaInIn.Mult(dMuIn,g);
        
        g.ScaleAddTo(p,out);

        if(noExpOut!=NULL){
            g.ScaleAddTo(1.0,*noExpOut);
        }        
    }           
    return norm;
}              




