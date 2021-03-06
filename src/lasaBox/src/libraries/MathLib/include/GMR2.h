#ifndef GMR2_H_
#define GMR2_H_

#include "MathLib.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

// EM default parameters

/* variation threshold for log likelihood EM stopping criterion */
#define EM_STOPPING_CRIT  1e-6  
/* added variance on covariances matrices at each EM round (to prevent those matrices 
   from becoming not invertibles */
#define VARIANCE_KEEPER 1e-6
//#define VARIANCE_KEEPER 0    
/* added variance at the end of EM */
#define VARIANCE_KEEPER_FINAL 0


/**
 * Class describing a gaussian mixture model
 *
 * \ingroup icub_MathLib
 * basic usage (training a 5 state model, with the input vectors in the dataset Matrix):
 * 
 * @code 
 * 
 * GaussianMixture gmm;
 * gmm.initEM_random(5,dataset);
 * gmm.doEM(dataset);
 * @endcode
 * 
 * The result of a training can be save and retrieve from a file
 * through the saveParams and loadParams methods. And we may perform
 * GMR with the doRegression methods.
 */

class GaussianMixture2 {
  
 public :
    int         mNbState;
    int         mDim; 
    Matrix      mMu;
    Matrix     *mSigma;
    Vector      mPriors;

  GaussianMixture2(){mSigma = NULL;};


   /**
    * Initialise the model with parameters stored in a file
    *
    * load the different parameters ( priors, means, covariances matrices ..)
    * from a file generated by the saveParams function, or Matlab
    *
    * \param filename file containing the desired GMM description
    */
  bool loadParams(const char filename[]);

  /**
   *Display current parameters on screen (for debugging purposes)
   */
  void debug(void);
    
  /**
   * Perform Gaussian Mixture regression.
   * 
   * \return a mean vector corresponding to the in vector. 
   * \param inComponents integer vector representing dimension of "in" Vector 
   * \param outComponents integer vector representing dimension of returned Vector 
   * \param SigmaOut will be set to the corresponding covariance Matrix
   */
  double doRegression(const Vector & in,                        
                        const Vector & inComponents,
                        const Vector & outComponents,
                        Vector& out,                        
                        Matrix& SigmaOut,
                        const Matrix * inWeights = NULL
                        );

  double GetPx(const Vector & in,                        
               const Vector & inComponents,
               const Matrix * inWeights = NULL
               );

  double GetGradX(const Vector & in,                        
                  const Vector & inComponents,
                  Vector& out,       
                  const Matrix * inWeights = NULL,
                  Vector *noExpOut = NULL 
                  );
  
  /**
   *Compute probabilty of v ( corresponding to dimension given 
   *in the Components vector) knowing state. 
   */

  double PdfState(const Vector &v, const Vector &Components,int state, const Matrix * weights = NULL);
  
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif

