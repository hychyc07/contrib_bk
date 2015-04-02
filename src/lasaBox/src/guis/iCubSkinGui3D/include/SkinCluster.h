#ifndef SKINCLUSTER_H_
#define SKINCLUSTER_H_

#include "MathLib/MathLib.h"
using namespace MathLib;

class SkinCluster
{
public:

    Matrix  mPos;
    Matrix  mNormal;

    Vector  mValue;

    Matrix  mDistance;
    Matrix  mWeights;

    Vector  mPotential;
    Vector  mOutput;
    Vector  mInput;

    Vector  mSecInput;

    double  mTau;

	double              mEpsilon;
	int                 mMinPts;
    int                 mSize;

	std::vector<bool>   mNoise;
	std::vector<bool>   mVisited;
	std::vector<bool>   mActive;

	std::vector<std::vector<int> >  mClusters;
	std::vector<int>                mClusterId;

    Matrix                          mClusterPos;
    Matrix                          mClusterNormal;
    Vector                          mClusterResponse;
    Vector                          mClusterSecondaryResponse;

public:
    SkinCluster();
    ~SkinCluster();

    void    Init(Matrix & pos, Matrix & normal, double eps, int minPts, double tau);
    void    SetWeights(double inSigma, double outSigma);   

    void    Update(double dt);

    void    Prepare();
    void    Cluster();

    void    ComputeClusterPos();

    void                            SetInput(const Vector& input);
    void                            SetInput(const Vector& input,const Vector& secInput);

    Vector&                         GetInput();
    Vector&                         GetOutput();
	std::vector<int>&               GetClusterId();
	std::vector<std::vector<int> >& GetClusters();
	Matrix&                         GetClusterPos();
	Matrix&                         GetClusterNormal();
    Vector&                         GetClusterResponse();
    Vector&                         GetClusterSecondaryResponse();

    std::vector<int>& GetNeighbors(int id, std::vector<int> &neighbors);

    std::vector<int>& GetNeighborsGroup(std::vector<int> &input, std::vector<int> &neighbors, double eps=-1);

};

#endif

