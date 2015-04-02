#include "SkinCluster.h"


SkinCluster::SkinCluster(){
}

SkinCluster::~SkinCluster(){
}

void    SkinCluster::Init(Matrix & pos, Matrix & normal, double eps, int minPts, double tau){
    mSize       = pos.RowSize();

    mEpsilon    = eps;
    mMinPts     = minPts;
    mTau        = tau;

    mNoise.resize(mSize);
    mVisited.resize(mSize);
    mActive.resize(mSize);
    mClusterId.resize(mSize);
    mClusters.clear();

    mPos    = pos;
    mNormal = normal;

    mPotential.Resize(mSize,false);     mPotential.Zero();
    mOutput.Resize(mSize,false);        mOutput.Zero();
    mInput.Resize(mSize,false);         mInput.Zero();


    mDistance.Resize(mSize,mSize,false);
    mDistance.Zero();
    mWeights.Resize(mSize,mSize,false);
    mWeights.Zero();
    for(int k1=0;k1<mSize;k1++){
        for(int k2=k1;k2<mSize;k2++){
            Vector dst = (mPos.GetRow(k1) - mPos.GetRow(k2));
            double dn = dst.Norm();
            mDistance(k1,k2) = mDistance(k2,k1) = dn;
        }
    }
}

void    SkinCluster::SetWeights(double inSigma, double outSigma){ //0.007 0.01
    for(int k1=0;k1<mSize;k1++){
        for(int k2=k1;k2<mSize;k2++){
            double dn = mDistance(k1,k2);
            mWeights(k1,k2)  = mWeights(k2,k1)  = ((exp(-(dn*dn)/(2.0*inSigma*inSigma))-exp(-(dn*dn)/(2.0*outSigma*outSigma))));  
        }
        mWeights.SetRow(mWeights.GetRow(k1) - mWeights.GetRow(k1).Max(),k1);
        mWeights.SetRow(mWeights.GetRow(k1) * (-double(mSize)/mWeights.GetRow(k1).Sum()),k1);
    }
    mWeights*=0.05;
}

void    SkinCluster::Update(double dt){
    mPotential += ((mPotential*(-1.0)) + (mWeights*mOutput) + (mInput-0.15))*dt/mTau;
    mOutput = mPotential;
    mOutput.Trunc(0,1.0);
}


void    SkinCluster::Prepare(){
	for (int i=0;i<mSize;i++){
        mNoise[i]       = false;
        mVisited[i]     = false;
        mActive[i]      = (mOutput(i)>0.0);
        mVisited[i]     = !mActive[i];
        mClusterId[i]   = 0;
    }
    mClusters.clear();
}
void    SkinCluster::Cluster(){
    int cId = 1;
	for (int i=0;i<mSize;i++){
        if(!mVisited[i]){
            mVisited[i]= true;

            std::vector<int> neighbors;
            GetNeighbors(i,neighbors);

			if (neighbors.size() < mMinPts){
				mNoise[i] = true;
			}else{

                std::vector<int>    cluster;
                cluster.push_back(i);
                mClusterId[i] = cId;

				for (int j=0; j < int(neighbors.size()); j++){
					int cj = neighbors[j];

					if (!mVisited[cj]){
                        mVisited[cj] = true;

                        std::vector<int> jneighbors;
						GetNeighbors(cj, jneighbors);

						if (jneighbors.size() >= mMinPts){
        					for (int k=0; k < int(jneighbors.size()); k++){
                                neighbors.push_back(jneighbors[k]);
                            }
						}
					}
                    if(mActive[cj]){
					    if (mClusterId[cj]==0){
                            cluster.push_back(cj);
                            mClusterId[cj] = cId;
					    }
                    }
                }
                mClusters.push_back(cluster);
				cId++;
            }

        }
    }        
    /*
    cout << "NbClusters: "<<mClusters.size()<<endl;
    for(int i=0;i<mClusters.size();i++){
        cout << "Cluster: ("<<i<<") ";
        for(int j=0;j<mClusters[i].size();j++){
            cout <<mClusters[i][j]<<" ";
        }
        cout << endl;
    }
    */
}

std::vector<int>& SkinCluster::GetNeighbors(int id, std::vector<int> &neighbors){
    neighbors.clear();
    for(int i=0;i<mSize;i++){
        if(i!=id){
            if(mActive[i]){
                if(mDistance(id,i)<mEpsilon)
                    neighbors.push_back(i);
            }
        }
    }
    return neighbors;
}

std::vector<int>& SkinCluster::GetNeighborsGroup(std::vector<int> &input, std::vector<int> &neighbors, double eps){
    double ceps = (eps<0.0?mEpsilon:eps);
    neighbors.clear();
    for(int i=0;i<mSize;i++){
        for(int j=0;j<input.size();j++){
            int k = input[j];
            if(mDistance(k,i)<ceps){
                neighbors.push_back(i);
                break;
            }
        }
    }
    return neighbors;
}


void    SkinCluster::ComputeClusterPos(){
    mClusterPos.Resize(mClusters.size(),3,false);
    mClusterPos.Zero();
    mClusterNormal.Resize(mClusters.size(),3,false);
    mClusterNormal.Zero();
    mClusterResponse.Resize(mClusters.size(),false);
    mClusterResponse.Zero();
    mClusterSecondaryResponse.Resize(mClusters.size(),false);
    mClusterSecondaryResponse.Zero();

    for(int i=0;i<mClusters.size();i++){
        std::vector<int> cCluster;
        GetNeighborsGroup(mClusters[i],cCluster,mEpsilon*1.5);

        Vector mean(3);
        Vector normal(3);
        double sum=0;
        double sum2=0;
        
        double secmean=0;

        for(int j=0;j<cCluster.size();j++){
            mean += mPos.GetRow(cCluster[j])*(mInput.AtNoCheck(cCluster[j]));

            normal += mNormal.GetRow(cCluster[j])*(mInput.AtNoCheck(cCluster[j]));

            sum += (mInput.AtNoCheck(cCluster[j]));
            double trc = (mInput.AtNoCheck(cCluster[j])-0.3);
            sum2 += MAX(0,trc);
            //cout << mInput.AtNoCheck(cCluster[j])<<" ";

            double secv = mSecInput.AtNoCheck(cCluster[j]);
            if(secmean<secv) secmean = secv;
            //secmean += mSecInput.AtNoCheck(cCluster[j]) * mInput.AtNoCheck(cCluster[j]);
        }
        mClusterResponse(i) = sum2;

        //secmean /= sum;
        mClusterSecondaryResponse(i) = secmean;

        mean /= sum;
        normal.Normalize();
        mClusterPos.SetRow(mean,i);
        mClusterNormal.SetRow(normal,i);
    }
}


void                            SkinCluster::SetInput(const Vector& input){mInput = input;mSecInput = input;}
void                            SkinCluster::SetInput(const Vector& input,const Vector& secInput){mInput = input;mSecInput = secInput;}

Vector&                         SkinCluster::GetInput(){return mInput;}
Vector&                         SkinCluster::GetOutput(){return mOutput;}
std::vector<int>&               SkinCluster::GetClusterId(){return mClusterId;}
std::vector<std::vector<int> >& SkinCluster::GetClusters(){return mClusters;}
Matrix&                         SkinCluster::GetClusterPos(){return mClusterPos;}
Matrix&                         SkinCluster::GetClusterNormal(){return mClusterNormal;}
Vector&                         SkinCluster::GetClusterResponse(){return mClusterResponse;}
Vector&                         SkinCluster::GetClusterSecondaryResponse(){return mClusterSecondaryResponse;}



