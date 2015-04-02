#include "GaussianProbabilityDistribution.h"

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

GaussianProbabilityDistribution::GaussianProbabilityDistribution(int count) :
  ProbabilityDistribution(count)
{
  
  covar.resize(count,count);
  mean.resize(count);

  covar.eye();
  mean.zero();

  Random::seed(Time::now());

}
    
GaussianProbabilityDistribution::~GaussianProbabilityDistribution() {
  for(unsigned int i=0; i<data.size(); i++) {
    if(data[i]!=NULL) delete data[i];
  }
  data.clear();
}
    
int GaussianProbabilityDistribution::addSample(Vector sample, double val) {

  if(sample.size() != dimension_count) {
    cout << "GaussianProbabilityDistribution::addSample() -- size mismatch!!" << endl;
    return 0;
  }

  Vector *v = new Vector(dimension_count);
  for(int i=0; i<dimension_count; i++) {
    (*v)[i] = sample[i]*val;
    // update mean
    mean[i] = mean[i] + (1.0/(data.size()+1.0))*(sample[i] - mean[i]);
  }
  data.push_back(v);
 
  // update covariance
  computeCovarFromData();

  return 1;
  
}

int GaussianProbabilityDistribution::drawSample(yarp::sig::Vector &sample) {
  
  Vector S(dimension_count);
  Matrix U(dimension_count,dimension_count);
  Matrix V(dimension_count,dimension_count);  

  Matrix Vinv(dimension_count,dimension_count);  
  Matrix S2(dimension_count,dimension_count);
  Matrix A(dimension_count,dimension_count);  
  Matrix AZ(dimension_count,dimension_count);  

  Matrix mu(dimension_count,1);  
  Matrix Z(dimension_count,1);  
  Matrix X(dimension_count,1);  
  
  SVD(covar, U, S, V);

  Vinv = luinv(V);
  S2.zero();
  
  for(int i=0; i<dimension_count; i++) {
      S2[i][i] = sqrt(S[i]);
      mu[i][0] = mean[i];
      Z[i][0] = Random::normal(0.0, 1.0);
  }
 
  A = U*S2*Vinv;
  X = mu + A*Z;

  sample.resize(dimension_count);
  for(int i=0; i<dimension_count; i++) {
      sample[i] = X[i][0];
  }

  return 1;
}   

int GaussianProbabilityDistribution::loadDistribution(std::string str) {

    cout << "loading distribution..." << endl;
    
    FILE *fid;
    int dims;
    double v;

    if( !(fid = fopen(str.c_str(),"r")) ) {
        cout << "NonParametricProbabilityDistribution::loadDistribution() -- can't open file=\'" << str.c_str() << "\'!!" << endl;
        return 0;
    }
    
    char *type;
    int i,j=0;

    type = (char *)malloc(32);
    fscanf(fid,"%s %d\n", type, &dims);

    if(!strcmp(type,"Gaussian")) {
        cout << "found Gaussian Distribution of size = " << dims << endl;
    } else {
        return 0;
    }
    
    dimension_count = dims;    

    cout << "mean: " << endl;
    for(i=0; i<dims; i++) {
      fscanf(fid,"%lf\t",&v);
      mean[i] = v;
      cout << v << endl;
    }

    cout << "covar: " << endl;
    for(i=0; i<dims; i++) {
      for(j=0; j<dims; j++) {
	fscanf(fid,"%lf\t",&v);
	covar[i][j] = v;
	cout << v << " ";
      }
      cout << endl;
    }

    while( fscanf(fid,"%lf\t",&v) != EOF) {
      //      cout << "data point: " << endl;
      Vector *Vd = new Vector(dims);
      (*Vd)[0] = v; 
      //cout << v << endl;
      for(i=1; i<dims; i++) {
	fscanf(fid,"%lf\t",&v);
	(*Vd)[i] = v;
	//cout << v << endl;
      }
      data.push_back(Vd);
    }
    
    fclose(fid);

    return 1;
} 

int GaussianProbabilityDistribution::saveDistribution(std::string str) {

    FILE *fid;
    
    cout << "saving distribution[size=" << data.size() << "]: " << str.c_str() << endl;
    if( (fid = fopen(str.c_str(),"w"))==NULL) {
      cout << "GaussianProbabilityDistribution::saveDistribution() -- can't open file=\'" << str.c_str() << "\'!!" << endl;
      return 0;
    }
    
    fprintf(fid,"Gaussian %d\n",dimension_count);
    
    for(int i=0; i<dimension_count; i++) {
      fprintf(fid,"%lf\t", mean[i]);
    }
    fprintf(fid,"\n");

    for(int i=0; i<dimension_count; i++) {
      for(int j=0; j<dimension_count; j++) {
	fprintf(fid,"%lf\t", covar[i][j]);
      }
      fprintf(fid,"\n");
    }

    for(unsigned int k=0; k<data.size(); k++) {
      for(int i=0; i<dimension_count; i++) {
	fprintf(fid,"%lf\t", (*data[k])[i]);
      }
      fprintf(fid,"\n");      
    }
    fclose(fid);
    return 1;

}
    
double GaussianProbabilityDistribution::getProbability(Vector sample) {

  Vector diff = sample - mean;
  Matrix covar_inv = luinv(covar);
  double covar_det = det(covar);
  double alpha, gamma;
  
  Matrix Mdiff(dimension_count,1);
  Matrix MdiffT(1,dimension_count);
  Matrix beta(1,1);
  Matrix Mtmp;

  for(int i=0; i<dimension_count; i++) {
    Mdiff[i][0] = diff[i];
    MdiffT[0][i] = diff[i];
  }
  
  alpha = 1.0/(pow(2*M_PI,dimension_count/2.0)*sqrt(covar_det));
  beta = MdiffT*covar_inv*Mdiff;
  gamma = exp(-beta[0][0]/2.0);

  return alpha*gamma;
    
}

void GaussianProbabilityDistribution::computeMeanFromData() {

  mean.zero();

  // estimate mean
  for(unsigned int i=0; i<data.size(); i++) {
    for(int k=0; k<dimension_count; k++) {
      mean[k] += (*data[i])[k];
    }
  }
  for(int k=0; k<dimension_count; k++) {
    mean[k] /= data.size();
  }

}

void GaussianProbabilityDistribution::computeCovarFromData() {

  // estimate covariance
  Matrix Mtmp(dimension_count, dimension_count);
  Matrix Mdiff(dimension_count,1);
  Matrix MdiffT(1,dimension_count);

  covar.zero();

  for(unsigned int i=0; i<data.size(); i++) {
    for(int k=0; k<dimension_count; k++) {
       Mdiff[k][0] = (*data[i])[k] - mean[k];
       MdiffT[0][k] = Mdiff[k][0];
    }
    Mtmp = Mdiff*MdiffT;
    covar = covar+Mtmp;
  }

  for(int i=0; i<dimension_count; i++) {
    for(int j=0; j<dimension_count; j++) {
      covar[i][j] = covar[i][j] / (double)data.size();
    }
  }

}
