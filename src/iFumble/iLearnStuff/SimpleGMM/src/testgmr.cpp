#include <iostream>
#include "SimpleGMR.h"
//#include "Matrix.h"
//#include "MathLib.h"
#include "LasaMatrix.h"
#include <sstream>
#include "yarp/os/Time.h"
//#include "Vector.h"
using namespace yarp;
using namespace yarp::os;
//using namespace yarp::sig;
//using namespace yarp::dev;
//using namespace yarp::math;
using namespace MathLib;

int testhighdim(int argc, char **argv) ;
int testlowdim(int argc, char **argv) ;


int main(int argc, char **argv) {
 testlowdim(argc,argv);
 testhighdim(argc,argv);
 
}

int testhighdim(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl	;
    
    int n_samples=20;
    int n_states=5;
    int dim=10;
    
    Matrix data(n_samples,dim);
    
    for(int sample=0;sample<n_samples;sample++){
	std::cout<<"sample no "<<sample;
	Vector learnvec(dim);
	learnvec[0]=((double)rand())/RAND_MAX;
	for(int dimno=1;dimno<dim;dimno++){
	  double randn=((double)rand())/RAND_MAX;  
	  double randn2=((double)rand())/RAND_MAX;  
	  if(randn<0.5)
	    learnvec[dimno]=0.5*learnvec[dimno-1]+0.5*randn2;
	  else
	    learnvec[dimno]=0.5- 0.5*learnvec[dimno-1]+0.5*randn2;
	}
	
	std::cout<<"learnvec is "<<learnvec<<std::endl;
	data.SetRow(learnvec,sample);
    } 
    std::cout<<"Now do learning"<<std::endl; 
    
    NewGaussianMixture gmm;
    double nowtime1=Time::now();  
    gmm.initEM_random(n_states,data);
    //gmm.initEM_kmeans(n_states,data);
    
    double nowtime2=Time::now();
    gmm.doEM(data);
    double nowtime3=Time::now(); 
    std::cout<<"INIT TOOK:"<<(nowtime2-nowtime1)<<"MS AND EMM TOOK "<<(nowtime3-nowtime2)<<"MS"<<std::endl;
    //for(int component=0;<component<gmm.nState;component++){
//	std::cout<<"COMPONENT: "<<gmm.
    //}
    gmm.saveParams("mystatebig.txt");
    //return 0;

std::cout<<"...."<<std::endl;
std::cout<<"...."<<std::endl; 

Time::delay(1);

  for(int regtry=1;regtry<dim;regtry++){
    Vector componentsknown(regtry);
    Vector dataknown(regtry);
    for(int i=0;i<regtry;i++){
	int attempt=rand()%dim;
	bool found=false;
	for(int j=i-1;j<i;j++){
	    if(componentsknown[j]==attempt)found=true;
	}
	if(found){
	  i--;
	}else{
	    componentsknown[i]=attempt;
	    dataknown[i]=((double)rand())/RAND_MAX;
	}
    }
    Vector componentsunknown(dim-regtry);
    for(int i=0;i<dim-regtry;i++){
	int attempt=rand()%dim;
	bool found=false;
	for(int j=i-1;j<i;j++){
	    if(componentsunknown[j]==attempt)found=true;
	}
	for(int j=0;j<regtry;j++){
	    if(componentsknown[j]==attempt)found=true;
	}	
	if(found){
	  i--;
	}else{
	    componentsunknown[i]=attempt;
	}
    }    
    Matrix newsigma;
    nowtime1=Time::now();  
    	Vector outcomps=gmm.doRegression(dataknown,
               newsigma,
		componentsknown,
               componentsunknown);
	       nowtime2=Time::now();
    std::cout<<"REGRESSION ("<<regtry<<" TOOK:"<<(nowtime2-nowtime1)<<std::endl;	       
    
  }

	//std::cout<<"regression got us "<<outcomps[0]<<" with variance "<<newsigma.At(0,0)<<std::endl;
	//NewGaussianMixture *mynewgaussm=gmm.conditionalMixture(v2,in,out);

}


int testlowdim(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl	;
    
    int n_samples=252;
    int n_states=2;
    
    Matrix data(n_samples,2);
    for(int sample=0;sample<n_samples;sample++){
      std::cout<<"sample no "<<sample;
	double randn=rand();
	randn=randn/RAND_MAX;
	double x,y,ydiff;
	x=((double)rand())/RAND_MAX;
	ydiff=0.1*((double)rand())/RAND_MAX;
	if(randn>0.5){
	    y=x+ydiff;
	}else{
	    y=1+ydiff-x;
	}
	Vector v(2); 
	v[0]=x; 
	v[1]=y; 
	std::cout<<" is "<<v[0]<<","<<v[1]<<std::endl;
	data.SetRow(v,sample);
    } 
    std::cout<<"Now do learning"<<std::endl; 
    
    NewGaussianMixture gmm;
    double nowtime1=Time::now();  
    gmm.initEM_random(n_states,data);
    double nowtime2=Time::now();
    gmm.doEM(data);
    double nowtime3=Time::now(); 
    std::cout<<"INIT TOOK:"<<(nowtime2-nowtime1)<<"MS AND EMM TOOK "<<(nowtime3-nowtime2)<<"MS"<<std::endl;
    //for(int component=0;<component<gmm.nState;component++){
//	std::cout<<"COMPONENT: "<<gmm.
    //}
    gmm.saveParams("mystate.txt");
/*    for(int sample=0;sample<n_samples;sample++){
	double x,y,yexpt,ydiff; 
	x=((double)rand())/RAND_MAX;
	yexpt=x;
	ydiff=((double)rand())/RAND_MAX;
	y=x+ydiff;
	Vector v(2);
	v[0]=x; 
	v[1]=yexpt; 	
	double pdf1=gmm.pdf(v);
	v[1]=y;
	double pdf2=gmm.pdf(v);
	std::cout<<"p("<<x<<"/"<<yexpt<<")="<<pdf1<<std::endl;
	std::cout<<"p("<<x<<"/"<<y<<")="<<pdf2<<std::endl;
    }*/
std::cout<<"...."<<std::endl;
std::cout<<"...."<<std::endl; 
  for(int xint=0;xint<22;xint++){
      	Vector v(2);
	double x=((double)xint)/20;
	v[0]=x;     
	
	//Matrix newsigma[n_states];
	//for(int i=0;i<n_states;i++)newsigma[i].Resize(1,1);
	Matrix newsigma(1,1);
	Vector v2(1);v2[0]=x;
	Vector in(1);in[0]=0;
	Vector out(1);out[0]=1;
	Vector outcomps=gmm.doRegression(v2,
               newsigma,
               in,
               out);
	//std::cout<<"regression got us "<<outcomps[0]<<" with variance "<<newsigma.At(0,0)<<std::endl;
	NewGaussianMixture *mynewgaussm=gmm.conditionalMixture(v2,in,out);
	ostringstream os;
	os<<"mystate"<<xint<<".txt";
	mynewgaussm->saveParams(os.str().data());
	//std::cout<<"conditional done"<<std::endl;
	std::string gaussstring=mynewgaussm->toStr();
	//std::cout<<"MYNEWGAUSSM IS "<<gaussstring;	
    for(int yint=0;yint<22;yint++){
	 
	double y=((double)yint)/20;

	v[1]=y;
	/*Matrix newy(1,1);
	Vector in(1);in[0]=1;
	Vector out(1);out[1]=1;
	Vector outcomps=gmm.doRegression(v,
               newy,
               in,
               out);*/
	double res;
	if(1){
	  Vector v3(1);v3[0]=y;
	  res=mynewgaussm->pdf(v3);
	// std::cout<<"y is "<<y<<" RES IS "<<res<<std::endl;
      }else{
	res=gmm.pdf(v);
      }
	//double res=gmm.pdf(v);
	int toout = ((int)(res*4));
	std::cout<<toout<<" ";
	//std::cout<<((int)(outcomps[0]*20))<<" ";
	//std::cout<<outcomps[0]<<std::endl;
	
    }
    std::cout<<std::endl;
    delete mynewgaussm;
  }
}

