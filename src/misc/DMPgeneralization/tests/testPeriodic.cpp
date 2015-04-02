#include <iCub/DMP/DMPPeriodic.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

using namespace yarp::os;
using namespace yarp::math;
/**********************************************************/
int main(int argc, char *argv[])
{
    
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;
    
    yarp::os::BufferedPort<yarp::sig::Vector> origPort;
    origPort.open("/testPeriodic/orig");
    yarp::os::BufferedPort<yarp::sig::Vector>  integratedPort;
    integratedPort.open("/testPeriodic/DMP");
    double alpha_x=3;
    double alpha_z=20;
    double beta_z=5;
    int N=15;
    //make up fake trajectory
    double samplingStep=0.01;
    int nSamples=1300;
    int nDofs=1;
    
    while(origPort.getOutputCount()<1 && integratedPort.getOutputCount()<1)
    {
        std::cout<<"waiting for connection on ports...\n";
        yarp::os::Time::delay(0.3);
    }
    
    
    dmp::Trajectory myTrajectory(nDofs, nSamples);
    yarp::sig::Vector times(nSamples);
    for (int i=0; i<nSamples;++i)
        times[i]=i*samplingStep;
  //  myTrajectory.set_time_vector(times);
    yarp::sig::Vector dof0(nSamples);
    //myTrajectory.dofs.resize(nSamples, nDofs);
    yarp::sig::Vector startingPoints(nDofs);
    startingPoints(0)=0.3;
    double frequency=1/alpha_x*3;
    for (int k=0; k<nDofs; ++k)
    {
        for (int i=0; i<nSamples;++i)
        {
          //  dof0(i)=startingPoints(0)+beta_z*sin((double) i/((double) nSamples/(alpha_x*3))*2*M_PI)+double(rand() % 100-50)/50/2;
           dof0(i)=startingPoints(0)+beta_z*sin(times[i]*(frequency*2*M_PI))+double(rand() % 100-50)/50/2;; // frequency is 1/alpha_x
          myTrajectory.add_trajectory_point(times[i], yarp::sig::Vector(1,dof0[i]));   
        }
      //  myTrajectory.set_trajectory_for_dof(k, dof0);//myTrajectory.dofs(i, k)=myTrajectory.dofs(i-1, k)+alpha_z*samplingStep*cos(i/nSamples*2*M_PI);
    }
  
//     std::cout << "Trajectory\n Time:\n" << myTrajectory.time.toString() << "Values:\n" << myTrajectory.dofs.toString(); 
    std::cout << "Trajectory\n Values:\n" << myTrajectory.get_trajectory_for_dof(0).toString()<<std::endl; 

  /*  
    dmp::Trajectory myTrajectory;
    
    myTrajectory.time.resize(nSamples);
    for (int i=0; i<nSamples;++i)
        myTrajectory.time(i)=i*samplingStep;
    
    myTrajectory.dofs.resize(nSamples, nDofs);
    yarp::sig::Vector startingPoints(nDofs);
    startingPoints(0)=0.3;
    
    for (int k=0; k<nDofs; ++k)
    {
      //  myTrajectory.dofs(0, k)=startingPoints(k);
        for (int i=0; i<nSamples;++i)
        {
            myTrajectory.dofs(i, k)=startingPoints(0)+beta_z*sin((double) i/((double) nSamples/alpha_x)*2*M_PI)+double(rand() % 100-50)/50/2;
                    }
            //myTrajectory.dofs(i, k)=myTrajectory.dofs(i-1, k)+alpha_z*samplingStep*cos(i/nSamples*2*M_PI);
    }*/

    yarp::sig::Vector startingVels(nDofs);
    startingVels(0)=alpha_x*beta_z;
//     std::cout << "Trajectory\n Time:\n" << myTrajectory.time.toString() << "Values:\n" << myTrajectory.dofs.toString(); 
 //   std::cout << "Trajectory\n Values:\n" << myTrajectory.dofs.toString(-1, -1, "\t"); 
    
    DMPPeriodic myDMP(nDofs,N, alpha_z, beta_z); //ATT alpha_x should be called omega...
    std::cout << "created dmp"<< myDMP <<std::endl; fflush(stdout);
    bool estOk=myDMP.estimate_from_trajectory(myTrajectory);
    if (!estOk)
    {
        std::cout << "estimation failed!" << std::endl;
        return 0;
    }

    myDMP.resetState();
   // startingPoints(0)=0.1;
    myDMP.set_positions(startingPoints);
    myDMP.set_velocities(startingVels);
    
    std::cout << "newDMP: "<< myDMP;
    size_t sampleCt=0;
    while (sampleCt<2500)
    {  
        if (!myDMP.integrate(samplingStep))
        {
            std::cout <<"integration error" <<std::endl;
            return 0;
        }
        sampleCt++;
        std::cout << myDMP.get_positions().toString()<< " ";
        
        if (sampleCt<nSamples)
        {
        yarp::sig::Vector& origToSend = origPort.prepare();
            origToSend.resize(1);
            origToSend[0]=myTrajectory.get_trajectory_for_dof(0)[sampleCt];
            
        }
        
        yarp::sig::Vector& dmpToSend=integratedPort.prepare();
     dmpToSend.resize(1);
        dmpToSend[0]= myDMP.get_positions()[0];
        
        integratedPort.writeStrict();
        origPort.writeStrict();
        yarp::os::Time::delay(0.01);
      
    }
    std::cout <<std::endl;
    origPort.close();
    integratedPort.close();
    
    return 0;
}
