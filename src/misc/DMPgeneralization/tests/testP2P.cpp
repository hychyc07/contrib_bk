#include <iCub/DMP/DMPPointToPoint.h>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace yarp::os;

/**********************************************************/
int main(int argc, char *argv[])
{
    double alpha_x=2;
    double alpha_z=20;
    double beta_z=5;
    int N=15;
    //make up fake trajectory
    double samplingStep=0.01;
    int nSamples=300;
    int nDofs=1;
    
    dmp::Trajectory myTrajectory(nDofs, nSamples);
    yarp::sig::Vector times(nSamples);
    for (int i=0; i<nSamples;++i)
        times[i]=i*samplingStep;
   // myTrajectory.set_time_vector(times);
    yarp::sig::Vector dof0(nSamples);
    //myTrajectory.dofs.resize(nSamples, nDofs);
    yarp::sig::Vector startingPoints(nDofs);
    startingPoints(0)=0.3;
    for (int k=0; k<nDofs; ++k)
    {
        for (int i=0; i<nSamples;++i)
        {
            dof0(i)=dof0(0)+beta_z*sin((double) i/(double) nSamples*2*M_PI/2);
            myTrajectory.add_trajectory_point(times[i], yarp::sig::Vector(1,dof0[i]));   
        }
     //   myTrajectory.set_trajectory_for_dof(k, dof0);//myTrajectory.dofs(i, k)=myTrajectory.dofs(i-1, k)+alpha_z*samplingStep*cos(i/nSamples*2*M_PI);
    }
  
//     std::cout << "Trajectory\n Time:\n" << myTrajectory.time.toString() << "Values:\n" << myTrajectory.dofs.toString(); 
    std::cout << "Trajectory\n Values:\n" << myTrajectory.get_trajectory_for_dof(0).toString()<<std::endl; 
    
    DMPPointToPoint myDMP(nDofs,N, alpha_x, alpha_z, beta_z);
    std::cout << "created dmp"<< myDMP <<std::endl; fflush(stdout);
    bool estOk=myDMP.estimate_from_trajectory(myTrajectory);
    if (!estOk)
    {
        std::cout << "estimation failed!" << std::endl;
        return 0;
    }
    std::cout << myDMP;
    myDMP.resetState();
    startingPoints(0)=0.1;
    myDMP.set_positions(startingPoints);
    size_t sampleCt=0;
    while (sampleCt<400)
    {  
        if (!myDMP.integrate(samplingStep))
        {
            std::cout <<"integration error" <<std::endl;
            return 0;
        }
        sampleCt++;
        std::cout << myDMP.get_positions().toString()<< " ";
    }
    std::cout <<std::endl;
    return 0;
}
