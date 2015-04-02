/*
 * JointImpedance.cpp
 *
 *  Created on: Jul 23, 2010
 *      Author: naveenoid
 */




#include "JointImpedance.hpp"

#include "controller.hpp"
#include <yarp/os/Value.h>
#include <yarp/os/ConstString.h>
#include <iCub/iDyn/iDyn.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include "gazeController.hpp"


/*
 * Project Defines
 */

/*
 * Additional Defines
 */
#include<iostream>

/*
 * Namespaces
 */

#include <yarp/math/Math.h>
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iKin;


/* Const Vectors */



namespace HandSome {

//static const Vector leftBestPose(-0.074778, 0.7859, -0.6138, 2.8824]);
//static const Vector rightBestPose(-0.1102,-0.6159,0.780,2.926]);


//JointImpedance::JointImpedance(): RateThread(0)
//{
//	driver = NULL;
//	arm = NULL;
//
//}
//
//JointImpedance::JointImpedance(int period): RateThread(period*1000)
//{
//	driver = NULL;
//	arm = NULL;

//}$


JointImpedance::JointImpedance(PolyDriver *pd1 , PolyDriver *pd2 ,ControllerThd *ctr,GazeControlThd *gc,double period, double rl): CartesianImpedance(pd1,pd2,period)
		//

{
	driver1 = pd1;
	driver2 = pd2;

	restLength = rl;

	gct = gc;

	ctrl = ctr;
        
std::cout<<"I received restlength "<<restLength<<std::endl;

	std::cout<<"I received the PolyDrivers : Im so cool Bitches!\n";
//
//    client1->view(arm1);
//    client2->view(arm2);

	desiredPose1.resize(7);
	desiredPose2.resize(7);

//	Value v1 = driver1->getValue("local");
//	ConstString temp1 = v1.asString();
//	ConstString temp2 = temp1.substr(temp1.find("left_arm"));

//	iCubArmDyn
	arm1 = new iCubArmDyn("left");//"left");

	arm1->blockLink(0);
	arm1->blockLink(1);
	arm1->blockLink(2);


	arm2 = new iCubArmDyn("right");//"left");

	arm2->blockLink(0);
	arm2->blockLink(1);
	arm2->blockLink(2);


   driver1->view(ienc1);
   driver2->view(ienc2);

   // IKin Solver Chain

	chain1 = dynamic_cast<iKinChain *>(arm1->asChain());

	chain2 = dynamic_cast<iKinChain *>(arm2->asChain());


}

JointImpedance::~JointImpedance() {
	delete(arm1);
	delete(arm2);

	delete(ienc1);
	delete(ienc2);

}

bool JointImpedance::threadInit() {
	std::cout<<"I just got initialised";


	return true;
}

void JointImpedance::run() {

	 std::cout<<"\nL : ";

	 currentPose1 = getCartesian(ienc1, arm1);

	 std::cout<<"\tR : ";

	 currentPose2 = getCartesian(ienc2, arm2);

	 std::cout<<"\t C :";

            if(restLength==0) {
	 computeDesired();
	}
            else
	{
                computeDesiredVariable();
        }
      Vector temp;

     for(int i = 0 ; i<3; i++)
     {
       	temp[i] = desiredPose1[i];
     }

     gct->setFixationPoint(temp);
	 computeJointSpace();
}

void JointImpedance::threadRelease()
{

	std::cout<<"I just completed";

//    delete(arm1);
}

Vector JointImpedance::getCartesian(IEncoders *ienc, iCubArmDyn *arm)
{
	   Vector encoders;
	   int jnt;

	   ienc->getAxes(&jnt);

	   encoders.resize(jnt);
	   ienc->getEncoders(encoders.data());
	   Vector q(7);
	   for(int i=0;i<7;i++)
	   {
		   q[i] = encoders[i]*CTRL_DEG2RAD;
	   }

	   arm->setAng(q);

	   Vector temp = arm->EndEffPose();

	   for(int i = 0; i< 3; i++)
	   {
		   std::cout<<temp[i]<<" ";
	   }

	   return(temp);

}
void JointImpedance::computeDesired(void)
{

	static const double leftBestPose[] = {-0.074778, 0.7859, -0.6138, 2.8824};
	static const double rightBestPose[] = {-0.1102,-0.6159,0.780,2.926};

	Vector centerPose;

	centerPose.resize(currentPose1.size());

	for(int i = 0; i< 3; i++)
	{
		double num = 0.5*(double(currentPose1[i]) + double(currentPose2[i]));
//		std::cout<<num <<" ";
		centerPose[i] = num;

		std::cout<<centerPose[i]<<" ";

		//		desiredPose1[i] = num;//centerPose[i];
//		desiredPose2[i] = num;//centerPose[i];
	}

	desiredPose1 = centerPose;
	desiredPose2 = centerPose;

        for(int i = 3;i<centerPose.size();i++)
        {
                desiredPose1[i] = leftBestPose[i-3];
                desiredPose2[i] = rightBestPose[i-3];
        }

}

void JointImpedance::computeDesiredVariable(){



    static const double leftBestPose[] = {-0.074778, 0.7859, -0.6138, 2.8824};
    static const double rightBestPose[] = {-0.1102,-0.6159,0.780,2.926};


	Vector CurrentPose1 = currentPose1;
	 Vector CurrentPose2 = currentPose2;
    double rl=restLength/2;
    float alfa,psi,xr,yr,zr,xl,yl,zl;
    double ro;
    Vector i(3),j(3),k(3);

    i[0]=1; i[1]=0; i[2]=0;
    j[0]=0; j[1]=1; j[2]=0;
    k[0]=0; k[1]=0; k[2]=1;




    Vector difVect;
  //  cout<<"difVect____"<<difVect.toString();
    double xComp,yComp,zComp;



    difVect=currentPose2-currentPose1;

    xComp=dot(i,difVect);
    //cout<<"xComp___"<<xComp;

    yComp=dot(j,difVect);
   // cout<<"    yComp___"<<yComp;

    zComp=dot(difVect,k);
   // cout<<"     zComp:"<<zComp;
    double XY[3]={xComp,yComp,0};

    Vector vXY(3,XY);




    //difx=difVect[0];
    //dify=difVect[1];
    //difz=difVect[2];

    //cout<< "This is the difVector between 2 hands:"<<difVect.toString();
    //cout<<endl;
    ro=sqrt(pow(difVect[0],2)+pow(difVect[1],2)+pow(difVect[2],2));

    //std::cout<< "this is the  distance between two hands  "<<ro;
    //std::cout<<endl;
    alfa=acos(zComp/ro);
    //alfa=acos(ro/ro);

    double alfaDeg=alfa*180/M_PI;
    //cout<< "Alfa is----------"<<alfaDeg;
    //cout<<endl;

    //cout<<"norma di V_______"<<norm2(vXY);
    //cout<<endl;
    
    double proXY=dot(i,vXY);
    psi=acos(proXY/sqrt(pow(vXY[0],2)+pow(vXY[1],2)+pow(vXY[2],2)));
    //cout<<"PSI_____"<<psi*180/M_PI;
    //cout<<endl;
    //psi=asin( (ro/ro)*(1/sin(acos(alfa))));

    double zMean=(ro*cos(alfa))/2;
    //cout<< "zMean is_______  "<<zMean;
    //cout<<endl;

    double xMean=(ro*sin(alfa)*cos(psi))/2;
    //cout<<"xMean is_______  "<<xMean;
    //cout<<endl;

    double yMean=(ro*sin(alfa)*sin(psi))/2;
    //cout<<"yMean is________ "<<yMean;
    //cout<<endl;

    double deltaZ=rl*cos(alfa);

    double deltaX=rl*sin(alfa)*cos(psi);

    double deltaY=rl*sin(alfa)*sin(psi);

    xr=xMean+deltaX;
    yr=yMean+deltaY;
    zr=zMean+deltaZ;

    desiredPose2[0]=xr; desiredPose2[1]=yr; desiredPose2[2]=zr;


    xl=xMean-deltaX;
    yl=yMean-deltaY;
    zl=zMean-deltaZ;

   desiredPose1[0]=xl; desiredPose1[1]=yl; desiredPose1[2]=zl;

 for(int i = 3;i<7;i++)
 {
         desiredPose1[i] = leftBestPose[i-3];
         desiredPose2[i] = rightBestPose[i-3];
 }

 std::cout<< "\nDes2:  "<<desiredPose2.toString();
 //std::cout <<endl;

 std::cout<< "\nDes1:  "<<desiredPose1.toString();
 std::cout <<std::endl;


    /*
    float psiDeg=psi*180/3.1415;
    cout<< "Psi is-----------"<<psiDeg;
    cout<<endl;

    cout<< "****this are the first new XYZ coordinate******";
    cout<<endl;

    //for RightHand;
    xr=ro*sin(alfa)*cos(psi);
    //xr=(restLength/2+ro/2)*sin(alfa)*cos(psi); */

/*
    cout<<"++++++Xr====="<<xr;
    cout<<endl;

    //yr=(restLength/2+ro/2)*sin(alfa)*sin(psi);
    cout<<"++++++Yr====="<<yr;
    cout<<endl;

    //zr=(restLength/2+ro/2)*cos(alfa);
    cout<<"++++++Zr====="<<zr;
    cout<<endl;
    //for LeftHand;

    //xl=(ro/2-restLength)*sin(alfa)*sin(psi);
    cout<<"++++++Xl====="<<xl;
    cout <<endl;

    //yl=(ro/2-restLength/2)*sin(alfa)*cos(psi);
    cout<<"++++++Yl====="<<yl;
    cout<<endl;

    //zl=(ro/2-restLength/2)*cos(alfa);
    cout<<"++++++Zl====="<<zl;
    cout<<endl; */



}

void JointImpedance::computeJointSpace(void)
{

	Vector q0_1,qf_1,qhat_1,xf_1,xhat_1, qhat_1Deg;
	Vector q0_2,qf_2,qhat_2,xf_2,xhat_2, qhat_2Deg;

	xf_1 = desiredPose1;
	xf_2 = desiredPose2;

	Vector dummy(1);

// 	q0_1=chain1->getAng();
// 	q0_2=chain2->getAng();
//
// 	  // dump DOF bounds using () operators and set
// 	  // a second joints configuration in the middle of the compact set.
// 	  // Remind that angles are expressed in radians
//    qf_1.resize(chain1->getDOF());
// 	 for (unsigned int i=0; i<chain1->getDOF(); i++)
// 	 {
// 	     double min_1=(*chain1)(i).getMin();
// 	     double max_1=(*chain1)(i).getMax();
//         qf_1[i]=(min_1+max_1)/2.0;
//
//	     double min_2=(*chain2)(i).getMin();
// 	     double max_2=(*chain2)(i).getMax();
// 	 	       qf_2[i]=(min_2+max_2)/2.0;
//
//
// 	       // last joint set to 1° higher than the bound
//       if (i==chain1->getDOF()-1)
//            qf_1[i]=max_1+1.0*(M_PI/180.0);
//
//
//       if (i==chain2->getDOF()-1)
//                   qf_2[i]=max_2+1.0*(M_PI/180.0);
//
//// 	        cout << "joint " << i << " in [" << (180.0/M_PI)*min << "," << (180.0/M_PI)*max
//// 	             << "] set to " << (180.0/M_PI)*qf[i] << endl;
// 	  }
//
// 	    // it is not allowed to overcome the bounds...
// 	    // ...see the result
//
// 	   qf_1=chain1->setAng(qf_1);
// 	   qf_2=chain2->setAng(qf_2);
//
// 	   // instantiate a IPOPT solver for inverse kinematic
// 	   // for both translational and rotational part
//
//
 	   iKinIpOptMin slv_1(*chain1,IKINCTRL_POSE_FULL,1e-3,100);
 	   qhat_1=slv_1.solve(chain1->getAng(),xf_1,0.0,dummy,dummy,0.0,dummy,dummy);



// 	   std::cout<<" 1: "<<qhat_1.size();

       //    for(int u=0;u<7;u++){

      iKinIpOptMin slv_2(*chain2,IKINCTRL_POSE_FULL,1e-3,100);
 	   qhat_2=slv_2.solve(chain2->getAng(),xf_2,0.0,dummy,dummy,0.0,dummy,dummy);

// 	  std::cout<<" joints 2: "<<qhat_2.size();

 	   qhat_1Deg = qhat_1;
 	   qhat_2Deg = qhat_2;


 	  for (int i = 0 ; i< 7; i++)
 	   	   {
 	   		   qhat_1Deg[i] = qhat_1[i]*CTRL_RAD2DEG;
 	   		   qhat_2Deg[i] = qhat_2[i]*CTRL_RAD2DEG;
 	   	   }

 	 std::cout<<"joints 1: "<<qhat_1Deg.toString();

 	std::cout<<"\njoints 2: "<<qhat_2Deg.toString()<<"\n";
}


}
