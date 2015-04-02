#include "crawlInvKin.h"

/********************************************
 * IK CLASS
 * ****************************************/

IKManager::IKManager()
{  
  this->rate = ((double)rate)/1000.0;
  
  leftArm = new iCubArm("left");

  legLength = 0.23;  
  dShoulder = 0.110; //distance from middle torso to shoulder in coronal plane (x)
  dHip = 0.068;
  DShoulder = 0.143; //distance from middle torso to shoulder in sagittal plane (y)
  DHip = 0.088;    
}

IKManager::~IKManager()
{
	delete chain;
    delete leftArm;
}

double IKManager::getArmAmplitude(double* positions, double leg_amplitude)
{     
    iKinChain *armChain, *legChain; 
    //we simply solve: r_l*w_l = r_a*w_a 
    //radius*amplitude of arm and leg should be the equal to have approx the same step length
	
    Vector pose_q, pose_x0;

    armChain=leftArm->asChain();    
    for(int i=0; i<4; i++)
    {
    	pose_q.push_back(positions[i]);
    }
    pose_q.push_back(-1.57);
    pose_q.push_back(-1.0);
    pose_q.push_back(0.1);

    armChain->setAng(pose_q);
    pose_x0=armChain->Pose(5);
    double armLength2=0;
    for(int i=0;i<3;i++) armLength2+=pose_x0[i]*pose_x0[i];
    double armLength=sqrt(armLength2);
    
    printf("leg length %f, arm length %f\n", legLength, armLength);
    double arm_amplitude = legLength*leg_amplitude/armLength;
    
    pose_q.clear();
    pose_x0.clear();
    armChain->clear();
    
	return arm_amplitude;
}

double IKManager::getTurnParams(double turn_angle, double amplitude, int side, int limb)
{ 
  double final_amplitude;
  double c, alpha, beta, Gamma, B, R, deltaLeg, deltaArm;
  
  double gamma = M_PI-turn_angle;
  printf("turn angle %f\n", turn_angle);
  c = sqrt(DShoulder*DShoulder + DHip*DHip -2*DHip*DShoulder*cos(gamma));
  double temp = (DShoulder*DShoulder-c*c-DHip*DHip)/(2*c*DHip);
  
  if(fabs(temp)>1)
  {
      printf("error while computing amplitudes\n");
      return amplitude;
  }
  
  
  alpha = acos(temp);  
  beta = M_PI - alpha -gamma;
  
  printf("c %f alpha %f beta %f\n", c, alpha, beta);

  deltaLeg=dHip*sin(alpha); 
  deltaArm=dShoulder*sin(beta);
        
  int turnWhere=0;
  turn_angle >0.0 ? turnWhere = 1 : turnWhere = -1; // 1: left, -1:right
  
  double distLimb = 0;
  limb == 1 ? distLimb=dShoulder : distLimb=dHip; //we don't care about the torso and head  
  
  double grand, small, rgrand, rsmall;
  
  grand=c+deltaLeg+deltaArm;
  small=c-deltaLeg-deltaArm;
  
  Gamma=M_PI-2*alpha;
  R= fabs(c/2/cos(M_PI/2-alpha));
  
  printf("radius of curvature is %f\n", R);
    
  rgrand=(R+distLimb)/R;
  rsmall=(R-distLimb)/R;

  switch(side)
  {    
    case 0:
    
        final_amplitude = amplitude;
        printf("Nothing to compute (torso or head)\n");
        
        break;
    
    case 1:  //left arm
    
        if(turnWhere==1)//turning left
        {
            final_amplitude=amplitude*rsmall;
            printf("Left side turning left, ");
        }
        if(turnWhere==-1)//turning right
        {
            final_amplitude=amplitude*rgrand;
            printf("Left side turning right,  ");
        }
        
        break;
    
    case 2://right arm
    
        if(turnWhere==1)//turning left
        {
            final_amplitude=amplitude*rgrand;
            printf("Right side turning left, ");
        }
        if(turnWhere==-1)//turning right
        {
            final_amplitude=amplitude*rsmall;
            printf("Right side turning right,  ");
        }
        
        break;
  }
  
  printf("final amplitude: %f\n", final_amplitude);
 
  return final_amplitude;
}




