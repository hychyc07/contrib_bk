/** 
* Copyright: 2011 RBCS IIT
* Copyright: 2010-2011 RobotCub Consortium
* Author: Juan G Victores
* Contrib: Serena Ivaldi (author of iDyn and example code)
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/ 


#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::ctrl;

int main() {

//    iCubArmDyn armDyn("left");
    iCubArmNoTorsoDyn armDyn("left"); //NOTORSO

    Vector q(armDyn.getN()); q.zero();
    Vector dq(q); Vector ddq(q);
    printf("getN: %d\n",armDyn.getN());
 /*   q[0]=-90*CTRL_DEG2RAD;
    q[1]=90*CTRL_DEG2RAD;
    q[2]=-75*CTRL_DEG2RAD;
    q[3]=0;
    q[4]=90*CTRL_DEG2RAD;*/
    q[0]=0*CTRL_DEG2RAD;
    q[1]=0*CTRL_DEG2RAD;
    q[2]=0*CTRL_DEG2RAD;
    q[3]=0*CTRL_DEG2RAD;
    q[4]=0*CTRL_DEG2RAD;
    q[5]=0*CTRL_DEG2RAD;
    q[6]=0*CTRL_DEG2RAD;
    armDyn.setAng(q);
    armDyn.setDAng(dq);
    armDyn.setD2Ang(ddq);

    printf("**************************************************************\n");
//    Matrix H_r_first;
//    H_r_first.resize(4,4); // Hleft or Hright
//    H_r_first.zero();
/*    double ct = cos(-15*CTRL_DEG2RAD);
    double st = sin(-15*CTRL_DEG2RAD);
        H_r_first(0,0) = ct;        // ct  0   st
        H_r_first(0,2) = st;        //  0   1    0
        H_r_first(1,1) = 1.0;
        H_r_first(2,0) = -st;
        H_r_first(2,2) = ct;*/
//j//    Matrix H_r_first = armDyn.getH(0);
//j//    Matrix H_first_r = SE3inv(H_r_first);
//    printf("H[r_0]:\n%s\n",H_r_first.toString().c_str());
//    Matrix H_first_com = armDyn.getCOM(0);
//    Matrix H_r_com0 = H_r_first*H_first_com;
//    printf("\nH_first_com[0]:\n%s\n",H_first_com.toString().c_str());
//    Matrix R_r_first = H_r_first.submatrix(0,2,0,2);
//    Vector axis_r_first = dcm2axis(R_r_first);
//    printf("axis/angle[root-0]: %f,%f,%f,%f,%f,%f,%f\n",H_r_first(0,3),H_r_first(1,3),H_r_first(2,3),axis_r_first[0],axis_r_first[1],axis_r_first[2],axis_r_first[3]);
    printf("**************************************************************\n");
    printf("**************************************************************\n");

    for(int joint = 0; joint<7; joint++){ //NOTORSO
        printf("Joint: %d\n",joint);
        printf("mass: %f\n",armDyn.getMass(joint));
        printf("inertia:\n%s\n",armDyn.getInertia(joint).toString().c_str());
//        Matrix inertia = armDyn.getInertia(joint);
//j        printf("inertia: %f, %f, %f\n",inertia[0][0],inertia[1][1],inertia[2][2]);
//j        printf("com:\n%s\n",armDyn.getCOM(joint).toString().c_str());
//        printf("H_com: %f, %f, %f\n", H_second_com[0][3],H_second_com[1][3],H_second_com[2][3]);
//        Matrix H_r_first = armDyn.getH(joint-1);
//        if (joint==2) {
//            Matrix axym = axis2dcm(axy);
        Matrix H_first_second = armDyn.getH(joint);
//j//        Matrix H_first_second = H_first_r * H_r_second;
//        printf("\nH_first_tip[%d]:\n%s\n",joint,H_first_second.toString().c_str());
        printf("H_first_tip[%d]: %.12f, %.12f, %.12f\n", joint,-H_first_second[0][3],H_first_second[2][3],H_first_second[1][3]);
        Matrix H_second_com = (armDyn.getCOM(joint));
        Matrix H_first_com = H_first_second*H_second_com;
//        printf("\nH_first_com[%d]:\n%s\n",joint,H_first_com.toString().c_str());
        printf("H_first_com[%d]: %.12f, %.12f, %.12f\n", joint,-H_first_com[0][3],H_first_com[2][3],H_first_com[1][3]);
//        if (joint==2)
//        printf("\ncomy:\n%s\n",comy.toString().c_str());
//        printf("comy: %f, %f, %f\n", comy[0][3],comy[1][3],comy[2][3]);
//        }

//        printf("\nH[r_%d]:\n%s\n",joint,H_r_second.toString().c_str());
//        printf("\nH[%d]:\n%s\n",joint,H_r_first.toString().c_str());
//        printf("\nH[%d]:\n%s\n",joint,H_r_first.toString().c_str());
//        printf("\nH[%d]:\n%s\n",joint,H_first_second.toString().c_str());
//        printf("is: %.10f, %.10f, %.10f\n", H_first_second[0][3],H_first_second[1][3],H_first_second[2][3]);
//      Matrix R_first_second = H_first_second.submatrix(0,2,0,2);
//        Matrix R_first_second = H_first_second.submatrix(0,2,0,2);
//        Vector axis_first_second = dcm2axis(R_first_second);
//        Vector axis = dcm2axis(_first_second);
//        printf("**************************************************************\n");
//        printf("axis/angle[%d-%d]: %f,%f,%f,%f,%f,%f,%f\n",joint,joint+1,H_first_second(0,3),H_first_second(1,3),H_first_second(2,3),axis_first_second[0],axis_first_second[1],axis_first_second[2],axis_first_second[3]);
        printf("**************************************************************\n");
    }

    return 0;
}
      
      
