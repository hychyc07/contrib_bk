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

Matrix makeCross(const Vector& in) {  // creates skew-sym equivalent to cross product
    Matrix ret(3,3);
    ret.zero();
    ret(0,1) = -in(2);  // a     0 -c  b
    ret(0,2) =  in(1);  // b ->  c  0 -a
    ret(1,0) =  in(2);  // c    -b  a  0
    ret(1,2) = -in(0);
    ret(2,0) = -in(1);
    ret(2,1) =  in(0);
    return ret;
}

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

//    Matrix H_r_first;
//    H_r_first.resize(4,4); // Hleft or Hright
//    H_r_first.zero();
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

    Matrix _I_ci(6,6);
    double _m;
    int joint = 0;
    for(int joint = 0; joint<7; joint++){ //NOTORSO

        printf("i: %d\n",joint);

        // Let's do it the Khatib way! (all refs [Khatib00])(more at [Chang99])
        
        // Step 0-A. Calculate I_ci.
        // (what Khatib calls spatial inertia matrix, at link COGs) (ec.14)
        _I_ci.zero();
        _m = armDyn.getMass(joint);
        printf("mass: %f\n",_m);
        _I_ci(0,0) = _I_ci(1,1) = _I_ci(2,2) = _m;
        Matrix _bottomRight = armDyn.getInertia(joint);  // RChackers tell us getInertia() is at COM (COM=ci).
        printf("inertia at (COM=ci) -> (becomes _bottomRight):\n%s\n",_bottomRight.toString().c_str());
        _I_ci.setSubmatrix(_bottomRight,3,3);
        printf("I_ci[%d]:\n%s\n",joint,_I_ci.toString().c_str());

        // Step 0-B. Calculate X_i_ci.
        // (spatial transformation matrices from joints to corresponding link COM) (ec.15)
        Matrix _X_i_ci(6,6);
        Matrix _R_i_ci(3,3);
        Vector _l_i_ci(3);
        // begin{WARNING} CAN BE BUGGY!!! WILL DO THIS BY HAND FOR EACH LINK

    if(joint==0) {  // SCR

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
_l_i_ci(0) = _H_iDyni_ci(2,3);
_l_i_ci(1) = -(_H_iDyni_ci(1,3));
_l_i_ci(2) = _H_iDyni_ci(0,3);

Vector v_scr_inter;
v_scr_inter.push_back(0);
v_scr_inter.push_back(0);
v_scr_inter.push_back(1);
v_scr_inter.push_back(M_PI);
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(-M_PI/2.0);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    } else if(joint==1) {  // SRC

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
_l_i_ci(0) = _H_iDyni_ci(1,3);
_l_i_ci(1) = _H_iDyni_ci(0,3);
_l_i_ci(2) = -(_H_iDyni_ci(2,3));

Vector v_scr_inter;
v_scr_inter.push_back(0);
v_scr_inter.push_back(0);
v_scr_inter.push_back(1);
v_scr_inter.push_back(M_PI/2.0);
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(M_PI);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    } else if(joint==2) { // SRC

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
//_l_i_ci(0) = 0.012743315746;
//_l_i_ci(1) = 0.003480818839;
//_l_i_ci(2) = -0.08938;
_l_i_ci(0) = -0.012743; // let's hard-code the ones used in our xml!
_l_i_ci(1) = 0.003481;
_l_i_ci(2) = 0.089380;

Vector v_scr_inter;
v_scr_inter.push_back(1);
v_scr_inter.push_back(0);
v_scr_inter.push_back(0);
v_scr_inter.push_back(M_PI/2.0);
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(15.0*M_PI/180.0);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    } else if(joint==3) { // rot1

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
_l_i_ci(0) = -0.0013; // let's hard-code the ones used in our xml!
_l_i_ci(1) = 0.00371;
_l_i_ci(2) = -0.001050;

Vector v_scr_inter;
v_scr_inter.push_back(1);
v_scr_inter.push_back(0);
v_scr_inter.push_back(0);
v_scr_inter.push_back(0); // just so we do not have to mofify the structure
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(165.0*M_PI/180.0);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    } else if(joint==4) { // rot2

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
_l_i_ci(0) = -0.00476; // let's hard-code the ones used in our xml!
_l_i_ci(1) = -0.000347;
_l_i_ci(2) = 0.066;

Vector v_scr_inter;
v_scr_inter.push_back(1);
v_scr_inter.push_back(0);
v_scr_inter.push_back(0);
v_scr_inter.push_back(M_PI);
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(-105.0*M_PI/180.0);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    } else if(joint==6) { // rot3?

Matrix _H_iDyni_ci(4,4);
_H_iDyni_ci = armDyn.getCOM(joint);
_l_i_ci(0) = -0.0080511; // let's hard-code the ones used in our xml!
_l_i_ci(1) = -0.007000;
_l_i_ci(2) = 0.0703;

Vector v_scr_inter;
v_scr_inter.push_back(0);
v_scr_inter.push_back(1);
v_scr_inter.push_back(0);
v_scr_inter.push_back(-M_PI/2.0);
Matrix R_scr_inter = axis2dcm(v_scr_inter).submatrix(0,2,0,2);

Vector v_inter_iDyni;
v_inter_iDyni.push_back(1);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(0);
v_inter_iDyni.push_back(-105.0*M_PI/180.0);
Matrix R_inter_iDyni = axis2dcm(v_inter_iDyni).submatrix(0,2,0,2);

_R_i_ci = R_scr_inter * R_inter_iDyni;
_X_i_ci.setSubmatrix(_R_i_ci,0,0);
_X_i_ci.setSubmatrix(_R_i_ci,3,3);

    }


        // end{WARNING} CAN BE BUGGY!!! WILL DO THIS BY HAND FOR EACH LINK
        printf("using l: %.10f, %.10f, %.10f\n", _l_i_ci[0],_l_i_ci[1],_l_i_ci[2]);
        _X_i_ci.setSubmatrix(makeCross(_l_i_ci)*_R_i_ci,3,0);

        // Step 0-C. Calculate I_i.
        // (spatial inertia matrix, at corresponding joints) (ec.16)
        Matrix _I_i = _X_i_ci * _I_ci * _X_i_ci.transposed();
        printf("_I_i[%d]:\n%s\n",joint,_I_i.toString().c_str());

        Matrix _I_i_bottomRight(_I_i.submatrix(3,5,3,5));
        printf("inertia at joint -> (_bottomRight of previous):\n%s\n",_I_i_bottomRight.toString().c_str());
        printf("diag: %.10f, %.10f, %.10f\n", _I_i_bottomRight[0][0],_I_i_bottomRight[1][1],_I_i_bottomRight[2][2]);

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
///////        printf("H_first_tip[%d]: %.12f, %.12f, %.12f\n", joint,-H_first_second[0][3],H_first_second[2][3],H_first_second[1][3]);
        Matrix H_second_com = (armDyn.getCOM(joint));
        Matrix H_first_com = H_first_second*H_second_com;
//        printf("\nH_first_com[%d]:\n%s\n",joint,H_first_com.toString().c_str());
//////        printf("(SCR) H_first_com[%d]: %.12f, %.12f, %.12f\n", joint,-H_first_com[0][3],H_first_com[2][3],H_first_com[1][3]);
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
      
      
