#include <stdio.h>
#include <yarp/os/Time.h>
#include <iCub/skinDynLib/skinContact.h>
#include "iCubDblTchSlv.h"
#include <iostream>
#include <fstream>

using namespace yarp::os;
using namespace iCub::skinDynLib;
using namespace std;

Matrix      findH0(skinContact &sc);
skinContact fromString (string s);

int main()
{
    // string type = "test";
    string type = "customLimb";

    iCubDoubleTouch_Problem problem("customLimb");

    int dim = problem.getNVars();

    iCubDoubleTouch_Variables g(dim); // guess
    iCubDoubleTouch_Variables s(dim); // solution

    for (int i = 0; i < dim; i++) { g.joints[i] = 0.0*CTRL_DEG2RAD; }
    g.joints[1]  = -50.0*CTRL_DEG2RAD;
    g.joints[3]  = -20.0*CTRL_DEG2RAD;
    g.joints[4]  =  25.8*CTRL_DEG2RAD;

    g.joints[5]  = -25.8*CTRL_DEG2RAD;
    g.joints[6]  =  20.0*CTRL_DEG2RAD;
    g.joints[8]  =  50.0*CTRL_DEG2RAD;

    string sc_b;
    sc_b = "((1424 3 4 2) (-0.026 -0.049 -0.017) (0.957265 -0.141658 0.460389) (0.0 0.0 0.0) (-0.026 -0.049 -0.017) (-0.892 0.132 -0.429) (213) 21.463334)";
    sc_b = "((1458 3 4 2) (-0.025562 -0.049899 -0.017432) (5.366549 -0.736607 2.724205) (0.0 0.0 0.0) (-0.025333 -0.050333 -0.017667) (-0.879806 0.120761 -0.446613) (212 213 215) 40.664625)";
    sc_b = "((1507 3 4 2) (-0.025491 -0.049609 -0.017612) (5.63259 -0.799401 2.90479) (0.0 0.0 0.0) (-0.025333 -0.050333 -0.017667) (-0.876412 0.124384 -0.451976) (212 213 215) 42.845816)";
    sc_b = "((1517 3 4 2) (-0.008038 -0.069189 0.04) (0.330047 -0.123022 -0.92668) (0.0 0.0 0.0) (-0.008 -0.069 0.04) (-0.330481 0.123184 0.927897) (125) 9.986876)";
    sc_b = "((1583 3 4 2) (-0.007731 -0.069675 0.040336) (1.231503 -0.416617 -3.979341) (0.0 0.0 0.0) (-0.007667 -0.069333 0.040333) (-0.291266 0.098535 0.941165) (124 125 127) 28.187345)";
    // s = "((1637 3 4 2) (-0.01119 -0.059426 0.037631) (3.322063 -1.238654 -6.426727) (0.0 0.0 0.0) (-0.012 -0.0595 0.03725) (-0.439237 0.163772 0.849729) (128 129 133 134) 37.816319) ((1638 3 4 2) (0.004389 -0.040332 -0.030367) (-0.606696 -1.374125 8.539399) (0.0 0.0 0.0) (0.002 -0.0384 -0.0302) (0.068741 0.155693 -0.967543) (293 305 317 345 353) 35.303449)";
    skinContact sc = fromString(sc_b);

    printf("sc %s\n", sc.toString().c_str());

        Matrix H0 = findH0(sc);
        problem.limb.setH0(SE3inv(H0));

    problem.limb.setAng(g.joints);

    Matrix H   = problem.limb.getH();
    Vector orp = H.getCol(3);
    Vector oro = dcm2axis(H);

    for (int i = 0; i < orp.size(); i++)
        g.ee[i] = orp[i];

    for (int j = orp.size(); j < orp.size()+oro.size(); j++)
        g.ee[j] = oro[j-orp.size()];

    double t0=Time::now();

    iCubDoubleTouch_Solver solver(problem,g);
    solver.solve(s);
    double t1=Time::now();

    printf("\n#### initial g ...\n");
    g.print();
    printf("\n#### computation done in %g [s] ...\n",t1-t0);
    printf("#### final g ...\n");
    s.print();

    // basic file operations

    ofstream myfile;
    myfile.open ("../matlab/iCubFwdKin/joints_H0.txt");
    myfile << s.joints.toString().c_str() << endl;
    myfile << H0.toString().c_str() << endl;
    myfile.close();

    return 0;
}

skinContact fromString (string s) {
    skinContact sc;

    Vector v(21,0.0);
    sscanf(s.c_str(), "((%lf %lf %lf %lf) (%lf %lf %lf) (%lf %lf %lf) (%lf %lf %lf) (%lf %lf %lf) (%lf %lf %lf) (%lf) %lf)",
                      &v(0), &v(1), &v(2), &v(3), &v(4), &v(5),
                      &v(6), &v(7), &v(8), &v(9), &v(10), &v(11),
                      &v(12), &v(13), &v(14), &v(15), &v(16),
                      &v(17), &v(18), &v(19), &v(20));

    /*v(0)  = 1424;     v(1)  = 3;           v(2)  = 4;
    v(3)  = 2;          v(4)  = -0.026;      v(5)  = -0.049;
    v(6)  = -0.017;     v(7)  = 0.957265;    v(8)  = -0.141658;
    v(9) = 0.460389;    v(10) = 0.0;         v(11) = 0.0;
    v(12) = 0.0;        v(13) = -0.026;      v(14) = -0.049;
    v(15) = -0.017;     v(16) = -0.892;      v(17) = 0.132;
    v(18) = -0.429;     v(19) = 213;         v(20) = 21.463334;*/

    printf("v %s\n", v.toString().c_str());

    sc.fromVector(v);
    return sc;
}

Matrix findH0(skinContact &sc)
{
    // Set the proper orientation for the touching end-effector
    Matrix H0(4,4);
    Vector x(3,0.0), z(3,0.0), y(3,0.0);

    x = sc.getNormalDir();
    z[0] = -x[2]/x[0]; z[2] = 1;
    y = -1*(cross(x,z));
    
    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);

    H0.zero();
    H0(3,3) = 1;
    H0.setSubcol(x,0,0);
    H0.setSubcol(y,0,1);
    H0.setSubcol(z,0,2);
    H0.setSubcol(sc.getGeoCenter(),0,3);

    printf("H0 DIR:\n%s\n", H0.toString().c_str());

    return H0;
}

// FOREARM LEFT ((x x x 2) ()
// ((1424 3 4 2) (-0.026 -0.049 -0.017) (0.957265 -0.141658 0.460389) (0.0 0.0 0.0) (-0.026 -0.049 -0.017) (-0.892 0.132 -0.429) (213) 21.463334)

// EMPTY LINE! Everyone wants gcc to be happy, RIGHT? :)