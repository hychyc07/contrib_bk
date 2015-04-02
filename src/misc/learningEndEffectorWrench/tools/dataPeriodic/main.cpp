/*
 * Copyright: Juan G. Victores (C) 2011
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

//#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>
//#include <deque>
//#include <iostream>
//#include <yarp/os/all.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
//#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>
//#include <yarp/sig/Matrix.h>
//#include <yarp/math/Math.h>
//#include <yarp/math/SVD.h>

using namespace yarp::os;
using namespace yarp::sig;
//using namespace yarp::math;
//using namespace std;

//#define N 6

int main(int argc, char** argv) {
    Network yarp;
    BufferedPort<Vector> port_q;
    BufferedPort<Vector> port_f;
    BufferedPort<Vector> port_d;
    port_q.open("/dataCollect/q:i");
    port_f.open("/dataCollect/f:i");
//    port_d.open("/dataCollect/d:o");
    Vector f_ee;
    Vector q;
    Vector d;
//    port_d.prepare() = d;

    if(yarp.connect("/icub/left_arm/state:o","/dataCollect/q:i")) {
        q = *(port_q.read(true));
        printf("[success] connected to left arm q.\n");
    } else {
        printf("[error] could not connect to left arm q, aborting...\n");
        return -1;
    }

    if(yarp.connect("/wholeBodyDynamics/left_arm/endEffectorWrench:o","/dataCollect/f:i")) {
        f_ee = *(port_f.read(true));
        printf("[success] connected to left arm external contacts.\n");
    } else {
        printf("[error] could not connect to left arm external contacts, aborting...\n");
        return -1;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////

    for (int i=0;i<800;i++){
        Time::delay(1.1);
        Vector* qRead = port_q.read(false);
        if (qRead) {
            q = *qRead;  // joint position values
        } else {
            printf("Get joint values failed!\n");
            break;
        }

        Vector* f_eeRead = port_f.read(false);
        if (f_eeRead) {
            f_ee = *f_eeRead;  // EE force in end effector frame
        } else {
            printf("Get external contacts failed!\n");
            break;
        }
        
        d = q.subVector(0,4);
        for (int k=0;k<6;k++)
            d.push_back(f_ee[k]);

        fprintf(stderr,"Sample %d: ",i+1);  // Get on screen through stderr

//        port_d.write();  // Write to port
        for (int k=0;k<11;k++) {
            fprintf(stderr,"%f ",d[k]);  // Get on screen through stderr
            printf("%f ",d[k]);  // Sent to stdout to pass to file using operator >
        }
        fprintf(stderr,"\n");  // Get on screen through stderr
        printf("\n");  // Sent to stdout to pass to file using operator >
    }

/*     Matrix A1(3,3);
    A1.zero();
    A1(1,1)=0.0022;
    Matrix A2(3,3);
    A2.eye();
    A2(1,1)=-1278633652283202.213;
    Matrix A3 = A2*A1;
    printf("Fread:\n%s\n",A3.toString().c_str());
*/
/*    deque<Matrix> OMEGA(N+1);
    Matrix zero6x6(6,6);
    zero6x6.eye();
    printf("OKAY\n");
    OMEGA.push_back(zero6x6);
    OMEGA.push_back(zero6x6);
    printf("Fread:\n%s\n",OMEGA[N+1].toString().c_str());
    printf("Fread+1:\n%s\n",OMEGA[N].toString().c_str());
    printf("Fread-1:\n%s\n",OMEGA[N-1].toString().c_str());*/
    return 0;
}
