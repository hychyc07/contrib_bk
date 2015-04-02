// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <iostream>
#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Random.h>

#include <controlBasis/GaussianProbabilityDistribution.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int main(int argc, char *argv[]) {

    GaussianProbabilityDistribution dist(2); 

    Random::seed(Time::now());

    Vector v_0(2);
    Vector v_1(2);

    string fname = "gaussian.txt";
    /*
    v_0[0] = 4;
    v_0[1] = 8;

    v_1[0] = 2;
    v_1[1] = 6;

    v_2[0] = 3;
    v_2[1] = 7;

    dist.addSample(v_0);
    dist.addSample(v_1);
    dist.addSample(v_2);
    */
    //    dist.loadDistribution(fname);   

    for(int i=0; i<1000; i++) {
        v_0[0] = Random::normal(5,2.5);
        v_0[1] = Random::normal(7,1.5);
        //        cout << "(" << v_0[0] << "," << v_0[1] << ")" << endl;
        dist.addSample(v_0);
    }
    dist.saveDistribution(fname);
    
    for(int i=0; i<10; i++) {
        cout << "drawing sample....   ";
        dist.drawSample(v_1);
        cout << "drew: (" << v_1[0] << "," << v_1[1] << ")" << endl;
    }
    
    /*
    double p;
   
    p = dist.getProbability(v_0);
    cout << "p: " << p << endl;

    p = dist.getProbability(v_1);
    cout << "p: " << p << endl;

    p = dist.getProbability(v_2);
    cout << "p: " << p << endl;
    */

    return 1;
}
