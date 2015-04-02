#include <yarp/os/Network.h>
#include <gestureRecognition.h>
#include <DictionaryLearning.h>


void printMatrixYarp(yarp::sig::Matrix &A) 
{
    cout << endl;
    for (int i=0; i<A.rows(); i++) 
    {
        for (int j=0; j<A.cols(); j++) 
        {
            cout<<A(i,j)<<" ";
        }
        cout<<endl;
    }
    cout << endl;
}

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("demoGestureRecognition/conf");
    rf.setDefaultConfigFile("gestureRecognition.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    GestRecognition mod;
    return mod.runModule(rf);

    return 1;
}

