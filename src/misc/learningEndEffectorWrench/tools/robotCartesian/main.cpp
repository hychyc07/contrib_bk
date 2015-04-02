// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Using the Cartesian Interface to control a limb in the operational space.
//

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/ctrl/math.h>

#include <stdio.h>

// next 2 lines for the getch implementation
#include <termios.h>
#include <unistd.h>

#include <fstream>

#include <ctime>
#include <sstream>

//
#define START_DIST 0.008
//

#define RADIUS 0.01
#define SPEED 0.1
#define CTRL_THREAD_PER     0.02    // [s]
#define CTRL_MODULE_PER     1.0    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     0.0    // [deg]
#define MIN_TORSO_PITCH     0.0    // [deg]

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class Repo {
  public:
    Repo() {}
    bool init(Property& option) {
        counter = 0;
        //
        if (!yarp.checkNetwork()) {
            printf("No yarp network, bye!\n");
            return false;
        }
        //
        port_q.open("/dataCollect/q:i");
        port_f.open("/dataCollect/f:i");
        //
        option.put("device","cartesiancontrollerclient");
        Value robotvalue = option.check("robot","icub","checking if given an alternate robot name");
        ConstString backSlash("/");
        option.put("remote",backSlash+robotvalue.asString()+"/cartesianController/left_arm");
        option.put("local","/cartesian_client/left_arm");
        //
        if (!client.open(option))
            return false;
        // open the view
        client.view(arm);
        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        arm->storeContext(&startup_context_id);

        // set trajectory time
        arm->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        // disable the torso
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        arm->setDOF(newDof,curDof);

        if(yarp.connect(backSlash+robotvalue.asString()+"/left_arm/state:o","/dataCollect/q:i")) {
            q = *(port_q.read(true));
            printf("[success] connected to left arm q.\n");
        } else {
            printf("[error] could not connect to left arm q, aborting...\n");
            return false;
        }

        if(yarp.connect("/wholeBodyDynamics/left_arm/endEffectorWrench:o","/dataCollect/f:i")) {
            f_ee = *(port_f.read(true));
            printf("[success] connected to left arm external contacts.\n");
        } else {
            printf("[error] could not connect to left arm external contacts, aborting...\n");
            return false;
        }

        time_t t = time(NULL);
	    tm* timePtr = localtime(&t);

        Value distvalue = option.check("dist",0,"set value");
        std::stringstream ss;
        ss << timePtr->tm_mon+1 << "-" << timePtr->tm_mday << "-cbt-" << (distvalue.asDouble())*1000 << ".dat";

        Value filevalue = option.check("out",ss.str().c_str(),"checking if given an alternate file name");
        printf("Logging to: %s\n",filevalue.asString().c_str());
        logger.open(filevalue.asString(), std::ios::out);

        return true;
    }

    bool close(){
        logger.close();
        // we require an immediate stop
        // before closing the client for safety reason
        arm->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        arm->restoreContext(startup_context_id);

        client.close();
        return true;
    }

    bool cross3D(double distance);
    bool box3D(double distance);

    int mygetch();

    double rndampl(const double amplitude) {
        return (2*amplitude*rand()/RAND_MAX)-amplitude;
    }

    PolyDriver client;
    Vector xd, od;
    int startup_context_id;
    double t, t0, t1;
    ICartesianControl *arm;
    BufferedPort<Vector> port_q;
    BufferedPort<Vector> port_f;
    Vector f_ee;
    Vector q;
    Vector d;
    Network yarp;

    int counter;
    std::ofstream logger;
};

int Repo::mygetch() {
    // can substitute next block for a timer:
    Time::delay(0.1); // seconds
    ///////////// [begin] The getch() implementation ///////////
    
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    
    ///////////// [end] The getch() implementation ///////////

    Vector* qRead = port_q.read(false);
    if (qRead) {
        q = *qRead;  // joint position values
    } else {
        printf("Get joint values failed!\n");
        return -1;
    }

    Vector* f_eeRead = port_f.read(false);
    if (f_eeRead) {
        f_ee = *f_eeRead;  // EE force in end effector frame
    } else {
        printf("Get external contacts failed!\n");
        return -1;
    }
        
    d = q.subVector(0,4);
    for (int k=0;k<6;k++)
        d.push_back(f_ee[k]);
    printf("Sample %d: ",counter+1);  // Get on screen

    for (int k=0;k<11;k++) {
//        fprintf(stderr,"%f ",d[k]);  // Get on screen through stderr
        printf("%f ",d[k]);  // Sent to stdout to pass to file using operator >
        logger << d[k] << " ";  // Sent to logger file
    }
//    fprintf(stderr,"\n");  // Get on screen through stderr
    logger << std::endl;  // Sent to logger file

    printf("now I grabbed data\n");
    counter++;
//    return ch;
    return 0;
}

bool Repo::cross3D(double distance) {
    // set initial position and orientation
    Vector xd(3);
    Vector od(4);
    xd[0]=-0.3;
    xd[1]=-0.1;
    xd[2]=+0.1;
    od[0]=0.0;
    od[1]=0.0;
    od[2]=1.0;
    od[3]=M_PI;
// uncomment this for karate instead of flat on table:
/*    Vector A(4); 
    A[0]=1.0;
    A[1]=0.0;
    A[2]=0.0;
    A[3]=M_PI/2.0;
    Matrix R = iCub::ctrl::axis2dcm(A);
    Matrix OD = iCub::ctrl::axis2dcm(od);
    Matrix res = R * OD;
    od = iCub::ctrl::dcm2axis(res);
*/
    // go there
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross center.\n");
//    mygetch();


    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[2]+=distance; // above
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 1. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[2]-=2*distance; // below
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 2. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[2]+=distance; // one side
    xd[1]-=distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 3. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[1]+=2*distance; // other side
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 4. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[1]-=distance; // behind
    xd[0]-=distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 5. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[0]+=2*distance; // in front
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached cross ref point 6. Any key...\n");
    mygetch();

/*    xd[0]-=distance; // return
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached ref point. Any key...\n");
    mygetch();*/

    return true;
}


bool Repo::box3D(double distance) {
    // set initial position and orientation
    Vector xd(3);
    Vector od(4);
    xd[0]=-0.3;
    xd[1]=-0.1;
    xd[2]=+0.1;
    od[0]=0.0;
    od[1]=0.0;
    od[2]=1.0;
    od[3]=M_PI;
// uncomment this for karate instead of flat on table:
/*    Vector A(4); 
    A[0]=1.0;
    A[1]=0.0;
    A[2]=0.0;
    A[3]=M_PI/2.0;
    Matrix R = iCub::ctrl::axis2dcm(A);
    Matrix OD = iCub::ctrl::axis2dcm(od);
    Matrix res = R * OD;
    od = iCub::ctrl::dcm2axis(res);
*/
    // go there
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box center.\n");
//    mygetch();


    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[2]+=distance; // above

    xd[1]-=distance;
    xd[0]-=distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 1. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[0]+=2*distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 2. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[1]+=2*distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 3. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[0]-=2*distance; // done top, at (0,1)
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 4. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[2]-=2*distance; // below
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 5. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[1]-=2*distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 6. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[0]+=2*distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 7. Any key...\n");
    mygetch();

    xd[0]+=rndampl(0.001);
    xd[1]+=rndampl(0.001);
    xd[1]+=2*distance;
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached box ref point 8. Any key...\n");
    mygetch();

/*    xd[0]-=distance; // return
    arm->goToPoseSync(xd,od);
    arm->waitMotionDone();
    printf("\nReached ref point. Any key...\n");
    mygetch();*/

    return true;
}

/*****************************************************************************/
int main(int argc, char *argv[]) {

    srand ( time(NULL) );

    YARP_REGISTER_DEVICES(icubmod)

    Property options;
    options.fromCommand(argc, argv);

    if(!options.check("dist")){
        printf("use: robotCartesian --robot [robot] --part [part] --dist [distance] --out [filename]\n");
        printf("0.008 -> 8\n");
        printf("0.010 -> 10\n");
        printf("0.012 -> 12\n");
        printf("0.014 -> 14\n");
        printf("0.016 -> 16\n");
        printf("0.018 -> 18\n");
        return -1;
    }

//    double startd = START_DIST;
    Value distvalue = options.check("dist",0,"set value");
    double startd = distvalue.asDouble();
    printf("using starting distance: %f.\n",startd);

    Repo myRepo;
    if(!myRepo.init(options)) {
        printf("leave\n");
        return -1;
    }

    myRepo.cross3D(startd);
    printf("\nDID cross %f.\n\n",startd); // 0.01x
    myRepo.box3D(startd);
    printf("\nDID box %f.\n\n",startd);
    startd+=0.010;
    myRepo.cross3D(startd);
    printf("\nDID cross %f.\n\n",startd); // 0.02x
    myRepo.box3D(startd);
    printf("\nDID box %f.\n\n",startd);
    startd+=0.010;
    myRepo.cross3D(startd);
    printf("\nDID cross %f.\n\n",startd); // 0.03x
    myRepo.box3D(startd);
    printf("\nDID box %f.\n\n",startd);
    startd+=0.010;
    myRepo.cross3D(startd);
    printf("\nDID cross %f.\n\n",startd); // 0.04x
    myRepo.box3D(startd);
    printf("\nDID box %f.\n\n",startd);

    myRepo.close();
    return 0;
}



