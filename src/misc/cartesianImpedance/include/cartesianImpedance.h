
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <yarp/os/Port.h>
#include <yarp/dev/PolyDriver.h>
#include <string>
#include <iCub/iKin/iKinFwd.h>
#include <stdio.h>
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace std;

#define GCOMP_PORT_RIGHT_ARM "/gravityCompensator/right_arm_torques:o"
#define GCOMP_PORT_LEFT_ARM "/gravityCompensator/left_arm_torques:o"

#define GSCHMIDT_THRESHOLD_DEG 1

#define TRAJ_STEP 0.002
#define TRAJ_LOW_ZLIM -0.1
#define TRAJ_UP_ZLIM 0.15

class cartesianImpedance
{   
   yarp::dev::PolyDriver robotDevice;

   yarp::dev::IPositionControl *pos;
   yarp::dev::IControlMode *ictrl;
   yarp::dev::IImpedanceControl *iimp;
   yarp::dev::ITorqueControl *itrq;

   BufferedPort<Bottle> gcomp;


   Bottle *gcomp_bottle;

   Vector homeTorques;
   int numberControlledJoints;
   int blockedIndex[7];
   Vector zero_vec;
   Matrix jac, jac_t;
   Matrix multiplier;
   FILE* outFile;
   Matrix Kj;
   Vector encoders_deg;
   Vector cmdTorques;
   Vector encoders_rad;
   Matrix eigval_mat;
   Matrix eigvec_mat_t;
   Vector proj( Vector v, Vector in_u );

   int sign_traj;

public:
   Vector init_cart_pos;
   Vector home_deg;
   Vector home_rad;
   Matrix eigvec_mat;
   yarp::dev::IEncoders *encs;
   double stiff_1, stiff_2, stiff_3;
   bool send_cmd;
   iKinLimb* limb;
   std::string controlled_part;
   std::string gcomp_port;
   bool verbose;
   std::string robotname;
   bool traj;

    cartesianImpedance()
    {
        //let's control the shoulder and the elbow
        numberControlledJoints = 4;

        jac.resize(6,7);
        jac_t.resize(7,6);
        multiplier.resize(7,6);
        // constructor
        pos=0;
        encs=0;
        ictrl=0;
        iimp=0;
        itrq=0;

        send_cmd = false;

        stiff_1 = stiff_2 = stiff_3 = 0;

        controlled_part = "right_arm";
        limb = new iCubArm("right");

        gcomp_port = GCOMP_PORT_RIGHT_ARM;

        for(int i=0;i<7;i++)
        	blockedIndex[i] = 0;

        blockedIndex[1] = 1;

        zero_vec.resize(6);
        zero_vec.zero();


        outFile = fopen("KMatrix.txt", "w");

        encoders_rad.resize(16);
        encoders_deg.resize(16);
        init_cart_pos.resize(6);

        traj  = false;
        verbose = false;

        sign_traj = 1;


    }

    bool open();

    bool close();

    void loop(); 

    bool interrupt();

    int checkOrtho();


};

   
   



   
