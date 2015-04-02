
#include <string>
#include <vector>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>


#define GRASP_CMD       "grasp"
#define RELEASE_CMD     "release"

enum gstatus_t {
    GRASPED,
    RELEASED,
    BUSY
    };

class Grasp
{
   
public:

    Grasp()
    {
       // constructor
        tactileThreshold = tactileReset = 0.0;
        withTactile = false;
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();
    void loop(); 
    bool interrupt();

private: 
    void gazeRest();
    void closeHand(void);
    void openHand(void);
    bool getTactileValue(double &sumTactile);

private:
    yarp::os::BufferedPort<yarp::os::Bottle> graspCmdPort;
    yarp::os::BufferedPort<yarp::os::Bottle> graspStatusPort;
    yarp::os::BufferedPort<yarp::sig::Vector> tactilePort;	// input port for compensated tactile data
    yarp::dev::PolyDriver driver;
    yarp::dev::IPositionControl *iHand;
    yarp::dev::PolyDriver armDriver;
    yarp::dev::ICartesianControl *iArm;

    gstatus_t status;
    gstatus_t action;     
    bool withTactile;
    double tactileThreshold;
    double tactileReset;
    std::vector<int> openPoses;
    std::vector<int> closePoses;
    double actionTime;
    double graspXOffset;
    double graspYOffset;
    double graspZOffset;
    std::string partName;
};

   
   



   
