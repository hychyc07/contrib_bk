
/**
 * @ingroup icub_demoAffv2
 *
 * \defgroup icub_affordanceModule affordanceModule
 *
 * This module is part of the application \ref demoAffv2 "demoAffv2"
 *
 *
 *\section intro_sec Description
 *
 * This module is the core of the affordance based behavior. It implements two different types of interactions: i) a simple random exploration for learning and tunning the network parameters, and ii) a social engaement game through simple oemulation of effects. This is done using  a predefined Bayesian network that contains the relations between the robot available actions, the object features and the effects of each action upon objects. The robot perceives which objects are around it and selects and action to mimick the effects with the highest probability.
 *
 *
 *\section lib_sec Libraries
 *
 * Probabilistic Network Library (PNL)
 *
 *
 *\section parameters_sec Parameters
 *
 * No parameters.
 *
 *\section portssa_sec Ports Accessed
 *
 * Input ports\n
 *
 *  /camshiftplus/all/o
  *
  *
 * Output ports\n
 *
  *
 *\section portsc_sec Ports Created
 *
 *  Input ports\n
  *
 * Output ports\n
  *
 *\section conf_file_sec Configuration Files
 *
 *
 * --file affordance Bayes net
 *  example: --file $ICUB_ROOT/conf/BNaffordances.txt
 * Examples of such files can be found at \in conf/BNaffordances.txt
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux. Windows portability issues may arise due to its dependency on the pnl library.
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./demoAfford  --file  $ICUB_ROOT/conf/BNaffordances.txt \n
 *
 * This file can be edited at \in src/demoAfford/main.cpp
 *
 *\authors Luis Montesano
 *
 **/

// yarp
//#include <yarp/os/Network.h>
//#include <yarp/os/Module.h>
#include <yarp/dev/all.h>
YARP_DECLARE_DEVICES(icubmod)

// iCub
#include <iCub/demoAff.h>

/*#ifdef USE_ICUB_MOD
    #include "drivers.h"
#endif*/

using namespace std;
using namespace yarp::os;
//using namespace iCub::contrib;


int main(int argc, char *argv[]) {
	YARP_REGISTER_DEVICES(icubmod)

    Network yarp;


    /* prepare and configure Resource Finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("demoAffv3.ini");  // overridden by --from parameter
    rf.setDefaultContext("/home/afonsogoncalves/Desktop/demoaffv3/conf");    // overridden by --context parameter
    rf.setDefault("hand_calibration_file","object_sensing.ini");
    rf.setDefault("hand_sequences_file_left","hand_sequences_left.ini");
    rf.setDefault("hand_sequences_file_right","hand_sequences_right.ini");
    rf.setDefault("affordance_database","BNaffordances.txt");
    rf.setDefault("aff_action_primitives","affActionPrimitives.ini");
    rf.setDefault("camera_calib_file","icubEyesGenova01.ini");
    rf.setDefault("table_configuration","table.ini");
    rf.setDefault("name","demoAffv2");

    rf.configure("ICUB_ROOT", argc, argv);



/*#ifdef USE_ICUB_MOD
    DriverCollection dev;
#endif*/

    DemoAff module;
    //module.setName("/demoAff"); // set default name of module
    return module.runModule(rf);
}
