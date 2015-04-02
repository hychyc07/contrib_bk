#include <iCub/tactileLearning/tactileLearning.h>

int main(int argc, char* argv[])
{
   // Initialize yarp network 
   Network yarp;

   // Create the module
   iCub::tactileLearning::TactileLearning module;

   // Prepare and configure the resource finder
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("tactileLearningConf.ini");		//overridden by --from parameter
   rf.setDefaultContext("tactileLearning/conf");		//overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);

   // Run the module: runModule() calls configure first and, if successful, it then runs
   module.runModule(rf);

   return 0;
}

