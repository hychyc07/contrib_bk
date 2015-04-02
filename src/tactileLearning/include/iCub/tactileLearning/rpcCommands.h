#ifndef __RPC_COMMANDS_H__
#define __RPC_COMMANDS_H__

#include <string>

namespace iCub{

namespace tactileLearning{

/**
* Enum containing all the commands accepted by the rpc port of the TactileLearning module.
* The last element of the enum (TactileLearningCommandSize) represents the total number of commands accepted by the module.
*/
typedef enum {get_optimal_trajectory, get_info, get_help, quit, tactileLearningCommandSize} TactileLearningCommand;
    
///Enum containing the responses to the the set commands
typedef enum {tactile_learning_ok, tactile_learning_not_ready, tactile_learning_error} TactileLearningResponse;

///The order of the command in this list MUST correspond to the order of the enum TactileLearningCommand
const std::string TactileLearningCommandList[] = {"get optimal trajectory", "get info", "get help", "quit"};

//The order in TactileLearningCommandDesc must correspond to the order in TactileLearningCommandList
const std::string TactileLearningCommandDesc[]  = {
	"get the optimal trajectory, i.e. the output of PI^2 algorithm", 
	"get information about the module", 
	"get help",
	"quit the module"};
}

}

#endif

