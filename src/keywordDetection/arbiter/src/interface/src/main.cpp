/*
 * main.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include <yarp/os/all.h>

#include <stdio.h>
#include <iostream>
#include "iCub/AInterface.h"



using namespace yarp::os;
using namespace std;


int main(int argc, char * argv[]){
	Network yarp;
	AInterface module;

	ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("");      //overridden by --from parameter
    rf.setDefaultContext("");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

	module.runModule(rf);
	return 0;
}




