/**
*
@ingroup icub_contrib_modules
\defgroup icub_YarpPortPlayer YarpPortPlayer

PortPlayer allows to replay files created by PortRecorder and to emulate the previously recorded ports.

\section intro_sec Description
PortPlayer allows to replay files created by PortRecorder and to emulate the previously recorded ports.

Provides :
-	reading a pre-recorded stream on a set of ports

\section lib_sec Libraries
- YARP_OS.lib
- ACE.lib

\section parameters_sec Parameters
--file <FILENAME> - specifies a configuration file specifying the module parameters. 
                    The default configuration file is in the module's path

--inputFile <INPUTFILE> - specify the file where containing recorded data.
						Default is "intput.txt"

\section portsc_sec Interface: Ports Created and data format
The created ports are "clones" of the recorded ports.

\section tested_os_sec Tested OS
Windows XP: working
Windows Vista / 7 : probably. Not tested.
Linux : Probably. Not tested.

\section example_sec Example Instantiation of the module
moduleRecorder --name /myRecorder --sourceNames portsToRecord.txt --outputFile dumpedData.txt

\author S.LALLEE and S.SKACHEK 05/02/10  05/02/10


Copyright: TBD.

This file can be edited at ...\trunk\chris\src\PortPlayer
**/


#include "portPlayer.h"



int main (int argc, char* argv[])
{
    Network yarp;
	PortPlayer player;
	return player.runModule(argc,argv);
}
