/**
*
@ingroup icub_contrib_modules
\defgroup icub_YarpPortRecorder YarpPortRecorder 

Record a set of ports outputs to a file. YarpPortPlayer allows to replay these files and to emulate ports with the previously 
recorded files.

\section intro_sec Description
This module acts as a recorder for a given set of ports. You can then replay the file produced by this module in 
YarpPortPlayer in order to emulate the behavior of the recorded ports.

Provides :
-	dumping of a set of ports
-   delay before recording begins
-   flexible interval between consequtive port dumps
-   flexible amount of records for each port
-   autostop after all port recorders have finished recording
-   controled via rpc port 

Future features :
-	support for recording BufferedPort<ImageOf<T>>

\section lib_sec Libraries
- YARP_OS.lib
- ACE.lib

\section parameters_sec Parameters
Command line parameter (compulsory):
--file <FILENAME> - specifies a configuration file with module parameters. 
                    The default configuration file is in the module's path.
					At the moment the module can not be started without 
					a configuration file.

Configuration file fields:
name - is a module name and constitutes the first part of local port names
captureinterval - time interval between consequtive port reads in milliseconds
delayb4capture - the delay in milliseconds between module execution and recording
filesizelimit -  the limitation for the output file size (this check is not yet implemented so do not matter)
outputfilename - output file name
rpc            - name+rpc constitutes an rpc port name
[PORTS]
remote1 - name of the first port to record
local1 -  name+local1 constitutes a local port name which will be connected to remote1
nrecords1 - number of records to be made from the remote1 port. If zero then there is no record limit.
            
remote2 --! 
local2    !-> similar to remote1,local1,nrecords1 but for the second port.
nrecords2_! 

remote"n" --!
local"n"    !-> Instructions of how to record port "n".
nrecords"n"-!                     

remarks:
Can add as many ports to record as neccessary. The port numbering should have no gaps. 
As there is no stop on filesize yet implemented, should be carefull when recording with no record limit 
to avoid stuffing your hard drive.'Exit' or 'quit' commands typed into terminal window stops recording 
and shuts the module.


\section portsa_sec Ports Accessed
The ports accessed are defined in the configuration file (see parameters).
One port name by line, no space into the port name.


\section portsc_sec Interface: Ports Created and data format
The created ports are specified in the configuration file. The port name would consist of this module name 
with appended ending. Both the module name and endings are specified in the configuration file.

rpc port is created and can be used to control the recorder by sending p','a','u','s'   or   'r','e','c' 
commands into port. rpc replies with a bottle containing integer and string.
Integer value is 1 if command is taken and -1 if not. The string describes the status of the recorder.




\section tested_os_sec Tested OS
Windows XP / 7: Tested.
Windows Vista : Not tested.
Linux :  Not tested.

\section example_sec Example Instantiation of the module
YarpPortRecorder --file default.ini

\author S.LALLEE and S.SKACHEK 05/02/10  


Copyright: TBD.

This file can be edited at ...\trunk\chris\src\YarpPortRecorder
**/


#include "YarpPortRecorder.h"



int main (int argc, char* argv[])
{
    Network yarp;
	YarpPortRecorder recorder;
	return recorder.runModule(argc,argv);
}