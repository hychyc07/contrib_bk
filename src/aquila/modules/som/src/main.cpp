//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, <Martin Peniak - www.martinpeniak.com>																																					//
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//

/**
*
@ingroup icub_module
\defgroup icub_selfOrganisingMap selfOrganisingMap

\section intro_sec Description
This Aquila-compliant module trains self-organising maps

\section lib_sec Libraries
- CUDA 5.0
- YARP

\section parameters_sec Parameters
Launch this module with --help parameter to see the list of possible options

\section portsa_sec Ports Accessed
- none

\section portsc_sec Ports Created
Input ports:
- /selfOrganisingMap/[id]:i
    - input port to which Aquila connects

Output ports:
- /selfOrganisingMap/[id]:o
    - output port to which Aquila connects

\section in_files_sec Input Data Files
Needs a training file, which can be set from:
    - terminal via the following command: '--input [FILENAME]'
    - Aquila via the top-level menu

\section out_data_sec Output Data Files
By default, trained self-organising map is saved to 'som.txt' file. This can be changed from:
    - terminal via the following command: 'set output [FILENAME]'
    - Aquila via the top-level menu

\section conf_file_sec Configuration Files
Default values for various paramters are loaded from a configuration file, which can be modified in /conf/config.ini

The file consists in a few sections:
\code
output som.txt
maxThreads 256
learningRate 0.1
sigma 0.0
numSubIterations 100
numOutputs 64
\endcode

\e output - output file used for saving trained self-organising map
\e maxThreads 256 - maximum number of GPU threads per block
\e learningRate 0.1 - learning rate used druing the training
\e sigma 0.0 - sigma parameters that changes the training mode if set to a non-zero value
\e numSubIterations - number of sub-iterations
\e numOutputs - number of self-organising map outputs

\section tested_os_sec Tested OS
Linux, OSX and Windows.

\section example_sec Example Instantiation of the Module
This will launch the training on the third GPU on the system (Id=2) for 1000 iterations reading
training data from the colour.txt file that can be found in /example folder.

selfOrganisingMap --input colour.txt --iterations 1000 --gpu 2

\author Martin Peniak
Copyright (C) 2008 RobotCub Consortium
CopyPolicy: Released under the terms of the BSD
This file can be edited at src/main.cpp.
**/

#include <QtCore/QCoreApplication>
#include "interface.h"

/**
* \brief Entry function.
* \param[in] argc - number of arguments
* \param[in] argv[] - argumnet list
* \return true if all went well
*/
int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("config.ini");
    rf.setDefaultContext("som/conf");
    rf.configure("AQUILA_ROOT", argc, argv);

    QCoreApplication *a = new QCoreApplication(argc, argv);
    Interface *i = new Interface();
    QObject::connect(i, SIGNAL(quitting()), i, SLOT(clean()), Qt::QueuedConnection);
    QObject::connect(i, SIGNAL(quitting()), a, SLOT(quit()), Qt::QueuedConnection);
    i->initialise(rf);

    return a->exec();
}
