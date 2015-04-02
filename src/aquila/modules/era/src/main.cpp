//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse																																											//
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
\defgroup icub_ERA ERA

\section intro_sec Description
This Aquila-compliant module implements ERA

\section lib_sec Libraries
- CUDA 5.0
- YARP

\section parameters_sec Parameters
Launch this module with --help parameter to see the list of possible options

\section portsa_sec Ports Accessed
- none

\section portsc_sec Ports Created
Input ports:
- /era/[id]:i
    - input port to which Aquila connects

Output ports:
- /era/[id]:o
    - output port to which Aquila connects

\section in_files_sec Input Data Files

\section out_data_sec Output Data Files

\section conf_file_sec Configuration Files
Default values for various paramters are loaded from a configuration file, which can be modified in /conf/config.ini

The file consists in a few sections:
\code

\endcode

\section tested_os_sec Tested OS
Linux, OSX and Windows.

\section example_sec Example Instantiation of the Module

\author Anthony Morse
Copyright (C) 2008 RobotCub Consortium
CopyPolicy: Released under the terms of the FreeBSD
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
    rf.setDefaultContext("era/conf");
    rf.configure("AQUILA_ROOT", argc, argv);

    QCoreApplication *a = new QCoreApplication(argc, argv);
    Interface *i = new Interface();
    QObject::connect(i, SIGNAL(quitting()), i, SLOT(clean()), Qt::QueuedConnection);
    QObject::connect(i, SIGNAL(quitting()), a, SLOT(quit()), Qt::QueuedConnection);
    i->initialise(rf);

    return a->exec();
}
