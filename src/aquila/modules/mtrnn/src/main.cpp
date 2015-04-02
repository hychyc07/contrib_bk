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
\defgroup icub_MTRNN MTRNN

\section intro_sec Description
This Aquila-compliant module trains multiple time-scales recurrent neural network (MTRNN)
using back-propagation through time training (BPTT) algorithm

\section lib_sec Libraries
- CUDA 5.0
- YARP

\section parameters_sec Parameters
Launch this module with --help parameter to see the list of possible options

\section portsa_sec Ports Accessed
- none

\section portsc_sec Ports Created
Input ports:
- /mtrnn/[id]:i
    - input port to which Aquila connects

Output ports:
- /mtrnn/[id]:o
    - output port to which Aquila connects

\section in_files_sec Input Data Files
Needs a training file, which can be set from:
    - terminal via the following command: '--training_set [FILENAME]'
    - Aquila via the top-level menu

\section out_data_sec Output Data Files
By default, trained MTRNN is saved to 'mtrnn.txt' file. This can be changed from:
    - terminal via the following command: 'set output [FILENAME]'
    - Aquila via the top-level menu

\section conf_file_sec Configuration Files
Default values for various paramters are loaded from a configuration file, which can be modified in /conf/config.ini

The file consists in a few sections:
\code
output mtrnn.txt
showProgress 1
maxThreads 256
iterations 10000
ioDelta 1
fastDelta 1
slowDelta 1
numFastNeurons 10
numSlowNeurons 10
weightRange 0.025
threshold 0.0001
learningRate 0.005
momentum 0.0
\endcode

\e output - output file used for saving trained MTRNN
\e show_progress - if 1 then training progress will be shown if 0 then not
\e maxThreads 256 - maximum number of GPU threads per block
\e iterations 10000 - maximum number of iterations
\e ioDelta 1 - delta_t value that of input-output neurons
\e fastDelta 1 - delta_t value that of fast neurons
\e slowDelta 1 - delta_t value that of slow neurons
\e numFastNeurons 10 - number of fast neurons
\e numSlowNeurons 10 - number of slow neurons
\e weightRange 0.025 - initial range of synapses (in this case from -0.025 to 0.025)
\e threshold 0.0001 - error threshold that, once reached, will cause the training to stop
\e learningRate 0.005 - learning rate used druing the training
\e momentum 0.0 - momentum used during the training

\section tested_os_sec Tested OS
Linux, OSX and Windows.

\section example_sec Example Instantiation of the Module
This will launch the training on 3 different GPUs (IDs: 0,1,2), which will divide the sequences provided in 'training_set.txt':

mtrnn --gpu 0 1 2 --trainingSet training_set.txt --iterations 10000 --learningRate 0.005

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
    rf.setDefaultContext("mtrnn/conf");
    rf.configure("AQUILA_ROOT", argc, argv);

    QCoreApplication *a = new QCoreApplication(argc, argv);
    Interface *i = new Interface();
    QObject::connect(i, SIGNAL(quitting()), i, SLOT(clean()), Qt::QueuedConnection);
    QObject::connect(i, SIGNAL(quitting()), a, SLOT(quit()), Qt::QueuedConnection);
    i->initialise(rf);

    return a->exec();
}
