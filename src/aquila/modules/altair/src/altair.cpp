//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Martin Peniak                                                                                                                                                                                //
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

#include <omp.h>
#include "altair.h"
#include "kernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

/*!
 * \brief Constructor.
 */
Altair::Altair()
{
    gpu = new GPU();
}

/*!
 * \brief Destructor.
 */
Altair::~Altair()
{
    clean();
}

/*!
 * \brief Opens image ports.
 */
bool Altair::openPorts(QString portPrefix)
{
	//open your ports here

	return true;
}

/*!
 * \brief Connects ports.
 */
void Altair::connectPorts()
{
	//conenct your ports here
}

/*!
 * \brief Disconnects ports.
 */
void Altair::disconnectPorts()
{
	//disconenct your ports here
}

/*!
 * \brief Thread loop.
 */
void Altair::run()
{
    running = true;
    emit started(1);

    while(running)
    {
	yarp::os::Time::delay(1);
	emit messageSet("waiting one second and nothing absolutely nothing");
    }

    emit stopped(2);
}

/*!
 * \brief Stops the thread.
 */
void Altair::stop()
{
    running  = false;
}

/*!
 * \brief Clean up.
 */
void Altair::clean()
{
	//do clean-up here
}

/*!
 * \brief Sets simulation mode.
 * \param[in] thresholdValue
 */
void Altair::setSimulationMode(bool active)
{
    simulationMode = active;
}


/*!
 * \brief Gets simulation mode.
 */
bool Altair::getSimulationMode()
{
    return simulationMode;
}

