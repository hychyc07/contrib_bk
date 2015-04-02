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

#include "esnInterface.h"

/*!
 * \brief Constructor.
 * \param[in] pGUI - pointer to GUI
 */
ESNInterface::ESNInterface(GUI *pGUI) : Interface(pGUI)
{
}

/*!
 * \brief Processes data received from module.
 */
void ESNInterface::processPortData()
{
    if(receivedBottle.get(0).asString()=="parameters")
    {
        emit parametersReceived(receivedBottle.get(1).asInt());
    }
    else if(receivedBottle.get(0).asString()=="activity")
    {
        processActivity(receivedBottle);
    }
}

/*!
 * \brief Processes ESN activity.
 * \param[in] bottle - ESN activity
 */
void ESNInterface::processActivity(yarp::os::Bottle &bottle)
{
    QVector<float> activity;
    for(int i=1; i<bottle.size(); i++)
    {
        activity.push_back((float)(bottle.get(i).asDouble()));
    }
    emit networkActivityReceived(activity);
}

/*!
 * \brief Sends parameters to module.
 * \param[in] esnSize - ESN size
 */
void ESNInterface::sendParameters(int esnSize)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("parameters");
    b.addInt(esnSize);
    outputPort.write(b);
}
