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

#include "interface.h"

/*!
 * \brief Constructor.
 * \param[in] pGUI - pointer to GUI
 */
Interface::Interface(GUI *pGUI)
{
    //initialise module pointer
    module = pGUI;

    //initialise port names
    inputPortName.push_back(module->server + QString("/") + module->binary + QString("/") + QString::number(module->instance) + QString(":i"));
    outputPortName.push_back(module->server + QString("/") + module->binary + QString("/") + QString::number(module->instance) + QString(":o"));

    //open input port
    if(!inputPort.open(inputPortName.toStdString().c_str()))
    {
        qCritical(" - %s_gui(%i): unable to open %s", module->binary.toStdString().c_str(), module->instance, inputPortName.toStdString().c_str());
    }

    //open output port
    if(!outputPort.open(outputPortName.toStdString().c_str()))
    {
        qCritical(" - %s_gui(%i): unable to open %s", module->binary.toStdString().c_str(), module->instance, outputPortName.toStdString().c_str());
    }

    //connect gui with module
    yarp::os::Network::connect(outputPort.getName(), QString(QString("/") + module->binary + QString("/") + QString::number(module->instance) + QString(":i")).toStdString().c_str(), 0, true);
    yarp::os::Network::connect(QString(QString("/") + module->binary + QString("/") + QString::number(module->instance) + QString(":o")).toStdString().c_str(), inputPort.getName(), 0, true);
}

/*!
 * \brief Thread loop processing messages received from module.
 * \note This function is called when interface is started. Once new data has arrived, module non-specific signals are emitted.
 * \note After that, virtual processPortData function is called, which processes module-specific data and emits signals.
 */
void Interface::run()
{
    while(isRunning())
    {
        //read data from module
        if(inputPort.read(receivedBottle))
        {
            //process module-non-specific data
            if(receivedBottle.get(0).asString()=="gpuList")
            {
                processGpuList();
            }
            else if(receivedBottle.get(0).asString()=="message")
            {
                printMessage();
            }
            else if(receivedBottle.get(0).asString()=="status")
            {
                emit statusReceived(receivedBottle.get(1).asInt());
            }
            else if(receivedBottle.get(0).asString()=="progress")
            {
                emit progressReceived(receivedBottle.get(1).asInt());
            }

            //process module-specific data - this should be overridden by inheriting classes
            processPortData();
        }
        else
        {
            qDebug(" - %s_gui(%i): leaving interface event loop", module->binary.toStdString().c_str(), module->instance);
            break;
        }
    }
}

/*!
 * \brief Processes data about GPU devices found on server running module.
 * \note Once the data was processed a signal with GPU list is emitted.
 */
void Interface::processGpuList()
{
    QVector<QStringList> gpuList;

    for(int i=1; i<receivedBottle.size(); i++)
    {
        if(receivedBottle.get(i).asString() == "beginning")
        {
            i++;
            int id = 0;
            QStringList property;

            while(receivedBottle.get(i).asString() != "end")
            {
                property.append(receivedBottle.get(i).asString().c_str());
                id++;
                i++;
            }

            gpuList.append(property);
        }
    }

    emit gpuListReceived(gpuList);
}

/*!
 * \brief Prints out or logs message received from module.
 */
void Interface::printMessage()
{
    qDebug(" - %s_module(%i): %s", module->binary.toStdString().c_str(), module->instance, receivedBottle.get(1).asString().c_str());
}

/*!
 * \brief Sends parameters request to module.
 * \note Module responds back with the list of parameters.
 */
void Interface::sendParametersRequest()
{
    yarp::os::Bottle b;
    b.addString("get");
    b.addString("parameters");
    outputPort.write(b);
}

/*!
 * \brief Sends ID of selected GPU device to module.
 * \param[in] gpuID - GPU device identification number
 * \note This function is used in single-GPU mode.
 */
void Interface::sendGpuID(int gpuID)
{
    yarp::os::Bottle b;  
    b.addString("set");
    b.addString("gpu");
    b.addInt(gpuID);
    outputPort.write(b);
}

/*!
 * \brief Sends ID of selected GPU devices to module.
 * \param[in] gpuIDs - vetor of GPU device identification numbers
 * \note This function is used in multi-GPU mode.
 */
void Interface::sendGpuIDs(QVector<int> gpuIDs)
{
    yarp::os::Bottle b;   
    b.addString("set");
    b.addString("gpu");
    for(int i=0; i<gpuIDs.size(); i++)
    {
        b.addInt(gpuIDs.at(i));
    }
    outputPort.write(b);
}

/*!
 * \brief Sends simulation module to module.
 * \param[in] simulationMode - simulation mode
 * \note Module will use iCub simulator is simulationMode=1 otherswise it will user iCub robot.
 */
void Interface::sendSimulationMode(int simulationMode)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("robot");
    if(simulationMode)
    {
        b.addString("icubSim");
    }
    else
    {
        b.addString("icub");
    }
    outputPort.write(b);
}

/*!
 * \brief Sends CPU request to module, which will updates its execution mode to CPU.
 */
void Interface::sendCpuRequest()
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("cpu");
    outputPort.write(b);
}

/*!
 * \brief Sends request for list of GPU devices found on module server.
 */
void Interface::sendGpuListRequest()
{
    yarp::os::Bottle b;
    b.addString("get");
    b.addString("gpu");
    b.addString("list");
    outputPort.write(b);
}

/*!
 * \brief Sends request to start module.
 */
void Interface::sendStartRequest()
{
    yarp::os::Bottle b;
    b.addString("start");
    outputPort.write(b);
}

/*!
 * \brief Sends request to stop module.
 */
void Interface::sendStopRequest()
{
    yarp::os::Bottle b;
    b.addString("stop");
    outputPort.write(b);
}

/*!
 * \brief Sends request to abort module.
 */
void Interface::sendAbortRequest()
{
    yarp::os::Bottle b;
    b.addString("abort");
    outputPort.write(b);
}

/*!
 * \brief Sends request to quit module.
 */
void Interface::sendQuitRequest()
{
    yarp::os::Bottle b;
    b.addString("quit");
    outputPort.write(b);
}

/*!
 * \brief Closes interface.
 * \note The interruption of the input port causes thread to exit.
 */
void Interface::close()
{
    //interrupt ports
    inputPort.interrupt();
    outputPort.interrupt();

    //close ports
    inputPort.close();
    outputPort.close();
}

/*!
 * \brief Processes module specific data received from module.
 * \note Inheriting classes need to override this function.
 */
void Interface::processPortData()
{
}
