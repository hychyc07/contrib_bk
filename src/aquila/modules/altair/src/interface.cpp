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

#include <QTextStream>
#include <QMetaType>
#include <time.h>
#include "interface.h"

/*!
 * \brief Constructor.
 */
Interface::Interface()
{
    altair = new Altair();
}

/*!
 * \brief Initialisation.
 * \note This function creates input and output ports with relevent names and IDs. It also sets parameters that were found
 * \note in the configuration file or passed from terminal, which would override configuration file parameter values.
 * \note If the module was initialised with '--ping' parameter then a ping port is temporarily created so that Aquila can
 * \note detect that this module was compiled and its working ok. This was implemented to support remote module launching.
 * \param[in] rf - ResourceFinder object
 */
void Interface::initialise(yarp::os::ResourceFinder &rf)
{
    moduleName = "altair";

    if(!rf.check("ping"))
    {
        //id is used identify an instance of this module
        instance = rf.find("id").asInt();

        //port names
        QString portPrefix = QString("/") + moduleName + QString("/") + QString::number(instance);
        inputPortName =  portPrefix + QString(":i");
        outputPortName = portPrefix + QString(":o");

        //get parameters
        if(rf.check("cpu")) altair->gpu->setGPUMode(false);
        altair->gpu->setDevice(rf.find("gpu").asInt());
        if(rf.check("robot"))
        {
            robotName = rf.find("robot").asString().c_str();
            if(robotName == "icubSim")
            {
                altair->setSimulationMode(true);
            }
            else if(robotName == "icub")
            {
                altair->setSimulationMode(false);
            }
        }

        //open altair ports
        if(!altair->openPorts(portPrefix))
        {
            qCritical("failed to open %s ports", moduleName.toStdString().c_str());
            emit quitting();
        }
        else
        {
            altair->connectPorts();
        }

        //open input port
        if(!inputPort.open(inputPortName.toStdString().c_str()))
        {
            qCritical("failed to open input port %s", inputPortName.toStdString().c_str());
            emit quitting();
        }
        else
        {
            //start listening for incomming messages
            this->start();
        }

        //open output port
        if(!outputPort.open(outputPortName.toStdString().c_str()))
        {
            qCritical("failed to open output port %s", inputPortName.toStdString().c_str());
            emit quitting();
        }
        else
        {
            //initialise messenger
            messenger = new Messenger(outputPort);
            QObject::connect(altair, SIGNAL(messageSet(QString)), messenger, SLOT(sendMessage(QString)));
            QObject::connect(altair, SIGNAL(started(int)), messenger, SLOT(sendStatus(int)));
            QObject::connect(altair, SIGNAL(stopped(int)), messenger, SLOT(sendStatus(int)));
        }

        //determine if the module runs from terminal or was initialised by Aquila
        if(rf.check("start"))
        {
            altair->terminalMode = true;
            altair->start();
            waitForConsole();
        }
        else
        {
            qDebug("%s: running in GUI mode and listening on %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
            qDebug("%s: to exit, send 'quit' command to %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
            altair->terminalMode = false;
        }
    }
    else
    {
        //get the port id
        int portID = rf.find("ping").asInt();

        //open port
        inputPort.open(QString(QString("/") + moduleName + QString("/ping/") + QString::number(portID)).toStdString().c_str());

        //wait till timed-out, close port and exit
        Time::delay(5);
        inputPort.interrupt();
        inputPort.close();
        emit quitting();
    }
}

/*!
 * \brief Waits for user input and close the application.
 */
void Interface::waitForConsole()
{
    QTextStream cin(stdin);
    qDebug("%s: running in terminal mode and listening on %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
    qDebug("%s: press <Enter> to exit", moduleName.toStdString().c_str());
    if(!cin.readLine().isNull())
    {
        emit quitting();
    }
}

/*!
 * \brief Respond to a message sent to the input port.
 */
void Interface::run()
{
    while(isRunning())
    {
        if(inputPort.read(command))
        {
            if(command.get(0).asString()=="start")
            {
                altair->start();
                messenger->sendMessage("starting");
            }
            else if(command.get(0).asString()=="stop")
            {
                altair->stop();
                messenger->sendMessage("stopping");
            }
            else if(command.get(0).asString()=="set")
            {
                if(command.get(1).asString()=="gpu")
                {
                    altair->gpu->setGPUMode(true);
                    altair->gpu->setDevice(command.get(2).asInt());
                    messenger->sendMessage(QString("GPU device set to ") + QString::number(altair->gpu->getDevice()));
                }
                else if(command.get(1).asString()=="cpu")
                {
                    altair->gpu->setGPUMode(false);
                    messenger->sendMessage(QString("CPU mode set"));
                }
                else if(command.get(1).asString()=="robot")
                {
                    robotName = command.get(2).asString().c_str();
                    if(robotName == "icubSim" && altair->getSimulationMode()==false)
                    {
			//do something here

                        messenger->sendMessage("switched connection to iCub simulator");
                    }
                    else if(robotName == "icub" && altair->getSimulationMode()==true)
                    {
			//do something here

                        messenger->sendMessage("switched connection to iCub robot");
                    }
                }
            }
            else if(command.get(0).asString()=="get")
            {
                if(command.get(1).asString()=="parameters")
                {
                    sendParameters();
                    messenger->sendMessage("parameters sent");
                }
                else if(command.get(1).asString()=="gpu")
                {
                    if(command.get(2).asString()=="id")
                    {
                        Bottle b;
                        b.addString("gpu");
                        b.addInt(altair->gpu->getDevice());
                        outputPort.write(b);
                        messenger->sendMessage("GPU device id sent");
                    }
                    else if(command.get(2).asString()=="list")
                    {
                        messenger->sendGpuDeviceList(altair->gpu->getDeviceList());
                        messenger->sendMessage("GPU device list sent");
                    }
                }
            }
            else if(command.get(0).asString()=="quit")
            {
                messenger->sendMessage("quitting");
                emit quitting();
            }
        }
    }
}

/*!
 * \brief Cleans up.
 */
void Interface::clean()
{
    inputPort.interrupt();
    outputPort.interrupt();
    inputPort.close();
    outputPort.close();
}

/*!
 * \brief Sends default parameters to the output port.
 */
void Interface::sendParameters()
{
    Bottle b;
    b.addString("parameters");

	//add your parameteres 

    outputPort.write(b);
}

