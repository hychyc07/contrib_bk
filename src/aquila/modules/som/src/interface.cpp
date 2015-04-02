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

#include <QTextStream>
#include <QMetaType>
#include <time.h>
#include "interface.h"

/*!
 * \brief Constructor.
 */
Interface::Interface()
{
    som = new SelfOrganisingMap();

    qRegisterMetaType< QVector<float> >("QVector<float>");
    QObject::connect(som, SIGNAL(trainingStarted()), this, SLOT(processTrainingStart()));
    QObject::connect(som, SIGNAL(trainingStopped(QVector<float>,QVector<float>,float)), this, SLOT(processTrainingStop(QVector<float>,QVector<float>,float)));
    QObject::connect(som, SIGNAL(mapSet(QVector<float>)), this, SLOT(sendMap(QVector<float>)));
    QObject::connect(som, SIGNAL(numInputsSet(int)), this, SLOT(sendNumInputs(int)));
    QObject::connect(som, SIGNAL(winnerSet(int)), this, SLOT(sendWinner(int)));
}

/*!
 * \brief Initialisation.
 * \note This function creates input and output ports with relevent names and IDs. It also sets parameters that were found
 * \note in the configuration file or passed from terminal, which would override configuration file parameter values.
 * \note If the module was initialised with '--ping' parameter then a ping port is temporarily created so that Aquila can
 * \note detect that this module was compiled and its working ok. This was implemented to support remote module launching.
 * \param[in] rf - ResourceFinder object
 * \return true on success
 */
void Interface::initialise(yarp::os::ResourceFinder &rf)
{
    moduleName = "som";

	if(!rf.check("ping"))
	{
        //id is used identify an instance of this module
		instance = rf.find("id").asInt();

        //port names
        inputPortName = QString("/") + moduleName + QString("/") + QString::number(instance) + QString(":i");
        outputPortName = QString("/") + moduleName + QString("/") + QString::number(instance) + QString(":o");

        //get parameters
        if(rf.check("cpu"))  som->gpu->setGPUMode(false);
        if(rf.check("seed")) som->math->setSeed(rf.find("seed").asInt());
        else	             som->math->setSeed((int)time(NULL));
        som->gpu->setDevice(rf.find("gpu").asInt());
        som->setMaxThreads(rf.find("maxThreads").asInt());
        som->setInitLearningRate((float)rf.find("learningRate").asDouble());
        som->setSigma((float)rf.find("sigma").asDouble());
        som->setNumSubiterations(rf.find("numSubIterations").asInt());
        som->setNumOutputs(rf.find("numOutputs").asInt());
        som->setVisualiseProgress(rf.check("visualiseProgress"));
        som->setIterationPause(rf.find("iterationPause").asInt());
        som->setTrainingFile(rf.find("input").asString().c_str());
        som->setMapFile(rf.find("output").asString().c_str());

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
            QObject::connect(this, SIGNAL(messageSet(QString)), messenger, SLOT(sendMessage(QString)));
            QObject::connect(som, SIGNAL(messageSet(QString)), messenger, SLOT(sendMessage(QString)));
            QObject::connect(som, SIGNAL(progressSet(int)), messenger, SLOT(sendProgress(int)));
        }

        //determine if the module runs from terminal or was initialised by Aquila
        if(som->getTrainingFile().isEmpty())
		{
            qDebug("%s: running in GUI mode and listening on %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
            qDebug("%s: to exit, send 'quit' command to %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
        }
		else
		{
            som->loadTrainingSet();
            som->randomiseWeights();
            som->start();
            waitForConsole();
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
            if(command.get(0).asString()=="set")
            {
                if(command.get(1).asString()=="parameters")
                {
                    som->setInitLearningRate((float)command.get(2).asDouble());
                    som->setNumSubiterations(command.get(3).asInt());
                    som->setNumOutputs(command.get(4).asInt());
                    som->setIterationPause(command.get(5).asInt());
                    messenger->sendMessage("new parameters set");
                }
                else if(command.get(1).asString()=="input")
                {
                    if(command.size()==3)
                    {
                        som->setTrainingFile(command.get(2).asString().c_str());
                        som->loadTrainingSet();
                        som->randomiseWeights();
                        messenger->sendMessage(QString("input initialsed from ") + som->getTrainingFile());
                    }
                    else
                    {
                        som->loadTrainingSet(command);
                        som->randomiseWeights();
                        messenger->sendMessage(QString("input initialsed from port data"));
                    }
                }
                else if(command.get(1).asString()=="realTimeInput")
                {
                    som->setLiveInput(command);
                    som->singleStep(command.get(4).asInt());
                }
                else if(command.get(1).asString()=="gpu")
                {
                    som->gpu->setGPUMode(true);
                    som->gpu->setDevice(command.get(2).asInt());
                    messenger->sendMessage(QString("GPU device set to ") + QString::number(som->gpu->getDevice()));
                }
                else if(command.get(1).asString()=="cpu")
                {
                    som->gpu->setGPUMode(false);
                    messenger->sendMessage(QString("CPU mode set"));
                }
                else if(command.get(1).asString()=="learningRate")
                {
                    som->setInitLearningRate((float)command.get(2).asDouble());
                    messenger->sendMessage(QString("learning rate set to ") + QString::number(som->getInitLearningRate()));
                }
                else if(command.get(1).asString()=="numSubIterations")
                {
                    som->setNumSubiterations(command.get(2).asInt());
                    messenger->sendMessage(QString("number of sub-iterations set to ") + QString::number(som->getNumSubiterations()));
                }
                else if(command.get(1).asString()=="numOutputs")
                {
                    som->setNumOutputs(command.get(2).asInt());
                    messenger->sendMessage(QString("number of outputs set to ") + QString::number(som->getNumOutputs()));
                }
                else if(command.get(1).asString()=="iterationPause")
                {
                    som->setIterationPause(command.get(2).asInt());
                    messenger->sendMessage(QString("iteration pause was set to ") + QString::number(som->getIterationPause()));
                }
            }
            else if(command.get(0).asString()=="get")
            {
                if(command.get(1).asString()=="parameters")
                {
                    sendParameters(som->getInitLearningRate(), som->getNumSubiterations(), som->getNumOutputs(), som->getIterationPause());
                    messenger->sendMessage("parameters sent");
                }
                else if(command.get(1).asString()=="input")
                {
                    Bottle b;
                    b.addString("input");
                    b.addString(som->getTrainingFile().toStdString().c_str());
                    outputPort.write(b);
                    messenger->sendMessage("input sent");
                }
                else if(command.get(1).asString()=="gpu")
                {
                    if(command.get(2).asString()=="id")
                    {
                        Bottle b;
                        b.addString("gpu");
                        b.addInt(som->gpu->getDevice());
                        outputPort.write(b);
                        messenger->sendMessage("GPU device id sent");
                    }
                    else if(command.get(2).asString()=="list")
                    {
                        messenger->sendGpuDeviceList(som->gpu->getDeviceList());
                        messenger->sendMessage("GPU device list sent");
                    }
                }
                else if(command.get(1).asString()=="learningRate")
                {
                    Bottle b;
                    b.addString("learningRate");
                    b.addDouble(som->getInitLearningRate());
                    outputPort.write(b);
                    messenger->sendMessage("learning rate sent");
                }
                else if(command.get(1).asString()=="numSubIterations")
                {
                    Bottle b;
                    b.addString("numSubIterations");
                    b.addInt(som->getNumSubiterations());
                    outputPort.write(b);
                    messenger->sendMessage("sub-iterations sent");
                }
                else if(command.get(1).asString()=="numInputs")
                {
                    Bottle b;
                    b.addString("numInputs");
                    b.addInt(som->getNumInputs());
                    outputPort.write(b);
                    messenger->sendMessage("number of inputs sent");
                }
                else if(command.get(1).asString()=="numOutputs")
                {
                    Bottle b;
                    b.addString("numOutputs");
                    b.addInt(som->getNumOutputs());
                    outputPort.write(b);
                    messenger->sendMessage("number of outputs sent");
                }
                else if(command.get(1).asString()=="iterationPause")
                {
                    Bottle b;
                    b.addString("iterationPause");
                    b.addInt(som->getIterationPause());
                    outputPort.write(b);
                    messenger->sendMessage("iteration pause time sent");
                }
                else if(command.get(1).asString()=="map")
                {
                    sendMap(som->getCurrentMap());
                    messenger->sendMessage("map sent");
                }
            }
            else if(command.get(0).asString()=="map")
            {
                if(command.get(1).asString()=="train")
                {
                    som->start();
                    messenger->sendMessage("training started");
                }
                else if(command.get(1).asString()=="save")
                {
                    som->setMapFile(command.get(2).asString().c_str());
                    if(!som->getMapFile().isEmpty())
                    {
                        som->saveMap();
                        messenger->sendMessage("map saved");
                    }
                    else
                    {
                        messenger->sendMessage("failed to save map");
                    }
                }
            }
            else if(command.get(0).asString()=="quit")
            {
                messenger->sendMessage("quitting");
                emit quitting();
            }
            else if(command.get(0).asString()=="abort")
            {
                messenger->sendMessage("aborting");
                som->stop();
            }
            else if(command.get(0).asString()=="visualiseProgress")
            {
                if(command.get(1).asString()=="on")
                {
                    som->setVisualiseProgress(true);
                    messenger->sendMessage("visualisation is on");
                }
                else if(command.get(1).asString()=="off")
                {
                    som->setVisualiseProgress(false);
                    messenger->sendMessage("visualisation is off");
                }
            }
        }
    }
}

/*!
 * \brief Cleans up.
 */
void Interface::clean()
{
    som->stop();
    inputPort.interrupt();
    outputPort.interrupt();
    inputPort.close();
    outputPort.close();
}

/*!
 * \brief Processes training start event.
 */
void Interface::processTrainingStart()
{
    messenger->sendStatus(1); //training started
}

/*!
 * \brief Processes training stop event.
 */
void Interface::processTrainingStop(QVector<float> map, QVector<float> limits, float time)
{
    sendTrainingTime(time);
    sendDataPointLimits(limits);
    sendMap(map);
    messenger->sendStatus(2); //training ended
}

/*!
 * \brief Sends the winner to the output port.
 * \note This function send the winner but also instance ID number in case other modules need it.
 */
void Interface::sendWinner(int winnerID)
{
    Bottle b;
    b.addString("set");
    b.addString("winner");
    b.addInt(winnerID);
    b.addString("somID");
    b.addInt(instance);
    outputPort.write(b);
}

/*!
 * \brief Sends the weights of self-organising map to the output port.
 */
void Interface::sendMap(QVector<float> map)
{
    Bottle b;
    b.addString("map");
    for(int i=0; i<map.size(); i++)
    {
        b.addDouble((double)map.at(i));
    }
    outputPort.write(b);
}

/*!
 * \brief Sends number of inputs to the output port.
 */
void Interface::sendNumInputs(int numInputs)
{
    Bottle b;
    b.addString("numInputs");
    b.addInt(numInputs);
    outputPort.write(b);
}

/*!
 * \brief Sends parameters to the output port.
 * \param[in] learningRate - initial learning rate
 * \param[in] numSubIterations - number of sub-iterations
 * \param[in] numOutputs - number of outputs
 * \param[in] iterationPause - iteration pause
 */
void Interface::sendParameters(float learningRate, int numSubIterations, int numOutputs, int iterationPause)
{
    Bottle b;
    b.addString("parameters");
    b.addDouble((double)learningRate);
    b.addInt(numSubIterations);
    b.addInt(numOutputs);
    b.addInt(iterationPause);
    outputPort.write(b);
}

/*!
 * \brief Sends the total training time (in ms) to the output port.
 */
void Interface::sendTrainingTime(float time)
{
	Bottle b;
	b.addString("time");
    b.addDouble((double)time);
	outputPort.write(b);
}

/*!
 * \brief Sends the original data min and max values to the output port.
 * \note These values (limits) are used to keep the reference about original data values
 * \note in case the outputs of a slef-organising map need to be scaled back to original values.
 */
void Interface::sendDataPointLimits(QVector<float> limits)
{
	Bottle b;
	b.addString("limits");
    b.addDouble(limits.at(0));
    b.addDouble(limits.at(1));
	outputPort.write(b);
}
