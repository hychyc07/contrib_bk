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
#include "interface.h"

/*!
 * \brief Constructor.
 */
Interface::Interface()
{
    mtrnn = new MTRNN();

    qRegisterMetaType< QVector<float> >("QVector<float>");
    QObject::connect(mtrnn, SIGNAL(errorSet(int, float)), this, SLOT(sendError(int, float)));
    QObject::connect(mtrnn, SIGNAL(trainingStarted()), this, SLOT(processTrainingStart()));
    QObject::connect(mtrnn, SIGNAL(trainingStopped(QVector<float>)), this, SLOT(processTrainingStop(QVector<float>)));
}

/*!
 * \brief Configures module.
 * \note This function creates input and output ports with relevent names and IDs. It also sets parameters that were found
 * \note in the configuration file or passed from terminal, which would override configuration file parameter values.
 * \note If the module was initialised with '--ping' parameter then a ping port is temporarily created so that Aquila can
 * \note detect that this module was compiled and its working ok. This was implemented to support remote module launching.
 * \param[in] rf - ResourceFinder object
 */
void Interface::initialise(yarp::os::ResourceFinder &rf)
{
    moduleName = "mtrnn";

	if(!rf.check("ping"))
	{
        //id is used identify an instance of this module
		instance = rf.find("id").asInt();

        //port names
        inputPortName = QString("/") + moduleName + QString("/") + QString::number(instance) + QString(":i");
        outputPortName = QString("/") + moduleName + QString("/") + QString::number(instance) + QString(":o");

        //get parameters
        if(rf.check("cpu")) mtrnn->gpu->setGPUMode(false);

        //if the 'gpu' parameter is followed by a string then mutiple GPUs have been requested
        //otherwise, if it is followed by an int then only a single GPU has been requested
        if(rf.find("gpu").isString())
        {
            QVector<int> device;
            QString tmp = rf.find("gpu").asString().c_str();
            QStringList deviceList = tmp.split(",");

            for(int i=0; i<deviceList.size(); i++)
            {
                device.append(deviceList.at(i).toInt());
            }

            mtrnn->gpu->setDevices(device);
        }
        else if(rf.find("gpu").isInt())
        {
            mtrnn->gpu->setDevice(rf.find("gpu").asInt());
        }

        mtrnn->setNetworkFile(rf.find("output").asString().c_str());
        mtrnn->setShowProgress(rf.check("showProgress"));
        mtrnn->setMaxThreads(rf.find("maxThreads").asInt());
        mtrnn->setMaxIterations(rf.find("iterations").asInt());
        mtrnn->math->setSeed(rf.find("seed").asInt());
        mtrnn->setIODeltaT(rf.find("ioDelta").asInt());
        mtrnn->setFastDeltaT(rf.find("fastDelta").asInt());
        mtrnn->setSlowDeltaT(rf.find("slowDelta").asInt());
        mtrnn->setNumFastNeurons(rf.find("numFastNeurons").asInt());
        mtrnn->setNumSlowNeurons(rf.find("numSlowNeurons").asInt());
        mtrnn->setFeedbackInterval(rf.find("feedbackInterval").asInt());
        mtrnn->setInitWeightRange((float)rf.find("weightRange").asDouble());
        mtrnn->setThreshold((float)rf.find("threshold").asDouble());
        mtrnn->setLearningRate((float)rf.find("learningRate").asDouble());
        mtrnn->setMomentum((float)rf.find("momentum").asDouble());

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
            qCritical("failed to open output port %s", outputPortName.toStdString().c_str());
            emit quitting();
        }
        else
        {
            //initialise messenger
            messenger = new Messenger(outputPort);
            QObject::connect(mtrnn, SIGNAL(messageSet(QString)), messenger, SLOT(sendMessage(QString)));
            QObject::connect(mtrnn, SIGNAL(progressSet(int)), messenger, SLOT(sendProgress(int)));
        }

        //determine if the module runs from terminal or was initialised by Aquila
        if(rf.check("input"))
        {
            mtrnn->terminalMode = true;
            mtrnn->setTrainingFile(rf.find("input").asString().c_str());
            mtrnn->loadTrainingData();
            mtrnn->initialise();
            waitForConsole();
        }
        else
        {
            mtrnn->terminalMode = false;
            qDebug("%s: running in GUI mode and listening on %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
            qDebug("%s: to exit, send 'quit' command to %s", moduleName.toStdString().c_str(), inputPortName.toStdString().c_str());
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
                    mtrnn->setLearningRate((float)command.get(2).asDouble());
                    mtrnn->setMomentum((float)command.get(3).asDouble());
                    mtrnn->setInitWeightRange((float)command.get(4).asDouble());
                    mtrnn->setThreshold((float)command.get(5).asDouble());
                    mtrnn->setMaxIterations(command.get(6).asInt());
                    mtrnn->math->setSeed(command.get(7).asInt());
                    mtrnn->setNumFastNeurons(command.get(8).asInt());
                    mtrnn->setNumSlowNeurons(command.get(9).asInt());
                    mtrnn->setIODeltaT(command.get(10).asInt());
                    mtrnn->setFastDeltaT(command.get(11).asInt());
                    mtrnn->setSlowDeltaT(command.get(12).asInt());
                    mtrnn->setFeedbackInterval(command.get(13).asInt());
                    messenger->sendMessage("new parameters set");
                }
                else if(command.get(1).asString()=="input")
                {
                    if(command.size() == 3)
                    {
                        mtrnn->setTrainingFile(command.get(2).asString().c_str());
                        messenger->sendMessage(QString("input initialsed from ") + mtrnn->getTrainingFile());
                    }
                    else
                    {
                        mtrnn->setTrainingData(command);
                        messenger->sendMessage("input initialsed from port data");
                    }
                }
                if(command.get(1).asString()=="gpu")
                {
                    //enable GPU-mode
                    mtrnn->gpu->setGPUMode(true);

                    //set single or multi-gpu mode
                    if(command.size()>3)
                    {
                        QVector<int> device;
                        QString list;

                        for(int i=0; i<command.size()-2; i++)
                        {
                            device.append(command.get(i+2).asInt());
                            list.append(QString::number(command.get(i+2).asInt()) + QString(" "));
                        }
                        mtrnn->gpu->setDevices(device);
                        messenger->sendMessage(QString("GPU devices set to ") + list);
                    }
                    else
                    {
                        int id = command.get(2).asInt();

                        if(id < mtrnn->gpu->numDevices)
                        {
                            mtrnn->gpu->setDevice(id);
                            messenger->sendMessage(QString("GPU device set to ") + QString::number(mtrnn->gpu->getDevice()));
                        }
                        else
                        {
                            messenger->sendMessage("GPU device id does not exist");
                        }
                    }
                }
                else if(command.get(1).asString()=="cpu")
                {
                    mtrnn->gpu->setGPUMode(false);
                    messenger->sendMessage(QString("CPU mode set"));
                }
                else if(command.get(1).asString()=="output")
                {
                    mtrnn->setNetworkFile(command.get(2).asString().c_str());
                    messenger->sendMessage(QString("output set to ") + mtrnn->getNetworkFile());
                }
            }
            else if(command.get(0).asString()=="get")
            {
                if(command.get(1).asString()=="parameters")
                {
                    sendParameters(mtrnn->getLearningRate(), mtrnn->getMomentum(), mtrnn->getWeightRange(), mtrnn->getThreshold(), mtrnn->getMaxIterations(), mtrnn->math->getSeed(), mtrnn->getNumFastNeurons(), mtrnn->getNumSlowNeurons(), mtrnn->getIODeltaT(), mtrnn->getFastDeltaT(), mtrnn->getSlowDeltaT(), mtrnn->getFeedbackInterval());
                    messenger->sendMessage("parameters sent");
                }
                else if(command.get(1).asString()=="gpu")
                {
                    if(command.get(2).asString()=="id")
                    {
                        Bottle b;
                        b.addString("gpu");
                        b.addInt(mtrnn->gpu->getDevice());
                        outputPort.write(b);
                        messenger->sendMessage("GPU device id sent");
                    }
                    else if(command.get(2).asString()=="list")
                    {
                        messenger->sendGpuDeviceList(mtrnn->gpu->getDeviceList());
                        messenger->sendMessage("GPU device list sent");
                    }
                }
                else if(command.get(1).asString()=="output")
                {
                    messenger->sendMessage(QString("output set to ") + mtrnn->getNetworkFile());
                }
                else if(command.get(1).asString()=="network")
                {
                    sendNetwork();
                    messenger->sendMessage("neural network sent");
                }
            }
            else if(command.get(0).asString()=="network")
            {
                if(command.get(1).asString()=="train")
                {
                    if(mtrnn->loadTrainingData())
                    {
                        mtrnn->initialise();
                        messenger->sendMessage("training started");
                    }
                }
                else if(command.get(1).asString()=="save")
                {
                    messenger->sendMessage("saving neural network");
                    mtrnn->setNetworkFile(command.get(2).asString().c_str());

                    if(!mtrnn->getNetworkFile().isEmpty())
                    {
                        mtrnn->saveNetwork();
                    }
                    else
                    {
                        messenger->sendMessage("failed to save neural network, no file output specified");
                    }
                }
            }
            else if(command.get(0).asString()=="abort")
            {
                mtrnn->stop();
                messenger->sendMessage("aborting");
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
    mtrnn->stop();
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
void Interface::processTrainingStop(QVector<float> errors)
{
    Bottle b;
    b.addString("errors");
    for(int i=0; i<errors.size(); i++)
    {
        b.addDouble((double)errors.at(i));
    }
    outputPort.write(b);

    messenger->sendStatus(2); //training ended
}

/*!
 * \brief Sends error at particular iteration to the output port.
 * \param[in] iteration - current itertaion
 * \param[in] error - current error
 */
void Interface::sendError(int iteration, float error)
{
    Bottle b;
    b.addString("error");
    b.addInt(iteration);
    b.addDouble((double)error);
    outputPort.write(b);
}

/*!
 * \brief Sends the weights of neural network to the output port.
 */
void Interface::sendNetwork()
{
    Bottle b;
    b.addString("network");

    QVector<float> weights =  mtrnn->getWeights();
    for(int i=0; i<weights.size(); i++)
    {
        b.addDouble((double)weights.at(i));
    }

    outputPort.write(b);
}

/*!
 * \brief Sends parameters to the output port.
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] weightRange - initial range of weights
 * \param[in] threshold - error threshold
 * \param[in] iterations - number of iterations
 * \param[in] seed - random number generator seed
 * \param[in] numFastNeurons - number of fast neurons
 * \param[in] numSlowNeurons - number of slow neurons
 * \param[in] ioDeltaT - input-output delta-t value
 * \param[in] fastDeltaT - fast delta-t value
 * \param[in] slowDeltaT - slow delta-t value
 * \param[in] portUpdateInterval - port update interval (Aquila plotting data)
 */
void Interface::sendParameters(float learningRate, float momentum, float weightRange, float threshold, int iterations, int seed, int numFastNeurons, int numSlowNeurons, int ioDeltaT, int fastDeltaT, int slowDeltaT, int portUpdateInterval)
{
    Bottle b;
    b.addString("parameters");
    b.addDouble((double)learningRate);
    b.addDouble((double)momentum);
    b.addDouble((double)weightRange);
    b.addDouble((double)threshold);
    b.addInt(iterations);
    b.addInt(seed);
    b.addInt(numFastNeurons);
    b.addInt(numSlowNeurons);
    b.addInt(ioDeltaT);
    b.addInt(fastDeltaT);
    b.addInt(slowDeltaT);
    b.addInt(portUpdateInterval);
    outputPort.write(b);
}
