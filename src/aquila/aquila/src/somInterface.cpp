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

#include "somInterface.h"

/*!
 * \brief Constructor.
 * \param[in] pGUI - pointer to GUI
 */
SOMInterface::SOMInterface(GUI *pGUI) : Interface(pGUI)
{
}

/*!
 * \brief Processes data received from module.
 */
void SOMInterface::processPortData()
{
    if(receivedBottle.get(0).asString()=="parameters")
    {
        emit parametersReceived((float)receivedBottle.get(1).asDouble(), receivedBottle.get(2).asInt(), receivedBottle.get(3).asInt(), receivedBottle.get(4).asInt());
    }
    else if (receivedBottle.get(0).asString()=="map")
    {
        processMap();
    }
    else if(receivedBottle.get(0).asString()=="numInputs")
    {
        emit numInputsReceived(receivedBottle.get(1).asInt());
    }
    else if(receivedBottle.get(0).asString()=="time")
    {
        emit trainingTimeReceived(receivedBottle.get(1).asDouble());
    }
    else if(receivedBottle.get(0).asString()=="limits")
    {
        emit dataPointLimitsReceived(receivedBottle.get(1).asDouble(), receivedBottle.get(2).asDouble());
    }
}

/*!
 * \brief Processes and broadcasts self-organising map.
 */
void SOMInterface::processMap()
{
    QVector<float> map;
    for(int i=1; i<receivedBottle.size(); i++)
    {
        map.append((float)receivedBottle.get(i).asDouble());
    }
    emit mapReceived(map);
}

/*!
 * \brief Sends parameters.
 * \param[in] learningRate - learning rate
 * \param[in] numSubIterations - number of sub-iterations
 * \param[in] numOutputs - number of outputs
 * \param[in] iterationPause - iteration pause
 */
void SOMInterface::sendParameters(float learningRate, int numSubIterations, int numOutputs, int iterationPause)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("parameters");
    b.addDouble((double)learningRate);
    b.addInt(numSubIterations);
    b.addInt(numOutputs);
    b.addInt(iterationPause);
    outputPort.write(b);
}

/*!
 * \brief Sends learning rate.
 * \param[in] learningRate - learning rate
 */
void SOMInterface::sendLearningRate(double learningRate)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("learninigRate");
    b.addDouble(learningRate);
    outputPort.write(b);
}

/*!
 * \brief Sends sub-iterations.
 * \param[in] numSubIterations - number of sub-iterations
 */
void SOMInterface::sendSubIterations(int numSubIterations)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("numSubIterations");
    b.addInt(numSubIterations);
    outputPort.write(b);
}

/*!
 * \brief Sends number of outputs.
 * \param[in] numOutputs - number of outputs
 */
void SOMInterface::sendNumOutputs(int numOutputs)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("numOutputs");
    b.addInt(numOutputs);
    outputPort.write(b);
}

/*!
 * \brief Sends iteration pause.
 * \param[in] iterationPause - iteration pause (in ms)
 */
void SOMInterface::sendIterationsPause(int iterationPause)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("iterationPause");
    b.addInt(iterationPause);
    outputPort.write(b);
}

/*!
 * \brief Sends visualise learning flag.
 * \param[in] visualiseLearning - learning visualisation flag
 */
void SOMInterface::sendVisualiseLearning(int visualiseLearning)
{
    yarp::os::Bottle b;
    b.addString("visualiseProgress");
    if(visualiseLearning)
    {
        b.addString("on");
    }
    else
    {
        b.addString("off");
    }
    outputPort.write(b);
}

/*!
 * \brief Sends training request to localhost module.
 * \param[in] fileName - name of the training file
 * \note This function is called when a localhost module is asked to load a training file.
 * \note Remotehost modules need to be sent training data via port using sendTrainingRequest function.
 */
void SOMInterface::sendTrainingRequest(QString fileName)
{
    if(!fileName.isEmpty())
    {
        yarp::os::Bottle b;
        b.addString("set");
        b.addString("input");
        b.addString(fileName.toStdString().c_str());
        outputPort.write(b);
        b.clear();
        b.addString("map");
        b.addString("train");
        outputPort.write(b);
    }
}


/*!
 * \brief Sends training request together with training date to remotehost running module.
 * \param[in] numSamples - number of samples
 * \param[in] numInputs - number of inputs
 * \param[in] trainingData - training data
 */
void SOMInterface::sendTrainingRequest(int numSamples, int numInputs, QVector<double> trainingData)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("input");
    b.addInt(numSamples);
    b.addInt(numInputs);
    for(int i=0;i<trainingData.size();i++)
    {
        b.addDouble(trainingData.at(i));
    }
    outputPort.write(b);
    b.clear();
    b.addString("map");
    b.addString("train");
    outputPort.write(b);
}

/*!
 * \brief Sends request to save self-organising map.
 * \param[in] fileName - name of the training file
 */
void SOMInterface::sendSaveRequest(QString fileName)
{
    if(!fileName.isEmpty())
    {
        yarp::os::Bottle b;
        b.addString("map");
        b.addString("save");
        b.addString(fileName.toStdString().c_str());
        outputPort.write(b);
    }
}
