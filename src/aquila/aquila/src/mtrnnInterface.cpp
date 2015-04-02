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

#include "mtrnnInterface.h"

/*!
 * \brief Constructor.
 * \param[in] pGUI - pointer to GUI
 */
MTRNNInterface::MTRNNInterface(GUI *pGUI) : Interface(pGUI)
{
}

/*!
 * \brief Processes data received from module.
 */
void MTRNNInterface::processPortData()
{
    if(receivedBottle.get(0).asString()=="parameters")
    {
        emit parametersReceived((float)receivedBottle.get(1).asDouble(), (float)receivedBottle.get(2).asDouble(), (float)receivedBottle.get(3).asDouble(), (float)receivedBottle.get(4).asDouble(), receivedBottle.get(5).asInt(), receivedBottle.get(6).asInt(), receivedBottle.get(7).asInt(), receivedBottle.get(8).asInt(), receivedBottle.get(9).asInt(), receivedBottle.get(10).asInt(), receivedBottle.get(11).asInt(), receivedBottle.get(12).asInt());
    }
    else if(receivedBottle.get(0).asString()=="error")
    {
        emit errorReceived(receivedBottle.get(1).asInt(), (float)receivedBottle.get(2).asDouble());
    }
    else if(receivedBottle.get(0).asString()=="errors")
    {
        processErrors();
    }
    else if(receivedBottle.get(0).asString()=="network")
    {
        processNetwork();
    }
}

/*!
 * \brief Processes neural network received from module running on remotehost.
 * \note This function emits signal with neural network that can be saved locally.
 */
void MTRNNInterface::processNetwork()
{
    QVector<float> network;
    for(int i=1; i<receivedBottle.size(); i++)
    {
        network.append((float)receivedBottle.get(i).asDouble());
    }
    emit networkReceived(network);
}

/*!
 * \brief Processes errors received from module.
 * \note This function emits signal with all the errors from the whole training.
 */
void MTRNNInterface::processErrors()
{
    QVector<float> errors(receivedBottle.size()-1);
    for(int i=1; i<receivedBottle.size(); i++)
    {
        errors[i-1] = (float)receivedBottle.get(i).asDouble();
    }
    emit errorsReceived(errors);
}

/*!
 * \brief Sends training file data to remotehost running module.
 * \param[in] numControlNeurons - number of control neurons
 * \param[in] numLinguisticNeurons - number of linguistic neurons
 * \param[in] numVisionNeurons - number of vision neurons
 * \param[in] numActionNeurons - number of action neurons
 * \param[in] numSequences - number training sequences
 * \param[in] sequenceWidth - number of elements in each step of the sequence
 * \param[in] minValue - minimum value present in the original training data
 * \param[in] maxValue - maximum value present in the original training data
 * \param[in] seqenceSteps - number of steps in each sequence
 * \param[in] seqenceData - data containing all sequences
 * \param[in] maxSequenceSteps - number of steps in the longest sequences
 * \param[in] totalSequenceSteps - total number of steps when all sequences are joint
 */
void MTRNNInterface::sendTrainingFileData(int numControlNeurons, int numLinguisticNeurons, int numVisionNeurons, int numActionNeurons, int numSequences, int sequenceWidth, float minValue, float maxValue, QVector<int> seqenceSteps, QVector<float> sequenceData, int maxSequenceSteps, int totalSequenceSteps)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("input");

    //add parameters
    b.addInt(numControlNeurons);
    b.addInt(numLinguisticNeurons);
    b.addInt(numVisionNeurons);
    b.addInt(numActionNeurons);
    b.addInt(numSequences);
    b.addInt(sequenceWidth);
    b.addDouble((double)minValue);
    b.addDouble((double)maxValue);
    b.addInt(sequenceData.size());
    b.addInt(maxSequenceSteps);
    b.addInt(totalSequenceSteps);

    //add the number of steps of each sequence
    for(int i=0; i<numSequences; i++)
    {
        b.addInt(seqenceSteps.at(i));
    }

    //add the data of all sequences
    for(int i=0; i<sequenceData.size(); i++)
    {
        b.addDouble((double)(sequenceData.at(i)));
    }

    outputPort.write(b);
}

/*!
 * \brief Sends training file data to remotehost running module.
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
 * \param[in] plotUpdateInterval - update interval of the error plot
 */
void MTRNNInterface::sendParameters(float learningRate, float momentum, float weightRange, float threshold, int iterations, int seed, int numFastNeurons, int numSlowNeurons, int ioDeltaT, int fastDeltaT, int slowDeltaT, int plotUpdateInterval)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("parameters");

    //add parameters
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
    b.addInt(plotUpdateInterval);

    outputPort.write(b);
}

/*!
 * \brief Sends training request to localhost module.
 * \param[in] fileName - name of the training file
 * \note This function is called when a localhost module is asked to load a training file.
 * \note Remotehost modules need to be sent training data via port using sendTrainingFileData function.
 */
void MTRNNInterface::sendTrainingRequest(QString fileName)
{
    if(!fileName.isEmpty())
    {
        yarp::os::Bottle b;
        b.addString("set");
        b.addString("input");

        //add path to the file
        b.addString(fileName.toStdString().c_str());
        outputPort.write(b);
        b.clear();

        //add training commands
        b.addString("network");
        b.addString("train");

        outputPort.write(b);
    }
}

/*!
 * \brief Sends training request to remotehost module.
 * \note This function is called after remotehost module was initialised with the training data using sendTrainingFileData function.
 */
void MTRNNInterface::sendTrainingRequest()
{
    yarp::os::Bottle b;
    b.addString("network");
    b.addString("train");
    outputPort.write(b);
}

/*!
 * \brief Sends request to module to save neural network.
 * \param[in] fileName - file name used for saving of neural network
 */
void MTRNNInterface::sendSaveRequest(QString fileName)
{
    if(!fileName.isEmpty())
    {
        yarp::os::Bottle b;
        b.addString("network");
        b.addString("save");
        b.addString(fileName.toStdString().c_str());
        outputPort.write(b);
    }
}

/*!
 * \brief Sends request to module to send back neural network.
 */
void MTRNNInterface::sendNetworkRequest()
{
    yarp::os::Bottle b;
    b.addString("get");
    b.addString("network");
    outputPort.write(b);
}
