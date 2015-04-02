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

#include <omp.h>
#include <assert.h>
#include "era.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

/*!
 * \brief Constructor.
 * \param[in] pPort - pointer to port
 */
PortListener::PortListener(Port *pPort, int portID)
{
    port = pPort;
    id = portID;
}

/*!
 * \brief Thread loop listening for incomming messages.
 */
void PortListener::run()
{
    running = true;
    while(running)
    {
        if(port->read(command))
        {
            if(command.get(0).asString()=="message")
            {
                emit messageSet(QString("som(") + QString::number(id) + QString("): ") + QString(command.get(1).asString().c_str()));
            }
            else if(command.get(0).asString()=="set")
            {
                if(command.get(1).asString()=="winner")
                {
                    int winner = command.get(2).asInt();
                    emit poolUnitActivationRequested("fovia", winner);
                }
            }
        }
    }
    qDebug("closing port listening on %s", port->getName().c_str());
}

/*!
 * \brief Stops thread loop.
 */
void PortListener::stop()
{
    running = false;
}

/*!
 * \brief Constructor.
 * \note This constructor saves a pointer to the Interface object,
 * \note looks for GPU devices and sets default execution mode.
 * \param[in] pInterface - pointer to Interface object
 */
ERA::ERA()
{
    gpu = new GPU();
    initialised = false;
    numSOMs = 0;
}

/*!
 * \brief Opens image ports.
 */
bool ERA::openPorts(QString portPrefix)
{
    //assign port names
    leftCam.inputPortName = portPrefix + QString("/cam/left:i");
    leftCam.outputPortName = portPrefix + QString("/cam/left:o");

    //open ports
    return (leftCam.inputPort.open(leftCam.inputPortName.toStdString().c_str()) &&  leftCam.outputPort.open(leftCam.outputPortName.toStdString().c_str()));
}

bool ERA::spawnSOMs(QString portPrefix)
{
    //get port name of the local yarprun server
    hostName = QHostInfo::localHostName();
    hostServerName = QString("/aquila/server/") + hostName;
    server.append(hostServerName);
    qDebug("- local aquila server name: %s", server.at(0).toStdString().c_str());

    //detect availabe IDs
    findFreeModuleIDs("/som/");

    QString program = "yarprun";
    for(int i=0; i<numSOMs; i++)
    {
        //launch self-organising map
        QStringList arguments = QStringList()<<"--on"<<server.at(0).toStdString().c_str()<<"--as"<<QString(QString("som_") + QString::number(moduleID.at(i))).toStdString().c_str()
                                             <<"--cmd"<<QString(QString("som --id ") + QString::number(moduleID.at(i))).toStdString().c_str();
        process = new QProcess(this);
        process->start(program, arguments);
        process->waitForStarted();

        //open ports used for communicating with self-organising maps
        QString tmp;
        tmp.append(portPrefix.toStdString().c_str());
        tmp.append("/som/");
        tmp.append(QString(QString::number(moduleID.at(i)) + QString("/:i")));
        somInputPortName.push_back(tmp);

        tmp.replace(tmp.size()-1, 1, "o");
        somOutputPortName.push_back(tmp);

        //open the ports
        Port *tmpPort = new Port();
        tmpPort->open(somInputPortName.at(i).toStdString().c_str());
        somInputPort.push_back(tmpPort);

        //start port listening thread
        PortListener *p = new PortListener(somInputPort.at(i), moduleID.at(i));
        somPortListener.append(p);
        QObject::connect(somPortListener.at(i), SIGNAL(messageSet(QString)), this, SIGNAL(messageSet(QString)), Qt::DirectConnection);
        QObject::connect(somPortListener.at(i), SIGNAL(poolUnitActivationRequested(QString, int)), this, SLOT(activatePoolUnit(QString, int)), Qt::DirectConnection);
        QObject::connect(this, SIGNAL(quitting()), somPortListener.at(i), SLOT(stop()), Qt::DirectConnection);
        somPortListener.at(i)->start();

        tmpPort = new Port();
        tmpPort->open(somOutputPortName.at(i).toStdString().c_str());
        somOutputPort.push_back(tmpPort);
    }

    //test and connect self-organising maps
    for(int i=0; i<numSOMs; i++)
    {
        int timeout = 0;
        bool success = false;
        QString portToCheck = QString("/") + QString("som/") + QString::number(moduleID.at(i)) + QString(":i");
        while(!success)
        {
            qDebug(" - era: probing som module at %s (trial %i out of %i)", portToCheck.toStdString().c_str(), timeout, MAX_TIMEOUT_ATTEMPTS);

            yarp::os::Time::delay(0.1);
            if(!success)
            {
                if(yarp::os::Network::exists(portToCheck.toStdString().c_str(), true))
                {
                    //connect to som module
                    connectSOM(i);

                    qDebug(" - era: connection with som module establised");
                    success = true;
                }
                else
                {
                    success = false;
                }
            }

            timeout++;
            if(timeout==MAX_TIMEOUT_ATTEMPTS)
            {
                qWarning(" - era: som module was not detected");
                break;
            }
        }
    }

	return true;
}

/*!
 * \brief Connects to a SOM module with a specified instance ID.
 * \param[in] somInstanceID - module instance ID
 */
void ERA::connectSOM(int somInstanceID)
{
    Network::connect(QString(QString("/som/") + QString::number(moduleID.at(somInstanceID)) + QString(":o")).toStdString().c_str(), somInputPortName.at(somInstanceID).toStdString().c_str());
    Network::connect(somOutputPortName.at(somInstanceID).toStdString().c_str(), QString(QString("/som/") + QString::number(moduleID.at(somInstanceID)) + QString(":i")).toStdString().c_str());
}

/*!
 * \brief Disconnects from a SOM module with a specified instance ID.
 * \param[in] somInstanceID - module instance ID
 */
void ERA::disconnectSOM(int somInstanceID)
{
    Network::disconnect(QString(QString("/som/") + QString::number(moduleID.at(somInstanceID)) + QString(":o")).toStdString().c_str(), somInputPortName.at(somInstanceID).toStdString().c_str());
    Network::disconnect(somOutputPortName.at(somInstanceID).toStdString().c_str(), QString(QString("/som/") + QString::number(moduleID.at(somInstanceID)) + QString(":i")).toStdString().c_str());
}

/*!
 * \brief Connects ports.
 */
void ERA::connectPorts()
{
    if(getSimulationMode())
    {
        Network::connect("/icubSim/cam/left", leftCam.inputPortName.toStdString().c_str());
    }
    else
    {
        Network::connect("/icub/cam/left", leftCam.inputPortName.toStdString().c_str());
    }
}

/*!
 * \brief Disconnects ports.
 */
void ERA::disconnectPorts()
{
    if(getSimulationMode())
    {
        Network::disconnect("/icubSim/cam/left", leftCam.inputPortName.toStdString().c_str());
    }
    else
    {
        Network::disconnect("/icub/cam/left", leftCam.inputPortName.toStdString().c_str());
    }
}

/*!
 * \brief Finds available module IDs.
 * \param[in] portPrefix - module port prefix
 */
void ERA::findFreeModuleIDs(QString portPrefix)
{
    int id = 100; //initial ID of child module
    QString portToCheck;

    //find available id
    while(id < (100+MAX_ENGINE_INSTANCES))
    {
        portToCheck = portPrefix+QString::number(id)+QString(":o");

        if(yarp::os::Network::exists(portToCheck.toStdString().c_str(), true)==false)
        {
            moduleID.push_back(id);
        }
        id++;
    }

}

/*!
 * \brief Sets up a SOM.
 * @param[in] somId - int - the id number of the som to transmit to
 * @param[in] numInputs - int - how many inputs do you want
 * @param[in] size - int - how many floats there are in data
 */
void ERA::setSomParameters(int somId, int numInputs, int numOutputs, bool gpu)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("numInputs");
    b.addInt(numInputs);
    somOutputPort.at(somId)->write(b);

    b.clear();
    b.addString("set");
    b.addString("numOutputs");
    b.addInt(numOutputs);
    somOutputPort.at(somId)->write(b);

    b.clear();
    b.addString("set");
    if(gpu) b.addString("gpu");
    else b.addString("cpu");
    somOutputPort.at(somId)->write(b);

    b.clear();
    b.addString("set");
    b.addString("numSubIterations");
    b.addInt(300);
    somOutputPort.at(somId)->write(b);

    b.clear();
    b.addString("set");
    b.addString("learningRate");
    b.addDouble(1.0);
    somOutputPort.at(somId)->write(b);
}

/*!
 * \brief Transmits data to a SOM.
 * @param[in] somId - int - the id number of the som to transmit to
 * @param[in] dataToSend - float * pointer to the data to be transmitted
 * @param[in] size - int - how many floats there are in data
 */
void ERA::sendToSom(int somId, float *dataToSend, int size, int learningOn)
{
    yarp::os::Bottle b;
    b.addString("set");
    b.addString("realTimeInput");
    b.addInt(size);
    b.addString("learningOn");
    b.addInt(learningOn);
    b.addString("data:");
    for(int i=0; i<size; i++)
    {
        b.addDouble(dataToSend[i]);
    }
    somOutputPort.at(somId)->write(b);
}

/*!
 * \brief Sets initial parameter values.
 */
void ERA::init()
{
    //set up SOM's
    setSomParameters(0, 36, 100, 0);//CPU);

    //partially pre-train SOM's with random but appropriate data
    //this is to ensure coverage of the space by the SOM, but learning can continue!!!
    partiallyPreTrainSom(0, 36, 2700);
    //partiallyPreTrainSom(1, 36, 2700);

    //initialize IAC equation parameters
    internalBias    = 0.1f;      //bias for internal activity 0-1 (typically .1)
    externalBias    = 1.0f;      //bias for external activity 0-1 (typically 1.0)
    iacMax          = 1.0f;      //the maximum unit activity (typically 1)
    iacMin          = -0.2f;     //the minimum unit activity (typically -0.2)
    decay           = 0.5f;      //the rate of decay of activity relative to the distance from rest (typically 0.5)
    rest            = -0.01f;    //the resting level of activity for any unit (typically -0.01)
    landa           = 0.05f;     //the IAC learning rate
    inhibition      = -0.8f;     //the level of constant inhibition within each pool (typically -0.8)
    learningScalar  = 0.0001f;   //used to subdue and amplify learning with events such as speech input

    //set up some pools
    setupPools();

    //intialize IAC connection matrix
    setupConnectionMatrix(2, 3);
    //printf("\nTESTING connection matrix: map %d and map %d connect, with weight between %d and %d at value %f\n", connectionMatrix.at(0).connectMaps[0], connectionMatrix.at(0).connectMaps[1], connectionMatrix.at(0).weights.at(0).i, connectionMatrix.at(0).weights.at(0).j, connectionMatrix.at(0).weights.at(0).weight);

    //mark the initialization process done
    initialised = true;
}

/*!
 * \brief Thread loop.
 */
void ERA::run()
{
    int X,Y;
    running = true;

    //let Aquila know the era started
    emit started(1);

    //set initial parameter values
    if(!initialised) init();

    //declare yarp images
    ImageOf<PixelRgb> *leftImage = NULL;

    //now enter the main running loop
    while(running)
    {
        //read the left image from the cam ports
        leftImage = leftCam.inputPort.read(false);

		if(leftImage!=NULL)
		{
			//*******
			//run SOM
			//*******

			for(int objectNo=0; objectNo<objects.size(); objectNo++)
			{

				X = objects.at(objectNo).X;
				Y = objects.at(objectNo).Y;
				if(X > 0 && X < leftImage->width() && Y > 0 && Y < leftImage->height())
				{
					//run SOM on the object
					getFovia(leftImage, X, Y, 5);
					sendToSom(0, colourSpectrum, 36, 0);
				}
			}
			objects.resize(0);

			//run SOM on centre of image
			//getFovia(leftImage, leftImage->width()/2, leftImage->height()/2, 5);
			//intrfc->sendToSom(0, colourSpectrum, 36, 0);

			//run SOM on test object
			//getFovia(leftImage, (leftImage->width()/2)+40, leftImage->height()/2, 5);
			//intrfc->sendToSom(0, colourSpectrum, 36, 0);

			//getBody();

			//field inputs are automatically updated as field input streams in through yarp ports and is managed in interface.cpp with calls to era.cpp functions

			//***************
			//run IAC network
			//***************
			iacStep();

			//*******************
			//work out what to do
			//*******************
			//say what you see
			speechOutput();

			//graph the output
			sendData();

			//display the modified image
			if(leftImage!=NULL)
			{
				//draw on image
				//markImage(leftImage, 100, 100, 0, 0, 255);

				//draw output
				leftCam.outputPort.prepare().copy(*leftImage);
				leftCam.outputPort.write();
			}
		}
    }

    //let Aquila know the era stopped
    emit stopped(2);
}

/*!
 * \brief Stops the thread.
 */
void ERA::stop()
{
    running  = false;
}

/*!
 * \brief defines some pools and sizes
 * \note this should be automated rather hand designed as it currently is
 */
void ERA::setupPools()
{
    emit messageSet(" Setting up pools");
    //define a pool
    PoolType newPool;
    PoolType newPool2;
    PoolActivityType state;
    PoolActivityType state2;

    //populate the pools
    //newPool.size = 100;
    newPool.kind = "som";
    newPool.input = "fovia";
    for(int i=0; i<100; i++)
    {
        state.activity = 0.0;
        state.netInput = 0.0;
        state.extInput = 0.0;
        newPool.state.push_back(state);
    }
    pool.push_back(newPool);
    pool.push_back(newPool);
    emit messageSet(" ..SOM pools set up..");

    //newPool2.size = 100;
    newPool2.kind = "field";
    newPool2.input = "speech";
    for(int i=0; i<100; i++)
    {
        state2.activity = 0.0;
        state2.netInput = 0.0;
        state2.extInput = 0.0;
        newPool2.state.push_back(state2);
    }
    pool.push_back(newPool2);
    emit messageSet(" speech pool set up");
}

/*!
 * \brief Sets up the connection strucutre of the IAC network, i.e. which som's and fields connect to each other If this function is called several times with different hubs, then mutliple hubs will exist in resulting the strucutre
 * \var int hub - tell this unit which index is a hub and it will connect all other fields and soms to that hub
 * \var int maxIdx - how many som's / fields are there
 * \note this should be imporved to allow a more flexible structure but for now it will do...
 */
void ERA::setupConnectionMatrix(int hub, int maxIdx)
{
    emit messageSet(" setting up connection matrix");

    //for now I am connecting all pools to the hub. find a more dynamic solution later...
    for(int idx=0; idx<maxIdx; idx++)
    {
        if(idx != hub) //dont connect the hub to itself!!!
        {
            //define the connection
            ConnectionMatrixType connection;
            connection.connectMaps[0] = hub;
            connection.connectMaps[1] = idx;

            //populate the weights matrix
            //printf("connecting - %d to %d, thats %d x %d = %d connections\n", hub, idx, poolSize.at(idx).size, poolSize.at(hub).size, poolSize.at(idx).size * poolSize.at(hub).size);
            for(int i=0; i<pool.at(idx).state.size()-1; i++)
            {
                for(int j=0; j<pool.at(hub).state.size()-1; j++)
                {
                    //define a single weight
                    WeightsType weights;
                    weights.i = i;
                    weights.j = j;
                    weights.weight = 0.0;
                    //connect the weight to the weights matrix
                    connection.weights.push_back(weights);
                }
            }
            //connect the weights matrix to the connection matrix
            connectionMatrix.push_back(connection);
        }
    }
    emit messageSet(" connection matrix generated");
}

/*!
 * \brief Cycle the IAC network 1 step: assumes external input has been assigned in the pool structure
 */
void ERA::iacStep()
{
    int poolIdxA, poolIdxB;
    int i,j;
    char message[100];
    float activity, netInput, activityA, activityB, weight;

    //calculate the within pool inhibition
    for(int poolIdx=0; poolIdx<pool.size(); poolIdx++)                                                          //for each individual pool
    {
        for(int unitIdxUpdate=0; unitIdxUpdate<pool.at(poolIdx).state.size(); unitIdxUpdate++)                  //for each unit within that pool
        {
           /* sprintf(message, " in Pool %d, unit %d has activity %f and extInput %f",
                    poolIdx,
                    unitIdxUpdate,
                    pool[poolIdx].state[unitIdxUpdate].activity,
                    pool[poolIdx].state[unitIdxUpdate].extInput);
            emit messageSet(message); */

            pool[poolIdx].state[unitIdxUpdate].netInput = 0.0;                                                  //reset its netActivity to 0
            for(int unitIdxInfluencing=0; unitIdxInfluencing<pool.at(poolIdx).state.size(); unitIdxInfluencing++) //for every other unit in that same pool
            {
                if(unitIdxUpdate != unitIdxInfluencing && pool.at(poolIdx).state.at(unitIdxInfluencing).activity > 0.0) //NOTE: inhibition is only from positively active units (this prevents occilation of inactive maps)
                {                                                                                               //and add the activity of the other units in the same pool * inhibition
                    pool[poolIdx].state[unitIdxUpdate].netInput += inhibition * pool.at(poolIdx).state.at(unitIdxInfluencing).activity;
                }
            }
        }
    }
    //calculate the spread of activity between pools, and update the weights
    for(int connected=0; connected<connectionMatrix.size(); connected++)                                        //for each pair of connected pools
    {
        poolIdxA = connectionMatrix.at(connected).connectMaps[1];
        poolIdxB = connectionMatrix.at(connected).connectMaps[0];
        for(int connection=0; connection<connectionMatrix.at(connected).weights.size(); connection++)           //for each connection between those pools
        {
            i = connectionMatrix.at(connected).weights.at(connection).i;
            j = connectionMatrix.at(connected).weights.at(connection).j;
            activityA = pool.at(poolIdxA).state.at(i).activity;
            activityB = pool.at(poolIdxB).state.at(j).activity;
            weight = connectionMatrix.at(connected).weights.at(connection).weight;
            if(activityA > 1 || activityA < -1)
            {
                sprintf(message, "!!!!! %f : this is unit %d in pool %d", activityA, i, poolIdxA);
                emit messageSet(message);
            }
            if(activityB > 1 || activityB < -1)
            {
                sprintf(message, "!!!!! %f : this is unit %d in pool %d", activityB, j, poolIdxB);
                emit messageSet(message);
            }
            //update the netInput
            pool[poolIdxA].state[i].netInput += pool.at(poolIdxB).state.at(j).activity * weight;                //sum the incoming acitivity by the weight of the connection between them
            pool[poolIdxB].state[j].netInput += pool.at(poolIdxA).state.at(i).activity * weight;
            //update the weights
            if(activityA * activityB > 0)
            {                                                                                                   //update the weights for positive correlation
                connectionMatrix[connected].weights[connection].weight += learningScalar * landa * activityA * activityB * (1 - weight);
            }
            else
            {                                                                                                   //and for negative correlation
                connectionMatrix[connected].weights[connection].weight += learningScalar * landa * activityA * activityB * (1 + weight);
            }
            //check weights, as the learning scalar can mess up the normailisation process
            if(connectionMatrix[connected].weights[connection].weight > 1.0)
            {
                connectionMatrix[connected].weights[connection].weight = 1.0;
                emit messageSet(" Huge weight encountered, consider changing the learning scalar behaviour");
            }
            if(connectionMatrix[connected].weights[connection].weight < -1.0)
            {
                connectionMatrix[connected].weights[connection].weight = -1.0;
                emit messageSet(" Huge weight encountered, consider changing the learning scalar behaviour");
            }
        }
    }
    //bias the internal and external input and finally update the activity of each node
    for(int poolIdx=0; poolIdx<pool.size(); poolIdx++)                                                          //for each individual pool
    {
        for(int unitIdxUpdate=0; unitIdxUpdate<pool.at(poolIdx).state.size(); unitIdxUpdate++)                  //for each unit within that pool
        {
            //update the internal and external bias
            pool[poolIdx].state[unitIdxUpdate].netInput *= internalBias;                                        //multiply net input by the internal bias
            pool[poolIdx].state[unitIdxUpdate].netInput += externalBias * pool.at(poolIdx).state.at(unitIdxUpdate).extInput; //add the external input (scalled by the external Bias)
            pool[poolIdx].state[unitIdxUpdate].extInput *= 0.9f; // = 0.0;                                       //reset the external input
            //update the final activity of each unit
            activity = pool.at(poolIdx).state.at(unitIdxUpdate).activity;
            netInput = pool.at(poolIdx).state.at(unitIdxUpdate).netInput;
            if(netInput > 0)
            {                                                                                                   //positive update rule
                pool[poolIdx].state[unitIdxUpdate].activity += ((iacMax - activity) * netInput) - (decay * (activity - rest));
            }
            else
            {                                                                                                   //negative update rule
                pool[poolIdx].state[unitIdxUpdate].activity += ((activity - iacMin) * netInput) - (decay * (activity - rest));
            }
            //check that unit activity is still correcrly bounded
            activity = pool.at(poolIdx).state.at(unitIdxUpdate).activity;
            if(activity > iacMax)
            {
                pool[poolIdx].state[unitIdxUpdate].activity = iacMax;
            }
            if(activity < iacMin)
            {
                pool[poolIdx].state[unitIdxUpdate].activity = iacMin;
            }
        }
    }
    if(learningScalar > 0.0001)
    {
        learningScalar *= 0.9f;
    }
}

/*!
 * \brief Return the index of the winning IAC unit in a particular field if that winner has activity > 0.
 */
int ERA::getIacWinner(QString inputFrom)
{
    int winnerIdx = -1;
    float winnerActivity = -1.0;

    for(int poolIdx=0; poolIdx<pool.size(); poolIdx++)
    {
        if(pool.at(poolIdx).input == inputFrom)
        {
            for(int i=0; i<pool.at(poolIdx).state.size()-1; i++)
            {
                if(pool.at(poolIdx).state.at(i).activity > 0 && pool.at(poolIdx).state.at(i).activity > winnerActivity)
                {
                    winnerActivity = pool.at(poolIdx).state.at(i).activity;
                    winnerIdx = i;
                }
            }
        }
    }
    return winnerIdx;
}

/*!
 * \brief Capture the foviated region and store it as an HSV specturm.
 */
void ERA::getFovia(ImageOf<PixelRgb> *image, int xCenter, int yCenter, int foviaSize)
{
    //int xCenter = image->width() /2;
    //int yCenter = image->height() /2;
    int HSV_bin = 0;
    int totalPixels = 0;
    float h,s,v;    

    for(int i=0; i<36; i++)
    {
        colourSpectrum[i] = 0.0; //the HSV profile in 10 degree bins
    }

    //put each pixel colour in the appropriate colour bin
    for(int x=xCenter - foviaSize; x<xCenter + foviaSize + 1; x++)
    {
        for(int y=yCenter - foviaSize; y<yCenter + foviaSize + 1; y++)
        {
            if(image->isPixel(x,y))
            {
                //get a pixel
                PixelRgb& pixel = image->pixel(x,y);

                //convert each pixel from RGB to HSV
                RGBtoHSV((float)pixel.r/255, (float)pixel.g/255, (float)pixel.b/255, &h, &s, &v);
                //printf("hsv : %f %f %f\n",h,s,v);

                //increment the colourSpectrum of the appropriate colour bin
                if(s > 0)
                {
                    HSV_bin = (int)h/10;            //if s == 0, then h = -1 (undefined)
                    colourSpectrum[HSV_bin] ++;
                    totalPixels ++;
                }

                //mark the fovia on the image
                if(x == xCenter - foviaSize || x == xCenter + foviaSize || y == yCenter - foviaSize || y == yCenter + foviaSize)
                {
                    pixel.r = 255 - pixel.r;
                    pixel.g = 255 - pixel.g;
                    pixel.b = 255 - pixel.b;
                }
            }
        }
    }

    //normalise the colourSpectrum
    if(totalPixels > 0)
    {
        for(int i=0; i<36; i++)
        {
            colourSpectrum[i] /= totalPixels;
        }
    }
}

/**
*@brief Converts RGB to HSV.
*@param[in] r - colours in the range 0-1
*@param[in] g
*@param[in] b
*@param[in] *h - pointers to the HSV values h = [0,360], s = [0,1], v = [0,1]
*@param[in] *s
*@param[in] *v
*/
void ERA::RGBtoHSV(float r, float g, float b, float *h, float *s, float *v)
{
    // r,g,b values are from 0 to 1
    // h = [0,360], s = [0,1], v = [0,1]
    //		if s == 0, then h = -1 (undefined)

    float min, max, delta;

    min = std::min( r, std::min( g, b ));
    max = std::max( r, std::max( g, b ));
    *v = max;				// v
    delta = max - min;
    if( max != 0 )
        *s = delta / max;		// s
    else {
        // r = g = b = 0		// s = 0, v is undefined
        *s = 0;
        *h = -1;
        return;
    }

    if( r == max )      *h = ( g - b ) / delta;		// between yellow & magenta
    else if( g == max )	*h = 2 + ( b - r ) / delta;	// between cyan & yellow
    else                *h = 4 + ( r - g ) / delta;	// between magenta & cyan

    *h *= 60;				// degrees
    if( *h < 0 )        *h += 360;
    //printf("hsv : %f %f %f\n",*h,*s,*v);
}

/*!
 * \brief Draw a cross on an image at target location and of colour.
 */
void ERA::markImage(ImageOf<PixelRgb> *image, int target_x, int target_y, int r, int g, int b)
{
    //draw an X at the target location in the original image
    int y1 = target_y - 5;
    int y2 = target_y + 5;
    for(int x=target_x - 5; x<target_x + 6; x++)
    {
        if(image->isPixel(x,y1))
        {
            PixelRgb& pixel = image->pixel(x,y1);
            pixel.r = r;
            pixel.g = g;
            pixel.b = b;
        }
        y1++;
        if(image->isPixel(x,y2))
        {
            PixelRgb& pixel = image->pixel(x,y2);
            pixel.r = r;
            pixel.g = g;
            pixel.b = b;
        }
        y2--;
    }
}

/*!
 * \brief Pre-trains the soms with random input to ensure a spread of the SOM.
 */
void ERA::partiallyPreTrainSom(int somId, int inputSize, int itterations)
{
    float sum = 0;
    float * fakeInput;
    fakeInput = (float*)malloc(inputSize * sizeof(float));

    emit messageSet("ERA - partial pre-training of SOM");

    for(int i=0; i<itterations; i++)
    {
        /*//set a random input
        for(int j=0; j<inputSize; j++)
        {
            fakeInput[j] = (float)rand();
            sum += fakeInput[j];
        }

        //normalise the input
        for(int j=0; j<inputSize; j++)
        {
            fakeInput[j] /= sum;
        }*/

        //set a single input to be on
        for(int j=0; j<inputSize; j++)
        {
            fakeInput[j] = 0.0;
        }
        fakeInput[rand()%inputSize] = 1.0;

        //send the random input
        sendToSom(somId, fakeInput, inputSize, 1);
    }
}

/*!
 * \brief Gets speech input and finds words either in the dictionary or adds them to the dictionary.
 */
void ERA::setSpeechInput(const Bottle& command)
{
    bool found = false;

    for(int i=2; i<command.size(); i++)
    {
        //read the word
        QString word = command.get(i).toString().c_str();

        //find if the word exists
        for(int i=0; i<dictionary.size(); i++)
        {
            if(dictionary.at(i).contains(word))
            {
                qDebug(" -----> %s found at %d", word.toStdString().c_str(), i);
                found = true;
                //activate the winning unit in the word field
                activatePoolUnit("speech", i);
                learningScalar = 2.0;  //boost IAC learning
                i = dictionary.size();  //to finish this loop without breaking the wider loop
            }
        }

        if(!found)
        {
            //add the new word
            dictionary.push_back(word);
            //activate the winning unit in the word field
            activatePoolUnit("speech", dictionary.size()-1);
            qDebug(" -----> %s", word.toStdString().c_str());
        }
    }
}

/*!
 * \brief Gets field input and finds words either in the dictionary or adds them to the dictionary.
 */
void ERA::setFieldInput(const Bottle& command)
{
    int poolIdx;
    int hub = 2;
    char message[100];
    bool foundPool = false;
    bool foundItem = false;

    //learningScalar = 2.0;  //boost IAC learning

    //read the pool name
    QString poolName = command.get(1).toString().c_str();

    //find the correct field
    for(int poolNo=0; poolNo<pool.size(); poolNo++)
    {
        if(pool.at(poolNo).input == poolName)
        {
            poolIdx = poolNo;
            poolNo = pool.size(); //end the loop
            foundPool = true;
        }
    }

    //if the pool name is not found then genearte a new pool!!!
    if(!foundPool)
    {
        //generate new pool
        sprintf(message, " Generating a new pool No.%d!!!", pool.size());
        emit messageSet(message);

        //define a pool
        PoolType newPool;
        //PoolActivityType state;

        //populate the pool
        //newPool.size = 100;
        newPool.kind = "field";
        newPool.input = poolName;
        //newPool.state.push_back(state);
        pool.push_back(newPool);
        emit messageSet(" Populated the pool");
        poolIdx = pool.size()-1;

        //Connect the new pool to the hub
        //define the connection
        ConnectionMatrixType connection;
        connection.connectMaps[0] = hub;
        connection.connectMaps[1] = poolIdx;
        connectionMatrix.push_back(connection);

        /*sprintf(message, " Connection added between maps %d to %d",
                  connectionMatrix.at(connectionMatrix.size()-1).connectMaps[0],
                  connectionMatrix.at(connectionMatrix.size()-1).connectMaps[1]);
        emit messageSet(message);*/
        //NOTE: there are no units in the new pool just yet so don't make the weights here
    }

    for(int i=2; i<command.size(); i+=3)    //( +=3 because we are expecting: label X Y label2 X2 Y2 label3 X3 Y3... etc.
    {
        //read the label
        QString label = command.get(i).toString().c_str();
        foundItem = false;

        //find if the label exists
        for(int unitIdx=0; unitIdx<pool.at(poolIdx).state.size(); unitIdx++)
        {
            if(pool.at(poolIdx).state.at(unitIdx).label == label)
            {
                //sprintf(message, " External input to pool %d, unit %d", poolIdx, unitIdx);
                //emit messageSet(message);

                //activate the unit
                pool[poolIdx].state[unitIdx].extInput = 1.0;

                //record its position
                pool[poolIdx].state[unitIdx].X = command.get(i+1).asInt();
                pool[poolIdx].state[unitIdx].Y = command.get(i+2).asInt();

                //mark found and exit loop
                foundItem = true;
                unitIdx = pool.at(poolIdx).state.size(); //exit loop
            }
        }

        //if the item is not found then create it
        if(!foundItem)
        {
            emit messageSet(" Generating a new unit!!!");

            //creat new unit
            PoolActivityType state;
            state.activity = 0.0;
            state.netInput = 0.0;
            state.extInput = 1.0;
            state.label = label;
            state.X = command.get(i+1).asInt();
            state.Y = command.get(i+2).asInt();
            pool[poolIdx].state.push_back(state);
            emit messageSet(" New unit added to population");

            sprintf(message, " New unit %d in Pool %d has %f %f %f %s %d %d",
                    pool[poolIdx].state.size()-1,
                    poolIdx,
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).activity,
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).netInput,
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).extInput,
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).label.toStdString().c_str(),
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).X,
                    pool[poolIdx].state.at(pool[poolIdx].state.size()-1).Y);
            emit messageSet(message);

            //add the appropriate connections
            //first find every connection matrix which includes the current poolIdx
            for(int i=0; i<connectionMatrix.size(); i++)
            {
                if(connectionMatrix.at(i).connectMaps[0] == poolIdx || connectionMatrix.at(i).connectMaps[1] == poolIdx)
                {
                    emit messageSet(" Connecting new unit to the hub");

                    //for each unit in the hub add a new weight to the new unit
                    for(int j=0; j<pool.at(hub).state.size()-1; j++)
                    {
                        //define a single weight
                        WeightsType weights;
                        weights.i = pool.at(poolIdx).state.size()-1;
                        weights.j = j;
                        weights.weight = 0.0;

                        //add the weight to the weights matrix
                        connectionMatrix[i].weights.push_back(weights);

                        /*sprintf(message, " Connection added between maps %d and %d, i = %d, j = %d, weight = %f ",
                                             connectionMatrix.at(i).connectMaps[0],
                                             connectionMatrix.at(i).connectMaps[1],
                                             connectionMatrix.at(i).weights.at(connectionMatrix.at(i).weights.size()-1).i,
                                             connectionMatrix.at(i).weights.at(connectionMatrix.at(i).weights.size()-1).j,
                                             connectionMatrix.at(i).weights.at(connectionMatrix.at(i).weights.size()-1).weight);
                        emit messageSet(message);*/
                    }                    
                }
            }
            emit messageSet(" all new connections added");
        }

        //tell ERA there is a new object at location X Y (foviate on it)
        ObjectType object;
        object.X = command.get(i+1).asInt();
        object.Y = command.get(i+2).asInt();
        objects.push_back(object);
    }
}

/*!
 * \brief Say's what it can see.
 */
void ERA::speechOutput()
{
    double threshold = 0.0;
    char message[100];
    for(int i=0; i<pool.size(); i++) //find the "speech" pool
    {
        if(pool.at(i).input == "speech")
        {
            for(int j=0; j<pool.at(i).state.size(); j++)
            {
                if(pool.at(i).state.at(j).activity > threshold)
                {
                    sprintf(message, "%s: %f", dictionary.at(j).toStdString().c_str(), pool.at(i).state.at(j).activity);//TO DO//
                    emit messageSet(message);
                }
            }
        }
    }
}

/*!
 * \brief Provides external input to a unit in a pool.
 */
void ERA::activatePoolUnit(QString inputFrom, int unitIdx)
{
    for(int i=0; i<pool.size(); i++)
    {
        if(pool.at(i).input == inputFrom)
        {
            pool[i].state[unitIdx].extInput = 1.0;
        }
    }
}

/*!
 * \brief Sends ERA Data.
 */
void ERA::sendData()
{
    double activity[500];
    double extInput[500];
    int n=0;

    //calculate the within pool inhibition
    for(int poolIdx=0; poolIdx<pool.size(); poolIdx++) //for each individual pool
    {
        for(int unitIdxUpdate=0; unitIdxUpdate<pool.at(poolIdx).state.size(); unitIdxUpdate++) //for each unit within that pool
        {
            if(n<500)
            {
                activity[n] = pool.at(poolIdx).state.at(unitIdxUpdate).activity;
                extInput[n] = pool.at(poolIdx).state.at(unitIdxUpdate).extInput;
                n ++;
            }
        }
    }
    emit updateGraphRequested(n, activity, extInput);
}

/*!
 * \brief Clean up.
 */
void ERA::clean()
{
    //close som modules before the ports are closed
    for(int i=0; i<somOutputPort.size(); i++)
    {
        yarp::os::Bottle b;
        b.addString("quit");
        somOutputPort.at(i)->write(b);
    }

    //close som port listeners
    emit quitting();

    //close camera ports
    leftCam.outputPort.close();
    leftCam.inputPort.close();

    //close som module ports
    for(int i=0; i<somOutputPort.size(); i++)
    {
        somInputPort.at(i)->close();
        somOutputPort.at(i)->close();
    }
}

/*!
 * \brief Sets the external input to a node in the IAC network.
 * \param[in] somID - which pool
 * \param[in] winner - which node
 */
void ERA::setWinner(int somID, int winner) //currently defunct... seems to cause a crash??? replaced by activatePoolUnit()
{
    pool[somID].state[winner].extInput = 1.0;
}

/*!
 * \brief Sets simulation mode.
 * \param[in] simulation - simulation mode on/off
 */
void ERA::setSimulationMode(bool simulation)
{
    simulationMode = simulation;
}

/*!
 * \brief Gets simulation mode.
 * \return bool simulationMode
 */
bool ERA::getSimulationMode()
{
    return simulationMode;
}
