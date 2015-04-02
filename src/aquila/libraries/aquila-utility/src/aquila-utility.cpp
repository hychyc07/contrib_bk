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

#include <omp.h>
#include "aquila-utility.h"
#include "kernels.h"

#define gpuAssert(condition){if((condition)!=0){fprintf(stderr,"\n FAILURE %s in %s, line %d\n",cudaGetErrorString(condition),__FILE__,__LINE__ );}}

namespace aquila
{
/*!
 * \brief Constructor.
 */
GPU::GPU()
{
	//get number of GPU devices
	numDevices = -1;
    cudaGetDeviceCount(&numDevices);

    //disable GPU mode if no devices were found
    if(numDevices>0)
    {
		//set dafault device
        active = true;
        numRequestedDevices = 1;
        setDevice(0);
    }
    else
    {
        active = false;
    }
}

/*!
 * \brief Destructor.
 */
GPU::~GPU()
{
}

/*!
 * \brief Sets GPU mode.
 * \param[in] gpuActive - GPU mode active (on/off)
 */
void GPU::setGPUMode(bool gpuActive)
{
    active = gpuActive;
    emit gpuModeSet(active);
}

/*!
 * \brief Sets current GPU device.
 * \param[in] id - device id
 * \return true on success
 */
bool GPU::setDevice(int id)
{
    if(id>=0 && id<numDevices)
    {
        //set device
        cudaSetDevice(id);

        //check if was set successfully
        int tmp = -1;
        cudaGetDevice(&tmp);
        if(tmp==id)
        {
            current = id;
            if(device.isEmpty())
            {
                device.push_back(current);
            }
        }
        else
        {
            qCritical(" - aquila-utility: failed to set device %i as active, current active device %i", id, current);
            return false;
        }
    }
    else
    {
		if(numDevices>0)
		{
			qCritical(" - aquila-utility: failed to set device %i as active; %i devices available, choose id between 0 and %i", id, numDevices, numDevices-1);
			return false;
		}
    }

    return true;
}

/*!
 * \brief Sets mulitple GPU devices, enables P2P mode where possible.
 * \param[in] deviceID[] - list of IDs of GPU devices to be added
 */
void GPU::setDevices(QVector<int> deviceID)
{
    numRequestedDevices = deviceID.size();
    device = deviceID;

    //enable P2P
    if(numRequestedDevices > 1)
    {
        for(int i=0; i<numRequestedDevices; i++)
        {            
            for(int j=i+1; j<numRequestedDevices; j++)
            {
                //Enable and test P2P access, device in the first index of gpuDevice array is always master
                int accessible = 0;

                cudaDeviceCanAccessPeer(&accessible, deviceID.at(j), deviceID.at(i));
                if(accessible)
                {
                    //two GPUs and both can communicate
                    if(numRequestedDevices==2)
                    {
                        peerToPeerActive = true;
                        cudaDeviceEnablePeerAccess(deviceID.at(j), 0);
                        qDebug("device %i can now access device %i via PCIe", deviceID.at(i), deviceID.at(j));
                    }
                    else
                    {
                        peerToPeerActive = false;
                    }
                }
                else
                {
                    peerToPeerActive = false;
                    qDebug("device %i cannot access device %i via PCIe",deviceID.at(i), deviceID.at(j));
                }
            }
        }
    }

    //the first device is master, rest are slaves
    cudaSetDevice(deviceID.at(0));
}

/*!
 * \brief Returns current GPU device.
 * \return id - device id
 */
int GPU::getDevice()
{
    int id = -1;
    cudaGetDevice(&id);
    return id;
}

/*!
 * \brief Returns detected GPU devices as a list of their names.
 * \return deviceList - properties of GPU devices found on the system
 */
QVector<QStringList> GPU::getDeviceList()
{
    QVector<QStringList> deviceList;
    property.clear();

    for(int i=0; i<numDevices; i++)
    {
        //initialise variables
        char buf[1000];
        int driverVersion = 0;
        int runtimeVersion = 0;
        QStringList tmp;

        //get version driver and runtime versions
        cudaDriverGetVersion(&driverVersion);
        cudaRuntimeGetVersion(&runtimeVersion);

        //get properties of current GPU device and add its name to the list
        cudaDeviceProp devicePropery;
        cudaGetDeviceProperties(&devicePropery, i);

        //add them to the list of GPU properties
        property.append(devicePropery);

        //add GPU brief summary
        sprintf(buf, "%s (%.2f GHz, %.0f MBytes)", property.at(i).name, property.at(i).clockRate * 1e-6f, (float)property.at(i).totalGlobalMem/1048576.0f); tmp.append(buf);

        //add device properies to string list
        sprintf(buf, "Name: \t\t\t%s", property.at(i).name); tmp.append(buf);
        sprintf(buf, "CUDA Driver Version / Runtime Version:\t%d.%d / %d.%d", driverVersion/1000, (driverVersion%100)/10, runtimeVersion/1000, (runtimeVersion%100)/10); tmp.append(buf);
        sprintf(buf, "CUDA Capability version number:\t%d.%d", property.at(i).major, property.at(i).minor); tmp.append(buf);
        sprintf(buf, "Total amount of global memory:\t%.0f MBytes)", (float)property.at(i).totalGlobalMem/1048576.0f); tmp.append(buf);
        sprintf(buf, "GPU Clock Speed:\t\t%.2f GHz", property.at(i).clockRate * 1e-6f); tmp.append(buf);
        sprintf(buf, "Total amount of constant memory:\t%u bytes", (unsigned int)property.at(i).totalConstMem); tmp.append(buf);
        sprintf(buf, "Total amount of shared memory per block:\t%u bytes", (unsigned int)property.at(i).sharedMemPerBlock); tmp.append(buf);
        sprintf(buf, "Total number of registers per block:\t%d", property.at(i).regsPerBlock); tmp.append(buf);
        sprintf(buf, "Warp size:\t\t\t%d", property.at(i).warpSize); tmp.append(buf);
        sprintf(buf, "Maximum number of threads per block:\t%d", property.at(i).maxThreadsPerBlock); tmp.append(buf);
        sprintf(buf, "Maximum sizes block dimensions:\t%d x %d x %d", property.at(i).maxThreadsDim[0], property.at(i).maxThreadsDim[1], property.at(i).maxThreadsDim[2]); tmp.append(buf);
        sprintf(buf, "Maximum sizes grid dimensions:\t%d x %d x %d", property.at(i).maxGridSize[0], property.at(i).maxGridSize[1], property.at(i).maxGridSize[2]); tmp.append(buf);
        sprintf(buf, "Maximum memory pitch:\t\t%u bytes", (unsigned int)property.at(i).memPitch); tmp.append(buf);
        sprintf(buf, "Texture alignment:\t\t%u bytes", (unsigned int)property.at(i).textureAlignment); tmp.append(buf);
        sprintf(buf, "Concurrent copy and execution:\t%s with %d copy engine(s)", (property.at(i).deviceOverlap ? "Yes" : "No"), property.at(i).asyncEngineCount); tmp.append(buf);
        sprintf(buf, "Run time limit on kernels:\t\t%s", property.at(i).kernelExecTimeoutEnabled ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Integrated GPU sharing Host Memory:\t%s", property.at(i).integrated ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Support host page-locked memory mapping:\t%s", property.at(i).canMapHostMemory ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Concurrent kernel execution:\t\t%s", property.at(i).concurrentKernels ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Alignment requirement for Surfaces:\t%s", property.at(i).surfaceAlignment ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Device has ECC support enabled:\t%s", property.at(i).ECCEnabled ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Device is using TCC driver mode:\t%s", property.at(i).tccDriver ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Device supports Unified Addressing (UVA):\t%s", property.at(i).unifiedAddressing ? "Yes" : "No"); tmp.append(buf);
        sprintf(buf, "Device PCI Bus ID / PCI location ID:\t%d / %d", property.at(i).pciBusID, property.at(i).pciDeviceID ); tmp.append(buf);
        const char *sComputeMode[] = {"Default", "Exclusive", "Prohibited", "Exclusive Process", "Unknown", NULL};
        sprintf(buf, "Compute Mode:\t\t%s", sComputeMode[property.at(i).computeMode]); tmp.append(buf);

        //add current device properties to the list
        deviceList.append(tmp);
    }

    return deviceList;
}

/*!
 * \brief Constructor.
 */
Messenger::Messenger(Port &targetPort)
{
    port = targetPort;
}

/*!
 * \brief Destructor.
 */
Messenger::~Messenger()
{
}

/*!
 * \brief Sends message to the output port.
 * \note Normally, a message is generated when this module received a command from the input port,
 * \note which typically originates from Aquila. Every such message is then sent to the output port
 * \note so that Aquila can stay synchrnoised with this module.
 * \param[in] message - a message to be sent to the output port
 */
void Messenger::sendMessage(QString message)
{
    if(!message.isEmpty())
    {
        Bottle b;
        b.addString("message");
        b.addString(message.toStdString().c_str());
        port.write(b);
        qDebug("messenger: %s", message.toStdString().c_str());
    }
}

/*!
 * \brief Sends current training progress (in %) to the output port.
 * \param[in] progress - current progress
 */
void Messenger::sendProgress(int progress)
{
    Bottle b;
    b.addString("progress");
    b.addInt(progress);
    port.write(b);
}

/*!
 * \brief Sends a list of detected GPU devices to the output port.
 * \note This list is read by Aquila, which then constructs a menu listing all devices.
 * \param[in] deviceList - the list of GPU devices
 */
void Messenger::sendGpuDeviceList(QVector<QStringList> deviceList)
{
    Bottle b;
    b.addString("gpuList");

    for(int i=0; i<(int)deviceList.size(); i++)
    {
        b.addString("beginning");
        for(int j=0; j<(int)deviceList.at(i).size(); j++)
        {
            b.addString(deviceList.at(i).at(j).toStdString().c_str());
        }
        b.addString("end");
    }

    port.write(b);
}

/*!
 * \brief Sends module status to the output port.
 * \note This status determines the current status of the module (e.g. currently training, finished, etc.).
 */
void Messenger::sendStatus(int status)
{
    Bottle b;
    b.addString("status");
    b.addInt(status);
    port.write(b);
}

/*!
 * \brief Constructor.
 */
Math::Math()
{
    seed = 0;
}

/*!
 * \brief Destructor.
 */
Math::~Math()
{
}

/*!
 * \brief Scales the input between any min and max values.
 * \note Function takes the input max/min range, desired max/min range and converts the input accordingly.
 * \param[in]  in - current value
 * \param[in]  oldMin - old minimum value
 * \param[in]  oldMax - old maximum value
 * \param[in]  newMin - new minimum value
 * \param[in]  newMax - new maximum value
 * \return result - scaled value
 */
float Math::scaleRange(float in, float oldMin, float oldMax, float newMin, float newMax)
{
    float scale = (oldMax-oldMin)/(newMax-newMin);
    float result = newMin+(in-oldMin)/scale;
    return result;
}

/*!
 * \brief Finds the next power of 2 number of x.
 * \param[in] x - number of which the next power of 2 is to be found.
 * \return x - new power of two of x
 */
int Math::nextPow2(int x)
{
    --x;
    x|=x>>1;
    x|=x>>2;
    x|=x>>4;
    x|=x>>8;
    x|=x>>16;

    return ++x;
}

/*!
 * \brief Sets seed for random number generation.
 * \param[in] value - seed value
 */
void Math::setSeed(int value)
{
    seed = value;
    srand(seed);
}

/*!
 * \brief Gets random float in a specified range.
 * \param[in] min - lower limit
 * \param[in] max - upper limit
 */
float Math::getRandomFloatInRange(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}

/*!
 * \brief Gets random integer between zero and specified limit.
 * \param[in] limit - the highest integer that can be generated
 */
int Math::getRandomIntegerUpTo(int limit)
{
    return rand()%limit;
}

/*!
 * \brief Returns current seed.
 * \return seed - current seed value
 */
int Math::getSeed()
{
    return seed;
}
}
