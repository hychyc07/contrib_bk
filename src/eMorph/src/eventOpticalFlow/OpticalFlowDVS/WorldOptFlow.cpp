/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


#include "WorldOptFlow.h"

WorldOptFlow::WorldOptFlow(AERGrabber * inPortPtr,
                     BufferedPort<VelocityBuffer> * outFlowPort,  BufferedPort<Bottle> * outBottlePort,
                     MyMatrix<POLARITY_TYPE> * wStatus, MyMatrix<POLARITY_TYPE> * pWStatus,
                     MyMatrix<TIMESTAMP_TYPE> * ts, yarp::os::Semaphore * eventsSignal)
                       : RateThread(1), localFlw(LUCAS_KANADE_NGHBR) {
    inPort = inPortPtr;
    worldStatus = wStatus;
    prevWorldStatus = pWStatus;
    timestamps = ts;
    localFlw.setGuaWeights(LUCAS_KANADE_GUAS_STDDEV);
    outPort = outFlowPort;
    outBPort = outBottlePort;
    newEventsSignal = eventsSignal;

    eventBuffersSize = SPDerivative_WNDW_SZ + 1;
    bufferInitialized = 0;



}

void WorldOptFlow::initialize(int step){
    int eventNo;
    CameraEvent ** eventsBfr;
    CameraEvent * evntPtr;
    short rwIdx, clmnIdx;


    eventsBfr = inPort-> getEvents(eventNo);
    eventBuffers[step] = eventsBfr;
    eventNosBuffer[step] = eventNo;

    for (int i = 0; i < eventNo; ++i) {
        evntPtr = eventsBfr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus->operator ()(rwIdx, clmnIdx) += POLARITY_WEIGHT*evntPtr->getPolarity();
    }

}

void WorldOptFlow::cleanup(){
	CameraEvent ** eventsBfr;

	cout << "WorldFlow start cleaning" << endl;

	for (int j = 0; j < eventBuffersSize - 1; ++j) {
		eventsBfr = eventBuffers[j];
		if (eventsBfr != NULL){
		   // each element of array is a CameraEvent *
		   for (int i = 0; i < eventNosBuffer[j]; ++i) {
 			   delete *(eventsBfr + i);
		   }
		   delete [] eventsBfr;
	   }
	}


	delete [] eventBuffers;
	delete [] eventNosBuffer;
	cout << "world flow is cleaned up" << endl;
}


bool WorldOptFlow::threadInit(){

	eventBuffers = new CameraEvent**[eventBuffersSize];
	eventNosBuffer = new int [eventBuffersSize];
	for (int i = 0; i < eventBuffersSize; ++i) {
		eventBuffers[i] = NULL;
		eventNosBuffer[i] = 0;
	}

	return true;
}


void WorldOptFlow::threadRelease(){

	cout << "calling thread release ... " << endl;

	cleanup();

	cout << "end thread release" << endl;
}


void WorldOptFlow::run(){

    int eventNo;
    CameraEvent ** eventsBfr;
    CameraEvent * evntPtr;;

    if (newEventsSignal->check()){


        if (bufferInitialized < SPDerivative_WNDW_SZ){
            initialize(bufferInitialized);
            bufferInitialized++;
            return;
        }

        eventsBfr = inPort->getEvents(eventNo);
        eventBuffers[eventBuffersSize - 1] = eventsBfr;
        eventNosBuffer[eventBuffersSize - 1] = eventNo;
        //directWrldStus(eventsBfr, eventNo);

        updtWrldStus();

//        clock_t start= clock();
        calVelocities(eventsBfr, eventNo);
//        clock_t end = clock();
//        double elapsed = ( (double) (end - start) ) / CLOCKS_PER_SEC;
//        cout.precision(10);
//        cout << "time :  " << elapsed << " " << eventNo << endl;






        prevWorldStatus->updateMatrix(worldStatus);


        // release the memory for events
        eventsBfr = eventBuffers[0];
        if (eventsBfr != NULL){
            // each element of array is a CameraEvent *
            for (int i = 0; i < eventNosBuffer[0]; ++i) {
                delete *(eventsBfr + i);
            }
            delete [] eventsBfr;
        }

        // shift the eventBufffers one step
        for (int i = 1; i < eventBuffersSize; ++i) {
            eventBuffers[i-1] = eventBuffers[i];
            eventNosBuffer[i-1] = eventNosBuffer[i];
        }
    }


}



void WorldOptFlow::directWrldStus(CameraEvent ** evntBffr, int bffrSize){
    CameraEvent * evntPtr;
    short rwIdx, clmnIdx;



    for (int i = SPATIAL_MARGINE_ADDUP; i < RETINA_SIZE_R + SPATIAL_MARGINE_ADDUP; ++i) {
        for (int j = SPATIAL_MARGINE_ADDUP; j < RETINA_SIZE_C+ SPATIAL_MARGINE_ADDUP ; ++j) {
            worldStatus ->operator ()(i,j) = 160;
        }
    }

    for (int cntr = 0; cntr < bffrSize; ++cntr) {
        evntPtr = evntBffr[cntr];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus ->operator ()(rwIdx, clmnIdx)  += POLARITY_WEIGHT*evntPtr->getPolarity();
    }



}


void WorldOptFlow::updtWrldStus(){
    CameraEvent * evntPtr;
    CameraEvent ** evntBffr;
    short rwIdx, clmnIdx;

    evntBffr = eventBuffers[0];
    for (int i = 0; i < eventNosBuffer[0]; ++i) {
        evntPtr = evntBffr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus->operator ()(rwIdx, clmnIdx) -= POLARITY_WEIGHT*evntPtr->getPolarity();
    }

    evntBffr = eventBuffers[eventBuffersSize - 1];
    for (int i = 0; i < eventNosBuffer[eventBuffersSize -1]; ++i) {
        evntPtr = evntBffr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus->operator ()(rwIdx, clmnIdx) += POLARITY_WEIGHT * evntPtr->getPolarity();
    }

}


/*

void WorldFlow::updtWrldStus(CameraEvent ** evntBffr, int bffrSize){
    CameraEvent * evntPtr;
    short rwIdx, clmnIdx;
    double alpha = .6; //.8 --> 5 ms, .9 --> 10 ms,.6 --> 2ms, .36 --> 1ms
    int tmp;

    for (int i = SPATIAL_MARGINE_ADDUP; i < RETINA_SIZE_R + SPATIAL_MARGINE_ADDUP; ++i) {
        for (int j = SPATIAL_MARGINE_ADDUP; j < RETINA_SIZE_C+ SPATIAL_MARGINE_ADDUP ; ++j) {
            worldStatus ->operator ()(i,j) = alpha * worldStatus->operator ()(rwIdx, clmnIdx) ;
        }
    }

    for (int cntr = 0; cntr < bffrSize; ++cntr) {
        evntPtr = evntBffr[cntr];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();

        worldStatus ->operator ()(rwIdx, clmnIdx) += (1 - alpha)*50*evntPtr->getPolarity();

        tmp = worldStatus->operator ()(rwIdx, clmnIdx);
        if ((tmp > RELIABLE_EVENT_THRSHLD) || (tmp < -RELIABLE_EVENT_THRSHLD)){
            evntPtr->setReliable(true);
        }


    }
}
*/

void WorldOptFlow::calVelocities(CameraEvent ** evntBffr, int bffrSize){

	CameraEvent * evntPtr;
	short evtRw, evtClm;
	VelocityBuffer vlctyBuffer;
	double velocity [3];


	for (int cntr = 0; cntr < bffrSize; ++cntr) {
		*velocity = 0;
		*(velocity + 1) = 0;

		evntPtr = evntBffr[cntr];

		if (evntPtr == 0)
			continue;

		evtRw = evntPtr->getRowIdx();
		evtClm = evntPtr->getColumnIdx();

		unsigned long tsDiff;
        tsDiff = timestamps-> operator()(evntPtr->getRowIdx(), evntPtr ->getColumnIdx() ) - evntPtr->getTimeStamp();
		timestamps-> operator()(evntPtr->getRowIdx(), evntPtr ->getColumnIdx() ) = evntPtr->getTimeStamp();

		localFlw.calVelocity(tsDiff, worldStatus, prevWorldStatus,
		   					 evtRw, evtClm, velocity);


        if (*velocity != 0 || *(velocity +1) != 0 ) {

			if (vlctyBuffer.addDataCheckFull(evtClm - SPATIAL_MARGINE_ADDUP,
											 evtRw - SPATIAL_MARGINE_ADDUP,
											 *velocity, *(velocity + 1),
											 evntPtr -> getTimeStamp(), *(velocity + 2) ) ){ //TODO
				//Buffer is full and it should be sent to the network

				if (outPort -> getOutputCount()){
					VelocityBuffer & outObj = outPort->prepare();
					outObj.setData(vlctyBuffer);
					outPort->write();
					vlctyBuffer.emptyBuffer();
				}
			}



			if (outBPort -> getOutputCount()){
			    
                Bottle & b = outBPort -> prepare();
				b.clear();
				b.addInt(evtClm - SPATIAL_MARGINE_ADDUP);
				b.addInt(evtRw - SPATIAL_MARGINE_ADDUP);
				b.addDouble((*velocity) );
				b.addDouble((*(velocity + 1) )  );
				outBPort->write();
			}

		}// end if velocity != 0

	}

  	if (!vlctyBuffer.isEmpty()){
		if (outPort -> getOutputCount()){
			VelocityBuffer & outObj = outPort->prepare();
			outObj.setData(vlctyBuffer);
			outPort->write();
			vlctyBuffer.emptyBuffer();
		}
	}

}



WorldOptFlow::~WorldOptFlow(){
	cout << "world flow is closed" << endl;
}


