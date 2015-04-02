// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Vishwanathan Mohan
  * email: Vishwanathan Mohan@iit.it
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

/**
 * @file tutorialThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _EPISODIC_MEM_THREAD_H_
#define _EPISODIC_MEM_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>

#define COMMAND_VOCAB_REQ          VOCAB3('R','E','Q')
#define COMMAND_VOCAB_ACK          VOCAB3('A','C','K')


class EpisodicMemThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
        
   
  	yarp::os::BufferedPort<yarp::os::Bottle > WorldSnap;     // output port to command the robot
	yarp::os::BufferedPort<yarp::os::Bottle > PlanF;      // input port to receive 3d information
	yarp::os::Port AckObs; 
    yarp::os::RpcServer ObserverResponse;  //acknowleges snapshot and gives strategy
	yarp::os::Semaphore mutex;
	yarp::os::BufferedPort<yarp::os::Bottle >  RememberedMemories;
	yarp::os::BufferedPort<yarp::os::Bottle >  PlanorXplore;
	yarp::os::RpcClient HubBottomup;
	yarp::os::BufferedPort<yarp::os::Bottle >  UsefulPastXperiences;
	yarp::os::BufferedPort<yarp::os::Bottle >  HumTopDownCompete;
	
	std::string name;           // rootname of all the ports opened by this thread

		int Episodes[25][1000];
		int NumEpi;
		int **data; 
		int **WhubEp;
		int N;
		double Uini[1000],SunhiDiff;
		int VnetAct[1000];
		int WHub2Epim[1000][42];
		int WHub2EpimT[42][1000];
		int WAct2Epim[1000][9];
		int OHub[42];
		int MemID[6], OLap[6], NovelObID[6], NNovobs;
		int VSSP[42], SumVSSP;
		int Action[9];
		int IndexM[5],PEnd;
		int NRelPast[5][1000],NRelEp,noverl;
		int NowObAcSeq[5][1000];
		double HubTopDown[5][42],SumTopDown;
		int PlanPastExp[1000];
		int NoB,Largeness;
		bool idle;
   
public:
    /**
    * constructor default
    */
    EpisodicMemThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    EpisodicMemThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~EpisodicMemThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /*
    * function that sets the inputPort name
    */
//    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
  //  void setModelPath(std::string inp) { modelPath = inp; };

    	void MemControl();

        int TDMemCompHub(int Nrelev);

		int FindOverlap(int NW);

		void InitializeAM();

		void DumpAM();

		void MemMaint();

		int RememberPast();

		void RetrievalFromCue(int Nrelpos);

		int Random_Zel(int lim);

		void RipRealSequence(int nme);

		void PlanFromPastXP(int NWine, int Noverlapp);

		void MemComb(int NWiner);

		void XploreCombact(int Ola);
};

#endif  //_EPISODIC_MEM_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

