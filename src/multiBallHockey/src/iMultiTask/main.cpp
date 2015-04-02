/***************************************************************************
 *   Copyright (C) 2011 by Zenon Mathews   *
 *   zenon.mathews@gmail.edu   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "fuserthread.h"
#include "headThread.h"
#include "taskThread.h"

#include "rightHandThread.h"

#include <stdlib.h>
#include <unistd.h>

#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

using namespace std;



using namespace yarp::os;

#define DOGAZE
//#define DOARMS



class CtrlModule: public RFModule
{
protected:
  //CtrlThread *thr;
   vector<TaskThread*> taskVec;	// Achtung: the first entry into the vector must be a HeadThread

public:
    virtual bool configure(ResourceFinder &rf)
    {
      
        Time::turboBoost();
	

        //thr=new CtrlThread(CTRL_THREAD_PER);
	IGazeControl  *igaze;

	// now initialize all the task threads into a vector
	std::cout << "starting head.." << std::endl;
	HeadThread* head = new HeadThread(20);
	std::cout << "starting hand.." << std::endl;
	RightHandThread* rightHand = new RightHandThread(20);
	
	taskVec.push_back(head);
	taskVec.push_back(rightHand);

    	// this does the memory
	std::cout << "starting fuser.." << std::endl;
    	FuserThread*  fuserThread = new FuserThread(20, taskVec);
	//std::cout << "fuser started.." << std::endl;
        if ((!fuserThread->start()) || (!head->start()) || (!rightHand->start()))
        {
            delete fuserThread;
	    delete head;
	    delete rightHand;
            return false;
        }
	std::cout << "all started.." << std::endl;
        return true;
    }

    virtual bool close()
    {
      
      for(int i = 0; i < taskVec.size(); i++){
        delete (taskVec.at(i));
      }

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};




int main(int argc, char **argv)

{
  
   
    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);


    return mod.runModule(rf);
}



	
