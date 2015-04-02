/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zenon.mathews@upf.edu   *
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

#include <stdlib.h>
#include <unistd.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>

using namespace yarp::os;



int main(int argc, char **argv)

{

    Network yarp;

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("or.ini");
    rf.setDefaultContext("traza/orBottle");
    rf.configure("ICUB_ROOT", argc, argv);
        
    ConstString robotName=rf.find("robot").asString();
    ConstString model=rf.findFile("model");
    
    cout<<"Running with:"<<endl;
    cout<<"robot: "<<robotName.c_str()<<endl;
    
    if (model=="")
    {
        cout<<"Sorry no model was found, check config parameters"<<endl;
       // return -1;
    }

    cout << "Using object model: " << model.c_str() << endl;
	

	

    if (!yarp.checkNetwork())
    {
        printf("No yarp network, quitting\n");
        return false;
    }

    	
	FuserThread*  fuserThread = new FuserThread(10);
	cout << "----------------------> going to call fuserthread..." << endl;
	fuserThread->start();
	//fuserThread->wait();
	
	while(true)
	  {
	    //if ((Time::now()-startTime)>5)
	      //done=true;
	  }


	cout << "main.cpp...returning 0" << endl;
    return 0;


}

