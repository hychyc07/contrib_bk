// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file main.cpp
 * @brief main code for the early filter module; this is part of the logpolar attention system.
 */

#include <yarp/os/all.h>
#include <pmp_lib/groupThread.h>
#include <iostream>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::pmplib;

class testThread : public jointThread
{
private:
	int count;
public:
	testThread() : jointThread(){count = 0;};

	virtual inline bool runnable() {      
		cout << getName(" ") << ": " <<count << endl;
		count++;
		if(count == 20) {
			printf("%s computation done \n", getName(" ").c_str());
			Time::delay(0.5);
			count = 0;
			return false;
		}
		Time::delay(0.2);
		return true;
    };
};

int main(int argc, char * argv[])
{
    
    yarp::os::Network  yarp;
	Time::turboBoost();
	
	testThread *a , *b , *c;

    std::string name;
    name = "threadA";
	a = new testThread();
    a->setName(name);
    
    name = "threadB";
	b = new testThread();
    b->setName(name);
    
    name = "threadC";
	c = new testThread();
    c->setName(name);


	groupThread g;
	g.add_thread(a);
	g.add_thread(b);
	g.add_thread(c);
	cout << "thread added" << endl;
    
    yarp::os::Time::delay(3.0);
    g.activate();
	g.join_all();

	yarp::os::Time::delay(1.0);
	/*g.remove_thread(a);
	cout << "Removed thread: " << a->getName(" ").c_str() << endl;
	g.remove_thread(b);
	*/
	cout << "size is: " << g.size() << endl;

	g.interrupt_all();


    //while(true) {}
    return 0;
}

