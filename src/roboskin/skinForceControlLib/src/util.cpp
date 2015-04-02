/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea DelPrete
  * email: andrea.delprete@iit.it
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

#include "iCub/skinForceControl/util.h"
#include <string>
#include <map>
#include <stdarg.h>     // va_list va_arg va_end va_start
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/dev/PolyDriver.h>

using namespace iCub::skinForceControl;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Semaphore DSemaphore::mutex;
map<int, std::string> DSemaphore::threadNames;

DSemaphore::DSemaphore(): Semaphore(){}
DSemaphore::DSemaphore(const std::string& _name): Semaphore(), name(_name){}
string DSemaphore::getName(){ return name; }
void DSemaphore::setName(const string& _name){ 
    name = _name; 
}
//void DSemaphore::wait(){
//    sendMsg("wait on sem "+name);
//    Semaphore::wait();
//}
//void DSemaphore::post(){
//    sendMsg("post on sem "+name);
//    Semaphore::post();
//}
void DSemaphore::registerThread(const string& threadName){
    DSemaphore::mutex.wait();
    DSemaphore::threadNames[ACE_Thread::self()] = threadName;
    DSemaphore::mutex.post();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status iCub::skinForceControl::bottleToVector(const Bottle& b, Vector& v) throw()
{
    Status res;
    for(int i=0; i<b.size(); i++)
    {
        const Value &x = b.get(i);
        if(x.isDouble())
            v.push_back(x.asDouble());
        else if(x.isInt())
            v.push_back(x.asInt());
        else if(x.isList())
            if(x.asList()->size()!=0)
                bottleToVector(*(x.asList()), v);
        else
            res.addErrMsg("Error converting bottle to vector: "+string(x.asString().c_str())+" is not a number.");
    }
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinForceControl::addToBottle(yarp::os::Bottle &b, const yarp::sig::Vector &v) throw()
{
    for(unsigned int i=0; i<v.size(); i++)
        b.addDouble(v[i]);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinForceControl::addToBottle(Bottle& b, const vector<Vector>& v) throw()
{
    for(unsigned int i=0; i<v.size(); i++)
        for(unsigned int j=0; j<v[i].size(); j++)
            b.addDouble(v[i][j]);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::buildVector(size_t s, const double v1, const double v2, ...)
{
    Vector res(s);
    if(s<2)
        return res;
    res[0] = v1;
    res[1] = v2;
    va_list listPointer;
    va_start(listPointer, v2);
    for(unsigned int i=2; i<s; i++)
        res[i] = va_arg(listPointer, double);
    va_end( listPointer );
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::skinForceControl::identifyCommand(const Bottle &commandBot, const vector<string> &cmdList, unsigned int &cmdId, Bottle &params)
{
	return identifyCommand(commandBot, &cmdList[0], cmdList.size(), cmdId, params);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::skinForceControl::identifyCommand(const Bottle &commandBot, const string *cmdList, unsigned int cmdListSize, unsigned int &cmdId, Bottle &params)
{
    for(unsigned int i=0; i<cmdListSize; i++){
		stringstream stream(cmdList[i]);
		string word;
		int wordCounter=0;
		bool found = true;

		while(stream>>word){
			if (commandBot.get(wordCounter).asString() != word.c_str()){
				found=false;
				break;
			}
            //printf("%s = %s\n", commandBot.get(wordCounter).asString().c_str(), word.c_str());
			wordCounter++;
		}
		if(found){
			cmdId = i;
            for(int k=wordCounter; k<commandBot.size(); k++)
                params.add(commandBot.get(k));
			return true;
		}
	}

	return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iCub::skinForceControl::filt (double v)
{
    double sampling = 1000;
    v/=1000; //from mHz to Hz
    double s = -2*tan(3.14159265*v/sampling); // v frequency in hertz
    v = (2+s)/(2-s);
    v *= 10000;
    return v;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::versor(Vector v)
{
    double nv = norm(v);
    if(nv==0.0) 
        return v;
    return v/nv;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double iCub::skinForceControl::norm1(const Vector &v)
{
    double res=0.0;
    for(unsigned int i=0; i<v.size(); i++)
        res += v[i];
    return res;
}