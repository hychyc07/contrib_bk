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

#include "iCub/distanceImitation/util.h"
#include <string>
#include <stdarg.h>     // va_list va_arg va_end va_start
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/dev/PolyDriver.h>

using namespace iCub::distanceImitation;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::distanceImitation::writeTrialInfoToFile(ofstream &file, const TrialInfo &info)
{
    if(!file.is_open() || !file.good())
        return false;
    file<< "filename\t"<< info.filename<< endl;
    file<< "d_min\t"<< info.d_min<< endl;
    file<< "T\t"<< info.T<< endl;
    file<< "speed\t" << info.speed<< endl;
    file<< "step\t"<< info.speed<< endl;
    file<< "N\t"<< info.N<< endl;
    file<< "k_max\t"<< info.k_max<< endl;
    file<< "gaze_home\t"<< info.gaze_home.toString(2).c_str()<< endl;
    file<< "q_home\t"<< info.q_home.toString(0).c_str()<< endl;
    file<< "x0\t"<< info.x0.toString(3).c_str()<< endl;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const std::string iCub::distanceImitation::currentDateTime() 
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y%m%d_%X", &tstruct);
    return buf;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Status iCub::distanceImitation::bottleToVector(const Bottle& b, Vector& v) throw()
{
    Status res;
    v.resize(0);
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
void iCub::distanceImitation::addToBottle(yarp::os::Bottle &b, const yarp::sig::Vector &v) throw()
{
    for(unsigned int i=0; i<v.size(); i++)
        b.addDouble(v[i]);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::distanceImitation::addToBottle(Bottle& b, const vector<Vector>& v) throw()
{
    for(unsigned int i=0; i<v.size(); i++)
        for(unsigned int j=0; j<v[i].size(); j++)
            b.addDouble(v[i][j]);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::distanceImitation::identifyCommand(const Bottle &commandBot, const vector<string> &cmdList, unsigned int &cmdId, Bottle &params)
{
	return identifyCommand(commandBot, &cmdList[0], cmdList.size(), cmdId, params);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::distanceImitation::identifyCommand(const Bottle &commandBot, const string *cmdList, unsigned int cmdListSize, unsigned int &cmdId, Bottle &params)
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