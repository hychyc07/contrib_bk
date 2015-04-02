/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea Del Prete
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

#ifndef DI_UTIL_H
#define DI_UTIL_H

#include <vector>
#include <list>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/common.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace std;

namespace iCub
{

namespace distanceImitation
{

    struct TrialInfo{
        string filename;
        double T;
        double speed;
        double d_min; 
        double step;
        unsigned int N;
        unsigned int k_max; 
        Vector gaze_home;
        Vector q_home;
        Vector x0;
    };

    bool writeTrialInfoToFile(ofstream &file, const TrialInfo &info);

   

    // Get current date/time, format is YYYYMMDD_HH:mm:ss
    const std::string currentDateTime();


    enum MsgType{ MSG_INFO=0, MSG_WARNING, MSG_ERROR};
    const std::string MsgType_s[]  = { "INFO", "WARNING", "ERROR" };

    /**
     * Utility class used mainly to play the role of exceptions when exceptions cannot be used.
     * A method that can fail returns a Status, containing a bool "ok" and a string "errstr".
     * If "ok" is true the operation succeeded, if "ok" is false the operation failed and 
     * "errstr" contains the message describing the error.
     */
    class Status
    {
    public:
        bool ok;
        std::string errstr;

        /**
         * Default constructor. 
         * Build a Status with "ok" equal to true and an empty "errstr".
         */
        inline Status(): ok(true), errstr("") {}

        /**
         * Constructor for a failure Status ("ok"=false)
         * @param errstr the error message
         */
        inline Status(std::string const &errstr): ok(false), errstr(errstr) {}

        /**
         * Constructor.
         * @param ok the bool value
         * @param errstr the error message
         */
        inline Status(bool ok, std::string const &errstr): ok(ok), errstr(errstr) {}

        inline operator bool () const { return ok; }
        
        /**
         * Merge the current status with the specified status s.
         * The bool are multiplied (AND operator) and the string are concateneted.
         * @param s the status to merge with the current status
         * @return a new status obtained merging the current status with the s
         */
        Status& operator&=(const Status &s){
            ok = ok && s.ok;
            if(s.errstr== "")
                errstr = errstr;
            else if(errstr=="")
                errstr = s.errstr;
            else
                errstr = errstr + "; "+s.errstr;
            return *this;
        }

        /**
         * Merge the current status with the specified status s.
         * The bool are multiplied (AND operator) and the string are concateneted.
         * @param s the status to merge with the current status
         * @return a new status obtained merging the current status with the s
         */
        Status operator&&(const Status &s){
            Status res;
            res.ok = ok && s.ok;
            if(s.errstr== "")
                res.errstr = errstr;
            else if(errstr=="")
                res.errstr = s.errstr;
            else
                res.errstr = errstr + "; "+s.errstr;
            return res;
        }

        /**
         * Set the ok value to false and concatenate the specified string
         * to the current errstr.
         * @param err the error message to concatenate to the current errstr
         */
        inline void addErrMsg(std::string const &err){ ok=false; errstr+="; "+err;}

        inline const char* toString(){ return errstr.c_str(); }

    };

    template <class T>
    T safeGet(const T& value, yarp::os::Semaphore& mutex){
        mutex.wait();
        T res = value;
        mutex.post();
        return res;
    }

    template <class T>
    void safeSet(T& var, const T& value, yarp::os::Semaphore& mutex){
        mutex.wait();
        var = value;
        mutex.post();
    }

    template <class T>
    inline std::string toString(const T& t){
        std::stringstream ss;
        ss << t;
        return ss.str();
    }

    inline yarp::os::Bottle toBottle(const int i){
        yarp::os::Bottle b;
        b.addInt(i);
        return b;
    }

    Status bottleToVector(const yarp::os::Bottle &b, yarp::sig::Vector &v) throw();

    void addToBottle(yarp::os::Bottle &b, const yarp::sig::Vector &v) throw();

    void addToBottle(yarp::os::Bottle& b, const std::vector<yarp::sig::Vector>& v) throw();

     /**
      * Identify the command in the bottle and return the correspondent id.
      * All the elements of the Bottle that are after the identified command are inserted 
      * in the params bottle.
      * @param commandBot a bottle containing the command to identify
      * @param cmdList a vector containing all the possible commands
      * @param cmdId the id of the identified command (if it's been identified)
      * @param params all the elements following the command in the commandBot
      * @return true if the command has been identified, false otherwise
      */
    bool identifyCommand(const yarp::os::Bottle &commandBot, const std::vector<std::string> &cmdList, unsigned int &cmdId, yarp::os::Bottle &params);
    bool identifyCommand(const yarp::os::Bottle &commandBot, const std::string *cmdList, unsigned int cmdListSize, unsigned int &cmdId, yarp::os::Bottle &params);

    

}

} // end namespace

#endif

