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

#ifndef SFC_UTIL_H
#define SFC_UTIL_H

#include <vector>
#include <map>
#include <sstream>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
#include <ace/Recursive_Thread_Mutex.h>
#include <ace/Thread.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/common.h"
#include "iCub/skinForceControl/robot_interfaces.h"

namespace iCub
{

namespace skinForceControl
{
    enum MsgType{ MSG_INFO=0, MSG_WARNING, MSG_ERROR};
    const std::string MsgType_s[]  = { "INFO", "WARNING", "ERROR" };

    class DSemaphore: public yarp::os::Semaphore
    {
    protected:
        static yarp::os::Semaphore mutex;
        static std::map<int, std::string> threadNames;
        std::string name;
        void sendMsg(const std::string& msg){
            /*if(DSemaphore::threadNames[ACE_Thread::self()] != "")
                printf("[Thread %s] %s\n", DSemaphore::threadNames[ACE_Thread::self()].c_str(), msg.c_str());
            else
                printf("[Thread %d] %s\n", ACE_Thread::self(), msg.c_str());*/
        }
    public:
        DSemaphore();
        DSemaphore(const std::string& _name);
        std::string getName();
        void setName(const std::string& _name);
        /*virtual void wait();
        virtual void post();*/
        static void registerThread(const std::string& threadName);
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
    class atomic{
    protected:
        T data;
        //yarp::os::Semaphore* mutex;
        ACE_Recursive_Thread_Mutex* mutex;

    public:
        atomic(){
            mutex = new ACE_Recursive_Thread_Mutex();
        }

        atomic(T _data):data(_data){
            mutex = new ACE_Recursive_Thread_Mutex();
        }

        atomic(T _data, ACE_Recursive_Thread_Mutex* _mutex):data(_data), mutex(_mutex){}
         
        atomic(const atomic &a): data(a.data){
            mutex = new ACE_Recursive_Thread_Mutex();
        }

        const atomic &operator=(const atomic &a){
            data = a.data;
            mutex = new ACE_Recursive_Thread_Mutex();
            return *this;
        }

        T get(){
            mutex->acquire();
            T res = data;
            mutex->release();
            return res;
        }

        void set(T _data){
            mutex->acquire();
            data = _data;
            mutex->release();
        }
    };

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
    * Build a vector and initialize it with the values specified as input parameters.
    * Note that this method doesn't work if the values are specified as integer such as:
    *    buildVector (3, 0, 1, 2);
    * The correct use is:
    *    buildVector (3, 0., 1., 2.);   OR   buildVector(5, 0.0, 1.0, 2.0);
    */
    yarp::sig::Vector buildVector(size_t s, const double v1, const double v2, ...);

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
    
    /**
     * Given the cut frequency of a low pass filter compute the corresponding pole.
     * @param v cut frequency in mHz
     */
    double filt (double v);

    yarp::sig::Vector versor(yarp::sig::Vector v);

    double norm1(const yarp::sig::Vector &v);

}

} // end namespace

#endif

