
#ifndef _GROUP_THREAD_H_
#define _GROUP_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <list>
#include <algorithm>

namespace iCub{
namespace pmplib{


class jointThread : public yarp::os::Thread
{
protected:
    bool idle;              // flag that indicates whether the thread is active
	yarp::os::Semaphore* jointSemaphore;
	std::string name;
	int id;
public:
    /**
    * constructor
    */
    jointThread();

    /**
     * destructor
     */
    ~jointThread();

    bool threadInit();     
    void threadRelease();
    void onStop();
	void run();

	/**
	* function that executes the main body of the thread.
	* This function in called inside the while loop in the run function
	* The thread continues executing this function untill the returned value
	* is true, and it stops whenever it becomes false
	*/
	virtual inline bool runnable() = 0;

	/**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
	void setId(int id);
    
    /**
     * function that assigns the shared resource
     */
	void setResource(yarp::os::Semaphore* s) 
	{
		jointSemaphore = s;
		//printf("semaphore added to %s\n",name.c_str()); 
	};
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);
	int getId();

    /**
     * function that activates the thread
     */
    void activate();
	void deactivate();
    
};


class groupThread
{
private:

	yarp::os::Semaphore*   s;
	std::list<jointThread*> threads;
    
public:
    /**
    * constructor
    */
    groupThread()
	{
		s = new yarp::os::Semaphore();
	};

    /**
     * destructor
     */
    ~groupThread()
	{
		interrupt_all();
		delete s;
	};

    
	/**
	* function that assigns a new thread to the group.
	* If the thread was already in the group, this function simply returns
	*/
	void add_thread(jointThread* thrd)
    {
        if(thrd)
        {
			//boost::lock_guard<shared_mutex> guard(m);
			std::list<jointThread*>::iterator const it=std::find(threads.begin(),threads.end(),thrd);
			if(it==threads.end())
			{
				threads.push_back(thrd);
				(threads.back())->setResource(s);
				std::string name = (threads.back())->getName(" ");
				//printf("added: %s\n",name.c_str());
			}
        }
    }
        
    void remove_thread(jointThread* thrd)
    {
        //boost::lock_guard<shared_mutex> guard(m);
        std::list<jointThread*>::iterator const it=std::find(threads.begin(),threads.end(),thrd);
        if(it!=threads.end())
        {
			if((*it)->isRunning())	((*it))->stop();
            threads.erase(it);
        }
    }

	void remove_all()
    {
        //boost::lock_guard<shared_mutex> guard(m);
        for(std::list<jointThread*>::iterator it=threads.begin(),end=threads.end();
            it!=end;
            ++it)
		{
			//threads.erase(it);
			if((*it)->isRunning())	((*it))->stop();
			threads.erase(it);
			//delete *it;
			//printf("removed\n");
		}   
    }

	size_t size() const
    {
		//boost::lock_guard<shared_mutex> guard(m);
        return threads.size();
    }

    /**
     * function that activates the threads
     */
    void activate()
	{
		for(std::list<jointThread*>::iterator it=threads.begin(),end=threads.end();
            it!=end;
            ++it)
		{
			if( !((*it))->isRunning()) ((*it))->start();
			((*it))->activate();
		}
	}; 

	void interrupt_all()
    {            
		//boost::lock_guard<shared_mutex> guard(m);
        for(std::list<jointThread*>::iterator it=threads.begin(),end=threads.end();
            it!=end;
            ++it)
        {
            if((*it)->isRunning())
			{
				((*it))->deactivate();
				((*it))->stop();
			}
        }
		threads.clear();
		//printf("interrupt\n");
    };
    
	void join_all()
	{
		// waiting for the resource to be ready
		//printf("waiting for the resource to be ready .......... \n");
		s->wait();
		//printf("finished \n");
		s->post();
	};
};
}
}

#endif  //_GROUP_THREAD_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------

