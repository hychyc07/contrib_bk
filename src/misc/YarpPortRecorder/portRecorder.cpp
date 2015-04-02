
#include "portRecorder.h"

PortRecorder::PortRecorder(string _sourcePort, int _ID, int _nrecords, ofstream* _file, Semaphore* _mutex)
				:BufferedPort<Bottle>()
				{
					sourcePort = _sourcePort;
					ID = _ID;
					file = _file;
					mutex = _mutex;
					mnrecords = _nrecords;
					mreccounter = 1;
					mstarttime = Time::now();
					mhasfinished = false;
				}


void PortRecorder::recordOnce()
{
   
	Bottle* b = read(false); //needs a nonblocking read in case the port is silent
	if (b!=NULL)
	{
	  double runningtime = Time::now()-mstarttime;
	  if (((mnrecords>0)&&(mreccounter<=mnrecords))||(mnrecords==0))
      {
		mutex->wait();
			(*file)<<	ID	<<	'\t'	<<  '\t' << mreccounter << '\t'<< runningtime << '\t'	<< b->toString().c_str() << endl;
		mutex->post();
		mreccounter++;
	  }
	  if ((mreccounter>mnrecords) && (mnrecords != 0)) mhasfinished = true;
    }
}

bool PortRecorder::Connect()
{
	if( Network::connect(sourcePort.c_str(),this->getName().c_str()) )
		return true;
	else
		return false;
}

bool PortRecorder::isFinished()
{
	return mhasfinished;
}

