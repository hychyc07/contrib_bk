#include "YarpPortRecorder.h"



bool YarpPortRecorder::open(yarp::os::Searchable &s)
{
	
	bool _res1=false;
	bool _res2=false;
	vector<string> _localportnames;
	vector<string> _remoteportnames;
	vector<int> _nrecords;
	string _rpcportname;
	Property options(s.toString());
	//Treat configuration file
    if (options.check("name")) 	
	{
		mmodulename=options.find("name").asString();
		if (options.check("captureinterval"))
		{
			mcaptureperiod=options.find("captureinterval").asInt();
			if(mcaptureperiod>=0)
			{
				if (options.check("delayb4capture"))
				{
					mdelayb4capture=options.find("delayb4capture").asInt();
					if(mdelayb4capture>=0)
					{
						if (options.check("filesizelimit"))
						{
							mfilesizelimit=options.find("filesizelimit").asInt();
                            if (mfilesizelimit>0)
							{
								if (options.check("outputfilename"))
								{
									string ouputFileName = options.find("outputfilename").asString().c_str();
                                    moutputFile = new ofstream(ouputFileName.c_str());
									if (moutputFile!=NULL)
									{
										if (options.check("rpc"))
										{
                                            string _rpcportending = options.find("rpc").asString().c_str();
											_rpcportname=mmodulename+_rpcportending;
											//first half of the config file is ok, proceed to port section treatment
										    _res1=true;
										}
										else
										{
											cout<<"rpc parameter was not found."<<ouputFileName<<endl;
										}
									
									}
									else
									{
										cout<<"Can not create the output file "<<ouputFileName<<endl;
									}
								}
								else
								{
									cout<<"Output file name parameter was not found."<<endl;
								}

							}
							else
							{
								cout<<"File size limit parameter should be positive."<<endl;
							}
						}
						else
						{
							cout<<"File size limit parameter was not found."<<endl;
						}

					}
					else
					{
						cout<<"Delay b4 capture parameter should not be negative."<<endl;
					}

				}
				else
				{
					cout<<"Delay bfore capture parameter was not found."<<endl;
				}

			}
			else
			{
				cout<<"Capture period should not be negative."<<endl;
			}
		}
		else
		{
			cout<<"Capture period parameter was not found."<<endl;
		}
	}
	if (_res1)
	{
		string _remoteportparameterbegins = "remote";
		string _localportparameterbegins="local";
		string _nrecordsparameterbegins="nrecords";
		string _remoteportfullparamname;
		string _localportfullparamname;
		string _nrecordsfullparamname;
		unsigned int _portnumber = 1;
		bool _continue = true;
		do
		{
			ostringstream _strm1,_strm2,_strm3;
		    _strm1<<_remoteportparameterbegins<<_portnumber;
			_strm2<<_localportparameterbegins<<_portnumber;
			_strm3<<_nrecordsparameterbegins<<_portnumber;

		    _remoteportfullparamname=_strm1.str();
		    _localportfullparamname=_strm2.str();
		    _nrecordsfullparamname=_strm3.str();
			
			bool _rmtportfound=options.findGroup("PORTS").check(_remoteportfullparamname.c_str());
			bool _lclportfound=options.findGroup("PORTS").check(_localportfullparamname.c_str());
			bool _nrecordsparfound = options.findGroup("PORTS").check(_nrecordsfullparamname.c_str());

			if ((_rmtportfound)&&(_lclportfound)&&(_nrecordsparfound))
			{
				string _rmtprtnm = options.findGroup("PORTS").find(_remoteportfullparamname.c_str()).asString().c_str();
				string _lclprtnm = options.findGroup("PORTS").find(_localportfullparamname.c_str()).asString().c_str();
				int _nrecord = options.findGroup("PORTS").find(_nrecordsfullparamname.c_str()).asInt();
				if (!_rmtprtnm.empty())
				{
					if (!_lclprtnm.empty())
					{
						if(_nrecord>=0)
						{
							_localportnames.push_back((mmodulename+_lclprtnm));
				            _remoteportnames.push_back(_rmtprtnm);
				            _nrecords.push_back(_nrecord);
				            _portnumber++;
							_res2=true;
						}
						else
						{
							_res2=false;
							_continue=false;
							cout<<"Number of records should be positive."<<endl;

						}
					}
					else
					{
						_res2=false;
						_continue=false;
						cout<<"Local port name should not be empty or contain spaces."<<endl;

					}
				}
				else
				{
					_res2=false;
					_continue=false;
					cout<<"Remote port name should not be empty or contain spaces."<<endl;
				}
				

			}
			else 
			{
				_continue = false;
			}

		}
		while(_continue);
	}

	if (_res2)
	{
		//Open RPC port
	     if(!mrpcPort.open(_rpcportname.c_str()))
		 {
			 cout<<"Can not open rpc port "<<_rpcportname<<endl;
			 return false;
		 }
         if (!attach(mrpcPort,true))
		 {
			 cout<<"Can not attach rpc port"<<_rpcportname<<endl;
			 return false;

		 }


		//comes here only when port section of the config file is ok
		//if there is a delay before capture then better to have it here before recorders are created
		//As the recorders store time of their creation upon construction then puting the below delay code after
		//recorder construction would introduce a this delay into the dump file, which is not desirable.
		if (mdelayb4capture>0) 
		{
			cout<<"Capture delayed "<<mdelayb4capture<<" msec."<<endl;
			Time::delay(mdelayb4capture/1000.0); 
		}
        //creating recorders
		for (vector<string>::size_type i=0;i<_remoteportnames.size();++i)
		{
			int recorderID = (int)i;
			PortRecorder* nextRecorder = new PortRecorder(_remoteportnames[i],recorderID,_nrecords[i],moutputFile,mmutex);
			if (nextRecorder!=NULL)
			{
				if (nextRecorder->open(_localportnames[i].c_str()))
				{
					mrecorders.push_back(nextRecorder);
					//Create the ouputfile header
			        (*moutputFile)<<recorderID<<'\t'<<_remoteportnames[i]<<endl;

				}
				else
				{
					cout<<"Cant open local port "<<_localportnames[i]<<endl;
					return false;
				}
			}
			else
			{
				cout<<"Cant not create port recorder "<<_localportnames[i]<<endl;
				return false;
			}
				
		}
		(*moutputFile)<<"#DATA#"<<endl;
		for(vector< PortRecorder* >::iterator pIt = mrecorders.begin() ; pIt != mrecorders.end() ; pIt++)
	    {
		   if (!(*pIt)->Connect())
		   {
			   cout<<"Cant not connect recorder " <<(*pIt)->getName().c_str()<<" to remote port"<<endl;
			   return false;
		   }
	    }
		(*misRecording)=true;
		return true;
	}
	else return false;
}

bool YarpPortRecorder::close()
{
	for(vector< PortRecorder* >::iterator pIt = mrecorders.begin() ; pIt != mrecorders.end() ; pIt++)
	{
		(*pIt)->close();
	}
	moutputFile->close();
	delete moutputFile;
	delete mmutex;

	mrpcPort.close();
	cout<<"Closing in 5 seconds..."<<endl;
	yarp::os::Time::delay(5);
	return true;
}

bool YarpPortRecorder::updateModule()
{
	int countfinished = 0;
    if ((*misRecording))
	{
		for (vector< PortRecorder*>::size_type it = 0; it<mrecorders.size(); it++)
		{
			if (!mrecorders[it]->isFinished())
				mrecorders[it]->recordOnce();
			else
				countfinished++;
		}
		if (countfinished == (int)mrecorders.size())
		{
			cout<<"All recorders have finished" <<endl;
			return false;
		}
		else return true;
	}
	else return true;
}

double YarpPortRecorder::getPeriod()
{
	return (mcaptureperiod/1000.0);
}

bool YarpPortRecorder::respond(const Bottle &command, Bottle &reply)
{
    cout << "Recieved command from rpc port" << endl;

    switch (command.get(0).asVocab())
    {

		case VOCAB3('r','e','c'):
        {
			if (!(*misRecording))
			{
				(*misRecording) = true;
				reply.addInt(1);
				reply.addString("Module recording.");
			}
			else
			{
				reply.addInt(-1);
				reply.addString("Module is already recording.");
			}
			cout<<reply.toString().c_str()<<endl;
            return true;
        }

		case VOCAB4('p','a','u','s'):
        {
			if (!(*misRecording))
			{
				reply.addInt(-1);
				reply.addString("Module is already paused.");
			}
			else
			{
				(*misRecording) = false;
				reply.addInt(1);
				reply.addString("Module paused.");
			}
			cout<<reply.toString().c_str()<<endl;
            return true;
        }
        default:
        {
			return Module::respond(command,reply);
        }
    }

    return false;
}
