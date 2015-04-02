#include "portPlayer.h"



bool PortPlayer::open(yarp::os::Searchable &s)
{
	Property options(s.toString());
	
	//Set the input file name
	string inputFileName;
    if (options.check("inputFile")) 	
	{
		inputFileName = options.find("inputFile").asString();
	}
	else
	{
		inputFileName = DEFAULT_INPUT_FILE_NAME;
	}

	inputFile = new ifstream(inputFileName.c_str());
	if (!inputFile->is_open())
	{
		cout<<"Problem while opening the input file."<<endl;
		return false;
	}

	//Set the loop parameter
    if (options.check("loop")) 	
	{
		isLooping = options.find("loop").asInt() == 1;
	}
	else
	{
		isLooping = true;
	}

	//Set the standard output parameter
    if (options.check("verbose")) 	
	{
		isVerbose = options.find("verbose").asInt() == 1;
	}
	else
	{
		isVerbose = true;
	}
	//Load the emulated ports names from the file
	//and create Players
	cout<<"Creating emulated ports : "<<endl;
	string line; getline (*inputFile,line);

	while (line != "#DATA#" && !inputFile->eof())
	{	
		int playerID;
		string playerName;
		stringstream ss(line); // Insert the string into a stream
		ss >> playerID;
		ss >> playerName;
		cout<<"Player ID : "<<playerID<<endl<<"PlayerName : "<<playerName<<endl;
		BufferedPort<Bottle>* currentPort = new BufferedPort<Bottle>;
		currentPort->open(playerName.c_str());
		this->Players.push_back(currentPort);
		getline (*inputFile,line); 
	}
	cout<<"Ports creation done."<<endl;

	//Load the file in memory
	cout<<"Loading file...";
	getline (*inputFile,line);
	while ( !inputFile->eof() )
	{	
		Record cr;

		stringstream ss(line); // Insert the string into a stream
		ss >> cr.playerIndex;
		ss >> cr.recordNumber;
		ss >> cr.playTime;
		string contentItem;
		string fullContent = "";
		while (!ss.eof())
		{
			ss >> contentItem;
			fullContent += " " + contentItem;
		}
		cr.content.fromString(fullContent.c_str());

		inMemoryFile.push_back(cr);
		getline (*inputFile,line);
	}
	cout<<"OK."<<endl<<endl;
	cout<<"Starting !"<<endl;
	startTime = Time::now();
	nextItem = inMemoryFile.begin();

   return true;
}

bool PortPlayer::close()
{
	inputFile->close();
	delete inputFile;
	for(vector<BufferedPort<Bottle>* >::iterator it = Players.begin() ; it!= Players.end() ; it++)
	{
		(*it)->close();
		delete (*it) ;
	}
	cout<<"Closing in 5 seconds..."<<endl;
	yarp::os::Time::delay(5);
	return true;
}

bool PortPlayer::updateModule()
{
	if (isPlaying)
	{
		double currentTime = Time::now();
		double timeSinceStart = currentTime - startTime;

		//While we don't have to wait
		while( nextItem->playTime <= timeSinceStart )
		{
			Record cr = *nextItem;

			Bottle& b = Players[cr.playerIndex]->prepare();
			b.copy(cr.content);
			if (isVerbose)
				cout<<"Playing : "<<b.toString().c_str()<<endl;

			Players[cr.playerIndex]->write();
			
			if (this->nextItem != this->inMemoryFile.end()-1)
				this->nextItem++;
			else
			{
				if (isLooping)
				{
					this->nextItem = this->inMemoryFile.begin();							
					startTime = currentTime;
					timeSinceStart = 0;
					return true;
				}
				else
				{
					isPlaying = false;
					return true;
				}
			}
		}
	}
	return true;
}

double PortPlayer::getPeriod()
{
	return (1/1000.0);
}
