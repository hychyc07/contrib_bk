/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zmathews@iua.upf.edu   *
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
#include "targetpositionsenderthread.h"



TargetPositionSenderThread::TargetPositionSenderThread()
{
	// initialize the target pos clients vector
	//udpPosClientsVec = new std::vector<UdpClient*>;

	sentEmptyString = false;

		
}

void TargetPositionSenderThread::sendAllTargetsPosToAll(std::vector<TargetObject*> curVector){
	QString sendString = " ";
	//cout << "udpPosClientsVec->size() = " << udpPosClientsVec->size() << endl;
	std::vector<TargetObject*>::iterator targIter;

	for (targIter = curVector.begin(); targIter != curVector.end(); targIter++){
	
	    TargetObject* curTarget = *targIter;
	    int id = curTarget->id;
	    float x = curTarget->x;
	    float y = curTarget->y;
	    float z = curTarget->z;




	    /// -------------------------------------------

		/// normal integer ID
		QString curString = QString("%1,%1,%1,%1,").arg(id).arg(x).arg(y).arg(z);	

		/// superschlaue ID
		//QString curString = curTarget->special_id + "," + QString("%1,%1,%1,").arg(x).arg(y).arg(z);


		sendString = sendString + curString;
		//std::cout << "sendString = " << sendString << std::endl;
	  
	}

	
	


	
	if (!(sendString == " ")){
	    //cout << "sendString not empty......." << endl;
	    sendClientPosData(sendString);
	    sentEmptyString = false;
	    
	}
	else{
	    if (!sentEmptyString){
		//cout << "----------> sendString  empty......." << endl;
		
		sendClientPosData(sendString);
		sentEmptyString = true;
	    }


	}



	//-------------------	special request for Ulysses Pan tilt cam position data
	targIter = curVector.begin();
	if(targIter != curVector.end()){
	    TargetObject* curTarget = *targIter;
	    int id = curTarget->id;
	    float x = curTarget->x;
	    float y = curTarget->y;
	    float z = 0;//curTarget->z;
	    
	    /// --- NOW convert into XIM coordinates ----
	    //x = ((x * 560.0)/600.0) - 280.0;  // x = 0...600 ---> xx = -280..280
	    //y = ((y * 520.0)/600.0) - 260.0;  // y = 0...600 ---> yy = -260..260
            x = x  - 280.0;  // x = 0...560 ---> xx = -280..280
	    y = y  - 267.0;  // y = 0...534 ---> yy = -267..267
	    y = y * -1;
	    z = 0; //
	    
	    /// -------------------------------------------	
	    QString specialSendString = QString("-1;position;"); /// for Ulysses special wish string format
	    QString curSpecialString = QString("%1,%1,%1\n").arg(x).arg(y).arg(z); /// for Ulysses special wish string format
	    specialSendString =  specialSendString + curSpecialString;	/// for Ulysses special wish string format 
	    sendClientPosSpecialData(specialSendString);/// for Ulysses special wish string format   
	}
	//-------------------	end special request for Ulysses 




}

void TargetPositionSenderThread::sendClientPosSpecialData(QString sendStr){ /// for Ulysses special wish string format 
  //specialPositionClient->sendData(sendStr);
}


void TargetPositionSenderThread::sendAllTargetsPosToAllReperCurso(int nofData, std::vector<TargetObject*> targetVector){
	QString sendString = "0,3000,3000,0,";
	//cout << "udpPosClientsVec->size() = " << udpPosClientsVec->size() << endl;

	int maxTargs = 0;
	if (nofData < targetVector.size()){
		maxTargs = nofData;
	}
	else{
		maxTargs = targetVector.size();
		
	}

	if (maxTargs > 0){
		sendString = "";
	}
	

	for (int i=0;i < maxTargs;i++){
		TargetObject* curTarget = targetVector.at(i);
		int id = curTarget->id;
		float x = curTarget->x;
		float y = curTarget->y;
		float z = 0;//curTarget->z;

		/// --- NOW convert into XIM coordinates ----
		//x = ((x * 560.0)/610.0) - 280.0;  // x = 0...610 ---> xx = -280..280
		//y = ((y * 520.0)/610.0) - 260.0;  // y = 0...610 ---> yy = -260..260
		 x = x  - 280.0;  // x = 0...560 ---> xx = -280..280
	   	 y = y  - 267.0;  // y = 0...534 ---> yy = -267..267
		y = y * -1;
		z = 0; //
		
		/// -------------------------------------------
		QString curString = QString("%1,%1,%1,%1,").arg(id).arg(x).arg(y).arg(z);	
		sendString = sendString + curString;
		//cout << "sendString = " << sendString << endl;		
	}
	//cout << "sendString = " << sendString << endl;		
	// now send this string to all the waiting clients
	// just send the empty string ONCE
	if (!(sendString == " ")){
		//cout << "sendString not empty......." << endl;
		sendClientPosData(sendString);
		sentEmptyString = false;
		
	}
	else{
		if (!sentEmptyString){
			//cout << "----------> sendString  empty......." << endl;
			
			sendClientPosData(sendString);
			sentEmptyString = true;
		}
		
			
		
	}
}

/// Ulysses special position fan
void TargetPositionSenderThread::addClientPosSpecialFan(std::string ip, int port){
  //specialPositionClient = new UdpClient(ip, port);
	
}

// this is the ip and port nr of a udp server to send client data to
void TargetPositionSenderThread::addClientPosFan(std::string ip, int port){
  //UdpClient* client = new UdpClient(ip, port);
  //udpPosClientsVec->push_back(client);
}

// sends client pos to all the udp servers waiting for this info
void TargetPositionSenderThread::sendClientPosData(QString posString){

	//for (int i=0;i < udpPosClientsVec->size();i++){
  //curClient = udpPosClientsVec->at(i);
  //	curClient->sendData(posString);
  //	}

}


void TargetPositionSenderThread::stop( )
{
    // set flag to request thread to exit
    // REMEMBER: The thread needs to be woken up once
    // after calling this method to actually exit!
    run_flag = false;
	
}

void TargetPositionSenderThread::run( )
{
//     // do as long as the flag is true
//     while( run_flag )
//     {
//	
//     }

}


TargetPositionSenderThread::~TargetPositionSenderThread()
{
}


