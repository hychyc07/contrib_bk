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
#ifndef TARGETPOSITIONSENDERTHREAD_H
#define TARGETPOSITIONSENDERTHREAD_H

#include <qthread.h>
#include <iostream>
#include <string>
#include <vector>



#include "targetobject.h"
#include "definitions.h"
/**
	@author Zenon Mathews <zmathews@iua.upf.edu>
*/
class TargetPositionSenderThread: public QThread{
public:
    TargetPositionSenderThread();
	/** main() function of the thread. */
    void run();
    /** Signal the thread to exit the next time it is woken up. */
    void stop();



    ~TargetPositionSenderThread();

	
	/// vector containing all the target pos clients
	//std::vector<UdpClient*>* udpPosClientsVec;

	// this is the ip and port nr of a udp server to send client data to
	void addClientPosFan(std::string ip, int port);

	// sends client pos to all the udp servers waiting for this info
	void sendClientPosData(QString posString);

	// makes one string out of all the target data and sends it using the above method
	void sendAllTargetsPosToAll(std::vector<TargetObject*> curVector);

	/// for repersurso....just send number of targets as number of data
	void sendAllTargetsPosToAllReperCurso(int nofData, std::vector<TargetObject*> targetVector);

	/// copy the vector (use mutex)
	void updateTargetsVector(std::vector<TargetObject*> curVector);

	// used just to send empty strings once
	bool sentEmptyString;


	/// for Ulysses special wish string format 
	void sendClientPosSpecialData(QString sendStr);

	/// for Ulysses special wish string format 
	void addClientPosSpecialFan(std::string ip, int port);
	
	
private:



    /** Keep the thread running as long this flag is true. */
    volatile bool run_flag;

};

#endif
