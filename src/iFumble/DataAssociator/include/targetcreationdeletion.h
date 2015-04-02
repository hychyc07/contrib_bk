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
#ifndef TARGETCREATIONDELETION_H
#define TARGETCREATIONDELETION_H


#include <iostream>
#include <vector>

//#include <math.h>
//#include <stdlib.h>

#include<sstream>
#include <time.h>
#include "cv.h"
#include "targetobject.h"
#include "timer.h"
#include "dataelement3d.h"
#include "qmutex.h"
#include "datacluster.h"

/**
	@author Zenon Mathews <zmathews@iua.upf.edu>
*/
class TargetCreationDeletion{
public:
    TargetCreationDeletion();

    ~TargetCreationDeletion();

	
	 /** for automatic target creation: stores the data position **/
	double possibleTargetPos[3]; // x, y, z
	double possibleTargetPosOld[3];
	Timer timer;			
	/** stores the next available ID for targets**/
	int nextID;

	/// returns 1 if it could create a new target (looks at distance to existing ones)
	int tryCreateNewTarget(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVec);

	
	void tryDeleteTargetNew(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVec);
	Timer noDataTimer;

	/// add a new target into the given vector
	void addTarget( std::vector<TargetObject*>* targetVec, int id, std::string name, double x, double y, double z, int red, int green, int blue);

	/// tries if a target could be deleted
	void tryDeleteTarget(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVec);

	/// removes the target from the given vector
	void removeTarget( std::vector<TargetObject*>* targetVec, int id);
	
	QMutex targVecMutex;
	void removeClosestTarget(std::vector<TargetObject*>* targetVec, double x, double y);

	// total weight on the floor (used to create and delete)
	double totalFloorWeight;
	int floorTotalNrPeople;
	int floorClusterTotalNrPeople;
	int camTotalNrPeople;
	double dCamTotal, dClusterTotal;
	std::vector<DataCluster> floorDataClusterVector; // just with floor
	std::vector<DataCluster> allDataClusterVector; // with all data
	int voteTargetNr;
	
	void setFloorClusterTotalNrPeople(std::vector<DataElement3D*>* allDataVector);
	
	void setTotalFloorWeight(double w);
	Timer noCamDataTimer;
	Timer noFloorDataTimer;
	
	int tryCreateNewTargetVote(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector);
	int tryDeleteTargetVote(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector);
	void clusterAllData(std::vector<DataElement3D*>* allDataVector);
		
};

#endif
