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
#include "targetcreationdeletion.h"

TargetCreationDeletion::TargetCreationDeletion()
{

	dCamTotal = 0.0;
	dClusterTotal = 0.0;
	camTotalNrPeople = 0;
	floorClusterTotalNrPeople = 0;
	totalFloorWeight = 0.0;
	nextID = 100;
	// initilize free IDs to all free
	for(int i =0; i < 3; i++){
		possibleTargetPos[i] = 0.0;
		possibleTargetPosOld[i] = 0.0;
	}

}


TargetCreationDeletion::~TargetCreationDeletion()
{
}




int TargetCreationDeletion::tryCreateNewTarget(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector){

	targVecMutex.lock();
	
	int returnInt = 0;
	bool done = false;
	int nOfTargets = targetVector->size();
	

	/// consider what floor says about the number of targets: if floor does not think that there is 
	/// a new person then exit
	//if ((floorTotalNrPeople >= 0) && (floorTotalNrPeople < (nOfTargets+1))) {targVecMutex.unlock();return 0;}

		
		double x;
		double y;
		int gain;
		double sig_x;
		double sig_y;
		double angle;
		bool ok;
	
		
	
	
	
		int nofData = 0;
		DataElement3D* curData; 
		if (allDataVector != NULL){
			//cout << "kuku.." << endl;
			if (!(allDataVector->empty())){
				//cout << " allDataVector not empty..." << endl;
				nofData = allDataVector->size();
			}
			else{
				nofData = 0;
			
			}

			/// check if cameras has more data than nof targets
			int camData = 0;
			for (int row = 0; row < nofData; row ++){
					curData = allDataVector->at(row);
					if (curData->sensorID == 0){
						camData++;
					}

			}

			if (camData > nOfTargets){
			
		
				for (int row = 0; row < nofData; row ++){
			
					curData = allDataVector->at(row);
					/// NEW: just use camera data
					if (curData->sensorID == 0){
						x = curData->x;
					
						y = curData->y;
		
						double curBestDist = 9999;
						double curBestData[3] = {-1000,-1000,-1000};
						TargetObject* targObj;
						double targX;
						double targY;
						double targZ;
					
						double dist = 9999; /// this is the distance to the closest target

						double z = 0.0;
					
						if (nOfTargets > 0){
							std::vector<TargetObject*>::iterator targIter; 
							for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
	 					
								targObj = *targIter;
	  							targX = targObj->x;	
								targY = targObj->y;
								//targZ = targObj->z;
		
								dist = sqrt((x - targX)*(x - targX) + (y - targY)*(y - targY));
		
								if (dist < curBestDist){
						
									curBestDist = dist;
									curBestData[0] = x;
									curBestData[1] = y;
									curBestData[2] = 0;
						
								}
						
	  						}
						/// this if is cruicial...in GUI coordinates..question is: how far away should the data be from the closest target to be considered as from a NEW target? 				
			
							// 11/11/2008 the first value was 150 before	
							if ((curBestDist > 180) && (curBestDist < 800)) {
								done =true;
								possibleTargetPos[0] = curBestData[0];
								possibleTargetPos[1] = curBestData[1];
								possibleTargetPos[2] = curBestData[2];
								break;
							}
						}
						else{ // if there are no targets yet create one immediately
							done = true;
							curBestDist = 1000;
							timer.previousActiveTime = 0;
						
							possibleTargetPos[0] = x;
							possibleTargetPos[1] = y;
							possibleTargetPos[2] = z;
							break;
						
						}

					} // if curData->sensorID ==0
				}

			
			}  /// if camData > nofTargets


		}  // end for, for going through all the sensors

	
	

	if (done){  // create target at possibletargetPos
		// generate random color 
		// initialize random seed: 
  		srand ( time(NULL) );
		int red = 255;//rand() % 255 + 1;
		int green = 255;//rand() % 255 + 1;
		int blue = 255;//rand() % 255 + 1;
		std::stringstream ss;
		ss << "Target " << nextID;
		
			// TEST: AVOID more than 1 targets
	        
		addTarget( targetVector, nextID, ss.str(), possibleTargetPos[0], possibleTargetPos[1], possibleTargetPos[2], red, green, blue);
		

		nextID++;

		 returnInt = 1;
		
	}
	else{

		/// if floor thinks that there are more people then find the lonely cam data and create a target there
		if ((floorTotalNrPeople >= 0) && (floorTotalNrPeople > nOfTargets)){

			int bestCamDataIndex = -1;
			float maxDist = 0.0;
			int nofData = 0;
			DataElement3D* curData; 
			if (allDataVector != NULL){
				if (!(allDataVector->empty())){
					nofData = allDataVector->size();
				}
				else{
					nofData = 0;
				}
	
				/// go through the data
				
				for (int row = 0; row < nofData; row ++){
					curData = allDataVector->at(row);
					/// only consider cam data
					if (curData->sensorID == 0){
						/// go through all the targets	
						double datax = curData->x;
						double datay = curData->y;
						std::vector<TargetObject*>::iterator targIter; 
						TargetObject* targObj;
						for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 	 	
							targObj = *targIter;
							double targX = targObj->x;	
							double targY = targObj->y;
		
							double curd = sqrt((targX - datax)*(targX - datax)+(targY - datay)*(targY - datay));

							if (curd > maxDist){
								maxDist = curd;
								bestCamDataIndex = row;
							}

						}
					}
				}

			}

			/// now see if the distance is big enough to create a new target
			if ((bestCamDataIndex > -1) & (maxDist > 100.0)){
				/// create a new target at this cam data

				possibleTargetPos[0] = (allDataVector->at(bestCamDataIndex))->x;
				possibleTargetPos[1] = (allDataVector->at(bestCamDataIndex))->y;
				possibleTargetPos[2] = 0.0;

				srand ( time(NULL) );
				int red = 255;//rand() % 255 + 1;
				int green = 255;//rand() % 255 + 1;
				int blue = 255;//rand() % 255 + 1;
				std::stringstream ss;
				ss << "Target " << nextID;
	
				
				addTarget( targetVector, nextID, ss.str(), possibleTargetPos[0], possibleTargetPos[1], possibleTargetPos[2], red, green, blue);
				 

				nextID++;

				returnInt = 1;
			}
		}
		
	}


	targVecMutex.unlock();

	return returnInt;

	
}



/// adds a new target object
void TargetCreationDeletion::addTarget( std::vector<TargetObject*>* targetVec, int id, std::string name, double x, double y, double z, int red, int green, int blue){
  if (targetVec->size() < 1) {
    //TargetObject* target = new TargetObject(id, name, x, y, z, red, green, blue, 0);
  TargetObject* target = new TargetObject(id, name, x, y, z, 1.0,1.0,1.0, red, green, blue, 0);
	targetVec->push_back(target);

  }
	
} 

/// just looks at distance to camera data
void TargetCreationDeletion::tryDeleteTargetNew(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector){
	
	targVecMutex.lock();

	
	int nOfTargets = targetVector->size(); 
	bool deleted = false;
	std::vector<TargetObject*>::iterator targIter; 
	int nofData = allDataVector->size();
	TargetObject * targObj;
	int targX, targY, targZ;
	DataElement3D* curData;


	/// see what floor says about the number of targets:
	/// but this is not good as if track is lost, the lost target will not be deleted
	//if ((floorTotalNrPeople >= 0) && (floorTotalNrPeople > (nOfTargets-1))){targVecMutex.unlock();return;}

	if (nofData > 0){
		noDataTimer.setActiveTime();
	}

	if (nOfTargets > 0){
	
		for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
			
			targObj = *targIter;
	  		targX = targObj->x;	
			targY = targObj->y;
			//targZ = targObj->z;
		
			//if ((targX < 0) || (targX > X_LENGTH) || (targY < 0.0) || (targY > Y_LENGTH)){
			//removeTarget(targetVector, targObj->id); 
			//break;
			//}
					
			if (targObj->timer.getInactiveTime() > 0.2){
			   removeTarget(targetVector, targObj->id); 
			   break;
			}
			
		}
	}


	targVecMutex.unlock();
}


/// returns 1 if a target could be deleted
void TargetCreationDeletion::tryDeleteTarget(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector){
	
	targVecMutex.lock();
	int curNofTargets = targetVector-> size(); 
	bool deleted = false;
	std::vector<TargetObject*>::iterator targIter; 
	if (curNofTargets > 0){
	
		
		for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		
	 	
		if (!deleted){
			/// increment target inactive time
			
			
		
			//std::cout << "id, inactive time = " << (*targIter)->id << ", " << (*targIter)->timer.getInactiveTime() << std::endl;

			/// now if inactiveTime reached 3 sec ,delete it

			if ((*targIter)->timer.getInactiveTime() > 2.0){ //5
				// now see if the target is near the exit door
				// i.e. use the world model
				if ((*targIter)->x > 510){
					removeTarget(targetVector, (*targIter)->id);
					//break;
					
					///collectAllData();
					//cout << "huhu" << endl;
					// compute total gains for each sensor
					///computeSensorTotalGains();
					//cout << "hola" << endl;
					// copy the current gains vec into prev one
					///prevTotalGainVector->assign( curTotalGainVector->begin(), curTotalGainVector->end() );
					///cout << "deletion completed: nof targets = " << targetVector->size() << endl;
					deleted = true;
					break;

				}
				// now also delete if some targets inactive for
				// a long time even though not close to door
				
				else if ((*targIter)->timer.getInactiveTime() > 1 * 4.0){ // 10
					removeTarget(targetVector, (*targIter)->id);
					deleted = true;
					break;
				}
				
			}
		}
		}
	}
	targVecMutex.unlock();
}

/// removes the target from the given vector
void TargetCreationDeletion::removeTarget( std::vector<TargetObject*>* targetVector, int id){

	std::vector<TargetObject*>::iterator targIter; 
	int ii = 0;
	 for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){
	  	if ( (*targIter)->id == id){

		
			targetVector->erase (targetVector->begin()+ ii);
	
			///std::cout << "erased id: " << id << std::endl;
			//freeIDs[ii] = true;
			break;
		}
		ii++;
	  }

}

void TargetCreationDeletion::setTotalFloorWeight(double w){
	totalFloorWeight = w;
	floorTotalNrPeople = (int)(totalFloorWeight / 60.0);

	if ((floorTotalNrPeople < 1)  && (totalFloorWeight > 10.0)){floorTotalNrPeople = 1;}
	if ((floorTotalNrPeople < 2)  && (totalFloorWeight > 120.0)){floorTotalNrPeople = 2;}
	
}



void  TargetCreationDeletion::setFloorClusterTotalNrPeople(std::vector<DataElement3D*>* allDataVector){
	
	int curCamTotal = 0;
	bool gotCamData = false;
	
	int clusterTotal = 0;
	bool gotClusterData = false;
	
	floorDataClusterVector.clear();
	
	// now go through the tracking data and cluster the floor data
	DataElement3D* curData;
	int nofData = allDataVector->size();
	for (int row = 0; row < nofData; row ++){
		curData = allDataVector->at(row);
	
		// only look at floor data
		if (curData->sensorID == 1){
			double x = curData->x;
			double y = curData->y;
			int notInClusterCounter = 0;
			gotClusterData = true;
			//DATA_CLUSTER_RADIUS
			for (int i = 0; i < floorDataClusterVector.size(); i++){
				double curmeanx = floorDataClusterVector.at(i).meanx;
				double curmeany = floorDataClusterVector.at(i).meany;

				float curD = sqrt((x - curmeanx)*(x - curmeanx)+(y - curmeany)*(y - curmeany));

				if (curD > DATA_CLUSTER_RADIUS){
					notInClusterCounter++;
				}
				else{
					// add the data to cluster
					(floorDataClusterVector.at(i)).addData(*curData);
					// and update mean of cluster
					(floorDataClusterVector.at(i)).computeMean();

					

				}
			}
		
			if (notInClusterCounter == floorDataClusterVector.size()){
				// create new cluster
				DataCluster newCluster;
				newCluster.meanx = x;
				newCluster.meany = y;
				floorDataClusterVector.push_back(newCluster);
				
			}
	
			

		}
		else{
			if (curData->sensorID == 0){curCamTotal++;}
			gotCamData = true;

			
			

		}
	}

	double tmp;
	if (gotCamData){
		dCamTotal = 0.9 * dCamTotal + 0.1 * curCamTotal;
		tmp = (ceil)(dCamTotal);
		camTotalNrPeople = (int)(tmp);
		//std::cout << "dCamTotal, tmp, curCamTotal,camTotalNrPeople = " << dCamTotal << ", " << tmp << ", " << curCamTotal << ", " << camTotalNrPeople  << std::endl;
		noCamDataTimer.setActiveTime();
	}
	else{
		if (noCamDataTimer.getInactiveTime() > 0.1){camTotalNrPeople = 0;}
	}

	if (gotClusterData){
		clusterTotal = floorDataClusterVector.size();
		dClusterTotal = 0.9 *  dClusterTotal+ 0.1 * clusterTotal;
		tmp = (ceil)(dClusterTotal);
		floorClusterTotalNrPeople = (int)tmp;
		noFloorDataTimer.setActiveTime();
	}
	else{
		if (noFloorDataTimer.getInactiveTime() > 0.1){floorClusterTotalNrPeople = 0;}
	}



	// now vote for the total nr of people
	if (floorClusterTotalNrPeople  == camTotalNrPeople)
	{
		voteTargetNr = floorClusterTotalNrPeople;
	}
	else if(floorClusterTotalNrPeople == floorTotalNrPeople)
	{
		voteTargetNr = floorClusterTotalNrPeople;
	}
	else if (camTotalNrPeople == floorTotalNrPeople)
	{
		voteTargetNr = camTotalNrPeople;
	}

}





void  TargetCreationDeletion::clusterAllData(std::vector<DataElement3D*>* allDataVector){
	
	
	allDataClusterVector.clear();
	
	// now go through the tracking data and cluster the floor data
	DataElement3D* curData;
	int nofData = allDataVector->size();
	for (int row = 0; row < nofData; row ++){
		curData = allDataVector->at(row);
	
		// only look at floor data
		if ((curData->sensorID == 1) || (curData->sensorID == 0)){
			double x = curData->x;
			double y = curData->y;
			int notInClusterCounter = 0;
			//gotClusterData = true;
			//DATA_CLUSTER_RADIUS
			for (int i = 0; i < allDataClusterVector.size(); i++){
				double curmeanx = allDataClusterVector.at(i).meanx;
				double curmeany = allDataClusterVector.at(i).meany;

				float curD = sqrt((x - curmeanx)*(x - curmeanx)+(y - curmeany)*(y - curmeany));

				if (curD > DATA_CLUSTER_RADIUS){
					notInClusterCounter++;
				}
				else{
					// add the data to cluster
					(allDataClusterVector.at(i)).addData(*curData);
					
					if (curData->sensorID == 0){(allDataClusterVector.at(i)).camData = true;};
					if (curData->sensorID == 1){(allDataClusterVector.at(i)).floorData = true;};
					
					// and update mean of cluster
					(allDataClusterVector.at(i)).computeMean();

					

				}
			}
		
			if (notInClusterCounter == allDataClusterVector.size()){
				// create new cluster
				DataCluster newCluster;
				newCluster.meanx = x;
				newCluster.meany = y;
				
				if (curData->sensorID == 0){newCluster.camData = true;};
				if (curData->sensorID == 1){newCluster.floorData = true;};

				allDataClusterVector.push_back(newCluster);
				
			}
	
			

		}
		
	}

	



}



int TargetCreationDeletion::tryCreateNewTargetVote(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector){
	targVecMutex.lock();
	int returnInt = 0;

	int nOfTargets = targetVector->size();
	
	int bestClusterIndex = -1;
	double curMaxDist = 0.0;	
	std::vector<TargetObject*>::iterator targIter; 

	if (voteTargetNr > nOfTargets){
		/// now create a target at a free cluster
		for (int i = 0; i < allDataClusterVector.size(); i++){
			double curmeanx = allDataClusterVector.at(i).meanx;
			double curmeany = allDataClusterVector.at(i).meany;

			// go through all the targets
			double curClustMax = 9999999999999.0;
			double curD;
			for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
				double x = (*targIter)->x;	
				double y = (*targIter)->y;	
				curD = sqrt((x - curmeanx)*(x - curmeanx)+(y - curmeany)*(y - curmeany));
			
				if (curD < curClustMax){curClustMax = curD;}
			}		

			
			if ((curClustMax > curMaxDist) && (allDataClusterVector.at(i).camData) && ((allDataClusterVector.at(i).floorData))){
				
				curMaxDist = curClustMax; 
				bestClusterIndex = i;
			}
			
		}

		if ((bestClusterIndex > -1) && (curMaxDist > 70.0)){
			
			
				/// create a target at this cluster
				srand ( time(NULL) );
				int red = 255;//rand() % 255 + 1;
				int green = 255;//rand() % 255 + 1;
				int blue = 255;//rand() % 255 + 1;
				std::stringstream ss;
				ss << "Target " << nextID;
				
				float newx = allDataClusterVector.at(bestClusterIndex).meanx;
				float newy = allDataClusterVector.at(bestClusterIndex).meany;
	
				addTarget( targetVector, nextID, ss.str(), newx, newy,0.0, red, green, blue);
			
	
				nextID++;
	
				returnInt = 1;
			
		}
	}

	targVecMutex.unlock();
	return returnInt;
}

int TargetCreationDeletion::tryDeleteTargetVote(std::vector<DataElement3D*>* allDataVector, std::vector<TargetObject*>* targetVector){
	int returnInt = 0;

	targVecMutex.lock();


	int nOfTargets = targetVector->size();
	
		
	std::vector<TargetObject*>::iterator targIter1; 
	std::vector<TargetObject*>::iterator targIter2; 
	int bestInd = -1;
	int curInd = -1;
	double closestDist = 999999999999.0;
	if (voteTargetNr < nOfTargets){
		/// delete one target of the closest ones
		for( targIter1 = targetVector->begin(); targIter1 != targetVector->end(); targIter1++){ 

			curInd++;
			double x1 = (*targIter1)->x;	
			double y1 = (*targIter1)->y;
			
			double curClosest = 99999999.0;
			for( targIter2 = targetVector->begin(); targIter2 != targetVector->end(); targIter2++){ 
			double x2 = (*targIter2)->x;	
			double y2 = (*targIter2)->y;

				double curD = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
			
				if (curD < curClosest){curClosest = curD;}
			}
			
			if (curClosest < closestDist){
				closestDist = curClosest;
				bestInd = curInd;	
			}
		}

		/// now delete the bestInd;

		if (bestInd > -1){
			int minID = (targetVector->at(bestInd))->id;
			removeTarget(targetVector, minID);
		}
	}

	targVecMutex.unlock();
	return returnInt;
}

void TargetCreationDeletion::removeClosestTarget(std::vector<TargetObject*>* targetVec, double x, double y){
	///std::cout << "in removeClosestTarget..." << std::endl;
	int nOfTargets = targetVec->size(); 
	int id = -1;
	double curDist = 99999.0;	
	double bestDist = 99999.0;
	std::vector<TargetObject*>::iterator targIter; 
	int curX, curY, curZ;
	TargetObject * targObj;
	int targX, targY, targZ;

	QString target_type;	

	if (nOfTargets > 0){
		for( targIter = targetVec->begin(); targIter != targetVec->end(); targIter++){ 
			curX = (*targIter)->x;	
			curY = (*targIter)->y;					
			curDist = sqrt((x - curX)*(x - curX) + (y - curY)*(y - curY));
			if (curDist < bestDist){
				bestDist = curDist;
				id = (*targIter)->id;
				target_type = (*targIter)->name;
			}
			
		}
		removeTarget( targetVec, id);
			///std::cout << "removing target type: " << target_type << std::endl;
		
	}

}

