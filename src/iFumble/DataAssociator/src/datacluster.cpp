/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zenon.mathews@upf.edu   *
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
#include "datacluster.h"

DataCluster::DataCluster()
{
	camData = false;
	floorData = false;
}

void DataCluster::addData(DataElement3D data){
	dataVector.push_back(data);
}

void DataCluster::computeMean(){
	double totalx = 0.0;
	double totaly = 0.0;
	for (int i = 0; i < dataVector.size(); i++){
		totalx += dataVector.at(i).x;
		totaly += dataVector.at(i).y;
	}
	meanx = totalx / dataVector.size();
	meany = totaly / dataVector.size();
	
}

DataCluster::~DataCluster()
{
}


