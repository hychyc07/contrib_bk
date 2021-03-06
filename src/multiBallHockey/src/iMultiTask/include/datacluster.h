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
#ifndef DATACLUSTER_H
#define DATACLUSTER_H

#include <vector>
#include "dataelement3d.h"

/**
	@author Zenon Mathews <zenon.mathews@upf.edu>
*/
class DataCluster{
public:
    DataCluster();
	
	bool camData;
	bool floorData;
	double meanx, meany; // the mean position of the cluster
	std::vector<DataElement3D> dataVector;
	void computeMean();
	void addData(DataElement3D data);

    ~DataCluster();

};

#endif
