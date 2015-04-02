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
#ifndef DATAELEMENT3D_H
#define DATAELEMENT3D_H

/**
	@author Zenon Mathews <zmathews@iua.upf.edu>
*/
class DataElement3D{
public:
	// x,y, z coordinates, the gain, the angle1, angle2 of rotation of ellipsoid, and the 
	// x, y and z variances (axis of the ellipsoid)
	int sensorID;
	///float x, y, z, gain, angle1, angle2, sigx, sigy, sigz;
	float x, y, z, hue, weight;
	bool dataUsed, weightUsed;
	int cam_data_to_target; // this is initialized to -1 and set to the target number if this
				// camera data is associated to a target
				// this is to avoid that one cam data is associated to more 
				// than one target (as this is impossible in XIM)

    ///DataElement3D(int sensID, float xx, float yy, float zz, float gainn, float anglee1, float anglee2, float sigxx, float sigyy, float sigzz);

	DataElement3D(int sensID, float xx, float yy, float zz, float huee, float weightt);

    ~DataElement3D();

};

#endif
