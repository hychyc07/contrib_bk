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
#include "dataelement3d.h"

// DataElement3D::DataElement3D(int sensID, float xx, float yy, float zz, float gainn, float anglee1, float anglee2, float sigxx, float sigyy, float sigzz): sensorID(sensID), x(xx), y(yy), z(zz), gain(gainn), angle1(anglee1), angle2(anglee2), sigx(sigxx), sigy(sigyy), sigz(sigzz),cam_data_to_target(-1)
// {
// }


DataElement3D::DataElement3D(int sensID, float xx, float yy, float zz, float huee, float weightt): sensorID(sensID), x(xx), y(yy), z(zz), hue(huee),weight(weightt),cam_data_to_target(-1),dataUsed(false),weightUsed(false)
{
}


DataElement3D::~DataElement3D()
{
}


