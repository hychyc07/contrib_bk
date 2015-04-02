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



#ifndef DEFINITIONS
#define DEFINITIONS



struct TwoDdata
    	{
      		float x;
		float y;
		float f1;
		float f2;
    	};

// icub july 2010
#define GATE_PARAM1 100.0
#define GATE_PARAM2 500.0
#define MIN_AXIS 5.0	
#define MAX_AXIS 400.0
#define PRED_FACTOR 1.5	

#define TAP_RIGHT_X 500.0
#define TAP_LEFT_X 100.0	
			
#define DRAW_VAL_GATE
/// coordinate system and for the window size of the graphic display of targets	
#define X_LENGTH 600
#define Y_LENGTH 600

#define TABLE_WIDTH 10.0
#define TABLE_HEIGHT 10.0

#define ROBOT_X X_LENGTH/2.0
#define ROBOT_Y Y_LENGTH-Y_LENGTH/4.0

/// july 2010: target object kalman state size: x,y,z,vx,vy,vz,ax,ay,az,hue,weight
#define KALMAN_SIZE 11 
/// measurement vector size: x,y,z,hue,weight
#define MES_SIZE 5

/// control vector size for the kalman process 
#define CONTROL_SIZE 0

// validation gate threshold for validationMatrix computation in dataassociator.cpp

/// define speed in HZ of the application
#define APP_SPEED 40.0

/// nof hue bins for hue computation 
#define NOF_HUEBINS 20
#define MAX_DIST_TO_HUE_POS 50.0

/// for spreading the camera data using a Gaussian
#define GAUSSIAN_SIGMA_DATA_SPREAD 10.0
#define NOF_SPREAD_DATA 10

/// MCMC computation of association probabilities
#define MONTE_CARLO_ITERATIONS 1000

/// weight of the floor data relative to the cam data
#define FLOOR_DATA_WEIGHT_INSIDE 0.3	
#define FLOOR_DATA_WEIGHT_BORDER 2.8
#define CAM_DATA_WEIGHT_INSIDE 2.0
#define CAM_DATA_WEIGHT_BORDER 0.8
#define GAZER_DATA_WEIGHT 0.5

/// to check if the associated data is not too close to some other target

#define MIN_INTER_TARGET_DIST_CAM 55.0
#define MIN_INTER_TARGET_DIST_FLOOR 55.0
#define MIN_INTER_TARGET_DIST_GAZER 25.0

/// define the maximum growth of validation gate if target is lost
#define MAX_GROWTH_SIZE_VAL_GATE 100.0

// the position data is filtered to Torque
#define SEND_XY_THRESHOLD 10


#define DATA_CLUSTER_RADIUS 80.0

#endif



