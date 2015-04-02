// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea, 
  * email: francesco.reak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file segment.h
 * @brief Implementation of the eventDriven thread (see colorVisionThread.h).
 */


/*
This is a library for color image segmentation.
 */




// The arithmetic in which MRF optimization is done.
typedef float real;


void compute_unary_potentials(
  const unsigned char *,
  const float *,
  int,
  int,
  real *
);

void reparameterize_unary_potentials(
  const unsigned *,
  int,
  real *,
  int,
  int,
  const real *
);

unsigned *grid_graph(
  int,
  int,
  int *
);

float trws_potts(
  const unsigned *,
  int,
  real,
  real *,
  int,
  int,
  real *,
  real
);

int extract_labeling(
  const real *,
  int,
  int,
  unsigned char *
);

unsigned connected_components(
  const unsigned char *,
  int, int,
  unsigned,
  unsigned *
);

void bounding_boxes(
  const unsigned *,
  const unsigned char *,
  int, int,
  int,
  int *
);
