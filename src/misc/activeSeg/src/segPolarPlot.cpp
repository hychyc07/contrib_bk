/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff & Ajay Mishra
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
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


/***************************************************/
/* Segment the polar edge map using Graph Cut      */
/* Author:   Ajay K Mishra         		   	 */
/* Date  :   Aug 10, 2009	    		   		 */
/* email id: mishraka@gmail.com    		      */
/***************************************************/
#include <cstdio>
#include <cstdlib>
#include "iCub/graph.h"
#include "iCub/instances.inc"
#include "iCub/segPolarPlot.h"
#define _USE_MATH_DEFINES
#include <cmath>

//dimensions in polar space!
static int width;
static int height;

//--> Calculate Binary potential; the cost of splitting across bright edges is low!
void  FindB(   double *imgEdge, 
			int i1, int j1, 
			int i2, int j2, 
			double* B)
{
    	double e = M_E; //2.71828;
    	double factor = 1.0;
 	double avgPixVal = (*(imgEdge + width*i1 + j1) + *(imgEdge + width*i2 + j2))/2.0;    	
	*B  = avgPixVal < 5.0/255 ? 50.0 : factor*pow(e, -(avgPixVal*20)); // multiplying factor was change from 5 to 20.(Feb 21, 10)	
}


//--> finds the most optimal path across the polar Edge map!
void	findTheCut(double*	imgEdge_in, const int width_in, const int height_in, unsigned char*	fgMap_out) 
{	
	// Global static variable get assigned!
	width   = width_in;  
    	height  = height_in;
	
	//Create a Graph with number of nodes
	typedef Graph<float,float,float> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ width*height, /*estimated # of edges*/ 2*width*height);

	//Create Node Id array
	GraphType::node_id** node=new GraphType::node_id*[height];
	for(int i = 0; i < height; i++){
		node[i]=new GraphType::node_id[width];
		for(int j = 0; j < width; j++){
			node[i][j] = g->add_node();
		}
	}


	//Set the data costs terms for each pixel and each label 
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			if ( j==0 ){
				g->add_tweights(node[i][j],100.0,0);
			}
			else if (j > width-10){
				g->add_tweights(node[i][j],0,100.0);
			}
			else{	
				g->add_tweights(node[i][j],0,0);
			}
		}
	}

	// Set the neighboring pixels!
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			int pixelQ;
			for(int k = 0; k < 2; k=k+1)
				for(int l = 0; l < 2; l=l+1){
					if (k==l)
						continue;
					else if (i+k == height && j+l >= 0 && j+l < width){
						pixelQ = (j+l); // nodes in the first row
						double cost;
						//printf("\n pair (%d,%d )",pixelP,pixelQ);
						FindB(imgEdge_in, i, j, 0, j+l, &cost); // cost of assigning different labels
						g->add_edge(node[i][j],node[0][j+l],(float)cost,(float)cost);
					}
					else if (i+k >= 0 && i+k < height && j+l >= 0 && j+l < width){
						pixelQ = (i+k)*width+(j+l);
						double cost;
						//printf("\n pair (%d,%d )",pixelP,pixelQ);
						FindB(imgEdge_in,i,j,i+k,j+l, &cost); // cost of assigning different labels
						g->add_edge(node[i][j],node[i+k][j+l],(float)cost,(float)cost);
					}
				}
		}
	}

	float flow = g->maxflow(); // calculate maxFlow

	// label "0" is the source, also foreground!
	for(int i = 0; i < height; i++)
		for(int j = 0; j < width; j++)
			*(fgMap_out + width*i + j) = g->what_segment(node[i][j]) == 0 ? 255 : 0;


	//DONE. Now, release the memory!
	for(int i=0;i<height;i++)
		delete []node[i];
	delete []node;
	delete g;
}


