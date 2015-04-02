#include <stdio.h>

//#include <cvSeqLabel.h>
#include "cvSeqLabel.h"

#define MAX_LABELS 100000

//private function
int getLeaf(int label, int *equiv_array)
{
	int minlabel;

	if(label>=MAX_LABELS || label<1)
		printf("getleaf%d\n",label-1);
             
	if(equiv_array[label-1])
	{
		minlabel = label;
		while(equiv_array[minlabel-1])
			minlabel = equiv_array[minlabel-1];
	}
	else //original leaf
	{
		minlabel = 0;
	}
	return minlabel;
}


/*
Function: cvSeqLabel 
Description: Implements the sequential labeling algorithm on binary images 
             to extract the connected components.
Dependencies: Uses the OpenCV library
Arguments: in - binary image containing background (0) and foreground (!0) pixels
           out - image containing the labels (0 - background, 1-255 connected objects' labels )

Limitations: Allows only 255 labels. Assumes output image is clear.
Author: Alex 03/01/2006
*/
int cvSeqLabel( IplImage *in, IplImage *out, IplImage *tmp)
{
	int i, j, label = 0, minlabel, originalb, originalc, equiv_array[MAX_LABELS];
	int width, height, stride;
	unsigned char *input, *output;
	int *temp;
	unsigned char *pai, *pbi, *pci, *pdi;
	int *pao, *pbo, *pco, *pdo;
	
	memset( equiv_array, 0, sizeof(int)*MAX_LABELS );

	input=(unsigned char*)in->imageData;
	output=(unsigned char*)out->imageData;
	temp=(int*)tmp->imageData;
		
	//fprintf(stdout, "initial temp[]=%d\n", temp[0]);
	
	width = out->width;
	height = out->height;
	stride = tmp->widthStep/sizeof(int);
	
	
	// first pixel
	if(input[0])
		temp[0] = ++label;

	//first line
	for(i = 1; i < width; i++)
	{
		if(input[i])
		{
			if(input[i-1])   // test left neighbor
			{
				temp[i] = temp[i-1];  //preserve connected label
			}
			else
			{
				if(label == MAX_LABELS-1)
				         return MAX_LABELS-1; // end of free labels
				temp[i] = ++label; //new label
			}
		}
	}

	pai = input + stride;	pci = input;
	pao = temp + stride;	pco = temp;
	

	//remaining lines
	for(i = 1; i < height; i++)
	{
        //first column
		if(*pai)
		{
			if(*pci) //test up neighbor
			{
				*pao = *pco; //preserve connected label
			}
			else
			{
				//label+=4;
				if(label == MAX_LABELS - 1)
				         return -2;
				*pao = ++label; //new label
			}
		}
		pbi = pai++; pbo = pao++; pdi = pci++; pdo = pco++;

		// remaining columns
		for(j = 1; j < width; j++)
		{
			if(*pai)
			{
				if(*pbi)
				{
					*pao = *pbo;
					if(*pci)				// resolving equivalences
						if( *pco != *pbo )
						{
							//if (originalb==-1)
							//	return -1;
							if( originalb = getLeaf(*pbo, equiv_array) )
							{
								//if (originalc = getLeaf(*pco, equiv_array))
								//	return -1;
								if( originalc = getLeaf(*pco, equiv_array) )
								{
									if( originalc != originalb)
                                        equiv_array[originalc-1] = originalb;
								}
								else  // c is leaf
								{
									if(*pco != originalb)
										equiv_array[*pco-1] = originalb;
								}
								*pco = originalb;
							}
							else // b is leaf
							{
								//originalc = getLeaf(*pco, equiv_array);
								//if (originalc==-1)
								//	return -1;
								if( originalc = getLeaf(*pco, equiv_array) )
								{
									if(originalc != *pbo) {
										equiv_array[originalc-1] = *pbo;
                                  }
								}
								else  // c is leaf
								{
									equiv_array[*pco-1] = *pbo;
								}
								*pco = *pbo;
							}
						}
				}
				else if(*pdi)
				{
					*pao = *pdo;
				}
				else if(*pci)
				{
					*pao = *pco;
				}
				else
				{
					//label+=4;
					if(label == MAX_LABELS-1)
				         return -3;
					*pao = ++label;  // new label
//					if(label > MAX_LABELS)
//						return -1;  //ran out of labels
				}
			}
			pai++; pbi++; pci++; pdi++; pao++; pbo++; pco++; pdo++;
		}
	}
	
	// fusing labels
	for( i = 1; i <= label; i++ )
	{
		minlabel = getLeaf(i, equiv_array);
		//if (minlabel==-1)
		//	return -1;
		if( minlabel )
		{
			equiv_array[i-1] = minlabel;
		}
	}
	
	// processing image
	for( i = 0; i < height; i++ )
	{
		for( j = 0; j < width; j++ )
		{
			if( temp[i*stride+j] )
			{
				
				if( equiv_array[ temp[i*stride+j] -1 ] )
				{
					temp[i*stride+j] = equiv_array[ temp[i*stride+j] -1 ];
				}
			}
		}
	}
	
	//convert from label image (integer) to output image (unsigned char)
	for( i = 0; i < height; i++ )
		for( j = 0; j < width; j++ )
			output[i*stride+j]  = (unsigned char)temp[i*stride+j];
			

	return label;						
}
