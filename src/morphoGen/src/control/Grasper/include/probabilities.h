#ifndef PROBAB_H
#define PROBAB_H
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include <math.h>

#include <yarp/sig/Matrix.h>

using namespace std;

/**
	Legacy class, providing methods for setting up the weights for each branching of the Posture Tree, for randomly selecting the branchings in the tree and for updating the weights after a (successful/unsuccessful) trial
*/

class Probab
{
public:

	/**
		Legacy enum listing the objects classes, for now only Flat or Tall. In future implementations should use the PoseGenerator one. This enum, for now, is kept for compatibily reasons.
	*/
    enum ObjectClass{
        Flat,
        Tall,
        ObjectClassNum
    };


	/**
		Legacy function for generating the weights for each tree branching. Each branching is a matrix.
		@param[in] weights_1 The object class/dominant hand branching, must be a matrix 2x3, it will be modified by the code
		@param[in] weights_2 The dominant hand/non-dominant hand branching, must be a matrix 3x4
		@param[in] weights_3 The dominant hand/finger configuration branching, must be a matrix 3x3
		@param[in] Nr an array containing the number of rows for each matrix, must be 3x1
		@param[in] Nc an array containing the number of columns for each matrix, must be 3x1
	*/
static inline void getWeights(double** &weights_1,double** &weights_2,double** &weights_3,int *Nr,int *Nc);

	/**
		Legacy function for selecting a posture from the matrixes. This corresponds to randomly select an element in the Posture Tree. The random number generator is assumed already initialized before running this function
		@param[in] VoteWr A matrix 2x3, representing the branch at object class/dominant hand level
		@param[in] VoteWrOtherHand A matrix 3x4, representing the branch at dominant hand/non-dominant hand level
		@param[in] VotePrimitive A matrix 3x3, representing the branch at dominant hand/finger configuration level
		@param[in] ObjClass a value represent the object class (may be only one of the enum values defined in this class)
		@param[in] orient1 an integer representing the orientation of the dominant hand (the value will be store here)
		@param[in] orient2 an integer representing the orientation of the non-dominant hand (the value will be store here)
		@param[in] handprimitive an integer representing the stretch of the fingers (the value will be store here)
	*/
static inline void GenerateWrOrientation(/*double ** */const yarp::sig::Matrix& VoteWr,/*double ** */const yarp::sig::Matrix& VoteWrOtherHand,/*double ** */const yarp::sig::Matrix& VotePrimitive,int ObjClass,int *orient1,int *orient2,int *handprimitive);

/**
		Legacy function for updating the matrix's values using a SOM-style learning. If the trial was (un)successful the values are updated accordingly
		@param[in] success an integer (0 or 1) indicating if the trial was successful or not
		@param[in] Node2 index of the matrix referring to the column to be updated (i.e. dominant hand for the object class/dominant hand pair)
		@param[in] Node1 index of the matrix referring to the row to be updated (i.e. object class for the object class/dominant hand pair)
		@param[in] weights the matrix to be updated
		@param[in] N1 the number of rows of the current matrix
		@param[in] N2 the number of columns of the current matrix
		@param[in] new_weights the result of the updated, it is a matrix of the same dimentions of "weights" (the value will be store here)
	*/
static inline void UpdateWeights(const int success,int Node2,int Node1,/*double ** */const yarp::sig::Matrix& weights,const int N1,const int N2,double** &new_weights);
};

//									VoteWr		  ,	   VoteWrOtherHand,	   VoteFinger
inline void Probab::getWeights(double** &weights_1,double** &weights_2,double** &weights_3,int *Nr,int *Nc){
	
	int i,j;
	
	if (weights_1) delete weights_1;
		
	weights_1 = new double* [Nr[0]];
	for (i = 0; i < Nr[0]; i++)
		weights_1[i] = new double[Nc[0]];
	
	
	if (weights_2) delete weights_2;
	
	weights_2 = new double* [Nr[1]];
	for (i = 0; i < Nr[1]; i++)
		weights_2[i] = new double[Nc[1]];


	if (weights_3) delete weights_3;
	
	weights_3 = new double* [Nr[2]];
	for (i = 0; i < Nr[2]; i++)
		weights_3[i] = new double[Nc[2]];
	

	weights_1[Flat][0]=0.33;        //flat object    : 0   \ dominant hand orientation - 0 degrees
	weights_1[Flat][1]=0.33;			// 73 dgerees
	weights_1[Flat][2]=0.33;			// -73 degrees
	
	weights_1[Tall][0]=0.33;          //tall object    : 1
	weights_1[Tall][1]=0.33;
	weights_1[Tall][2]=0.33;
	

	weights_2[0][0]=0.33;//1;          //dominant hand orientation \ non-dominant hand orientation - 0 degrees
	weights_2[0][1]=0.33;//0;			// 73 degrees
	weights_2[0][2]=0.33;//0;			// -73 degrees
	weights_2[0][3]=0;			//not used...

	weights_2[1][0]=0.33;//1;
	weights_2[1][1]=0.33;//0;
	weights_2[1][2]=0.33;//0;
	weights_2[1][3]=0;
	
	weights_2[2][0]=0.33;//0;
	weights_2[2][1]=0.33;//0;
	weights_2[2][2]=0.33;//1;
	weights_2[2][3]=0;
	
	weights_3[0][0]=0.5;//1:          //dominant hand orientation \ finger orientation	- 0 degrees stretch
	weights_3[0][1]=0.5;//0;			// 10 degrees stretch
	weights_3[0][2]=0;			// 0 degrees stretch

	weights_3[1][0]=0.5;
	weights_3[1][1]=0.5;
	weights_3[1][2]=0;

	weights_3[2][0]=0.5;
	weights_3[2][1]=0.5;
	weights_3[2][2]=0;


    
}

inline void Probab::GenerateWrOrientation(/*double ** */const yarp::sig::Matrix& VoteWr,/*double ** */const yarp::sig::Matrix& VoteWrOtherHand,/*double ** */const yarp::sig::Matrix& VotePrimitive,int ObjClass,int *orient1,int *orient2,int *handprimitive){
	int i,j;
	double Tvotes,RandNum;
	double dist[3],Probs[3];
	//Select the orientation of the dominant hand
	Tvotes = 0.0;
	for (j=0;j<3;j++){		//it was j<2
		Tvotes += VoteWr[ObjClass][j];
	}

	RandNum = ((double)rand()/(double)RAND_MAX);

	for (j=0;j<3;j++){		//it was j<2
		Probs[j] = VoteWr[ObjClass][j]/Tvotes;//calculate probability by normalizing the scores
		dist[j] = RandNum*Probs[j];
	}
	double max = -10;
	for(i=0;i<3;i++){		//it was i<2
		if (dist[i] > max){
			max = dist[i];
			*orient1 = i;
		}
	}
	//Now select the orientation of the other hand
	Tvotes = 0.0;
	for (j=0;j<4;j++){		//it was j<3
		Tvotes += VoteWrOtherHand[*orient1][j];
	}

	RandNum = ((double)rand()/(double)RAND_MAX);

	for (j=0;j<3;j++){		//it was VoteWr //shall we change it to j<4 ?
		Probs[j] = VoteWrOtherHand[ObjClass][j]/Tvotes;//calculate probability by normalizing the scores	[ObjClass][]
		dist[j] = RandNum*Probs[j];
	}
	max = -10;
	for(i=0;i<3;i++){		//it was i<3
		if (dist[i] > max){
			max = dist[i];
			*orient2 = i;
		}
	}
	//Now select the dominant hand primitive
	Tvotes = 0.0;
	for (j=0;j<3;j++){		//it was j<2
		Tvotes += VotePrimitive[*orient1][j];
	}
	

	RandNum = ((double)rand()/(double)RAND_MAX);

	for (j=0;j<3;j++){		//it was VoteWr     //shall we change it to j<3 ?
		Probs[j] = VotePrimitive[ObjClass][j]/Tvotes;//calculate probability by normalizing the scores		[ObjClass][]
		dist[j] = RandNum*Probs[j];

		
	}
	max = -10;
	for(i=0;i<3;i++){		//it was i<2
		
		if (dist[i] > max){
			max = dist[i];
			*handprimitive = i;
		}
	}

}

inline void Probab::UpdateWeights(const int success,int Node2,int Node1,/*double ** */const yarp::sig::Matrix& weights,const int N1,const int N2,double** &new_weights){
	//here, the weights are updated using SOM based learning
	int i,j;
	double mue=1; //learning rate
	double sigma = N2/2; //radius of the region
	double diff;       //distance between i and i*
		
	if (new_weights) delete new_weights;
	
	new_weights = new double* [N1];
	for (i = 0; i < N1; i++)
		new_weights[i] = new double[N2];
	
	for(i=0;i<N1;i++){
		for(j=0;j<N2;j++){
			new_weights[i][j] = weights[i][j];
		}
	}
	for(i=0;i<N2;i++){
			diff = (Node2 - i)*(Node2 - i);
			new_weights[Node1][i] = weights[Node1][i] + (success-0.5)*mue*(exp(-(diff/(sigma)))-0.5); //exp(-d*d/2*sigma*sigma)*(In[i]-weights[i][j]);
	}
	
}
#endif