#include <stdio.h>
#include <math.h>
#include "forceClosure.h"

extern "C" {
#include "qhull_a.h"
}

#define Vfaible 1

//Cette fonction retourne dans sint et cost les coordonnées
//des points d'un cercle discretisé en n points.
//sint et cost ont chacun |n+1| éléments.
//Le signe de n indique la direction de parcours des points du cercle.
void circleTable(float **sint,float **cost,const int n)
{
    int i;
    /* Table size, the sign of n flips the circle direction */
    const int size = abs(n);
    /* Determine the angle between samples */
    const float angle = 2*M_PI/(float)n;
    /* Allocate memory for n samples, plus duplicate of first entry at the end */
    *sint = new float[size+1];
    *cost = new float[size+1];
    /* Bail out if memory allocation fails*/
    if (!(*sint) || !(*cost))
    {
        delete [] (*sint);
        delete [] (*cost);
        printf("circleTable: erreur d'allocation mémoire");
    }
    /* Compute cos and sin around the circle */
    for (i=0; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }
    /* Last sample is duplicate of the first */
    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];
}



//Cette fonction fait un test de force closure de la prise
//dont les positions (dans le repère de l'objet) et les normales
//des nbContacts sont passées en argument ainsi que les coefficients
//de frottemente de chaque contact. Les cônes de frottements
//sont discrétisés (linéarisés) en nbSegments.
//Elle retourne 1 si la rpise est FC, 0 sinon.
int testeForceClosure(Vecteur *posContact,Vecteur *normale,float mu,int nbContacts,int nbSegments,float* rayon_boule,float facteur_echelle)
{    
   int i,j,nbPoints=nbContacts*nbSegments;
   float norm;
   Vecteur u(3),v(3);
   coordT min_dist = 0; //la distance entre l'origine et la facette la p)lus proche
   
   float *sint,*cost;
   //float clock1,clock2,clock3;
  // clock1 = clock();
   
  /* float pas=2*M_PI/nbSegments;
    for (int k=0;k<nbSegments;k++)
    {
        sint[k] = sin(k*pas);
        cost[k] = cos(k*pas);
    } ; */
   circleTable(&sint,&cost,-nbSegments);
   Vecteur s(3),r(3);
   //tableau des "primitive contact wrenches":
   coordT *array = new coordT[6*nbPoints];
   if(array==NULL) printf("array=NULL");
   
   for(i=0;i<nbContacts;i++)
   {
     baseOrthonormale(normale[i],&u,&v);  
     for(j=0;j<nbSegments;j++)
     {  
       s=normale[i] + mu*cost[j]*u + mu*sint[j]*v;          
       s.normalisation();
       r=(posContact[i]*s)/facteur_echelle;
       //norm=sqrt(s(1)*s(1)+s(2)*s(2)+s(3)*s(3)+r(1)*r(1)+r(2)*r(2)+r(3)*r(3));
       array[6*(nbSegments*i+j)]=s.data[0];///norm;
       array[6*(nbSegments*i+j)+1]=s.data[1];//norm;
       array[6*(nbSegments*i+j)+2]=s.data[2];//norm;   
       array[6*(nbSegments*i+j)+3]=r.data[0];//norm;
       array[6*(nbSegments*i+j)+4]=r.data[1];//norm;
       array[6*(nbSegments*i+j)+5]=r.data[2];//norm;
     }
   }

   delete [] sint;
   delete [] cost;
   sint=NULL;
   cost=NULL;

   double minOffset;
   //qhull variables
   boolT ismalloc;
   int curlong, totlong, exitcode;
   char options[100];  
   facetT *facet;
   ismalloc = False; // True if qh_freeqhull should 'free(array)'
   //FILE *qhfp = NULL;

   options[0]='\0';

///////////////////////////

  FILE *qhfp = fopen("logfile.txt","w");
  
   if(!qhfp)
   {
	 fprintf(stderr,"Could not open qhull logfile!\n");
	 qh_init_A(NULL, stdout, stderr, 0, NULL);
   }
   else
   {  qh_init_A(NULL, qhfp, qhfp, 0, NULL); }
///////////////////////////////////////////////////
   qh_init_A(NULL, qhfp, qhfp, 0, NULL); 

   if ((exitcode = setjmp(qh errexit))) {

	delete [] array;
	qh NOerrexit= True;
	qh_freeqhull(!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);
	if(curlong || totlong)  	// optional 
	   printf("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
	if(qhfp) fclose(qhfp);
    return 0;
   }
   /*printf("hhhhh11111\n");
   printf("options= %s\n",options);
    printf("hhhhh3333\n");*/
   
   qh_initflags(options);
   //printf("hhhhh22222\n");
   qh_init_B(array,nbPoints,6,ismalloc); //nbPoints de dimension 6
   qh_qhull();
   qh_check_output();
   //qh_getarea(qh facet_list);
  // clock2 = clock();
  
  
   FORALLfacets 
   {
     if (facet->offset > 0) 
     { delete [] array; 
       array=NULL; 
       qh NOerrexit= True;
       qh_freeqhull(!qh_ALL);
       qh_memfreeshort (&curlong, &totlong);
       if (curlong || totlong)  	// optional 
       printf("qhull internal printf (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
    ///////////////
      // if(qhfp) fclose(qhfp);
    //return 0;
    if(qhfp) fclose(qhfp);
       return 0;   }
   }
    
////////////
//////////added by Sahar march 2008 for computing the quality of the grasp //////////////////
   min_dist=1000;//fabs(qh facet_list->offset);
   FORALLfacets 
   {
     ///added by Sahar //////////
     coordT a=facet->offset ;
     //a=fabs(a);
     if (-a < min_dist)
        min_dist = -a;
   }
   *rayon_boule = min_dist;         
   //printf("%f \n",min_dist);
  //qh_setdelaunay();
 
 /* coordT  pointB[6]={1,0,0,0,0,0}      ;
  boolT   isoutside  ;
  //boolT   bestoutside  ;
  realT   bestdist   ;
  facetT*  facetB    ;
  facetB= qh_findbestfacet (pointB, qh_ALL, &bestdist, &isoutside);
  printf("  %d %d %f\n" , qh_ALL,isoutside,bestdist);
  //facetB= qh_findbestfacet (pointB,bestoutside , &bestdist, &isoutside); */
  
////////////// end Sahar ///////////////////////////////////////////////////////////////

  delete [] array; 
  array = NULL; 
  qh NOerrexit= True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);
  if (curlong || totlong)  	// optional 
     printf("qhull internal printf (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
/////////////
/*if(qhfp) fclose(qhfp);
    return 0;*/
   
    //printf("temps_convexhull= %f\n",(clock2-clock1)/CLOCKS_PER_SEC);
    //printf("temps_test= %f\n",(clock3-clock2)/CLOCKS_PER_SEC);
  if(qhfp) fclose(qhfp);
  options[0]='\0';
  return 1;
}




