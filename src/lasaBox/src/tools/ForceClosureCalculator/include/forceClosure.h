//Ce fichier contient toutes les fonctions n�cessaires pour
//le test de la propri�t� de force-closure (FC) d'une prise
//quelconque � trois doigts. Il utilise des fonctions du programme
//qHull (www.qhull.org).
#ifndef FORCE_CLOSURE_H
#define FORCE_CLOSURE_H

#include "matrices.h"

//coefficient de frottement des contacts
//#define MU 0.8

void circleTable(float **sint,float **cost,const int n);
int testeForceClosure(Vecteur *posContact,Vecteur *normale,float mu,int nbContacts,int nbSegments,float* rayon_boule,float facteur_echelle);

#endif
