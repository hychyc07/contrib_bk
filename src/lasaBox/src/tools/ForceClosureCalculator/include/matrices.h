/*Définition des classes Vecteur et Matrice et de différentes
fonctions mathématiques.*/

#ifndef MATRICES_H
#define MATRICES_H

//pour la fonction de génération de nombres
//aléatoires
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS2 1.2e-7
#define RNMX (1.0-EPS2)

#include <stdlib.h>
#include <math.h>
class Vecteur
{
  public:
    int dimension; //dimension du vecteur
    float *data;   //pointeur vers le contenu du vecteur
    
    Vecteur(); //construit par défaut un vecteur de dimension 0
    Vecteur(int n); //construit un vecteur de dimension n
    Vecteur(float x,float y); //constructeur pour un
                                      //vecteur 2D
    Vecteur(float x,float y,float z); //constructeur pour un
                                      //vecteur 3D
    Vecteur(const Vecteur &); //construit une copie du vecteur donné
    ~Vecteur();
    
    void initVecteur2D(float x,float y);
    void initVecteur3D(float x,float y,float z);
    void reinit(int n); //redimensionne le vecteur
    void raz(); //met tous les éléments du vecteur à 0
    Vecteur ajouteUn(); //ajoute un 1 à la fin du vecteur
    void qFromAngleAndAxis(float angle,float ax,float ay,float az);
    void AngleAndAxisFromQ(Vecteur *q);
    void qFromEuler(float alpha,float beta,float gamma);
    void qFromR(class Matrice *R);
    
    float operator () (int i); // récupère le i-ème
                                //élément du vecteur
    float elt(int i); // récupère le i-ème
                                //élément du vecteur                                
    void operator=(const Vecteur &);
    Vecteur operator* (const Vecteur &); //produit vectoriel
                               //(pour les vecteurs de dimension 3)                                
    Vecteur operator* (float a);
    Vecteur operator/ (float a);
    Vecteur operator+ (const Vecteur &);
    Vecteur operator- (const Vecteur &);  
    Vecteur operator - () const;
    const Vecteur& operator += ( const Vecteur& v );
    const Vecteur& operator -= ( const Vecteur& v );
    float norme(); //retourne la norme euclidienne du vecteur
    float norme1(); //retourne la norme 1 du vecteur
    void normalisation();  //normalise le vecteur
    float maximum(); //retourne le maximum des éléments du vecteur
    float minimum(); //retourne le minimum des éléments du vecteur
    int imaximum(); //retourne l'indice du maximum des éléments du vecteur
    int iminimum(); //retourne l'indice du minimum des éléments du vecteur   
    Vecteur abs(); //retourne un vecteur composé des valeurs 
                  //absolues des termes du vecteur auquel on 
                  //l'applique
    Vecteur nabla(Vecteur v); //opérateur nabla: multiplie terme
                       //à terme le vecteur v au vecteur appelant
                       //(retourne un vecteur de même dimension)
    //retourne un vecteur composé des termes du vecteur 
    //appelant compris entre les indices i1 et i2
    Vecteur sousVecteur(int i1,int i2); 
    void affiche();
};


class Matrice
{
  public:
    int nbLignes;
    int nbColonnes;
    float *data;
    Matrice();    //construit par défaut une matrice 0x0
    Matrice(int n,int m); //construit une matrice (n lignes,m colonnes)                     
    Matrice(const Matrice &);  //construit une copie 
                               //de la matrice donnée
    ~Matrice();
    void reinit(int n,int m); //redimensionne la matrice  
    void setIdentity();
    void setFromQuaternion(Vecteur *q);
    void setFromAngleAndAxis(float alpha,float ux,float uy,float uz);
    void setFromAngleAxisProduct(float ax,float ay,float az);
    void initMatriceTH(Vecteur q,float tx,float ty,float tz);
    void setRotationX(float angle);
    void setRotationY(float angle);
    void setRotationZ(float angle);
    void transpose();
    float determinant();
    void inverse();
    void normalisationParColonnes();
    Vecteur extractionColonne(int j);
    float operator () (int i,int j); // récupère le (i,j)-ème
                                     //élément de la matrice
    float elt(int i,int j);   // récupère le (i,j)-ème
                              //élément de la matrice                              
    void operator=(const Matrice &);
    Matrice operator* (const Matrice &);
    Matrice operator* (float a);
    Vecteur operator* (const Vecteur &v);
    Matrice operator+ (const Matrice &);
    Matrice operator- (const Matrice &);
    Matrice operator - () const;
    const Matrice& operator += ( const Matrice& v );
    const Matrice& operator -= ( const Matrice& v );
    void affiche();
};

Matrice transpose(Matrice M);
Matrice inverse(Matrice M);
Matrice inverseMatriceTH(Matrice M);
int eigen(Matrice *M,Vecteur *valp,Matrice *vecp);
int nullspace(Matrice M,Matrice *nullmat);
Vecteur vec(Matrice *A);       

float produitScalaire(Vecteur v1,Vecteur v2); 
Vecteur produitQuaternion(Vecteur qb,Vecteur qc);
Vecteur inverseQuaternion(Vecteur q);
float norme(Vecteur v);
float norme1(Vecteur v);
Vecteur abs(Vecteur v);

Vecteur operator* (float,Vecteur);
Matrice operator* (float,Matrice);

void *concateneTableaux(void *tab1,int n1,void *tab2,int n2,char type);
void *agranditTableau(void *tab1,int n1,int n,char type);

float random(float a, float b);
float ran1(long *idum);
void setIdum(long n);

inline float signe(float x){
  if( x > 0 )
    return 1;
  else
    return -1;
}

inline bool isNaN(float x){
  if( x!=x )
   return true;
  else 
   return false;
}

inline float max(float a,float b){
  if( a>b )
   return a;
  else 
   return b;
}

inline float min(float a,float b){
  if( a<b )
   return a;
  else 
   return b;
}

int solutionEquationTrigo(float a,float b,float c,float *x1,float *x2);
void baseOrthonormale(Vecteur u,Vecteur *v,Vecteur *w);
Vecteur projectionOrthPointSurPlan(Vecteur eq,Vecteur point);
Vecteur projectionOrthPointSurDroite(Vecteur *e1,Vecteur *e2,Vecteur *p);
void equationPlan(Vecteur normale,Vecteur point,Vecteur *param);
void equationPlanTriangle(float *point1,float *point2,float * point3,float *param);
bool testeIntersectionSphereTriangle(Vecteur a,Vecteur b,Vecteur c,float rayon,Vecteur centre);
bool testeIntersectionSphereBoite(float dimX,float dimY,float dimZ,float rayon,Vecteur *centre);

//Cette fonction calcule l'intersection entre la demi-droite
//dont l'origine et le vecteur directeur u sont passés en paramètres
//et le triangle(p1,p2,p3).
//Elle recopie dans *p le point d'intersection avec le triangle
//et dans *normale la normale à la surface du triangle.
//Elle retourne 0 s'il n'y a pas d'intersection, 1 sinon. 
inline int intersectionRayonTriangle(Vecteur p1,Vecteur p2,Vecteur p3,Vecteur origine,Vecteur u,Vecteur *p,Vecteur *normale)
{
  //L'équation du plan du triangle est a*x + b*y + c*z + d = 0
  //L'équation de la demi-droite: origine + alpha*u, avec alpha
  //un scalaire positif.
  //L'intersection entre le plan et la droite est
  // (-d-a*Ox-b*0y-c*0z)/(a*ux+b*uy+c*uz)
  //si a*ux+b*uy+c*uz!=0 sinon il n'y a pas d'intersection.
  //On teste ensuite si le point est dans le triangle.
  //Pour cela, on teste pour chacun des plans définis
  //par l'origine de la droite et deux des sommets du triangle
  //de quel côté se trouve le point d'intersection calculé
  //précédemment. S'il est du "bon" côté pour chacun des trois
  //plans, il appartient au triangle.
    Vecteur eqPlan(4),intersect(3),op(3);
    Vecteur v1(3),v2(3),v3(3),n(3);
    float a,a0,alpha,beta;
    equationPlanTriangle(p1.data,p2.data,p3.data,eqPlan.data);
    a=eqPlan(1)*u(1)+eqPlan(2)*u(2)+eqPlan(3)*u(3);
    if(a<0.001) { return 0; }
    else 
    { 
      a0=eqPlan(1)*origine(1)+eqPlan(2)*origine(2)+eqPlan(3)*origine(3);
    }
    normale->data[0]=eqPlan(1);
    normale->data[1]=eqPlan(2);    
    normale->data[2]=eqPlan(3);
    alpha=(-eqPlan(4)-a0)/a;
    intersect= origine + alpha*u;
    v1=p1-origine;
    v2=p2-origine;
    v3=p3-origine;
    op=intersect-origine;
    n=v2*v1;
    n.normalisation();
    a=-n(1)*origine(1)-n(2)*origine(2)-n(3)*origine(3);
    if(n(1)*op(1)+n(2)*op(2)+n(3)*op(3)>0)
      return 0;
    n=v1*v3;
    n.normalisation();
    a=-n(1)*origine(1)-n(2)*origine(2)-n(3)*origine(3);
    if(n(1)*op(1)+n(2)*op(2)+n(3)*op(3)>0)
      return 0;  
    n=v3*v2;
    n.normalisation();
    a=-n(1)*origine(1)-n(2)*origine(2)-n(3)*origine(3);
    if(n(1)*op(1)+n(2)*op(2)+n(3)*op(3)>0)
      return 0;  
    p->data[0]=intersect.data[0];
    p->data[1]=intersect.data[1];    
    p->data[2]=intersect.data[2];
    return 1;
}

//Cette fonction calcule les paramètres de l'équation ax+by+c=0
//de la droite passant par p1 et p2.
inline void equationDroite(float *a,float *b,float *c,float *p1,float *p2)
{
  if( fabs(p1[0]-p2[0])<0.0001 )
  {  *b=0;
     *a=1;
     *c=p1[0]; }
  else
  {  *b=1;
     *a=-(p2[1]-p1[1])/(p2[0]-p1[0]);
     *c= -(*a)*p1[0] - p1[1];   }
}

#endif

int myinverse(Matrice M);
