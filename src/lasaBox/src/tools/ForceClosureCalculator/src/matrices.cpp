#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrices.h"
#include "basis.h"
#include "vmblock.h"
#include "feigen.h"
//#include <fltk/error.h>


#define EPS 0.00000001
#define DEBUG
void warning(const char *string)
{
  printf("%s\n",string);
}

long Idum=22222; //pour le générateur de nombres aléatoires

//Constructeur par défaut: vecteur vide
Vecteur::Vecteur()
{
  dimension=0;
  data=NULL;
}

//Constructeur d'un vecteur de dimension n, rempli avec des zéros
Vecteur::Vecteur(int n)
{
  dimension=n;
  data=NULL;
  data=new float[n];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::Vecteur(int): erreur d'allocation mémoire"); 
     exit(0);
  }  
  #endif
  for(int i=0;i<n;i++)
  { data[i]=0; }
}

//Constructeur d'un vecteur 2D
Vecteur::Vecteur(float x,float y)
{
  dimension=2;
  data=NULL;
  data=new float[2];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::Vecteur(float x,float y): erreur d'allocation mémoire"); 
     exit(0);
  }  
  #endif
  data[0]=x;
  data[1]=y;
}

//Constructeur d'un vecteur 3D
Vecteur::Vecteur(float x,float y,float z)
{
  dimension=3;
  data=NULL;
  data=new float[3];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::Vecteur(float x,float y,float z): erreur d'allocation mémoire"); 
     exit(0);
  }  
  #endif  
  data[0]=x;
  data[1]=y;
  data[2]=z;
}


Vecteur::~Vecteur()
{
  if(data!=NULL) delete [] data;
} 

void Vecteur::initVecteur2D(float x,float y)
{
  if(data!=NULL)
  { 
    delete[] data; 
    data=NULL;
  }
  dimension=2;
  data=NULL;
  data=new float[2];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::initVecteur3D: erreur d'allocation mémoire"); 
     exit(0);
  }   
  #endif      
  data[0]=x;
  data[1]=y;
}  

void Vecteur::initVecteur3D(float x,float y,float z)
{
  if(data!=NULL)
  { 
    delete[] data; 
    data=NULL;
  }
  dimension=3;
  data=NULL;
  data=new float[3];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::initVecteur3D: erreur d'allocation mémoire"); 
     exit(0);
  }   
  #endif      
  data[0]=x;
  data[1]=y;
  data[2]=z;   
}  

//Fonction de redimensionnement d'un vecteur
void Vecteur::reinit(int n)
{   
  if(data!=NULL)
  { 
    if(dimension==n) return;           
    delete [] data; 
    data=NULL;
  }
  dimension=n;
  data=NULL;
  data=new float[n];
  for(int i=0;i<n;i++)
  { data[i]=0; }
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::reinit: erreur d'allocation mémoire"); 
     exit(0);
  }  
  #endif        
}   

void Vecteur::raz()
{
  for(int i=0;i<dimension;i++)
  {  data[i]=0; }
}


Vecteur Vecteur::ajouteUn()
{
  Vecteur v=Vecteur(dimension+1);
  #ifdef DEBUG
  if(data==NULL)
  { 
    warning("Vecteur::ajouteUn:entrée vide"); 
    exit(0);
  }  
  #endif
  for(int i=0;i<dimension;i++)
  {
    v.data[i]=data[i];      
  }
  v.data[dimension]=1;
  return v; 
}


//Remplit le vecteur avec le quaternion correspondant à la rotation
//d'angle "angle" (en rad) et d'axe (ax,ay,az)
void Vecteur::qFromAngleAndAxis(float angle,float ax,float ay,float az)
{
  if( dimension != 4 )
  {  reinit(4);  }
  float l = ax*ax + ay*ay + az*az;
  if (l > EPS ) {
    angle *= 0.5;
    data[0] = cos(angle);
    l = sin(angle) / sqrt(l);
    data[1] = ax*l;
    data[2] = ay*l;
    data[3] = az*l;
  }
  else {
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
  }
}

void Vecteur::AngleAndAxisFromQ(Vecteur *q)
{
  #ifdef DEBUG
  if( q->dimension!=4 )
  { 
    warning("Vecteur::AngleAndAxisFromQ: q n'est pas de dimension 4");
    exit(0);
  }   
  if( (q->norme()<1-EPS)||(q->norme()>1+EPS) )
  {
    warning("Vecteur::AngleAndAxisFromQ: q n'est pas un quaternion unité");
    q->normalisation();
  }    
  #endif
  reinit(4);
  float theta=2*acos(q->elt(1));
  float stheta2=sin(theta/2);
  data[0]=theta;
  if( theta< EPS )
  {
    data[1]=1;
    data[2]=0;
    data[3]=0;
  }
  else
  {
    data[1]=q->elt(2)/stheta2;
    data[2]=q->elt(3)/stheta2;
    data[3]=q->elt(4)/stheta2;

    float norm=sqrt(data[1]*data[1]+data[2]*data[2]+data[3]*data[3]);
    data[1]=data[1]/norm;
    data[2]=data[2]/norm;
    data[3]=data[3]/norm;
  }

}

void Vecteur::qFromEuler(float alpha,float beta,float gamma)
{
    float c1=cos(alpha/2);
    float s1=sin(alpha/2);
    float c2=cos(beta/2);
    float s2=sin(beta/2);
    float c3=cos(gamma/2);
    float s3=sin(gamma/2);

    float c1c2 = c1*c2;
    float s1s2 = s1*s2;

    data[0]=(c1c2*c3  + s1s2*s3);  
    data[1]=(c1c2*s3  - s1s2*c3);  
    data[2]=(c1*s2*c3 + s1*c2*s3); 
    data[3]=(s1*c2*c3 - c1*s2*s3);  
}

void Vecteur::qFromR(Matrice *R)
{
  #ifdef DEBUG
  if((R->nbLignes!=3)||(R->nbColonnes!=3)||(R->data==NULL))
  {
    warning("Vecteur::qFromR: mauvaise entrée");
    exit(0);
  }
  #endif
  if(dimension!=4)
  { reinit(4); }
  
  float tr,s;
  tr = R->data[0] + R->data[4] + R->data[8];
  if (tr >= 0)
 {
    s = sqrt(tr + 1);
    data[0] = 0.5*s;
    s = 0.5/s;
    data[1] = (R->elt(3,2) - R->elt(2,3))*s;
    data[2] = (R->elt(1,3) - R->elt(3,1))*s;
    data[3] = (R->elt(2,1) - R->elt(1,2))*s;
    this->normalisation();
  }
  else {
    // find the largest diagonal element and jump to the appropriate case
    if ( R->elt(2,2) > R->elt(1,1)) 
    {
      if ( R->elt(3,3) > R->elt(2,2)) { goto case_2; }
      goto case_1;
    }
    if ( R->elt(3,3) > R->elt(1,1)) { goto case_2; }
    goto case_0;

    case_0:
     s = sqrt(( R->elt(1,1) - (R->elt(2,2) + R->elt(3,3))) + 1);
     data[1] = 0.5*s;
     s = 0.5/s;
     data[2] = ( R->elt(1,2) + R->elt(2,1)) * s;
     data[3] = ( R->elt(3,1) + R->elt(1,3)) * s;
     data[0] = ( R->elt(3,2) - R->elt(2,3)) * s;
     this->normalisation();
     return;

    case_1:
     s = sqrt((R->elt(2,2) - (R->elt(3,3) + R->elt(1,1))) + 1);
     data[2] = 0.5*s;
     s = 0.5/s;
     data[3] = (R->elt(2,3) + R->elt(3,2)) * s;
     data[1] = (R->elt(1,2) + R->elt(2,1)) * s;
     data[0] = (R->elt(1,3) - R->elt(3,1)) * s;
     this->normalisation();    
     return;

    case_2:
     s = sqrt((R->elt(3,3) - (R->elt(1,1) + R->elt(2,2))) + 1);
     data[3] = 0.5*s;
     s = 0.5/s;
     data[1] = (R->elt(3,1) + R->elt(1,3)) * s;
     data[2] = (R->elt(2,3) + R->elt(3,2)) * s;
     data[0] = (R->elt(2,1) - R->elt(1,2)) * s;
     this->normalisation();    
     return;
  }
 
}


Vecteur::Vecteur (const Vecteur &v)
{

  data=NULL;
  dimension = v.dimension;
  data = new float[dimension];
  #ifdef DEBUG
  if(data==NULL)
  { 
     warning("Vecteur::Vecteur (const Vecteur &): erreur d'allocation mémoire"); 
     exit(0);
  }  
  #endif   
  for(int i=0;i<dimension;i++)
  { data[i]=v.data[i];  }
}


inline float Vecteur::operator () (int i)
{
  #ifdef DEBUG
  if(i<1)
  {
      warning("Vecteur::operator(): indice trop petit"); 
      return 0;
  }
  if(i>dimension)  
  {
      warning("Vecteur::operator(): indice trop grand"); 
      return 0;
  }
  #endif
  return data[i-1];
}

inline float Vecteur::elt(int i)
{
  #ifdef DEBUG
  if(i<1)
  {
      warning("Vecteur::elt(int,int): indice trop petit"); 
      return 0;
  }
  if(i>dimension)  
  {
      warning("Vecteur::elt(int,int): indice trop grand"); 
      return 0;
  }
  #endif
  return data[i-1];
}

void Vecteur::operator=(const Vecteur &v)
{
   dimension=v.dimension;
   if(data!=NULL)
     delete[] data;
   data=NULL;
   data=new float[dimension];
   for(int i=0;i<dimension;i++)
   {  data[i]=v.data[i];   }
}

Vecteur Vecteur::operator * (const Vecteur &v)
{
   #ifdef DEBUG
   if( (dimension!=3)||(v.dimension!=3) )
   {  
     warning("Vecteur::operator *:les vecteurs ne sont pas de dimension 3");
     return Vecteur();  
   }
   #endif
   Vecteur res=Vecteur(3);
   res.data[0] = (data[1])*(v.data[2]) - (data[2])*(v.data[1]);
   res.data[1] = (data[2])*(v.data[0]) - (data[0])*(v.data[2]);
   res.data[2] = (data[0])*(v.data[1]) - (data[1])*(v.data[0]);
   return res;
}

Vecteur operator* (float a,Vecteur v)
{
  Vecteur res(v.dimension);
  #ifdef DEBUG
  if(v.data==NULL)
  { 
    warning("operator*(float,Vecteur): vecteur vide");
    exit(0);
  }
  #endif
  for(int i=0;i<v.dimension;i++)
  { res.data[i]=a*v.data[i]; }
  return res;
  
}


Vecteur Vecteur::operator * (float a)
{
  #ifdef DEBUG      
  if(data==NULL)
  {  warning("Vecteur::operator *:vecteur vide");
     exit(0);   }
  #endif
  Vecteur res=Vecteur(dimension);
  for(int i=0;i<dimension;i++)
  {  res.data[i]=a*data[i];  }
  return res;
}

Vecteur Vecteur::operator / (float a)
{
  #ifdef DEBUG
  if(data==NULL)
  {  
    warning("Vecteur::operator *:vecteur vide");
    exit(0); 
  }
  #endif
  Vecteur res=Vecteur(dimension);
  for(int i=0;i<dimension;i++)
  {  res.data[i]=data[i]/a;  }
  return res;
}

Vecteur Vecteur:: operator + (const Vecteur &v)
{    
   #ifdef DEBUG
   if( dimension!=v.dimension )
   {
     warning("Vecteur::operator+: dimensions des vecteurs non compatibles");
     //exit(0);
   }  
   #endif
   Vecteur res=Vecteur(dimension);
   for(int i=0;i<dimension;i++)
   {
     res.data[i]=data[i]+v.data[i];     
   }  
   return res;
}

Vecteur Vecteur:: operator - (const Vecteur &v)
{    
   #ifdef DEBUG
   if( dimension!=v.dimension )
   {
     warning("Vecteur::operator-: dimensions des vecteurs non compatibles");
     //exit(0);
   }  
   #endif
   Vecteur res=Vecteur(dimension);
   for(int i=0;i<dimension;i++)
   {
     res.data[i]=data[i]-v.data[i];     
   }  
   return res;
}


Vecteur Vecteur::operator - () const
{
   Vecteur res=Vecteur(dimension);
   for(int i=0;i<dimension;i++)
   {
     res.data[i]= -data[i];     
   }  
   return res;
}


const Vecteur& Vecteur::operator += ( const Vecteur& v )
{
   #ifdef DEBUG
   if( dimension!=v.dimension )
   {
     warning("Vecteur::operator +=: dimensions des vecteurs non compatibles");
   }  
   #endif
   for(int i=0;i<dimension;i++)
   {
     data[i]+= v.data[i];     
   }  
}


const Vecteur& Vecteur::operator -= ( const Vecteur& v )
{
   #ifdef DEBUG
   if( dimension!=v.dimension )
   {
     warning("Vecteur::operator -=: dimensions des vecteurs non compatibles");
   }  
   #endif
   for(int i=0;i<dimension;i++)
   {
     data[i]-= v.data[i];     
   }  
}



float Vecteur::norme()
{
  #ifdef DEBUG    
  if(data==NULL)
   { warning("Vecteur::norme: vecteur vide");
     exit(0);  }
  #endif
  float norm=0;
  for(int i=0;i<dimension;i++)
   {
     norm+= data[i]*data[i];
   }
  return sqrt(norm);    
}

float Vecteur::norme1()
{
  #ifdef DEBUG
  if(data==NULL)
   { warning("Vecteur::norme: vecteur vide"); 
     exit(0);  }
  #endif
  float norm=0;
  for(int i=0;i<dimension;i++)
   {
     norm+= fabs(data[i]);
   }
  return norm;    
}

void Vecteur::normalisation()
{
   #ifdef DEBUG
   if(data==NULL)
   { warning("Vecteur::normalisation: vecteur vide");
     exit(0);   }
   #endif
   float norm=0;
   for(int i=0;i<dimension;i++)
   {
     norm+= data[i]*data[i];
   } 
   if(norm!=0)
   {  
     norm=sqrt(norm);        
     for(int i=0;i<dimension;i++)
      {
        data[i]=data[i]/norm;
      } 
   }
   return;   
}

float Vecteur::maximum()
{
  #ifdef DEBUG     
  if(data==NULL)
  {
    warning("Vecteur::maximum(): vecteur vide");
    return 0;
  }
  #endif
  float max=data[0];
  for(int i=1;i<dimension;i++)
  {
    if(data[i]>max)
    { max=data[i]; }
  }
  return max;
}

float Vecteur::minimum()
{
  #ifdef DEBUG  
  if(data==NULL)
  {
   warning("Vecteur::minimum(): vecteur vide");
   return 0;
  }
  #endif
  float min=data[0];
  for(int i=1;i<dimension;i++)
  {
    if(data[i]<min)
    { min=data[i]; }
  } 
  return min;
}

int Vecteur::imaximum()
{
  #ifdef DEBUG     
  if(data==NULL)
  {
    warning("Vecteur::maximum(): vecteur vide");
    return 0;
  }
  #endif
  float max=0;
  int imax=0;
  for(int i=0;i<dimension;i++)
  {
    if(data[i]>max)
    { max=data[i];
      imax=i;
    }
  }
  return imax+1;
}

int Vecteur::iminimum()
{
  #ifdef DEBUG  
  if(data==NULL)
  {
   warning("Vecteur::minimum(): vecteur vide");
   return 0;
  }
  #endif
  float min=0;
  int imin=0;
  for(int i=0;i<dimension;i++)
  {
    if(data[i]<min)
    { min=data[i];
      imin=i;
    }
  } 
  return imin+1;
}

Vecteur Vecteur::nabla(Vecteur v)
{
  Vecteur res(dimension);   
  #ifdef DEBUG     
  if(v.dimension!=dimension)
  { 
    warning("Vecteur::nabla: dimensions incompatibles");
    exit(0);
  }
  #endif
  for(int i=0;i<dimension;i++)
  {
    res.data[i]=data[i]*v.data[i];
  }
  return res;
        
}

//Cette méthode retourne un vecteur composé des termes
//du vecteur appelant compris entre les indices i1 et i2.
Vecteur Vecteur::sousVecteur(int i1,int i2)
{
  #ifdef DEBUG  
  if( (i1<1)||(i1>dimension)||(i2<1)||(i2>dimension)||(i1>=i2) )
  {  warning("Vecteur::sousVecteur(int,int):mauvais indice(s)");      }
  #endif
  Vecteur res(i2-i1+1);
  for(int i=0;i<i2-i1+1;i++)
  {  res.data[i]=data[i+(i1-1)];  }
  return res;
}


Matrice::Matrice()
{
  nbLignes=0;
  nbColonnes=0;
  data=NULL;
}
 
Matrice::Matrice(int n,int m)
{
  nbLignes=n;
  nbColonnes=m;
  data=NULL;
  data= new float[n*m];
  if(data==NULL)
  { 
     warning("Matrice::Matrice(int,int): erreur d'allocation mémoire"); 
     exit(0);
  }
  //On remplit la matrice avec des zéros:
  for(int i=0;i<n*m;i++)
  {  data[i]=0;  }
}    

Matrice::Matrice(const Matrice &m)
{
  nbLignes = m.nbLignes;
  nbColonnes = m.nbColonnes;
  data = new float[nbLignes*nbColonnes];
  for(int i=0;i<nbLignes;i++)
  { 
    for(int j=0;j<nbColonnes;j++)
      {  data[j+i*nbColonnes]=m.data[j+i*nbColonnes];  }
  }
}

Matrice::~Matrice()
{
  if(data!=NULL) delete [] data; 
} 
       
void Matrice::reinit(int n,int m)
{
  if(data!=NULL)
  {
    if( (nbLignes==n)&&(nbColonnes==m) ) return;
      delete [] data;   
  }
  nbLignes=n;
  nbColonnes=m;
  data=NULL;
  data= new float[n*m];
  if(data==NULL)
   { 
     warning("Matrice::reinit: erreur d'allocation mémoire"); 
     exit(0);
   }      
  for(int i=0;i<n*m;i++)
  { data[i]=0;  }      
}      
                   
           
void Matrice::setIdentity()
{
    if(nbLignes!=nbColonnes)
    { 
      warning("Matrice::setIdentity: erreur: matrice non carré");
      return;
    }
    for(int i=0;i<nbLignes;i++)
	{
		for(int j=0;j<nbColonnes;j++)
		{
			if( i==j )
			   data[j+nbColonnes*i]=1;
			else
	           data[j+nbColonnes*i]=0;
		}
	}
}

void Matrice::setFromQuaternion(Vecteur *q)
{
  #ifdef DEBUG  
  if( q->dimension!=4 )
  { 
    warning("Matrice::setFromQuaternion(Vecteur *): q n'est pas de dimension 4");
    exit(0);
  }   
  #endif
  if( (q->norme()<1-EPS)||(q->norme()>1+EPS) )
  {
    //warning("Matrice::setFromQuaternion: q n'est pas un quaternion unité");
    q->normalisation();
  }
  if( (nbLignes!=3)||(nbColonnes!=3) )
    {  reinit(3,3);  }   
  // q = (s,vx,vy,vz)
  float qq1 = 2*q->elt(2)*q->elt(2);
  float qq2 = 2*q->elt(3)*q->elt(3);
  float qq3 = 2*q->elt(4)*q->elt(4);
  data[0] = 1 - qq2 - qq3;
  data[1] = 2*(q->elt(2)*q->elt(3) - q->elt(1)*q->elt(4));
  data[2] = 2*(q->elt(2)*q->elt(4) + q->elt(1)*q->elt(3));
  data[3] = 2*(q->elt(2)*q->elt(3) + q->elt(1)*q->elt(4));
  data[4] = 1 - qq1 - qq3;
  data[5] = 2*(q->elt(3)*q->elt(4) - q->elt(1)*q->elt(2));
  data[6] = 2*(q->elt(2)*q->elt(4) - q->elt(1)*q->elt(3));
  data[7] = 2*(q->elt(3)*q->elt(4) + q->elt(1)*q->elt(2));
  data[8] = 1 - qq1 - qq2;
}

void Matrice::setFromAngleAndAxis(float alpha,float ux,float uy,float uz)
{
   Vecteur q(4);
   q.qFromAngleAndAxis(alpha,ux,uy,uz);
   this->setFromQuaternion(&q);
}


void Matrice::setFromAngleAxisProduct(float ax,float ay,float az)
{
  Vecteur r(4);    
  float d=sqrt(ax*ax + ay*ay + az*az);
  if(d>0.0001)
  {  r.data[0]=fmod(d,2*M_PI);
     r.data[1]=ax/d;      
     r.data[2]=ay/d;  
     r.data[3]=az/d;  } 
  else
  {  r.data[0]=0;   
     r.data[1]=0;      
     r.data[2]=0;  
     r.data[3]=1;  } 
  this->setFromAngleAndAxis(r.data[0],r.data[1],r.data[2],r.data[3]);
}

void Matrice::initMatriceTH(Vecteur q,float tx,float ty,float tz)
{
  if( (nbLignes!=4)||(nbColonnes!=4) )
  {  reinit(4,4);  }   
  
  Matrice R(3,3);
  R.setFromQuaternion(&q);
  
  data[0]=R(1,1);  data[1]=R(1,2);  data[2]=R(1,3);   data[3]=tx;
  data[4]=R(2,1);  data[5]=R(2,2);  data[6]=R(2,3);   data[7]=ty;
  data[8]=R(3,1);  data[9]=R(3,2);  data[10]=R(3,3);  data[11]=tz;
  data[12]=0;      data[13]=0;      data[14]=0;       data[15]=1;
}

void Matrice::setRotationX(float angle)
{
  if( (nbLignes!=3)||(nbColonnes!=3) )
  {  reinit(3,3);  }   
  data[0]=1;  data[1]=0;           data[2]=0; 
  data[3]=0;  data[4]=cos(angle);  data[5]=-sin(angle); 
  data[6]=0;  data[7]=sin(angle);  data[8]=cos(angle); 

}

void Matrice::setRotationY(float angle)
{
  if( (nbLignes!=3)||(nbColonnes!=3) )
  {  reinit(3,3);  }   
  data[0]=cos(angle);   data[1]=0;  data[2]=sin(angle); 
  data[3]=0;            data[4]=1;  data[5]=0; 
  data[6]=-sin(angle);  data[7]=0;  data[8]=cos(angle); 

}

void Matrice::setRotationZ(float angle)
{
  if( (nbLignes!=3)||(nbColonnes!=3) )
  {  reinit(3,3);  }   
  data[0]=cos(angle);  data[1]=-sin(angle); data[2]=0; 
  data[3]=sin(angle);  data[4]=cos(angle);  data[5]=0; 
  data[6]=0;           data[7]=0;           data[8]=1; 
}

//Cette fonction remplace la matrice qui l'appelle par 
//sa transposée.
void Matrice::transpose()
{
  Matrice tmp=Matrice(nbLignes,nbColonnes);
  for(int i=0;i<nbLignes;i++)
  {
    for(int j=0;j<nbColonnes;j++)
    {
      tmp.data[j+i*nbColonnes]=data[j+i*nbColonnes]; 
    }
  }
  reinit(nbColonnes,nbLignes);
  for(int i=0;i<nbLignes;i++)
  {
    for(int j=0;j<nbColonnes;j++)
    {
      data[j+i*nbColonnes]=tmp.data[i+j*nbLignes]; 
    }
  }
}

//Cette fonction retourne la transposée de la matrice M.
Matrice transpose(Matrice M)
{
  Matrice res= Matrice(M.nbColonnes,M.nbLignes);
  for(int i=0;i<M.nbColonnes;i++)
  {
    for(int j=0;j<M.nbLignes;j++)
    {
      res.data[j+i*M.nbLignes]=M.data[i+j*M.nbColonnes]; 
    }
  }
   return res;
}


float Matrice::determinant()   
{
  #ifdef DEBUG  
  if(nbLignes!=nbColonnes)
  { warning("Matrice::determinant(): matrice non carrée");
    return 0;  }
  #endif
  float sum=0;
  if(nbLignes==2)
  {  
	sum = data[0]*data[3] - data[2]*data[1];
	return sum;
  }
  int m=nbLignes-1;
  Matrice M(m,m);
  for(int p=0;p<nbLignes;p++)
  {
    int i1=0;      
    for(int i=0;i<nbLignes;i++)
    {    
      if(i==p)
        continue;
      for(int j=1;j<nbColonnes;j++)
      {  
        M.data[j-1+m*i1]=data[j + nbColonnes*i];  
      }
      i1++;
    }
    sum = sum + data[nbColonnes*p]*pow(-1,p)*M.determinant();
 }
 return sum;
}

//Fonction de calcul de l'inverse matricielle (utilise la méthode de Jordan avec recherche 
//de pivot maximum).
//Elle remplace la matrice qui l'appelle
//par son inverse.
void Matrice::inverse()
{
   #ifdef DEBUG  
   if(nbLignes!=nbColonnes)
   {
      warning("inverseMatrice: erreur matrice non carrée\n");
      exit(0);
   }   
   #endif
   int dim=nbLignes;                                                       
   int i,j,k,l,err;
   int nmax=dim+1,n2max=2*nmax-1;
   float a[nmax][nmax],b[nmax][nmax];
   float max,pivot,coef;
   float t[nmax][n2max];
   for(i=0;i<dim;i++)
	{  a[i][0]=0; a[0][i]=0; }
   for(i=1;i<=dim;i++)
	{  for(j=1;j<=dim;j++)
		{  a[i][j]=data[j-1+dim*(i-1)];
		}
	}
   for(i=1;i<=dim;i++)
	{
	  for(j=1;j<=dim;j++)
		{
	     t[i][j]=a[i][j];
		 if(i==j) 
			t[i][j+dim]=1.0;
		 else 
			t[i][j+dim]=0.0;
		}
   }
   err=1;
   k=1;
   while (err==1 && k<=dim)
   {
		max=fabs(t[k][k]);
		l=k;
		for(i=k+1;i<=dim;i++)
		{
			if(max<fabs(t[i][k]))
			{
			 max=fabs(t[i][k]);
			 l=i;
			} 
		}
	    if(max!=0)
		{
		  for(j=k;j<=2*dim;j++)
			{
			  max=t[k][j];
			  t[k][j]=t[l][j];
			  t[l][j]=max;
			}
		  pivot=t[k][k];
		  for(j=k+1;j<=2*dim;j++)
			{ t[k][j] /= pivot; }
		  for(i=1;i<=dim;i++)
			{ if(i!=k)
				{
				  coef=t[i][k];
				  for(j=k+1;j<=2*dim;j++)
				  {	 t[i][j] -= coef*t[k][j]; }
				}
			}
		}
		else
		{ err=0;}
        k++;
   }
   for(i=1;i<=dim;i++)
	{ for(j=1;j<=dim;j++) { b[i][j]=t[i][j+dim]; } 
	}

   if(err==0) 
   { 
	   warning("inverseMatrice: erreur: matrice non inversible\n");
       for(i=0;i<dim;i++)
			{  for(j=0;j<dim;j++)
				{ data[j+i*dim]=0; }
			}
	   return;
   }
   for(i=1;i<=dim;i++)
   {  for(j=1;j<=dim;j++)
		{ data[j-1+dim*(i-1)]=b[i][j];
		}
   }

}



//Fonction de calcul de l'inverse matricielle (utilise la méthode
//de Jordan avec recherche de pivot maximum).
//Elle retourne l'inverse de la matrice M.
Matrice inverse(Matrice M)
{
   #ifdef DEBUG  
   if(M.nbLignes!=M.nbColonnes)
   {
      warning("inverse: erreur matrice non carrée\n");
      exit(0);
   }   
   #endif
   int dim=M.nbLignes;                                                       
   int i,j,k,l,err;
   int nmax=dim+1,n2max=2*nmax-1;
   float a[nmax][nmax],b[nmax][nmax];
   float max,pivot,coef;
   float t[nmax][n2max];
   for(i=0;i<dim;i++)
	{  a[i][0]=0; a[0][i]=0; }
   for(i=1;i<=dim;i++)
	{  for(j=1;j<=dim;j++)
		{  a[i][j]=M.data[j-1+dim*(i-1)];
		}
	}

   for(i=1;i<=dim;i++)
	{
	  for(j=1;j<=dim;j++)
		{
	     t[i][j]=a[i][j];
		 if(i==j) 
			t[i][j+dim]=1.0;
		 else 
			t[i][j+dim]=0.0;
		}
   }
   err=1;
   k=1;
   while (err==1 && k<=dim)
   {
		max=fabs(t[k][k]);
		l=k;
		for(i=k+1;i<=dim;i++)
		{
			if(max<fabs(t[i][k]))
			{
			 max=fabs(t[i][k]);
			 l=i;
			} 
		}
	    if(max!=0)
		{
		  for(j=k;j<=2*dim;j++)
			{
			  max=t[k][j];
			  t[k][j]=t[l][j];
			  t[l][j]=max;
			}
		  pivot=t[k][k];
		  for(j=k+1;j<=2*dim;j++)
			{ t[k][j] /= pivot; }
		  for(i=1;i<=dim;i++)
			{ if(i!=k)
				{
				  coef=t[i][k];
				  for(j=k+1;j<=2*dim;j++)
				  {	 t[i][j] -= coef*t[k][j]; }
				}
			}
		}
		else
		{ err=0;}
        k++;
   }
   for(i=1;i<=dim;i++)
	{ for(j=1;j<=dim;j++) { b[i][j]=t[i][j+dim]; } 
	}
   
   Matrice res=Matrice(M.nbLignes,M.nbLignes);
   
   if(err==0) 
   { 
	   warning("inverse: erreur: matrice non inversible\n");
       for(i=0;i<dim;i++)
			{  for(j=0;j<dim;j++)
				{ res.data[j+i*dim]=0; }
			}
	   return res;
   }
   
   for(i=1;i<=dim;i++)
   {  for(j=1;j<=dim;j++)
		{ res.data[j-1+dim*(i-1)]=b[i][j];
		}
   }
  
  return res;
}

//Cette fonction retourne l'inverse d'une matrice 4x4
//qui doit être une matrice de transformation homogène, ce qui
//n'est pas vérifié par la fonction.
Matrice inverseMatriceTH(Matrice M)
{
  Matrice res(4,4);
  #ifdef DEBUG  
  if( (M.nbLignes!=4)||(M.nbColonnes!=4) )
  { warning("inverseMatriceTH: mauvaise entrée");
    return res; 
  }
  #endif
  Matrice R(3,3);
  R.data[0]=M(1,1);   R.data[1]=M(1,2);   R.data[2]=M(1,3);
  R.data[3]=M(2,1);   R.data[4]=M(2,2);   R.data[5]=M(2,3);
  R.data[6]=M(3,1);   R.data[7]=M(3,2);   R.data[8]=M(3,3); 
  Vecteur d(M(1,4),M(2,4),M(3,4));
  
  res.data[0]=R(1,1); res.data[1]=R(2,1); res.data[2]=R(3,1); 
  res.data[3]=-R(1,1)*d(1)-R(2,1)*d(2)-R(3,1)*d(3);
  res.data[4]=R(1,2); res.data[5]=R(2,2); res.data[6]=R(3,2); 
  res.data[7]=-R(1,2)*d(1)-R(2,2)*d(2)-R(3,2)*d(3);
  res.data[8]=R(1,3); res.data[9]=R(2,3); res.data[10]=R(3,3); 
  res.data[11]=-R(1,3)*d(1)-R(2,3)*d(2)-R(3,3)*d(3);
  res.data[12]=0; res.data[13]=0; res.data[14]=0; res.data[15]=1;
  
  return res;  
}

//Normalisation de chaque colonne de la matrice.
void Matrice::normalisationParColonnes()
{
   int i,j;
   float norm;
   for(j=0;j<nbColonnes;j++)
   {
      norm=0;
      for(i=0;i<nbLignes;i++)
      {
         norm+=pow(data[j+i*nbColonnes],2);
      }       
      norm=sqrt(norm);   
      if(norm>EPS)            
      {
         for(i=0;i<nbLignes;i++)
         {
           data[j+i*nbColonnes]=data[j+i*nbColonnes]/norm;
         }   
      }    
   }     
}


Vecteur Matrice::extractionColonne(int j)
{
  #ifdef DEBUG 
  if( (nbLignes==0) || (nbColonnes==0) || (data==NULL) )
  {
      warning("Matrice::extractionColonne(int): matrice vide"); 
      exit(0);
  }  
  if( (j<1) || (j>nbColonnes) ) 
  {
      warning("Matrice::extractionColonne(int): indice invalide"); 
      exit(0);
  }
  #endif   
  Vecteur res(nbLignes);
  for(int i=0;i<nbLignes;i++)
  {  res.data[i]=data[ j-1 + i*nbColonnes];  }
  return res; 
}


inline float Matrice::operator () (int i,int j)
{
  #ifdef DEBUG    
  if( (i<1) || (i>nbLignes) || (j<1) || (j>nbColonnes) ) 
  {
      warning("Matrice::operator(): indice invalide"); 
      exit(0);
  }
  #endif
  return data[(j-1)+nbColonnes*(i-1)];
}

inline float Matrice::elt(int i,int j)
{
  #ifdef DEBUG    
  if( (i<1) || (i>nbLignes) || (j<1) || (j>nbColonnes) ) 
  {
      warning("Matrice::elt(int,int): indice invalide"); 
      exit(0);
  }
  #endif
  return data[(j-1)+nbColonnes*(i-1)];
}

void Matrice::operator=(const Matrice &m)
{
  nbLignes = m.nbLignes;
  nbColonnes = m.nbColonnes;
  if(data!=NULL) delete[] data;
  data = new float[nbLignes*nbColonnes];
  for(int i=0;i<nbLignes;i++)
  { 
    for(int j=0;j<nbColonnes;j++)
      {  data[j+i*nbColonnes]=m.data[j+i*nbColonnes];  }
  }
}




Matrice Matrice::operator * (const Matrice &M)
{
   #ifdef DEBUG     
   if(nbColonnes!=M.nbLignes)
   {
     warning("Matrice::operator*(Matrice): dimensions non compatibles");  
     exit(0);
   }
   #endif
   Matrice Res=Matrice(nbLignes,M.nbColonnes); 
   float sum;
   for(int i=0;i<nbLignes;i++)
   {
     for(int j=0;j<M.nbColonnes;j++)
	  {
        sum=0;
	    for(int k=0;k<nbColonnes;k++)
	    { sum=sum + data[k+i*nbColonnes]*M.data[j+k*M.nbColonnes]; }
	      Res.data[j+i*Res.nbColonnes]=sum;
	 }
 }
 
 return Res;
} 


Matrice Matrice::operator * (float a)
{
   #ifdef DEBUG     
   if(data==NULL)
   { 
     warning("Matrice::operator*(float): matrice vide");
     exit(0);
   }
   #endif
   Matrice res=Matrice(nbLignes,nbColonnes); 
   for(int i=0;i<nbLignes;i++)
   {
     for(int j=0;j<nbColonnes;j++)
	  {
         res.data[j+i*nbColonnes]=a*data[j+i*nbColonnes];
	  }
   }
 
 return res;
} 

Vecteur Matrice::operator * (const Vecteur &v)
{
   #ifdef DEBUG  
   if( (data==NULL)|| (v.data==NULL) )
   {  warning("Matrice::operator*(Vecteur): entrées vides");
      exit(0); }
   if( v.dimension!=nbColonnes )
   {  warning("Matrice::operator*(Vecteur): vecteur de mauvaise dimension");
      exit(0); }
   #endif
   Vecteur res=Vecteur(nbLignes); 
   float sum;
   for(int i=0;i<nbLignes;i++)
   {
     sum=0;
     for(int j=0;j<nbColonnes;j++)
	 {
       sum+=data[i*nbColonnes+j]*v.data[j];
	 }
	 res.data[i]=sum;
   }
 
 return res;
} 

Matrice Matrice:: operator + (const Matrice &M)
{    
   #ifdef DEBUG  
   if( (nbLignes!=M.nbLignes)||(nbColonnes!=M.nbColonnes) )
   {
     warning("Matrice::operator+: dimensions non compatibles");
     exit(0);  
   }  
   #endif
   Matrice res=Matrice(nbLignes,nbColonnes);
   for(int i=0;i<nbLignes;i++)
   {
      for(int j=0;j<nbColonnes;j++)
      {
        res.data[j+i*nbColonnes]=data[j+i*nbColonnes]+M.data[j+i*nbColonnes];   
      }
   }  
   return res;
}

Matrice Matrice::operator - (const Matrice &M)
{    
   #ifdef DEBUG  
   if( (nbLignes!=M.nbLignes)||(nbColonnes!=M.nbColonnes) )
   {
     warning("Matrice::operator+: dimensions non compatibles");
     return Matrice();  
   }  
   #endif
   Matrice res=Matrice(nbLignes,nbColonnes);
   for(int i=0;i<nbLignes;i++)
   {
      for(int j=0;j<nbColonnes;j++)
      {
        res.data[j+i*nbColonnes]=data[j+i*nbColonnes]-M.data[j+i*nbColonnes];   
      }
   }  
   return res;
}


Matrice operator* (float a,Matrice M)
{
  Matrice res(M.nbLignes,M.nbColonnes);
  #ifdef DEBUG  
  if(M.data==NULL)
  { 
    warning("operator*(float,Matrice): matrice vide");
    return res; 
  }
  #endif
  for(int i=0;i<M.nbLignes*M.nbColonnes;i++)
  { res.data[i]=a*M.data[i]; }
  return res;
}


Matrice Matrice::operator - () const
{
   Matrice res=Matrice(nbLignes,nbColonnes);
   for(int i=0;i<nbLignes*nbColonnes;i++)
   {
     res.data[i]= -data[i];     
   }  
   return res;
}


const Matrice& Matrice::operator += ( const Matrice& m )
{
   #ifdef DEBUG
   if( nbLignes!=m.nbLignes || nbColonnes!=m.nbColonnes )
   {
     warning("Matrice::operator +=: dimensions des matrices non compatibles");
   }  
   #endif
   for(int i=0;i<nbLignes*nbColonnes;i++)
   {
     data[i]+= m.data[i];     
   }  
}

const Matrice& Matrice::operator -= ( const Matrice& m )
{
   #ifdef DEBUG
   if( nbLignes!=m.nbLignes || nbColonnes!=m.nbColonnes )
   {
     warning("Matrice::operator +=: dimensions des matrices non compatibles");
   }  
   #endif
   for(int i=0;i<nbLignes*nbColonnes;i++)
   {
     data[i]-= m.data[i];     
   }  
}



int eigen(Matrice *M,Vecteur *valp,Matrice *vecp)
{
    #ifdef DEBUG  
    if( (M->data==NULL) || (M->nbLignes<=1) )
    { 
      warning("eigen(Matrice *,Vecteur *,Matrice *): matrice vide ou de taille 1 x 1");
      return 0;  
    }
    if(M->nbLignes!=M->nbColonnes)
    { 
      warning("eigen(Matrice *,Vecteur *,Matrice *): matrice non carrée");
      return 0;  
    }    
    #endif
    REAL   **mat,             // input  matrix                         
             **a,              // copy of the input matrix              
         **ev = NULL,          // Eigenvectors if vec <> 0              
             *wr,              // Eigenvalues (Real part)               
             *wi;              // Eigenvalues (Imaginary parts)         

    int n=M->nbLignes,
       *cnt,                 // Iteration counter                       
         rc,                 // Return Code                             
      vec = 1,               // flag for eigenvectors (=0 -> none)      
      ortho = 0,             // flag for orthogonal                     
                             // Hessenberg reduction                    
      ev_norm = 1;           // flag for normalization of Eigenvectors  

    int register i, j, k;
    REAL   v, w, norm;
    void   *vmblock;
    // Allocate Memory .................................................
    
    vmblock = vminit();
    mat  = (REAL **)vmalloc(vmblock, MATRIX,  n, n);
    a    = (REAL **)vmalloc(vmblock, MATRIX,  n, n);
    wr   = (REAL *) vmalloc(vmblock, VEKTOR,  n, 0);
    wi   = (REAL *) vmalloc(vmblock, VEKTOR,  n, 0);
    cnt  = (int  *) vmalloc(vmblock, VVEKTOR, n, sizeof(*cnt));
    ev = (REAL **)vmalloc(vmblock, MATRIX, n, n);
    for(i=0;i<n;i++)
    {   
      for(j=0;j<n;j++)
      { 
        mat[i][j]= M->elt(i+1,j+1);
      }
    }
  
    rc=eigen(vec, ortho, ev_norm, n, mat, ev, wr, wi, cnt);
    valp->reinit(M->nbLignes);
    for(i=0;i<M->nbLignes;i++)
    { valp->data[i]= wr[i]; }
    vecp->reinit(M->nbLignes,M->nbLignes);
    for(i=0;i<M->nbLignes;i++)
    {   
      for(j=0;j<M->nbColonnes;j++)
      { 
         vecp->data[j + i*M->nbColonnes]= ev[i][j];
      }
    }    
    return 1;   
}

int nullspace(Matrice M,Matrice *nullmat)
{
   int i,j,n;
   Matrice M2;
   if(M.nbLignes == M.nbColonnes)
   {  M2=M; 
      n=M.nbColonnes;
   }
   else
   {
      if(M.nbLignes < M.nbColonnes)
      {   M2=transpose(M)*M;
          n=M.nbColonnes;
      }
      else
      {   M2=M*transpose(M);
          n=M.nbLignes;
      }
   }
   
 
   Vecteur valp;
   Matrice vecp;
   eigen(&M2,&valp,&vecp);
   //valp.affiche();
   int cnt=0;
   for(i=0;i<n;i++)
   {
      if(fabs(valp(i+1))<0.000001)
        cnt++;
   }
   if(cnt==0)
     return 0;
     
   int n2=cnt;

   nullmat->reinit(n,n2);
   cnt=0;
   for(j=0;j<n;j++)
   {
      if(fabs(valp(j+1))<0.000001)
      {  
        for(i=0;i<n;i++)
        {                             
          nullmat->data[ cnt + i*n2]= vecp(i+1,j+1);
        }
        cnt++;
      }
   }   
   return n2;
}

float produitScalaire(Vecteur v1,Vecteur v2)
{
  #ifdef DEBUG  
  if(v1.dimension!=v2.dimension)
  { warning("produitScalaire: dimensions des vecteurs non compatibles"); }
  #endif   
  float res=0;
  for(int i=0;i<v1.dimension;i++)
  { res+=v1.data[i]*v2.data[i];  }  
  return res;  
}

//Retourne qb*qc ce qui est pareil à la rotation qc suivi par 
//la rotation qb
Vecteur produitQuaternion(Vecteur qb,Vecteur qc)
{
  Vecteur qa(4);
  #ifdef DEBUG  
  if( (qb.dimension!=4)||(qc.dimension!=4) )
  { 
    warning("produitQuaternion: une des entrées n'est pas de dimension 4");
    return qa;
  }   
  if( (qb.norme()<1-EPS)||(qb.norme()>1+EPS) )
  {
    warning("produitQuaternion: qb n'est pas un quaternion unité");
    qb.normalisation();
  }
  if( (qc.norme()<1-EPS)||(qc.norme()>1+EPS) )
  {
    warning("produitQuaternion: qc n'est pas un quaternion unité");
    qc.normalisation();
  } 
  #endif
  qa.data[0] = qb(1)*qc(1) - qb(2)*qc(2) - qb(3)*qc(3) - qb(4)*qc(4);
  qa.data[1] = qb(1)*qc(2) + qb(2)*qc(1) + qb(3)*qc(4) - qb(4)*qc(3);
  qa.data[2] = qb(1)*qc(3) + qb(3)*qc(1) + qb(4)*qc(2) - qb(2)*qc(4);
  qa.data[3] = qb(1)*qc(4) + qb(4)*qc(1) + qb(2)*qc(3) - qb(3)*qc(2);
  return qa;     
}

Vecteur inverseQuaternion(Vecteur q)
{
  Vecteur qres(4);
  #ifdef DEBUG  
  if( q.dimension!=4 )
  { 
    warning("inverseQuaternion: l'entrée n'est pas de dimension 4");
    return qres;
  }   
  if( (q.norme()<1-EPS)||(q.norme()>1+EPS) )
  {
    warning("inverseQuaternion: q n'est pas un quaternion unité");
    q.normalisation();
  }
  #endif
  float a=q(1)*q(1)+q(2)*q(2)+q(3)*q(3)+q(4)*q(4);
  
  qres.data[0]=q(1)/a;
  qres.data[1]=-q(2)/a;
  qres.data[2]=-q(3)/a;
  qres.data[3]=-q(4)/a;
  
  return qres;
}

float norme(Vecteur v)
{
  #ifdef DEBUG    
  if( v.data==NULL )
  { warning("norme: vecteur vide");
     exit(0);  }
  #endif
  float norm=0;
  for(int i=0;i<v.dimension;i++)
  { norm+= v.data[i]*v.data[i]; }
  return sqrt(norm);    
}

float norme1(Vecteur v)
{
  #ifdef DEBUG
  if(v.data==NULL)
   { warning("norme1: vecteur vide"); 
     exit(0);  }
  #endif
  float norm=0;
  for(int i=0;i<v.dimension;i++)
   {
     norm+= fabs(v.data[i]);
   }
  return norm;    
}

Vecteur abs(Vecteur v)
{
  Vecteur result(v.dimension);
  for(int i=0;i<v.dimension;i++)
  { result.data[i]=fabs(v.data[i]);  }
  return result;
}

void Vecteur::affiche()
{
  char *string=new char[dimension*10]; 
  int n=0;
  for(int i=0;i<dimension;i++)
  {  n+=sprintf(string+n,"%2.3f ",data[i]);  }
  warning(string);
  delete[] string;
  return;
}

void Matrice::affiche()
{
  char *string=new char[nbLignes*nbColonnes*10]; 
  int n=0;
  for(int i=0;i<nbLignes;i++)
  {
    for(int j=0;j<nbColonnes;j++)
    {  n+=sprintf(string+n,"%2.3f ",data[nbColonnes*i+j]);  }
    n+=sprintf(string+n,"\n");
  }
  //sprintf(string+n,'\0');
  warning(string);
  delete[] string;
  return;
}


//Cette fonction crée et retourne un tableau composé des n1 1ers
//éléments de tab1 suivi des n2 1ers éléments de tab2.
//tab1 et tab2 sont libérés. Le type des variables
//du tableau est indiqué par la valeur du caractère type.
void *concateneTableaux(void *tab1,int n1,void *tab2,int n2,char type)
{
   void *tmp; 
   void *result;
   switch(type)
   {
   case 'v': //Vecteur
          tmp=new Vecteur[n1];
          result=new Vecteur[n1+n2];
          for(int i=0;i<n1;i++)
          { ((Vecteur *)tmp)[i]=((Vecteur *)tab1)[i]; }  
          delete [] (Vecteur *)tab1;
          for(int i=0;i<n1;i++)
          { ((Vecteur *)result)[i]=((Vecteur *)tmp)[i];  }
          for(int i=0;i<n2;i++)
          { ((Vecteur *)result)[n1+i]=((Vecteur *)tab2)[i];  }
          delete [] ((Vecteur *)tmp);  
          delete [] ((Vecteur *)tab2); 
          return (void *)result;       
    break;
    case 'd': //int
          tmp=new int[n1]; 
          result=new int[n1+n2];
          for(int i=0;i<n1;i++)
          { ((int *)tmp)[i]=((int *)tab1)[i]; }  
          delete [] (int *)tab1;
          for(int i=0;i<n1;i++)
          {  ((int *)result)[i]=((int *)tmp)[i];  }
          for(int i=0;i<n2;i++)
          {  ((int *)result)[n1+i]=((int *)tab2)[i];   }
           delete [] ((int *)tmp); 
           delete [] ((int *)tab2);  
           return (void *)result;   
     break;
     case 'b': //bool
          tmp=new bool[n1]; 
          result=new bool[n1+n2];
          for(bool i=0;i<n1;i++)
          { ((bool *)tmp)[i]=((bool *)tab1)[i]; }  
          delete [] (bool *)tab1;
          for(bool i=0;i<n1;i++)
          { ((bool *)result)[i]=((bool *)tmp)[i]; }
          for(bool i=0;i<n2;i++)
          { ((bool *)result)[n1+i]=((bool *)tab2)[i];  }
           delete [] ((bool *)tmp); 
           delete [] ((bool *)tab2);  
           return (void *)result;   
     break;     
     default:
         warning("concateneTableaux: mauvais paramètre (char type)");    
     break;
   }
}

//Cette fonction crée et retourne un tableau composé des n1 1ers
//éléments de tab1 suivi de n cases vides.
//tab1 est libérés. Le type des variables
//du tableau est indiqué par la valeur du caractère type.
void *agranditTableau(void *tab1,int n1,int n,char type)
{
   void *result;
   switch(type)
   {
    case 'v': //Vecteur
          result=new Vecteur[n1+n];
          for(int i=0;i<n1;i++)
          { ((Vecteur *)result)[i]=((Vecteur *)tab1)[i]; }  
          if(tab1!=NULL) delete [] (Vecteur *)tab1;
          return (void *)result;       
    break;
    case 'd': //int
          result=new int[n1+n];
          for(int i=0;i<n1;i++)
          { ((int *)result)[i]=((int *)tab1)[i]; }  
          if(tab1!=NULL) delete [] (int *)tab1;
          return (void *)result;   
     break;
     case 'b': //bool
          result=new bool[n1+n];
          for(int i=0;i<n1;i++)
          { ((bool *)result)[i]=((bool *)tab1)[i]; }  
          if(tab1!=NULL) delete [] (bool *)tab1;
          return (void *)result;   
     break;     
     default:
         warning("agranditTableau: mauvais paramètre (char type)");    
     break;
   }
}


float random(float a, float b)
{ 
  float v;
  v = ran1(&Idum);
  v = (b - a)*v + a;
  return v;
}


//fonction tirée du livre "Numerical Recipes in C" (chapitre 7)
float ran1(long *idum)
//“Minimal” random number generator of Park and Miller with Bays-Durham shuffle and added
//safeguards. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of the endpoint
//values). Call with idum a negative integer to initialize; thereafter, do not alter idum between
//successive deviates in a sequence. RNMX should approximate the largest floating value that is
//less than 1.
{
    int j;
    long k;
    static long iy=0;
    static long iv[NTAB];
    float temp;
    if (*idum <= 0 || !iy)
    { //Initialize.
       if (-(*idum) < 1) *idum=1; //Be sure to prevent idum = 0.
       else *idum = -(*idum);
       for (j=NTAB+7;j>=0;j--)
       { //Load the shuffle table (after 8 warm-ups).
         k=(*idum)/IQ;
         *idum=IA*(*idum-k*IQ)-IR*k;
         if (*idum < 0) *idum += IM;
         if (j < NTAB) iv[j] = *idum;
       }
       iy=iv[0];
    }
    k=(*idum)/IQ; //Start here when not initializing.
    *idum=IA*(*idum-k*IQ)-IR*k; //Compute idum=(IA*idum) % IM without overif
    if(*idum < 0) *idum += IM; //flows by Schrage’s method.
    j=iy/NDIV; //Will be in the range 0..NTAB-1.
    iy=iv[j]; //Output previously stored value and refill the
    iv[j] = *idum; //shuffle table.
    if ((temp=AM*iy) > RNMX) return RNMX; //Because users don’t expect endpoint values.
    else return temp;
}

void setIdum(long n)
{
  if(n>0)
  { Idum=-n; }
  else
  { Idum=n; }
}


//A partir d'un vecteur u de dimension 3, cette fonction 
//retourne deux vecteurs v et w tels que (u,v,w) (ou (v,w,u)) 
//soit une base orthonormale directe.
void baseOrthonormale(Vecteur u,Vecteur *v,Vecteur *w)
{
    u.normalisation();
    v->reinit(3);
    w->reinit(3);
    if( (fabs(u.data[0])<=EPS) || (fabs(u.data[1])<=EPS) || (fabs(u.data[2])<=EPS) )
    {  
      if(fabs(u.data[0])<=EPS)  {   v->data[0]=1;  }
      else                      {   v->data[0]=0;  }
      if(fabs(u.data[1])<=EPS)  {   v->data[1]=1;  }
      else                      {   v->data[1]=0;  }
      if(fabs(u.data[2])<=EPS)  {   v->data[2]=1;  }
      else                      {   v->data[2]=0;  }
    }
    else 
    {   v->data[0]=0;
        v->data[1]=1;
        v->data[2]=-u.data[1]/u.data[2];
    }
    v->normalisation();
    Vecteur tmp=u*(*v);
    w->data[0]=tmp.data[0];
    w->data[1]=tmp.data[1];
    w->data[2]=tmp.data[2];    
}

//Cette fonction calcule la projection orthogonale
//d'un point sur un plan d'équation ( ax + by + cz = d) dont
//les paramètres sont dans le vecteur eq: eq=(a,b,c,d).
Vecteur projectionOrthPointSurPlan(Vecteur eq,Vecteur point)
{
   Vecteur result(3);
   #ifdef DEBUG     
   if( (eq.dimension!=4) || (point.dimension!=3) )
   {
     warning("projectionOrthPointSurPlan(Vecteur,Vecteur):entrée(s) de mauvaise dimension");
     return result;       
   }
   #endif
   //Soit p2=(x2,y2,z2) le projeté orthogonal sur le plan d'équation
   // ax + by + cz = d, du point p1=(x1,y1,z1).
   //Soit n=(a,b,c).
   //On a n.p2 = d
   //et (p1-p2) = r*n avec r un réel
   //donc r*(n.n) = n.p1 - n.p2 = n.p1 - d
   //=> r = (n.p1 - d)/(n.n)
   
   float r = ( eq(1)*point(1) + eq(2)*point(2) + eq(3)*point(3) - eq(4)) / (eq(1)*eq(1)+eq(2)*eq(2)+eq(3)*eq(3));
   result = point - r*eq.sousVecteur(1,3); 
   return result;
}


//Cette fonction calcule le projeté orthogonal du point P
//sur la droite (AB).
Vecteur projectionOrthPointSurDroite(Vecteur *a,Vecteur *b,Vecteur *p)
{
   Vecteur result(3);
   #ifdef DEBUG     
   if( (a->dimension!=3) || (b->dimension!=3) || (p->dimension!=3) )
   {
     warning("projectionOrthPointSurDroite(Vecteur,Vecteur,Vecteur):entrée(s) de mauvaise dimension");
     return result;       
   }
   #endif
   //Soit H le projeté de P sur AB.
   //On a AH= alpha*AB.
   //HP est minimal si AH.HP=0
   // alpha*AB.HP= 0 = alpha*AB.(AP - AH) = alpha*AB.(AP - alpha*AB)
   // alpha*AB.AP - alpha*alpha*AB.AB = 0
   //-> alpha*(AB*AP - alpha*AB.AB) = 0
   //-> alpha = AB*AP/AB.AB
   
   //u=AB . AP
   float u=(b->data[0]-a->data[0])*(p->data[0]-a->data[0])+(b->data[1]-a->data[1])*(p->data[1]-a->data[1])+(b->data[2]-a->data[2])*(p->data[2]-a->data[2]);
   //alpha = ( AB . AP )/(AB.AB)
   float alpha=u/( pow(b->data[0]-a->data[0],2) + pow(b->data[1]-a->data[1],2) + pow(b->data[2]-a->data[2],2) );
   result.data[0]= a->data[0] + alpha*(b->data[0]-a->data[0]);
   result.data[1]= a->data[1] + alpha*(b->data[1]-a->data[1]);
   result.data[2]= a->data[2] + alpha*(b->data[2]-a->data[2]);
   return result;
}


//Cette fonction retourne les paramètres (a,b,c,d) de l'équation
//(a*x + b*y + c*z=d) d'un plan défini par sa normale et par
//un de ses points.
void equationPlan(Vecteur normale,Vecteur point,Vecteur *param)
{
   #ifdef DEBUG  
   if( (normale.dimension!=3)||(point.dimension!=3)||(param->dimension!=4) )
   { warning("equationPlan(Vecteur,Vecteur,Vecteur*):entrée(s) de mauvaise dimension");
     return; }
   #endif
   param->data[0]=normale.data[0];
   param->data[1]=normale.data[1];
   param->data[2]=normale.data[2];   
   
   param->data[3]=point.data[0]*normale.data[0]+point.data[1]*normale.data[1]+point.data[2]*normale.data[2];
}

//Cette fonction calcule l'équation d'un plan (a*x + b*y + c*z=d) défini
//par trois points. Le sens de la normale du plan sera tel que
//les points 1,2 et 3 tournent dans le sens direct autour
//de la normale quand on les regarde la normale pointant
//l'observateur.
void equationPlanTriangle(float *point1,float *point2,float * point3,float *param)
{
  float u12[3],u13[3],n[3];
  u12[0]=point2[0]-point1[0];
  u12[1]=point2[1]-point1[1];  
  u12[2]=point2[2]-point1[2];
  
  u13[0]=point3[0]-point1[0];
  u13[1]=point3[1]-point1[1];  
  u13[2]=point3[2]-point1[2];  
  
  n[0]=u12[1]*u13[2] - u12[2]*u13[1];
  n[1]=u12[2]*u13[0] - u12[0]*u13[2];
  n[2]=u12[0]*u13[1] - u12[1]*u13[0];

  float norm=sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
  param[0]=n[0]/norm;
  param[1]=n[1]/norm;
  param[2]=n[2]/norm; 
  
  param[3]= (param[0]*point1[0]+param[1]*point1[1]+param[2]*point1[2]);
}


//Cette fonction calcule les deux solutions de l'équation
// a*cos(x) + b*sin(x) = c où x est l'inconnue.
//Les solutions sont recopiées dans *x1 et *x2 et la fonction retourne
//1 s'il y a une solution, 0 sinon.
int solutionEquationTrigo(float a,float b,float c,float *x1,float *x2)
{
  float d= pow(b,4) + a*a*b*b -  b*b*c*c;
  if( d < 0 )
  { //warning("solutionEquationTrigo(float,float,float): il n'y a pas de solution");
    return 0;
  }
  float A=0.5*( 2*a*c + 2*sqrt(d) )/( a*a+b*b );
  *x1=atan2( (c-a*A)/b , A );
  
  A=0.5*( 2*a*c - 2*sqrt(d) )/( a*a+b*b );
  *x2=atan2( (c-a*A)/b , A );

  return 1;
}


bool testeIntersectionSphereTriangle(Vecteur a,Vecteur b,Vecteur c,float rayon,Vecteur centre)
{
  #ifdef DEBUG
   if((a.dimension!=3)||(b.dimension!=3)||(c.dimension!=3)||(centre.dimension!=3)||(rayon<0))
    warning("testeIntersectionSphereTriangle(Vecteur,Vecteur,Vecteur,float,Vecteur): mauvaise(s) entrée(s)");
  #endif
  Vecteur paramAB(4),paramBC(4),paramAC(4),pn(3),ppp(3);
  float x1,x2,d1,d2,d3,u,alpha;
  Vecteur param(4);
  equationPlanTriangle(a.data,b.data,c.data,param.data);
  Vecteur n(param(1),param(2),param(3));
  pn= a + n;
  equationPlanTriangle(pn.data,a.data,b.data,paramAB.data); 
  pn= c + n;  
  equationPlanTriangle(pn.data,c.data,a.data,paramAC.data);   
  pn= b + n; 
  equationPlanTriangle(pn.data,b.data,c.data,paramBC.data); 
  
  float xABp,xABc,xACp,xACb,xBCp,xBCa;
  xABp=paramAB(1)*centre(1)+paramAB(2)*centre(2)+paramAB(3)*centre(3) - paramAB(4);
  xABc=paramAB(1)*c(1)+paramAB(2)*c(2)+paramAB(3)*c(3) - paramAB(4);
  xACp=paramAC(1)*centre(1)+paramAC(2)*centre(2)+paramAC(3)*centre(3) - paramAC(4);
  xACb=paramAC(1)*b(1)+paramAC(2)*b(2)+paramAC(3)*b(3) - paramAC(4);
  xBCp=paramBC(1)*centre(1)+paramBC(2)*centre(2)+paramBC(3)*centre(3) - paramBC(4);
  xBCa=paramBC(1)*a(1)+paramBC(2)*a(2)+paramBC(3)*a(3) - paramBC(4);  
  
  if( xABp*xABc < 0 )
  {
     //u = AB . AP
     u=(b(1)-a(1))*(centre(1)-a(1))+(b(2)-a(2))*(centre(2)-a(2))+(b(3)-a(3))*(centre(3)-a(3));
     //alpha = ( AB . AP )/(AB.AB)
     alpha=u/( pow(b(1)-a(1),2) + pow(b(2)-a(2),2) + pow(b(3)-a(3),2) );
     if( (alpha>=0) && (alpha<=1) )
     {
        ppp= a + alpha*(b-a);
        if(norme(centre-ppp)<rayon)
          return true;
        else
          return false;
     }
     else
     {
        if(norme(centre-a)<norme(centre-b))
        { 
          if(norme(centre-a)<rayon)
            return true;
          else
            return false;
        }
        else
        { 
          if(norme(centre-b)<rayon)
            return true;
          else
            return false;
        }         
     }  
  }
  
  if( xACp*xACb < 0 )
  {  
     //u=CA.CP
     u=(a(1)-c(1))*(centre(1)-c(1))+(a(2)-c(2))*(centre(2)-c(2))+(a(3)-c(3))*(centre(3)-c(3));
     //alpha = ( CA . CP )/(CA.CA)
     alpha=u/( pow(a(1)-c(1),2) + pow(a(2)-c(2),2) + pow(a(3)-c(3),2) );
     if( (alpha>=0) && (alpha<=1) )
     {
        ppp= c + alpha*(a-c);
        if(norme(centre-ppp)<rayon)
            return true;
        else
            return false;      
     }
     else
     {
        if(norme(centre-c)<norme(centre-a))
        { 
          if(norme(centre-c)<rayon)
            return true;
           else
            return false;  
        }
        else
        { 
          if(norme(centre-a)<rayon)
            return true;
          else
            return false;  
        }          
     }
  }
  
  if( xBCp*xBCa < 0 )
  {  
     //u= CB . BP
     u=(b(1)-c(1))*(centre(1)-c(1))+(b(2)-c(2))*(centre(2)-c(2))+(b(3)-c(3))*(centre(3)-c(3));
     //alpha= (CB . BP) / (CB . CB) 
     alpha=u/( pow(b(1)-c(1),2) + pow(b(2)-c(2),2) + pow(b(3)-c(3),2) );
     if( (alpha>=0) && (alpha<=1) )
     {
        ppp= c + alpha*(b-c);        
        if(norme(centre-ppp)<rayon)
            return true;
        else
            return false;         
     }
     else
     {
        if(norme(centre-c)<norme(centre-b))
        { 
          if(norme(centre-c)<rayon)
            return true;
          else
            return false;  
        }
        else
        { 
          if(norme(centre-b)<rayon)
            return true;
          else
            return false; 
        }         
     }              
  }  
  
  ppp=projectionOrthPointSurPlan(param,centre); 
  if(norme(centre-ppp)<rayon)
    return true;
  else
    return false;  
}

bool testeIntersectionSphereBoite(float dimX,float dimY,float dimZ,float rayon,Vecteur *centre)
{ 
   Vecteur closest(3);
   if ( centre->data[0] < -dimX/2 ) 
   {    closest.data[0] =-dimX/2;   }
   else
   {  
      if ( dimX/2 < centre->data[0] ) 
        closest.data[0] = dimX/2; 
      else 
        closest.data[0] = centre->data[0];
   }
   if ( centre->data[1] < -dimY/2 ) 
   {  closest.data[1] = -dimY/2;   }
   else
   {  
      if ( dimY/2 < centre->data[1] ) 
        closest.data[1] = dimY/2 ; 
      else 
        closest.data[1] = centre->data[1];
   }     
   if ( centre->data[2] < -dimZ/2 ) 
   {  closest.data[2] = -dimZ/2 ;  }
   else
   {  
     if ( dimZ/2 < centre->data[2] ) 
       closest.data[2] = dimZ/2 ; 
     else 
      closest.data[2] = centre->data[2] ;
   }   
   if( norme(closest - *centre) <= rayon )
      return true;
   else 
     return false;
}    

// opérateur vec: vec(A) est formé des colonnes de A concaténées verticalement 
//          ( a11 a12 a13  ...  a1m )
//    A =   ( a21 a22 a23  ...  a2m )
//          (  .   .   .         .  )
//          ( an1 an2 an3  ...  anm ) 
//          
//  vec(A) = ( a11 a21 ... an1 a12 a22 ... an2 ... ... a1m a2m ... anm )'
Vecteur vec(Matrice *A)       
{
    #ifdef DEBUG  
    if( (A->data==NULL) || (A->nbLignes<=1) )
    { 
      warning("vec(Matrice *) : matrice vide ou de taille 1 x 1");
      return 0;  
    }
    #endif
    Vecteur res(A->nbLignes*A->nbColonnes);
    for(int i=0;i<A->nbLignes*A->nbColonnes;i++)
    {  res.data[i]=A->data[i];   }
    return res;
}


/*Sahar Function */
//Fonction de calcul de l'inverse matricielle (utilise la méthode
//de Jordan avec recherche de pivot maximum).
//Elle retourne l'inverse de la matrice M.
int myinverse(Matrice M)
{
   int erreur;
   #ifdef DEBUG  
   if(M.nbLignes!=M.nbColonnes)
   {
      warning("inverse: erreur matrice non carrée\n");
      exit(0);
   }   
   #endif
   int dim=M.nbLignes;                                                       
   int i,j,k,l,err;
   int nmax=dim+1,n2max=2*nmax-1;
   float a[nmax][nmax],b[nmax][nmax];
   float max,pivot,coef;
   float t[nmax][n2max];
   for(i=0;i<dim;i++)
	{  a[i][0]=0; a[0][i]=0; }
   for(i=1;i<=dim;i++)
	{  for(j=1;j<=dim;j++)
		{  a[i][j]=M.data[j-1+dim*(i-1)];
		}
	}

   for(i=1;i<=dim;i++)
	{
	  for(j=1;j<=dim;j++)
		{
	     t[i][j]=a[i][j];
		 if(i==j) 
			t[i][j+dim]=1.0;
		 else 
			t[i][j+dim]=0.0;
		}
   }
   err=1;
   k=1;
   while (err==1 && k<=dim)
   {
		max=fabs(t[k][k]);
		l=k;
		for(i=k+1;i<=dim;i++)
		{
			if(max<fabs(t[i][k]))
			{
			 max=fabs(t[i][k]);
			 l=i;
			} 
		}
	    if(max!=0)
		{
		  for(j=k;j<=2*dim;j++)
			{
			  max=t[k][j];
			  t[k][j]=t[l][j];
			  t[l][j]=max;
			}
		  pivot=t[k][k];
		  for(j=k+1;j<=2*dim;j++)
			{ t[k][j] /= pivot; }
		  for(i=1;i<=dim;i++)
			{ if(i!=k)
				{
				  coef=t[i][k];
				  for(j=k+1;j<=2*dim;j++)
				  {	 t[i][j] -= coef*t[k][j]; }
				}
			}
		}
		else
		{ err=0;}
        k++;
   }
  /* for(i=1;i<=dim;i++)
	{ for(j=1;j<=dim;j++) { b[i][j]=t[i][j+dim]; } 
	}
   
   Matrice res=Matrice(M.nbLignes,M.nbLignes);*/
   //erreur = (int*)malloc(sizeof(int));
   erreur = 1-err; // =1 si erreur
   /*if (erreur == 0)
      printf("waouuuuu\n");*/
   /*if(err==0) 
   { 
	   warning("inverse: erreur: matrice non inversible\n");
	   /*printf("nonnnnn\n");
       for(i=0;i<dim;i++)
			{  for(j=0;j<dim;j++)
				{ res.data[j+i*dim]=0; }
			}
	   return res;*/
 //  }
   
   /*for(i=1;i<=dim;i++)
   {  for(j=1;j<=dim;j++)
		{ res.data[j-1+dim*(i-1)]=b[i][j];
		}
   }
  
  return res;*/
  return erreur;
}            

 
