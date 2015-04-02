/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "Base3DObject.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "StdTools/Various.h"
#include "StdTools/LogStream.h"
using namespace std;



REALTYPE *Base3DObject::sVerData  = NULL;
int      *Base3DObject::sTriData  = NULL;
int      *Base3DObject::sPolySize = NULL;
int       Base3DObject::sObjCount = 0;

Base3DObject::Base3DObject(){

    sObjCount++;
    if(sVerData==NULL)  sVerData  = new REALTYPE[BASE3DOBJ_FILE_MAX_VERTICES*3];
    if(sTriData==NULL)  sTriData  = new      int[BASE3DOBJ_FILE_MAX_TRIANGLES*3];
    if(sPolySize==NULL) sPolySize = new      int[BASE3DOBJ_FILE_MAX_TRIANGLES];
    mNbVertices     = 0;
    mNbPolygons     = 0;
    mPolygons       = NULL;
    mPolygonSize    = NULL;
    mPolygonStart   = NULL;
    mEdgeNeighbours     = NULL;
    mVertexNeighbours       = NULL;
    mVertexNeighbourSize   = NULL;
    mVertexNeighbourStart   = NULL;
    mIsVisible      = NULL;

    mVertices.Resize(0,0);
    mNormals.Resize(0,0);
}

Base3DObject::~Base3DObject(){
    Free();

    sObjCount--;
    if(sObjCount<=0){
        delete [] sVerData;
        delete [] sTriData;
        delete [] sPolySize;
        sVerData    = NULL;
        sTriData    = NULL;
        sPolySize   = NULL;
        sObjCount   = 0;
    }
}

void Base3DObject::Free(){
    if(mPolygons!=NULL) delete [] mPolygons;
    mPolygons = NULL;
    if(mPolygonSize!=NULL) delete [] mPolygonSize;
    mPolygonSize = NULL;
    if(mPolygonStart!=NULL) delete [] mPolygonStart;
    mPolygonStart = NULL;
    if(mEdgeNeighbours!=NULL) delete [] mEdgeNeighbours;
    mEdgeNeighbours     = NULL;
    if(mVertexNeighbours!=NULL) delete [] mVertexNeighbours;
    mVertexNeighbours       = NULL;
    if(mVertexNeighbourSize!=NULL) delete [] mVertexNeighbourSize;
    mVertexNeighbourSize   = NULL;
    if(mVertexNeighbourStart!=NULL) delete [] mVertexNeighbourStart;
    mVertexNeighbourStart = NULL;
    if(mIsVisible!=NULL) delete [] mIsVisible;
    mIsVisible      = NULL;
    mVertices.Resize(0,0);
    mNormals.Resize(0,0);
    mNbVertices  = 0;
    mNbPolygons = 0;
}

bool    Base3DObject::LoadFromObjFile(const char *filename, bool invNormals){

    Free();

    ifstream ifile;
    ifile.open(filename);
    //cerr <<"Opening file <"<<filename<<">"<<endl;
    if(!ifile.is_open()){
        cerr <<"Error while opening file <"<<filename<<">"<<endl;
        return false;
    }

    char cline[512];
    char c;

    REALTYPE *verPos = sVerData;
    int      *triPos = sTriData;

    mNbVertices  = 0;
    mNbPolygons = 0;

    int polyIndex[256];
    int polyIndexCnt=0;
    ifile.getline(cline,512);
    while(!ifile.eof()){
        if(cline[0]=='v'){
            if(cline[1]==' '){
                float x,y,z;
                sscanf(cline,"%c %f %f %f",&c,&x,&y,&z);
                (*verPos++) = REALTYPE(x);
                (*verPos++) = REALTYPE(y);
                (*verPos++) = REALTYPE(z);
                mNbVertices++;
            }
        }else if (cline[0]=='f'){
            stringstream ss(RemoveSpaces(cline+1));
            int cnt = 0;
            char c;
            float tmp;
            while(!ss.eof()){
                ss >> polyIndex[cnt];
                ss.get(c);
                if(c=='/'){
                    while((c!=' ')&&(!ss.eof())){
                        ss.get(c);
                    }
                }else{
                    ss.unget();
                }
                //cout << polyIndex[cnt]<<endl;

                cnt++;
            }
            if(cnt>2){
                sPolySize[mNbPolygons] = cnt;
                if(!invNormals){
                    for(int i=0;i<cnt;i++)
                        *(triPos++) = polyIndex[i]-1;
                }else{
                    for(int i=cnt-1;i>=0;i--)
                        *(triPos++) = polyIndex[i]-1;
                }
                polyIndexCnt += cnt;
                mNbPolygons++;
            }
        }
        ifile.getline(cline,512);
    }
    ifile.close();

//    cout << "OK"<<endl;
    mPolygons   = new int      [polyIndexCnt];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);

    memcpy(mVertices.Array(), sVerData,  mNbVertices  *3 *sizeof(REALTYPE));
    memcpy(mPolygons,        sTriData,  polyIndexCnt    *sizeof(int));
    memcpy(mPolygonSize,      sPolySize, mNbPolygons    *sizeof(int));

    int cumSize = 0;
    mPolygonStart = new int      [mNbPolygons];
    for(int i=0;i<mNbPolygons;i++){
        mPolygonStart[i] = cumSize;
        cumSize += mPolygonSize[i];
    }

    gLOG.SetCurrentEntry("Base3DObject");
    gLOG.Append("Object  <%s> loaded. Found %d vertices and %d polygons",filename, mNbVertices,mNbPolygons);

    //cout <<"Loaded Object: "<<filename<<" Vertices: "<<mNbVertices<<" Polygons: "<<mNbPolygons<<endl;

    CalcNormals();

    return true;
}

#define NORMAL_MERGE_VALUE 0.866025404

void Base3DObject::CalcNormals(bool perVertex){
    bNormalsPerVertex = perVertex;

    if(bNormalsPerVertex){
        CalcNormals(false);
        bNormalsPerVertex = true;
        Matrix tmpNormals = mNormals;

        int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];
        mNormals.Resize(cumSize,3,false);

        Matrix cumNormals(cumSize,3);
        Vector cumCount(cumSize);

        CalcNeighbours();



        int cnt      = 0;
        int localCnt = 0;

        REALTYPE *v1,*v2,*v3;
        Vector3 a,b,n;
        Vector cn;
        for(int i=0;i<mNbPolygons;i++){
            localCnt = cnt;

            for(int j=0;j<mPolygonSize[i];j++){

                n.x() = tmpNormals(i,0);
                n.y() = tmpNormals(i,1);
                n.z() = tmpNormals(i,2);
                a = n;

                for(int k=0;k<mVertexNeighbourSize[mPolygons[localCnt]];k++){
                    int id = mVertexNeighbours[mVertexNeighbourStart[mPolygons[localCnt]]+k];
                    if(id!=i){
                        b.x() = tmpNormals(id,0);
                        b.y() = tmpNormals(id,1);
                        b.z() = tmpNormals(id,2);
                        if(n.Dot(b)>=NORMAL_MERGE_VALUE){
                            a+=b;
                        }
                    }
                }
                n = a;
                n.Normalize();
                mNormals(localCnt,0) = n.x();
                mNormals(localCnt,1) = n.y();
                mNormals(localCnt,2) = n.z();
                localCnt++;
            }
            cnt += mPolygonSize[i];
        }
    }else{
        mNormals.Resize(mNbPolygons,3,false);
        int cnt = 0;
        REALTYPE *v1,*v2,*v3;
        Vector3 a,b,n;
        for(int i=0;i<mNbPolygons;i++){
            v1 = mVertices.Array()+(mPolygons[cnt+0])*3;
            v2 = mVertices.Array()+(mPolygons[cnt+1])*3;
            v3 = mVertices.Array()+(mPolygons[cnt+2])*3;
            a.x() =  v2[0]-v1[0];
            a.y() =  v2[1]-v1[1];
            a.z() =  v2[2]-v1[2];
            b.x() =  v3[0]-v1[0];
            b.y() =  v3[1]-v1[1];
            b.z() =  v3[2]-v1[2];
            a.Cross(b,n);
            n.Normalize();
            mNormals(i,0) = n.x();
            mNormals(i,1) = n.y();
            mNormals(i,2) = n.z();
            cnt+=mPolygonSize[i];
        }
    }
}


void Base3DObject::CalcPlanes(){
    mPlanes.Resize(mNbPolygons,4,false);

    int cnt = 0;
    REALTYPE *v1,*v2,*v3;
    Vector3 a,b,n;
    for(int i=0;i<mNbPolygons;i++){
        v1 = mVertices.Array()+(mPolygons[cnt+0])*3;
        v2 = mVertices.Array()+(mPolygons[cnt+1])*3;
        v3 = mVertices.Array()+(mPolygons[cnt+2])*3;

        mPlanes(i,0) = v1[1]*(v2[2]-v3[2]) + v2[1]*(v3[2]-v1[2]) + v3[1]*(v1[2]-v2[2]);
        mPlanes(i,1) = v1[2]*(v2[0]-v3[0]) + v2[2]*(v3[0]-v1[0]) + v3[2]*(v1[0]-v2[0]);
        mPlanes(i,2) = v1[0]*(v2[1]-v3[1]) + v2[0]*(v3[1]-v1[1]) + v3[0]*(v1[1]-v2[1]);
        mPlanes(i,3) = -( v1[0]*( v2[1]*v3[2] - v3[1]*v2[2] ) + v2[0]*(v3[1]*v1[2] - v1[1]*v3[2]) + v3[0]*(v1[1]*v2[2] - v2[1]*v1[2]) );

        cnt+=mPolygonSize[i];
    }
    //mPlanes.Print();
}

void  Base3DObject::CalcNeighbours(){
    int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];

    if(mEdgeNeighbours!=NULL) delete [] mEdgeNeighbours;
    mEdgeNeighbours     = new int [cumSize];
    for(int i=0;i<cumSize;i++) mEdgeNeighbours[i] = -1;


    if(mIsVisible!=NULL) delete [] mIsVisible;
    mIsVisible      = new int [mNbPolygons];


    if(mVertexNeighbours!=NULL) delete [] mVertexNeighbours;
    mVertexNeighbours     = new int [cumSize];

    if(mVertexNeighbourSize!=NULL) delete [] mVertexNeighbourSize;
    mVertexNeighbourSize     = new int [mNbVertices];
    memset(mVertexNeighbourSize,0,mNbVertices*sizeof(int));

    int cnt=0;
	for(int i=0;i<mNbPolygons;i++){
		for(int ki=0;ki<mPolygonSize[i];ki++){
            mVertexNeighbourSize[mPolygons[cnt]]++;
            cnt++;
        }
    }

    if(mVertexNeighbourStart!=NULL) delete [] mVertexNeighbourStart;
    mVertexNeighbourStart = new int [mNbVertices];

    int vcumSize = 0;
    for(int i=0;i<mNbVertices;i++){
        mVertexNeighbourStart[i] = vcumSize;
        vcumSize += mVertexNeighbourSize[i];
    }

    memset(mVertexNeighbourSize,0,mNbVertices*sizeof(int));

    cnt=0;
	for(int i=0;i<mNbPolygons;i++){
		for(int ki=0;ki<mPolygonSize[i];ki++){
            mVertexNeighbours[mVertexNeighbourStart[mPolygons[cnt]]+mVertexNeighbourSize[mPolygons[cnt]]] = i;
            mVertexNeighbourSize[mPolygons[cnt]]++;
            cnt++;
        }
    }


    int ioff = 0;
    int joff = 0;
	for(int i=0;i<mNbPolygons-1;i++){

		for(int ki=0;ki<mPolygonSize[i];ki++){

            if(mEdgeNeighbours[ioff+ki]<0){
                bool bFound = false;

                joff = ioff + mPolygonSize[i];
    		    for(int j=i+1;j<mNbPolygons;j++){

					for(int kj=0;kj<mPolygonSize[j];kj++){

						int p1i=mPolygons[ioff+ki];
						int p2i=mPolygons[ioff+((ki+1)%mPolygonSize[i])];

						int p1j=mPolygons[joff+kj];
						int p2j=mPolygons[joff+((kj+1)%mPolygonSize[j])];

                        //cout << i<<" "<<j<<": "<<ki<<" "<<kj<<": "<<p1i<<" "<<p2i<<" "<<p1j<<" "<<p2j<<endl;

                        if(((p1i==p1j) && (p2i==p2j)) || ((p1i==p2j)&&(p2i==p1j))){
                            mEdgeNeighbours[ioff+ki] = j;
                            mEdgeNeighbours[joff+kj] = i;
                            //cout << "found"<<endl;
                            bFound = true;
                            break;
	                    }
                    }
                    if(bFound) break;
                    joff += mPolygonSize[j];
                }
            }
        }
        ioff += mPolygonSize[i];
    }
}

void Base3DObject::ConvertToTriangles(){
    bool bOnlyTriangles = true;
    for(int i=0;i<mNbPolygons;i++){
        if(mPolygonSize[i]>3){
            bOnlyTriangles = false;
            break;
        }
    }
    if(bOnlyTriangles) return;

    int *triPos        = sTriData;
    int newNbTriangles = 0;
    int polyIndexCnt   = 0;
    for(int i=0;i<mNbPolygons;i++){
        for(int j=0;j<mPolygonSize[i]-2;j++){
            *(triPos++) = mPolygons[polyIndexCnt];
            *(triPos++) = mPolygons[polyIndexCnt+j+1];
            *(triPos++) = mPolygons[polyIndexCnt+j+2];
            sPolySize[newNbTriangles] = 3;
            newNbTriangles++;
        }
        polyIndexCnt+=mPolygonSize[i];
    }
    mNbPolygons = newNbTriangles;
    delete [] mPolygons;
    delete [] mPolygonSize;
    mPolygons   = new int      [mNbPolygons*3];
    mPolygonSize = new int      [mNbPolygons];
    memcpy(mPolygons,        sTriData,  mNbPolygons *3*sizeof(int));
    memcpy(mPolygonSize,      sPolySize, mNbPolygons   *sizeof(int));

    //cout <<"Triangulation:  Vertices: "<<mNbVertices<<" Polygons: "<<mNbPolygons<<endl;

    CalcNormals();
}

void Base3DObject::AddOffset(const Vector3& offset){
    Vector off(offset.Array(),3);
    mVertices.SAddToRow(off);
}

void Base3DObject::Transform(const Matrix3& trans){
    SharedMatrix mat(trans);
    Matrix tmp(mNbPolygons,3,false);
    mVertices.MultTranspose2(mat,tmp);
    mVertices.Swap(tmp);
    mNormals.MultTranspose2(mat,tmp);
    mNormals.Swap(tmp);
}

void Base3DObject::Transform(const Matrix4& trans){
    Transform(trans.GetOrientation());
    AddOffset(trans.GetTranslation());
}

void  Base3DObject::Render(){}







void Base3DObject::CheckVisibility(const Vector3& point){
	//set visual parameter
    Vector p(4);
    p(0) = point.AtNoCheck(0);
    p(1) = point.AtNoCheck(1);
    p(2) = point.AtNoCheck(2);
    p(3) = 1.0;
    Vector res;
    mPlanes.Mult(p,res);
    for(int i=0;i<mNbPolygons;i++){
        if(res.AtNoCheck(i)>0.0)
            mIsVisible[i] = 1;
        else
            mIsVisible[i] = 0;
    }
    /*
    for(int i=0;i<mNbPolygons;i++){
        cout << i<<" "<<mIsVisible[i]<<endl;
    }
    */
}




Base3DObject*   Base3DObject::Clone(Base3DObject* res){
    if(res==NULL)
        res = new Base3DObject();

    res->mNbVertices    = mNbVertices;
    res->mNbPolygons    = mNbPolygons;

    res->mPolygonSize   = new int [mNbPolygons];
    memcpy(res->mPolygonSize,mPolygonSize,mNbPolygons*sizeof(int));

    int cumSize = 0; for(int i=0;i<mNbPolygons;i++) cumSize += mPolygonSize[i];

    res->mPolygons      = new int [cumSize];
    memcpy(res->mPolygons,mPolygons,cumSize*sizeof(int));

    res->mVertices      = mVertices;
    res->mNormals       = mNormals;

    cout <<"OBJ CLONE: "<<cumSize<<" "<<res->mNbPolygons<<endl;
    return res;
}



void Base3DObject::GenerateCylinder(int nbSlices){
    Free();
    mNbVertices = nbSlices*2;
    mNbPolygons = nbSlices+2;
    mPolygons    = new int      [nbSlices*4+2*nbSlices];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int i=0;i<nbSlices;i++){
        REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
        REALTYPE x = 0.5*cos(a);
        REALTYPE y = 0.5*sin(a);
        mVertices(i,0) = mVertices(i+nbSlices,0) = x;
        mVertices(i,1) = mVertices(i+nbSlices,1) = y;
        mVertices(i,2)                = -0.5;
        mVertices(i+nbSlices,2) =  0.5;
    }
    for(int i=0;i<nbSlices-1;i++){
        mPolygons[i*4+0] = i;
        mPolygons[i*4+1] = i+1;
        mPolygons[i*4+2] = nbSlices+i+1;
        mPolygons[i*4+3] = nbSlices+i;
        mPolygonSize[i]  = 4;
    }
    int i=nbSlices-1;
    mPolygons[i*4+0] = i;
    mPolygons[i*4+1] = 0;
    mPolygons[i*4+2] = nbSlices+0;
    mPolygons[i*4+3] = nbSlices+i;
    mPolygonSize[i]  = 4;
    for(int i=0;i<nbSlices;i++){
        mPolygons[nbSlices*4+i] = nbSlices-i-1;
        mPolygons[nbSlices*5+i] = nbSlices+i;
    }
    mPolygonSize[nbSlices+0]  = nbSlices;
    mPolygonSize[nbSlices+1]  = nbSlices;
    CalcNormals();
}
void Base3DObject::GenerateSphere(int nbSlices, int nbStacks){
    Free();
    mNbVertices = (nbStacks-1)*nbSlices+2;
    mNbPolygons = nbSlices*(nbStacks);
    mPolygons    = new int      [2*nbSlices*3 + (nbStacks-2)*nbSlices*4];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int j=0;j<nbStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbStacks)*PI-PI/2.0;
        REALTYPE z = 0.5*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(i+j*nbSlices,0) = x;
            mVertices(i+j*nbSlices,1) = y;
            mVertices(i+j*nbSlices,2) = z;
        }
    }
    mVertices((nbStacks-1)*nbSlices+0,0) = mVertices((nbStacks-1)*nbSlices+1,0) = R_ZERO;
    mVertices((nbStacks-1)*nbSlices+0,1) = mVertices((nbStacks-1)*nbSlices+1,1) = R_ZERO;
    mVertices((nbStacks-1)*nbSlices+0,2) = -0.5;
    mVertices((nbStacks-1)*nbSlices+1,2) =  0.5;

    for(int j=0;j<nbStacks-2;j++){
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
            mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
            mPolygonSize[(i+j*nbSlices)]  = 4;
        }
        int i=nbSlices-1;
        mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
        mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
        mPolygonSize[(i+j*nbSlices)]  = 4;
    }
    for(int j=0;j<2;j++){
        int off = (nbStacks-2)*nbSlices;
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[off*4+i*3+0] = (nbStacks-1)*nbSlices+0;
            mPolygons[off*4+i*3+1] = i+1;
            mPolygons[off*4+i*3+2] = i;
            mPolygonSize[off+i]  = 3;

            mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks-1)*nbSlices+1;
            mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-2)*nbSlices+i;
            mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-2)*nbSlices+i+1;
            mPolygonSize[off+(nbSlices+i)]    = 3;
        }
        int i= nbSlices-1;
        mPolygons[off*4+i*3+0] = (nbStacks-1)*nbSlices+0;
        mPolygons[off*4+i*3+1] = 0;
        mPolygons[off*4+i*3+2] = i;
        mPolygonSize[off+i]  = 3;
        mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks-1)*nbSlices+1;
        mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-2)*nbSlices+i;
        mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-2)*nbSlices+0;
        mPolygonSize[off+(nbSlices+i)]    = 3;
    }
    CalcNormals();
}

void Base3DObject::GenerateCapsule(REALTYPE capRatio, int nbSlices, int nbSideStacks){
    Free();

    int nbStacks = 2*nbSideStacks;

    mNbVertices = (nbStacks-1)*nbSlices+2   + nbSlices;
    mNbPolygons = nbSlices*(nbStacks)       + nbSlices;
    mPolygons    = new int      [2*nbSlices*3 + (nbStacks-2)*nbSlices*4 + nbSlices*4];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int j=0;j<nbStacks;j++){
        int jj = (j<nbSideStacks?j:j-1);

        REALTYPE e = REALTYPE(jj+1)/REALTYPE(nbStacks)*PI-PI/2.0;
        REALTYPE z = 0.5*capRatio*(j<nbSideStacks?-1.0:1.0)+0.5*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(i+j*nbSlices,0) = x;
            mVertices(i+j*nbSlices,1) = y;
            mVertices(i+j*nbSlices,2) = z;
        }
    }
    mVertices((nbStacks)*nbSlices+0,0) = mVertices((nbStacks)*nbSlices+1,0) = R_ZERO;
    mVertices((nbStacks)*nbSlices+0,1) = mVertices((nbStacks)*nbSlices+1,1) = R_ZERO;
    mVertices((nbStacks)*nbSlices+0,2) = -0.5*capRatio -0.5;
    mVertices((nbStacks)*nbSlices+1,2) =  0.5*capRatio +0.5;

    for(int j=0;j<nbStacks-1;j++){
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
            mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+i+1;
            mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
            mPolygonSize[(i+j*nbSlices)]  = 4;
        }
        int i=nbSlices-1;
        mPolygons[(i+j*nbSlices)*4+0] = j*nbSlices+i;
        mPolygons[(i+j*nbSlices)*4+1] = j*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+2] = (j+1)*nbSlices+0;
        mPolygons[(i+j*nbSlices)*4+3] = (j+1)*nbSlices+i;
        mPolygonSize[(i+j*nbSlices)]  = 4;
    }
    //for(int j=0;j<2;j++){
        int off = (nbStacks-1)*nbSlices;
        for(int i=0;i<nbSlices-1;i++){
            mPolygons[off*4+i*3+0] = (nbStacks)*nbSlices+0;
            mPolygons[off*4+i*3+1] = i+1;
            mPolygons[off*4+i*3+2] = i;
            mPolygonSize[off+i]  = 3;

            mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks)*nbSlices+1;
            mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-1)*nbSlices+i;
            mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-1)*nbSlices+i+1;
            mPolygonSize[off+(nbSlices+i)]    = 3;
        }
        int i= nbSlices-1;
        mPolygons[off*4+i*3+0] = (nbStacks)*nbSlices+0;
        mPolygons[off*4+i*3+1] = 0;
        mPolygons[off*4+i*3+2] = i;
        mPolygonSize[off+i]  = 3;
        mPolygons[off*4+(nbSlices+i)*3+0] = (nbStacks)*nbSlices+1;
        mPolygons[off*4+(nbSlices+i)*3+1] = (nbStacks-1)*nbSlices+i;
        mPolygons[off*4+(nbSlices+i)*3+2] = (nbStacks-1)*nbSlices+0;
        mPolygonSize[off+(nbSlices+i)]    = 3;
    //}
    CalcNormals();
}

/*
void Base3DObject::GenerateCapsule(REALTYPE capRatio, int nbSlices, int nbSideStacks){
    Free();
    int offset=0;

    mNbVertices = nbSlices*2 + (nbSlices*(nbSideStacks-1)+1)*2;
    mNbPolygons = nbSlices   + (nbSlices*(nbSideStacks))    *2;
    mPolygons    = new int      [nbSlices*4+    (nbSlices*3 + (nbSideStacks-1)*nbSlices*4)*2];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    for(int i=0;i<nbSlices;i++){
        REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
        REALTYPE x = 0.5*cos(a);
        REALTYPE y = 0.5*sin(a);
        mVertices(i,0) = mVertices(i+nbSlices,0) = x;
        mVertices(i,1) = mVertices(i+nbSlices,1) = y;
        mVertices(i,2)                = -0.5;
        mVertices(i+nbSlices,2) =  0.5;
    }

    offset += 2*nbSlices;

    for(int j=0;j<nbSideStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbSideStacks)*PI/2.0;
        REALTYPE z = 0.5+0.5*capRatio*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(offset+i+j*nbSlices,0) = x;
            mVertices(offset+i+j*nbSlices,1) = y;
            mVertices(offset+i+j*nbSlices,2) = z;
        }
    }
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,0) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,1) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,2) = 0.5+0.5*capRatio;

    offset += (nbSideStacks-1)*nbSlices+1;

    for(int j=0;j<nbSideStacks-1;j++){
        REALTYPE e = REALTYPE(j+1)/REALTYPE(nbSideStacks)*PI/2.0;
        REALTYPE z = -0.5-0.5*capRatio*sin(e);
        for(int i=0;i<nbSlices;i++){
            REALTYPE a = REALTYPE(i)/REALTYPE(nbSlices)*TWOPI;
            REALTYPE x = 0.5*cos(a)*cos(e);
            REALTYPE y = 0.5*sin(a)*cos(e);
            mVertices(offset+i+j*nbSlices,0) = x;
            mVertices(offset+i+j*nbSlices,1) = y;
            mVertices(offset+i+j*nbSlices,2) = z;
        }
    }
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,0) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,1) = R_ZERO;
    mVertices(offset+(nbSideStacks-1)*nbSlices+0,2) = -0.5-0.5*capRatio;








    for(int i=0;i<nbSlices-1;i++){
        mPolygons[i*4+0] = i;
        mPolygons[i*4+1] = i+1;
        mPolygons[i*4+2] = nbSlices+i+1;
        mPolygons[i*4+3] = nbSlices+i;
        mPolygonSize[i]  = 4;
    }
    int i=nbSlices-1;
    mPolygons[i*4+0] = i;
    mPolygons[i*4+1] = 0;
    mPolygons[i*4+2] = nbSlices+0;
    mPolygons[i*4+3] = nbSlices+i;
    mPolygonSize[i]  = 4;








    CalcNormals();

}
*/


void Base3DObject::GenerateCube(){
    Free();
    mNbVertices = 8;
    mNbPolygons = 6;
    mPolygons    = new int      [mNbPolygons*4];
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);
    REALTYPE v[8][3] = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5,  0.5},
        {-0.5,  0.5,  0.5},
        {-0.5,  0.5, -0.5},
        { 0.5, -0.5, -0.5},
        { 0.5, -0.5,  0.5},
        { 0.5,  0.5,  0.5},
        { 0.5,  0.5, -0.5}};
    int f[6][4] = {
        {0,1,2,3},
        {4,5,1,0},
        {5,6,2,1},
        {7,3,2,6},
        {4,7,6,5},
        {7,4,0,3}};
    memcpy(mVertices.Array(),v,mNbVertices*3*sizeof(REALTYPE));
    memcpy(mPolygons,f,mNbPolygons*4*sizeof(int));
    for(int i=0;i<mNbPolygons;i++) mPolygonSize[i] = 4;
    CalcNormals();
}

void Base3DObject::GenerateHeightField(const Matrix &heightData,REALTYPE sx,REALTYPE sy,REALTYPE sz){
    Free();
    mNbVertices = heightData.RowSize()*heightData.ColumnSize() + 2*heightData.RowSize() + 2*heightData.ColumnSize() - 4;
    mNbPolygons = +1 + 2*(heightData.ColumnSize()-1) + 2*(heightData.RowSize()-1) +(heightData.RowSize()-1)*(heightData.ColumnSize()-1);

                  //5;
    mPolygons    = new int      [ (heightData.RowSize()-1)*(heightData.ColumnSize()-1)*4 +
                                  2*(heightData.ColumnSize()-1)*4+
                                  2*(heightData.RowSize()-1)*4+
                                  2*heightData.RowSize() + 2*heightData.ColumnSize() - 4];/* +
                                  2*(heightData.ColumnSize()+2) +
                                  4];*/
    mPolygonSize = new int      [mNbPolygons];
    mVertices.Resize(mNbVertices,3,false);

    int row = int(heightData.RowSize());
    int col = int(heightData.ColumnSize());

    REALTYPE cxo = (REALTYPE(col)-1.0)/2;
    REALTYPE cyo = (REALTYPE(row)-1.0)/2;
    int off = 0;
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++){
            mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
            mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
            mVertices(off,2) = heightData.AtNoCheck(i,j)*sz;
            off++;
        }
    }
    for(int j=0;j<col;j++){
        mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
        mVertices(off,1) = -cyo*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int i=1;i<row;i++){
        mVertices(off,0) = +cxo*sx;
        mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int j=col-2;j>=0;j--){
        mVertices(off,0) = (REALTYPE(j)-cxo)*sx;
        mVertices(off,1) = +cyo*sy;
        mVertices(off,2) = -0.0;
        off++;
    }
    for(int i=row-2;i>0;i--){
        mVertices(off,0) = -cxo*sx;
        mVertices(off,1) = (REALTYPE(i)-cyo)*sy;
        mVertices(off,2) = -0.0;
        off++;
    }

    int poff = 0;
    off = 0;
    for(int i=0;i<row-1;i++){
        for(int j=0;j<col-1;j++){
            mPolygons[off+0] = i*col+j;
            mPolygons[off+1] = i*col+(j+1);
            mPolygons[off+2] = (i+1)*col+(j+1);
            mPolygons[off+3] = (i+1)*col+j;

            mPolygonSize[poff] = 4;

            off+=4;
            poff++;
        }
    }

    int goff = row*col;
    for(int j=0;j<col-1;j++){
        mPolygons[off++] = j+1;
        mPolygons[off++] = j;
        mPolygons[off++] = goff+j;
        mPolygons[off++] = goff+j+1;
        mPolygonSize[poff++] = 4;
    }

    goff = row*col+col-1;
    for(int j=0;j<row-1;j++){
        mPolygons[off++] = col-1+(j+1)*col;
        mPolygons[off++] = col-1+j*col;
        mPolygons[off++] = goff+j;
        mPolygons[off++] = goff+j+1;
        mPolygonSize[poff++] = 4;
    }

    goff = row*col+col-1+row-1;
    for(int j=0;j<col-1;j++){
        mPolygons[off++] = row*col-2-j;
        mPolygons[off++] = row*col-j-1;
        mPolygons[off++] = goff+j;
        mPolygons[off++] = goff+j+1;
        mPolygonSize[poff++] = 4;
    }

    goff = row*col+col-1+row-1+col-1;
    for(int j=0;j<row-2;j++){
        mPolygons[off++] = (row-2-j)*col;
        mPolygons[off++] = (row-j-1)*col;
        mPolygons[off++] = goff+j;
        mPolygons[off++] = goff+j+1;
        mPolygonSize[poff++] = 4;
    }
    mPolygons[off++] = col;
    mPolygons[off++] = goff+row-2;
    mPolygons[off++] = row*col;
    mPolygons[off++] = 0;
    mPolygonSize[poff++] = 4;


    goff = row*col;
    for(int j=2*row+2*col-4-1;j>=0;j--){
        mPolygons[off++] = goff+j;
    }
    mPolygonSize[poff++] = 2*row+2*col-4;


    CalcNormals();
}
