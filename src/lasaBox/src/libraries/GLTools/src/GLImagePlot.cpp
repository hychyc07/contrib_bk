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

#ifdef WIN32
#pragma warning( disable : 4786)
#endif

#include <GL/glut.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string>
using namespace std;


#include "GLImagePlot.h"
#include "GLTools.h"

GLImagePlot::GLImagePlot(pGLSubWindow parentWindow)
: GLSubWindow(parentWindow){
  m_Buffer    = NULL;
  m_Val       = NULL;
  Clear();
}

GLImagePlot::~GLImagePlot(){
  Clear();
}

void GLImagePlot::Clear(){
  m_SizeX     = 0;
  m_SizeY     = 0;
  m_Type      = 0;
  m_Depth     = 0;


  if(m_Buffer!=NULL)
    delete [] m_Buffer;
  m_Buffer    = NULL;

  m_Val       = NULL;
}


void GLImagePlot::SetImage(unsigned char* Val, int sizeX, int sizeY, int depth){
  if((sizeX<=0)||(sizeY<=0)||(depth<=0)||(depth>4))
    return;

  if(Val==NULL)
    return;
  
  if(m_Buffer!=NULL)
    delete [] m_Buffer;

  m_SizeX   = sizeX;
  m_SizeY   = sizeY;
  m_Depth   = depth;
  m_Val     = (void*)Val;
  m_Type    = 1;

  m_Buffer  = new float[sizeX*sizeY*4];
}

void GLImagePlot::SetImage(float* Val, int sizeX, int sizeY, int depth){
  if((sizeX<=0)||(sizeY<=0)||(depth<=0)||(depth>4))
    return;

  if(Val==NULL)
    return;

  if(m_Buffer!=NULL)
    delete [] m_Buffer;

  m_SizeX   = sizeX;
  m_SizeY   = sizeY;
  m_Depth   = depth;
  m_Val     = (void*)Val;
  m_Type    = 2;

  m_Buffer  = new float[sizeX*sizeY*4];
}


void GLImagePlot::Render(){

  if(m_Val==NULL)
    return;
  
  int vp[4];
  glGetIntegerv(GL_VIEWPORT,vp);
  glViewport(vp[0],vp[1]+1,vp[2],vp[3]);

  glDisable(GL_DEPTH_TEST);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  //glRasterPos2i(m_ClientRect.m_Width-2,0);
  //glRasterPos2i(2,m_ClientRect.m_Height-2);
  glPixelZoom(((float)m_ClientRect.m_Width)/m_SizeX,-((float)m_ClientRect.m_Height)/m_SizeY);
  //printf("PZ %f %f\n", ((float)m_ClientRect.m_Width)/m_SizeX,-((float)m_ClientRect.m_Height)/m_SizeY);
  GLenum format=GL_LUMINANCE,type=GL_UNSIGNED_BYTE;

  if(m_Depth==1)
    format = GL_LUMINANCE;
  else if(m_Depth==2)
    format = GL_LUMINANCE_ALPHA;
  else if(m_Depth==3)
    format = GL_RGB;
  else if(m_Depth==4)
    format = GL_RGBA;

  if(m_Type==1)
    type = GL_UNSIGNED_BYTE;
  else if(m_Type==2)
    type = GL_FLOAT;
  
  //printf("PZ %f %f %d %d\n", ((float)m_ClientRect.m_Width)/m_SizeX,-((float)m_ClientRect.m_Height)/m_SizeY,m_Depth,m_Type);
  
  
  glRasterPos2i(0,1);
  glDrawPixels(m_SizeX,m_SizeY,format,type,m_Val);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
	
  glEnable(GL_DEPTH_TEST);
  glViewport(vp[0],vp[1],vp[2],vp[3]);

}

void  GLImagePlot::OnNormalKey(char key){  
}

void  GLImagePlot::OnSpecialKey(int key){
}

void  GLImagePlot::Resize(int w, int h){
}
