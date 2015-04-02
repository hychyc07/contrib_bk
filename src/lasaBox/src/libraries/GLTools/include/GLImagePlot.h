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

#ifndef __GLIMAGEPLOT_H__
#define __GLIMAGEPLOT_H__


#include <GL/glut.h>
#include <vector>
using namespace std;

#include "GLSubWindow.h"
#include "GL2DPlot.h"

#define COLORMAP_SIZE   64

class GLImagePlot : public GLSubWindow
{
protected:
  int               m_SizeX;
  int               m_SizeY;
  int               m_Depth;
  int               m_Type;

  void *            m_Val;
  float *           m_Buffer;

public:
  GLImagePlot(pGLSubWindow parentWindow = NULL);
  ~GLImagePlot();

          void  Clear();
          void  SetImage(float* Val,         int sizeX, int sizeY, int depth);
          void  SetImage(unsigned char* Val, int sizeX, int sizeY, int depth);

          void  CopyImage(float* Val,         int sizeX, int sizeY, int depth);
          void  CopyImage(unsigned char* Val, int sizeX, int sizeY, int depth);

  virtual void  Render();

  virtual void  Resize(int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

};
typedef GLImagePlot *pGLImagePlot;

#endif
