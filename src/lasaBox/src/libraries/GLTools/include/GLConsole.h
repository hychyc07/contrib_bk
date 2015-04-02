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

#ifndef __GLCONSOLE_H__
#define __GLCONSOLE_H__



#include "GLBaseWindow.h"
#include "GLSubWindow.h"
#include "StdTools/Console.h"
#include "StdTools/Timer.h"


class GLConsole : public Console, public GLSubWindow
{
protected:
  int           m_CursorStatus;
  Timer         m_CursorTimer;
  int           m_DisplayOffset;
  //int   m_Width;
  //int   m_Height;
public:
  GLConsole(pGLSubWindow parentWindow = NULL);
  ~GLConsole();

  virtual   void  Render();
//            void  Resize(int width, int height);

  virtual void  OnResize(int x, int y, int w, int h);
  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);

};
typedef GLConsole *pGLConsole;

#endif
