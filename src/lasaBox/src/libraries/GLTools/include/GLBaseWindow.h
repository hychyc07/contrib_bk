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

#ifndef __GLBASEWINDOW_H__
#define __GLBASEWINDOW_H__

#include <stdlib.h>

#include "StdTools/Timer.h"
#include "GL2DPlot.h"
#include "GLSubWindow.h"
#include "GLCamera.h"

#define GLBW_BTNUP      0
#define GLBW_BTNDOWN    1
#define GLBW_LEFTBTN    0
#define GLBW_RIGHTBTN   1

#define GLBW_KEY_UP     0x100
#define GLBW_KEY_DOWN   0x101
#define GLBW_KEY_LEFT   0x102
#define GLBW_KEY_RIGHT  0x103


class GLBaseWindow;
typedef GLBaseWindow *pGLBaseWindow;

class GLBaseWindow
{
protected:

  // Mouse tracking
  int               m_CurrentMouseButton;
  int               m_CurrentMouseX;
  int               m_CurrentMouseY;

  // Snapshots
  //GLSnapshot        m_Snapshot;
  //bool              m_Snap;

  // Console
  //GLConsole         m_Console;

  // Double click Timer
  Timer             m_DblClickTimer;
  int               m_LastPressedButton;

  int               m_GridSize;

  Timer             m_FrameRateTimer;
  int               m_FrameRate;
  int               m_FrameRateCounter;

public:
  GLBaseWindow();
//  GLBaseWindow(int argc, char **argv, pTree config);
  virtual ~GLBaseWindow();

  virtual void  Init();
  virtual void  Render();
  virtual void  Free();
  virtual void  OnIdle();
  virtual void  Resize          (int width, int height);
  virtual void  InputMouseButton(int button, int state, int x, int y);
  virtual void  InputMouseMotion(int x, int y);
  virtual void  InputNormalKey  (unsigned char key, int x, int y);
  virtual void  InputSpecialKey (int key, int x, int y);

          void  SetCurrent();

          //void  Snap(char *filename, int x=-1, int y=-1,int w=100, int h=100, GLSnapshotMode mode=GLSnap_RGB);
          //void  Snap(char *filename, GLSnapshotMode mode);

          void  AddSubWindows(pGLSubWindow win);

          int   GetFrameRate();

public:
          void  BaseFree();
          void  BaseRender();
          void  BaseResize          (int width, int height);
          void  BaseInputNormalKey  (unsigned char key, int x, int y);
          void  BaseInputSpecialKey (int key, int x, int y);
          int   BaseInputMouseButton(int button, int state, int x, int y);
          void  BaseInputMouseMotion(int x, int y);

protected:
  GLSubWindow_List  m_SubWindows;
  GLSubWindow      *m_CurrSubWindow;

public:
  static int    m_WindowWidth;
  static int    m_WindowHeight;

private:
  static  void  GLBWin_Render();
  static  void  GLBWin_Resize(int width, int height);
  static  void  GLBWin_InputMouseButton(int button, int state, int x, int y);
  static  void  GLBWin_InputMouseMotion(int x, int y);
  static  void  GLBWin_InputNormalKey(unsigned char key, int x, int y);
  static  void  GLBWin_InputSpecialKey(int key, int x, int y);

  static  pGLBaseWindow    m_GLBWin;
};





#endif
