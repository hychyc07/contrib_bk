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

#ifndef __GLSUBWINDOW_H__
#define __GLSUBWINDOW_H__

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
using namespace std;




class Rect;
typedef Rect *pRect;
class Rect
{
public:
  int m_X,
      m_Y,
      m_Width,
      m_Height;
public:
  Rect();
  Rect(int x, int y, int w, int h);
  void Set(int x, int y, int w, int h);
  void Copy(pRect rect);
  bool IsInside(int px, int py);
};

class GLSubWindow;
typedef GLSubWindow *pGLSubWindow;
typedef vector<pGLSubWindow> GLSubWindow_List;

class GLSubWindow
{
public:
  static int        m_GridSize;
  static int        m_Title_Height;
  static int        m_Font_Size;

public:
  pGLSubWindow      m_ParentWindow;
  GLSubWindow_List  m_ChildWindows;

  int               m_ParentWidth;
  int               m_ParentHeight;

  Rect              m_WindowRect;
  Rect              m_ClientRect;

  float             m_BackgroundColor[3];
  float             m_BackgroundAlpha;

  bool              m_Miminized;

  bool              m_HasFocus;

  string            m_Title;

  bool              m_Move;
  int               m_MoveStartX;
  int               m_MoveStartY;
  int               m_MouseButton;

  bool              m_Resize;
  int               m_ResizeStartX;
  int               m_ResizeStartY;

  int               m_MagnetHoriz;
  int               m_MagnetVert;

public:
          GLSubWindow(pGLSubWindow parentWindow = NULL);
  virtual ~GLSubWindow();

          void  ClearChildrenWindows();

          void  GetAbsoluteRect(pRect rect);
          void  GetViewportRect(pRect rect);

          void  SetSize(int w, int h);
          void  SetPos (int x, int y);
          void  Minimize(bool state);

          //void  GetAbsolutePos(int *absX, int *absY);

          void  RenderWindow();
          void  RefreshWindow();

          void  SetTitle(string title);
          void  SetFocus(bool focus);
          void  SetHorizontalMagnets(int mask);
          void  SetVerticalMagnets(int mask);
          void  AddSubWindow(pGLSubWindow win);

  virtual void  Render();

  virtual void  Refresh();

  virtual void  OnNormalKey(char key);
  virtual void  OnSpecialKey(int key);
  virtual int   OnClick(int button, int state, int x, int y);
  virtual int   OnDoubleClick(int button, int x, int y);
  virtual int   OnMove(int x, int y);
  virtual void  OnResize(int x, int y, int w, int h);

          void  OnWindowNormalKey(char key);
          void  OnWindowSpecialKey(int key);
          int   OnWindowClick(int button, int state, int x, int y);
          int   OnWindowDoubleClick(int button, int x, int y);
          int   OnWindowMove(int x, int y);
          void  OnParentWindowResize(int w, int h);

          void  CheckMagnets();

          int   IsParentMinimized();
};

#endif
