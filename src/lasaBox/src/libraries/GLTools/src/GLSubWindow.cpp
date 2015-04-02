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

#include "GLSubWindow.h"
#include "GLBaseWindow.h"
#include "GLTools.h"

#define TRUE  1
#define FALSE 0

Rect::Rect(){
  Set(0,0,0,0);
}

Rect::Rect(int x, int y, int w, int h){
  Set(x,y,w,h);
}

void Rect::Set(int x, int y, int w, int h){
  m_X       = x;
  m_Y       = y;
  m_Width   = w;
  m_Height  = h;
}
void Rect::Copy(pRect rect){
  if(rect!=NULL)
    Set(rect->m_X,rect->m_Y,rect->m_Width,rect->m_Height);
}

bool Rect::IsInside(int px, int py){
  return ((px>=m_X)&&(px<=m_X+m_Width)&&(py>=m_Y)&&(py<=m_Y+m_Height));
}


int        GLSubWindow::m_GridSize            = 16;
int        GLSubWindow::m_Title_Height        = 16;
int        GLSubWindow::m_Font_Size           = 14;

GLSubWindow::GLSubWindow(pGLSubWindow parentWindow){
  m_ParentWindow = parentWindow;
  if(m_ParentWindow!=NULL){
    m_ParentWidth  = m_ParentWindow->m_WindowRect.m_Width;
    m_ParentHeight = m_ParentWindow->m_WindowRect.m_Height;
  }else{
    m_ParentWidth  = GLBaseWindow::m_WindowWidth;
    m_ParentHeight = GLBaseWindow::m_WindowHeight;
  }

  SetSize(100,100);
  SetPos(10,10);
  m_Miminized = false;
  m_BackgroundColor[0]  = 0.5;
  m_BackgroundColor[1]  = 0.5;
  m_BackgroundColor[2]  = 0.5;
  m_BackgroundAlpha     = 0.75;
  m_Title               = "---";
  m_Move                = false;
  m_MoveStartX          = 0;
  m_MoveStartY          = 0;
  m_MouseButton         = -1;
  m_Resize              = false;
  m_ResizeStartX        = 0;
  m_ResizeStartY        = 0;
  m_HasFocus            = false;
  if(parentWindow!=NULL)
    parentWindow->AddSubWindow(this);

  m_MagnetHoriz     = 0;
  m_MagnetVert      = 0;
}

GLSubWindow::~GLSubWindow(){
  ClearChildrenWindows();
}

void GLSubWindow::AddSubWindow(pGLSubWindow win){
  if(win!=NULL){
    win->m_ParentWindow = this;
    m_ChildWindows.push_back(win);
  }
}

void  GLSubWindow::ClearChildrenWindows(){
  unsigned int i;
  for(i=0;i<m_ChildWindows.size();i++){
    delete m_ChildWindows[i];
  }
  m_ChildWindows.clear();
}
void  GLSubWindow::SetTitle(string title){
  m_Title = title;
}

void  GLSubWindow::GetAbsoluteRect(pRect rect){
  if(m_ParentWindow!=NULL){
    m_ParentWindow->GetAbsoluteRect(rect);
    rect->m_X += m_WindowRect.m_X;
    rect->m_Y += m_WindowRect.m_Y;
  }else{
    rect->m_X = m_WindowRect.m_X;
    rect->m_Y = m_WindowRect.m_Y;
  }
  rect->m_Width   = m_WindowRect.m_Width;
  rect->m_Height  = m_WindowRect.m_Height;
}

void  GLSubWindow::GetViewportRect(pRect rect){
  if(m_ParentWindow!=NULL){
    m_ParentWindow->GetAbsoluteRect(rect);
    rect->m_X += m_ClientRect.m_X;
    rect->m_Y += m_ClientRect.m_Y;
  }else{
    rect->m_X = m_ClientRect.m_X;
    rect->m_Y = m_ClientRect.m_Y;
  }
  rect->m_Width   = m_ClientRect.m_Width;
  rect->m_Height  = m_ClientRect.m_Height;

  rect->m_Y = GLBaseWindow::m_WindowHeight - rect->m_Y - m_ClientRect.m_Height;
}

void  GLSubWindow::SetSize(int w, int h){
  if(h<m_Title_Height) h = m_Title_Height;
  m_WindowRect.m_Width  = w;
  m_WindowRect.m_Height = h;

  m_ClientRect.m_Width  = w;
  m_ClientRect.m_Height = h - m_Title_Height;
  //Resize(w,h - m_Title_Height);
  CheckMagnets();
  OnResize(m_WindowRect.m_X,m_WindowRect.m_Y,m_WindowRect.m_Width,m_WindowRect.m_Height);
}

void  GLSubWindow::OnResize(int x, int y, int w, int h){
}

void  GLSubWindow::SetPos (int x, int y){
  m_WindowRect.m_X = x;
  m_WindowRect.m_Y = y;
  m_ClientRect.m_X = x;
  m_ClientRect.m_Y = y + m_Title_Height;

  CheckMagnets();
  OnResize(m_WindowRect.m_X,m_WindowRect.m_Y,m_WindowRect.m_Width,m_WindowRect.m_Height);
}
/*void  GLSubWindow::GetAbsolutePos(int *absX, int *absY){
  int tx = 0;
  int ty = 0;

  if((absX==NULL)||(absY==NULL))
    return;

  if(m_ParentWindow!=NULL){
    m_ParentWindow->GetAbsolutePos(tx,ty);
  }
  *absX = tx + m_WindowRect.m_X;
  *absY = ty + m_WindowRect.m_Y;
}*/

void  GLSubWindow::Minimize(bool state){
  m_Miminized = state;
  CheckMagnets();
}
int GLSubWindow::IsParentMinimized(){
  if(m_ParentWindow!=NULL){
    if(m_ParentWindow->m_Miminized)
      return TRUE;
    return m_ParentWindow->IsParentMinimized();
  }else{
    return FALSE;
  }
}
void  GLSubWindow::RenderWindow(){

  Rect absRect;
  GetAbsoluteRect(&absRect);

  int wHeight = GLBaseWindow::m_WindowHeight;//glutGet(GLUT_WINDOW_HEIGHT);

  if(m_Miminized){
    glViewport(absRect.m_X,
               wHeight-(absRect.m_Y+m_WindowRect.m_Height-m_ClientRect.m_Height),
               m_WindowRect.m_Width,
               m_WindowRect.m_Height-m_ClientRect.m_Height);
  }else{
    glViewport(absRect.m_X,
               wHeight-(absRect.m_Y+m_WindowRect.m_Height),
               m_WindowRect.m_Width,
               m_WindowRect.m_Height);
  }

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  if(m_Miminized){
    gluOrtho2D(0, m_WindowRect.m_Width, 0, m_WindowRect.m_Height-m_ClientRect.m_Height);
  }else{
    gluOrtho2D(0, m_WindowRect.m_Width, 0, m_WindowRect.m_Height);
  }
  glScalef(1.0f, -1.0f, 1.0f);
  if(m_Miminized){
    glTranslatef(0.0f, -(m_WindowRect.m_Height-m_ClientRect.m_Height), 0.0f);
  }else{
    glTranslatef(0.0f, -m_WindowRect.m_Height, 0.0f);
  }
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);

  if(!m_Miminized){
    glColor4f(m_BackgroundColor[0],
              m_BackgroundColor[1],
              m_BackgroundColor[2],
              m_BackgroundAlpha);
    glBegin(GL_QUADS);
      glVertex2i(                    0,                     0);
      glVertex2i( m_WindowRect.m_Width,                     0);
      glVertex2i( m_WindowRect.m_Width, m_WindowRect.m_Height);
      glVertex2i(                    0, m_WindowRect.m_Height);
    glEnd();
    glColor4f(m_BackgroundColor[0]*0.8f,
              m_BackgroundColor[1]*0.8f,
              m_BackgroundColor[2]*0.8f,
              m_BackgroundAlpha);
    glBegin(GL_LINE_LOOP);
      glVertex2i(                    0,                     0);
      glVertex2i( m_WindowRect.m_Width,                     0);
      glVertex2i( m_WindowRect.m_Width, m_WindowRect.m_Height);
      glVertex2i(                    0, m_WindowRect.m_Height);
    glEnd();
  }

  if(m_HasFocus){
    glColor4f(m_BackgroundColor[1]*1.4f,
              m_BackgroundColor[1]*1.4f,
              m_BackgroundColor[2]*1.4f,
              1.0f);
  }else{
    glColor4f(m_BackgroundColor[0],
              m_BackgroundColor[1],
              m_BackgroundColor[2],
              m_BackgroundAlpha);
  }

  glBegin(GL_QUADS);
    glVertex2i(                    0,                                           0);
    glVertex2i( m_WindowRect.m_Width,                                           0);
    glVertex2i( m_WindowRect.m_Width, m_WindowRect.m_Height-m_ClientRect.m_Height);
    glVertex2i(                    0, m_WindowRect.m_Height-m_ClientRect.m_Height);
  glEnd();
  if(m_HasFocus){
    glColor4f(m_BackgroundColor[1],
              m_BackgroundColor[1],
              m_BackgroundColor[2],
              1.0f);
  }else{
    glColor4f(m_BackgroundColor[0]*0.8f,
              m_BackgroundColor[1]*0.8f,
              m_BackgroundColor[2]*0.8f,
              m_BackgroundAlpha);
  }
  glBegin(GL_LINE_LOOP);
    glVertex2i(                    0,                                           0);
    glVertex2i( m_WindowRect.m_Width,                                           0);
    glVertex2i( m_WindowRect.m_Width, m_WindowRect.m_Height-m_ClientRect.m_Height);
    glVertex2i(                    0, m_WindowRect.m_Height-m_ClientRect.m_Height);
  glEnd();
  GLTools::DisplayText(3,m_WindowRect.m_Height-m_ClientRect.m_Height-2,m_Title.c_str(),m_Font_Size);

  glColor4f(0.0,0.0,0.0,1.0);
  /*glBegin(GL_LINE_LOOP);
    glVertex2i(                      0,                                             0);
    glVertex2i( m_WindowRect.m_Width-1,                                             0);
    glVertex2i( m_WindowRect.m_Width-1, m_WindowRect.m_Height-m_ClientRect.m_Height-1);
    glVertex2i(                      0, m_WindowRect.m_Height-m_ClientRect.m_Height-1);
  glEnd();
*/
  if(!m_Miminized){
    /*glBegin(GL_LINE_LOOP);
      glVertex2i(                      0,                       0);
      glVertex2i( m_WindowRect.m_Width-1,                       0);
      glVertex2i( m_WindowRect.m_Width-1, m_WindowRect.m_Height-1);
      glVertex2i(                      0, m_WindowRect.m_Height-1);
    glEnd();
*/
    glViewport(absRect.m_X,
               wHeight-(absRect.m_Y+m_WindowRect.m_Height),
               m_ClientRect.m_Width,
               m_ClientRect.m_Height);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, m_ClientRect.m_Width, 0, m_ClientRect.m_Height);
    glScalef(1.0f, -1.0f, 1.0f);
    glTranslatef(0.0f, -m_ClientRect.m_Height, 0.0f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    Render();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    for(unsigned int i=0;i<m_ChildWindows.size();i++)
      m_ChildWindows[i]->RenderWindow();
  }


  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void  GLSubWindow::SetFocus(bool focus){
  m_HasFocus = focus;
  if(focus==false)
    for(unsigned int i=0;i<m_ChildWindows.size();i++)
      m_ChildWindows[i]->SetFocus(focus);
}
void  GLSubWindow::RefreshWindow(){
  Refresh();
  for(unsigned int i=0;i<m_ChildWindows.size();i++){
    m_ChildWindows[i]->RefreshWindow();
  }
}


void  GLSubWindow::Render(){}
void  GLSubWindow::Refresh(){}
void  GLSubWindow::OnNormalKey(char key){}
void  GLSubWindow::OnSpecialKey(int key){}

int   GLSubWindow::OnClick(int button, int state, int x, int y){
  return FALSE;
}
int  GLSubWindow::OnDoubleClick(int button, int x, int y){
  return FALSE;
}
int  GLSubWindow::OnMove(int x, int y){
  return FALSE;
}


void  GLSubWindow::OnWindowNormalKey(char key){
  unsigned int i;
  for(i=0;i<m_ChildWindows.size();i++)
    m_ChildWindows[i]->OnWindowNormalKey(key);

  if(m_HasFocus)
    OnNormalKey(key);
}
void  GLSubWindow::OnWindowSpecialKey(int key){
  unsigned int i;
  for(i=0;i<m_ChildWindows.size();i++)
    m_ChildWindows[i]->OnWindowSpecialKey(key);

  if(m_HasFocus)
    OnSpecialKey(key);
}

int   GLSubWindow::OnWindowClick(int button, int state, int x, int y){
  int cFlag = FALSE;
  unsigned int i;

  if(IsParentMinimized())
    return FALSE;

  GLSubWindow *ActiveSubWin = NULL;
  for(i=0;i<m_ChildWindows.size();i++){
    int focus= m_ChildWindows[i]->OnWindowClick(button,state,x,y);
    if(focus)
      ActiveSubWin = m_ChildWindows[i];
    cFlag |= focus;
  }
  for(i=0;i<m_ChildWindows.size();i++){
    if(ActiveSubWin != m_ChildWindows[i])
      m_ChildWindows[i]->SetFocus(false);
  }

  Rect rect;
  GetAbsoluteRect(&rect);
  if(m_Miminized)
    rect.m_Height -= m_ClientRect.m_Height;
  if(state==GLBW_BTNDOWN){
    if(rect.IsInside(x,y)){
      m_HasFocus = true & (!cFlag);
    }else{
      m_HasFocus = false;
    }
  }

  if(cFlag)
    return TRUE;

  if(!m_HasFocus)
    return FALSE;

  if(state == GLBW_BTNDOWN){
    m_MouseButton = button;
    GetAbsoluteRect(&rect);
    rect.m_Height -= m_ClientRect.m_Height;
    if(rect.IsInside(x,y)){
      if(m_MouseButton == GLBW_LEFTBTN){
        m_MoveStartX = m_WindowRect.m_X-x;
        m_MoveStartY = m_WindowRect.m_Y-y;
        m_Move = true;
        return TRUE;
      }
      if(m_MouseButton == GLBW_RIGHTBTN){
        m_Resize = true;
        m_MoveStartX    = m_WindowRect.m_X;
        m_MoveStartY    = m_WindowRect.m_Y;
        m_ResizeStartX  = m_WindowRect.m_Width  - x*(m_MagnetHoriz&0x02?-1:1);
        m_ResizeStartY  = m_WindowRect.m_Height - y*(m_MagnetVert&0x02?-1:1);
        return TRUE;
      }
    }
  }else{
    m_MouseButton = -1;
    m_Resize      = false;
    m_Move        = false;
  }

  OnClick(button,state,x,y);

  return FALSE | m_HasFocus;
}
int  GLSubWindow::OnWindowDoubleClick(int button, int x, int y){
  int cFlag = FALSE;
  unsigned int i;
  for(i=0;i<m_ChildWindows.size();i++)
    cFlag |= m_ChildWindows[i]->OnWindowDoubleClick(button,x,y);

  if(cFlag)
    return TRUE;

  if(!m_HasFocus)
    return FALSE;


  if(button == GLBW_LEFTBTN){
    Rect rect;
    GetAbsoluteRect(&rect);//.Copy(&m_WindowRect);
    rect.m_Height -= m_ClientRect.m_Height;
    if(rect.IsInside(x,y)){
      Minimize(!m_Miminized);
      return TRUE;
    }
  }
  OnDoubleClick(button,x,y);
  return FALSE;
}

int  GLSubWindow::OnWindowMove(int x, int y){
  int cFlag = FALSE;
  unsigned int i;
  for(i=0;i<m_ChildWindows.size();i++)
    cFlag |= m_ChildWindows[i]->OnWindowMove(x,y);

  if(cFlag)
    return TRUE;

  if(!m_HasFocus)
    return FALSE;

  RefreshWindow();

  if(m_Move){
    SetPos(((m_MoveStartX+x)/m_GridSize)*m_GridSize,((m_MoveStartY+y)/m_GridSize)*m_GridSize);
    //return TRUE;
  }

  if(m_Resize && !m_Miminized){
    //SetPos(m_MoveStartX,m_MoveStartY);
    int sx = ((m_ResizeStartX+x*(m_MagnetHoriz&0x02?-1:1))/m_GridSize)*m_GridSize;
    int sy = ((m_ResizeStartY+y*(m_MagnetVert&0x02?-1:1))/m_GridSize)*m_GridSize;
    if((sx>=m_GridSize)&&(sy>=0)){
        SetSize(sx,sy);
    }
    //return TRUE;
  }

  return OnMove(x,y);
}

void  GLSubWindow::OnParentWindowResize(int w, int h){
    m_ParentWidth   = w;
    m_ParentHeight  = h;
    CheckMagnets();
}
void  GLSubWindow::SetHorizontalMagnets(int mask){
    m_MagnetHoriz = mask;
}
void  GLSubWindow::SetVerticalMagnets(int mask){
    m_MagnetVert = mask;
}

void  GLSubWindow::CheckMagnets(){
    if((m_MagnetHoriz==0)&&(m_MagnetVert==0))
        return;

    int w = m_ParentWidth;
    int h = m_ParentHeight;


    int newPosX  = m_WindowRect.m_X;
    int newPosY  = m_WindowRect.m_Y;
    int newSizeX = m_WindowRect.m_Width;
    int newSizeY = m_WindowRect.m_Height;
    bool hasToMove   = false;
    bool hasToResize = false;

    if(m_MagnetHoriz!=0){
        if(m_MagnetHoriz & 0x01){
            if(m_WindowRect.m_X != 0){
                newPosX   = 0;
                hasToMove = true;
            }
            if(m_MagnetHoriz & 0x02){
                newSizeX = w;
                hasToResize = true;
            }
        }else if(m_MagnetHoriz & 0x02){
            newPosX = w-m_WindowRect.m_Width;
            hasToMove = true;
        }
    }
    if(m_MagnetVert!=0){
        if(m_MagnetVert & 0x01){
            if(m_WindowRect.m_Y != 0){
                newPosY   = 0;
                hasToMove = true;
            }
            if(m_MagnetVert & 0x02){
                newSizeY = h;
                hasToResize = true;
            }
        }else if(m_MagnetVert & 0x02){
            if(m_Miminized){
                newPosY = h-m_Title_Height;
            }else{
                newPosY = h-m_WindowRect.m_Height;
                hasToMove = true;
            }
        }
    }
    if(hasToResize || hasToMove){

        if(newSizeY<m_Title_Height)
            newSizeY = m_Title_Height;

        m_WindowRect.m_X        = newPosX;
        m_WindowRect.m_Y        = newPosY;
        m_WindowRect.m_Width    = newSizeX;
        m_WindowRect.m_Height   = newSizeY;

        m_ClientRect.m_X        = m_WindowRect.m_X;
        m_ClientRect.m_Y        = m_WindowRect.m_Y + m_Title_Height;
        m_ClientRect.m_Width    = m_WindowRect.m_Width;
        m_ClientRect.m_Height   = m_WindowRect.m_Height - m_Title_Height;

        OnResize(m_WindowRect.m_X,m_WindowRect.m_Y,m_WindowRect.m_Width,m_WindowRect.m_Height);
    }
}
