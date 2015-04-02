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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

#include "GLConsole.h"
#include "GLTools.h"

GLConsole::GLConsole(pGLSubWindow parentWindow)
: Console(),GLSubWindow(parentWindow){
  m_CursorStatus  = 1;
  m_CursorTimer.Start(500);
  SetMaxLines(20);
  m_DisplayOffset = m_Font_Size;
}

GLConsole::~GLConsole(){
}



void GLConsole::Render(){

  Update();
/*
  glColor4f(0.0,0.0,0.0,1.0);
  glBegin(GL_LINE_LOOP);
  glVertex2i(   0, (m_MaxLines)*m_Font_Size+8);
  glVertex2i( m_ClientRect.m_Width-1, (m_MaxLines)*m_Font_Size+8);
  glVertex2i( m_ClientRect.m_Width-1, (m_MaxLines+1)*m_Font_Size+12);
  glVertex2i(   0, (m_MaxLines+1)*m_Font_Size+12);
  glEnd();
*/
  glDisable(GL_DEPTH_TEST);

  int y = m_DisplayOffset;//m_Font_Size;
  int i;
  string s;

  y += m_Font_Size*(m_MaxLines-m_Lines.size());
  for(i=0;i<(int)m_Lines.size();i++){
    GLTools::DisplayText(1,y, m_Lines[i].c_str(),m_Font_Size);
    y += m_Font_Size;
  }

  s = "> ";
  s.append(m_CurrCmd);

  if(m_HasFocus){
    if(m_CursorStatus==1){
      s.append("_");
    }
    if(m_CursorTimer.IsDone()){
      m_CursorStatus = 1-m_CursorStatus;
      m_CursorTimer.Start(500);
    }
  }

  GLTools::DisplayText(1,y, s.c_str(),m_Font_Size);

  glEnable(GL_DEPTH_TEST);

}

void  GLConsole::OnNormalKey(char key){
  if(key==3) // Ctrl+C
    m_CurrCmd = "";

  if((key>=' ') && (key<'~')){
    AddChar(key);
  }
  if((key==8)||(key==127))
    EraseChar();

  if(key==13)
    Accept();

  if(key=='\t')
    AutoCompletion();

}
void  GLConsole::OnSpecialKey(int key){
  switch (key) {
  case GLBW_KEY_UP:
    HistoryPrev(); break;
  case GLBW_KEY_DOWN:
    HistoryNext(); break;
  }
}

void  GLConsole::OnResize(int x, int y, int w, int h){
  int mlines = (h-m_Title_Height)/(m_Font_Size);
  m_DisplayOffset = (h-m_Title_Height) - m_Font_Size*mlines;
  mlines = (mlines>1?mlines:1);
  SetMaxLines(mlines);
}
