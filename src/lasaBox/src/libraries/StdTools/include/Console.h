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

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#ifdef WIN32
#pragma warning( disable : 4786)
#endif

#include <vector>
#include <string>
#include <sstream>
using namespace std;

typedef vector<string> String_List;



class Console
{
public:
    class Command {
    public:
      string  m_Name;
      string  m_FilePath;
    public:
                    Command(string name);
                    Command(string name, string filepath);
      virtual       ~Command();
      virtual int   Execute(string args);
    };
    typedef Command *pCommand;
    typedef vector<pCommand> Command_List;


private:
  static streambuf *    m_Stdout;


protected:
  Command_List          m_Commands;
  Command_List          m_OwnedCommands;

  String_List           m_Lines;
  int                   m_MaxLines;
  String_List           m_History;
  int                   m_MaxHistory;
  int                   m_CurrHistory;

  string                m_CurrCmd;

  vector<int>           m_AutoMatches;


  int                   m_CurrX;
  int                   m_CurrY;
  int                   m_CursorPos;

  ostringstream         m_oStream;

public:
            Console();
    virtual   ~Console();

            void      Free();
            void      Clear();
            void      SetMaxLines(int size);
            void      SetMaxHistory(int size);

            void      AddLine(string line);
            void      AddHistory(string line);

            void      AddCommand(pCommand cmd, bool bOwnership=false);
            pCommand  FindCommand(string name);

            void      HistoryPrev();
            void      HistoryNext();

            void      MoveRight(bool bSkipWord=false);
            void      MoveLeft(bool bSkipWord=false);


            void      AddChar(char c);
            void      EraseChar(bool bkw = true);
            void      ClearLine();
            void      Accept(bool bToHistory = true);
    virtual int       AutoCompletion();
            vector<string>    AutoCompleteFilename();

            void      Execute(string cmd, bool bToHistory = true);

            void      Print(string line);
            void      Update();

            ostream   *GetStream();

            streambuf *GetStreamBuf();
            void      SetStdout();

};


#endif
