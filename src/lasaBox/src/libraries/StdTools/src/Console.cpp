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

#include "Console.h"

#include "Various.h"

#include <iostream>
using namespace std;

Console::Command::Command(string name){
  m_Name     = name;
  m_FilePath = ".";
}
Console::Command::Command(string name, string filepath){
  m_Name     = name;
  m_FilePath = filepath;
}
Console::Command::~Command(){
}

int Console::Command::Execute(string args){
  return 0;
}

streambuf * Console::m_Stdout = NULL;

Console::Console(){
  if(m_Stdout == NULL)
    m_Stdout = cout.rdbuf();

  Free();
}
Console::~Console(){
  Free();

  if(cout.rdbuf() == GetStreamBuf())
    cout.rdbuf(m_Stdout);
}

void Console::Clear(){
  m_Lines.clear();
  m_History.clear();
  m_CurrY       = 0;
  m_CurrX       = 0;
  m_CurrCmd     = "";
  m_CurrHistory = 0;
  m_oStream.str("");
  m_CursorPos   = 0;
}

void Console::Free(){
  for(int i=0;i<m_OwnedCommands.size();i++)
    delete m_OwnedCommands[i];

  m_Commands.clear();
  m_OwnedCommands.clear();
  m_MaxLines    = 20;
  m_MaxHistory  = 20;
  Clear();
}

void Console::SetMaxLines(int size){
  if(size<=0)
    return;

  if(size>m_MaxLines)
    m_MaxLines = size;


  while(size<m_MaxLines){
    if(m_Lines.size()>(unsigned int)size)
      m_Lines.erase(m_Lines.begin());
    m_MaxLines--;
  }
}

void Console::SetMaxHistory(int size){
  if(size<=0)
    return;
  while(size<m_MaxHistory){
    m_History.erase(m_History.begin());
    m_CurrHistory--;
  }
}

void Console::AddLine(string line){
  if(m_Lines.size() >= (unsigned int)m_MaxLines){
    m_Lines.erase(m_Lines.begin());
    m_Lines.push_back(line);
  }else{
    m_Lines.push_back(line);
  }
}

void Console::AddHistory(string line){
  if(m_History.size() >= (unsigned int)m_MaxHistory){
    m_History.erase(m_History.begin());
    m_History.push_back(line);
  }else{
    m_History.push_back(line);
  }
  m_CurrHistory = m_History.size()-1;
}

void Console::MoveRight(bool bSkipWord){
  if(bSkipWord){
    bool spFound=false;
    bool bDone = false;
    for(int i=m_CursorPos;i<int(m_CurrCmd.size());i++){
      if(m_CurrCmd[i]==' '){
        spFound=true;
      }else{
        if(spFound==true){
          m_CursorPos=i;
          bDone = true;
          break;
        }
      }
    }
    if(!bDone)
      m_CursorPos=m_CurrCmd.size();
  }else{
    m_CursorPos++;
  }
  if(m_CursorPos>int(m_CurrCmd.size()))
    m_CursorPos=m_CurrCmd.size();
}

void Console::MoveLeft(bool bSkipWord){
 if(bSkipWord){
    bool bDone = false;
    bool spFound=false;
    bool txFound=false;
    for(int i=m_CursorPos-1;i>=0;i--){
      if(m_CurrCmd[i]==' '){
        spFound=true;
        if(txFound){
          m_CursorPos=i+1;
          bDone=true;
          break;
        }
      }else{
        txFound = true;
        /*if(spFound==true){
          m_CursorPos=i+2;
          bDone=true;
          break;
        }*/
      }
    }
    if(!bDone)
      m_CursorPos=0;
  }else{
    m_CursorPos--;
  }
  if(m_CursorPos<0)
    m_CursorPos=0;
 }

void Console::AddChar(char c){
  string s(1,c);
  //m_CurrCmd.append(s);
  m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos)+s+m_CurrCmd.substr(m_CursorPos,m_CurrCmd.size()-m_CursorPos);
  MoveRight();
}

void Console::EraseChar(bool bkw){
  if(bkw){
    if((m_CurrCmd.size()>0)&&(m_CursorPos>0)){
      //m_CurrCmd = m_CurrCmd.substr(0,m_CurrCmd.size()-1);
      m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos-1)+m_CurrCmd.substr(m_CursorPos,m_CurrCmd.size()-m_CursorPos);
      MoveLeft();
    }
  }else{
    if((m_CurrCmd.size()>0)&&(m_CursorPos<(int)m_CurrCmd.size())){
      m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos)+m_CurrCmd.substr(m_CursorPos+1,m_CurrCmd.size()-m_CursorPos);
    }
  }
}

void Console::ClearLine(){
  m_CurrCmd = "";
  m_CursorPos=m_CurrCmd.size();
}
void Console::Accept(bool bToHistory){
  if(bToHistory){
    if(m_CurrCmd.size()>0)
      AddHistory(m_CurrCmd);
  }

    string s = "> ";
    s.append(m_CurrCmd);
    Print(s);

    size_t cmdStart = m_CurrCmd.find_first_not_of(" ");
    if(cmdStart != string::npos){

        m_CurrCmd = m_CurrCmd.substr(cmdStart);
        size_t cmdEnd   = m_CurrCmd.find_first_of(" ");
        string cmd;
        string args;
        if(cmdEnd == string::npos){
            cmd   = m_CurrCmd;
            args  = "";
        }
        else{
            cmd   = m_CurrCmd.substr(0,cmdEnd);
            args  = m_CurrCmd.substr(cmdEnd+1);
        }

        pCommand pCmd = FindCommand(cmd);
        if(pCmd!=NULL){
            pCmd->Execute(args);
        }else{
            if(cmd.size()==0){
                Print(cmd);
            }else{
                string s = cmd;
                s = s.append(": Command not found");
                Print(s);
            }
        }
    }

  m_CurrCmd = "";
  m_CursorPos = 0;
}

void Console::Execute(string cmd, bool bToHistory){
  ClearLine();
  for(unsigned int i=0;i<cmd.size();i++){
    AddChar(cmd.at(i));
  }
  Accept(bToHistory);
}


Console::pCommand  Console::FindCommand(string name){
  unsigned int i;
  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.compare(name)==0)
      return m_Commands[i];
  }
  return NULL;
}

int Console::AutoCompletion(){
  unsigned int i;

  m_AutoMatches.clear();

  size_t cmdStart = m_CurrCmd.find_first_not_of(" ");
  string cmdToCompare = "";
  string cmdIndent    ="";
  if(cmdStart != string::npos){
    cmdToCompare = m_CurrCmd.substr(cmdStart);
    if(cmdStart>0){
      cmdIndent = m_CurrCmd.substr(0,cmdStart);
    }
  }else{
    cmdIndent = m_CurrCmd;
  }
  int len = cmdToCompare.size();

  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.substr(0,len).compare(cmdToCompare)==0)
      m_AutoMatches.push_back(i);
  }
  if(m_AutoMatches.size()==1){
    m_CurrCmd = cmdIndent + m_Commands[m_AutoMatches[0]]->m_Name;
    m_CurrCmd.append(" ");
    m_CursorPos = m_CurrCmd.size();
  }else if(m_AutoMatches.size()>1){
    int maxLen = len;
    while(1){
      string first = m_Commands[m_AutoMatches[0]]->m_Name;
      bool ok = true;
      for(i=1;i<m_AutoMatches.size();i++)
          ok &= (m_Commands[m_AutoMatches[i]]->m_Name.substr(0,maxLen+1).compare(first.substr(0,maxLen+1))==0);
      if(ok)
          maxLen++;
      else
          break;
    }
    string s = "> ";
    s.append(m_CurrCmd);
    Print(s);
    s = "";
    for(int i=0;i<int(m_AutoMatches.size());i++){
      if(i>0) s.append(" ");
      s.append(m_Commands[m_AutoMatches[i]]->m_Name);
    }
    Print(s);
    m_CurrCmd   = cmdIndent + m_Commands[m_AutoMatches[0]]->m_Name.substr(0,maxLen);
    m_CursorPos = m_CurrCmd.size();
  }

  return m_AutoMatches.size();
}

vector<string> Console::AutoCompleteFilename(){
  vector<string> results;
  unsigned int pos = m_CurrCmd.find_last_of(" ");
  if(pos!=string::npos){

    string sDir = ".";
    vector<string> cmds = Tokenize(m_CurrCmd);
    if(cmds.size()>=1){
      sDir = FindCommand(cmds[0])->m_FilePath;
    }

    vector<string> filesList = ScanDir(sDir);
    string target = "";

    if(m_CurrCmd[m_CurrCmd.size()-1]!=' '){
      target.append(m_CurrCmd.substr(pos+1));
    }

    results = ::AutoCompletion(filesList,target);
    if(results.size()==1){
      target.append(" ");
    }
    m_CurrCmd = m_CurrCmd.substr(0,pos+1);
    m_CurrCmd.append(target);
    m_CursorPos = m_CurrCmd.size();

  }
  return results;
}

void Console::AddCommand(pCommand cmd,bool bOwnership){
  if(cmd==NULL)
    return;
  m_Commands.push_back(cmd);
  if(bOwnership)
    m_OwnedCommands.push_back(cmd);
}

void Console::HistoryPrev(){
  if(m_History.size()>0){
    m_CurrCmd = m_History[m_CurrHistory];
    m_CursorPos = m_CurrCmd.size();
    m_CurrHistory--;
    if(m_CurrHistory<0)
      m_CurrHistory=0;
  }
}
void Console::HistoryNext(){
  if(m_History.size()>0){
    m_CurrHistory++;
    if(m_CurrHistory<(int)m_History.size()){
      m_CurrCmd = m_History[m_CurrHistory];
      m_CursorPos = m_CurrCmd.size();
    }else{
      m_CurrCmd = "";
      m_CursorPos = 0;
      m_CurrHistory--;
    }
  }
}

void Console::Print(string line){
  AddLine(line);
}

void Console::Update(){
  string s = m_oStream.str();

  unsigned int cpos = 0;
  unsigned int opos = 0;

  if(s.size()>0){
    //while(cpos!=string::npos){
    for(unsigned int i=0;i<s.size();i++){
      opos = cpos;
      cpos = s.find("\n",opos);
      string ss = s.substr(opos,cpos-opos);
      if(ss.size()>0)
        Print(ss);
      if(cpos==string::npos)
        break;
      cpos++;
    }
    m_oStream.str("");
  }
}

ostream  *Console::GetStream(){
  return &m_oStream;
}

streambuf *Console::GetStreamBuf(){
  return m_oStream.rdbuf();
}

void Console::SetStdout(){
  cout.rdbuf(GetStreamBuf());
}
/*
ofstream outFile("some_file_name.dat");//create output file object
streambuf * cout_backup=cout.rdbuf();//create backup of standard out
cout.rdbuf(outFile.rdbuf());//assign cout stream to outFile stream
cout<<"hello!"<<endl;//call any function that calls cout
cout.rdbuf(cout_backup);//restore the standard stream
outFile.close();//best to be neat and tidy about these things
*/
