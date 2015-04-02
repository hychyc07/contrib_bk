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

#include "Various.h"

#include <stdio.h>
#include <dirent.h>
#include <string.h>

#ifdef WIN32
#pragma warning( disable : 4996)
#include <windows.h>
#else
#include <sys/time.h>
#include <sys/stat.h>
#endif


string Int01ToString(int i){
  char tmp[256];
  sprintf(tmp,"%01d",i);
  return tmp;
}

string Int02ToString(int i){
  char tmp[256];
  sprintf(tmp,"%02d",i);
  return tmp;
}

string Int03ToString(int i){
  char tmp[256];
  sprintf(tmp,"%03d",i);
  return tmp;
}

string Int04ToString(int i){
  char tmp[256];
  sprintf(tmp,"%04d",i);
  return tmp;
}

string IntToString(int i){
  char tmp[256];
  sprintf(tmp,"%d",i);
  return tmp;
}
string FloatToString(float f){
  char tmp[256];
  sprintf(tmp,"%f",f);
  return tmp;
}
string DoubleToString(double d){
  char tmp[256];
  sprintf(tmp,"%f",d);
  return tmp;
}
string BoolToString(bool b){
  char tmp[256];
  if(b) sprintf(tmp,"true");
  else sprintf(tmp,"false");
  return tmp;
}

string RemoveSpaces(string s){
  basic_string <char>::size_type pos;
  if(s.size()==0)
    return s;

  string ms = s;
  pos = ms.find_first_of(" \r\t\n");
  while(pos==0){
    ms = ms.substr(1);
    pos = ms.find_first_of(" \r\t\n");
  }
  pos = ms.find_last_of(" \r\t\n");
  while(pos==ms.size()-1){
    ms = ms.substr(0,ms.size()-1);
    pos = ms.find_last_of(" \r\t\n");
  }
  return ms;
}

string GetPathFromFilename(string fname){
  string path="";
  unsigned int s = fname.length();
  int slashPos = -1;

  for(unsigned int i=0;i<s;i++){
    if(fname[i]=='/')
      slashPos = i;
  }
  if(slashPos==-1)
    return path;

  path = fname.substr(0,slashPos);
  return path;
}

string GetFileFromFilename(string fname){
  string filename="";
  unsigned int s = fname.length();
  int slashPos = -1;

  for(unsigned int i=0;i<s;i++){
    if(fname[i]=='/')
      slashPos = i;
  }
  if(slashPos==-1)
    return fname;

  filename = fname.substr(slashPos+1);
  return filename;
}


vector<string> Tokenize(string params){

  vector<string> tokens;
  tokens.clear();

  size_t endPos   = 0;
  int startPos = 0;
  while(endPos!=string::npos){
    endPos = params.find(" ",startPos);
    if(endPos!=string::npos){
      if(endPos-startPos>=1)
        tokens.push_back(params.substr(startPos,endPos-startPos));
      startPos = endPos+1;
    }
  }
  if(startPos<(int)params.size())
    tokens.push_back(params.substr(startPos));

  return tokens;
}

vector<string> Tokenize(string params, string delim, string exDelim){

  vector<string> tokens;
  tokens.clear();

  size_t endPos   = 0;
  int startPos = 0;
  while(endPos!=string::npos){
    endPos = params.find_first_of(delim,startPos);
    if(endPos!=string::npos){
      if(endPos-startPos>=1)
        tokens.push_back(params.substr(startPos,endPos-startPos));
      string cDelim = params.substr(endPos,1);
      if(cDelim.find_first_of(exDelim,0)==string::npos)
        tokens.push_back(cDelim);
      startPos = endPos+1;
    }
  }
  if(startPos<(int)params.size())
    tokens.push_back(params.substr(startPos));

  return tokens;
}

string Serialize(vector<string> tokens, int start, int cnt){
  string res;
  if(tokens.size()<=0)
    return res;

  if(((int)tokens.size())<=start)
    return res;

  if(cnt==-1){
    cnt = tokens.size() - start;
  }

  if(cnt > ((int)tokens.size()) - start)
    return res;

  res = tokens[start];
  for(int i=start+1;i<start+cnt;i++){
    res += " ";
    res +=tokens[i];
  }
  return res;
}

#ifdef WIN32          // Linux Wrap for Win32

#else                 // Windows warp for linux

long int GetTickCount(){
  long int result;
  struct timeval mCTV;
  gettimeofday(&mCTV,NULL);
  result  = mCTV.tv_sec *1000;
  result += mCTV.tv_usec/1000;
  return result;
}

#endif

#ifdef WIN32

vector<string> ScanDir(string dirName){

  BOOL            fFinished;
  HANDLE          hList;
  TCHAR           szDir[MAX_PATH+1];
  WIN32_FIND_DATA FileData;

  vector<string>  result;

  // Get the proper directory path
  sprintf(szDir, "%s/*", dirName.c_str());

  // Get the first file
  hList = FindFirstFile(szDir, &FileData);
  if (hList == INVALID_HANDLE_VALUE){
    return result;
  }else{

    // Traverse through the directory structure
    fFinished = FALSE;
    while (!fFinished){
      // Check the object is a directory or not
      if (FileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        if ((strcmp(FileData.cFileName, ".") != 0) && (strcmp(FileData.cFileName, "..") != 0)){
          string sDir = FileData.cFileName;
          sDir.append("/");
          result.push_back(sDir);
        }
      }else{
        result.push_back(FileData.cFileName);
      }

      if (!FindNextFile(hList, &FileData)){
        //if (GetLastError() == ERROR_NO_MORE_FILES){
        fFinished = TRUE;
        //}
      }
    }
  }
  FindClose(hList);
  return result;
}

#else
vector<string> ScanDir(string dirName){
  vector<string>  result;
  return result;
}

#endif


vector<string> AutoCompletion(vector<string> & choices, string & target){
  vector<string> result;
  unsigned int i;
  int len = target.size();

  for(i=0;i<choices.size();i++){
    if(choices[i].substr(0,len).compare(target)==0)
      result.push_back(choices[i]);
  }

  if(result.size()==1){
    target = result[0];
    target.append(" ");

  }else if(result.size()>1){
    int maxLen = len;
    while(1){
      string first = result[0];
      bool ok = true;
      for(i=1;i<result.size();i++)
	      ok &= (result[i].substr(0,maxLen+1).compare(first.substr(0,maxLen+1))==0);
      if(ok)
	       maxLen++;
      else
	      break;
    }
    target = result[0].substr(0,maxLen);
  }

  return result;
}


int GetConsecutiveFileCount(const char * dir, const char *nameTag, int maxS){
    DIR *dirp;
    struct dirent *dent;
    int i;

    dirp = opendir(dir);
    if (!dirp)
        return 0;

    int res = 0;

    for(int i=0;i<maxS;i++){
        char cName[256];
        sprintf(cName,nameTag,i);

        bool bFound = false;
        seekdir(dirp,0);
        while ((dent = readdir(dirp)) != NULL)
        {
            if(strcmp(cName,dent->d_name)==0){
                bFound = true;
                break;
            }
        }
        if(bFound){
            res = i+1;
        }else{
            break;
        }
    }

    closedir(dirp);
    return res;
}
bool FileExists(string strFilename) {
    struct stat stFileInfo;
    int intStat;

    // Attempt to get the file attributes
    intStat = stat(strFilename.c_str(),&stFileInfo);
    if(intStat == 0) {
        return true;
    }else{
        return false;
    }
}
