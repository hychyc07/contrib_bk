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

#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef WIN32
  #include <windows.h>
#else
  #include <errno.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <fcntl.h>
  #include <unistd.h>
#endif



/**
 * \class Timer
 *
 * \ingroup icub_StdTools
 *
 * \brief A basic timer working on a milisecond time base
 */

class Timer
{
protected:

#ifndef WIN32
  struct timeval m_TV;
#endif

  long           m_TimeToGo;
  long           m_StartTime;

public:
  Timer();

  /// Start the timer with a given target time
  void  Start (long TimeToGoMs);
  /// Return true is the target time has been reached
  int   IsDone();

  static long GetTimeMs();
};

/**
 * \class Chrono
 *
 * \ingroup icub_StdTools
 *
 * \brief A basic chrono working on a milisecond time base
 */
class Chrono
{
protected:

#ifndef WIN32
  struct timeval m_TV;
  struct timeval m_StartTime;
  struct timeval m_PauseTime;
#else
  __int64        m_Frequency;
  __int64        m_StartTime;
  __int64        m_PauseTime;
#endif
  bool           bIsPaused;

public:
  Chrono();

  /// Start the chrono
  void  Start();

  void  Pause();
  void  Resume();
  /// Return the elapsed time in milliseconds
  double  ElapsedTime() const;
  /// Return the elapsed time in milliseconds
  long  ElapsedTimeMs() const;
  /// Useless
  long  ElapsedTimeUs() const;

#ifndef WIN32
  /// Return the time elapsed between the start of the chrono and the passed value
  double ElapsedTime(struct timeval baseTime) const;
#else
  /// Return the time elapsed between the start of the chrono and the passed value
  double ElapsedTime(__int64 baseTime) const;
#endif
};

class PerformanceEstimator
{
private:
    static Chrono               mChrono;
public:
    double                      mTime;
    double                      mMaxTime;
    double                      mMinTime;
    double                      mAccum;
    int                         mCounter;
    int                         mCount;
    double                      mCurrTime;
public:
    PerformanceEstimator(int count=10){
        mCount      = count;
        Reset();
    }
    void        SetCount(int count){mCount = count;}
    double      GetTime() const {return mTime;}
    double      GetMinTime() const {return mMinTime;}
    double      GetMaxTime() const {return mMaxTime;}
    void        Tic(){
        mCurrTime = mChrono.ElapsedTime();
    }
    void        Toc(){
        if(mCurrTime>=0){
            AddMeasurement(mChrono.ElapsedTime() - mCurrTime);
        }
    }
    void        AddMeasurement(double   time){
        mAccum += time;

        if(time>mMaxTime)                   mMaxTime = time;
        if((mMinTime < 0)||(time<mMinTime)) mMinTime = time;

        mCounter++;
        if(mCounter>mCount){
            mTime           = mAccum / double(mCount);
            mAccum          = 0.0;
            mCounter        = 0;
        }
    }
    void Reset(){
        mCurrTime   = -1;
        mTime       = 0.0;
        mAccum      = 0.0;
        mCounter    = 0;
        mMaxTime    = -1;
        mMinTime    = -1;
    }
};


#endif
