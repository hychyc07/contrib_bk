/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zmathews@iua.upf.edu   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "timer.h"

Timer::Timer()
{

	
}


Timer::~Timer()
{
}

void Timer::setActiveTime(){
	struct timeval tim1;
	struct timezone tz;
	gettimeofday(&tim1, &tz);
	previousActiveTime = (double)tim1.tv_sec + (double) (tim1.tv_usec)/1000000.0; // seconds
	
}

double Timer::getInactiveTime(){
	struct timeval tim1;
	struct timezone tz;
	gettimeofday(&tim1, &tz);
	double time2 = (double)tim1.tv_sec + (double) (tim1.tv_usec)/1000000.0;
	double delay = time2 - previousActiveTime;
	//if (delay < 0){delay = delay * -1;}
	return delay;



}



