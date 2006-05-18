//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file TimeStamp.h
/// \brief Routines to handle time stamping of the field network
///
//
//////////////////////////////////////////////////////////////////////////////

#include "TimeStamp.h"

using namespace H3D;

#ifdef WIN32
bool TimeStamp::init_done = false;
double TimeStamp::start_time = 0;
LARGE_INTEGER TimeStamp::start_perf_count;
LARGE_INTEGER TimeStamp::perf_freq;
#endif


#ifdef WIN32
double TimeStamp::getCurrentTime() {
  if( !init_done ) {
    struct __timeb64 timebuffer;
    _ftime64( &timebuffer );
    start_time = timebuffer.time + timebuffer.millitm / 1e3;
    if( !QueryPerformanceCounter( &start_perf_count ) ) {
      throw PerformanceCounterNotSupported( "", H3D_FULL_LOCATION );
    }
    QueryPerformanceFrequency( &perf_freq );
    init_done = true;
  }
  LARGE_INTEGER current_count;
  QueryPerformanceCounter( &current_count );
  
  LARGE_INTEGER diff;
  diff.QuadPart = current_count.QuadPart - start_perf_count.QuadPart;
  double seconds_since_start = 
    (double)diff.QuadPart / (double)perf_freq.QuadPart;
  return start_time + seconds_since_start;
}
#else
#ifdef HAVE_SYS_TIME_H
double TimeStamp::getCurrentTime() {
  struct timeval tp;
  struct timezone tzp;
  gettimeofday( &tp, &tzp );
  return tp.tv_sec + (double) tp.tv_usec / 1e6; 
}
#endif
#endif
