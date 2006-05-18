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
/// TimeStamp stores an internal time value that represents the seconds
/// elapsed since January 1, 1970.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __TIMESTAMP_H__
#define __TIMESTAMP_H__

#ifdef WIN32
#include <sys/timeb.h>
#else
#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#else
#error Compiler does not support any of the default getCurrentTime() implementations in TimeStamp.h
#endif
#endif
#include <HAPI.h>
#include <Exception.h>


namespace H3D {
  struct HAPI_API TimeStamp {
    H3D_API_EXCEPTION( PerformanceCounterNotSupported );

    TimeStamp() {
      time = getCurrentTime();
    }
    TimeStamp( double _time ) : time( _time ) {}
        
    static TimeStamp now() {
      return TimeStamp( getCurrentTime() );
    }

        
    operator double() const { return time; }
    bool operator==( const TimeStamp &arg ) const { return time==arg.time; }
    bool operator!=( const TimeStamp &arg ) const { return time!=arg.time; }
    bool operator> ( const TimeStamp &arg ) const { return time>arg.time;  }
    bool operator< ( const TimeStamp &arg ) const { return time<arg.time;  }
    bool operator>=( const TimeStamp &arg ) const { return time>=arg.time; }
    bool operator<=( const TimeStamp &arg ) const { return time<=arg.time; }

  protected:
    double time; // seconds since epoch

    static double getCurrentTime();

#ifdef WIN32
    static bool init_done;
    static double start_time;
    static LARGE_INTEGER start_perf_count;
    static LARGE_INTEGER perf_freq;
#endif
  };
}


#endif
