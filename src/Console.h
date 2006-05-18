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
/// \file Console.cpp
/// \brief Debug console stream
///
//  This node provides a static stream that all error and warning messages
//  are sent to. The stream can be redirected to any arbitrary stream, and
//  defaults to cerr. The stream can be controlled to set a minimum 
//  error level to show (setting 0 will show all messages). The default level
//  is 3.
//
//  Example usage:
//
//  Console.setOutputStream( cout );
//  Console.setOutputLevel( 2 );
//  Console(0) << "Warning:" << endl;
//  Console    << " This is a warning" << endl;
//  Console    << " This is still the same warning" << endl;
//
//  Console(1) << "Level 1" << endl;
//  Console(2) << "Level 2" << endl;
//  Console.setShowTime( true );
//  Console(3) << "Level 3, with time" << endl;
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <ostream>
#include <sstream>
#include <string>
#include <iostream>
#include "H3DApi.h"
#include "TimeStamp.h"

using namespace std;


namespace H3D {

  template <class CharT, class TraitsT = std::char_traits<CharT> >
  class basic_debugbuf : public std::basic_stringbuf<CharT, TraitsT>  {
    int outputlevel, level;
    ostream *outputstream;
    TimeStamp starttime;
    bool showtime;
    bool showlevel;
  public:
    basic_debugbuf(  ) : 
      outputstream( &cerr ),
      outputlevel( 3 ),
      level( 0 ),
      showtime(false),
      showlevel( true ) {
    }

    virtual ~basic_debugbuf() {
      outputlevel=-1;
      sync();
    }

    void setShowTime( bool show ) { showtime = show; }

    void setShowLevel( bool show ) { showlevel = show; }

    void setOutputStream( ostream &s ) { outputstream = &s; }

    void setOutputLevel( int _outputlevel ) { outputlevel = _outputlevel; }

    void setLevel( int _level ) { level = _level; }
  
  protected:

    int sync()  {
      TimeStamp time;

      if ( outputlevel >= 0  &&  level >= outputlevel ) {
        if ( showlevel || showtime )
          *outputstream << "[";
        if ( showlevel )
          *outputstream << level;
        if ( showlevel && showtime )
          *outputstream << " - ";
        if ( showtime ) {
          outputstream->width(10);
          *outputstream << (time-starttime);
        }
        if ( showlevel || showtime )
          *outputstream << "] ";
        *outputstream << std::basic_stringbuf<CharT, TraitsT>::str().c_str(); 
      }
      str(std::basic_string<CharT>());    // Clear the string buffer
      return 0;
    }
  
  };


  template<class CharT, class TraitsT = std::char_traits<CharT> >
  class basic_dostream : public std::basic_ostream<CharT, TraitsT> {
  public:
  
    basic_dostream() : 
      std::basic_ostream<CharT, TraitsT>(new basic_debugbuf<CharT, TraitsT>()) {
    }

    ~basic_dostream() {
      delete std::ios::rdbuf(); 
    }

    void setShowTime( bool show ) { 
      static_cast< basic_debugbuf<CharT, TraitsT>* >(std::ios::rdbuf())->
        setShowTime( show );  
    }

    void setShowLevel( bool show ) { 
      static_cast< basic_debugbuf<CharT, TraitsT>* >(std::ios::rdbuf())->
        setShowLevel( show );  
    }

    void setOutputStream( ostream &s ) { 
      static_cast< basic_debugbuf<CharT, TraitsT>* >(std::ios::rdbuf())->
        setOutputStream( s );  
    }

    void setOutputLevel( int _outputlevel ) {
      static_cast<basic_debugbuf<CharT, TraitsT>* >( std::ios::rdbuf() )->
        setOutputLevel( _outputlevel ); 
    }

    void setLevel( int _level ) { 
      static_cast< basic_debugbuf<CharT, TraitsT>* >(std::ios::rdbuf())->
        setLevel( _level );  
    }

    basic_dostream & operator ()( int l ) {
      setLevel( l );
      return *this;
    }

  };

  typedef basic_dostream<char>    ConsoleStream;

  extern H3DAPI_API ConsoleStream Console;

}
#endif
