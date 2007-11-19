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
//
/// \file HApi.h
/// \brief Base header file that handles all configuration related settings
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage
/// Copyright 2004, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __HAPI_H__
#define __HAPI_H__

#define USE_DOUBLE_PRECISION

/// Undef if you do not have OpenHaptics(www.sensable.com) installed. 
/// Required for the Sensable Phantom haptics devices.
#define HAVE_OPENHAPTICS

/// Undef if you do not have Chai3d installed
#define HAVE_CHAI3D

/// Undef if you do not have DHD API(www.forcedimension,com) installed.
/// Required for the Omega and Delta haptics devices.
#define HAVE_DHDAPI

/// Undef if you do not have FALCON API(www.forcedimension,com) installed.
/// Required for the Falcon haptics device.
#define HAVE_FALCONAPI

/// Undef if you do not have the fparser library
/// (http://iki.fi/warp/FunctionParser/) installed.
/// Required for the ParsedFunction class
#define HAVE_FPARSER

/// Undef if you do not have the C-API wrapper for the HapticMaster
/// library. HapticMaster devices will then not be supported.
#define HAVE_HAPTIC_MASTER_API

/// Undef if you do not have Haptik(www.haptiklibrary.org) installed. 
/// HaptikDevice will not be usable.
//#define HAVE_HAPTIK_LIBRARY 

/// Undef if you do not the APIs required for supporting Quanser devices.
/// www.quanser.com
//#define HAVE_QUANSERAPI

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the HAPI_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see HAPI_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef WIN32
#include <windows.h>


#ifdef HAPI_LIB
#define HAPI_API
#else

#ifdef HAPI_EXPORTS
#define HAPI_API __declspec(dllexport)
#else
#define HAPI_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports 
#pragma warning( disable: 4251 )
#endif


#endif // HAPI API else
#endif // WIN32

#if defined(__APPLE__) && defined(__MACH__)
#define MACOSX
#define HAPI_API
#define HAVE_SYS_TIME_H
#endif

#if defined(__linux)
#define LINUX
#define HAPI_API 
#define HAVE_SYS_TIME_H
#endif


/*namespace H3D {
  /// Function for determining if the machine we are running on is uses
  /// little endian byte order or not.
  inline bool isLittleEndian() {
    union probe{ 
      unsigned int num;
      unsigned char bytes[sizeof(unsigned int)];
    };
    //initialize first member of p with unsigned 1
    probe p = { 1U };
    // in a big endian architecture, p.bytes[0] equals 0  
    bool little_endian = (p.bytes[0] == 1U); 
    return little_endian;
  }
}*/

#endif



