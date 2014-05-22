//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
//
/// \file HAPI.h
/// \brief Base header file that handles all configuration related settings
///
//////////////////////////////////////////////////////////////////////////////

/// \mainpage HAPI Documentation
/// Copyright 2004 - 2014, <a href="http://www.sensegraphics.com">SenseGraphics AB</a>

#ifndef __HAPI_H__
#define __HAPI_H__

#include <H3DUtil/H3DUtil.h>

#define USE_DOUBLE_PRECISION

/// Undef if you do not have OpenHaptics(www.sensable.com) installed. 
/// Required for the Sensable Phantom haptics devices.
#cmakedefine HAVE_OPENHAPTICS

/// Undef if you do not have Chai3d installed
#cmakedefine HAVE_CHAI3D
#cmakedefine CHAI3D_VERSION_2_0

/// Undef if you do not have Entact API(http://www.entactrobotics.com/) 
/// installed. Required for support for haptics devices from Entact.
#cmakedefine HAVE_ENTACTAPI

/// Undef if you do not have DHD API(www.forcedimension.com) installed.
/// Required for the Omega and Delta haptics devices.
#cmakedefine HAVE_DHDAPI

/// Undef if you do not have Virtuose API(www.haption.com) installed.
/// Required for the Virtuose series haptics devices from  Haption.
#cmakedefine HAVE_VIRTUOSEAPI

/// Undef if you do not have FALCON API(www.novint.com) installed.
#cmakedefine HAVE_FALCONAPI
#cmakedefine HAVE_NIFALCONAPI

// Select which communication library libnifalcon should use
#cmakedefine NIFALCONAPI_LIBUSB
#cmakedefine NIFALCONAPI_LIBFTD2XX
#cmakedefine NIFALCONAPI_LIBFTDI

/// Undef if you do not have the fparser library
/// (http://iki.fi/warp/FunctionParser/) installed.
/// Required for the ParsedFunction class
#cmakedefine HAVE_FPARSER

/// Undef if you do not have Haptik(www.haptiklibrary.org) installed. 
/// HaptikDevice will not be usable.
#cmakedefine HAVE_HAPTIK_LIBRARY 

/// Undef if you do not have SimballMedical API from GCoder
/// (http://www.g-coder.com/ installed. Required for the 
/// SimballHapticsDevice class.
#cmakedefine HAVE_SIMBALLMEDICAL_API

/// Undef if you do not want HAPI to have support for Haptic Master.
/// (http://www.moog.com/SimulationAndTesting/products/models/-/p_13598/t_1735/)
/// Required for the HapticMasterDevice class.
#cmakedefine HAVE_HAPTIC_MASTER_API

/// Undef if you do not have the API(s) required for supporting Quanser
/// devices.
/// www.quanser.com
//#define HAVE_QUANSERAPI

/// Undef if you do not have the MLHI API installed.
/// Required for the MLHIDevice class.
#cmakedefine HAVE_MLHI

/// Undef if you do not want to build HAPI with OpenGL support. This will
/// disable some features like FeedbackBufferCollector.
#cmakedefine HAVE_OPENGL


// note that _WIN32 is always defined when _WIN64 is defined.
#if( defined( _WIN64 ) || defined(WIN64) )
// set when on 64 bit Windows
#define H3D_WIN64
#define H3D_ARCH64
#elif( defined( _WIN32 ) || defined(WIN32) )
// set when on 32 bit Windows
#define H3D_WIN32
#define H3D_ARCH32
#endif

#if __GNUC__
#if __x86_64__ || __ppc64__
#define H3D_ARCH64
#else
#define H3D_ARCH32
#endif
#endif

#if( defined( H3D_WIN32 ) || defined( H3D_WIN64 ) )
// set when on 32 or 64 bit Windows
#define H3D_WINDOWS
#endif

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the HAPI_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see HAPI_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#ifdef H3D_WINDOWS
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
#endif // H3D_WINDOWS

#if defined(__APPLE__) && defined(__MACH__)
#define MACOSX
#define H3D_OSX
#define HAPI_API
#define HAVE_SYS_TIME_H
#endif

#if defined(__linux)
#define LINUX
#define H3D_LINUX
#define HAPI_API 
#define HAVE_SYS_TIME_H
#endif

#define HAPI_MAJOR_VERSION ${HAPI_MAJOR_VERSION}
#define HAPI_MINOR_VERSION ${HAPI_MINOR_VERSION}
#define HAPI_BUILD_VERSION ${HAPI_BUILD_VERSION}

/// HAPI namespace
namespace HAPI {
  /// Will return the version of HAPI as a double on the form
  /// HAPI_MAJOR_VERSION.HAPI_MINOR_VERSION
  double HAPI_API getHAPIVersion();
}

#endif



