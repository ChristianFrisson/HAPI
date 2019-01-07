//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
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
/// \file Config.h
/// \brief Base header file that handles all configuration related settings
/// for OpenHapticsRenderer.
///
//////////////////////////////////////////////////////////////////////////////
#ifndef __CONFIG_H__
#define __CONFIG_H__

#if defined(__APPLE__) && defined(__MACH__)
#define OPENHAPTICSRENDERER_API
#endif

#if defined(__linux)
#define OPENHAPTICSRENDERER_API
#endif

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler. All files within this DLL
// are compiled with the OPENHAPTICSRENDERER_EXPORTS symbol defined on the command
// line. this symbol should not be defined on any project that uses
// this DLL. This way any other project whose source files include
// this file see OPENHAPTICSRENDERER_API functions as being imported from a DLL,
// whereas this DLL sees symbols defined with this macro as being
// exported.
#if defined(WIN32) || defined(__WIN32__)
#include <windows.h>
#ifdef OPENHAPTICSRENDERER_EXPORTS
#define OPENHAPTICSRENDERER_API __declspec(dllexport)
#else
#define OPENHAPTICSRENDERER_API __declspec(dllimport)
#endif
#ifdef _MSC_VER
// disable dll-interface warnings for stl-exports 
#pragma warning( disable: 4251 )
#endif


#endif

#endif



