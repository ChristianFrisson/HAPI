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
/// \file HapticTimeFunctionEffect.cpp
/// \brief cpp file for HapticTimeFunctionEffect
///
//
//////////////////////////////////////////////////////////////////////////////

#include "HapticTimeFunctionEffect.h" 

using namespace HAPI;

HapticTimeFunctionEffect::HapticTimeFunctionEffect(
                          const Matrix4 & _transform,
                          bool _interpolate,
                          HAPIFunctionObject *_x_function,
                          HAPIFunctionObject *_y_function,
                          HAPIFunctionObject *_z_function,
                          const HAPITime &_x_zero_time,
                          const HAPITime &_y_zero_time,
                          const HAPITime &_z_zero_time ):
  HAPIForceEffect( _transform, _interpolate ),
  x_function( _x_function ),
  y_function( _y_function ),
  z_function( _z_function ),
  x_zero_time( _x_zero_time ),
  y_zero_time( _y_zero_time ),
  z_zero_time( _z_zero_time ) {
}

HAPIForceEffect::EffectOutput HapticTimeFunctionEffect::calculateForces(
                              HAPIHapticsDevice *hd,
                              HAPITime dt ) {
  HAPITime current_time = H3DUtil::TimeStamp();
  HAPIFloat x_current_time = current_time - x_zero_time;
  HAPIFloat y_current_time = current_time - y_zero_time;
  HAPIFloat z_current_time = current_time - z_zero_time;
  Vec3 force = Vec3( x_function->evaluate( &x_current_time ),
                     y_function->evaluate( &y_current_time ),
                     z_function->evaluate( &z_current_time ) );
  return EffectOutput( force );
}