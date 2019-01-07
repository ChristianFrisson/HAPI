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
/// \file HapticSpring.cpp
/// \brief cpp file for HapticSpring
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticSpring.h>

using namespace HAPI;

HapticSpring::HapticSpring( const Vec3 &_position,
                            HAPIFloat _spring_constant,
                            bool _use_ref_count_lock ):
  HAPIForceEffect( _use_ref_count_lock ),
  position( _position ),
  spring_constant( _spring_constant ),
  damping( 0 ),
  first_time( true ),
  step_length( 0 ),
  position_interpolation( 1 ) {}

HapticSpring::HapticSpring( const Vec3 &_position,
                            HAPIFloat _spring_constant,
                            HAPIFloat _damping,
                            HAPIFloat _position_interpolation,
                            bool _use_ref_count_lock ):
  HAPIForceEffect( _use_ref_count_lock ),
  position( _position ),
  spring_constant( _spring_constant ),
  damping( _damping ),
  first_time( true ),
  step_length( 0 ),
  position_interpolation( _position_interpolation ) {
  
  if( position_interpolation < 0 )
    position_interpolation = 0;
  else if( position_interpolation > 1 )
    position_interpolation = 1;
}


HapticSpring::EffectOutput HapticSpring::calculateForces( const EffectInput &input ) {
  Vec3 relative_velocity = input.hd->getVelocity();
  if( first_time ) {
    interpolated_position = position;
    first_time = false;
  } else {
    Vec3 to_point = position - interpolated_position;
    HAPIFloat to_point_length = to_point.length();
    if( to_point_length > Constants::epsilon ) {
      to_point /= to_point_length;
      HAPIFloat travel = H3DUtil::H3DMin( to_point_length, step_length );
      to_point *= travel;
      interpolated_position += to_point;
      Vec3 instant_spring_velocity = to_point / input.deltaT;
      relative_velocity -= instant_spring_velocity;
    }
  }

  force = 
    ( interpolated_position - input.hd->getPosition() ) * spring_constant - 
    damping * relative_velocity;
  return EffectOutput( force );
}
