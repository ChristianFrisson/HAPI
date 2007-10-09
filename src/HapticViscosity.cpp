//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file HapticViscosity.cpp
/// \brief cpp file for HapticViscosity
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticViscosity.h> 

using namespace HAPI;

HapticViscosity::HapticViscosity( const HAPIFloat &_viscosity,
                                  const HAPIFloat &_radius,
                                  const HAPIFloat &_damping_factor,
                                  bool _interpolate ):
  HAPIForceEffect( _interpolate ),
  viscosity( _viscosity ),
  radius( _radius ){
  // constant calculated from stokes law.
  the_constant = _damping_factor * -6.0 *
                 H3DUtil::Constants::pi * _viscosity * _radius;
}

HAPIForceEffect::EffectOutput HapticViscosity::calculateForces(
                                                HAPIHapticsDevice *hd,
                                               HAPITime dt ) {
  Vec3 hd_vel = hd->getVelocity();
  Vec3 the_force = the_constant * hd_vel;
  return EffectOutput( the_force );
}
