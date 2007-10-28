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
/// \file HapticShapeConstraint.cpp
/// \brief cpp file for HapticShapeConstraint
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticShapeConstraint.h> 

using namespace HAPI;

HapticShapeConstraint::HapticShapeConstraint(
                          Collision::CollisionObject *_shape,
                          const HAPIFloat &_spring_constant ):
  shape( _shape ),
  spring_constant( _spring_constant ){
}

HAPIForceEffect::EffectOutput HapticShapeConstraint::calculateForces(
                              HAPIHapticsDevice *hd,
                              HAPITime dt ) {
  Vec3 closest_point = Vec3(), temp;
  Vec3 hd_pos = hd->getPosition();
  shape->closestPoint( hd_pos, closest_point, temp, temp );
  Vec3 the_force = spring_constant * (closest_point - hd_pos);
  return EffectOutput( the_force );
}
