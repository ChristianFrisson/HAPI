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
/// \file HapticPrimitive.cpp
/// \brief cpp file for HapticPrimitive
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticPrimitive.h>
#include <HAPI/PlaneConstraint.h>

using namespace HAPI;

bool HapticPrimitive::lineIntersectShape( const Vec3 &from, 
                                          const Vec3 &to,
                                          Collision::IntersectionInfo &result,
                                          Collision::FaceType face ) {
  return primitive->lineIntersect( from, to, result, face );
}

void HapticPrimitive::closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                           Vec3 &n, Vec3 &tc ) {
  primitive->closestPoint( p, cp, n, tc );
}

bool HapticPrimitive::movingSphereIntersectShape( HAPIFloat radius,
                                                  const Vec3 &from, 
                                                  const Vec3 &to ) {
  return primitive->movingSphereIntersect( radius, from, to );
}

void HapticPrimitive::getConstraintsOfShape( const Vec3 &point,
                                             Constraints &constraints,
                                             Collision::FaceType face,
                                             HAPIFloat radius ) {
  unsigned int size = constraints.size();
  primitive->getConstraints( point, constraints, face, radius );
  for( unsigned int i = size; i < constraints.size(); ++i ) {
    PlaneConstraint &pc = constraints[i];
    pc.haptic_shape.reset(this);
  }
}

#ifdef HAVE_OPENGL
void HapticPrimitive::glRenderShape() {
  primitive->render();
}
#endif

void HapticPrimitive::getTangentSpaceMatrixShape( const Vec3 &point,
                                                  Matrix4 &result_mtx ) {
  primitive->getTangentSpaceMatrix( point, result_mtx );
}
