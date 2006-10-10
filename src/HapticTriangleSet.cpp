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
/// \file HapticTriangleSet.cpp
/// \brief cpp file for HapticTriangleSet
///
//
//////////////////////////////////////////////////////////////////////////////

#include "HapticTriangleSet.h" 

using namespace H3D;

bool HapticTriangleSet::lineIntersect( const Vec3d &from, 
                                       const Vec3d &to,
                                       Bounds::IntersectionInfo &result ) { 
  Matrix4d inv = transform.inverse();
  // TODO: find closest?
  bool have_intersection = false;
  for( unsigned int i = 0; i < triangles.size(); i++ ) {
    Bounds::Triangle &t = triangles[i];
    if( t.lineIntersect( inv * from, inv * to, result ) )	have_intersection = true;
  }
  
  if( have_intersection ) {
    result.point = transform * result.point;
    result.normal = transform.getRotationPart() * result.normal;
  }
  return have_intersection;
}

void HapticTriangleSet::getConstraints( const Vec3d &point,
                                         H3DDouble radius,
                                         std::vector< PlaneConstraint > &constraints ) {
  if( triangles.size() > 0 ) {
    // TODO: check if transform has uniform scale
    bool uniform_scale = true;

    if( uniform_scale ) {
      Vec3d p = transform.inverse() * point;
      unsigned int size = constraints.size();
      for( unsigned int i = 0; i < triangles.size(); i++ ) {
        Bounds::Triangle &t = triangles[i];
        t.getConstraints( p, radius, constraints );
      }

      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.normal = transform.getScaleRotationPart() * pc.normal;
        pc.normal.normalizeSafe();
        pc.point = transform * pc.point;
        pc.haptic_shape = this;
      }
    } else {
      // TODO: fix this
      unsigned int size = constraints.size();
      for( unsigned int i = 0; i < triangles.size(); i++ ) {
        Bounds::Triangle &t = triangles[i];
        t.getConstraints( point, radius, constraints );
      }
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = pc.point;
        pc.haptic_shape = this;
      }
    }
  }
}



