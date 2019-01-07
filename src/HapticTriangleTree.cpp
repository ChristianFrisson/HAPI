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
/// \file HapticTriangleTree.cpp
/// \brief cpp file for HapticTriangleTree
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticTriangleTree.h>
#include <HAPI/PlaneConstraint.h>

using namespace HAPI;


bool HapticTriangleTree::lineIntersectShape( const Vec3 &from, 
                                          const Vec3 &to,
                                          Collision::IntersectionInfo &result,
                                          Collision::FaceType face ) {
  return tree->lineIntersect( from, to, result, face );
}

void HapticTriangleTree::closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                           Vec3 &n, Vec3 &tc ) {
  tree->closestPoint( p, cp, n, tc );
}

bool HapticTriangleTree::movingSphereIntersectShape( HAPIFloat radius,
                                                  const Vec3 &from, 
                                                  const Vec3 &to ) {
  return tree->movingSphereIntersect( radius, from, to );
}

void HapticTriangleTree::getConstraintsOfShape( const Vec3 &point,
                                             Constraints &constraints,
                                             Collision::FaceType face,
                                             HAPIFloat radius ) {
  unsigned int size = constraints.size();
  tree->getConstraints( point, constraints, face, radius );
  for( unsigned int i = size; i < constraints.size(); ++i ) {
    PlaneConstraint &pc = constraints[i];
    pc.haptic_shape.reset(this);
  }
}

#ifdef HAVE_OPENGL
void HapticTriangleTree::glRenderShape() {
  tree->render();
}
#endif

void HapticTriangleTree::getTangentSpaceMatrixShape( const Vec3 &point,
                                                     Matrix4 &result_mtx ) {
  std::vector< Collision::Triangle > triangles;
  std::vector< Collision::LineSegment > lines;
  std::vector< Collision::Point > points;
  tree->getPrimitivesWithinRadius( point,
                                   Constants::epsilon * 10,
                                   triangles, lines,
                                   points );
  if( !triangles.empty() ) {
    if( triangles.size() == 1 ) {
      triangles[0].getTangentSpaceMatrix( point, result_mtx );
    } else {
      unsigned int closest_triangle = 0;
      Vec3 temp_cp, temp_n;
      triangles[0].closestPoint( point, temp_cp, temp_n, temp_n );
      HAPIFloat distance = ( temp_cp - point).lengthSqr(); 
      HAPIFloat temp_distance;
      for( unsigned int i = 1; i < triangles.size(); ++i ) {
        triangles[i].closestPoint( point, temp_cp, temp_n, temp_n );
        temp_distance = (temp_cp - point).lengthSqr();
        if( temp_distance < distance ) {
          closest_triangle = i;
          distance = temp_distance;
        }
      }
      
      triangles[closest_triangle].
        getTangentSpaceMatrix( point, result_mtx );
    }
  }
}
