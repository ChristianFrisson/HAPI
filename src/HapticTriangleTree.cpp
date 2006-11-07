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
/// \file HapticTriangleTree.cpp
/// \brief cpp file for HapticTriangleTree
///
//
//////////////////////////////////////////////////////////////////////////////

#include "HapticTriangleTree.h" 

using namespace HAPI;

void HapticTriangleTree::getConstraints( const Vec3 &point,
                                         HAPIFloat radius,
                                         std::vector< PlaneConstraint > &constraints,
                                         Bounds::FaceType face ) {
  if( tree ) {
    // TODO: check if transform has uniform scale
    bool uniform_scale = true;

    if( uniform_scale ) {
      Vec3 p = transform.inverse() * point;
      unsigned int size = constraints.size();
      tree->getConstraints( p, constraints, face );
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.normal = transform.getScaleRotationPart() * pc.normal;
        pc.normal.normalizeSafe();
        pc.point = transform * pc.point;
        pc.haptic_shape = this;
      }
    } else {
      unsigned int size = constraints.size();
      tree->getConstraints( point, transform, constraints, face );
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = pc.point;
        pc.haptic_shape = this;
      }
    }
    
  }
  //result.push_back( Bounds::PlaneConstraint( (radius+0.0025) * v, v ) );
}



