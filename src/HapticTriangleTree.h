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
/// \file HapticShape.h
/// \brief Header file for HapticShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICTRIANGLETREE_H__
#define __HAPTICTRIANGLETREE_H__

#include <HAPIHapticShape.h>

namespace HAPI {

  /// Base class for haptic shapes, i.e. geometrical objects that are rendered
  /// by letting their surfaces constrain the proxy. A HapticShape has a Surface
  /// object associated to it that defines the properties of the surface, e.g.
  /// stiffness and friction properties.
  ///
  class HAPI_API HapticTriangleTree: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticTriangleTree( Bounds::BinaryBoundTree *triangle_tree,
                        void *_userdata,
                        HAPISurfaceObject *_surface,
                        const Matrix4 & _transform,
                        int _shape_id = -1,
                        Bounds::FaceType _touchable_face = 
                        Bounds::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face ),
      tree( triangle_tree ) {}

    ~HapticTriangleTree() {
      delete tree;
    }
    
    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Bounds::IntersectionInfo &result,
                                Bounds::FaceType face = Bounds::FRONT_AND_BACK  ) { 
      Matrix4 inv = transform.inverse();
      bool intersect = tree->lineIntersect( inv * from, inv * to, 
                                            result, face );
      if( intersect ) {
        result.point = transform * result.point;
        result.normal = transform.getRotationPart() * result.normal;
      }
      return intersect;
    }

    virtual void getConstraints( const Vec3 &point,
                                 HAPIFloat radius,
                                 std::vector< PlaneConstraint > &constraints,
                                 Bounds::FaceType face = Bounds::FRONT_AND_BACK   );
    
    /// The Surface object describing the properties of the surface of the 
    /// HapticShape.
    Bounds::BinaryBoundTree *tree;
      
  };
}

#endif
