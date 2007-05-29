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
/// \file HapticBinaryTreePrimitive.h
/// \brief Header file for HapticBinaryTreePrimitive
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICBINARYTREEPRIMITIVE_H__
#define __HAPTICBINARYTREEPRIMITIVE_H__

#include <HAPIHapticShape.h>
#include <AutoRefVector.h>

namespace HAPI {

  /// A shape defined by a set of primitives.
  class HAPI_API HapticBinaryTreePrimitive: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticBinaryTreePrimitive( const vector< Bounds::GeometryPrimitive * > &_primitives,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       int _max_nr_primitives_in_leafs = 1,
                       string _tree_type = "AABB",
                       int _shape_id = -1,
                       Bounds::FaceType _touchable_face = 
                       Bounds::FRONT_AND_BACK):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face ) {
        tree = new Bounds::AABBTreePrimitive( _primitives, _max_nr_primitives_in_leafs );
      }

    template< class Iterator >
    HapticBinaryTreePrimitive( Iterator begin,
                       Iterator end,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       int _shape_id = -1,
                       Bounds::FaceType _touchable_face = 
                       Bounds::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face ),
      primitives( begin, end ) {
      }

    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Bounds::IntersectionInfo &result,
                                Bounds::FaceType face = Bounds::FRONT_AND_BACK  );

    virtual void getConstraints( const Vec3 &point,
                                 std::vector< PlaneConstraint > &constraints,
                                 Bounds::FaceType face = Bounds::FRONT_AND_BACK );

    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    virtual void glRender();

    /// The primitives
    H3DUtil::AutoRefVector< Bounds::GeometryPrimitive > primitives;
    string tree_type;
    Bounds::BBTreePrimitive *tree;
      
  };
}

#endif
