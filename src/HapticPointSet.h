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
/// \file HapticPointSet.h
/// \brief Header file for HapticPointSet
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICPOINTSET_H__
#define __HAPTICPOINTSET_H__

#include <HAPIHapticShape.h>

namespace HAPI {

  /// A shape defined by a set of points.
  class HAPI_API HapticPointSet: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticPointSet( const vector< Bounds::Point > &_points,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       int _shape_id = -1,
                       Bounds::FaceType _touchable_face = 
                       Bounds::FRONT_AND_BACK):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face ),
      points( _points ) {}

    template< class Iterator >
    HapticPointSet( Iterator begin,
                       Iterator end,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       int _shape_id = -1,
                       Bounds::FaceType _touchable_face = 
                       Bounds::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, 
                       _shape_id, _touchable_face ),
      points( begin, end ) {}

    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Bounds::IntersectionInfo &result,
                                Bounds::FaceType face = Bounds::FRONT_AND_BACK  );

    virtual void getConstraints( const Vec3 &point,
                                 std::vector< PlaneConstraint > &constraints,
                                 Bounds::FaceType face = Bounds::FRONT_AND_BACK );

    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    virtual void glRender();

    inline virtual int nrPoints() {
      return points.size();
    }

    /// The triangles.
    vector< Bounds::Point > points;
      
  };
}

#endif
