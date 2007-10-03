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
    HapticPointSet( const vector< Collision::Point > &_points,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       void (*_clean_up_func)( void * ) = 0,
                       int _shape_id = -1,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK):
      HAPIHapticShape( _userdata, _surface, _transform, _clean_up_func,
                       _shape_id, _touchable_face ),
      points( _points ) {}

    template< class Iterator >
    HapticPointSet( Iterator begin,
                       Iterator end,
                       void *_userdata,
                       HAPISurfaceObject *_surface,
                       const Matrix4 & _transform,
                       void (*_clean_up_func)( void * ) = 0,
                       int _shape_id = -1,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, _clean_up_func,
                       _shape_id, _touchable_face ),
      points( begin, end ) {}

    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Collision::IntersectionInfo &result,
                                Collision::FaceType face = Collision::FRONT_AND_BACK  );

    virtual void getConstraints( const Vec3 &point,
                                 Constraints &constraints,
                                 Collision::FaceType face = Collision::FRONT_AND_BACK,
                                 HAPIFloat radius = -1 );

    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    virtual void glRender();

    inline virtual int nrPoints() {
      return points.size();
    }

    /// The triangles.
    vector< Collision::Point > points;
      
  };
}

#endif
