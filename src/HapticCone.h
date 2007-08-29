
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
/// \file HapticCone.h
/// \brief Header file for HapticCone
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICCONE_H__
#define __HAPTICCONE_H__

#include <HAPISurfaceObject.h>
#include <HAPIHapticShape.h>

namespace HAPI {

  /// Class for rendering a haptic Cone.
  class HAPI_API HapticCone: public HAPIHapticShape {
  public:
    /// Constructor.
    /// \param _bottomRadius The radius of the bottom.
    /// \param _height The height of the cone.
    /// \param _surface The Surface of the box.
    HapticCone( HAPIFloat _bottomRadius,
                HAPIFloat _height,
                  void *_userdata,
                  HAPISurfaceObject *_surface,
                  const Matrix4 &_transform,
                  void (*_clean_up_func)(void *) = 0,
                  int _shape_id = -1,
                  Bounds::FaceType _touchable_face = 
                  Bounds::FRONT_AND_BACK ):
      HAPIHapticShape( _userdata, _surface, _transform, _clean_up_func,
                       _shape_id, _touchable_face ),
      bottomRadius( _bottomRadius ),
      height( _height ) {
      }
#ifdef HAVE_OPENHAPTICSfff
fdas
    /// Intersect the line segment from start_point to end_point with
    /// the object.  
    /// This is used by the callback functions of the HLCustomObject.
    /// \param start_point The start point of the line segment.
    /// \param end_point The end point of the line segment.
    /// \param intersection_point Return parameter that should be set to the
    /// point of intersection between the line segment and object.
    /// \param intersection_normal Return parameter that should be set to the
    /// normal of the surface of the objet at the intersection point.
    /// \param face Return parameter that should be set to HL_FRONT if the front
    /// of the surface is touched and to HL_BACK if the backside of the surface 
    /// is touched. 
    /// \returns true if there is an intersection, false otherwise.
    ///
    virtual bool intersectSurface( const Vec3 &start_point, 
                                   const Vec3 &end_point,
                                   Vec3 &intersection_point, 
                                   Vec3 &intersection_normal,
                                   HLenum &face );

    /// Find the closest point to query_point on the surface of the
    /// object. 
    /// This is used by the callback functions of the HLCustomObject.
    /// \param query_point The point to find the closest point to.
    /// \param geom Return parameter to return a hlLocalFeature
    /// \param closest_point Return parameter that should be set to the closest
    /// point to the surface to query_point.
    /// the surface at the closest point.
    /// 
    virtual bool closestFeature( const Vec3 &query_point, 
                                 const Vec3 &target_point,
                                 HLgeom *geom,
                                 Vec3 &closest_point );    

    /// hlRender is overriden to set up which sides of the sphere is touchable. 
    virtual void hlRender( HLHapticsDevice *hd );
#endif
// TODO: implement these, maybe need Cone Primitive in CollisionObject
    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Bounds::IntersectionInfo &result,
                             Bounds::FaceType face = Bounds::FRONT_AND_BACK ); 

    virtual void getConstraints( const Vec3 &point,
                                 Constraints &constraints,
                              Bounds::FaceType face = Bounds::FRONT_AND_BACK );

    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    /// The radius of the base of the cone in millimetres
    HAPIFloat bottomRadius;
    /// The height of the cone in  millimetres
    HAPIFloat height;
  
  };
}

#endif
