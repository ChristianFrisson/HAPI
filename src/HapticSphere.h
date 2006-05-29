
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
/// \file HapticSphere.h
/// \brief Header file for HapticSphere
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSPHERE_H__
#define __HAPTICSPHERE_H__

#include <HAPISurfaceObject.h>
#include <HAPIHapticShape.h>

namespace H3D {

  /// Class for rendering a haptic Sphere.
  class HAPI_API HapticSphere: public HAPIHapticShape {
  public:
    /// Constructor.
    /// \param _radius The radius of the sphere in metres.
    /// \param _solid   If false, both the inside and outside of the sphere is 
    /// rendered. If true, only the outside is rendered.
    /// \param _surface The Surface of the sphere.
    HapticSphere( H3DFloat _radius,
                  bool _solid,
                  void *_userdata,
                  HAPISurfaceObject *_surface,
                  const Matrix4f &_transform ):
      HAPIHapticShape( _userdata, _surface, _transform ),
      radius( _radius ),
      solid( _solid ) {}
#ifdef HAVE_OPENHAPTICSf
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
    virtual bool intersectSurface( const Vec3f &start_point, 
                                   const Vec3f &end_point,
                                   Vec3f &intersection_point, 
                                   Vec3f &intersection_normal,
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
    virtual bool closestFeature( const Vec3f &query_point, 
                                 const Vec3f &target_point,
                                 HLgeom *geom,
                                 Vec3f &closest_point );    

    /// hlRender is overriden to set up which sides of the sphere is touchable. 
    virtual void hlRender( HLHapticsDevice *hd );
#endif
    virtual bool lineIntersect( const Vec3d &from, 
                                const Vec3d &to,
                                Bounds::IntersectionInfo &result ); 

    virtual void getConstraints( const Vec3d &point,
                                 H3DDouble radius,
                                 std::vector< PlaneConstraint > &constraints );

    /// The radius of the sphere in metres. 
    H3DFloat radius;

    /// If false, both the inside and outside of the sphere is rendered.
    /// If true, only the outside is rendered.
    bool solid;

  
  };
}

#endif
