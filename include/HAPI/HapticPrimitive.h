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
/// \file HapticPrimitive.h
/// \brief Header file for HapticPrimitive
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICPRIMITIVE_H__
#define __HAPTICPRIMITIVE_H__

#include <HAPI/HAPISurfaceObject.h>
#include <HAPI/HAPIHapticShape.h>

namespace HAPI {

  /// The HapticPrimitive class is a HAPIHapticShape which uses
  /// a GeometryPrimitive to define the geometry. 
  class HAPI_API HapticPrimitive: public HAPIHapticShape {
  public:
    /// Constructor.
    HapticPrimitive( const Matrix4 &_transform,
                     Collision::GeometryPrimitive *_primitive,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticShape( _transform, _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      primitive( _primitive ) {}

    /// Constructor.
    HapticPrimitive( Collision::GeometryPrimitive *_primitive,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticShape( _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      primitive( _primitive ) {}

#ifdef HAVE_OPENGL
    /// An upper bound on how many triangles are renderered.
    virtual int nrTriangles() {
      if( dynamic_cast< Collision::Triangle * >( primitive.get() ) )
        return 1;
      else if( dynamic_cast< Collision::LineSegment * >( primitive.get() ) ||
               dynamic_cast< Collision::Point * >( primitive.get() ) )
        return 0;
      // GeometryPrimitive sphere and plane
      return -1;
    }

    /// An upper bound on how many points are renderered.
    virtual int nrPoints() {
      if( dynamic_cast< Collision::Point * >( primitive.get() ) )
        return 1;
      else if( dynamic_cast< Collision::Triangle * >( primitive.get() ) ||
               dynamic_cast< Collision::LineSegment * >( primitive.get() ) )
        return 0;
      // GeometryPrimitive sphere and plane
      return -1;
    }

    /// An upper bound on how many lines are renderered.
    virtual int nrLines() {
      if( dynamic_cast< Collision::LineSegment * >( primitive.get() ) )
        return 1;
      else if( dynamic_cast< Collision::Triangle * >( primitive.get() ) ||
               dynamic_cast< Collision::Point * >( primitive.get() ) )
        return 0;
      // GeometryPrimitive sphere and plane
      return -1;
    }
#endif

  protected:

    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment(in local coords).
    /// \param to The end of the line segment(in local coords).
    /// \param result Contains info about closest intersection, if 
    /// line intersects object(in local coords).
    /// \param face The sides of the object that can be intersected. E.g.
    /// if FRONT, intersections will be reported only if they occur from
    /// the front side, i.e. the side in which the normal points. 
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersectShape( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face = 
                                     Collision::FRONT_AND_BACK); 

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to(in local coords).
    /// \param closest_point Return parameter for closest point
    /// (in local coords)
    /// \param normal Return parameter for normal at closest point
    /// (in local coords).
    /// \param tex_coord Return paramater for texture coordinate at closest 
    /// point.
    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc );

    /// Detect collision between a moving sphere and the object.
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to );

    /// Get constraint planes of the shape. A proxy of a haptics renderer
    /// will always stay above any constraints added.
    ///
    /// \param point Point to constrain(in local coords).
    /// \param constraints Where to add the constraints.
    /// \param face Determines which faces of the shape will be seen as
    /// constraining.
    /// \param radius Only add constraints within this radius. If set to -1
    /// all constraints will be added.
    virtual void getConstraintsOfShape( const Vec3 &point,
                                        Constraints &constraints,
                                        Collision::FaceType face = 
                                        Collision::FRONT_AND_BACK,
                                        HAPIFloat radius = -1 );

    /// Calculates a matrix transforming a vector from local space of the shape
    /// to texture space of the shape.
    /// \param point The point at which to find the tangent vectors
    /// in local coordinates
    /// \param result_mtx Stores the calculated matrix
    virtual void getTangentSpaceMatrixShape( const Vec3 &point,
                                             Matrix4 &result_mtx );

#ifdef HAVE_OPENGL
    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes. The rendering should be done in local space of the 
    /// shape, i.e. ignoring the transform matrix in the shape.
    virtual void glRenderShape();
#endif

    H3DUtil::AutoRef< Collision::GeometryPrimitive > primitive;

  };
}

#endif
