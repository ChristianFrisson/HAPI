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
/// \file HapticTriangleSet.h
/// \brief Header file for HapticTriangleSet
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICTRIANGLESET_H__
#define __HAPTICTRIANGLESET_H__

#include <HAPI/HAPIHapticShape.h>

namespace HAPI {

  /// \ingroup Shapes
  /// \class HapticTriangleSet
  /// \brief A shape defined by a set of triangles.
  class HAPI_API HapticTriangleSet: public HAPIHapticShape {
  public:
    typedef enum {
      CONVEX_FRONT,
      CONVEX_BACK,
      NOT_CONVEX
    } ConvexType;

    /// Constructor.
    HapticTriangleSet( const std::vector< Collision::Triangle > &_triangles,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0
                        ):
      HAPIHapticShape( _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      triangles( _triangles ),
      convex( _convex ) {}

    /// Constructor.
    HapticTriangleSet( const Matrix4 &_transform,
                       const std::vector< Collision::Triangle > &_triangles,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0 ):
      HAPIHapticShape( _transform, _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      triangles( _triangles ),
      convex( _convex ) {}

    template< class Iterator >
    HapticTriangleSet( Iterator begin,
                       Iterator end,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0 ): 
      HAPIHapticShape( _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      triangles( begin, end ),
      convex( _convex ) {}


    /// Constructor.
    HapticTriangleSet( const Vec3 &_velocity,
                       const Rotation &_angular_velocity,
                       const Vec3 &_growth_rate,
                       const std::vector< Collision::Triangle > &_triangles,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0
                        ):
      HAPIHapticShape( _velocity,  _angular_velocity, 
                       _growth_rate, _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      triangles( _triangles ),
      convex( _convex ) {}

    /// Constructor.
    HapticTriangleSet( const Matrix4 &_transform,
                       const Vec3 &_velocity,
                       const Rotation &_angular_velocity,
                       const Vec3 & _growth_rate,
                       const std::vector< Collision::Triangle > &_triangles,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0 ):
      HAPIHapticShape( _transform, _velocity, _angular_velocity, 
                       _growth_rate, _surface, _touchable_face, 
                       _userdata, _shape_id, _clean_up_func ),
      triangles( _triangles ),
      convex( _convex ) {}

    template< class Iterator >
    HapticTriangleSet( const Vec3 &_velocity,
                       const Rotation &_angular_velocity,
                       const Vec3 &_growth_rate,
                       Iterator begin,
                       Iterator end,
                       HAPISurfaceObject *_surface, 
                       ConvexType _convex = NOT_CONVEX,
                       Collision::FaceType _touchable_face = 
                       Collision::FRONT_AND_BACK,
                       void *_userdata = NULL,
                       int _shape_id = -1, 
                       void (*_clean_up_func)( void * ) = 0 ): 
      HAPIHapticShape( _velocity, _angular_velocity, 
                       _growth_rate,_surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      triangles( begin, end ),
      convex( _convex ) {}


#ifdef HAVE_OPENGL
    /// Returns the number of triangles in the set.
    inline virtual int nrTriangles() {
      return (int)triangles.size();
    }
#endif
    
    /// The triangles.
    std::vector< Collision::Triangle > triangles;
    ConvexType convex;

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
                                     Collision::FRONT_AND_BACK  );
    
    
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
                                        Collision::FRONT_AND_BACK ,
                                        HAPIFloat radius = -1 );

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to (in local coords).
    /// \param cp Return parameter for closest point (in local coords).
    /// \param n Return parameter for normal at closest point
    /// (in local coords).
    /// \param tc Return paramater for texture coordinate at closest point.
    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc );

    /// Detect collision between a moving sphere and the object.
    /// \param radius The radius of the sphere(in local coords).
    /// \param from The start position of the sphere(in local coords).
    /// \param to The end position of the sphere(in local coords).
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to );

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
    /// buffer shapes. 
    virtual void glRenderShape();
#endif

  };
}

#endif
