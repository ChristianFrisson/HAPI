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
/// \file HAPIHapticShape.h
/// \brief Header file for HAPIHapticShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIHAPTICSHAPE_H__
#define __HAPIHAPTICSHAPE_H__

// HAPI includes
#include <HAPI/HAPIHapticObject.h>
#include <HAPI/HAPIGLShape.h>
#include <HAPI/HAPISurfaceObject.h>
#include <HAPI/HAPIShapeRenderOptions.h>
#include <HAPI/CollisionObjects.h>

// H3D Util includes
#include <H3DUtil/AutoPtrVector.h>
#include <H3DUtil/RefCountedClass.h>

// stl includes
#include <vector>
#include <algorithm>
#include <list>

namespace HAPI {


  /// Base class for haptic shapes, i.e. geometrical objects that are
  /// rendered by letting their surfaces constrain the proxy. A HAPIHapticShape
  /// has a Surface object associated to it that defines the properties of the
  /// surface, e.g. stiffness and friction properties.
  ///
  /// Several functions below 
  class HAPI_API HAPIHapticShape: public HAPIHapticObject,
                                  public Collision::CollisionObject,
                                  public HAPIGLShape {
  public:

   /// Constructor.
    HAPIHapticShape( const Matrix4 &_transform,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticObject( ),
      Collision::CollisionObject( true ),

      clean_up_func( _clean_up_func ),
      surface( _surface ),
      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      transform( _transform ),
      have_inverse( false ),
      have_transform( true ) {
      inverse = transform.inverse();
    }

    /// Constructor.
    HAPIHapticShape( HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticObject( ),
      Collision::CollisionObject( true ),
      clean_up_func( _clean_up_func ),
      surface( _surface ),
      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      have_inverse( false ),
      have_transform( false ) {}

    /// Destructor.
    virtual ~HAPIHapticShape();

    /// Scale object uniformly on all axis by the given scaling factor.
    void scale( HAPIFloat s );

    /// Translate object.
    void translate( const Vec3 &t );

    /// Rotate object.
    void rotate( const Rotation &r );

    /// Transform the object using the given transform matrix. Must be
    /// a transformation matrix or the result will be undefined.
    inline void transformShape( const Matrix4 &t ) {
      have_transform = true;
      have_inverse = false;
      transform = t * transform;
    }

    /// Set the transformation matrix from local to global space.
    inline void setTransform( const Matrix4 &t ) {
      have_transform = true;
      have_inverse = false;
      transform = t;
    }

    /// Get the transformation matrix from local to global space.
    inline const Matrix4 &getTransform( const Matrix4 &t ) {
      return transform;
    }

    /// Get the transformation matrix from global to local space.
    inline const Matrix4 &getInverse( const Matrix4 &t ) {
      return inverse;
    }

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to.
    /// \param closest_point Return parameter for closest point
    /// \param normal Return parameter for normal at closest point.
    /// \param tex_coord Return paramater for texture coordinate at closest 
    /// point
    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about closest intersection, if 
    /// line intersects object
    /// \param face The sides of the object that can be intersected. E.g.
    /// if FRONT, intersections will be reported only if they occur from
    /// the front side, i.e. the side in which the normal points. 
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Collision::IntersectionInfo &result,
                                Collision::FaceType face );

    /// Detect collision between a moving sphere and the object.
    /// \param radius The radius of the sphere
    /// \param from The start position of the sphere
    /// \param to The end position of the sphere.
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersect( HAPIFloat radius,
                                        const Vec3 &from, 
                                        const Vec3 &to );

    /// Get constraint planes of the shape. A proxy of a haptics renderer
    /// will always stay above any constraints added.
    /// \param point Point to constrain.
    /// \param constraints Where to add the constraints.
    /// \param face Determines which faces of the shape will be seen as
    /// constraining.
    /// \param radius Only add constraints within this radius. If set to -1
    /// all constraints will be added.
    virtual void getConstraints( const Vec3 &point,
                                 Constraints &constraints,
                                 Collision::FaceType face = 
                                 Collision::FRONT_AND_BACK,
                                 HAPIFloat radius = -1 );
    
    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes.
    virtual void glRender();

    /// Generate a new shape id.
    static int genShapeId();
    
    /// Deallocate all resources allocated to the shape id and return
    /// it for reuse.
    static void delShapeId( int id );

    /// The Surface object describing the properties of the surface.
    H3DUtil::AutoRef< HAPISurfaceObject > surface;

    /// Add an HAPIShapeRenderOptions instance with options on how how
    /// to render this shape haptically.
    inline void addRenderOption( HAPIShapeRenderOptions *o ) {
      options.push_back( o );
    }

    /// Remove an HAPIShapeRenderOptions instance from the currently
    /// used options.
    inline void removeRenderOption( HAPIShapeRenderOptions *o ) {
      H3DUtil::AutoPtrVector< HAPIShapeRenderOptions >::iterator i =
        std::find( options.begin(), options.end(), o );
      if( i != options.end() ) {
        options.erase( i );
      }
    }

    /// Get the currently used render options.
    inline const H3DUtil::AutoPtrVector< HAPIShapeRenderOptions > &
    getRenderOptions() {
      return options;
    }

    /// Get a specific render option.
    template< class T > 
    void getRenderOption( T * &option ) {
      for( H3DUtil::AutoPtrVector< HAPIShapeRenderOptions >::const_iterator i =
             options.begin();
           i != options.end(); i++ ) {
        T *o = dynamic_cast< T * >( *i );
        if( o ) {
          option = o;
          return;
        }
      }
      option = NULL;
    } 

    H3DUtil::AutoPtrVector< HAPIShapeRenderOptions > options;

    void *userdata;

    int shape_id;
    Collision::FaceType touchable_face;

    static int current_max_id;
    static list< int > free_ids;

    bool have_transform;
    bool have_inverse;

    Matrix4 transform;
    Matrix4 inverse;
    
  protected:
    void (*clean_up_func)( void * );

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to(in local coords).
    /// \param closest_point Return parameter for closest point
    /// (in local coords)
    /// \param normal Return parameter for normal at closest point
    /// (in local coords).
    /// \param tex_coord Return paramater for texture coordinate at closest 
    /// point.
    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc ) = 0;

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
                                     Collision::FaceType face ) = 0;

    /// Detect collision between a moving sphere and the object.
    /// \param radius The radius of the sphere(in local coords).
    /// \param from The start position of the sphere(in local coords).
    /// \param to The end position of the sphere(in local coords).
    /// \returns true if intersected, false otherwise.
    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) = 0;
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
                                        HAPIFloat radius = -1 ) = 0;

    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes. The rendering should be done in local space of the 
    /// shape, i.e. ignoring the transform matrix in the shape.
    virtual void glRenderShape() = 0;
  };
}

#endif
