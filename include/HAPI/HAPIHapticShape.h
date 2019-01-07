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


  /// \ingroup AbstractClasses
  /// \class HAPIHapticShape
  /// \brief Base class for haptic shapes.
  ///
  /// Haptic shapes are geometrical objects that are
  /// rendered by letting their surfaces constrain the proxy. A HAPIHapticShape
  /// has a Surface object associated to it that defines the properties of the
  /// surface, e.g. stiffness and friction properties.
  ///
  /// Several functions below
  class HAPI_API HAPIHapticShape: public HAPIHapticObject,
                                  public Collision::CollisionObject
#ifdef HAVE_OPENGL
                                  , public HAPIGLShape
#endif
  {
  public:

    /// Constructor.
    /// \param _transform The transformation from local to global space.
    /// \param _surface The surface of the HAPIHapticShape.
    /// \param _touchable_face Tells which face (side) of the shape that
    /// should be possible to touch.
    /// \param _userdata Extra data supplied by the user. HAPIHapticShape
    /// have no idea what it contains.
    /// \param _shape_id The id of this HAPIHapticShape.
    /// \param _clean_up_func A function that can be used for cleaning up
    /// contents of userdata when this HAPIHapticShape is destroyed. The
    /// argument to the function will be the contents of userdata member
    /// variable.
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

      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      surface( _surface ),
      have_transform( true ),
      have_inverse( false ),
      transform( _transform ),
      forced_dynamic( false ),
      first_use_time( -1 ),
      clean_up_func( _clean_up_func ) {
        setTransform( _transform );
        inverse = transform.inverse();
        setNonUniformScalingFlag();
    }

    /// Constructor.
    /// \param _surface The surface of the HAPIHapticShape.
    /// \param _touchable_face Tells which face (side) of the shape that
    /// should be possible to touch.
    /// \param _userdata Extra data supplied by the user. HAPIHapticShape
    /// have no idea what it contains.
    /// \param _shape_id The id of this HAPIHapticShape.
    /// \param _clean_up_func A function that can be used for cleaning up
    /// contents of userdata when this HAPIHapticShape is destroyed. The
    /// argument to the function will be the contents of userdata member
    /// variable.
    HAPIHapticShape( HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticObject( ),
      Collision::CollisionObject( true ),

      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      surface( _surface ),
      have_transform( false ),
      have_inverse( false ),
      forced_dynamic( false ),
      first_use_time( -1 ),
      clean_up_func( _clean_up_func ),
      non_uniform_scaling( false ) {
        scale_factor = Vec3( 1.0, 1.0, 1.0 );
    }

    /// Constructor.
    /// \param _transform The transformation from local to global space.
    /// \param _velocity The velocity(m/s) the shape center of mass of the shape
    /// is moving in.
    /// \param _angular_velocity The rotational velocity around the center 
    /// of mass(in radians/s)
    /// \param _growth_rate The time derivative of the scale factor(scale units/s)
    /// \param _surface The surface of the HAPIHapticShape.
    /// \param _touchable_face Tells which face (side) of the shape that
    /// should be possible to touch.
    /// \param _userdata Extra data supplied by the user. HAPIHapticShape
    /// have no idea what it contains.
    /// \param _shape_id The id of this HAPIHapticShape.
    /// \param _clean_up_func A function that can be used for cleaning up
    /// contents of userdata when this HAPIHapticShape is destroyed. The
    /// argument to the function will be the contents of userdata member
    /// variable.
    HAPIHapticShape( const Matrix4 &_transform,
                     const Vec3 &_velocity,
                     const Rotation &_angular_velocity,
                     const Vec3 &_growth_rate,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticObject( ),
      Collision::CollisionObject( true ),

      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      surface( _surface ),
      have_transform( true ),
      have_inverse( false ),
      transform( _transform ),
      velocity( _velocity ),
      angular_velocity( _angular_velocity ),
      growth_rate( _growth_rate ),
      forced_dynamic( false ),
      first_use_time( -1 ),
      clean_up_func( _clean_up_func ) {
        setTransform( _transform );
        inverse = transform.inverse();
        setNonUniformScalingFlag();
    }

    /// Constructor.
    /// \param _velocity The velocity(m/s) the shape center of mass of the shape
    /// is moving in.
    /// \param _angular_velocity The rotational velocity around the center 
    /// of mass(in radians/s)
    /// \param _growth_rate The time derivative of the scale factor(scale units/s)
    /// \param _surface The surface of the HAPIHapticShape.
    /// \param _touchable_face Tells which face (side) of the shape that
    /// should be possible to touch.
    /// \param _userdata Extra data supplied by the user. HAPIHapticShape
    /// have no idea what it contains.
    /// \param _shape_id The id of this HAPIHapticShape.
    /// \param _clean_up_func A function that can be used for cleaning up
    /// contents of userdata when this HAPIHapticShape is destroyed. The
    /// argument to the function will be the contents of userdata member
    /// variable.
    HAPIHapticShape( const Vec3 &_velocity,
                     const Rotation &_angular_velocity,
                     const Vec3 & _growth_rate,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0
                     ):
      HAPIHapticObject( ),
      Collision::CollisionObject( true ),

      userdata( _userdata ),
      shape_id( _shape_id ),
      touchable_face( _touchable_face ),
      surface( _surface ),
      have_transform( false ),
      have_inverse( false ),
      velocity( _velocity ),
      angular_velocity( _angular_velocity ),
      growth_rate( _growth_rate ),
      forced_dynamic( false ),
      first_use_time( -1 ),
      clean_up_func( _clean_up_func ) {
        scale_factor = Vec3( 1.0, 1.0, 1.0 );
    }


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
    void transformShape( const Matrix4 &t );

    /// Set the transformation matrix from local to global space.
    void setTransform( const Matrix4 &t );

    /// Get the transformation matrix from local to global space.
    Matrix4 getTransform();

    /// Get the transformation matrix from local to global space that
    /// was used in last haptics loop.
    inline Matrix4 getLastTransform() {
      return last_transform;
    }

    /// Get the transformation matrix from global to local space that
    /// was used in last haptics loop.
    inline Matrix4 getLastInverse() {
      return last_inverse;
    }

    /// Initialize the current shape for a replace of the given shape
    /// in the haptics rendering(if s is NULL then it is not replacing
    /// any previous shape). This is usually used when a haptic
    /// shape is supposed to be replaced by another one with the same
    /// id. It means e.g. saving the previous transform matrices.
    void initializeTransfer( HAPIHapticShape *s );

    /// Advance the object according to velocity, angular velocity etc
    /// to new position given a time step. This will update the position,
    /// rotation and scale parameters of the shape.
    void moveTimestep( HAPITime dt );

    /// Advance the point according to velocity, angular velocity etc
    /// to new position given a time step. The new position is returned.
    Vec3 moveTimestep( const HAPI::Vec3 &p, HAPITime dt );

    /// Advance the vector according to angular velocity etc
    /// to new direction given a time step. The new direction is returned.
    Vec3 moveTimestepVec( const HAPI::Vec3 &p, HAPITime dt );

    /// Get the transformation matrix from global to local space.
    const Matrix4 &getInverse();

    /// Returns true if the object is a dynamic object, i.e. it is moving,
    /// rotating or changing size.
    bool isDynamic();

    /// Transform a point from shape local space to global space.
    Vec3 toGlobal( const Vec3 &p );

    /// Transform a vector from shape local space to global space.
    inline Vec3 toGlobalVec( const Vec3 &v ) {
      return getRotation() * v;
    }

    /// Transform a point from global space to shape local space.
    Vec3 toLocal( const Vec3 &p );

    /// Transform a vector from global space to shape local space.
    inline Vec3 toLocalVec( const Vec3 &v ) {
      return (-getRotation()) * v;
    }

    /// Get the position of the shape.
    inline const Vec3 &getPosition() const {
      return position;
    }

    /// Set the position of the shape.
    inline void setPosition( const Vec3 &p) {
      position = p;
      updateTransform();
    }

    /// Get the rotation of the shape.
    inline const Rotation & getRotation() const {
      return rotation;
    }

    /// Set the rotation of the shape.
    inline void setRotation( const Rotation &r) {
      rotation = r;
      updateTransform();
    }

    /// Get the scaling factor of the shape.
    inline const Vec3 &getScale() const {
      return scale_factor;
    }

    /// Set the (non-uniform) scaling factor of the shape.
    inline void setScale( HAPIFloat s ) {
      scale_factor = Vec3( s, s, s );
      updateTransform();
      setNonUniformScalingFlag();
    }

    /// Set the (uniform) scaling factor of the shape.
    inline void setScale( const Vec3 &s) {
      scale_factor = s;
      updateTransform();
      setNonUniformScalingFlag();
    }

    /// Get the velocity of the shape(m/s).
    inline const Vec3 &getVelocity() const {
      return velocity;
    }

    /// Set the velocity of the shape(m/s).
    inline void setVelocity( const Vec3 &v) {
      velocity = v;
    }

    /// Get the angular velocity of the shape(radians per second).
    inline const Rotation &getAngularVelocity() {
      return angular_velocity;
    }

    /// Set the angular velocity of the shape(radians per second).
    inline void setAngularVelocity( const Rotation &r) {
      angular_velocity = r;
    }

    /// Get the growth rate of the shape(scale units per second).
    inline const Vec3 &getGrowthRate() {
      return growth_rate;
    }

    /// Set the (uniform) growth rate of the shape(radians per second).
    inline void setGrowthRate( HAPIFloat g ) {
      growth_rate = Vec3( g, g, g );
    }

    /// Set the growth rate of the shape(radians per second).
    inline void setGrowthRate( const Vec3 &g ) {
      growth_rate = g;
    }

    /// If set to true the shape is considered to be dynamic even if it does
    /// not have velocity, angular velocity or growth. Can be used if you are
    /// moving the object manually by setPosition and want to avoid fallthrough
    /// caused by the object moving. If a shape is dynamic some renderers will
    /// do extra calculations to avoid fallthrough.
    inline void setForceDynamic( bool s ) {
      forced_dynamic = s;
    }

    /// Get the current force dynamic state.
    inline bool getForceDynamic() {
      return forced_dynamic;
    }

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to.
    /// \param cp Return parameter for closest point
    /// \param n Return parameter for normal at closest point.
    /// \param tc Return paramater for texture coordinate at closest point.
    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc );

    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about closest intersection, if 
    /// line intersects object
    /// \param face The sides of the object that can be intersected. E.g.
    /// if FRONT, intersections will be reported only if they occur from
    /// the front side, i.e. the side in which the normal points. 
    /// \param consider_movement If true, the movement of the shape in 
    /// the last haptics loop is also considered. Be aware that this can
    /// produce intersection points that are not between "from" and "to"
    /// if the shape has moved since the intersection point will then
    /// be moved with the shape.
    /// \returns true if intersected, false otherwise.
    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Collision::IntersectionInfo &result,
                                Collision::FaceType face,
                                bool consider_movement );
    
    /// Detect collision between a line segment and the object.
    /// \param from The start of the line segment.
    /// \param to The end of the line segment.
    /// \param result Contains info about closest intersection, if 
    /// line intersects object
    /// \param face The sides of the object that can be intersected. E.g.
    /// if FRONT, intersections will be reported only if they occur from
    /// the front side, i.e. the side in which the normal points. 
    /// \returns true if intersected, false otherwise.
    inline virtual bool lineIntersect( const Vec3 &from, 
                                       const Vec3 &to,
                                       Collision::IntersectionInfo &result,
                                       Collision::FaceType face ) {
      return lineIntersect( from, to, result, face, false );
    }

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

#ifdef HAVE_OPENGL    
    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes.
    virtual void glRender();
#endif

    /// Generate a new shape id.
    static int genShapeId();
    
    /// Deallocate all resources allocated to the shape id and return
    /// it for reuse.
    static void delShapeId( int id );

    /// Add an HAPIShapeRenderOptions instance with options on how
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
           i != options.end(); ++i ) {
        T *o = dynamic_cast< T * >( *i );
        if( o ) {
          option = o;
          return;
        }
      }
      option = NULL;
    } 

    /// Get the HAPISurfaceObject used by this shape.
    HAPISurfaceObject *getSurface() {
      return surface.get();
    }

    /// Set the HAPISurfaceObject used by this shape.
    void setSurface(  HAPISurfaceObject *s ) {
      surface.reset( s );
    }

    /// Get the face of the shape that is currently touchable.
    Collision::FaceType getTouchableFace() {
      return touchable_face;
    }

    /// Set which side/s of the shape that are to be touchable.
    void setTouchableFace( Collision::FaceType tf ) {
      touchable_face = tf;
    }

    /// Get the userdata of this shape.
    void *getUserData() {
      return userdata;
    }

    /// Get the id of the shape. -1 means that no id has been assigned to
    /// the shape.
    int getShapeId() {
      return shape_id;
    }

    /// Set the id of the shape. 
    void setShapeId( int id) {
      shape_id = id;
    }

    /// Calculates a matrix transforming a vector from global space
    /// to texture space of the shape.
    /// \param point The point at which to find the tangent vectors in global
    /// coordinates
    /// \param result_mtx Stores the calculated matrix.
    virtual void getTangentSpaceMatrix( const Vec3 &point,
                                        Matrix4 &result_mtx );

  protected:
    /// Update the transform matrix from its parts.
    void updateTransform();
    
    H3DUtil::AutoPtrVector< HAPIShapeRenderOptions > options;

    void *userdata;

    int shape_id;
    Collision::FaceType touchable_face;

    static int current_max_id;
    static std::list< int > free_ids;
  
    /// The Surface object describing the properties of the surface.
    H3DUtil::AutoRef< HAPISurfaceObject > surface;

    bool have_transform;
    bool have_inverse;
    /// Flag used internally to check if normals need to be transformed
    /// differently due to non uniform scaling.
    bool non_uniform_scaling;

    /// Sets the non_uniform_scaling flag.
    inline void setNonUniformScalingFlag() {
      if( H3DUtil::H3DAbs( scale_factor.x - scale_factor.y ) <
          H3DUtil::Constants::f_epsilon
          && H3DUtil::H3DAbs( scale_factor.y - scale_factor.z ) <
             H3DUtil::Constants::f_epsilon ) {
        non_uniform_scaling = false;
      } else {
        non_uniform_scaling = true;
      }
    }

    Matrix4 transform;
    Matrix4 inverse;

    // The transform matrix from the last haptics loop this shape was used.
    Matrix4 last_transform;

    // The inverse of the transform matrix from the last haptics loop this
    // shape was used.
    Matrix4 last_inverse;
    
    /// The velocity of the shape(m/s)
    Vec3 velocity;

    /// The angular velocity of the shape(radians/s)
    Rotation angular_velocity;

    /// The growth rate of the shape.
    Vec3 growth_rate;

    /// The position of the shape.
    Vec3 position;

    /// The orientation of the shape.
    Rotation rotation;

    /// The scaling of the shape.
    Vec3 scale_factor;

    bool forced_dynamic;
    HAPITime first_use_time;
    void (*clean_up_func)( void * );

    /// Get the closest point and normal on the object to the given point p.
    /// \param p The point to find the closest point to (in local coords).
    /// \param cp Return parameter for closest point (in local coords).
    /// \param n Return parameter for normal at closest point
    /// (in local coords).
    /// \param tc Return parameter for texture coordinate at closest point.
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

    /// Calculates a matrix transforming a vector from local space of the shape
    /// to texture space of the shape.
    /// \param point The point at which to find the tangent vectors
    /// in local coordinates
    /// \param result_mtx Stores the calculated matrix
    virtual void getTangentSpaceMatrixShape( const Vec3 &point,
                                             Matrix4 &result_mtx ) = 0;


#ifdef HAVE_OPENGL
    /// Render a graphical representation of the shape using OpenGL. This
    /// is used by the OpenHapticsRenderer when using feedback or depth
    /// buffer shapes. The rendering should be done in local space of the 
    /// shape, i.e. ignoring the transform matrix in the shape.
    virtual void glRenderShape() = 0;
#endif

    friend class OpenHapticsRenderer;
  };
}

#endif
