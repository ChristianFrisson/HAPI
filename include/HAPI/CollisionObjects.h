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
/// \file CollisionObjects.h
/// \brief Header file for CollisionObjects. The collision algorithm codes
/// are written with help of the book "Real-Time Collision Detection" by
/// Christer Ericson (Morgan Kaufmann, 2005). Home page of book
/// http://realtimecollisiondetection.net/
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __COLLISIONSTRUCTURES_H__
#define __COLLISIONSTRUCTURES_H__

#include <HAPI/HAPITypes.h>
#include <HAPI/IntersectionInfo.h>

#include <vector>
#include <assert.h>
#include <H3DUtil/RefCountedClass.h>
#include <H3DUtil/AutoRef.h>
#include <H3DUtil/AutoRefVector.h>

namespace HAPI {
  class Constraints; // Forward declaration

  namespace Collision {
    /// Intersect segment S(t)=sa+t(sb-sa), 0<=t<=1 against cylinder 
    /// specified by p, q and r. The segment finds the first along the segment.
    /// If the segment intersects the cylinder the point of intersection can
    /// be calculated from t.
    /// \returns true if segment intersects cylinder.
    bool HAPI_API intersectSegmentCylinder( Vec3 sa, Vec3 sb, 
                                            Vec3 p, Vec3 q, 
                                            HAPIFloat r, 
                                            HAPIFloat & t );

    class PlaneConstraint; 

    
    /// \ingroup AbstractClasses
    /// \class CollisionObject
    /// \brief The base class for objects that can be used in collision
    /// detection.
    class HAPI_API CollisionObject : public H3DUtil::RefCountedClass {
    public:
      /// Constructor.
      CollisionObject(){};
      
      CollisionObject( bool _use_lock ) :
        H3DUtil::RefCountedClass( _use_lock ){};
         
      typedef HAPI::Constraints Constraints;
      //typedef std::vector< Collision::PlaneConstraint > Constraints;

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 ) {}
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) = 0;

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
                                  IntersectionInfo &result,
                               FaceType face = Collision::FRONT_AND_BACK ) = 0;

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ){ return false; }

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result){ return false; }
    
    /// Render the object. The caller of the function need to set up OpenGL
    /// state in case the rendering should be done differently
    /// (wireframe for example).
    virtual void render() {}
  };

    /// \ingroup AbstractClasses
    /// \class BoundObject
    /// \brief The base class for bounding objects.
    class HAPI_API BoundObject: public CollisionObject {
    public:
      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) = 0;

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to ) {
        IntersectionInfo info;
        return( insideBound( from ) || 
                insideBound( to ) || 
                lineIntersect( from, to, info ) );
                
      }

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        return( insideBound( from ) || 
                insideBound( to ) || 
                movingSphereIntersect( radius, from, to ) );
      }

      /// The closest point on the bound to the given point. To know the
      /// closest point to the primitives in the bound, use closestPoint.
      virtual Vec3 boundClosestPoint( const Vec3 &p ) = 0;

    };

    /// \ingroup AbstractClasses
    /// \class BoundPrimitive
    /// \brief The base class for simple bound primitives such as bounding
    /// spheres and bounding boxes.
    class HAPI_API BoundPrimitive: public BoundObject {
    public:
      /// Constructor.
      BoundPrimitive() {}

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const std::vector< Vec3 >&points ) = 0;

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      virtual Vec3 longestAxis() const = 0;
    };
    
    /// \ingroup AbstractClasses
    /// \class GeometryPrimitive
    /// \brief The base class for all geometry primitives used in collision
    /// detection.
    class HAPI_API GeometryPrimitive: public CollisionObject {
    public:
      /// Get constraint planes of the shape. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Returns a point representing the primitive. Is used for example when
      /// building BinaryTreeBound.
      virtual Vec3 pointRepresentation() const { return Vec3(); }

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

    };
    
    /// \ingroup CollisionStructures
    /// \class Plane
    /// \brief A collision primitive for a plane.
    ///
    /// The plane is defined by a point and a normal. The given normal
    /// is assumed to be of length 1.
    class HAPI_API Plane: public GeometryPrimitive {
    public:
      /// Constructor.
      /// \param p A point in the plane.
      /// \param n The normal of the plane. The normal is assumed to be
      /// normalized.
      Plane( const Vec3 &p, const Vec3 &n ):
        point( p ), normal( n ) {}
      
      /// Returns a point representing the primitive. In this case it is the 
      /// point defining the plane.
      inline virtual Vec3 pointRepresentation() const {
        return point;
      }
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param n Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &n,
                                 Vec3 &tex_coord  ) {
        HAPIFloat t = normal * ( p - point );
        closest_point = p - t * normal;
        n = normal;
        tex_coord = Vec3();
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// A point in the plane.
      Vec3 point;

      /// The normal of the plane.
      Vec3 normal;
    };


    /// \ingroup CollisionStructures
    /// \class Sphere
    /// \brief A collision primitive for a sphere.
    ///
    /// The Sphere is defined by a center and a radius.
    class HAPI_API Sphere: public GeometryPrimitive {
    public:
      /// Constructor.
      /// \param _center The center of the sphere.
      /// \param _radius The radius of the sphere.
      Sphere( const Vec3 &_center, HAPIFloat _radius );

      /// Destructor
      ~Sphere();
      
      /// Returns a point representing the primitive. In this case it is the 
      /// center of the sphere.
      inline virtual Vec3 pointRepresentation() const {
        return center;
      }

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// The coordinate of the center of the sphere.
      Vec3 center;

      /// The radius of the sphere.
      HAPIFloat radius;
    };


    /// \ingroup CollisionStructures
    /// \class Triangle
    /// \brief Represents a triangle primitive.
    class HAPI_API Triangle: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Triangle() {}

      /// Constructor.
      /// a, b and c are the coordinates for each vertex
      /// ta, tb, and tc are the texture coordinates for each vertex.
      Triangle( const Vec3& a_, const Vec3& b_, const Vec3& c_,
                const Vec3& ta_ = Vec3(), 
                const Vec3& tb_ = Vec3(), 
                const Vec3& tc_ = Vec3()) : 
        a(a_),b(b_),c(c_),
        ta( ta_ ), tb( tb_ ), tc( tc_ ) {
        ab = b - a;
        ac = c - a;
        normal = ab % ac;
        normal.normalizeSafe();
      }

      bool getConstraint(  const Vec3 &point,
                           Collision::PlaneConstraint *constraint,
                           FaceType face = Collision::FRONT_AND_BACK );

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// \deprecated This function is replaced with movingSphereIntersect and will be removed in
      /// future versions.
      /// \cond MAKE_SURE_DOXYGEN_IS_NOT_FOOLED_BY_MACRO
      DEPRECATED("movingSphereIntersect", )
      /// \endcond
      bool movingSphereIntersectRobust( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );


      /// Returns a point representing the primitive. In this case it is the 
      /// center of the triangle.
      inline virtual Vec3 pointRepresentation() const {
        return ( a + b + c ) / 3;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();
      
      
    public:
      /// The corners of the triangle.
      Vec3 a,b,c;

      /// The texture coordinates at the corners of the triangle.
      Vec3 ta, tb, tc;

      // The normalized normal of the triangle plane. Help variable for
      // collision functions.
      Vec3 normal;

      // The vectors from a to c and from a to b. Help variables for collision
      // functions.
      Vec3 ac, ab;
    };

    /// \ingroup CollisionStructures
    /// \class LineSegment
    /// \brief Represents a line segment primitive.
    ///
    /// A line segment is a line from point a to point b.
    class HAPI_API LineSegment: public GeometryPrimitive  {
    public:
      /// Default constructor.
      LineSegment() {}

      /// Constructor.
      /// \param _start is the point where the linesegment starts.
      /// \param _end is the point where the linesegement ends.
      LineSegment( const Vec3& _start, const Vec3& _end) : 
        start( _start ), end( _end ) {}

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Returns the closest point on the object to the line segment
      /// from -> to.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param s The point on line segment from->to at which closest
      /// point is can be calculated as from + s * ( to - from )
      /// \param t The point on this LineSegment at which closest
      /// point is can be calculated as start + t * ( end - start )
      /// \param c0 The closest point on line from->to.
      /// \param c1 The closest point on this LineSegment.
      /// \returns Square distance between the closest points.
      HAPIFloat closestPointOnLine( const Vec3 &from, const Vec3 &to,
                                    HAPIFloat &s, HAPIFloat &t,
                                    Vec3 &c0, Vec3 &c1 );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the line segment.
      inline virtual Vec3 pointRepresentation() const {
        return ( start + end ) / 2;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();
      
      /// The start of the line segment.
      Vec3 start;
      /// The end of the line segment.
      Vec3 end;

    protected:
      inline HAPIFloat clamp( HAPIFloat n, HAPIFloat min, HAPIFloat max) {
        if (n < min) return min;
        if (n > max) return max;
        return n;
      }
    };

    /// \ingroup CollisionStructures
    /// \class Point
    /// \brief Represents a point primitive.
    class HAPI_API Point: public GeometryPrimitive  {
    public:
      /// Default constructor.
      Point() {}

      /// Constructor.
      Point( const Vec3& _p ) : 
        position( _p ) {}

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord ) {
        closest_point = position;
        normal = p - closest_point;
        normal.normalizeSafe();
        tex_coord = Vec3(); 
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. This primitive is a point
      /// so the pointRepresentation is easy to calculate.
      inline virtual Vec3 pointRepresentation() const {
        return position;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();
      
      /// The position of the point
      Vec3 position;
    };

    /// \ingroup CollisionStructures
    /// \class Cylinder
    /// \brief Represents a cylinder primitive. The cylinder is specified
    /// by radius and height and the center axis of the cylinder is along
    /// the positive y-axis. The cylinder starts in origo.
    /// Texture coordinates are calculated as the following:
    /// On the cylinder part a texture is assumed to wrap counterclockwise
    /// seen from above, starting at the back of the cylinder. There is a
    /// vertical seem at the back of the cylinder intersecting x=0.
    /// On the top and bottom caps a texture is assumed to be centered
    /// in the middle of the cap. Seen from above a texture would appear
    /// right side up when up is in the direction of the -z axis.
    class HAPI_API Cylinder: public Collision::GeometryPrimitive {
    public:
      // Used in calculations to know on which part of the cylinder a point
      // is, for example a point of intersection.
      typedef enum {
        CYLINDER = 0,
        START_CAP = 1,
        END_CAP = 2
      } CylinderPart;

      /// Default constructor.
      Cylinder() {}

      /// Constructor.
      /// \param _radius The cylinder radius.
      /// \param _height The height of the cylinder
      /// \param _start_cap If true then the cylinder has a cap at y = 0.
      /// \param _end_cap If true then the cylinder has a cap at y = _height.
      Cylinder( const HAPIFloat &_radius,
                const HAPIFloat &_height,
                bool _start_cap = true,
                bool _end_cap = true ) :
        height( _height ),
        radius( _radius ),
        start_cap( _start_cap ),
        end_cap( _end_cap ) {}

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the cylinder.
      inline virtual Vec3 pointRepresentation() const {
        return Vec3( 0, height/2, 0 );
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Given a point on the cylinder return the texture coordinate.
      /// \param point The point on the cylinder.
      /// \param part Which part of the cylinder the point is on.
      inline Vec3 getTextureCoordinate( const Vec3 &point,
                                        CylinderPart part ) {
        if( part == Cylinder::CYLINDER ) {
          return Vec3( ( H3DUtil::Constants::pi +
                         H3DUtil::H3DAtan2( point.x, point.z ) )
                        / ( 2 * H3DUtil::Constants::pi ),
                       point.y / height,
                       0 );
        } else {
          HAPIFloat radius_2 = 2 * radius;
          return Vec3( (point.x + radius) / radius_2,
                       (-point.z + radius ) / radius_2, 0 );
        }
      }


      /// The height of the cylinder.
      HAPIFloat height;

      /// The radius of the cylinder.
      HAPIFloat radius;

      /// If true then the cylinder have a cap at the start_point.
      /// True by default.
      bool start_cap;

      /// If true then the cylinder have a cap at the end_point.
      /// True by default.
      bool end_cap;
    protected:

      /// Detect collision between a line segment and the object.
      /// For internal use.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param start_point The start point of the cylinder. Assumed to be
      /// somewhere on the y-axis.
      /// \param end_point The end point of the cylinder. Assumed to be
      /// somewhere on the y-axis.
      /// \param _radius The radius of the cylinder to intersect with. In order
      /// to get intersection with cylinders in the same place but other
      /// radius.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param part Returns which part of the cylinder intersection occured
      /// at.
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      bool lineIntersect( const Vec3 &from, 
                          const Vec3 &to,
                          const HAPIFloat &start_point,
                          const HAPIFloat &end_point,
                          HAPIFloat _radius,
                          IntersectionInfo &result,
                          CylinderPart &part,
                          FaceType face = Collision::FRONT_AND_BACK );

      /// Get the closest point and normal on the object to the given point p.
      /// For internal use.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      /// \param inside Set to true if closest point is inside cylinder.
      /// \param part Set which part of the cylinder the closest point is on.
      void closestPoint( const Vec3 &p,
                         Vec3 &closest_point,
                         Vec3 &normal,
                         Vec3 &tex_coord,
                         bool &inside,
                         CylinderPart &part );
    };

    /// \ingroup CollisionStructures
    /// \class AABox
    /// \brief Represents an axis aligned box primitive.
    /// Texture coordinates are calculated as if
    /// textures are applied individually to each face of the box. On the
    /// front (+Z), back (-Z), right (+X), and left (-X) faces of the box,
    /// when viewed from the outside with the +Y-axis up, the texture is
    /// mapped onto each face with the same orientation as if the image were
    /// displayed normally in 2D. On the top face of the box (+Y), when viewed
    /// from above and looking down the Y-axis toward the origin with the
    /// -Z-axis as the view up direction, the texture is mapped onto the face
    /// with the same orientation as if the image were displayed normally in
    /// 2D. On the bottom face of the box (-Y), when viewed from below looking
    /// up the Y-axis toward the origin with the +Z-axis as the view up
    /// direction, the texture is mapped onto the face with the same
    /// orientation as if the image were displayed normally in
    /// 2D.
    class HAPI_API AABox: public Collision::GeometryPrimitive {
    public:
      typedef enum {
        INSIDE = 0,
        OUTSIDE = 1,
        ON_SURFACE = 2
      } PointLocation;

      /// Default constructor.
      AABox() {}

      /// Constructor.
      /// \param _min The minimum corner of the box. It is assumed that
      /// _min.x <= _max.x and _min.y <= _max.y and _min.z <= _max.z.
      /// \param _max The maximum corner of the box.
      AABox( const Vec3& _min,
             const Vec3& _max ) {
        min = _min;
        max = _max;
      }

      /// Get the center of the box.
      inline Vec3 getCenter() {
        return (max + min) / 2;
      }

      /// Get the size of the box.
      inline Vec3 getSize() {
        return max - min;
      }
      
      /// Get the corner with the largest coordinate for each axis.
      inline Vec3 getMaxCorner() {
        return max;
      }

      /// Get the corner with the smallest coordinate for each axis.
      inline Vec3 getMinCorner() {
        return min;
      }

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Calculates a matrix transforming from local space to texture space.
      /// \param point The point at which to find the tangent vectors
      /// \param result_mtx Where the result is stored.
      virtual void getTangentSpaceMatrix( const Vec3 &point,
                                          Matrix4 &result_mtx );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Returns a point representing the primitive. In this case it is the 
      /// center of the box.
      inline virtual Vec3 pointRepresentation() const {
        return ( min + max ) / 2;
      }

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Given a point and a normal on the box return the texture
      /// coordinate.
      /// \param point The point on the box.
      /// \param normal The outwards normal of the box at the point. The normal
      /// must not be zero length.
      inline Vec3 getTextureCoordinate( const Vec3 &point,
                                        const Vec3 &normal ) {
        if( normal.z > Constants::epsilon ) {
          return Vec3( ( point.x - min.x ) / ( max.x - min.x ),
                       ( point.y - min.y ) / ( max.y - min.y ),
                       0 );
        } else if( normal.z < -Constants::epsilon ) {
          return Vec3( ( point.x - max.x ) / ( min.x - max.x ),
                       ( point.y - min.y ) / ( max.y - min.y ),
                       0 );
        } else if( normal.x > Constants::epsilon ) {
          return Vec3( ( point.z - max.z ) / ( min.z - max.z ),
                       ( point.y - min.y ) / ( max.y - min.y ),
                       0 );
        } else if( normal.x < -Constants::epsilon ) {
          return Vec3( ( point.z - min.z ) / ( max.z - min.z ),
                       ( point.y - min.y ) / ( max.y - min.y ),
                       0 );
        } else if( normal.y > Constants::epsilon ) {
          return Vec3( ( point.x - min.x ) / ( max.x - min.x ),
                       ( point.z - max.z ) / ( min.z - max.z ),
                       0 );
        } else if( normal.y < -Constants::epsilon ) {
          return Vec3( ( point.x - min.x ) / ( max.x - min.x ),
                       ( point.z - min.z ) / ( max.z - min.z ),
                       0 );
        }
        // Should never come here.
        return Vec3();
      }

      /// The min corner of the axis aligned box.
      Vec3 min;
      /// The max corner of the axis aligned box.
      Vec3 max;
    protected:

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param _min The minimum corner of the axis aligned box.
      /// \param _max The maximum corner of the axis aligned box..
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return parameter for texture coordinate at closest 
      /// point
      /// \param point_location Contains information about whether the closest
      /// point is inside, outside or on the surface of the axis aligned box.
      virtual void closestPoint( const Vec3 &p,
                                 const Vec3 &_min,
                                 const Vec3 &_max,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord,
                                 PointLocation &point_location );

      /// Detect collision between a line segment and the object.
      /// For internal use.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param _min The minimum corner of the axis aligned box.
      /// \param _max The maximum corner of the axis aligned box..
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param from_location Contains information about whether the starting
      /// point of the line is inside, outside or on the surface of the axis
      /// aligned box.
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      bool lineIntersect( const Vec3 &from, 
                          const Vec3 &to,
                          const Vec3 &_min,
                          const Vec3 &_max,
                          IntersectionInfo &result,
                          PointLocation &from_location,
                          FaceType face = Collision::FRONT_AND_BACK );

      /// Support function used by movingSphereintersect.
      inline Vec3 corner( int n ) {
        return Vec3( ((n & 1) ? max.x : min.x ),
                     ((n & 2) ? max.y : min.y ),
                     ((n & 4) ? max.z : min.z ) );
      }
    };

    /// \ingroup CollisionStructures
    /// \class AABoxBound
    /// \brief Represents an axis-aligned bounding box.
    class HAPI_API AABoxBound: public BoundPrimitive {
    public:
      /// Constructor.
      AABoxBound( const Vec3 &_min = Vec3(),
                  const Vec3 &_max = Vec3() );

      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      inline virtual Vec3 longestAxis() const {
        Vec3 dims = max - min;
        
        if (dims.x >= dims.y && dims.x >= dims.z) return Vec3(1,0,0);
        if (dims.y >= dims.x && dims.y >= dims.z) return Vec3(0,1,0);
        else                          return Vec3(0,0,1);
      }

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const std::vector< Vec3 > &points );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK ) {
        return boundIntersect( from, to );
      }

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ) {
        Vec3 old_min = min;
        Vec3 old_max = max;

        Vec3 r = Vec3( radius, radius, radius ); 
        min -= r;
        max += r;

        bool res = boundIntersect( from, to );
        
        min = old_min;
        max = old_max;
        return res;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to );
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the given point.
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        Vec3 result;
        // for each coordinate axis, if the point coordinate value
        // is outside box, clamp it to the box, else keep it as it is
        for( int i = 0; i < 3; ++i ) {
          HAPIFloat v = p[i];
          if( v < min[i] ) v = min[i];
          if( v > max[i] ) v = max[i];
          result[i] = v;
        }
        return result;
      }

      /// The min corner of the bounding box.
      Vec3 min;

      /// The max corner of the bounding box.
      Vec3 max;
     
    };

    /// \ingroup CollisionStructures
    /// \class OrientedBoxBound
    /// \brief Represents an oriented bounding box.
    class HAPI_API OrientedBoxBound: public AABoxBound {
    public:
      
      /// Constructor.
      OrientedBoxBound( const Vec3 &_min = Vec3(),
                        const Vec3 &_max = Vec3(),
                        const Rotation &_orientation = Rotation() );

      /// Constructor.
      OrientedBoxBound( const Rotation &_orientation,
                        const Vec3 &_center,
                        const Vec3 &_size );

      
      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      inline virtual Vec3 longestAxis() const;

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const std::vector< Vec3 > &points );

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if 
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK ) {
        return boundIntersect( from, to );
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, 
                                   const Vec3 &to );
      
      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord  );

      /// The center of the oriented bounding box.
      Vec3 center;

      /// The orientation of the oriented bounding box.
      Rotation orientation;

      /// Half size of the bounding box. Used with center and orientation
      /// to define the oriented bounding box.
      Vec3 halfsize;
      
    };


    /// \ingroup CollisionStructures
    /// \class SphereBound
    /// \brief Represents a bounding sphere.
    class HAPI_API SphereBound: public BoundPrimitive {
    public:
      /// Default constructor.
      SphereBound();

      /// Constructor.
      SphereBound(const Vec3& c, HAPIFloat r);

      /// Destructor
      ~SphereBound();
      
      /// Render the object. The caller of the function need to set up OpenGL
      /// state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p );

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the given point.
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        Vec3 cp, tmp;
        closestPoint( p, cp, tmp, tmp );
        return cp;
      }

      /// Detect collision between a line segment and the object.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if
      /// line intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Update the bound primitive to contain all the given points.
      virtual void fitAroundPoints( const std::vector< Vec3 > &points );

      /// Returns a Vec3 specifying the longest axis of the bound primitive.
      /// 1 0 0 means the x-axis, 0 1 0 the y-axis and 0 0 1 the z-axis.
      virtual Vec3 longestAxis() const { return Vec3( 1, 0, 0 ); }

      /// The center of the sphere
      Vec3 center;

      /// The radius of the sphere.
      HAPIFloat radius;
    };

    /// \ingroup AbstractClasses
    /// \class BinaryBoundTree
    /// \brief The base class for bound objects structured as a binary tree.
    /// 
    /// Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same. Each leaf
    /// in the tree contains triangle, line and point primitives contained
    /// within its parent bound.
    class HAPI_API BinaryBoundTree: public BoundObject {
    public: 
      /// A function type for creating a bound primitive for a node in the tree.
      typedef BoundPrimitive *( *BoundNewFunc)(); 

      /// Convenience template for creating BoundNewFunc functions.
      template< class N >
      static BoundPrimitive *newInstance() { return new N; }

      /// Default constructor.
      BinaryBoundTree(  ): left( NULL ), right( NULL ), new_func( NULL ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. The func argument
      /// specifies a function for creating  bound of the wanted type in each
      /// tree node.
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      BinaryBoundTree( BoundNewFunc func, 
                       const std::vector< Triangle > &triangles,
                       unsigned int max_nr_triangles_in_leaf = 1 );

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// The func argument specifies
      /// a function for creating  bound of the wanted type in each tree node.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      BinaryBoundTree( BoundNewFunc func, 
                       const std::vector< Triangle > &triangle_vector,
                       const std::vector< LineSegment > &linesegment_vector,
                       const std::vector< Point > &point_vector,
                       unsigned int max_nr_triangles_in_leaf = 1 );


      /// Array of 3*(nr triangles in tree) triangle indices specifying
      /// for each triangle edge which triangle is its neighbour. The index
      /// is into the triangles received by the getAllTriangles function.
      const std::vector<int> &getNeighbours();

      /// Returns true if the tree is a leaf, i.e. has no sub-tress
      /// and hence just contains triangles. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      /// Get constraint planes of the shape. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Adds the triangles found in the tree, that are within the distance 
      /// radius from p, to the triangles vector.
      virtual void getTrianglesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             std::vector< Triangle > &triangles);

      /// Adds the triangles, lines and points found in the tree, that are 
      /// within the distance radius from p, to their respective vector.
      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                             std::vector< Triangle > &triangles,
                                             std::vector< LineSegment > &lines,
                                             std::vector< Point > &points );

      /// Adds the triangles that are intersected by the volume swept by a
      /// sphere when moving from "from" to "to".
      virtual void getTrianglesIntersectedByMovingSphere(
                    HAPIFloat radius,
                    Vec3 from,
                    Vec3 to,
                    std::vector< Triangle > &triangles);

      /// Adds the triangles, lines and points that are intersected by 
      /// the volume swept by a sphere when moving from "from" to "to".
      virtual void getPrimitivesIntersectedByMovingSphere(
                    HAPIFloat radius,
                    Vec3 from,
                    Vec3 to,
                    std::vector< Triangle > &triangles,
                    std::vector< LineSegment > &lines,
                    std::vector< Point > &points );

      /// Render the objects in the leafs. The caller of the function need to
      /// set up OpenGL state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void renderBounds( int depth );

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) {
        if( bound.get() )
          return bound->insideBound( p );
        else
          return false;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to ) {
        if( bound.get() )
          return bound->boundIntersect( from, to );
        else
          return false;
      }

      /// Detect collision between a line segment and the object. Will check
      /// for collision between the triangles, lines or points contained in the
      /// leaves of the tree and the line segment.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if line
      /// intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        if( bound.get() )
          return bound->boundMovingSphereIntersect( radius, from, to );
        else 
          return false;
      }

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the given point. If tree is a leaf,
      /// the closest point to the triangles, lines or points in the leaf is
      /// returned. To know the closest point to the primitives in the bound,
      /// use closestPoint.
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        if( !isLeaf() && bound.get() )
          return bound->boundClosestPoint( p );
        else {
          Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all triangles in the tree to the given vector.
      virtual void getAllTriangles( std::vector< Triangle > &triangles );

      /// Add all triangles, lines and points in the tree to the given vector.
      virtual void getAllPrimitives( std::vector< Triangle > &triangles,
                                    std::vector< LineSegment > &lines,
                                    std::vector< Point > &points );

      /// The bound for this node in the tree, will be a bound around
      /// all its children (left and right subtree).
      H3DUtil::AutoRef< BoundPrimitive > bound;

      /// The triangles for this node in the tree. Will only be non-empty
      /// for leafs.
      std::vector< Triangle > triangles;

      /// The line segments for this node in the tree. Will only be non-empty
      /// for leafs.
      std::vector< LineSegment > linesegments;

      /// The points for this node in the tree. Will only be non-empty
      /// for leafs.
      std::vector< Point > points;

      /// Array of 3*nr_triangles triangle indices specifying for each triangle
      /// edge which triangle is its neighbour. 
      std::vector< int > neighbours;

      /// The left subtree.
      H3DUtil::AutoRef< BinaryBoundTree > left;
      /// The right subtree.
      H3DUtil::AutoRef< BinaryBoundTree > right;
      /// The function to create new bounding primitives in for a tree node.
      BoundNewFunc new_func;
    };


    /// \ingroup CollisionStructures
    /// \class AABBTree
    /// \brief A BinaryBoundTree where the bounding primitive
    /// for each node is a AABoxBound (axis-aligned bounding box).
    class HAPI_API AABBTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      AABBTree(): BinaryBoundTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBTree( const std::vector< Triangle > &triangles,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< AABoxBound >),
                         triangles,
                         max_nr_triangles_in_leaf ) {}

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBTree( const std::vector< Triangle > &triangles,
                const std::vector< LineSegment > &linesegment_vector,
                const std::vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< AABoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };


    /// \ingroup CollisionStructures
    /// \class OBBTree
    /// \brief a BinaryBoundTree where the bounding primitive
    /// for each node is an OrientedBoxBound.
    class HAPI_API OBBTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      OBBTree(): BinaryBoundTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBTree( const std::vector< Triangle > &triangles,
               unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< OrientedBoxBound >), triangles, 
                         max_nr_triangles_in_leaf ) {}
      
      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBTree( const std::vector< Triangle > &triangles,
                const std::vector< LineSegment > &linesegment_vector,
                const std::vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< OrientedBoxBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    /// \ingroup CollisionStructures
    /// \class SphereBoundTree
    /// \brief A BinaryBoundTree where the bounding
    /// primitive for each node is a SphereBound object.
    class HAPI_API SphereBoundTree: public BinaryBoundTree {
    public:
      /// Default constructor.
      SphereBoundTree(): BinaryBoundTree() {}

      /// Constructor.
      /// Builds a binary tree from a vector of triangles. 
      /// max_nr_triangles_in_leaf specifies the maximum number of triangles 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SphereBoundTree( const std::vector< Triangle > &triangles,
                       unsigned int max_nr_triangles_in_leaf = 1): 
        BinaryBoundTree( &(newInstance< SphereBound > ),
                         triangles,
                         max_nr_triangles_in_leaf ) {}

      /// Builds a binary tree from a vector of triangles, lines and points.
      /// max_nr_triangles_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SphereBoundTree( const std::vector< Triangle > &triangles,
                const std::vector< LineSegment > &linesegment_vector,
                const std::vector< Point > &point_vector,
                unsigned int max_nr_triangles_in_leaf = 1 ):
        BinaryBoundTree( &(newInstance< SphereBound >), triangles,
                         linesegment_vector, point_vector,
                         max_nr_triangles_in_leaf ) {}
    };

    /// \ingroup AbstractClasses
    /// \class BBPrimitiveTree
    /// \brief The base class for bound objects using structured as a binary
    /// tree which contains pointers to the GeometryPrimitive class.
    ///
    /// Each node in the tree has a BoundPrimitive
    /// specifying a bound for itself and each subtree has the same. The leaf
    /// nodes can contain any GeometryPrimitive objects.
    class HAPI_API BBPrimitiveTree: public BoundObject {
    public: 
      /// A function type for creating a bound primitive for a node in the tree.
      typedef BoundPrimitive *( *BoundNewFunc)(); 

      /// Convenience template for creating BoundNewFunc functions.
      template< class N >
      static BoundPrimitive *newInstance() { return new N; }

      /// Default constructor.
      BBPrimitiveTree(  ): left( NULL ), right( NULL ), new_func( NULL ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of GeometryPrimitive pointers.
      /// The func argument specifies a function for creating  bound of the
      /// wanted type in each tree node. max_nr_primitives_in_leaf specifies
      /// the maximum number of primitives that are allowed to be in a bound
      /// of a leaf in the tree. 
      BBPrimitiveTree( BoundNewFunc func, 
                       const std::vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1 );

      /// Returns true if the tree is a leaf, i.e. has no sub-trees
      /// and hence just contains primitives. false otherwise.
      inline bool isLeaf() { 
        return left.get() == NULL && right.get() == NULL; 
      }

      /// Get constraint planes of the object. A proxy of a haptics renderer
      /// will always stay above any constraints added.
      /// \param point The point to constrain.
      /// \param constraints Where to add the constraints.
      /// \param face Determines which faces of the shape will be seen as
      /// constraining.
      /// \param radius Only add constraints within this radius. If set to -1
      /// all constraints will be added.
      virtual void getConstraints( const Vec3 &point,
                                   Constraints &constraints,
                                   FaceType face = Collision::FRONT_AND_BACK,
                                   HAPIFloat radius = -1 );

      /// Adds the primitives found in the tree, that are 
      /// within the distance radius from p, to the vector of primitives.
      virtual void getPrimitivesWithinRadius( const Vec3 &p,
                                             HAPIFloat radius,
                                    std::vector< GeometryPrimitive * > &primitives);

      /// Adds the primitives that are intersected by the volume swept by a
      /// sphere when moving from "from" to "to".
      virtual void getPrimitivesIntersectedByMovingSphere(
                      HAPIFloat radius,
                      Vec3 from,
                      Vec3 to,
                      std::vector< GeometryPrimitive * > &primitives);

      /// Render the objects in the leafs. The caller of the function need to
      /// set up OpenGL state in case the rendering should be done differently
      /// (wireframe for example).
      virtual void render();

      /// Render the outlines of the tree at a given tree depth for debugging 
      /// purposes.
      virtual void renderBounds( int depth );

      /// Returns true if the given point is inside the bound, and
      /// false otherwise.
      virtual bool insideBound( const Vec3 &p ) {
        if( bound.get() )
          return bound->insideBound( p );
        else
          return false;
      }

      /// The boundIntersect returns true if the line segment intersects the
      /// bound or if the line segment is totally inside the bound.
      virtual bool boundIntersect( const Vec3 &from, const Vec3 &to ) {
        if( bound.get() )
          return bound->boundIntersect( from, to );
        else
          return false;
      }

      /// Detect collision between a line segment and the object. Will check
      /// for collision between the primitives contained in the leaves of the
      /// tree and the line segment.
      /// \param from The start of the line segment.
      /// \param to The end of the line segment.
      /// \param result Contains info about the closest intersection, if line
      /// intersects object
      /// \param face The sides of the object that can be intersected. E.g.
      /// if FRONT, intersections will be reported only if they occur from
      /// the front side, i.e. the side in which the normal points. 
      /// \returns true if intersected, false otherwise.
      virtual bool lineIntersect( const Vec3 &from, 
                                  const Vec3 &to,
                                  IntersectionInfo &result,
                                  FaceType face = Collision::FRONT_AND_BACK );

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to );

      /// Detect collision between a moving sphere and the object and returns
      /// information about the closest intersection. Is of course slower
      /// than the version of movingSphereIntersect which does not return
      /// information about the intersection.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \param result Contains information about the closest intersection
      /// if an intersection is detected.
      /// \returns true if intersected, false otherwise.
      virtual bool movingSphereIntersect( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to,
                                          IntersectionInfo &result);

      /// Detect collision between a moving sphere and the object.
      /// \param radius The radius of the sphere
      /// \param from The start position of the sphere
      /// \param to The end position of the sphere.
      /// \returns true if intersected, false otherwise.
      virtual bool boundMovingSphereIntersect( HAPIFloat radius,
                                               const Vec3 &from, 
                                               const Vec3 &to ) {
        if( bound.get() )
          return bound->boundMovingSphereIntersect( radius, from, to );
        else 
          return false;
      }

      /// Get the closest point and normal on the object to the given point p.
      /// \param p The point to find the closest point to.
      /// \param closest_point Return parameter for closest point
      /// \param normal Return parameter for normal at closest point.
      /// \param tex_coord Return paramater for texture coordinate at closest 
      /// point
      virtual void closestPoint( const Vec3 &p,
                                 Vec3 &closest_point,
                                 Vec3 &normal,
                                 Vec3 &tex_coord );

      /// The closest point on the bound to the given point. If tree is a leaf,
      /// the closest point to the primitives in the leaf is returned.
      /// To know the closest point to the primitives in the bound,
      /// use closestPoint
      virtual Vec3 boundClosestPoint( const Vec3 &p ) {
        if( !isLeaf() && bound.get() )
          return bound->boundClosestPoint( p );
        else {
          Vec3 cp, tmp;
          closestPoint( p, cp, tmp, tmp );
          return cp;
        }
      }

      /// Add all primitives in the tree to the given vector.
      virtual void getAllPrimitives(
        std::vector< GeometryPrimitive * > &primitives );

      /// The bound for this node in the tree, will be a bound around
      /// all its children (left and right subtree).
      H3DUtil::AutoRef< BoundPrimitive > bound;

      /// The primitives for this node in the tree. Will only be non-empty
      /// for leafs.
      H3DUtil::AutoRefVector< GeometryPrimitive > primitives;

      /// The left subtree.
      H3DUtil::AutoRef< BBPrimitiveTree > left;
      /// The right subtree.
      H3DUtil::AutoRef< BBPrimitiveTree > right;
      /// The function to create new bounding primitives in for a tree node.
      BoundNewFunc new_func;
    };


    /// \ingroup CollisionStructures
    /// \class AABBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding
    /// primitive for each node is a AABoxBound (axis-aligned bounding box).
    class HAPI_API AABBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      AABBPrimitiveTree(): BBPrimitiveTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      AABBPrimitiveTree( const std::vector< GeometryPrimitive * > &_primitives,
                unsigned int max_nr_primitives_in_leaf = 1 ):
        BBPrimitiveTree( &(newInstance< AABoxBound >),
                         _primitives,
                         max_nr_primitives_in_leaf ) {}
    };


    /// \ingroup CollisionStructures
    /// \class OBBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding
    /// primitive for each node is a OrientedBoxBound.
    class HAPI_API OBBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      OBBPrimitiveTree(): BBPrimitiveTree( ) {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      OBBPrimitiveTree( const std::vector< GeometryPrimitive * > &_primitives,
               unsigned int max_nr_primitives_in_leaf = 1 ):
        BBPrimitiveTree( &(newInstance< OrientedBoxBound >), _primitives, 
                         max_nr_primitives_in_leaf ) {}
    };

    /// \ingroup CollisionStructures
    /// \class SBPrimitiveTree
    /// \brief A BBPrimitiveTree where the bounding 
    /// primitive for each node is a SphereBound object.
    class HAPI_API SBPrimitiveTree: public BBPrimitiveTree {
    public:
      /// Default constructor.
      SBPrimitiveTree(): BBPrimitiveTree() {}

      /// Constructor.
      /// Builds a binary tree from a vector of primitives. 
      /// max_nr_primitives_in_leaf specifies the maximum number of primitives 
      /// that are allowed to be in a bound of a leaf in the tree. 
      SBPrimitiveTree( const std::vector< GeometryPrimitive * > &_primitives,
                       unsigned int max_nr_primitives_in_leaf = 1): 
        BBPrimitiveTree( &(newInstance< SphereBound > ),
                         _primitives,
                         max_nr_primitives_in_leaf ) {}
    };

}
  using Collision::PlaneConstraint;
}

#endif
