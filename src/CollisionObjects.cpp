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
/// \file CollisionStructures.cpp
/// \brief Cpp file for CollisionStructures
///
//
//////////////////////////////////////////////////////////////////////////////

#include "CollisionObjects.h"
#include <stack>

using namespace HAPI;
using namespace Bounds;

bool Bounds::intersectSegmentCylinder( Vec3 sa, Vec3 sb,
                                       Vec3 p, Vec3 q,
                                       HAPIFloat r, 
                                       HAPIFloat & t ) {
  Vec3 d = q - p, m = sa - p, n = sb - sa;
  HAPIFloat md = m * d;
  HAPIFloat nd = n * d;
  HAPIFloat dd = d * d;
  //Test if segment fully outside either endcap of cylinder
  if( md < 0.0f && md + nd < 0.0f )
    return false; // Segment outside 'p' side of cylinder
  if( md > dd && md + nd > dd )
    return false; // Segment outside 'q' side of cylinder
  HAPIFloat nn = n * n;
  HAPIFloat mn = m * n;
  HAPIFloat a = dd * nn - nd * nd;
  HAPIFloat k = m * m - r * r;
  HAPIFloat c = dd * k - md * md;
  if( H3DUtil::H3DAbs(a) < Constants::epsilon ) {
    // Segment runs parallel to cylinder axis
    if( c > 0.0f )
      return false; // 'a' and thus the segment lie outside cylinder.
    // Now known that segment intersects cylinder; figure out how it intersects
    if( md < 0.0f ) return false; // Intersect segment against 'p' endcap
    else if( md > dd ) return false; // Intersect segment against 'q' endcap
    else t = 0.0f; // 'a' lies inside cylinder
    return true;
  }
  HAPIFloat b = dd * mn - nd * md;
  HAPIFloat discr = b * b - a * c;
  if( discr < 0.0f )
    return false; // no real roots; no intersection
  t = (-b - H3DUtil::H3DSqrt( discr ) ) / a;
  if( t < 0.0f || t > 1.0f )
    return false; // Intersection lies outside segment
  if( md + t * nd < 0.0f ) {
    // Do not care about detecting collision with endcaps
    return false;
  } else if( md + t * nd > dd ) {
    // Do not care about detecting collision with endcaps
    return false;
  }
  // Segment intersects cylinder between the endcaps; t is correct
  return true;
}

bool PlaneConstraint::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     Bounds::IntersectionInfo &result ) {
      Vec3 from_to = to - from;
      if( normal * from_to > Constants::epsilon ) { 
        /*if( result.normal.z == 0.5 ) {
          cerr << "a" << endl;
          cerr << normal * from_to << endl;
          }*/
        return false;
      }


      HAPIFloat denom = normal * from_to;
      if( denom * denom < Constants::epsilon ) {
        /*        if( result.normal.z == 0.5 )
                  cerr << "b" << endl; */
        return false;
      }
      HAPIFloat u = ( normal * ( point - from ) ) / denom; 
      /*      if( result.normal.z == 0.5 )
              cerr << u << endl;*/
      if( u <= 1 + Constants::epsilon && u >= 0 - Constants::epsilon ) {

        if( u < 0 ) u = 0;
        if( u > 1 ) u = 1;
                //cerr << u << endl;
        result.point = from + u * from_to;
        result.normal = normal;
        // TODO: temp. Mybe use derivatives to get correct tex coord
        result.tex_coord = tex_coord;
        return true;
      }
      return false;
    }


void AABoxBound::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
    glColor3d( 1, 1, 0 );
  glBegin( GL_LINE_STRIP );
  glVertex3d( min.x, min.y, min.z );
  glVertex3d( min.x, max.y, min.z );
  glVertex3d( max.x, max.y, min.z );
  glVertex3d( max.x, min.y, min.z );
  glVertex3d( min.x, min.y, min.z );
  glEnd();
  
  glBegin( GL_LINE_STRIP );
  glVertex3d( min.x, min.y, max.z );
  glVertex3d( min.x, max.y, max.z );
  glVertex3d( max.x, max.y, max.z );
  glVertex3d( max.x, min.y, max.z );
  glVertex3d( min.x, min.y, max.z );
  glEnd();
  
  glBegin( GL_LINES );
  glVertex3d( min.x, min.y, max.z );
  glVertex3d( min.x, min.y, min.z );
  glVertex3d( max.x, min.y, max.z );
  glVertex3d( max.x, min.y, min.z );
  glVertex3d( min.x, max.y, max.z );
  glVertex3d( min.x, max.y, min.z );
  glVertex3d( max.x, max.y, max.z );
  glVertex3d( max.x, max.y, min.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

void AABoxBound::fitAroundPoints( const vector< Vec3 > &points ) {
  if( points.size() == 0 ) {
    max = Vec3( 0, 0, 0 );
    min = Vec3( 0, 0, 0 );
    return;
  }
  min = points[0];
  max = points[0];
  
  for(unsigned int i=1 ; i<points.size() ; i++) {
    if (points[i].x < min.x) min.x = points[i].x;
    if (points[i].y < min.y) min.y = points[i].y;
    if (points[i].z < min.z) min.z = points[i].z;
    
    if (points[i].x > max.x) max.x = points[i].x;
    if (points[i].y > max.y) max.y = points[i].y;
    if (points[i].z > max.z) max.z = points[i].z;
  }
}


bool AABoxBound::boundIntersect( const Vec3 &from, 
                                 const Vec3 &to ) {
  Vec3 c = 0.5f*(min+max);
	Vec3 e = max-c;
	Vec3 m = 0.5f*(from+to);
	Vec3 d = to-m;

	m = m-c;

  HAPIFloat adx = H3DUtil::H3DAbs(d.x);
	if (H3DUtil::H3DAbs(m.x) > e.x + adx) return false;
  HAPIFloat ady = H3DUtil::H3DAbs(d.y);
	if (H3DUtil::H3DAbs(m.y) > e.y + ady) return false;
	HAPIFloat adz = H3DUtil::H3DAbs(d.z);
	if (H3DUtil::H3DAbs(m.z) > e.z + adz) return false;

  //	bias with an epsilon to increase robustness when segment is parallel 
  // to a box plane...
	HAPIFloat epsilon = (e*e)/100000;
	adx += epsilon;
	ady += epsilon;
	adz += epsilon;

	if (H3DUtil::H3DAbs(m.y*d.z - m.z*d.y) > e.y*adz + e.z*ady) return false;
	if (H3DUtil::H3DAbs(m.z*d.x - m.x*d.z) > e.x*adz + e.z*adx) return false;
	if (H3DUtil::H3DAbs(m.x*d.y - m.y*d.x) > e.x*ady + e.y*adx) return false;

	return true;
}



void AABoxBound::closestPoint( const Vec3 &p,
                               Vec3 &closest_point,
                               Vec3 &normal,
                               Vec3 &tex_coord ) {
  // TODO: implement
  assert( false );
}

bool AABoxBound::insideBound( const Vec3 &p ) {
  return ( p.x <= max.x &&
           p.x >= min.x &&
           p.y <= max.y &&
           p.y >= min.y &&
           p.z <= max.z &&
           p.z >= min.z );
}

void SphereBound::closestPoint( const Vec3 &p,
                                Vec3 &closest_point,
                                Vec3 &normal,
                                Vec3 &tex_coord ) {
  Vec3 dir = p - center;
  if( dir * dir < Constants::epsilon ) {
    closest_point = center + Vec3( radius, 0, 0 ); 
    normal = Vec3( 1, 0, 0 );
  } else {
    dir.normalize();
    closest_point = center + dir * radius;
    normal = dir;
  }
  // Todo
  tex_coord = Vec3();
}

bool SphereBound::insideBound( const Vec3 &p ) {
  Vec3 v = p - center;
  HAPIFloat d2 = v * v;
  HAPIFloat r2 = radius * radius;
  return d2 <= r2;
}

bool SphereBound::boundIntersect( const Vec3 &from, 
                                  const Vec3 &to ) {
  Vec3 ab = to - from;
	Vec3 ap = center-from;
	Vec3 bp = center-to;
	HAPIFloat r2 = radius * radius;

	HAPIFloat e = ap * ab;
	if (e<0)  return ap*ap <= r2;
	
	HAPIFloat f = ab * ab;
	if (e>=f) return bp*bp <= r2;
	
	return (ap*ap - e*e/f) <= r2;
}

bool SphereBound::movingSphereIntersect( HAPIFloat radius,
                                              const Vec3 &from, 
                                              const Vec3 &to ) {
  Sphere sphere( center, radius );
  return sphere.movingSphereIntersect( radius, from, to );
}

bool SphereBound::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                 FaceType face ) {
  Sphere sphere( center, radius );
  return sphere.lineIntersect( from, to, result, face );
}

void SphereBound::render( ) {
  if( !gl_quadric ) gl_quadric = gluNewQuadric();
  glDisable( GL_LIGHTING );
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslated( center.x, center.y, center.z );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
    glColor3d( 1, 1, 0 );
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  gluSphere( gl_quadric, radius, 10, 10 );
  glPopMatrix();
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  glEnable( GL_LIGHTING );
}

void SphereBound::fitAroundPoints( const vector< Vec3 > &points ) {
  if( points.size() == 0 ) {
    center = Vec3( 0, 0, 0 );
    radius = 0;
    return;
  }

	AABoxBound box;
  box.fitAroundPoints( points );
	Vec3 c = 0.5 * (box.min + box.max);
	HAPIFloat r = 0;
	
  //	find largest square distance
	for(unsigned int i=0 ; i<points.size() ; i++) {
    Vec3 from_center = points[i]-c;
    HAPIFloat d = from_center * from_center;
    if (d > r) r = d;
  }
	
  center = c;
  radius = H3DUtil::H3DSqrt( r );
}

struct StackElement {
  BinaryBoundTree *tree;
  vector< int > triangles;
  vector< int > linesegments;
  vector< int > points;
}; 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                                  const vector< Triangle > &triangle_vector,
                                  unsigned int max_nr_triangles_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElement > stack;
	
  if( triangle_vector.size() == 0 ) return;
 
  //	add the starting subtree to stack...
	StackElement start;
	start.tree = this;

  vector< Vec3 > point_reps;
  point_reps.reserve( triangle_vector.size() );

	start.triangles.reserve( triangle_vector.size() );
  for( unsigned int i = 0; i < triangle_vector.size(); i++ ) {
    start.triangles.push_back( i );
    point_reps.push_back( triangle_vector[i].pointRepresentation() );
  }



	stack.push( start );
  //cerr << start.triangles.size() << endl;
  //	BUILD TREE
	while( !stack.empty() ) {
		const std::vector< int > &stack_triangles = stack.top().triangles;
		BinaryBoundTree* stack_tree = stack.top().tree;

	  if (max_nr_triangles_in_leaf < 0 ||
        stack_triangles.size() <= max_nr_triangles_in_leaf ) {
      //	build a leaf
			stack_tree->triangles.reserve( stack_triangles.size() );
      for( unsigned int i = 0; i < stack_triangles.size(); i++ ) {
        stack_tree->triangles.push_back( triangle_vector[stack_triangles[i]] );
      }
      stack.pop();
			continue;
		}

		std::vector<Vec3 > fitting_points;
		points.reserve( stack_triangles.size() * 3 );
		for (unsigned int i=0 ; i<stack_triangles.size() ; i++ ) {
      const Triangle &tri = triangle_vector[ stack_triangles[i ] ];
      fitting_points.push_back( tri.a );
      fitting_points.push_back( tri.b );
      fitting_points.push_back( tri.c);
    }
    
    //	build bounding shape
		stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( fitting_points );
		
    //	DIVIDE SUBSET AND RECURSE
    //	longest axis
		Vec3 axis = stack_tree->bound->longestAxis();

    //	middle point of current set
		Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; i++) {
      mid = mid + point_reps[stack_triangles[i]];
    }

		mid = (1.0f/stack_triangles.size()) * mid;	//divide by N
    
    //	build subsets based on axis/middle point
		std::vector<int> left;
		std::vector<int> right;
    
    left.reserve( stack_triangles.size() / 2 );
    right.reserve( stack_triangles.size() / 2 );

    
		for( unsigned int i = 0 ; i<stack_triangles.size() ; i++ ) {
      Vec3 mid_to_point = 
        point_reps[ stack_triangles[i] ] - mid;
      if ( mid_to_point * axis < 0 )
				left.push_back(stack_triangles[i]);
			else
				right.push_back(stack_triangles[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left.size() == 0) || (right.size() == 0) ) {
			left.clear();
			right.clear();
			for (unsigned int i = 0 ; i<stack_triangles.size()/2 ; i++) 
        left.push_back(stack_triangles[i]);
			for (unsigned int i = stack_triangles.size()/2 ; i<stack_triangles.size() ; i++) 
        right.push_back(stack_triangles[i]);
		}
		
		stack.pop();

    //	do recurse
		if ( left.size() != 0) {
      stack_tree->left.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->left.get();
			element.triangles.swap( left );
			stack.push( element );
		}
		
		if ( right.size() != 0) {
			stack_tree->right.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->right.get();
			element.triangles.swap( right );
			stack.push( element );
		}
	}
} 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                       const vector< Triangle > &triangle_vector,
                       const vector< LineSegment > &linesegment_vector,
                       const vector< Point > &point_vector,
                       unsigned int max_nr_triangles_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElement > stack;
	
  if( triangle_vector.size() == 0 &&
      linesegment_vector.size() == 0 &&
      point_vector.size() == 0 ) return;
 
  //	add the starting subtree to stack...
	StackElement start;
	start.tree = this;

  vector< Vec3 > point_reps_triangles;
  point_reps_triangles.reserve( triangle_vector.size() );

  vector< Vec3 > point_reps_linesegments;
  point_reps_linesegments.reserve( linesegment_vector.size() );

  vector< Vec3 > point_reps_points;
  point_reps_points.reserve( point_vector.size() );

	start.triangles.reserve( triangle_vector.size() );
  for( unsigned int i = 0; i < triangle_vector.size(); i++ ) {
    start.triangles.push_back( i );
    point_reps_triangles.push_back( triangle_vector[i].pointRepresentation() );
  }

  start.linesegments.reserve( linesegment_vector.size() );
  for( unsigned int i = 0; i < linesegment_vector.size(); i++ ) {
    start.linesegments.push_back( i );
    point_reps_linesegments.push_back(
      linesegment_vector[i].pointRepresentation() );
  }

  start.points.reserve( point_vector.size() );
  for( unsigned int i = 0; i < point_vector.size(); i++ ) {
    start.points.push_back( i );
    point_reps_points.push_back( point_vector[i].pointRepresentation() );
  }

	stack.push( start );
  //cerr << start.triangles.size() << endl;
  //	BUILD TREE
	while( !stack.empty() ) {
		const std::vector< int > &stack_triangles = stack.top().triangles;
    const std::vector< int > &stack_linesegments = stack.top().linesegments;
    const std::vector< int > &stack_points = stack.top().points;
		BinaryBoundTree* stack_tree = stack.top().tree;

	  if (max_nr_triangles_in_leaf < 0 ||
        ( stack_triangles.size() <= max_nr_triangles_in_leaf &&
          stack_linesegments.size() <= max_nr_triangles_in_leaf &&
          stack_points.size() <= max_nr_triangles_in_leaf ) ) {
      //	build a leaf
			stack_tree->triangles.reserve( stack_triangles.size() );
      for( unsigned int i = 0; i < stack_triangles.size(); i++ ) {
        stack_tree->triangles.push_back( triangle_vector[stack_triangles[i]] );
      }

      stack_tree->linesegments.reserve( stack_linesegments.size() );
      for( unsigned int i = 0; i < stack_linesegments.size(); i++ ) {
        stack_tree->linesegments.push_back(
          linesegment_vector[stack_linesegments[i]] );
      }

      stack_tree->points.reserve( stack_points.size() );
      for( unsigned int i = 0; i < stack_points.size(); i++ ) {
        stack_tree->points.push_back( point_vector[stack_points[i]] );
      }
      stack.pop();
			continue;
		}

		std::vector<Vec3 > fitting_points;
    fitting_points.reserve( stack_triangles.size() * 3 +
      stack_linesegments.size() * 2 + stack_points.size() );
		for (unsigned int i=0 ; i<stack_triangles.size() ; i++ ) {
      const Triangle &tri = triangle_vector[ stack_triangles[i ] ];
      fitting_points.push_back( tri.a );
      fitting_points.push_back( tri.b );
      fitting_points.push_back( tri.c);
    }

    for (unsigned int i=0 ; i<stack_linesegments.size() ; i++ ) {
      const LineSegment &ls = linesegment_vector[ stack_linesegments[i ] ];
      fitting_points.push_back( ls.start );
      fitting_points.push_back( ls.end );
    }

    for (unsigned int i=0 ; i<stack_points.size() ; i++ ) {
      const Point &p = point_vector[ stack_points[i ] ];
      fitting_points.push_back( p.position );
    }
    
    //	build bounding shape
		stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( fitting_points );
		
    //	DIVIDE SUBSET AND RECURSE
    //	longest axis
		Vec3 axis = stack_tree->bound->longestAxis();

    //	middle point of current set
		Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; i++) {
      mid = mid + point_reps_triangles[stack_triangles[i]];
    }

    for(unsigned int i = 0 ; i < stack_linesegments.size() ; i++) {
      mid = mid + point_reps_linesegments[stack_linesegments[i]];
    }

    for(unsigned int i = 0 ; i < stack_points.size() ; i++) {
      mid = mid + point_reps_points[stack_points[i]];
    }

    mid = (1.0f/(stack_triangles.size() + stack_linesegments.size() +
      stack_points.size()) ) * mid;	//divide by N
    
    //	build subsets based on axis/middle point
		std::vector<int> left_triangles;
		std::vector<int> right_triangles;
    
    left_triangles.reserve( stack_triangles.size() / 2 );
    right_triangles.reserve( stack_triangles.size() / 2 );
    
		for( unsigned int i = 0 ; i<stack_triangles.size() ; i++ ) {
      Vec3 mid_to_point = 
        point_reps_triangles[ stack_triangles[i] ] - mid;
      if ( mid_to_point * axis < 0 )
				left_triangles.push_back(stack_triangles[i]);
			else
				right_triangles.push_back(stack_triangles[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left_triangles.size() == 0) || (right_triangles.size() == 0) ) {
			left_triangles.clear();
			right_triangles.clear();
			for (unsigned int i = 0 ; i<stack_triangles.size()/2 ; i++) 
        left_triangles.push_back(stack_triangles[i]);
			for (unsigned int i = stack_triangles.size()/2;
        i<stack_triangles.size() ; i++) 
        right_triangles.push_back(stack_triangles[i]);
		}

    //	build subsets based on axis/middle point
		std::vector<int> left_linesegments;
		std::vector<int> right_linesegments;
    
    left_linesegments.reserve( stack_linesegments.size() / 2 );
    right_linesegments.reserve( stack_linesegments.size() / 2 );
    
		for( unsigned int i = 0 ; i<stack_linesegments.size() ; i++ ) {
      Vec3 mid_to_point = 
        point_reps_linesegments[ stack_linesegments[i] ] - mid;
      if ( mid_to_point * axis < 0 )
				left_linesegments.push_back(stack_linesegments[i]);
			else
				right_linesegments.push_back(stack_linesegments[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left_linesegments.size() == 0) || (right_linesegments.size() == 0) ) {
			left_linesegments.clear();
			right_linesegments.clear();
			for (unsigned int i = 0 ; i<stack_linesegments.size()/2 ; i++) 
        left_linesegments.push_back(stack_linesegments[i]);
			for (unsigned int i = stack_linesegments.size()/2 ;
        i<stack_linesegments.size() ; i++) 
        right_linesegments.push_back(stack_linesegments[i]);
		}

    //	build subsets based on axis/middle point
		std::vector<int> left_points;
		std::vector<int> right_points;
    
    left_points.reserve( stack_points.size() / 2 );
    right_points.reserve( stack_points.size() / 2 );
    
		for( unsigned int i = 0 ; i<stack_points.size() ; i++ ) {
      Vec3 mid_to_point = 
        point_reps_points[ stack_points[i] ] - mid;
      if ( mid_to_point * axis < 0 )
				left_points.push_back(stack_points[i]);
			else
				right_points.push_back(stack_points[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left_points.size() == 0) || (right_points.size() == 0) ) {
			left_points.clear();
			right_points.clear();
			for (unsigned int i = 0 ; i<stack_points.size()/2 ; i++) 
        left_points.push_back(stack_points[i]);
			for (unsigned int i = stack_points.size()/2;
        i<stack_points.size() ; i++) 
        right_points.push_back(stack_points[i]);
		}
		
		stack.pop();

    //	do recurse
		if ( left_triangles.size() != 0 ||
         left_linesegments.size() != 0 ||
         left_points.size() != 0 ) {
      stack_tree->left.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->left.get();
			element.triangles.swap( left_triangles );
      element.linesegments.swap( left_linesegments );
      element.points.swap( left_points );
			stack.push( element );
		}
		
		if ( right_triangles.size() != 0 ||
         right_linesegments.size() != 0 ||
         right_points.size() != 0 ) {
			stack_tree->right.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->right.get();
			element.triangles.swap( right_triangles );
      element.linesegments.swap( right_linesegments );
      element.points.swap( right_points );
			stack.push( element );
		}
	}
}

void BinaryBoundTree::render() {
  render( 0 );
}

void Triangle::render() {
//  if( !collided ) return;
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_LINE_STRIP );
  glVertex3d( a.x, a.y, a.z );
  glVertex3d( b.x, b.y, b.z );
  glVertex3d( c.x, c.y, c.z );
  glVertex3d( a.x, a.y, a.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

void BinaryBoundTree::render( int depth) {
  if( !isLeaf() ) {
    if( depth == 0 ) {
      bound->render();
    } else {
      if( left.get() ) left->render( depth - 1 );
      if( right.get() ) right->render( depth - 1 );
    }
  } else {
    for( unsigned int i = 0; i < triangles.size(); i++ ) { 
      triangles[i].render();
    }
    for( unsigned int i = 0; i < linesegments.size(); i++ ) { 
      linesegments[i].render();
    }
    for( unsigned int i = 0; i < points.size(); i++ ) { 
      points[i].render();
    }
  }
}

 void Triangle::closestPoint( const Vec3 &p,
                              Vec3 &closest_point,
                              Vec3 &normal,
                              Vec3 &tex_coord ) {

  Vec3 ab = b-a;
  Vec3 ac = c-a;
  Vec3 ap = p-a ;

  normal = ab % ac;
  normal.normalizeSafe();

  HAPIFloat d1 = ab * ap;
  HAPIFloat d2 = ac * ap;
  if( d1 <= Constants::epsilon && d2 <= Constants::epsilon ) {
    //cerr << "1" << endl;
    tex_coord = ta;
    closest_point = a;
    return;
  }

  Vec3 bp = p - b;
  HAPIFloat d3 = ab * bp;
  HAPIFloat d4 = ac * bp;

  if( d3 >= -Constants::epsilon && d4 <= d3 ) {
    //    cerr << "2" << endl;
    tex_coord = tb;
    closest_point = b;
    return;
  }
  
  HAPIFloat vc = d1*d4 - d3*d2;
  if( vc <= Constants::epsilon && 
      d1 >= -Constants::epsilon && 
      d3 <= Constants::epsilon ) {
    // cerr << "3" << endl;
    HAPIFloat v = d1 / ( d1 - d3 );
    closest_point = a + v * ab;
    tex_coord = ta + v * (tb - ta);
    return;
  }

  Vec3 cp = p - c;
  HAPIFloat d5 = ab * cp;
  HAPIFloat d6 = ac * cp;
  if( d6 >= -Constants::epsilon && d5 <= d6 ) {
    //cerr << "4" << endl;
    tex_coord = tc;
    closest_point = c;
    return;
  }

  HAPIFloat vb = d5*d2 - d1*d6;
  if( vb <= Constants::epsilon && 
      d2 >= -Constants::epsilon &&
      d6 <= Constants::epsilon ) {
    //cerr << "5" << endl;
    HAPIFloat w = d2 / ( d2 - d6 );
    closest_point = a + w * ac;
    tex_coord = ta + w * (tc - ta );
    return;
  }
  
  HAPIFloat va = d3*d6 - d5*d4;
  if( va <= Constants::epsilon &&
      (d4-d3) >= -Constants::epsilon &&
      (d5-d6) >= -Constants::epsilon ) {
    //cerr << "6" << endl;
    HAPIFloat w = (d4-d3) /((d4 -d3) + (d5 - d6 ) );
    closest_point = b + w * (c-b);
    tex_coord = tb + w * (tc - tb );
    return;
  }

  //cerr << "7" << endl;
  HAPIFloat denom = 1 / ( va + vb + vc );
  HAPIFloat v = vb * denom;
  HAPIFloat w = vc * denom;
  closest_point = a + ab * v + ac * w;
  tex_coord = ta + (tb - ta) * v + (tc - ta ) * w;
  /*
  Vec3 normal = ca % cb;
  if( H3DUtil::H3DAbs( normal * normal ) > Constants::epsilon ) {
    normal.normalizeSafe();
    
    Vec3 pp = p - (-ap).dotProduct( -normal ) * normal;
    
    // determine dominant axis
    int axis;
    if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.y )) {
      if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.z )) axis = 0; 
      else axis = 2;
    } else {
      if (H3DUtil::H3DAbs( normal.y ) > H3DUtil::H3DAbs( normal.z )) axis = 1; 
      else axis = 2;
    }
    
    int u_axis = (axis + 1 ) % 3;
    int v_axis = (axis + 2 ) % 3;
    
    Vec3 cp = pp - c;
    
    HAPIFloat denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);
    
    HAPIFloat du = 
      (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;
    
    if( du >= 0 && du <= 1 ) {
      HAPIFloat dv = 
        (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;
      if( dv >= 0 && dv <= 1 ) {
        HAPIFloat dw = 1 - du - dv;
        if( dw >= 0 && dw <= 1 ) {
          return pp;
        }
      }
    }
  }

  // point is on edge.

  Vec3 bp = p-b ;
  Vec3 cp = p-c ;

  HAPIFloat ca_length2 = ca.lengthSqr();
  HAPIFloat cb_length2 = cb.lengthSqr();
  HAPIFloat ab_length2 = ab.lengthSqr();

  HAPIFloat ca_t = (cp * ca) / ca_length2;
  HAPIFloat cb_t = (cp * cb) / cb_length2;
  HAPIFloat ab_t = (ap * ab) / ab_length2;

  if( ca_t < 0 && cb_t < 0 )
    return c;

  if( cb_t > 1 && ab_t > 1 )
    return b;

  if( ca_t > 1 && ab_t < 0 )
    return a;
  
  bool discard_ca = ca_t < 0 || ca_t > 1;
  bool discard_cb = cb_t < 0 || cb_t > 1;;
  bool discard_ab = ab_t < 0 || ab_t > 1;;

  Vec3 ca_p = c + ca * ca_t;
  Vec3 cb_p = c + cb * cb_t;
  Vec3 ab_p = a + ab * ab_t;
  
  HAPIFloat dist_cap = (ca_p - p).lengthSqr();  
  HAPIFloat dist_cbp = (cb_p - p).lengthSqr();  
  HAPIFloat dist_abp = (ab_p - p).lengthSqr();  

  Vec3 closest;
  HAPIFloat dist;

  if( !discard_ca ) {
    closest = ca_p;
    dist = dist_cap;
  } else if( !discard_cb ) {
    closest = cb_p;
    dist = dist_cbp;
  } else {
    closest = ab_p;
    dist = dist_abp;
  }

  if( !discard_cb && dist_cbp < dist ) {
    closest = cb_p;
    dist = dist_cbp;
  } 

  if( !discard_ab && dist_abp < dist ) {
    closest = ab_p;
  }

  return closest;*/
}

#if 0
bool Triangle::lineIntersect( const Vec3 &from, 
                              const Vec3 &to,
                              IntersectionInfo &result ) {
  Vec3 ca = a-c;
  Vec3 cb = b-c;
  Vec3 dir = to - from;
  Vec3 normal = ca % cb;
  /*if( H3DUtil::H3DAbs( normal * normal ) < Constants::epsilon ||
      H3DUtil::H3DAbs( dir * dir ) < Constants::epsilon ||
      H3DUtil::H3DAbs( normal * dir ) < Constants::epsilon ) {
    return false;
  }*/
  normal.normalizeSafe();

  HAPIFloat dir_length = dir.length();
  dir.normalizeSafe();

  HAPIFloat t = ( normal * ( c - from ) ) / ( normal * dir );
  // check that the point is within the line segement
  if( t < 0 || t > dir_length ) return false;

  Vec3 p = from + t * dir;

  // determine dominant axis
  int axis;
  if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.y )) {
    if (H3DUtil::H3DAbs( normal.x ) > H3DUtil::H3DAbs( normal.z )) axis = 0; 
    else axis = 2;
  } else {
    if (H3DUtil::H3DAbs( normal.y ) > H3DUtil::H3DAbs( normal.z )) axis = 1; 
    else axis = 2;
  }

  int u_axis = (axis + 1 ) % 3;
  int v_axis = (axis + 2 ) % 3;

  Vec3 cp = p - c;

  HAPIFloat denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);

  HAPIFloat du = 
    (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;

  if( du < 0 || du > 1 ) return false;

  HAPIFloat dv = 
    (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;

  if( dv < 0 || dv > 1 ) return false;

  HAPIFloat dw = 1 - du - dv;

  if( dw < 0 || dw > 1 ) {
    return false;
  }

  collided = true;
  result.point = p;

  if( dir * normal > 0 ) 
    result.normal = -normal;
  else 
    result.normal = normal;
  
  return true;
  
}

#if 0
                              // line intersect, not segment 
bool Triangle::lineIntersect( const Vec3 &p, 
                              const Vec3 &q,
                              IntersectionInfo &result ) {
  Vec3 pq = q - p;
  Vec3 pa = a - p;
  Vec3 pb = b - p;
  Vec3 pc = c - p;

  Vec3 m = pq.crossProduct( pc );
  HAPIFloat u = pb * m;
  if( u < 0 ) return false;
  HAPIFloat v = -(pa * m );
  if( v < 0 ) return false;
  HAPIFloat w = (pq % pb) * pa;
  if( w < 0 ) return false;

  // compute barycentric coordinates
  HAPIFloat denom = 1.0 / ( u + v + w );
  u *= denom;
  v *= denom;
  w *= denom;

  result.point = u*a + v*b + w*c;
  result.normal = (a - c) % (b - c);
  result.normal.normalizeSafe();
  if( pq * result.normal > 0 ) 
    result.normal = -result.normal;
  return true;
}

#endif

#else
bool Triangle::lineIntersect( const Vec3 &p, 
                              const Vec3 &q,
                              IntersectionInfo &result,
                              FaceType face  ) {
  
  Vec3 v0 = a;
  Vec3 v1 = b;
  Vec3 v2 = c;

  Vec3 tc0 = ta;
  Vec3 tc1 = tb;
  Vec3 tc2 = tc;
 
  Vec3 ab = v1 - v0;
  Vec3 ac = v2 - v0;
  Vec3 qp = p - q;

  // Compute normal.
  Vec3 n = ab % ac;

  Vec3 original_normal = n;

  HAPIFloat epsilon = 1e-5;

  HAPIFloat d = qp * n;

  FaceType intersection_face = Bounds::FRONT;
  if ( d < 0 ) {
    // line segment pointing away from triangle
    if( face == Bounds::FRONT ) return false;

    // flip normal in calculations since they assume starting point is
    // in the direction of the normal
    d = -d;
    n = -n;
    Vec3 tmp = v1;
    v1 = v2;
    v2 = tmp;
    
    tmp = tc1;
    tc1 = tc2;
    tc2 = tmp;

    tmp = ab;
    ab = ac;
    ac = tmp;
    intersection_face = Bounds::BACK;
  } else if( d == 0 || face == Bounds::BACK ) {
    return false;
  }

  Vec3 ap = p - v0;
  HAPIFloat t = ap * n;

  if( t < -epsilon || t > d + epsilon ) return false;

  Vec3 e = qp % ap;
  HAPIFloat v = ac * e;
  if( v < -epsilon || v > d + epsilon ) return false;
  HAPIFloat w = -(ab * e );
  if( w < -epsilon || v + w > d + epsilon) return false;

  HAPIFloat ood = 1 / d;
  t *= ood; 
  v *= ood;
  w *= ood;
  HAPIFloat u = 1 - v - w;

  result.point = u*v0 + v*v1 + w*v2;
  result.normal = original_normal;
  result.normal.normalizeSafe();
  result.tex_coord =  u*tc0 + v*tc1 + w*tc2;
  result.face = intersection_face;
  result.intersection = true;
  //if( (-qp) * result.normal > 0 ) 
  //  result.normal = -result.normal;
  return true;
}

#endif

bool Triangle::movingSphereIntersect( HAPIFloat radius,
                                      const Vec3 &from, 
                                      const Vec3 &to ) {

// TODO: Look in book if better way
  Vec3 closest, tmp;
  closestPoint( from, closest, tmp, tmp );
  Vec3 ttt = from - closest;
  HAPIFloat aba = ttt * ttt;
  if( aba <= radius * radius ) return true; 
                     
  /*  if( from.z * to.z < 0 ) {
cerr << "FD";
                                        }
  */               
  Vec3 ca = a-c;
  Vec3 cb = b-c;
  Vec3 normal = ca % cb;

  Vec3 t = from - a;
  if( t * normal < 0 )
    normal = -normal;

  normal.normalizeSafe();

  Plane plane( a, normal );
  IntersectionInfo info;
  Vec3 D = from - radius * normal;
  Vec3 v = to - from;



  // if res is false the plane is parallell to the line and
  // then we could have a hit, this can give problems.
  bool res = plane.lineIntersect( D, D + v, info );
  Vec3 P = info.point;
  Sphere sphere( from, radius );

  // TODO: check if P within triangle, then collision

  if( res ) {
    bool inside = true;
    Vec3 a0 = a - P;
    Vec3 b0 = b - P;
    Vec3 c0 = c - P;
    HAPIFloat ab = a * b;
    HAPIFloat ac = a * c;
    HAPIFloat bc = b * c;
    // Make sure plane normals for pab and pbc point in the same direction
    if( bc * ac - (c * c) * ab < 0.0f ) inside = false;
    else if( ab * bc - ac * (b * b) ) inside = false;

    if( inside )
      return true;
  }

  Vec3 Q;
  closestPoint( P, Q, tmp, tmp );

  return sphere.lineIntersect( Q, Q - v, info );
}

// at the moment this function does not return a IntersectionInfo
bool Triangle::movingSphereIntersectRobust( HAPIFloat radius,
                                          const Vec3 &from, 
                                          const Vec3 &to ) {
  // TODO: Look in book if better way
  Vec3 closest, tmp;
  closestPoint( from, closest, tmp, tmp );
  Vec3 ttt = from - closest;
  HAPIFloat aba = ttt * ttt;
  if( aba <= radius * radius ) return true; 
                     
  /*  if( from.z * to.z < 0 ) {
cerr << "FD";
                                        }
  */               
  Vec3 ca = a-c;
  Vec3 cb = b-c;
  Vec3 normal = ca % cb;

  Vec3 t = from - a;
  if( t * normal < 0 )
    normal = -normal;

  normal.normalizeSafe();

  Plane plane( a, normal );
  IntersectionInfo info;
  Vec3 D = from - radius * normal;
  Vec3 v = to - from;


  // if res is false the plane is parallell to the line and
  // then we could have a hit, this can give problems.
  bool res = plane.lineIntersect( D, D + v, info );
  Vec3 P = info.point;

  // p 204 to test triangle
  if( res ) {
    bool inside = true;
    Vec3 a0 = a - P;
    Vec3 b0 = b - P;
    Vec3 c0 = c - P;
    HAPIFloat ab = a * b;
    HAPIFloat ac = a * c;
    HAPIFloat bc = b * c;
    // Make sure plane normals for pab and pbc point in the same direction
    if( bc * ac - (c * c) * ab < 0.0f ) inside = false;
    else if( ab * bc - ac * (b * b) ) inside = false;
    
    if( inside )
      return true;
  }

  HAPIFloat cyl_t;
  bool intersect = Bounds::intersectSegmentCylinder(
    from, to, a, b, radius, cyl_t );
  HAPIFloat intersection_point_t;
  if( intersect ) {
    intersection_point_t = cyl_t;
    return true;
  }

  if( Bounds::intersectSegmentCylinder( from, to, a, c, radius, cyl_t ) ) {
    if( intersect ) {
      if( cyl_t < intersection_point_t )
        intersection_point_t = cyl_t;
    }
    else {
      return true;
      intersect = true;
      intersection_point_t = cyl_t;
    }
  }

  if( Bounds::intersectSegmentCylinder( from, to, b, c, radius, cyl_t ) ) {
    if( intersect ) {
      if( cyl_t < intersection_point_t )
        intersection_point_t = cyl_t;
    }
    else {
      return true;
      intersect = true;
      intersection_point_t = cyl_t;
    }
  }

  if( intersect )
    return true;

  Sphere sphere( a, radius );
  intersect = sphere.lineIntersect( from, to, info );
  HAPIFloat square_dist;
  if( intersect ) {
    return true;
    P = info.point;
    square_dist = ( P - from ).lengthSqr();
  }

  sphere.center = b;
  if( sphere.lineIntersect( from, to, info ) ) {
    if( intersect ) {
      HAPIFloat temp_dist = ( info.point - from ).lengthSqr();
      if( temp_dist < square_dist ) {
        square_dist = temp_dist;
        P = info.point;
      }
    }
    else {
      return true;
      intersect = true;
      P = info.point;
    }
  }

  sphere.center = c;
  if( sphere.lineIntersect( from, to, info ) ) {
    if( intersect ) {
      HAPIFloat temp_dist = ( info.point - from ).lengthSqr();
      if( temp_dist < square_dist ) {
        square_dist = temp_dist;
        P = info.point;
      }
    }
    else {
      return true;
      intersect = true;
      P = info.point;
    }
  }

  return intersect;
}


void BinaryBoundTree::clearCollidedFlag() {
  if( !isLeaf() ) {
    bound->collided = false;
    left->clearCollidedFlag();
    right->clearCollidedFlag();
  } else {
    for( unsigned int i = 0; i < triangles.size(); i++ )
      triangles[i].collided = false;
    for( unsigned int i = 0; i < linesegments.size(); i++ )
      linesegments[i].collided = false;
    for( unsigned int i = 0; i < points.size(); i++ )
      points[i].collided = false;
  }
}



bool BinaryBoundTree::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     IntersectionInfo &result,
                                     FaceType face ) {
  if ( isLeaf() )	{
    // TODO: find closest?
    IntersectionInfo tempResult;
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      if( t.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
		}

    for( unsigned int i = 0; i < linesegments.size(); i++ ) {
      LineSegment &ls = linesegments[i];
      if( ls.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
		}

    for( unsigned int i = 0; i < points.size(); i++ ) {
      Point &p = points[i];
      if( p.lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
      }
		}
    return result.intersection;
		//return false;
	}	else 	{
		if ( bound->boundIntersect( from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) overlap |= left->lineIntersect( from, to, result, face );
			if (right.get()) overlap |= right->lineIntersect( from, to, result, face );
			
			return overlap;
		}
		else return false;
	}
}

bool BinaryBoundTree::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
  if ( isLeaf() )	{
    // TODO: find closest?
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      if( t.movingSphereIntersectRobust( radius, from, to ) )	return true;
		}

    for( unsigned int i = 0; i < linesegments.size(); i++ ) {
      LineSegment &ls = linesegments[i];
      if( ls.movingSphereIntersect( radius, from, to ) )	return true;
		}

    for( unsigned int i = 0; i < points.size(); i++ ) {
      Point &pt = points[i];
      if( pt.movingSphereIntersect( radius, from, to ) )	return true;
		}
		return false;
	}	else 	{
		if ( bound->boundMovingSphereIntersect( radius, from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) {
        overlap = left->movingSphereIntersect( radius, from, to );
        if( overlap ) return overlap;
      }
			if (right.get()) 
        overlap = right->movingSphereIntersect( radius,from, to );
			
			return overlap;
		}
		else return false;
	}
}


Matrix3 covarianceMatrix( const vector< Vec3 >&points ) {
  // TODO: if no points?

  unsigned int nr_points = points.size();

	Vec3 c = points[0];
	for(unsigned int i=1 ; i < nr_points; i++) 
    c+= points[i];

	c /= points.size();
	
  HAPIFloat e00 = 0, e01 = 0, e02 = 0,
                    e11 = 0, e12 = 0,
                             e22 = 0;
	for (unsigned int i = 0; i < nr_points; i++)	{
		Vec3 p = points[i] - c;
		
		e00 += p.x * p.x;
		e11 += p.y * p.y;
		e22 += p.z * p.z;
		e01 += p.x * p.y;
		e02 += p.x * p.z;
		e12 += p.y * p.z;
	}
	
	Matrix3 cov;
	cov[0][0] = e00 / nr_points;
	cov[1][1] = e11 / nr_points;
	cov[2][2] = e22 / nr_points;
	cov[0][1] = cov[1][0] = e01 / nr_points;
	cov[0][2] = cov[2][0] = e02 / nr_points;
	cov[1][2] = cov[2][1] = e12 / nr_points;
	
	return cov;
}

void symmetricSchurDecomposition( const Matrix3 &A, 
                                  unsigned int p, 
                                  unsigned int q, 
                                  HAPIFloat &c, 
                                  HAPIFloat &s) {
  // TODO: epsilon??
	if (H3DUtil::H3DAbs(A[p][q]) > 0.0001f) {
		HAPIFloat r = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
		HAPIFloat t = (r >= 0.0f ? 
                  1.0f / (r + H3DUtil::H3DSqrt(1.0f + r*r)) : 
                  -1.0f / (-r + H3DUtil::H3DSqrt(1.0f + r*r)) );
		
		c = 1.0f / H3DUtil::H3DSqrt(1.0f + t*t);
		s = t * c;
	}	else{
		c = 1.0f;
		s = 0.0f;
	}
}

void eigenValuesAndEigenVectorsJacobi(const Matrix3& A, 
                                      Vec3& eigValues, 
                                      Matrix3& eigVectors, 
                                      const unsigned int maxIterations = 50 ) {
  //!NOTE: on return rows (not cols) of eigVectors contains eigenvalues

	Matrix3 a = A;
	Matrix3 v;
	
	unsigned int i, j, n, p, q;
	HAPIFloat prevoff, c, s;
	Matrix3 J, b, t;

  // Repeat for some maximum number of iterations
	for (n = 0; n < maxIterations; n++)	{
		p = 0; q = 1;
		for ( i = 0; i < 3; i++) for (j = 0; j < 3; j++ ) {
			if (i == j) continue;
			if (H3DUtil::H3DAbs(a[i][j]) > H3DUtil::H3DAbs(a[p][q])) {p = i; q = j;}
		}

		symmetricSchurDecomposition(a, p, q, c, s);
		for (i = 0; i < 3; i++) {
			J[i][0] = J[i][1] = J[i][2] = 0.0f;
			J[i][i] = 1.0f;
		}
		J[p][p] =  c; J[p][q] = s;
		J[q][p] = -s; J[q][q] = c;

    // cumulate rotations
		v = v * J;

    // diagonalize
		a = (J.transpose() * a) * J;
    
    //	eval norm
		HAPIFloat off = 0.0;
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)	{
			if (i == j) continue;
			off += a[i][j] * a[i][j];
		}
		
    // stop condition
		if (n > 2 && off >= prevoff) break;
		prevoff = off;
	}
	
	eigValues = Vec3(a[0][0],a[1][1],a[2][2]);
	eigVectors = v.transpose();
}

void OrientedBoxBound::fitAroundPoints( const vector< Vec3 > &points ) {
  Matrix3 covariance = covarianceMatrix( points );
	
	Vec3 eig_values;
	Matrix3 eig_vectors;

	eigenValuesAndEigenVectorsJacobi(covariance, eig_values, eig_vectors);
	
	
  //	guess an oriented reference frame
	unsigned int length_order[3];
  //	initialize
	for(unsigned int i=0 ; i<3 ; i++) length_order[i] = i;
	
  //	do a bubble sort on indices
	for( unsigned int i=0 ; i<2 ; i++) {
    for(unsigned int j=i+1 ; j<3 ; j++)	{
      if ( eig_values[length_order[i]] < eig_values[length_order[j]] ) {
        // swap
        unsigned int temp( length_order[i] );
        length_order[i] = length_order[j];
        length_order[j] = temp;
      }
    }
  }

	Vec3 principal( eig_vectors[length_order[0]][0],
                   eig_vectors[length_order[0]][1],
                   eig_vectors[length_order[0]][2] );

	Vec3 secondary( eig_vectors[length_order[1]][0],
                   eig_vectors[length_order[1]][1],
                   eig_vectors[length_order[1]][2] );

  principal.normalize();
  secondary.normalize();
  Vec3 v = principal % secondary;

	Matrix3 frame( principal.x, principal.y, principal.z,
                  secondary.x, secondary.y, secondary.z, 
                  v.x, v.y, v.z );
		
  //	build box in the new frame
  min = frame * points[0];
	max = min;
	for(unsigned int i=1 ; i<points.size() ; i++)	{
		Vec3 pt = frame * points[i];
		
		if (pt.x < min.x) min.x = pt.x;
		if (pt.y < min.y) min.y = pt.y;
		if (pt.z < min.z) min.z = pt.z;
		
		if (pt.x > max.x) max.x = pt.x;
		if (pt.y > max.y) max.y = pt.y;
		if (pt.z > max.z) max.z = pt.z;
	}
	
  //	fill oriented box
  orientation = Rotation( frame );
  
  //cerr << frame * (0.5f*(min+max)) << endl;
	center = frame.transpose() * (0.5f*(min+max)) ;
  halfsize = 0.5f*(max-min);
}

bool OrientedBoxBound::boundIntersect( const Vec3 &from, 
                                       const Vec3 &to ) {
  Vec3 rp = orientation * from;
	Vec3 rq = orientation * to;
	
	return AABoxBound::boundIntersect( rp, rq );
}

void OrientedBoxBound::closestPoint( const Vec3 &p,
                                     Vec3 &closest_point,
                                     Vec3 &normal,
                                     Vec3 &tex_coord ) {
  // TODO: implement
  assert( false );
}

bool OrientedBoxBound::insideBound( const Vec3 &p ) {
  Vec3 rp = orientation * p; 
  return ( rp.x <= max.x &&
           rp.x >= min.x &&
           rp.y <= max.y &&
           rp.y >= min.y &&
           rp.z <= max.z &&
           rp.z >= min.z );
}

void OrientedBoxBound::render( ) {
  Vec3 center = -orientation * ((max + min)/2);
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else
    glColor3d( 1, 1, 0 );
  glTranslated( center.x, center.y, center.z );
  glRotated( -orientation.angle *180 / H3DUtil::Constants::pi, 
             orientation.axis.x, orientation.axis.y, orientation.axis.z );
  Vec3 minf( - halfsize );
  Vec3 maxf( halfsize );
  glBegin( GL_LINE_STRIP );
  glVertex3d( minf.x, minf.y, minf.z );
  glVertex3d( minf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, minf.y, minf.z );
  glVertex3d( minf.x, minf.y, minf.z );
  glEnd();
  
  glBegin( GL_LINE_STRIP );
  glVertex3d( minf.x, minf.y, maxf.z );
  glVertex3d( minf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, minf.y, maxf.z );
  glVertex3d( minf.x, minf.y, maxf.z );
  glEnd();
  
  glBegin( GL_LINES );
  glVertex3d( minf.x, minf.y, maxf.z );
  glVertex3d( minf.x, minf.y, minf.z );
  glVertex3d( maxf.x, minf.y, maxf.z );
  glVertex3d( maxf.x, minf.y, minf.z );
  glVertex3d( minf.x, maxf.y, maxf.z );
  glVertex3d( minf.x, maxf.y, minf.z );
  glVertex3d( maxf.x, maxf.y, maxf.z );
  glVertex3d( maxf.x, maxf.y, minf.z );
  glEnd();
  glEnable( GL_LIGHTING );
  glPopMatrix();
}

void GeometryPrimitive::getConstraints( const Vec3 &point,
                           std::vector< PlaneConstraint > &constraints,
                                  FaceType face ) {
  Vec3 closest_point, cp_normal, cp_tex_coord;
  closestPoint( point, closest_point, cp_normal, cp_tex_coord );
  //cerr << closest_point << endl;
  Vec3 normal = point - closest_point;

  if( face == Bounds::FRONT ) {
    if( normal * cp_normal < 0 ) return;
  } else if( face == Bounds::BACK ) {
    if( normal * cp_normal > 0 ) return;
  }
  normal.normalizeSafe();
  //cerr << closest_point << endl;
  constraints.push_back( PlaneConstraint( closest_point, normal, 
                                          cp_tex_coord, NULL, this ) );
}

void Triangle::getConstraints( const Vec3 &point,
                               const Matrix4 &matrix,
                               std::vector< PlaneConstraint > &constraints,
                               FaceType face ) {
  Vec3 oa = a;
  Vec3 ob = b;
  Vec3 oc = c;
  a = (matrix * a);
  b = (matrix * b);
  c = (matrix * c);
  Triangle::getConstraints( point, constraints, face );
  a = oa;
  b = ob;
  c = oc;
}


void BinaryBoundTree::getConstraints( const Vec3 &point,
                                      std::vector< PlaneConstraint > &constraints,
                                      FaceType face ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, constraints, face );
		}
    for( unsigned int i = 0; i < linesegments.size(); i++ ) {
      LineSegment &ls = linesegments[i];
      ls.getConstraints( point, constraints, face );
		}
    for( unsigned int i = 0; i < points.size(); i++ ) {
      Point &pt = points[i];
      pt.getConstraints( point, constraints, face );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, constraints, face );
    if (right.get()) right->getConstraints( point, constraints, face );
    
  }
}

void BinaryBoundTree::getConstraints( const Vec3 &point,
                                      const Matrix4 &matrix,
                                      std::vector< PlaneConstraint > &constraints,
                                      FaceType face ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, matrix, constraints, face );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, matrix, constraints, face );
    if (right.get()) right->getConstraints( point, matrix, constraints, face );
  }
}


void BinaryBoundTree::getTrianglesWithinRadius( const Vec3 &p,
                                                HAPIFloat radius,
                                                vector< Triangle > &result ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
	}	else 	{
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getTrianglesWithinRadius( p, radius, result );
    if (right.get()) right->getTrianglesWithinRadius( p, radius, result );
  }
}

void BinaryBoundTree::getPrimitivesWithinRadius( const Vec3 &p,
                                                HAPIFloat radius,
                                                vector< Triangle > &tris,
                                                vector< LineSegment > &lines,
                                                vector< Point > &pts ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        tris.push_back( *i );
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); i++ ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        lines.push_back( *i );
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); i++ ) {
      Vec3 cp, tmp;
      (*i).closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        pts.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
	}	else 	{
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getPrimitivesWithinRadius( p, radius, tris, lines, pts );
    if (right.get()) right->getPrimitivesWithinRadius( p, radius, tris, lines, pts );
  }
}

void LineSegment::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_LINES );
  glVertex3d( start.x, start.y, start.z );
  glVertex3d( end.x, end.y, end.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

/// Returns the closest point on the object to the given point p.
void LineSegment::closestPoint( const Vec3 &p,
                                Vec3 &closest_point,
                                Vec3 &normal,
                                Vec3 &tex_coord ) {
  Vec3 ab = end - start;
  HAPIFloat ab2 = ab * ab;
  if( ab2 < Constants::epsilon ) {
    closest_point = start;
  } else {
    HAPIFloat t = 
      (p - start).dotProduct( ab ) / ab2;
    if( t < 0 ) t = 0;
    if( t > 1 ) t = 1;
    closest_point = start + t * ab;
  }
  normal = closest_point - p;
  normal.normalizeSafe();
  // TODO:
  tex_coord = Vec3();
}

/// Detect collision between a line segment and the object.
bool LineSegment::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                 FaceType face ) {
  // TODO: implement
  return false;
}

// Detect collision between a moving sphere and the object.
bool LineSegment::movingSphereIntersect( HAPIFloat radius,
                                         const Vec3 &from, 
                                         const Vec3 &to ) {
  Vec3 tmp;
  HAPIFloat t;
  HAPIFloat r2 = radius * radius;
  HAPIFloat d2 = 
    LineSegment( from, to ).closestPointOnLine( start, end, 
                                                t, t, 
                                                tmp, tmp );
  return d2 <= r2;
}

HAPIFloat LineSegment::closestPointOnLine( const Vec3 &from, const Vec3 &to,
                                           HAPIFloat &s, HAPIFloat &t,
                                           Vec3 &c1, Vec3 &c2 ) {
  Vec3 p1 = from;
  Vec3 q1 = to;
  Vec3 p2 = start;
  Vec3 q2 = end;

  Vec3 d1 = q1 - p1; // Direction vector of segment S1
  Vec3 d2 = q2 - p2; // Direction vector of segment S2
  Vec3 r = p1 - p2;

  // Squared length of segment S1, always nonnegative
  HAPIFloat a = d1 * d1; 
  // Squared length of segment S2, always nonnegative
  HAPIFloat e = d2 * d2; 
  HAPIFloat f = d2 * r;

  // Check if either or both segments degenerate into points
  if (a <= Constants::epsilon && e <= Constants::epsilon) {
    // Both segments degenerate into points
    s = t = 0.0f;
    c1 = p1;
    c2 = p2;
    Vec3 v = c1 - c2;
    return v * v;
  }
  if (a <= Constants::epsilon) {
    // First segment degenerates into a point
    s = 0.0f;
    t = f / e; // s = 0 => t = (b*s + f) / e = f / e
    t = clamp(t, 0.0f, 1.0f);
  } else {
    HAPIFloat c = d1 *r;
    if (e <= Constants::epsilon) {
      // Second segment degenerates into a point
      t = 0.0f;
      s = clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
    } else {
      // The general nondegenerate case starts here
      HAPIFloat b = d1 * d2;
      HAPIFloat denom = a*e-b*b; // Always nonnegative

      // If segments not parallel, compute closest point on L1 to L2, and
      // clamp to segment S1. Else pick arbitrary s (here 0)
      if (denom != 0.0f) {
        s = clamp((b*f - c*e) / denom, 0.0f, 1.0f);
      } else s = 0.0f;

      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
      t = (b*s + f) / e;

      // If t in [0,1] done. Else clamp t, recompute s for the new value
      // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
      // and clamp s to [0, 1]
      if (t < 0.0f) {
        t = 0.0f;
        s = clamp(-c / a, 0.0f, 1.0f);
      } else if (t > 1.0f) {
        t = 1.0f;
        s = clamp((b - c) / a, 0.0f, 1.0f);
      }
    }
  }

  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  Vec3 v = c1 - c2;
  return v * v;
}
  
void LineSegment::getConstraints( const Vec3 &point,
                                  const Matrix4 &matrix,
                                  std::vector< PlaneConstraint > &constraints,
                                  FaceType face 
                                      
) {
  Vec3 oa = start;
  Vec3 ob = end;

  start = (matrix * start);
  end = (matrix * end);

  getConstraints( point, constraints, face );
  start = oa;
  end = ob;
}

void Point::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3d( 1, 0, 0 );
  else {
    glColor3d( 0, 0, 1 );
  }

  glBegin( GL_POINTS );
  glVertex3d( position.x, position.y, position.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

/// Detect collision between a line segment and the object.
bool Point::lineIntersect( const Vec3 &from, 
                                 const Vec3 &to,
                                 IntersectionInfo &result,
                                  FaceType face ) {
  Vec3 ab = to - from;
  // Project c onto ab, but deferring divide by Dot( ab, ab )
  HAPIFloat t = ( position - from ) * ab;
  if( t <= 0.0f ) {
    // c project outside the [from, to] interval, on from side.
    return false;
  }
  else {
    HAPIFloat denom = ab * ab;
    if( t >= denom ) {
      // c projects outside the [from, to] interval, on to side.
      return false;
    }
    else {
      // c projects inside the [from, to] interval
      t = t / denom;
      Vec3 d = from + t * ab;
      if( (position - d).lengthSqr() ) {
        result.point = position;
        return true;
      }
    }
  }
  return false;
}
      
// Detect collision between a moving sphere and the object.
bool Point::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to ) {
  HAPIFloat r2 = radius * radius;
  Vec3 closest_point, tmp; 
  LineSegment( from, to ).closestPoint( position, closest_point, 
                                        tmp, tmp );
  Vec3 v = position - closest_point;
  return v * v <= r2;
}

void Point::getConstraints( const Vec3 &point,
                            const Matrix4 &matrix,
                            std::vector< PlaneConstraint > &constraints,
                            FaceType face  ) {
  Vec3 oa = position;
  position = (matrix * position);
  getConstraints( point, constraints, face );
  position = oa;
}


bool Plane::lineIntersect( const Vec3 &from, 
                           const Vec3 &to,
                           IntersectionInfo &result,
                           FaceType face ) {
  Vec3 from_to = to - from;

  HAPIFloat denom = normal * from_to;

  if( denom > Constants::epsilon ) { 
    // line pointing away from normal
    if( face == Bounds::FRONT ) return false;
    result.face = Bounds::BACK;
  } else {
    if( face == Bounds::BACK ) return false;
    result.face = Bounds::FRONT;
  }
   
  if( denom * denom < Constants::epsilon ) {
    return false;
  }
  
  HAPIFloat u = ( normal * ( point - from ) ) / denom; 
  if( u <= 1 + Constants::epsilon && u >= 0 - Constants::epsilon ) {
    
    if( u < 0 ) u = 0;
    if( u > 1 ) u = 1;
    result.point = from + u * from_to;
    result.normal = normal;
    return true;
  }
  return false;
}

bool Plane::movingSphereIntersect( HAPIFloat radius,
                                   const Vec3 &from, 
                                   const Vec3 &to ) {
  HAPIFloat d = normal * point;

  // Code from p222 in Realtime Collision detection book

  // Get the distance for both a and b from plane p
  HAPIFloat adist = from * normal - d;
  HAPIFloat bdist = to * normal - d;
  // Intersects if on different sides of plane (distances have different signs)
  if ( adist * bdist < 0.0f ) return true;
  // Intersects if start or end position within radius from plane
  if ( H3DUtil::H3DAbs(adist) <= radius || 
       H3DUtil::H3DAbs( bdist ) <= radius ) return true;
  // No intersection
  return false;
}


bool Sphere::lineIntersect( const Vec3 &from, 
                            const Vec3 &to,
                            IntersectionInfo &result,
                            FaceType face ) {
  Vec3 start_point = from - center;
  Vec3 end_point = to - center;
 
  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3 p, v;
  HAPIFloat r2 = radius * radius;

  HAPIFloat a0  = start_point * start_point - r2;
  if (a0 <= 0) {
    // start_point is inside sphere
    a0 = end_point * end_point - r2;
    if( a0 <= 0 || face == Bounds::FRONT ) {
      // both start_point and end_point are inside sphere so no intersection
      return false;
    } else {
      // start_point is inside and end_point is outside. We will have an 
      // intersection.
      p = end_point;
      v = start_point - end_point;
      result.face = Bounds::BACK;
    }
  } else {
    // start_point is outside sphere
    if( face == Bounds::BACK ) return false;

    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    HAPIFloat a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
    result.face = Bounds::FRONT;
  }
  // use implicit quadratic formula to find the roots
  HAPIFloat a = v.x*v.x + v.y*v.y + v.z*v.z;
  HAPIFloat b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  HAPIFloat c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  HAPIFloat s = b*b - 4*a*c;

  HAPIFloat u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    HAPIFloat u0 = (-b + sqrt(s))/(2*a);
    HAPIFloat u1 = (-b - sqrt(s))/(2*a);
    u = u0 < u1 ? u0 : u1;
  }  else {
    // line does not intersect
    return false;
  }

  // check that the intersection point is within the line segment
  if( u > 1.0 || u < 0.0) {
    return false;
  }
  
  // calculate the intersection point and normal
  result.normal =  p + u*v;
  result.point = result.normal + center;
  result.normal.normalize();
  return true;
}

bool Sphere::movingSphereIntersect( HAPIFloat r,
                                    const Vec3 &from, 
                                    const Vec3 &to ) {
  // Code from p224 in Realtime Collision detection book

  // Vector between sphere centers
  Vec3 s = from - center;    
  // Motion of sphere
  Vec3 v = to - from;         

  // Sum of sphere radii
  HAPIFloat radius_sum = radius + r;
  
  HAPIFloat c = s * s - radius_sum * radius_sum;
  if (c < 0.0f) {
    // Spheres initially overlapping so exit directly
    return true;
  }

  HAPIFloat a = v * v;
  // Spheres not moving relative each other
  if (a < Constants::epsilon) return false; 
  HAPIFloat b = v * s;
  // Spheres not moving towards each other
  if (b >= 0.0f) return false; 
  HAPIFloat d = b * b - a * c;
  // No real-valued root, spheres do not intersect
  if (d < 0.0f) return false;    

  return true;
}

void BinaryBoundTree::getTrianglesIntersectedByMovingSphere( HAPIFloat radius,
                                                             Vec3 from,
                                                             Vec3 to,
                                                             vector< Triangle > &result) {
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result.push_back( *i );
    }
	}	else 	{
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getTrianglesIntersectedByMovingSphere( radius, from, to, result );
    if (right.get()) right->getTrianglesIntersectedByMovingSphere( radius, from, to, result );
  }
}

void BinaryBoundTree::getPrimitivesIntersectedByMovingSphere(
                HAPIFloat radius,
                Vec3 from,
                Vec3 to,
                vector< Triangle > &result_triangles,
                vector< LineSegment > &result_lines,
                vector< Point > &result_points ) {
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_triangles.push_back( *i );
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); i++ ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_lines.push_back( *i );
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); i++ ) {
      if( (*i).movingSphereIntersect( radius, from, to ) )
        result_points.push_back( *i );
    }
	}	else 	{
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getPrimitivesIntersectedByMovingSphere( radius,
      from, to, result_triangles, result_lines, result_points );
    if (right.get()) right->getPrimitivesIntersectedByMovingSphere( radius,
      from, to, result_triangles, result_lines, result_points );
  }
}

void BinaryBoundTree::closestPoint( const Vec3 &p,
                                    Vec3 &closest_point,
                                    Vec3 &closest_normal,
                                    Vec3 &tex_coord ) {
  if ( isLeaf() )	{
    Vec3 cp, cn;
    HAPIFloat d2;
    if( triangles.size() == 0 ) return;
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      Vec3 point, normal;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - cp;
      if( i == triangles.begin() ) {
        cp = point;
        cn = normal;
        d2 = v * v;
      } else {
        HAPIFloat new_d2 = v * v;
        if( new_d2 < d2 ) {
          cp = point;
          cn = normal;
          d2 = v * v;
        }
      }
    }

    for( vector< LineSegment >::iterator i = linesegments.begin();
         i != linesegments.end(); i++ ) {
      Vec3 point, normal;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - cp;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        cp = point;
        cn = normal;
        d2 = v * v;
      }
    }

    for( vector< Point >::iterator i = points.begin();
         i != points.end(); i++ ) {
      Vec3 point, normal;
      (*i).closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - cp;
      HAPIFloat new_d2 = v * v;
      if( new_d2 < d2 ) {
        cp = point;
        cn = normal;
        d2 = v * v;
      }
    }
    closest_point = cp;
    closest_normal = cn;
	}	else 	{
   
    if( left.get() && right.get() ) {
      Vec3 cp, cn, tc;
      left->closestPoint( p, cp, cn, tc  );
      right->closestPoint( p, closest_point, closest_normal, tex_coord );
      Vec3 v = p - cp;
      HAPIFloat ld2 = v * v;
      v = p - closest_point;
      if( ld2 < v * v ) {
        closest_point = cp;
        closest_normal = cn;
        tex_coord = tc;
      }
      
    } else {
      if (left.get()) {
        left->closestPoint( p, closest_point, closest_normal, tex_coord  );
      } else {
        right->closestPoint( p, closest_point, closest_normal, tex_coord );
      }
    }
  }
}

void BinaryBoundTree::getAllTriangles( vector< Triangle > &tris ) {
  if ( isLeaf() )	{
    tris.insert( tris.end(), triangles.begin(), triangles.end() );
	}	else 	{
    if (left.get()) left->getAllTriangles( tris );
    if (right.get()) right->getAllTriangles( tris );
  }
}

void BinaryBoundTree::getAllPrimitives( vector< Triangle > &tris,
                                    vector< LineSegment > &lins,
                                    vector< Point > &poins ) {
  if ( isLeaf() )	{
    tris.insert( tris.end(), triangles.begin(), triangles.end() );
    lins.insert( lins.end(), linesegments.begin(), linesegments.end() );
    poins.insert( poins.end(), points.begin(), points.end() );
	}	else 	{
    if (left.get()) left->getAllTriangles( tris );
    if (right.get()) right->getAllTriangles( tris );
  }
}

Vec3 OrientedBoxBound::longestAxis() const {
  Vec3 a = AABoxBound::longestAxis();
  return orientation * a;
}


struct StackElementPrimitive {
  BBTreePrimitive *tree;
  vector< int > primitives;
}; 
BBTreePrimitive::BBTreePrimitive(
            BoundNewFunc func, 
            const vector< GeometryPrimitive * > &primitive_vector,
            unsigned int max_nr_primitives_in_leaf ):
  left( NULL ), right( NULL ), new_func( func ) {

  std::stack< StackElementPrimitive > stack;
	
  if( primitive_vector.size() == 0 ) return;
 
  //	add the starting subtree to stack...
	StackElementPrimitive start;
	start.tree = this;

  vector< Vec3 > point_reps;
  point_reps.reserve( primitive_vector.size() );

	start.primitives.reserve( primitive_vector.size() );
  for( unsigned int i = 0; i < primitive_vector.size(); i++ ) {
    start.primitives.push_back( i );
    point_reps.push_back( primitive_vector[i]->pointRepresentation() );
  }



	stack.push( start );
  //cerr << start.triangles.size() << endl;
  //	BUILD TREE
	while( !stack.empty() ) {
		const std::vector< int > &stack_primitives = stack.top().primitives;
		BBTreePrimitive* stack_tree = stack.top().tree;

	  if (max_nr_primitives_in_leaf < 0 ||
        stack_primitives.size() <= max_nr_primitives_in_leaf ) {
      //	build a leaf
			stack_tree->primitives.reserve( stack_primitives.size() );
      for( unsigned int i = 0; i < stack_primitives.size(); i++ ) {
        stack_tree->primitives.push_back(
          primitive_vector[stack_primitives[i]] );
      }
      stack.pop();
			continue;
		}

		std::vector<Vec3 > points;
		points.reserve( stack_primitives.size() * 3 );
		for (unsigned int i=0 ; i<stack_primitives.size() ; i++ ) {
      GeometryPrimitive *a_primitive = primitive_vector[ stack_primitives[i ]];
      Triangle *tri = dynamic_cast< Triangle * >(a_primitive);
      if( tri ) {
        points.push_back( tri->a );
        points.push_back( tri->b );
        points.push_back( tri->c );
      } else {
        Sphere * sph = dynamic_cast< Sphere * >(a_primitive);
        if( sph ) {
          Vec3 x_axis = Vec3( 1, 0, 0 );
          Vec3 y_axis = Vec3( 0, 1, 0 );
          Vec3 z_axis = Vec3( 0, 0, 1 );

          points.push_back( sph->center + sph->radius * x_axis +
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis -
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis +
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis -
                            sph->radius * y_axis + sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis +
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center + sph->radius * x_axis -
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis +
                            sph->radius * y_axis - sph->radius * z_axis );
          points.push_back( sph->center - sph->radius * x_axis -
                            sph->radius * y_axis - sph->radius * z_axis );
        } else {
          LineSegment * lineseg = dynamic_cast< LineSegment * >(a_primitive);
          if( lineseg ) {
            points.push_back( lineseg->start );
            points.push_back( lineseg->end );
          } else {
            Point * pt = dynamic_cast< Point * >(a_primitive);
            if( pt ) {
              points.push_back( pt->position );
            } else {
              Plane * pl = dynamic_cast< Plane * >(a_primitive);
              if( pl ) {
                // TODO
              }
            }
          }
        }
      }
    }
    
    //	build bounding shape
		stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( points );
		
    //	DIVIDE SUBSET AND RECURSE
    //	longest axis
		Vec3 axis = stack_tree->bound->longestAxis();

    //	middle point of current set
		Vec3 mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_primitives.size() ; i++) {
      mid = mid + point_reps[stack_primitives[i]];
    }

		mid = (1.0f/stack_primitives.size()) * mid;	//divide by N
    
    //	build subsets based on axis/middle point
		std::vector<int> left;
		std::vector<int> right;
    
    left.reserve( stack_primitives.size() / 2 );
    right.reserve( stack_primitives.size() / 2 );

    
		for( unsigned int i = 0 ; i<stack_primitives.size() ; i++ ) {
      Vec3 mid_to_point = 
        point_reps[ stack_primitives[i] ] - mid;
      if ( mid_to_point * axis < 0 )
				left.push_back(stack_primitives[i]);
			else
				right.push_back(stack_primitives[i]);
    }
		
    //	anity check... sometimes current subset cannot be divided by longest 
    // axis: change axis or just half
		if ( (left.size() == 0) || (right.size() == 0) ) {
			left.clear();
			right.clear();
			for (unsigned int i = 0 ; i<stack_primitives.size()/2 ; i++) 
        left.push_back(stack_primitives[i]);
			for (unsigned int i = stack_primitives.size()/2 ;
            i<stack_primitives.size() ; i++) 
        right.push_back(stack_primitives[i]);
		}
		
		stack.pop();

    //	do recurse
		if ( left.size() != 0) {
      stack_tree->left.reset( new BBTreePrimitive );
			
			StackElementPrimitive element;
			element.tree = stack_tree->left.get();
			element.primitives.swap( left );
			stack.push( element );
		}
		
		if ( right.size() != 0) {
			stack_tree->right.reset( new BBTreePrimitive );
			
			StackElementPrimitive element;
			element.tree = stack_tree->right.get();
			element.primitives.swap( right );
			stack.push( element );
		}
	}
}

void BBTreePrimitive::getConstraints(
                                  const Vec3 &point,
                                  std::vector< PlaneConstraint > &constraints,
                                  FaceType face ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < primitives.size(); i++ ) {
      GeometryPrimitive *gp = primitives[i];
      gp->getConstraints( point, constraints, face );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, constraints, face );
    if (right.get()) right->getConstraints( point, constraints, face );
    
  }
}

void BBTreePrimitive::getConstraints(
                                  const Vec3 &point,
                                  const Matrix4 &matrix,
                                  std::vector< PlaneConstraint > &constraints,
                                  FaceType face ) {
 // if ( isLeaf() )	{
 //   for( unsigned int i = 0; i < primitives.size(); i++ ) {
 //     GeometryPrimitive *gp = primitives[i];
 //     gp->getConstraints( point, matrix, constraints, face );
	//	}
	//}	else 	{
	//	//if ( bound->boundIntersect( from, to ) )	{
 //   if (left.get()) left->getConstraints( point, matrix, constraints, face );
 //   if (right.get()) right->getConstraints( point, matrix, constraints, face );
 // }
}


void BBTreePrimitive::getPrimitivesWithinRadius(
                                    const Vec3 &p,
                                    HAPIFloat radius,
                                    vector< GeometryPrimitive * > &result ) {
  HAPIFloat r2 = radius * radius;
  if ( isLeaf() )	{
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); i++ ) {
      Vec3 cp, tmp;
      (*i)->closestPoint( p, cp, tmp, tmp );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
	}	else 	{
    Vec3 cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getPrimitivesWithinRadius( p, radius, result );
    if (right.get()) right->getPrimitivesWithinRadius( p, radius, result );
  }
}

void BBTreePrimitive::closestPoint( const Vec3 &p,
                                    Vec3 &closest_point,
                                    Vec3 &closest_normal,
                                    Vec3 &tex_coord ) {
  if ( isLeaf() )	{
    Vec3 cp, cn;
    HAPIFloat d2;
    if( primitives.size() == 0 ) return;
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); i++ ) {
      Vec3 point, normal;
      (*i)->closestPoint( p, point, normal, tex_coord );
      Vec3 v = p - cp;
      if( i == primitives.begin() ) {
        cp = point;
        cn = normal;
        d2 = v * v;
      } else {
        HAPIFloat new_d2 = v * v;
        if( new_d2 < d2 ) {
          cp = point;
          cn = normal;
          d2 = v * v;
        }
      }
    }
    closest_point = cp;
    closest_normal = cn;
	}	else 	{
   
    if( left.get() && right.get() ) {
      Vec3 cp, cn, tc;
      left->closestPoint( p, cp, cn, tc  );
      right->closestPoint( p, closest_point, closest_normal, tex_coord );
      Vec3 v = p - cp;
      HAPIFloat ld2 = v * v;
      v = p - closest_point;
      if( ld2 < v * v ) {
        closest_point = cp;
        closest_normal = cn;
        tex_coord = tc;
      }
      
    } else {
      if (left.get()) {
        left->closestPoint( p, closest_point, closest_normal, tex_coord  );
      } else {
        right->closestPoint( p, closest_point, closest_normal, tex_coord );
      }
    }
  }
}

void BBTreePrimitive::getAllPrimitives( vector< GeometryPrimitive * > &prim ) {
  if ( isLeaf() )	{
    prim.insert( prim.end(), primitives.begin(), primitives.end() );
	}	else 	{
    if (left.get()) left->getAllPrimitives( prim );
    if (right.get()) right->getAllPrimitives( prim );
  }
}

void BBTreePrimitive::clearCollidedFlag() {
  if( !isLeaf() ) {
    bound->collided = false;
    left->clearCollidedFlag();
    right->clearCollidedFlag();
  } else {
    for( unsigned int i = 0; i < primitives.size(); i++ )
      primitives[i]->collided = false;
  }
}



bool BBTreePrimitive::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     IntersectionInfo &result,
                                     FaceType face ) {
  if ( isLeaf() )	{
    // TODO: find closest?
    IntersectionInfo tempResult;
    for( unsigned int i = 0; i < primitives.size(); i++ ) {
      GeometryPrimitive *gp = primitives[i];
      if( gp->lineIntersect( from, to, tempResult, face ) ) {
        if( result.intersection) {
          if( (tempResult.point - from).lengthSqr() < 
              (result.point - from).lengthSqr() )
            result = tempResult;
        }
        else
          result = tempResult;
        return true;
      }
		}
		return false;
	}	else 	{
		if ( bound->boundIntersect( from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) overlap |=
        left->lineIntersect( from, to, result, face );
			if (right.get()) overlap |=
        right->lineIntersect( from, to, result, face );
			
			return overlap;
		}
		else return false;
	}
}

bool BBTreePrimitive::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
  if ( isLeaf() )	{
    // TODO: find closest?
    for( unsigned int i = 0; i < primitives.size(); i++ ) {
      GeometryPrimitive *gp = primitives[i];
      if( gp->movingSphereIntersect( radius, from, to ) )	return true;
		}
		return false;
	}	else 	{
		if ( bound->boundMovingSphereIntersect( radius, from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) {
        overlap = left->movingSphereIntersect( radius, from, to );
        if( overlap ) return overlap;
      }
			if (right.get()) 
        overlap = right->movingSphereIntersect( radius,from, to );
			
			return overlap;
		}
		else return false;
	}
}

void BBTreePrimitive::render() {
  render( 0 );
}

void BBTreePrimitive::render( int depth) {
  if( !isLeaf() ) {
    if( depth == 0 ) {
      bound->render();
    } else {
      if( left.get() ) left->render( depth - 1 );
      if( right.get() ) right->render( depth - 1 );
    }
  } else {
    for( unsigned int i = 0; i < primitives.size(); i++ ) { 
      primitives[i]->render();
    }
  }
}

void BBTreePrimitive::getPrimitivesIntersectedByMovingSphere(
                                      HAPIFloat radius,
                                      Vec3 from,
                                      Vec3 to,
                                      vector< GeometryPrimitive * > &result) {
  if ( isLeaf() )	{
    for( H3DUtil::AutoRefVector< GeometryPrimitive >::const_iterator
          i = primitives.begin();
         i != primitives.end(); i++ ) {
      if( (*i)->movingSphereIntersect( radius, from, to ) )
        result.push_back( *i );
    }
	}	else 	{
    if(  !bound->boundMovingSphereIntersect( radius, from, to ) ) return;

    if (left.get()) left->getPrimitivesIntersectedByMovingSphere(
      radius, from, to, result );
    if (right.get()) right->getPrimitivesIntersectedByMovingSphere(
      radius, from, to, result );
  }
}
