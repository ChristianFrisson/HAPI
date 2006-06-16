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

using namespace H3D;
using namespace Bounds;

bool PlaneConstraint::lineIntersect( const Vec3d &from, 
                                     const Vec3d &to,
                                     Bounds::IntersectionInfo &result ) {
      Vec3d from_to = to - from;
      if( normal * from_to > Constants::f_epsilon ) { 
        /*if( result.normal.z == 0.5 ) {
          cerr << "a" << endl;
          cerr << normal * from_to << endl;
          }*/
        return false;
      }
      H3DDouble denom = normal * from_to;
      if( denom * denom < Constants::d_epsilon ) {
        /*        if( result.normal.z == 0.5 )
                  cerr << "b" << endl; */
        return false;
      }
      H3DDouble u = ( normal * ( point - from ) ) / denom; 
      /*      if( result.normal.z == 0.5 )
              cerr << u << endl;*/
      if( u <= 1 + Constants::f_epsilon && u >= 0 - Constants::f_epsilon ) {
        result.point = from + u * from_to;
        result.normal = normal;
        return true;
      }
      return false;
    }


void AABoxBound::render() {
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3f( 1, 0, 0 );
  else
    glColor3f( 1, 1, 0 );
  glBegin( GL_LINE_STRIP );
  glVertex3f( min.x, min.y, min.z );
  glVertex3f( min.x, max.y, min.z );
  glVertex3f( max.x, max.y, min.z );
  glVertex3f( max.x, min.y, min.z );
  glVertex3f( min.x, min.y, min.z );
  glEnd();
  
  glBegin( GL_LINE_STRIP );
  glVertex3f( min.x, min.y, max.z );
  glVertex3f( min.x, max.y, max.z );
  glVertex3f( max.x, max.y, max.z );
  glVertex3f( max.x, min.y, max.z );
  glVertex3f( min.x, min.y, max.z );
  glEnd();
  
  glBegin( GL_LINES );
  glVertex3f( min.x, min.y, max.z );
  glVertex3f( min.x, min.y, min.z );
  glVertex3f( max.x, min.y, max.z );
  glVertex3f( max.x, min.y, min.z );
  glVertex3f( min.x, max.y, max.z );
  glVertex3f( min.x, max.y, min.z );
  glVertex3f( max.x, max.y, max.z );
  glVertex3f( max.x, max.y, min.z );
  glEnd();
  glEnable( GL_LIGHTING );
}

void AABoxBound::fitAroundPoints( const vector< Vec3f > &points ) {
  if( points.size() == 0 ) return;
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


bool AABoxBound::boundIntersect( const Vec3d &from, 
                                 const Vec3d &to ) {
  Vec3d c = 0.5f*(min+max);
	Vec3d e = max-c;
	Vec3d m = 0.5f*(from+to);
	Vec3d d = to-m;

	m = m-c;

	H3DDouble adx = H3DAbs(d.x);
	if (H3DAbs(m.x) > e.x + adx) return false;
	H3DDouble ady = H3DAbs(d.y);
	if (H3DAbs(m.y) > e.y + ady) return false;
	H3DDouble adz = H3DAbs(d.z);
	if (H3DAbs(m.z) > e.z + adz) return false;

  //	bias with an epsilon to increase robustness when segment is parallel 
  // to a box plane...
	H3DDouble epsilon = (e*e)/100000;
	adx += epsilon;
	ady += epsilon;
	adz += epsilon;

	if (H3DAbs(m.y*d.z - m.z*d.y) > e.y*adz + e.z*ady) return false;
	if (H3DAbs(m.z*d.x - m.x*d.z) > e.x*adz + e.z*adx) return false;
	if (H3DAbs(m.x*d.y - m.y*d.x) > e.x*ady + e.y*adx) return false;

	return true;
}

Vec3d AABoxBound::closestPoint( const Vec3d &p ) {
  assert( false );
  return Vec3d();
}

bool AABoxBound::insideBound( const Vec3d &p ) {
  return ( p.x <= max.x &&
           p.x >= min.x &&
           p.y <= max.y &&
           p.y >= min.y &&
           p.z <= max.z &&
           p.z >= min.z );
}

Vec3d SphereBound::closestPoint( const Vec3d &p ) {
  Vec3d dir = p - center;
  if( dir * dir < Constants::f_epsilon )
    return center + Vec3d( radius, 0, 0 ); 
  else {
    dir.normalize();
    return center + dir * radius;
  }
}

bool SphereBound::insideBound( const Vec3d &p ) {
  Vec3d v = p - center;
  H3DDouble d2 = v * v;
  H3DDouble r2 = radius * radius;
  return d2 <= r2;
}

bool SphereBound::boundIntersect( const Vec3d &from, 
                                  const Vec3d &to ) {
  Vec3d ab = to - from;
	Vec3d ap = center-from;
	Vec3d bp = center-to;
	H3DDouble r2 = radius * radius;

	H3DDouble e = ap * ab;
	if (e<0)  return ap*ap <= r2;
	
	H3DDouble f = ab * ab;
	if (e>=f) return bp*bp <= r2;
	
	return (ap*ap - e*e/f) <= r2;
}

bool SphereBound::lineIntersect( const Vec3d &from, 
                                 const Vec3d &to,
                                 IntersectionInfo &result ) {
  Vec3d start_point = from - center;
  Vec3d end_point = to - center;
  bool flip_normal = false;

  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3d p, v;
  H3DDouble r2 = radius * radius;

  H3DDouble a0  = start_point * start_point - r2;
  if (a0 <= 0) {
    // start_point is inside sphere
    a0 = end_point * end_point - r2;
    if( a0 <= 0 ) {
      // both start_point and end_point are inside sphere so no intersection
      return false;
    } else {
      // start_point is inside and end_point is outside. We will have an 
      // intersection.
      p = end_point;
      v = start_point - end_point;
      flip_normal = true;
    }
  } else {
    // start_point is outside sphere
    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    H3DDouble a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
  }
  // use implicit quadratic formula to find the roots
  H3DDouble a = v.x*v.x + v.y*v.y + v.z*v.z;
  H3DDouble b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  H3DDouble c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  H3DDouble s = b*b - 4*a*c;

  H3DDouble u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    H3DDouble u0 = (-b + sqrt(s))/(2*a);
    H3DDouble u1 = (-b - sqrt(s))/(2*a);
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
  if( flip_normal ) 
    result.normal = -result.normal;
  return true;
}

void SphereBound::render( ) {
  if( !gl_quadric ) gl_quadric = gluNewQuadric();
  glDisable( GL_LIGHTING );
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glTranslatef( center.x, center.y, center.z );
  if( collided )
    glColor3f( 1, 0, 0 );
  else
    glColor3f( 1, 1, 0 );
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  gluSphere( gl_quadric, radius, 10, 10 );
  glPopMatrix();
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  glEnable( GL_LIGHTING );
}

void SphereBound::fitAroundPoints( const vector< Vec3f > &points ) {
	AABoxBound box;
  box.fitAroundPoints( points );
	Vec3f c = 0.5 * (box.min + box.max);
	H3DFloat r = 0;
	
  //	find largest square distance
	for(unsigned int i=0 ; i<points.size() ; i++) {
    Vec3f from_center = points[i]-c;
    H3DFloat d = from_center * from_center;
    if (d > r) r = d;
  }
	
  center = c;
  radius = H3DSqrt( r );
}

struct StackElement {
  BinaryBoundTree *tree;
  vector< Triangle > triangles;
}; 

BinaryBoundTree::BinaryBoundTree( BoundNewFunc func, 
                                  const vector< Triangle > &triangle_vector ):
  new_func( func ), left( NULL ), right( NULL ) {
  
  std::stack< StackElement > stack;
	
  if( triangle_vector.size() == 0 ) return;
 
  //	add the starting subtree to stack...
	StackElement start;
	start.tree = this;
	start.triangles = triangle_vector;
	stack.push( start );
  //cerr << start.triangles.size() << endl;
  //	BUILD TREE
	while( !stack.empty() ) {
		const std::vector< Triangle > &stack_triangles = stack.top().triangles;
		BinaryBoundTree* stack_tree = stack.top().tree;

	  if (stack_triangles.size() == 1 ) {
      //	build a leaf
			stack_tree->triangles = stack_triangles;
      		stack.pop();
			continue;
		}

		std::vector<Vec3f > points;
		points.resize( stack_triangles.size() * 3 );
		for (unsigned int i=0 ; i<stack_triangles.size() ; i++ ) {
      points[i*3]   = stack_triangles[i].a;
      points[i*3+1] = stack_triangles[i].b;
      points[i*3+2] = stack_triangles[i].c;
    }
    
    //	build bounding shape
		stack_tree->bound.reset( new_func() );
    stack_tree->bound->fitAroundPoints( points );
		
    //	DIVIDE SUBSET AND RECURSE
    //	longest axis
		Vec3f axis = stack_tree->bound->longestAxis();

    //	middle point of current set
		Vec3f mid(0,0,0);

    for(unsigned int i = 0 ; i < stack_triangles.size() ; i++) {
      mid = mid + stack_triangles[i].pointRepresentation();
    }

		mid = (1.0f/stack_triangles.size()) * mid;	//divide by N
    
    //	build subsets based on axis/middle point
		std::vector<Triangle> left;
		std::vector<Triangle> right;
    
		for( unsigned int i = 0 ; i<stack_triangles.size() ; i++ ) {
      Vec3f mid_to_point = stack_triangles[i].pointRepresentation() - mid;
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
			element.triangles = left;
			stack.push( element );
		}
		
		if ( right.size() != 0) {
			stack_tree->right.reset( new BinaryBoundTree );
			
			StackElement element;
			element.tree = stack_tree->right.get();
			element.triangles = right;
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
    glColor3f( 1, 0, 0 );
  else {
    glColor3f( 0, 0, 1 );
  }

  glBegin( GL_LINE_STRIP );
  glVertex3f( a.x, a.y, a.z );
  glVertex3f( b.x, b.y, b.z );
  glVertex3f( c.x, c.y, c.z );
  glVertex3f( a.x, a.y, a.z );
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
  }
}

Vec3d Triangle::closestPoint( const Vec3d &p ) {
 
  Vec3d ab = Vec3d(b)-Vec3d(a);
  Vec3d ac = Vec3d(c)-Vec3d(a);
  Vec3d ap = Vec3d(p)-Vec3d(a) ;

  H3DDouble d1 = ab * ap;
  H3DDouble d2 = ac * ap;
  if( d1 <= Constants::d_epsilon && d2 <= Constants::d_epsilon )
    return a;

  Vec3d bp = p - b;
  H3DDouble d3 = ab * bp;
  H3DDouble d4 = ac * bp;

  if( d3 >= -Constants::d_epsilon && d4 <= d3 )
    return b;
  
  H3DDouble vc = d1*d4 - d3*d2;
  if( vc <= Constants::d_epsilon && 
      d1 >= -Constants::d_epsilon && 
      d3 <= Constants::d_epsilon ) {
    H3DDouble v = d1 / ( d1 - d3 );
    return Vec3d(a) + v * ab;
  }

  Vec3d cp = p - c;
  H3DDouble d5 = ab * cp;
  H3DDouble d6 = ac * cp;
  if( d6 >= -Constants::d_epsilon && d5 <= d6 )
    return c;

  H3DDouble vb = d5*d2 - d1*d6;
  if( vb <= Constants::d_epsilon && 
      d2 >= -Constants::d_epsilon &&
      d6 <= Constants::d_epsilon ) {
    H3DDouble w = d2 / ( d2 - d6 );
    return Vec3d(a) + w * ac;
  }
  
  H3DDouble va = d3*d6 - d5*d4;
  if( va <= Constants::d_epsilon &&
      (d4-d3) >= -Constants::d_epsilon &&
      (d5-d6) >= -Constants::d_epsilon ) {
    H3DDouble w = (d4-d3) /((d4 -d3) + (d5 - d6 ) );
    return Vec3d(b) + w * (Vec3d(c)-Vec3d(b));
  }

  H3DDouble denom = 1 / ( va + vb + vc );
  H3DDouble v = vb * denom;
  H3DDouble w = vc * denom;
  return Vec3d(a) + ab * v + ac * w;
  /*
  Vec3d normal = ca % cb;
  if( H3DAbs( normal * normal ) > Constants::d_epsilon ) {
    normal.normalizeSafe();
    
    Vec3d pp = p - (-ap).dotProduct( -normal ) * normal;
    
    // determine dominant axis
    int axis;
    if (H3DAbs( normal.x ) > H3DAbs( normal.y )) {
      if (H3DAbs( normal.x ) > H3DAbs( normal.z )) axis = 0; 
      else axis = 2;
    } else {
      if (H3DAbs( normal.y ) > H3DAbs( normal.z )) axis = 1; 
      else axis = 2;
    }
    
    int u_axis = (axis + 1 ) % 3;
    int v_axis = (axis + 2 ) % 3;
    
    Vec3d cp = pp - c;
    
    H3DDouble denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);
    
    H3DDouble du = 
      (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;
    
    if( du >= 0 && du <= 1 ) {
      H3DDouble dv = 
        (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;
      if( dv >= 0 && dv <= 1 ) {
        H3DDouble dw = 1 - du - dv;
        if( dw >= 0 && dw <= 1 ) {
          return pp;
        }
      }
    }
  }

  // point is on edge.

  Vec3d bp = p-b ;
  Vec3d cp = p-c ;

  H3DDouble ca_length2 = ca.lengthSqr();
  H3DDouble cb_length2 = cb.lengthSqr();
  H3DDouble ab_length2 = ab.lengthSqr();

  H3DDouble ca_t = (cp * ca) / ca_length2;
  H3DDouble cb_t = (cp * cb) / cb_length2;
  H3DDouble ab_t = (ap * ab) / ab_length2;

  if( ca_t < 0 && cb_t < 0 )
    return c;

  if( cb_t > 1 && ab_t > 1 )
    return b;

  if( ca_t > 1 && ab_t < 0 )
    return a;
  
  bool discard_ca = ca_t < 0 || ca_t > 1;
  bool discard_cb = cb_t < 0 || cb_t > 1;;
  bool discard_ab = ab_t < 0 || ab_t > 1;;

  Vec3d ca_p = c + ca * ca_t;
  Vec3d cb_p = c + cb * cb_t;
  Vec3d ab_p = a + ab * ab_t;
  
  H3DDouble dist_cap = (ca_p - p).lengthSqr();  
  H3DDouble dist_cbp = (cb_p - p).lengthSqr();  
  H3DDouble dist_abp = (ab_p - p).lengthSqr();  

  Vec3d closest;
  H3DDouble dist;

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

bool Triangle::lineIntersect( const Vec3d &from, 
                              const Vec3d &to,
                              IntersectionInfo &result ) {
  Vec3d ca = a-c;
  Vec3d cb = b-c;
  Vec3d dir = to - from;
  Vec3d normal = ca % cb;
  if( H3DAbs( normal * normal ) < Constants::d_epsilon ||
      H3DAbs( dir * dir ) < Constants::d_epsilon ||
      H3DAbs( normal * dir ) < Constants::d_epsilon ) {
    return false;
  }
  normal.normalizeSafe();

  H3DDouble dir_length = dir.length();
  dir.normalizeSafe();

  H3DDouble t = ( normal * ( c - from ) ) / ( normal * dir );
  // check that the point is within the line segement
  if( t < 0 || t > dir_length ) return false;

  Vec3d p = from + t * dir;

  // determine dominant axis
  int axis;
  if (H3DAbs( normal.x ) > H3DAbs( normal.y )) {
    if (H3DAbs( normal.x ) > H3DAbs( normal.z )) axis = 0; 
    else axis = 2;
  } else {
    if (H3DAbs( normal.y ) > H3DAbs( normal.z )) axis = 1; 
    else axis = 2;
  }

  int u_axis = (axis + 1 ) % 3;
  int v_axis = (axis + 2 ) % 3;

  Vec3d cp = p - c;

  H3DDouble denom = (ca[u_axis] * cb[v_axis] - ca[v_axis] * cb[u_axis]);

  H3DDouble du = 
    (cp[u_axis] * cb[v_axis] - cp[v_axis] * cb[u_axis]) / denom;

  if( du < 0 || du > 1 ) return false;

  H3DDouble dv = 
    (ca[u_axis] * cp[v_axis] - ca[v_axis] * cp[u_axis]) / denom;

  if( dv < 0 || dv > 1 ) return false;

  H3DDouble dw = 1 - du - dv;

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


void BinaryBoundTree::clearCollidedFlag() {
  if( !isLeaf() ) {
    bound->collided = false;
    left->clearCollidedFlag();
    right->clearCollidedFlag();
  } else {
    for( unsigned int i = 0; i < triangles.size(); i++ )
      triangles[i].collided = false;
  }
}

bool BinaryBoundTree::lineIntersect( const Vec3d &from, 
                                     const Vec3d &to,
                                     IntersectionInfo &result ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      if( t.lineIntersect( from, to, result ) )	return true;
		}
		return false;
	}	else 	{
		if ( bound->boundIntersect( from, to ) )	{
			bool overlap = false;
      bound->collided = true;
			
			if (left.get()) overlap |= left->lineIntersect( from, to, result );
			if (right.get()) overlap |= right->lineIntersect( from, to, result );
			
			return overlap;
		}
		else return false;
	}
}

Matrix3f covarianceMatrix( const vector< Vec3f >&points ) {
  // TODO: if no points?

  unsigned int nr_points = points.size();

	Vec3f c = points[0];
	for(unsigned int i=1 ; i < nr_points; i++) 
    c+= points[i];

	c /= points.size();
	
  H3DFloat e00 = 0, e01 = 0, e02 = 0,
                    e11 = 0, e12 = 0,
                             e22 = 0;
	for (unsigned int i = 0; i < nr_points; i++)	{
		Vec3f p = points[i] - c;
		
		e00 += p.x * p.x;
		e11 += p.y * p.y;
		e22 += p.z * p.z;
		e01 += p.x * p.y;
		e02 += p.x * p.z;
		e12 += p.y * p.z;
	}
	
	Matrix3f cov;
	cov[0][0] = e00 / nr_points;
	cov[1][1] = e11 / nr_points;
	cov[2][2] = e22 / nr_points;
	cov[0][1] = cov[1][0] = e01 / nr_points;
	cov[0][2] = cov[2][0] = e02 / nr_points;
	cov[1][2] = cov[2][1] = e12 / nr_points;
	
	return cov;
}

void symmetricSchurDecomposition( const Matrix3f &A, 
                                  unsigned int p, 
                                  unsigned int q, 
                                  H3DFloat &c, 
                                  H3DFloat &s) {
  // TODO: epsilon??
	if (H3DAbs(A[p][q]) > 0.0001f) {
		H3DFloat r = (A[q][q] - A[p][p]) / (2.0f * A[p][q]);
		H3DFloat t = (r >= 0.0f ? 
                  1.0f / (r + H3DSqrt(1.0f + r*r)) : 
                  -1.0f / (-r + H3DSqrt(1.0f + r*r)) );
		
		c = 1.0f / H3DSqrt(1.0f + t*t);
		s = t * c;
	}	else{
		c = 1.0f;
		s = 0.0f;
	}
}

void eigenValuesAndEigenVectorsJacobi(const Matrix3f& A, 
                                      Vec3f& eigValues, 
                                      Matrix3f& eigVectors, 
                                      const unsigned int maxIterations = 50 ) {
  //!NOTE: on return rows (not cols) of eigVectors contains eigenvalues

	Matrix3f a = A;
	Matrix3f v;
	
	unsigned int i, j, n, p, q;
	H3DFloat prevoff, c, s;
	Matrix3f J, b, t;

  // Repeat for some maximum number of iterations
	for (n = 0; n < maxIterations; n++)	{
		p = 0; q = 1;
		for ( i = 0; i < 3; i++) for (j = 0; j < 3; j++ ) {
			if (i == j) continue;
			if (H3DAbs(a[i][j]) > H3DAbs(a[p][q])) {p = i; q = j;}
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
		H3DFloat off = 0.0f;
		for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)	{
			if (i == j) continue;
			off += a[i][j] * a[i][j];
		}
		
    // stop condition
		if (n > 2 && off >= prevoff) break;
		prevoff = off;
	}
	
	eigValues = Vec3f(a[0][0],a[1][1],a[2][2]);
	eigVectors = v.transpose();
}

void OrientedBoxBound::fitAroundPoints( const vector< Vec3f > &points ) {
  Matrix3f covariance = covarianceMatrix( points );
	
	Vec3f eig_values;
	Matrix3f eig_vectors;

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

	Vec3f principal( eig_vectors[length_order[0]][0],
                   eig_vectors[length_order[0]][1],
                   eig_vectors[length_order[0]][2] );

	Vec3f secondary( eig_vectors[length_order[1]][0],
                   eig_vectors[length_order[1]][1],
                   eig_vectors[length_order[1]][2] );

  principal.normalize();
  secondary.normalize();
  Vec3f v = principal % secondary;

	Matrix3f frame( principal.x, principal.y, principal.z,
                  secondary.x, secondary.y, secondary.z, 
                  v.x, v.y, v.z );
		
  //	build box in the new frame
  min = frame * points[0];
	max = min;
	for(unsigned int i=1 ; i<points.size() ; i++)	{
		Vec3f pt = frame * points[i];
		
		if (pt.x < min.x) min.x = pt.x;
		if (pt.y < min.y) min.y = pt.y;
		if (pt.z < min.z) min.z = pt.z;
		
		if (pt.x > max.x) max.x = pt.x;
		if (pt.y > max.y) max.y = pt.y;
		if (pt.z > max.z) max.z = pt.z;
	}
	
  //	fill oriented box
  orientation = Rotation( frame );
  
  cerr << frame * (0.5f*(min+max)) << endl;
	center = frame.transpose() * (0.5f*(min+max)) ;
  halfsize = 0.5f*(max-min);
}

bool OrientedBoxBound::boundIntersect( const Vec3d &from, 
                                       const Vec3d &to ) {
  Vec3d rp = orientation * from;
	Vec3d rq = orientation * to;
	
	return AABoxBound::boundIntersect( rp, rq );
}

Vec3d OrientedBoxBound::closestPoint( const Vec3d &p ) {
  assert( false );
  return Vec3d();
}

bool OrientedBoxBound::insideBound( const Vec3d &p ) {
  Vec3d rp = orientation * p; 
  return ( rp.x <= max.x &&
           rp.x >= min.x &&
           rp.y <= max.y &&
           rp.y >= min.y &&
           rp.z <= max.z &&
           rp.z >= min.z );
}

void OrientedBoxBound::render( ) {
  Vec3f center = -orientation * ((max + min)/2);
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glDisable( GL_LIGHTING );
  if( collided )
    glColor3f( 1, 0, 0 );
  else
    glColor3f( 1, 1, 0 );
  glTranslatef( center.x, center.y, center.z );
  glRotatef( -orientation.angle *180 / H3D::Constants::pi, 
             orientation.axis.x, orientation.axis.y, orientation.axis.z );
  Vec3f minf( - halfsize );
  Vec3f maxf( halfsize );
  glBegin( GL_LINE_STRIP );
  glVertex3f( minf.x, minf.y, minf.z );
  glVertex3f( minf.x, maxf.y, minf.z );
  glVertex3f( maxf.x, maxf.y, minf.z );
  glVertex3f( maxf.x, minf.y, minf.z );
  glVertex3f( minf.x, minf.y, minf.z );
  glEnd();
  
  glBegin( GL_LINE_STRIP );
  glVertex3f( minf.x, minf.y, maxf.z );
  glVertex3f( minf.x, maxf.y, maxf.z );
  glVertex3f( maxf.x, maxf.y, maxf.z );
  glVertex3f( maxf.x, minf.y, maxf.z );
  glVertex3f( minf.x, minf.y, maxf.z );
  glEnd();
  
  glBegin( GL_LINES );
  glVertex3f( minf.x, minf.y, maxf.z );
  glVertex3f( minf.x, minf.y, minf.z );
  glVertex3f( maxf.x, minf.y, maxf.z );
  glVertex3f( maxf.x, minf.y, minf.z );
  glVertex3f( minf.x, maxf.y, maxf.z );
  glVertex3f( minf.x, maxf.y, minf.z );
  glVertex3f( maxf.x, maxf.y, maxf.z );
  glVertex3f( maxf.x, maxf.y, minf.z );
  glEnd();
  glEnable( GL_LIGHTING );
  glPopMatrix();
}

void Triangle::getConstraints( const Vec3d &point,
                               H3DDouble radius,
                               std::vector< PlaneConstraint > &constraints ) {
  Vec3d closest_point = closestPoint( point );
  //cerr << closest_point << endl;
  Vec3d normal = point - closest_point;
  normal.normalizeSafe();
  //cerr << closest_point << endl;
  constraints.push_back( PlaneConstraint( closest_point, normal ) );
}

void Triangle::getConstraints( const Vec3d &point,
                               H3DDouble radius,
                               const Matrix4d &matrix,
                               std::vector< PlaneConstraint > &constraints ) {
  Vec3f oa = a;
  Vec3f ob = b;
  Vec3f oc = c;
  a = (Vec3f)(matrix * a);
  b = (Vec3f)(matrix * b);
  c = (Vec3f)(matrix * c);
  getConstraints( point, radius, constraints );
  a = oa;
  b = ob;
  c = oc;
}


void BinaryBoundTree::getConstraints( const Vec3d &point,
                                      H3DDouble radius,
                                      std::vector< PlaneConstraint > &constraints ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, radius, constraints );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, radius, constraints );
    if (right.get()) right->getConstraints( point, radius, constraints );
    
  }
}

void BinaryBoundTree::getConstraints( const Vec3d &point,
                                      H3DDouble radius,
                                      const Matrix4d &matrix,
                                      std::vector< PlaneConstraint > &constraints ) {
  if ( isLeaf() )	{
    for( unsigned int i = 0; i < triangles.size(); i++ ) {
      Triangle &t = triangles[i];
      t.getConstraints( point, radius, matrix, constraints );
		}
	}	else 	{
		//if ( bound->boundIntersect( from, to ) )	{
    if (left.get()) left->getConstraints( point, radius, matrix, constraints );
    if (right.get()) right->getConstraints( point, radius, matrix, constraints );
    
  }
}


void BinaryBoundTree::getTrianglesWithinRadius( const Vec3d &p,
                                                H3DDouble radius,
                                                vector< Triangle > &result ) {
  H3DDouble r2 = radius * radius;
  if ( isLeaf() )	{
    for( vector< Triangle >::iterator i = triangles.begin();
         i != triangles.end(); i++ ) {
      Vec3d cp = (*i).closestPoint( p );
      if( (cp - p).lengthSqr() <= r2 )
        result.push_back( *i );
    }
    //cerr << "!: " << triangles.size() << endl;
           //result.insert( result.end(), triangles.begin(), triangles.end() );
	}	else 	{
    Vec3d cp = bound->boundClosestPoint( p );
    if( (cp - p).lengthSqr() > r2 && !bound->insideBound(p) ) return;

    if (left.get()) left->getTrianglesWithinRadius( p, radius, result );
    if (right.get()) right->getTrianglesWithinRadius( p, radius, result );
  }
}
