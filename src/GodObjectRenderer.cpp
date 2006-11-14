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
/// \file GodObjectRenderer.cpp
/// \brief cpp file for GodObjectRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "GodObjectRenderer.h"
#include "H3DMath.h"
#include "HAPIHapticsDevice.h"

using namespace HAPI;

HAPIHapticsRenderer::HapticsRendererRegistration 
GodObjectRenderer::renderer_registration(
                            "GodObject",
                            &(newInstance< GodObjectRenderer >)
                            );

// epsilon value for deciding if a point is the same
const HAPIFloat length_sqr_point_epsilon = 1e-12; //12

// epsilon value for deciding if a normal is the same.
const HAPIFloat length_sqr_normal_epsilon = 1e-12;

void GodObjectRenderer::onOnePlaneContact( 
                            const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact ) {
  
  // create a local coordinate system with the PlaneConstraint normal
  // as y-axis
  contact.y_axis = c.normal;
  Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
  contact.x_axis = contact.y_axis % a;
  contact.z_axis = contact.x_axis % contact.y_axis;
  contact.x_axis.normalizeSafe();
  contact.z_axis.normalizeSafe();
  assert( c.haptic_shape );

  // call the surface to determine forces and proxy movement
  c.haptic_shape->surface->onContact( contact );
  
  // add a contact
  tmp_contacts.push_back( make_pair( c.haptic_shape, contact ) );
}

void GodObjectRenderer::onTwoPlaneContact( 
           const PlaneConstraint &p0,
           const PlaneConstraint &p1,
           HAPISurfaceObject::ContactInfo &contact ) {
  
  Vec3 contact_global = contact.globalContactPoint();

  // the direction along the intersection of the two planes. 
  Vec3 line_dir = p0.normal % p1.normal;
  line_dir.normalizeSafe();

  // transformation matrix from local coordinate system with the normals
  // of the constraint planes as axis to global space
  Matrix4 m( p0.normal.x, p1.normal.x, line_dir.x, contact_global.x, 
             p0.normal.y, p1.normal.y, line_dir.y, contact_global.y, 
             p0.normal.z, p1.normal.z, line_dir.z, contact_global.z,
             0, 0, 0, 1 );

  // global -> local
  Matrix4 m_inv = m.inverse();
  
  // probe position in local coordinates
  Vec3 local_pos = m_inv * contact.globalProbePosition();

  // check that both planes constrain, if not just do one plane contact
  // calculations
  if( local_pos.x > Constants::epsilon ) {
    onOnePlaneContact( p1, contact );
  } else if( local_pos.y > Constants::epsilon ) {
    onOnePlaneContact( p0, contact );
  } else {
    // both planes constrains

    HAPIFloat sum = local_pos.x + local_pos.y;
    
    HAPIFloat weight = 0;
    if( sum < 0 )
      weight = local_pos.x / sum;
    
    // create a local coordinate system with the PlaneConstraint normal
    // as y-axis
    contact.y_axis = p0.normal * local_pos.x + p1.normal * local_pos.y;
    contact.y_axis.normalizeSafe();
    
    Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;
    contact.z_axis.normalizeSafe();
    contact.x_axis.normalizeSafe();
    
    Vec3 line_dir_local = contact.vectorToLocal( line_dir );

    assert( p0.haptic_shape );

    // calculate the force and proxy movement for the first plane
    p0.haptic_shape->surface->onContact( contact );
    
    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p0_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
            0, 
            contact.proxy_movement_local.y ) *
      line_dir_local;

    Vec3 p0_force = contact.force_global;
    
    assert( p1.haptic_shape );
    // calculate the force and proxy movement for the second plane
    p1.haptic_shape->surface->onContact( contact );

    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p1_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
             0, 
             contact.proxy_movement_local.y ) *
      line_dir_local;

    Vec3 p1_force = contact.force_global;
    
    // calculate and set the final output parameters for force and 
    // proxy movement
    Vec3 proxy_movement = 
      H3DUtil::H3DMin( p0_proxy_movement, p1_proxy_movement ) * 
      line_dir_local;

    contact.proxy_movement_local = Vec2( proxy_movement.x, proxy_movement.z ); 
    contact.force_global = p0_force * weight + p1_force * ( 1 - weight );

    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape, contact ) );
    if( p0.haptic_shape != p1.haptic_shape )
      tmp_contacts.push_back( make_pair( p1.haptic_shape, contact ) );
  }
}


void GodObjectRenderer::onThreeOrMorePlaneContact(  
          vector< PlaneConstraint > &constraints,
          HAPISurfaceObject::ContactInfo &contact ) {

  assert( constraints.size() >= 3 );

  // find the first three planes that all constrain the proxy
  vector< PlaneConstraint >::iterator i = constraints.begin();
  PlaneConstraint &p0 = (*i++);
  PlaneConstraint &p1 = (*i++);
  PlaneConstraint &p2 = (*i++);

  Vec3 contact_global = contact.globalContactPoint();
  Vec3 probe_local_pos;

  while( i != constraints.end() ) {
    // transformation matrix from local coordinate system with the normals
    // of the constraint planes as axis to global space
    Matrix4 m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
               p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
               p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
               0, 0, 0, 1 );
    Matrix4 m_inv = m.inverse();
    probe_local_pos = m_inv * contact.globalProbePosition();
    
    if( probe_local_pos.x > Constants::epsilon ) {
      // discard p0
      std::swap( p0, *i++ ); 
    } else if( probe_local_pos.y > Constants::epsilon ) {
      // discard p1
      std::swap( p1, *i++ ); 
    } else if( probe_local_pos.z > Constants::epsilon ) {
      // discard p2
      std::swap( p2, *i++ );
    } else {
      break;
    }
  }

  // check that all three planes constrain, if not just do two plane contact
  // calculations
  if( probe_local_pos.x > Constants::epsilon ) {
    onTwoPlaneContact( p1, p2, contact );
  } else if( probe_local_pos.y > Constants::epsilon ) {
    onTwoPlaneContact( p0, p2, contact );
  } else if( probe_local_pos.z > Constants::epsilon ) {
    onTwoPlaneContact( p0, p1, contact );
  } else {

    // find weighting factors between different planes
    HAPIFloat sum = probe_local_pos.x + probe_local_pos.y + probe_local_pos.z;
    
    Vec3 weight;
    if( sum < 0 )
      weight = probe_local_pos / sum;

    // create a local coordinate system with the x-axis pointing towards the
    // proxy
    contact.y_axis = 
      contact.globalContactPoint() - contact.globalProbePosition();
    contact.y_axis.normalizeSafe();

    Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;

    contact.x_axis.normalizeSafe();
    contact.y_axis.normalizeSafe();
    contact.z_axis.normalizeSafe();

    // calculate the force and proxy movement for the first plane
    assert( p0.haptic_shape );
    p0.haptic_shape->surface->onContact( contact );
    Vec3 p0_force = contact.force_global;
    
    // calculate the force and proxy movement for the second plane
    assert( p1.haptic_shape );
    p1.haptic_shape->surface->onContact( contact );
    Vec3 p1_force = contact.force_global;

    // calculate the force and proxy movement for the third plane
    assert( p2.haptic_shape );
    p2.haptic_shape->surface->onContact( contact );
    Vec3 p2_force = contact.force_global;
    
    // proxy is constrained by three planes and cannot move in any 
    // direction
    contact.proxy_movement_local = Vec2( 0, 0 ); 
    
    // calulate final force
    contact.force_global = 
      p0_force * weight.x + 
      p1_force * weight.y + 
      p2_force * weight.z;


    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape, contact ) );
    if( p0.haptic_shape != p1.haptic_shape ) {
      tmp_contacts.push_back( make_pair( p1.haptic_shape, contact ) );
    }
    
    if( p0.haptic_shape != p2.haptic_shape &&
        p1.haptic_shape != p2.haptic_shape ) {
      tmp_contacts.push_back( make_pair( p2.haptic_shape, contact ) );
    }
  }    
}


inline bool planeIntersect( Vec3 p1, Vec3 n1, Vec3 p2, Vec3 n2,
                            Vec3 &p, Vec3 &dir ) {
  HAPIFloat d1 = n1 * p1;
  HAPIFloat d2 = n2 * p2;
  
  // Direction of intersection line
  dir = n1 % n2;

  // Check if planes are parallell
  HAPIFloat denom = dir * dir;
  if( denom < Constants::epsilon ) return false;
  
  // Compute point on intersection line.
  p = ( (d1 * n2 - d2 * n1 ) % dir ) / denom;
  return true;
}
  
inline Vec3 projectOntoLine( Vec3 p, Vec3 point1, Vec3 point2, bool segment = false ) {
  Vec3 ab = point2 - point1;

  // Project c onto ab.
  HAPIFloat t = (p - point1) * ab / (ab * ab );

  if( segment ) {
    if( t < 0 ) t = 0;
    if( t > 1 ) t = 1;
  }
  
  return  point1 + t * ab;
} 

inline Vec3 projectOntoPlane( Vec3 p, Vec3 point_on_plane, Vec3 normal ) {
  HAPIFloat t = normal * ( p - point_on_plane );
  return p - t * normal;
}                         


inline Vec3 projectOntoPlaneIntersection( const Vec3 &p, 
                            Vec3 p1,
                            Vec3 n1,
                            Vec3 p2,
                            Vec3 n2 ) {
  Vec3 lp, ld;
  if( !planeIntersect( p1, n1, p2, n2, lp, ld ) ) {
     return projectOntoPlane( p, p1, n1 ); 
  }

  return projectOntoLine( p, lp, lp + ld );
}



HapticForceEffect::EffectOutput 
GodObjectRenderer::renderHapticsOneStep( HAPIHapticsDevice *hd,
                                         const HapticShapeVector &shapes ) {
  // get the current device values
  HAPIHapticsDevice::DeviceValues input = hd->getDeviceValues();

  // clear all previous contacts
  tmp_contacts.clear();

  Vec3 proxy_pos = proxy_position;
  HapticForceEffect::EffectOutput output;
  bool has_intersection = false;
    
  HAPIFloat d2;
  Bounds::IntersectionInfo closest_intersection;
  
  // the intersections that are closest to the user (several if the 
  // intersections are in the same point)
  vector< PlaneConstraint > closest_constraints;

  // calculate the forces generated by the active haptic shapes
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       i++ ) {
    Bounds::IntersectionInfo intersection;
    if( (*i)->lineIntersect( proxy_pos, input.position, intersection,
                             (*i)->touchable_face) ){
      // shape is intersected, create a plane constraint at the
      // intersection point
      PlaneConstraint pc( intersection.point, 
                          intersection.face == Bounds::FRONT ?
                          intersection.normal : -intersection.normal,
                          *i );

      if( !has_intersection ) {
        // first shape that was intersected, and therefore also the
        // closest intersection
        closest_intersection = intersection;
        Vec3 v = intersection.point - proxy_pos;
        d2 = v * v;
        has_intersection = true;
        closest_constraints.push_back( pc  );
      } else {
        // this is not the first intersection, check if it closer than
        // previous ones
        Vec3 v = intersection.point - proxy_pos;
        HAPIFloat distance = v * v; 
      
        if( (closest_intersection.point - intersection.point).lengthSqr()
            < length_sqr_point_epsilon ) {
          // intersection point is the same as previous intersections,
          // check if the normal is the same as a previous untersection.
          // if it is the new intersection is discarded.
          bool unique_constraint = true;
          for( vector< PlaneConstraint >::iterator j = 
                 closest_constraints.begin();
               j != closest_constraints.end(); j++ ) {
            if( ( intersection.normal - (*j).normal ).lengthSqr() < 
                length_sqr_normal_epsilon ) {
              // discard new intersectsion
              unique_constraint = false;
            }
          }

          // only add the constraint to the closest_constraints vector
          // if it is an unique constraint.
          if( unique_constraint ) {
            closest_constraints.push_back( pc );
          } 
        } else {
          // intersection point is not the same as previous intersection
          if( distance < d2 ) {
            // new intersection is closer than old intersection
            closest_intersection = intersection;
            d2 = distance;
            closest_constraints.clear();
            closest_constraints.push_back( pc );
          } 
        }
      }
    } 
  }
   
  unsigned int nr_constraints = closest_constraints.size();

  Vec3 new_proxy_pos, new_force;
  // the minimum distance the proxy will be from the surface. The proxy
  // will be moved above the surface with the given factor in order to
  // avoid roundoff errors and fallthrough problems.
  HAPIFloat min_distance = 1e-1;

  HAPISurfaceObject::ContactInfo contact;

  // fill in contact information
  contact.contact_point_global = closest_intersection.point;
  contact.probe_position_global = input.position;
  contact.proxy_radius = 0;

  if( nr_constraints == 0 ) {
    // TODO: 0.05??
    new_proxy_pos = proxy_pos + (input.position-proxy_pos)*0.05;
  } else {
    if( nr_constraints == 1 ) {
      onOnePlaneContact( closest_constraints[0], contact );
    } else if( nr_constraints == 2 ) {
      onTwoPlaneContact( closest_constraints[0],
                         closest_constraints[1], contact );
    } if( nr_constraints >= 3 ) {
      onThreeOrMorePlaneContact( closest_constraints,
                                 contact );
    } 

    new_proxy_pos = 
      closest_intersection.point + 
      contact.y_axis * min_distance +
      contact.proxy_movement_local.x * contact.x_axis + 
      contact.proxy_movement_local.y * contact.z_axis;
    new_force = contact.force_global;
    
    if( nr_constraints < 3 ) {
      // try to move the proxy from proxy_pos -> new_proxy_pos and check for 
      // intersection with shapes. If intersection the new goal will be 
      // new_proxy_pos projected onto the plane created by intersection point
      // and its normal and the previous constraint planes.
      
      // the point from which to move
      Vec3 from_point = proxy_pos;
      bool have_new_goal = true;
      bool first_loop = true;

      while( have_new_goal ) {
        have_new_goal = false;
        
        /*
          bool done = false;
          int counter = 0;
          while( !done && counter < 25 ) {
          done = true;
          // make sure the proxy is above any constraints
          for( vector< Bounds::PlaneConstraint >::iterator i = 
          closest_constraints.begin();
          i != closest_constraints.end(); i++ ) {
          HAPIFloat d = (*i).normal * (new_proxy_pos - (*i).point );
          if( d < min_distance - Constants::epsilon) {
          //cerr << (*i).normal << " " << d << endl;
          new_proxy_pos = new_proxy_pos + (*i).normal * (-d+min_distance);
          done = false;
          }
          }
          counter++;
          }
          
          if( counter > 4 ) cerr << "C";*/
        
        // find the closest intersection
        has_intersection = false;
        for( HapticShapeVector::const_iterator i = shapes.begin();
             i != shapes.end();
             i++ ) {
          Bounds::IntersectionInfo intersection;
          if( (*i)->lineIntersect( from_point, new_proxy_pos, intersection,
                                   (*i)->touchable_face) ){
            if( !has_intersection ) {
              closest_intersection = intersection;
              Vec3 v = intersection.point - proxy_pos;
              d2 = v * v;
              has_intersection = true;
            } else {
              Vec3 v = intersection.point - proxy_pos;
              HAPIFloat distance = v * v; 
              if( distance < d2 ) {
                closest_intersection = intersection;
                d2 = distance;
              } 
            }
          } 
        }
        
        if( has_intersection ) {
          Vec3 normal = 
            closest_intersection.face == Bounds::FRONT ?
            closest_intersection.normal : -closest_intersection.normal;
          from_point = closest_intersection.point + normal * min_distance;
          
          if( nr_constraints == 1 && first_loop ) {
            // project onto plane intersection between intersection plane and
            // closest constraint
            from_point = 
              projectOntoPlaneIntersection( from_point, 
                           closest_intersection.point + 
                           normal * min_distance,
                           normal,
                           closest_constraints[0].point + 
                           closest_constraints[0].normal * min_distance,
                           closest_constraints[0].normal );
            
            new_proxy_pos = 
              projectOntoPlaneIntersection( new_proxy_pos, 
                           closest_intersection.point + normal * min_distance,
                           normal,
                           closest_constraints[0].point + 
                           closest_constraints[0].normal * min_distance,
                           closest_constraints[0].normal );
            have_new_goal = true;
          } else {
            Vec3 move = closest_intersection.point - from_point;
            HAPIFloat l = move.length();
            if( l >= min_distance + 1e-8) {
              move = move * ( (l-min_distance) / l );
              new_proxy_pos = from_point + move;
            } else {
              new_proxy_pos = from_point;
            }
          }
        }
        first_loop = false;
      }
    }
  }
  
  proxy_position = new_proxy_pos;
  output.force = new_force;
    
  contacts_lock.lock();
  contacts.swap( tmp_contacts );
  contacts_lock.unlock();

   // add the resulting force and torque to the rendered force.
    
  return output;
}
