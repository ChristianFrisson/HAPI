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
/// \file RuspiniRenderer.cpp
/// \brief cpp file for RuspiniRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <RuspiniRenderer.h>
#include <H3DMath.h>
#include <HAPIHapticsDevice.h>
#include <PlaneConstraint.h>

using namespace HAPI;

const Vec3 UNINITIALIZED_PROXY_POS = Vec3( -200, -200, -202 );

HAPIHapticsRenderer::HapticsRendererRegistration 
RuspiniRenderer::renderer_registration(
                            "Ruspini",
                            &(newInstance< RuspiniRenderer >)
                            );

// epsilon value for deciding if a point is the same
const HAPIFloat length_sqr_point_epsilon = 1e-12; //12

// epsilon value for deciding if a normal is the same.
const HAPIFloat length_sqr_normal_epsilon = 1e-12;

RuspiniRenderer::RuspiniRenderer( HAPIFloat _proxy_radius ):
  proxy_radius( _proxy_radius ),
  proxy_position( UNINITIALIZED_PROXY_POS ) {
}

void RuspiniRenderer::onOnePlaneContact( const PlaneConstraint &c, 
                                    HAPISurfaceObject::ContactInfo &contact ) {
  // create a local coordinate system with the PlaneConstraint normal
  // as y-axis
  contact.y_axis = c.normal;
  Vec3 a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
  contact.x_axis = contact.y_axis % a;
  contact.z_axis = contact.x_axis % contact.y_axis;
  contact.x_axis.normalizeSafe();
  contact.z_axis.normalizeSafe();
  contact.setGlobalOrigin( contact.contact_point_global );
  assert( c.haptic_shape.get() );

  contact.primitive = c.primitive;
  c.haptic_shape->surface->getProxyMovement( contact );

  Vec3 new_proxy_pos =  
    contact.contact_point_global + 
    contact.proxy_movement_local.x * contact.x_axis + 
    contact.proxy_movement_local.y * contact.z_axis;

  contact.setGlobalOrigin(  tryProxyMovement( proxy_position, 
                                              new_proxy_pos, c.normal ) );
  
  // call the surface to determine forces and proxy movement
  c.haptic_shape->surface->getForces( contact );
  
  // add a contact
  tmp_contacts.push_back( make_pair( c.haptic_shape, contact ) );
}

void RuspiniRenderer::onTwoPlaneContact( const PlaneConstraint &p0,
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
    contact.setGlobalOrigin( contact.contact_point_global );

    Vec3 line_dir_local = contact.vectorToLocal( line_dir );

    assert( p0.haptic_shape.get() );

    contact.primitive = p0.primitive;
    // calculate the force and proxy movement for the first plane
    p0.haptic_shape->surface->getProxyMovement( contact );
    
    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p0_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
            0, 
            contact.proxy_movement_local.y ) *
      line_dir_local;

    assert( p1.haptic_shape.get() );

    contact.primitive = p1.primitive;
    // calculate the force and proxy movement for the second plane
    p1.haptic_shape->surface->getProxyMovement( contact );

    // constrain the proxy movement to the intersection between 
    // the two planes
    HAPIFloat p1_proxy_movement = 
      Vec3( contact.proxy_movement_local.x, 
             0, 
             contact.proxy_movement_local.y ) *
      line_dir_local;

    // calculate and set the final output parameters for force and 
    // proxy movement
    Vec3 proxy_movement = 
      H3DUtil::H3DMin( p0_proxy_movement, p1_proxy_movement ) * 
      line_dir_local;

    contact.proxy_movement_local = Vec2( proxy_movement.x, proxy_movement.z ); 

    Vec3 new_proxy_pos =  
      contact.contact_point_global + 
      contact.proxy_movement_local.x * contact.x_axis + 
      contact.proxy_movement_local.y * contact.z_axis;
    
    contact.setGlobalOrigin( tryProxyMovement( proxy_position, 
                                             new_proxy_pos, contact.y_axis ) );

    contact.primitive = p0.primitive;
    p0.haptic_shape->surface->getForces( contact );
    Vec3 p0_force = contact.force_global;
    contact.primitive = p1.primitive;
    p1.haptic_shape->surface->getForces( contact );
    Vec3 p1_force = contact.force_global;

    contact.force_global = p0_force * weight + p1_force * ( 1 - weight );

    // add contacts
    tmp_contacts.push_back( make_pair( p0.haptic_shape.get(), contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() )
      tmp_contacts.push_back( make_pair( p1.haptic_shape.get(), contact ) );
  }
}


void RuspiniRenderer::onThreeOrMorePlaneContact(  
          Constraints &constraints,
          HAPISurfaceObject::ContactInfo &contact ) {
  assert( constraints.size() >= 3 );

  // find the first three planes that all constrain the proxy
  Constraints::iterator i = constraints.begin();
  PlaneConstraint &p0 = (*i++);
  PlaneConstraint &p1 = (*i++);
  PlaneConstraint &p2 = (*i++);

  Vec3 contact_global = contact.globalContactPoint();
  Vec3 probe_local_pos;

  if( i == constraints.end() ) {
    // transformation matrix from local coordinate system with the normals
    // of the constraint planes as axis to global space
    Matrix4 m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
               p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
               p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
               0, 0, 0, 1 );
    Matrix4 m_inv = m.inverse();
    probe_local_pos = m_inv * contact.globalProbePosition();
  } else {
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
    contact.z_axis.normalizeSafe();
    contact.setGlobalOrigin( contact.contact_point_global );

    // calculate the force and proxy movement for the first plane
    assert( p0.haptic_shape.get() );
    p0.haptic_shape->surface->getForces( contact );
    Vec3 p0_force = contact.force_global;
    
    // calculate the force and proxy movement for the second plane
    assert( p1.haptic_shape.get() );
    p1.haptic_shape->surface->getForces( contact );
    Vec3 p1_force = contact.force_global;

    // calculate the force and proxy movement for the third plane
    assert( p2.haptic_shape.get() );
    p2.haptic_shape->surface->getForces( contact );
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
    tmp_contacts.push_back( make_pair( p0.haptic_shape.get(), contact ) );
    if( p0.haptic_shape.get() != p1.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p1.haptic_shape.get(), contact ) );
    }
    
    if( p0.haptic_shape.get() != p2.haptic_shape.get() &&
        p1.haptic_shape.get() != p2.haptic_shape.get() ) {
      tmp_contacts.push_back( make_pair( p2.haptic_shape.get(), contact ) );
    }
  }    
}




HAPIForceEffect::EffectOutput 
RuspiniRenderer::renderHapticsOneStep( HAPIHapticsDevice *hd,
                                       const HapticShapeVector &shapes ) {
  HAPIHapticsDevice::DeviceValues input = hd->getDeviceValues();

  if( proxy_position == UNINITIALIZED_PROXY_POS ) {
    proxy_position = input.position;
  }

  // clear all previous contacts
  tmp_contacts.clear();
  Vec3 proxy_pos = proxy_position;
  HAPIForceEffect::EffectOutput output;
  bool has_intersection = false;
  HAPIFloat d2;
  Collision::IntersectionInfo closest_intersection;

  constraints.clear();  
  //constraints.reserve( 3000 );
  closest_constraints.clear();
  other_constraints.clear();

  // only get the planes that are within the distance the proxy can move
  HAPIFloat r = (proxy_radius * 2 + 
                 (input.position - proxy_position).length() ) * 1.1;

  //  vector< Collision::PlaneConstraint > constraints;
  // get the constraints from the current shapes.
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       i++ ) {
    (*i)->getConstraints( proxy_pos, constraints, (*i)->touchable_face, r );
  }
  
  // move them out by the proxy radius in the direction of the normal. 
  for( Constraints::iterator i = constraints.begin();
       i != constraints.end(); i++ ) {
    (*i).point += (*i).normal * proxy_radius;
  }

  
  // make sure the proxy is above any constraints
  bool done = false;
  int counter = 0;
  while( !done && counter < 25 ) {
    done = true;
    for( Constraints::iterator i = constraints.begin();
         i != constraints.end(); i++ ) {
      HAPIFloat d = (*i).normal * (proxy_pos - (*i).point );
      if( d < 0 && d > -proxy_radius ) {
        //cerr << (*i).normal << " " << d << endl;
        proxy_pos = proxy_pos + (*i).normal * (-d+1e-15);
        done = false;
      }
    }
    counter++;
  }

  proxy_position = proxy_pos;

  // the constraints which contraint point is closest to the proxy.
  // if more than one the constraint point is the same


  // the constraints not in closest_constraints

//  other_constraints.reserve( constraints.size() );

  Collision::IntersectionInfo intersection;

  // find the closest constraining PlaneConstraints
  for( Constraints::iterator i = constraints.begin();
       i != constraints.end(); i++ ) {
    if( (*i).lineIntersect( proxy_pos, input.position, intersection ) ) {

      if( !has_intersection ) {
        // this is the first intersected plane, so it is also the closest
        closest_intersection = intersection;
        Vec3 v = intersection.point - proxy_pos;
        d2 = v * v;
        has_intersection = true;
        closest_constraints.push_back( *i );
      } else {
        // the is a previously intersected plane, check which is closest
        Vec3 v = intersection.point - proxy_pos;
        HAPIFloat distance = v * v; 
      
        if( (closest_intersection.point - intersection.point).lengthSqr()
            < length_sqr_point_epsilon ) {
          // intersection point is the same as for previosly intersected 
          // plane

          // check if same normal, if so the new plane should be ignored
          // since it is the same as a previous one.
          bool unique_constraint = true;
          for( Constraints::iterator j = 
                 closest_constraints.begin();
               j != closest_constraints.end(); j++ ) {
            if( ( intersection.normal - (*j).normal ).lengthSqr() < 
                length_sqr_normal_epsilon ) {
              // same normal, ignore plane
              unique_constraint = false;
            }
          }

          // only add the constraint to the closest_constraints vector
          // if it is an unique constraint.
          if( unique_constraint ) {
            closest_constraints.push_back( *i );
          } 
        } else if( distance < d2 ) {
          // intersection point is closer than for previous planes,
          // update closest intersection info.
          closest_intersection = intersection;
          d2 = distance;
          other_constraints.insert( other_constraints.end(),
                                          closest_constraints.begin(),
                                          closest_constraints.end() );
          closest_constraints.clear();
          closest_constraints.push_back( *i );
        } else {
          // intersection is further away than previous intersections. 
          other_constraints.push_back( *i );
        }
      }
    } else {
      // plane is not intersected
      other_constraints.push_back( *i );
    }
  } 

  unsigned int nr_constraints = closest_constraints.size();

  Vec3 new_proxy_pos, new_force;

  HAPISurfaceObject::ContactInfo contact;
  contact.hd = hd;

  contact.contact_point_global = closest_intersection.point;
  contact.probe_position_global = input.position;
  contact.proxy_radius = proxy_radius;
  contact.tex_coord = closest_intersection.tex_coord;

  // calculate wanted new proxy position based on the constraint planes
  // and their surfaces
  if( nr_constraints == 0 ) {
    new_proxy_pos = proxy_pos + (input.position-proxy_pos)*0.05;
  } else {
    if( nr_constraints == 1 ) {
      onOnePlaneContact( closest_constraints.front(), contact );
    } else if( nr_constraints == 2 ) {
      onTwoPlaneContact( closest_constraints.front(),
                         *(closest_constraints.begin() + 1), contact );
    } if( nr_constraints >= 3 ) {
      onThreeOrMorePlaneContact( closest_constraints,
                                 contact );
    } 

    new_proxy_pos = contact.globalOrigin();

    new_force = contact.force_global;
  }
    

      
  proxy_position = new_proxy_pos;
  output.force = new_force;
    
  contacts_lock.lock();
  contacts.swap( tmp_contacts );
  contacts_lock.unlock();

  return output;
}


Vec3 RuspiniRenderer::tryProxyMovement( Vec3 from, Vec3 to, Vec3 normal ) {
  // from = closest_intersection.point;
  // to = new_proxy_pos

  bool has_intersection = false;
  Collision::IntersectionInfo intersection;
  Vec3 closest_point;
  HAPIFloat d2;

  // try to move the proxy from proxy_pos -> new_proxy_pos and check for 
  // intersection with the other plane constraints. 
  Constraints::iterator inter_i;
  for( Constraints::iterator i = 
         other_constraints.begin();
       i != other_constraints.end(); i++ ) {
    
    Vec3 from_point = from;
    Vec3 to_point = to;
#if 0
    // since a constraint at a triangle edge might be unwanted if it e.g. has
    // a neighbouring triangle in the same plane, we make sure that this is not
    // the case by updating the constraint plane until the normal does not change/*
    if( (*i).lineIntersect( from_point, 
                            to_point, intersection ) ) {
      const Matrix4 &transform = (*i).haptic_shape->transform;
      const Matrix4 &inv = transform.inverse();
      Vec3 s = inv.getScalePart();
      HAPIFloat scale = max( s.x, max( s.y, s.z ) ); 

      Vec3 f = from_point;/* + normal * 1e-3;*/
      Vec3 t = to_point;/* + normal * 1e-3;*/

      if( true || (*i).primitive->movingSphereIntersect( proxy_radius * scale,
                                                 inv * f, 
                                                 inv * t ) ) {
        
        if( !has_intersection ) {
          // first intersection, so closest
          closest_point = /*transform * */ intersection.point;
          Vec3 v = closest_point - from;
          d2 = v * v;
          has_intersection = true;
        } else {
          // check if the new intersection is closer than the closest of the
          // previous
          Vec3 p = /* transform * */ intersection.point;
          Vec3 v = p - from;
          HAPIFloat distance = v * v;
          if( distance < d2 ) {
            closest_point = p;
            d2 = distance;
          }
        }
      }
    }
  } 

#else
    bool done = false;
    unsigned int counter = 0;
    bool plane_intersected = false;
    while( !done && counter < 5 && 
           (*i).lineIntersect( from_point, 
                               to_point, intersection ) ) {
      plane_intersected = true;
      constraints.clear();
      PlaneConstraint pc = *i;
      Vec3 p = intersection.point;
      // get updated constraint
      (*i).primitive->getConstraints( p, constraints, 
                                      (*i).haptic_shape->touchable_face );
      if( !constraints.empty() )
        *i = constraints.front();

      (*i).point += (*i).normal * proxy_radius;
      (*i).haptic_shape = pc.haptic_shape;
      
      // if normal has not changed, stop updating constraint
      if( ( 1 - pc.normal * (*i).normal ) < 1e-8 ) {
        done = true;
      } else {
        from_point = intersection.point;

        // make sure the point is above the new constraint
        HAPIFloat d = (*i).normal * (from_point - (*i).point );
        if( d < 0 ) {
          from_point = from_point + (*i).normal * (-d+1e-15);
        } 
      }
      counter++;
    }

    
    if( (plane_intersected && counter >= 5 ) || done ) {
      if( !has_intersection ) {
        // first intersection, so closest
        closest_point = intersection.point;
        Vec3 v = intersection.point - from;
        d2 = v * v;
        has_intersection = true;
      } else {
        // check if the new intersection is closer than the closest of the
        // previous
        Vec3 v = intersection.point - from;
        HAPIFloat distance = v * v;
        if( distance < d2 ) {
          closest_point = intersection.point;
          d2 = distance;
        }
      }
    } 
  }
#endif
  if( has_intersection ) return closest_point;
  else return to;
}
