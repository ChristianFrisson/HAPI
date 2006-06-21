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
/// \file RuspiniRenderer.cpp
/// \brief cpp file for RuspiniRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <RuspiniRenderer.h>

using namespace H3D;

inline void onOnePlaneContact( const PlaneConstraint &c, 
                               HAPISurfaceObject::ContactInfo &contact ) {
  //cerr << "1";
  contact.y_axis = c.normal;
  Vec3d a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
  contact.x_axis = contact.y_axis % a;
  contact.z_axis = contact.x_axis % contact.y_axis;
  assert( c.haptic_shape );
  c.haptic_shape->surface->onContact( contact );
}

inline void onTwoPlaneContact( const PlaneConstraint &p0,
                               const PlaneConstraint &p1,
                               HAPISurfaceObject::ContactInfo &contact ) {
  Vec3d contact_global = contact.globalContactPoint();

  Vec3d line_dir = p0.normal % p1.normal;
  line_dir.normalizeSafe();
  Matrix4d m( p0.normal.x, p1.normal.x, line_dir.x, contact_global.x, 
              p0.normal.y, p1.normal.y, line_dir.y, contact_global.y, 
              p0.normal.z, p1.normal.z, line_dir.z, contact_global.z,
              0, 0, 0, 1 );
  Matrix4d m_inv = m.inverse();
  
  Vec3d local_pos = m_inv * contact.globalProbePosition();

  if( local_pos.x > Constants::f_epsilon ) {
    onOnePlaneContact( p1, contact );
  } else if( local_pos.y > Constants::f_epsilon ) {
    onOnePlaneContact( p0, contact );
  } else {
    //cerr << "2";
    H3DDouble sum = local_pos.x + local_pos.y;
    
    H3DDouble weight = 0;
    if( sum < 0 )
      weight = local_pos.x / sum;
    
    contact.y_axis = p0.normal * weight + p1.normal * (1 - weight );
    contact.y_axis.normalizeSafe();
    
    Vec3d a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;
    contact.z_axis.normalizeSafe();
    contact.x_axis.normalizeSafe();
    
    Vec3d line_dir_local = contact.vectorToLocal( line_dir );

    Matrix3d mm = Matrix3d( contact.x_axis.x, contact.y_axis.x,  contact.z_axis.x,
                 contact.x_axis.y,  contact.y_axis.y,  contact.z_axis.y,
                 contact.x_axis.z,  contact.y_axis.z,  contact.z_axis.z ).inverse();
    Vec3d fff = mm * line_dir;

    assert( p0.haptic_shape );
    p0.haptic_shape->surface->onContact( contact );
    H3DDouble p0_proxy_movement = 
      Vec3d( contact.proxy_movement_local.x, 
             0, 
             contact.proxy_movement_local.y ) *
      line_dir_local;

    Vec3d p0_force = contact.force_global;
    
    assert( p1.haptic_shape );
    p1.haptic_shape->surface->onContact( contact );
    H3DDouble p1_proxy_movement = 
      Vec3d( contact.proxy_movement_local.x, 
             0, 
             contact.proxy_movement_local.y ) *
      line_dir_local;

    Vec3d p1_force = contact.force_global;
    
    Vec3d proxy_movement = 
      H3DMin( p0_proxy_movement, p1_proxy_movement ) * 
      line_dir_local;
    if( p0_proxy_movement > 0.1 )
      cerr << p0_proxy_movement << endl;
    contact.proxy_movement_local = Vec2d( proxy_movement.x, proxy_movement.z ); 
      
    
    //cerr << p0_proxy_movement.x << endl;
    contact.force_global = p0_force * weight + p1_force * ( 1 - weight );
  }
}


inline void onThreeOrMorePlaneContact(  vector< PlaneConstraint > &constraints,
                                        HAPISurfaceObject::ContactInfo &contact ) {
  vector< PlaneConstraint >::iterator i = constraints.begin();
  PlaneConstraint &p0 = (*i++);
  PlaneConstraint &p1 = (*i++);
  PlaneConstraint &p2 = (*i++);

  Vec3d contact_global = contact.globalContactPoint();

  while( i != constraints.end() ) {
    Matrix4d m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
                p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
                p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
                0, 0, 0, 1 );
    Matrix4d m_inv = m.inverse();
    Vec3d local_pos = m_inv * contact.globalProbePosition();
    
    if( local_pos.x > Constants::f_epsilon ) {
      std::swap( p0, *i++ ); 
    } else if( local_pos.y > Constants::f_epsilon ) {
      std::swap( p1, *i++ ); 
    } else if( local_pos.z > Constants::f_epsilon ) {
      std::swap( p2, *i++ );
    } else {
      break;
    }
  }


  Matrix4d m( p0.normal.x, p1.normal.x, p2.normal.x, contact_global.x, 
              p0.normal.y, p1.normal.y, p2.normal.y, contact_global.y, 
                p0.normal.z, p1.normal.z, p2.normal.z, contact_global.z,
              0, 0, 0, 1 );
  
  Matrix4d m_inv = m.inverse();
  
  Vec3d local_pos = m_inv * contact.globalProbePosition();
  
  if( local_pos.x > Constants::f_epsilon ) {
    onTwoPlaneContact( p1, p2, contact );
  } else if( local_pos.y > Constants::f_epsilon ) {
    onTwoPlaneContact( p0, p2, contact );
  } else if( local_pos.z > Constants::f_epsilon ) {
    onTwoPlaneContact( p0, p1, contact );
  } else {
    //cerr << "3";
    H3DDouble sum = local_pos.x + local_pos.y + local_pos.z;
    
    Vec3d weight;
    if( sum < 0 )
      weight = local_pos / sum;

    contact.y_axis = 
      p0.normal * weight.x + 
      p1.normal * weight.y + 
      p2.normal * weight.z;
    
    Vec3d a( contact.y_axis.z, contact.y_axis.x, contact.y_axis.y );
    contact.x_axis = contact.y_axis % a;
    contact.z_axis = contact.x_axis % contact.y_axis;

    contact.x_axis.normalizeSafe();
    contact.y_axis.normalizeSafe();
    contact.z_axis.normalizeSafe();

    assert( p0.haptic_shape );
    p0.haptic_shape->surface->onContact( contact );
    Vec3d p0_force = contact.force_global;
    
    assert( p1.haptic_shape );
    p1.haptic_shape->surface->onContact( contact );
    Vec3d p1_force = contact.force_global;

    assert( p2.haptic_shape );
    p2.haptic_shape->surface->onContact( contact );
    Vec3d p2_force = contact.force_global;
    
    contact.proxy_movement_local = Vec2d( 0, 0 ); 
    
    contact.force_global = 
      p0_force * weight.x + 
      p1_force * weight.y + 
      p2_force * weight.z;
  }
}




HapticForceEffect::EffectOutput RuspiniRenderer::renderHapticsOneStep( HapticForceEffect::EffectInput input,
                                                                       const HapticShapeVector &shapes ) {
 
 Vec3d proxy_pos = proxy_position;
  HapticForceEffect::EffectOutput output;
  bool has_intersection = false;
  H3DDouble d2;
  Bounds::IntersectionInfo closest_intersection;
  vector< Bounds::PlaneConstraint > constraints;
  
  // calculate the forces generated by the active haptic shapes
  
  // get the constraints from the current shapes.
  for( HapticShapeVector::const_iterator i = shapes.begin();
       i != shapes.end();
       i++ ) {
    (*i)->getConstraints( proxy_pos, 0.1, constraints );
  }
  
  H3DDouble proxy_radius = 0.01;
  
  bool done = false;
  int counter = 0;
  while( !done && counter < 25 ) {
    done = true;
    // make sure the proxy is above any constraints
    for( vector< Bounds::PlaneConstraint >::iterator i = constraints.begin();
         i != constraints.end(); i++ ) {
      H3DDouble d = (*i).normal * (proxy_pos - (*i).point );
      if( d < 0 && d > -proxy_radius ) {
        //cerr << (*i).normal << " " << d << endl;
        proxy_pos = proxy_pos + (*i).normal * (-d+1e-15);
        done = false;
      }
    }
    counter++;
      
  }

  vector< PlaneConstraint > intersected_constraints;
  vector< PlaneConstraint > closest_constraints;

  for( vector< Bounds::PlaneConstraint >::iterator i = constraints.begin();
       i != constraints.end(); i++ ) {
    Bounds::IntersectionInfo intersection;
    Vec3d vv = input.position -  proxy_pos;
    vv.normalizeSafe();
    if( (*i).lineIntersect( proxy_pos, input.position, intersection ) ) {
      if( !has_intersection ) {
        closest_intersection = intersection;
        Vec3d v = intersection.point - proxy_pos;
        d2 = v * v;
        has_intersection = true;
        closest_constraints.push_back( *i );
      } else {
        Vec3d v = intersection.point - proxy_pos;
        H3DDouble distance = v * v; 
      
        if( (closest_intersection.point - intersection.point).lengthSqr()
            < 1e-12 ) {
          bool unique_constraint = true;
          for( vector< Bounds::PlaneConstraint >::iterator j = 
                 closest_constraints.begin();
               j != closest_constraints.end(); j++ ) {
            if( ( intersection.normal - (*j).normal ).lengthSqr() < 
                1e-12 ) {
              unique_constraint = false;
            }
          }

          // only add the constraint to the closest_constraints vector
          // if it is an unique constraint.
          if( unique_constraint ) {
            closest_constraints.push_back( *i );
          } else {
            intersected_constraints.push_back( *i );
          }
        } else if( distance < d2 ) {
          closest_intersection = intersection;
          d2 = distance;
          intersected_constraints.insert( intersected_constraints.end(),
                                          closest_constraints.begin(),
                                          closest_constraints.end() );
          closest_constraints.clear();
          closest_constraints.push_back( *i );
        } else {
          intersected_constraints.push_back( *i );
        }
      }
    } else {
      intersected_constraints.push_back( *i );
    }
  } 

  unsigned int nr_constraints = closest_constraints.size();

  Vec3d new_proxy_pos, new_force;

  HAPISurfaceObject::ContactInfo contact;

  contact.contact_point_global = closest_intersection.point;
  contact.probe_position_global = input.position;

  //cerr << nr_constraints << endl;

  if( nr_constraints == 0 ) {
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
      contact.proxy_movement_local.x * contact.x_axis + 
      contact.proxy_movement_local.y * contact.z_axis;
    new_force = contact.force_global;
  }
    
  has_intersection = false;
  Vec3d closest_point;
  for( vector< Bounds::PlaneConstraint >::iterator i = 
         intersected_constraints.begin();
       i != intersected_constraints.end(); i++ ) {
      
    Bounds::IntersectionInfo intersection;
      
    Vec3d vv = new_proxy_pos - proxy_pos;
    vv.normalizeSafe();
    if( (*i).lineIntersect( closest_intersection.point, 
                            new_proxy_pos, intersection ) ) {
      if( !has_intersection ) {
        closest_point = intersection.point;
        Vec3d v = intersection.point - proxy_pos;
        d2 = v * v;
        has_intersection = true;
      } else {
        Vec3d v = intersection.point - proxy_pos;
        H3DDouble distance = v * v;
        if( distance < d2 ) {
          closest_point = intersection.point;
          d2 = distance;
        }
      }
    } 
  }
    
  if( has_intersection ) {
    //           cerr << proxy_pos << endl;
    //           cerr << new_proxy_pos << endl;
    new_proxy_pos = closest_point;
    //cerr << new_proxy_pos << " " << pos << endl;
  }

  proxy_position = new_proxy_pos;
  output.force = new_force;
    
   // add the resulting force and torque to the rendered force.
    
  return output;
}
