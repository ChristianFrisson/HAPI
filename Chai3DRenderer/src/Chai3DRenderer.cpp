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
/// \file Chai3DRenderer.cpp
/// \brief cpp file for Chai3DRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/Chai3DRenderer.h>

#ifdef HAVE_CHAI3D

#include <H3DUtil/H3DMath.h>

#include <HAPI/HapticTriangleSet.h>
#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/HAPIFrictionSurface.h>

// Chai3D includes
#ifdef CHAI3D_VERSION_2_0
#include <math/CVector3d.h>
#include <math/CMatrix3d.h>
#else
#include <CVector3d.h>
#include <CMatrix3d.h>
#endif

//using namespace H3D;
using namespace HAPI;

#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#endif


HAPIHapticsRenderer::HapticsRendererRegistration 
Chai3DRenderer::renderer_registration(
                            "Chai3D",
                            &(newInstance< Chai3DRenderer >)
                            );

/// Initialize the renderer to be used with the given haptics device.
void Chai3DRenderer::initRenderer( HAPIHapticsDevice *hd ) {
  chai3d_tool->initialize();
  chai3d_tool->setRadius( 0.0025 );
  chai3d_tool->computeGlobalPositions();
  chai3d_tool->start();
  chai3d_tool->setForcesON();
  setProxyRadius( 0.0025 );
}

// Release all resources that has been used in the renderer for
// the given haptics device.
void Chai3DRenderer::releaseRenderer( HAPIHapticsDevice *hd ) {
  chai3d_tool->setForcesOFF();
  chai3d_tool->stop();
}

HAPIForceEffect::EffectOutput 
Chai3DRenderer::renderHapticsOneStep( 
                     HAPIHapticsDevice *hd,
                     const HapticShapeVector &shapes ) {

  H3DDevice *dev = static_cast< H3DDevice * >( chai3d_tool->getDevice() );
  dev->setNewValues( hd->getPosition(),
                     hd->getVelocity(),
                     hd->getOrientation(),
                     hd->getButtonStatus() );
  chai3d_tool->updatePose();
  mesh_change_lock.lock();
#ifdef CHAI3D_VERSION_2_0
  chai3d_tool->computeInteractionForces();
#else
  chai3d_tool->computeForces();
#endif

  cVector3d f = chai3d_tool->m_lastComputedGlobalForce;
  
  Contacts new_contacts;

  // set the contacts
  cProxyPointForceAlgo *p = chai3d_tool->getProxy();
  if( p ) {
    cTriangle *t0, *t1, *t2;
#ifdef CHAI3D_VERSION_2_0
    unsigned int nr_contacts = p->getNumContacts();
#else
    cVector3d cp = p->getContactPoint();
    unsigned int nr_contacts = p->getContacts( t0, t1, t2 );
#endif

    if( nr_contacts > 0 ) {
#ifdef CHAI3D_VERSION_2_0
      t0 = p->m_contactPoint0->m_triangle;
      cVector3d cp = p->m_contactPoint0->m_globalPos;
#endif
      HAPISurfaceObject::ContactInfo ci;
      
      // using normal for vertex0 as normal. should be ok since no smooth
      // haptics rendering is done.
      cVector3d n =  t0->getVertex0()->getNormal();
      ci.y_axis = Vec3( n.y, n.z, n.x );
      ci.contact_point_global = Vec3( cp.y, cp.z, cp.x );
      ci.force_global = Vec3( f.y, f.z, f.x );
      HAPIFloat dot_product = ci.force_global * ci.y_axis;
      if( H3DUtil::H3DAbs(dot_product) > Constants::epsilon 
          && dot_product < 0 )
        ci.y_axis = -ci.y_axis;
      std::map< cMesh *, HAPIHapticShape * >::iterator i = 
        mesh_map.find( t0->getParent() );
      if( i != mesh_map.end() ) {
        new_contacts.push_back( std::make_pair( (*i).second, ci ) );
      }

      if( nr_contacts > 1 ) {
#ifdef CHAI3D_VERSION_2_0
          t1 = p->m_contactPoint1->m_triangle;
          cp = p->m_contactPoint1->m_globalPos;
#endif
        HAPISurfaceObject::ContactInfo ci;
        cVector3d n =  t1->getVertex0()->getNormal();
        ci.y_axis = Vec3( n.y, n.z, n.x );
        ci.contact_point_global = Vec3( cp.y, cp.z, cp.x );
        ci.force_global = Vec3( f.y, f.z, f.x );
        std::map< cMesh *, HAPIHapticShape * >::iterator i = 
          mesh_map.find( t1->getParent() );
        if( i != mesh_map.end() && t1->getParent() != t0->getParent() ) {
          new_contacts.push_back( std::make_pair( (*i).second, ci ) );
        }

        if( nr_contacts > 2 ) {
#ifdef CHAI3D_VERSION_2_0
          t2 = p->m_contactPoint2->m_triangle;
          cp = p->m_contactPoint2->m_globalPos;
#endif
          HAPISurfaceObject::ContactInfo ci;
          cVector3d n =  t2->getVertex0()->getNormal();
          ci.y_axis = Vec3( n.y, n.z, n.x );
          ci.contact_point_global = Vec3( cp.y, cp.z, cp.x );
          ci.force_global = Vec3( f.y, f.z, f.x );
          std::map< cMesh *, HAPIHapticShape * >::iterator i = 
            mesh_map.find( t2->getParent() );
          if( i != mesh_map.end() && 
              t2->getParent() != t0->getParent() &&
              t2->getParent() != t1->getParent()) {
            new_contacts.push_back( std::make_pair( (*i).second, ci ) );
          }
        }
      }
    }
  }
  mesh_change_lock.unlock();

  contacts_lock.lock();
  contacts.swap( new_contacts );
  contacts_lock.unlock();

  return HAPIForceEffect::EffectOutput( Vec3( f.y, f.z, f.x ) );
}


void Chai3DRenderer::preProcessShapes( HAPIHapticsDevice *hd,
                                       const HapticShapeVector &shapes ) {
  std::map< cMesh *, HAPIHapticShape * > new_mesh_map;
  vector< cMesh * > new_meshes( shapes.size() );
  new_meshes.clear();

  for( HapticShapeVector::const_iterator s = shapes.begin();
       s != shapes.end(); ++s ) {
    
    HAPIHapticShape *shape = (*s);
    Chai3DSurface *chai3d_surface = 
      dynamic_cast< Chai3DSurface * >( shape->getSurface() );
    HAPIFrictionSurface *friction_surface = 
      dynamic_cast< HAPIFrictionSurface * >( shape->getSurface() );

    if( chai3d_surface || friction_surface ) {
      cMesh *mesh = new cMesh(world);
      
      new_meshes.push_back( mesh );
      new_mesh_map[mesh] = shape;

      HapticTriangleSet *tri_set = 
        dynamic_cast< HapticTriangleSet * >( shape );

      if( tri_set ) {
        Matrix4 transform = tri_set->getTransform();
        
        int index = 0;
        for( vector< Collision::Triangle >::iterator i = 
               tri_set->triangles.begin();
             i != tri_set->triangles.end(); ++i ) {
          Vec3 a = transform * (*i).a;
          Vec3 b = transform * (*i).b;
          Vec3 c = transform * (*i).c;
          mesh->newVertex( a.z, a.x, a.y );
          mesh->newVertex( b.z, b.x, b.y );
          mesh->newVertex( c.z, c.x, c.y );
          mesh->newTriangle(index,index+1,index+2);
          index += 3;
        }
        mesh->computeAllNormals();
        cMaterial mat;
        if( chai3d_surface )
          chai3d_surface->chai3dMaterial( mat );
        else if( friction_surface ) {
          if( friction_surface->use_relative_values )
            mat.setStiffness( friction_surface->stiffness *
                              hd->getMaxStiffness() );
          else
            mat.setStiffness( friction_surface->stiffness );
          mat.setStaticFriction( friction_surface->static_friction );
          mat.setDynamicFriction( friction_surface->dynamic_friction );
        }
        mesh->setMaterial( mat );
      }
      mesh->computeGlobalPositions();
    }
  } 

  mesh_change_lock.lock();
  world->clearAllChildren();
  meshes.clear();

  for( unsigned int i = 0; i < new_meshes.size(); ++i ) {
    world->addChild(new_meshes[i]);
  }
  meshes.swap( new_meshes );
  mesh_map.swap( new_mesh_map );
  mesh_change_lock.unlock();
}


HAPI::Vec3 Chai3DRenderer::getProxyPosition() {
  assert( chai3d_tool->getProxy() != NULL );
  cVector3d p = chai3d_tool->getProxy()->getProxyGlobalPosition();
  return Vec3( p.y, p.z, p.x );
}


int Chai3DRenderer::H3DDevice::command(int a_command, void* a_data)
{
    int result = CHAI_MSG_OK;

    if (m_systemReady) {
       switch (a_command)
       {
           // read position of phantom device
           case CHAI_CMD_GET_POS_3D: {
                cVector3d* pos = (cVector3d *) a_data;
                // Note: we're doing a change in reference here to
                //pass from the H3D convention to the CHAI one.
                pos->y = position.x;
                pos->z = position.y;
                pos->x = position.z;
           }
           break;

           // read normalized position of phantom device
           case CHAI_CMD_GET_POS_NORM_3D: {
                cVector3d* position = (cVector3d *) a_data;
                // Note: we're doing a change in reference here to pass from the phantom
                // convention to the CHAI one.
                //result = ReadNormalizedPositionPhantom(m_phantomHandle, position->y, position->z, position->x);
           }
           break;

           // read velocity of phantom device
           case CHAI_CMD_GET_VEL_3D: {
                cVector3d* vel = (cVector3d *) a_data;
                // Note: we're doing a change in reference here to pass from the H3D
                // convention to the CHAI one.
                vel->y = velocity.x;
                vel->z = velocity.y;
                vel->x = velocity.z;
           }
           break;

           // read orientation matrix of phantom
           case CHAI_CMD_GET_ROT_MATRIX: {
                cMatrix3d* mat3 = (cMatrix3d *) a_data;
                // Note: we're doing a change in reference here to pass from the H3D
                // convention to the CHAI one.
                Rotation r = orientation;
                r.axis.y = orientation.axis.x;
                r.axis.z = orientation.axis.y;
                r.axis.x = orientation.axis.z;
                Matrix3 m( r );
                mat3->set(m[0][0], m[0][1], m[0][2], 
                          m[1][0], m[1][1], m[1][2], 
                          m[2][0], m[2][1], m[2][2] );
           }
           break;

           // read user switch from phantom stylus
           case CHAI_CMD_GET_SWITCH_0: {
             int* switchstate = (int *) a_data;
             // Only care about button 0
             *switchstate = (buttons & 1)?1:0;
           }
           break;

           // read user switch from phantom stylus
           case CHAI_CMD_GET_SWITCH_1: {
             int* switchstate = (int *) a_data;
             // Only care about button 0
             *switchstate = (buttons & 2)?1:0;
           }
           break;

           // read all user switches from phantom stylus
           case CHAI_CMD_GET_SWITCH_MASK:
           {
             int* switchstate = (int *) a_data;
             *switchstate = buttons;
           }
           break;

           // read scale factor from normalized coords to mm
           case CHAI_CMD_GET_NORMALIZED_SCALE_FACTOR: {
           }
           break;

           // function is not implemented for phantom devices
           default:
                result = CHAI_MSG_NOT_IMPLEMENTED;
        }
    }
    else
    {
        result = CHAI_MSG_SYSTEM_NOT_READY;
    }

    return result;
}

#ifdef CHAI3D_VERSION_2_0
int Chai3DRenderer::H3DDevice::getPosition( cVector3d &a_position ) {
  a_position.y = position.x;
  a_position.z = position.y;
  a_position.x = position.z;
  return 0;
}

int Chai3DRenderer::H3DDevice::getLinearVelocity(
  cVector3d &a_linearVelocity ) {
  a_linearVelocity.y = velocity.x;
  a_linearVelocity.z = velocity.y;
  a_linearVelocity.x = velocity.z;
  return 0;
}

int Chai3DRenderer::H3DDevice::getRotation( cMatrix3d &a_rotation ) {
  Rotation r = orientation;
  r.axis.y = orientation.axis.x;
  r.axis.z = orientation.axis.y;
  r.axis.x = orientation.axis.z;
  Matrix3 m( r );
  a_rotation.set( m[0][0], m[0][1], m[0][2], 
                  m[1][0], m[1][1], m[1][2], 
                  m[2][0], m[2][1], m[2][2] );
  return 0;
}

int Chai3DRenderer::H3DDevice::getUserSwitch( int a_switchIndex,
                                              bool &a_status ) {
  if( a_switchIndex == 0 )
    // Only care about button 0
    a_status = (buttons & 1)?true:false;
  else if( a_switchIndex == 1 )
    a_status = (buttons & 2)?true:false;
  return 0;
}
#endif

#endif //HAVE_CHAI3D
