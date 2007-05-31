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
/// \file Chai3DRenderer.cpp
/// \brief cpp file for Chai3DRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <Chai3DRenderer.h>

#include <H3DMath.h>
//#include "DeviceInfo.h"

#include <HapticTriangleSet.h>
#include <HAPIHapticsDevice.h>
#include <FrictionSurface.h>

// Chai3D includes
#include <cVector3d.h>
#include <cMatrix3d.h>

//using namespace H3D;
using namespace HAPI;

HAPIHapticsRenderer::HapticsRendererRegistration 
Chai3DRenderer::renderer_registration(
                            "Chai3D",
                            &(newInstance< Chai3DRenderer >)
                            );

/// Initialize the renderer to be used with the given haptics device.
void Chai3DRenderer::initRenderer( HAPIHapticsDevice *hd ) {
  chai3d_tool->initialize();
  chai3d_tool->setRadius( 0.05 );
  chai3d_tool->computeGlobalPositions();
  chai3d_tool->start();
  chai3d_tool->setForcesON();
  setProxyRadius( 0.5 );
}

/// Release all resources that has been used in the renderer for
/// the given haptics device.
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
  chai3d_tool->computeForces();
  mesh_change_lock.unlock();
  cVector3d f = chai3d_tool->m_lastComputedGlobalForce;
  return HAPIForceEffect::EffectOutput( Vec3( f.y, f.z, f.x ) );
}


void Chai3DRenderer::preProcessShapes( HAPIHapticsDevice *hd,
                                       const HapticShapeVector &shapes ) {
  
  vector< cMesh * > new_meshes( shapes.size() );
  new_meshes.clear();

  for( HapticShapeVector::const_iterator s = shapes.begin();
       s != shapes.end(); s++ ) {
    
    HAPIHapticShape *shape = (*s);
    Chai3DSurface *chai3d_surface = 
      dynamic_cast< Chai3DSurface * >( shape->surface );
    FrictionSurface *friction_surface = 
      dynamic_cast< FrictionSurface * >( shape->surface );

    if( chai3d_surface || friction_surface ) {
      cMesh *mesh = new cMesh(world);
      
      new_meshes.push_back( mesh );
      
      // TODO: fix for other shape types
      HapticTriangleSet *tri_set = 
        dynamic_cast< HapticTriangleSet * >( shape );
      
      if( tri_set ) {
        
        int index = 0;
        for( vector< Bounds::Triangle >::iterator i = 
               tri_set->triangles.begin();
             i != tri_set->triangles.end(); i++ ) {
          Vec3 a = tri_set->transform * (*i).a;
          Vec3 b = tri_set->transform * (*i).b;
          Vec3 c = tri_set->transform * (*i).c;
          mesh->newVertex( a.z, a.x, a.y );
          mesh->newVertex( b.z, b.x, b.y );
          mesh->newVertex( c.z, c.x, c.y );
          mesh->newTriangle(index,index+1,index+2);
          index += 3;
        }
        cMaterial mat;
        if( chai3d_surface )
          chai3d_surface->chai3dMaterial( mat );
        else if( friction_surface ) {
          mat.setStiffness( friction_surface->stiffness / 1000 );
          mat.setStaticFriction( friction_surface->static_friction );
          mat.setDynamicFriction( friction_surface->dynamic_friction );
        }
        mesh->setMaterial( mat );
      }
    }
  } 

  mesh_change_lock.lock();
  world->clearAllChildren();
  meshes.clear();

  for( unsigned int i = 0; i < new_meshes.size(); i++ ) {
    world->addChild(new_meshes[i]);
  }
  meshes.swap( new_meshes );
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
/*
           // set force to phantom device
           case CHAI_CMD_SET_FORCE_3D:
           {
             // TODO:
                //cVector3d *force;
                //force = (cVector3d *) a_data;
                //result = SetForcePhantom(m_phantomHandle, force->y, force->z, force->x);
           }
           break;

           // set force and torque to phantom stylus
           case CHAI_CMD_SET_FORCE_TORQUE_3D:
           {
             // TODO:
             //   cVector3d *genforce;
             //   genforce = (cVector3d *) a_data;
             //   result = SetForceTorquePhantom(m_phantomHandle,
             //                genforce[0].y, genforce[0].z, genforce[0].x,
              //               genforce[1].y, genforce[1].z, genforce[1].x);
           }
           break;
*/
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
             // TODO:
             //  double* scale = (double*)a_data;
             //  result = GetWorkspaceScalePhantom(m_phantomHandle,*scale);               
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
