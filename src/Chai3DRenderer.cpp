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

#include "Chai3DRenderer.h"
#include "H3DMath.h"
#include "DeviceInfo.h"

#include "HapticTriangleSet.h"

// Chai3D includes
#include <cVector3d.h>
#include <cMatrix3d.h>

using namespace H3D;
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
  cProxyPointForceAlgo *p = chai3d_tool->getProxy();
  if( p )
    p->setProxyRadius( 0.5 );
}

/// Release all resources that has been used in the renderer for
/// the given haptics device.
void Chai3DRenderer::releaseRenderer( HAPIHapticsDevice *hd ) {
  chai3d_tool->setForcesOFF();
  chai3d_tool->stop();
}

HapticForceEffect::EffectOutput 
Chai3DRenderer::renderHapticsOneStep( 
                     HAPIHapticsDevice *hd,
                     const HapticShapeVector &shapes ) {
  H3DDevice *dev = static_cast< H3DDevice * >( chai3d_tool->getDevice() );
  dev->setNewValues( hd->getPosition(),
                     hd->getVelocity(),
                     hd->getOrientation(),
                     hd->getButtonStatus() );
  chai3d_tool->updatePose();
  chai3d_tool->computeForces();
  cVector3d f = chai3d_tool->m_lastComputedGlobalForce;
  return HapticForceEffect::EffectOutput( Vec3( f.y, f.z, f.x ) );
}


void Chai3DRenderer::preProcessShapes( HAPIHapticsDevice *hd,
                                       const HapticShapeVector &shapes ) {
  if( shapes.size() > 0 ) {
    HAPIHapticShape *s = shapes[0];
    cMesh *mesh = NULL;
    if( meshes.size() == 0  ) {
      mesh = new cMesh(world);
      world->addChild(mesh);
      meshes.push_back( mesh );
      
      HapticTriangleSet *tri_set = dynamic_cast< HapticTriangleSet * >( s );
      
      if( tri_set ) {
        int index = 0;
        for( vector< Bounds::Triangle >::iterator i = tri_set->triangles.begin();
             i != tri_set->triangles.end(); i++ ) {
          //Vec3 a = (*i).a;
          //          Vec3 b =(*i).b;
          //Vec3 c =  (*i).c;
          Vec3 a = tri_set->transform * (*i).a;
          Vec3 b = tri_set->transform * (*i).b;
          Vec3 c = tri_set->transform * (*i).c;
          mesh->newVertex( a.z, a.x, a.y );
          mesh->newVertex( b.z, b.x, b.y );
          mesh->newVertex( c.z, c.x, c.y );
          mesh->newTriangle(index,index+1,index+2);
          index += 3;
        }
        cMaterial material;
        material.setStiffness(0.050);
        mesh->m_material = material;
      }
    } else {
      mesh = meshes[0];
      const Matrix4 &m = s->transform;
      Rotation r = (Rotation) m.getRotationPart();
      Matrix3 mm( m[2][2], m[2][0], m[2][1],
                  m[0][2], m[0][0], m[0][1],
                  m[1][2], m[1][0], m[1][1] );
      //cerr << mm << endl << endl;
      //cerr << mm.inverse() << endl << endl;
      Rotation r2( Matrix3( m[2][2], m[2][0], m[2][1],
              m[0][2], m[0][0], m[0][1],
              m[1][2], m[1][0], m[1][1] ) );
      cMatrix3d rm;
      rm.set( m[2][2], m[2][0], m[2][1],
              m[0][2], m[0][0], m[0][1],
              m[1][2], m[1][0], m[1][1] );
      //cerr << r * Vec3( 1,2,3) << endl;
      //cerr << r2 * Vec3( 3,1,2) << endl;
      cVector3d v( 3, 1, 2 );
      rm.mul( v );
      //mesh->setPos( m[2][3], m[0][3], m[1][3] );
      //mesh->setRot( rm );
    }
  }
}


HAPI::Vec3 Chai3DRenderer::getProxyPosition() {
  assert( chai3d_tool->getProxy() != NULL );
  cVector3d p = chai3d_tool->getProxy()->getProxyGlobalPosition();
 // cerr << Vec3( p.y, p.z, p.x ) << endl;
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
