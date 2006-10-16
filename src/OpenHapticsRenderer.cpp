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
/// \file OpenHapticsRenderer.cpp
/// \brief cpp file for OpenHapticsRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "OpenHapticsRenderer.h"
#include "H3DMath.h"
#include <HD/hd.h>
#include "HLShape.h"
#include "PhantomHapticsDevice.h"
#include "DeviceInfo.h"

using namespace H3D;
using namespace HAPI;

HAPIHapticsRenderer::HapticsRendererRegistration 
OpenHapticsRenderer::renderer_registration(
                            "OpenHaptics",
                            &(newInstance< OpenHapticsRenderer >)
                            );

/// Initialize the renderer to be used with the given haptics device.
void OpenHapticsRenderer::initRenderer( HAPIHapticsDevice *hd ) {
  HHLRC haptic_context = initHLLayer( hd );
  if( haptic_context ) {
    context_map[ hd ] = haptic_context;
  }
}

/// Release all resources that has been used in the renderer for
/// the given haptics device.
void OpenHapticsRenderer::releaseRenderer( HAPIHapticsDevice *hd ) {
  ContextMap::iterator i = context_map.find( hd );
  if( i != context_map.end() ) {
    hlMakeCurrent( NULL );
    hlDeleteContext( context_map[ hd ] );
    context_map.erase( i );
  }
}

HapticForceEffect::EffectOutput 
OpenHapticsRenderer::renderHapticsOneStep( 
                     HAPIHapticsDevice *hd,
                     const HapticShapeVector &shapes ) {
  PhantomHapticsDevice *pd = dynamic_cast< PhantomHapticsDevice * >( hd );
  if( !pd ) return HapticForceEffect::EffectOutput();

  hdMakeCurrentDevice( pd->getDeviceHandle() );

  // add the resulting force and torque to the rendered force.
  HDdouble force[3];
  hdGetDoublev( HD_CURRENT_FORCE, force );
  
  HDdouble torque[3];
  hdGetDoublev( HD_CURRENT_TORQUE, torque );

  // add the resulting force and torque to the rendered force.
    
  return HapticForceEffect::EffectOutput( Vec3( force[0], force[1], force[2] ), 
                                          Vec3( torque[0], torque[1], torque[2] ) );
}


void OpenHapticsRenderer::preProcessShapes( HAPIHapticsDevice *hd,
                                            const HapticShapeVector &shapes ) {
  // todo: Move to where it can be used
  //HLShape::resetShapeIdDB();
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glLoadIdentity();
  glClear(GL_DEPTH_BUFFER_BIT);   
  hlMatrixMode( HL_VIEWTOUCH );
  hlLoadIdentity();
  hlMatrixMode( HL_TOUCHWORKSPACE );
  hlLoadIdentity();

  // TODO: fix matrices
  //hlPushMatrix();

  const Matrix4d &pcal = hd->getPositionCalibrationInverse();
  HLdouble m[16] = { pcal[0][0], pcal[1][0], pcal[2][0], pcal[3][0], 
                     pcal[0][1], pcal[1][1], pcal[2][1], pcal[3][1], 
                     pcal[0][2], pcal[1][2], pcal[2][2], pcal[3][2], 
                     pcal[0][3], pcal[1][3], pcal[2][3], pcal[3][3] }; 
  hlLoadIdentity();
  // scale to convert to metres
  hlScalef( 1000, 1000, 1000 );
  // apply calibration matrix
  hlMultMatrixd( m );
  
  HHLRC haptic_context = NULL;
  
  if( context_map.find( hd ) == context_map.end() ) {
    haptic_context = initHLLayer( hd );
  } else {
    haptic_context = context_map[ hd ];
  }

  if( haptic_context ) {

    hlMakeCurrent( haptic_context );  
    hlBeginFrame();
    hlCheckEvents();
    for( HapticShapeVector::const_iterator i = shapes.begin();
         i != shapes.end();
         i++ ) {
      HLShape *hl = dynamic_cast< HLShape * >( *i );
      if( hl ) {    
        // TODO: Fix
        DeviceInfo *di = DeviceInfo::getActive();
        if( di ) {
          const NodeVector &devices = di->device->getValue();
          if( devices.size() > 0 )
            hl->hlRender( (HLHapticsDevice *) devices[0] );
        }
      }
    }
    
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
    
    hlEndFrame(); 
    //hlMatrixMode( HL_VIEWTOUCH );
    //hlPopMatrix();
    //hlMatrixMode( HL_TOUCHWORKSPACE );
    //hlPopMatrix();
    
    // check for any errors
    HLerror error;
    while ( HL_ERROR(error = hlGetError()) ) {
        cerr << "ERRRRRROR" << endl;
      //Console(4) << HLHapticsDeviceInternal::getHLErrorString( error )
      //<< endl;
    }   
  }
}


HHLRC OpenHapticsRenderer::initHLLayer( HAPIHapticsDevice *hd ) {
  PhantomHapticsDevice *pd = dynamic_cast< PhantomHapticsDevice * >( hd );
  if( !pd ) return NULL;

  if( pd->getDeviceState() != HAPIHapticsDevice::UNINITIALIZED ) {
    if( context_map.find( pd ) == context_map.end() ) {
      // Create a haptic context for the device. The haptic context maintains 
      // the state that persists between frame intervals and is used for
      // haptic rendering.
      HHD jj = pd->getDeviceHandle();
      context_map[ pd ] = hlCreateContext( pd->getDeviceHandle() );
      hlMakeCurrent( context_map[ pd ] );  

      hlEnable(HL_HAPTIC_CAMERA_VIEW);
      hlEnable(HL_ADAPTIVE_VIEWPORT );
      return context_map[ pd ];
    }
  }
  return NULL;
}
