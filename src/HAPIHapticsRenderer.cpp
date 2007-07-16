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
/// \file HAPIHapticsRenderer.cpp
/// \brief CPP file for HAPIHapticsRenderer
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPIHapticsRenderer.h>

using namespace HAPI;


list< HAPIHapticsRenderer::HapticsRendererRegistration > 
*HAPIHapticsRenderer::registered_renderers;

bool HAPIHapticsRenderer::initialized = false; 

map< HAPIHapticsDevice *,
vector< HAPIHapticsRenderer::WorkAroundToCleanUpHLContext * > >
      HAPIHapticsRenderer::clean_up_stuff;

HAPIHapticsRenderer::~HAPIHapticsRenderer() { }

void HAPIHapticsRenderer::cleanUpStuff( HAPIHapticsDevice *hd ) {
  if( clean_up_stuff.find( hd ) != clean_up_stuff.end() ) {
    vector< WorkAroundToCleanUpHLContext * >
      &clean_up_classes = clean_up_stuff[ hd ];
    for( unsigned int i = 0; i < clean_up_classes.size(); i++ ) {
      clean_up_classes[i]->cleanUp();
      delete clean_up_classes[i];
      clean_up_classes[i] = 0;
    }
    clean_up_stuff.erase( hd );
  }
}
