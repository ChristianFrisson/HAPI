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
/// \file HAPIHapticsRenderer.cpp
/// \brief CPP file for HAPIHapticsRenderer
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HAPIHapticsRenderer.h>

using namespace HAPI;


HAPIHapticsRenderer::local_auto_ptr< std::list< HAPIHapticsRenderer::HapticsRendererRegistration > > 
HAPIHapticsRenderer::registered_renderers(NULL);

bool HAPIHapticsRenderer::initialized = false; 

std::map< HAPIHapticsDevice *,
std::vector< HAPIHapticsRenderer::WorkAroundToCleanUpHLContext * > >
      HAPIHapticsRenderer::clean_up_stuff;

HAPIHapticsRenderer::~HAPIHapticsRenderer() { }

void HAPIHapticsRenderer::cleanUpStuff( HAPIHapticsDevice *hd ) {
  if( clean_up_stuff.find( hd ) != clean_up_stuff.end() ) {
    std::vector< WorkAroundToCleanUpHLContext * >
      &clean_up_classes = clean_up_stuff[ hd ];
    for( unsigned int i = 0; i < clean_up_classes.size(); ++i ) {
      clean_up_classes[i]->cleanUp();
      delete clean_up_classes[i];
      clean_up_classes[i] = 0;
    }
    clean_up_stuff.erase( hd );
  }
}
