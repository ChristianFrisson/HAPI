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
/// \file ForceFieldWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticForceField found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "ForceFieldWidgetsPage.h"

// HAPI includes
#include <HAPI/HapticForceField.h>

using namespace HAPI;

// control ids
enum
{
  X_ValueText,
  Y_ValueText,
  Z_ValueText
};

IMPLEMENT_WIDGETS_PAGE( ForceFieldWidgetsPage, _T("ForceField") );

ForceFieldWidgetsPage::ForceFieldWidgetsPage( wxBookCtrlBase *book,
                                              AnyHapticsDevice *_hd )
                                              : WidgetsPage( book, _hd )
{
  m_txt_forceX = NULL;
  m_txt_forceY = NULL;
  m_txt_forceZ = NULL;

  wxSizer *sizerTop = new wxBoxSizer( wxHORIZONTAL );

  wxSizer *sizerLeft = new wxStaticBoxSizer( wxVERTICAL, this,
                                             _T("Force Field values" ) );

  sizerLeft->Add( createXYZInputControls( this,
                                          _T("Force (N)"),
                                          X_ValueText,
                                          &m_txt_forceX,
                                          Y_ValueText,
                                          &m_txt_forceY,
                                          Z_ValueText,
                                          &m_txt_forceZ ),
                  0, wxALL | wxGROW, 5 );

  sizerTop->Add( sizerLeft, 0, wxALL, 10 );

  // final initializations
  Reset();

  SetSizer( sizerTop );

  sizerTop->Fit(this);
}

void ForceFieldWidgetsPage::Reset()
{
  m_txt_forceX->SetValue( _T("1.0" ) );
  force.x = 1.0;
  m_txt_forceY->SetValue( _T("0.0" ) );
  force.y = 0.0;
  m_txt_forceZ->SetValue( _T("0.0" ) );
  force.z = 0.0;
}

void ForceFieldWidgetsPage::createForceEffect() {
  double val;
  if( m_txt_forceX->GetValue().ToDouble(&val) ) {
    force.x = val;
  }

  if( m_txt_forceY->GetValue().ToDouble(&val) ) {
    force.y = val;
  }

  if( m_txt_forceZ->GetValue().ToDouble(&val) ) {
    force.z = val;
  }

  force_effect.reset( new HapticForceField( force ) );
  hd->clearEffects();
  hd->addEffect( force_effect.get() );
  hd->transferObjects();
}

