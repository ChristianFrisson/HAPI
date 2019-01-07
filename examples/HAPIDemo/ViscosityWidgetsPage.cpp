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
/// \file ViscosityWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticViscosity found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "ViscosityWidgetsPage.h"
using namespace HAPI;

enum
{
  Viscosity_ValueText,
  Radius_ValueText,
  Damping_Factor_ValueText
};

IMPLEMENT_WIDGETS_PAGE( ViscosityWidgetsPage, _T("Viscosity") );

ViscosityWidgetsPage::ViscosityWidgetsPage( wxBookCtrlBase *book,
                                            AnyHapticsDevice *_hd )
                                        : WidgetsPage( book, _hd )
{
  m_txt_viscosity = NULL;
  m_txt_radius = NULL;
  m_txt_damping_factor = NULL;

  wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

  wxSizer *sizerLeft =
    new wxStaticBoxSizer( wxVERTICAL, this, _T("Viscosity effect values") );

  sizerLeft->Add( createXYZInputControls( this,
                                          _T("Values in meter"),
                                          Viscosity_ValueText,
                                          &m_txt_viscosity,
                                          Radius_ValueText,
                                          &m_txt_radius,
                                          Damping_Factor_ValueText,
                                          &m_txt_damping_factor,
                                          _T("viscosity(Pa s):"),
                                          _T("radius(m):      "),
                                          _T("damping:        ") ),
                  0, wxALL | wxGROW, 5 );

  sizerTop->Add( sizerLeft, 0, wxALL, 10 );

  // final initializations
  Reset();

  SetSizer( sizerTop );

  sizerTop->Fit(this);
}

void ViscosityWidgetsPage::Reset()
{
  m_txt_viscosity->SetValue( _T("0.01" ) );
  viscosity = 0.01;
  m_txt_radius->SetValue( _T("0.0025" ) );
  radius = 0.0025;
  m_txt_damping_factor->SetValue( _T("1" ) );
  damping_factor = 0.5;
}

void ViscosityWidgetsPage::createForceEffect() {
  double val;
  if( m_txt_viscosity->GetValue().ToDouble(&val) ) {
    viscosity = val;
  }

  if( m_txt_radius->GetValue().ToDouble(&val) ) {
    radius = val;
  }

  if( m_txt_damping_factor->GetValue().ToDouble(&val) ) {
    damping_factor = val;
  }

  force_effect.reset( new HapticViscosity( viscosity,
                                           radius,
                                           damping_factor ) );
  hd->clearEffects();
  hd->addEffect( force_effect.get() );
  hd->transferObjects();
}

