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
/// \file SpringWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticSpring found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "SpringWidgetsPage.h"

// HAPI includes
#include <HAPI/HapticSpring.h>

using namespace HAPI;

enum
{
  X_ValueText,
  Y_ValueText,
  Z_ValueText,
  spring_constant_ValueText
};

IMPLEMENT_WIDGETS_PAGE( SpringWidgetsPage, _T("Spring") );

SpringWidgetsPage::SpringWidgetsPage( wxBookCtrlBase *book,
                                      AnyHapticsDevice *_hd )
                                  : WidgetsPage(book, _hd)
{
  m_txt_positionX = NULL;
  m_txt_positionY = NULL;
  m_txt_positionZ = NULL;
  m_txt_spring_constant = NULL;

  wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

  wxSizer *sizerLeft =
    new wxStaticBoxSizer( wxVERTICAL, this, _T("Spring Force") );

  sizerLeft->Add( createXYZInputControls( this,
                                          _T("Position in meters"),
                                          X_ValueText,
                                          &m_txt_positionX,
                                          Y_ValueText,
                                          &m_txt_positionY,
                                          Z_ValueText,
                                          &m_txt_positionZ ),
                  0, wxALL | wxGROW, 5 );

   wxSizer *sizerRow = CreateSizerWithTextAndLabel( _T("spring constant:"),
                                                    spring_constant_ValueText,
                                                    &m_txt_spring_constant,
                                                    this );

  sizerLeft->Add( sizerRow, 0, wxALL, 5 );

  sizerTop->Add( sizerLeft, 0, wxALL, 10 );

  // final initializations
  Reset();

  SetSizer(sizerTop);

  sizerTop->Fit(this);
}

void SpringWidgetsPage::Reset()
{
  m_txt_positionX->SetValue( _T("0.0" ) );
  position.x = 0.0;
  m_txt_positionY->SetValue( _T("0.0" ) );
  position.y = 0.0;
  m_txt_positionZ->SetValue( _T("0.0" ) );
  position.z = 0.0;
  m_txt_spring_constant->SetValue( _T("100") );
  spring_constant = 100;
}

void SpringWidgetsPage::createForceEffect() {
  double val;
  if( m_txt_positionX->GetValue().ToDouble(&val) ) {
    position.x = val;
  }

  if( m_txt_positionY->GetValue().ToDouble(&val) ) {
    position.y = val;
  }

  if( m_txt_positionZ->GetValue().ToDouble(&val) ) {
    position.z = val;
  }

  if( m_txt_spring_constant->GetValue().ToDouble(&val) ) {
    spring_constant = val;
  }

  force_effect.reset( new HapticSpring( position, spring_constant ) );
  hd->clearEffects();
  hd->addEffect( force_effect.get() );
  hd->transferObjects();
}
