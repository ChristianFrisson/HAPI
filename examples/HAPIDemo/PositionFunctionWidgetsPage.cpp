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
/// \file PositionFunctionWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticPositionFunctionEffect found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "PositionFunctionWidgetsPage.h"

#ifdef HAVE_FPARSER

// HAPI includes
#include <HAPI/HapticPositionFunctionEffect.h>

using namespace HAPI;

enum
{
  x_function_ValueText,
  y_function_ValueText,
  z_function_ValueText
};

enum {
  zero_time_zero,
  zero_time_now
};

IMPLEMENT_WIDGETS_PAGE( PositionFunctionWidgetsPage, _T("PositionFunction") );

PositionFunctionWidgetsPage::PositionFunctionWidgetsPage(
  wxBookCtrlBase *book, AnyHapticsDevice *_hd ) : WidgetsPage( book, _hd )
{
  m_txt_x_function = NULL;
  m_txt_y_function = NULL;
  m_txt_z_function = NULL;
  x_function = 0;
  y_function = 0;
  z_function = 0;

  wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

  wxSizer *sizerLeft =
    new wxStaticBoxSizer( wxVERTICAL, this, _T("Position Function") );

  wxSizer *sizerForce = createXYZInputControls( this,
                           _T("Position functions for each dimension"),
                                                x_function_ValueText,
                                                &m_txt_x_function,
                                                y_function_ValueText,
                                                &m_txt_y_function,
                                                z_function_ValueText,
                                                &m_txt_z_function,
                                                _T("x(x,y,z):"),
                                                _T("y(x,y,z):"),
                                                _T("z(x,y,z):") );

  sizerForce->SetMinSize( 200, 10 );
  sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5 );

  sizerTop->Add(sizerLeft, 0, wxALL, 10);

  // final initializations
  Reset();

  SetSizer(sizerTop);

  sizerTop->Fit(this);
}

void PositionFunctionWidgetsPage::Reset()
{
  m_txt_x_function->SetValue( _T("x*0 + 50 * y + z*0" ) );
  m_txt_y_function->SetValue( _T("0" ) );
  m_txt_z_function->SetValue( _T("0" ) );
}

void PositionFunctionWidgetsPage::createForceEffect() {
  
  x_function = new ParsedFunction();
  x_function->setFunctionString(
    HAPIDemo::toStr( m_txt_x_function->GetValue()), "x,y,z" );
  y_function = new ParsedFunction();
  y_function->setFunctionString(
    HAPIDemo::toStr( m_txt_y_function->GetValue()), "x,y,z" );
  z_function = new ParsedFunction();
  z_function->setFunctionString(
    HAPIDemo::toStr( m_txt_z_function->GetValue()), "x,y,z" );

  force_effect.reset( new HapticPositionFunctionEffect( x_function,
                                                        y_function,
                                                        z_function ) );
  hd->clearEffects();
  hd->addEffect( force_effect.get() );
  hd->transferObjects();
}
#endif // HAVE_FPARSER
