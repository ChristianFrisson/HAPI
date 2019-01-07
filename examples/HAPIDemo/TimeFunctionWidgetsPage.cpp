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
/// \file TimeFunctionWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticTimeFunctionEffect found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "TimeFunctionWidgetsPage.h"
#ifdef HAVE_FPARSER

// HAPI includes
#include <HAPI/HapticTimeFunctionEffect.h>

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

BEGIN_EVENT_TABLE( TimeFunctionWidgetsPage, WidgetsPage )
  EVT_RADIOBOX( wxID_ANY, TimeFunctionWidgetsPage::OnCheckOrRadioBox )
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE( TimeFunctionWidgetsPage, _T("TimeFunction") );

TimeFunctionWidgetsPage::TimeFunctionWidgetsPage( wxBookCtrlBase *book,
                                                  AnyHapticsDevice *_hd )
                                              : WidgetsPage(book, _hd)
{
  m_txt_x_function = NULL;
  m_txt_y_function = NULL;
  m_txt_z_function = NULL;
  x_function = 0;
  y_function = 0;
  z_function = 0;

  wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

  wxSizer *sizerLeft =
    new wxStaticBoxSizer( wxVERTICAL, this, _T("Time Function") );

  wxSizer *sizerForce = createXYZInputControls( this,
                              _T("Time functions for each dimension"),
                                                x_function_ValueText,
                                                &m_txt_x_function,
                                                y_function_ValueText,
                                                &m_txt_y_function,
                                                z_function_ValueText,
                                                &m_txt_z_function,
                                                _T("x(t):"),
                                                _T("y(t):"),
                                                _T("z(t):"));

  // should be in sync with enums zero_time_zero(now)!
  static const wxString time_choice[] =
  {
    _T("0"),
    _T("now")
  };

  x_radio_zero_time = new wxRadioBox( this, wxID_ANY, _T("x_zero_time"),
                                      wxDefaultPosition, wxDefaultSize,
                                      WXSIZEOF(time_choice), time_choice );

  sizerForce->Add( x_radio_zero_time, 0, wxALL, 5 );

  y_radio_zero_time = new wxRadioBox( this, wxID_ANY, _T("y_zero_time"),
                                      wxDefaultPosition, wxDefaultSize,
                                      WXSIZEOF(time_choice), time_choice);

  sizerForce->Add( y_radio_zero_time, 0, wxALL, 5 );

  z_radio_zero_time = new wxRadioBox( this, wxID_ANY, _T("z_zero_time"),
                                      wxDefaultPosition, wxDefaultSize,
                                      WXSIZEOF(time_choice), time_choice);

  sizerForce->Add( z_radio_zero_time, 0, wxALL, 5 );

  sizerForce->SetMinSize( 200, 10 );

  sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5);

  sizerTop->Add(sizerLeft, 0, wxALL, 10);

  // final initializations
  Reset();

  SetSizer( sizerTop );

  sizerTop->Fit(this);
}

void TimeFunctionWidgetsPage::Reset()
{
  m_txt_x_function->SetValue(_T("sin(t)" ));
  m_txt_y_function->SetValue(_T("0" ));
  m_txt_z_function->SetValue(_T("0" ));
  x_radio_zero_time->SetSelection( zero_time_now ); 
  use_x_zero = false;
  y_radio_zero_time->SetSelection( zero_time_now ); 
  use_y_zero = false;
  z_radio_zero_time->SetSelection( zero_time_now ); 
  use_z_zero = false;
}

void TimeFunctionWidgetsPage::OnCheckOrRadioBox(
  wxCommandEvent& WXUNUSED(event) )
{
  switch( x_radio_zero_time->GetSelection() )
  {
    case zero_time_zero:
      use_x_zero = true;
      break;

    case zero_time_now:
      use_x_zero = false;
      break;

    default:
      wxFAIL_MSG( _T("unexpected radiobox selection") );
  }

  switch( y_radio_zero_time->GetSelection() )
  {
    case zero_time_zero:
      use_y_zero = true;
      break;

    case zero_time_now:
      use_y_zero = false;
      break;

    default:
      wxFAIL_MSG(_T("unexpected radiobox selection"));
  }

  switch( z_radio_zero_time->GetSelection() )
  {
    case zero_time_zero:
      use_z_zero = true;
      break;

    case zero_time_now:
      use_z_zero = false;
      break;

    default:
      wxFAIL_MSG(_T("unexpected radiobox selection"));
  }
}

void TimeFunctionWidgetsPage::createForceEffect() {
  HAPITime current_time = H3DUtil::TimeStamp();
  HAPITime x_zero_time = 0, y_zero_time = 0, z_zero_time = 0;
  if( !use_x_zero )
    x_zero_time = current_time;
  if( !use_y_zero )
    y_zero_time = current_time;
  if( !use_z_zero )
    z_zero_time = current_time;

  x_function = new ParsedFunction();
  x_function->setFunctionString(
    HAPIDemo::toStr( m_txt_x_function->GetValue() ), "t" );
  y_function = new ParsedFunction();
  y_function->setFunctionString(
    HAPIDemo::toStr( m_txt_y_function->GetValue() ), "t" );
  z_function = new ParsedFunction();
  z_function->setFunctionString(
    HAPIDemo::toStr( m_txt_z_function->GetValue() ), "t" );

  force_effect.reset( new HapticTimeFunctionEffect( x_function,
                                                    y_function,
                                                    z_function,
                                                    x_zero_time,
                                                    y_zero_time,
                                                    z_zero_time ) );
  hd->clearEffects();
  hd->addEffect( force_effect.get() );
  hd->transferObjects();
}
#endif //HAVE_FPARSER
