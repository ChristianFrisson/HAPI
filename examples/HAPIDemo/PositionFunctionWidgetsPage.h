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
/// \file PositionFunctionWidgetsPage.h
/// \brief header file used to collect user input and create the force effect
/// HapticPositionFunctionEffect found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////
// FILE IS BASED ON THIS EXAMPLE. MAYBE MODIFIED BEYOND RECOGNITION
// Program:     wxWidgets Widgets Sample
// Name:        button.cpp
// Purpose:     Part of the widgets sample showing wxButton
// Author:      Vadim Zeitlin
// Created:     10.04.01
// Id:          $Id: button.cpp,v 1.17 2005/08/28 08:54:53 MBN Exp $
// Copyright:   (c) 2001 Vadim Zeitlin
// License:     wxWindows license
/////////////////////////////////////////////////////////////////////////////
#ifndef __POSITIONFUNCTIONWIDGETSPAGE_H__
#define __POSITIONFUNCTIONWIDGETSPAGE_H__

// for compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
#pragma hdrstop
#endif

// for all others, include the necessary headers
#ifndef WX_PRECOMP
#include "wx/app.h"
#include "wx/log.h"

#include "wx/button.h"
#include "wx/checkbox.h"
#include "wx/radiobox.h"
#include "wx/statbox.h"
#include "wx/textctrl.h"
#endif

#include "wx/artprov.h"
#include "wx/sizer.h"

// HAPIDemo includes
#include "HAPIDemo.h"

// HAPI includes
#include <HAPI/ParsedFunction.h>

#ifdef HAVE_FPARSER

class PositionFunctionWidgetsPage : public WidgetsPage
{
public:
  // Constructor
  PositionFunctionWidgetsPage( wxBookCtrlBase *book,
                               HAPI::AnyHapticsDevice *_hd );
  // Destructor
  virtual ~PositionFunctionWidgetsPage(){};

  virtual wxControl *GetWidget() const { return 0; }

  // virtual function to create the force effect given input from the page.
  // Should also add the force effect to the haptic loop.
  virtual void createForceEffect();

protected:
  // reset the parameters
  void Reset();

  wxTextCtrl *m_txt_x_function;
  wxTextCtrl *m_txt_y_function;
  wxTextCtrl *m_txt_z_function;
  HAPI::ParsedFunction * x_function;
  HAPI::ParsedFunction * y_function;
  HAPI::ParsedFunction * z_function;

private:
  DECLARE_WIDGETS_PAGE(PositionFunctionWidgetsPage)
};
#endif // HAVE_FPARSER

#endif
