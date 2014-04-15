//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
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
/// \file HapticShapeConstraintWidgetsPage.h
/// \brief header file used to collect user input and create the force effect
/// HapticShapeConstraint found in HAPI.
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

#ifndef __HAPTICSHAPECONSTRAINTWIDGETSPAGE_H__
#define __HAPTICSHAPECONSTRAINTWIDGETSPAGE_H__

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

class HapticShapeConstraintWidgetsPage : public WidgetsPage
{
public:
  HapticShapeConstraintWidgetsPage( wxBookCtrlBase *book,
                                    HAPI::AnyHapticsDevice *_hd );
  virtual ~HapticShapeConstraintWidgetsPage() {};

  virtual wxControl *GetWidget() const { return 0; }

  // virtual function to create the force effect given input from the page.
  // Should also add the force effect to the haptic loop.
  virtual void createForceEffect();

protected:
  // event handlers
  void OnCheckOrRadioBox( wxCommandEvent& event );

  // reset the parameters
  void Reset();

  wxPanel *rightPanel;
  wxSizer *panel_sizer;
  wxSizer *sizerTop;

  wxRadioBox *m_radio_shapes;
  int choosen_shape;

  // Sphere
  wxPanel *sphere_panel;
  wxTextCtrl *m_txt_spring_constant;
  HAPI::HAPIFloat spring_constant;
  wxTextCtrl *m_txt_sphere_radius;
  HAPI::HAPIFloat sphere_radius;

  // Plane
  wxPanel *plane_panel;
  wxTextCtrl *m_txt_plane_pointX;
  wxTextCtrl *m_txt_plane_pointY;
  wxTextCtrl *m_txt_plane_pointZ;
  HAPI::Vec3 plane_point;
  wxTextCtrl *m_txt_plane_normalX;
  wxTextCtrl *m_txt_plane_normalY;
  wxTextCtrl *m_txt_plane_normalZ;
  HAPI::Vec3 plane_normal;

  // Triangle
  wxPanel *triangle_panel;
  wxTextCtrl *m_txt_triangle_1X;
  wxTextCtrl *m_txt_triangle_1Y;
  wxTextCtrl *m_txt_triangle_1Z;
  HAPI::Vec3 triangle_vertex1;
  wxTextCtrl *m_txt_triangle_2X;
  wxTextCtrl *m_txt_triangle_2Y;
  wxTextCtrl *m_txt_triangle_2Z;
  HAPI::Vec3 triangle_vertex2;
  wxTextCtrl *m_txt_triangle_3X;
  wxTextCtrl *m_txt_triangle_3Y;
  wxTextCtrl *m_txt_triangle_3Z;
  HAPI::Vec3 triangle_vertex3;

  // LineSet
  wxPanel *lineSet_panel;
  wxTextCtrl *m_txt_line_set_points;
  vector< HAPI::Collision::LineSegment > line_set_lines;

  // PointSet
  wxPanel *pointSet_panel;
  wxTextCtrl *m_txt_point_set_points;
  vector< HAPI::Collision::Point > point_set_points;

  // TriangleSet
  wxPanel *triangleSet_panel;
  wxTextCtrl *m_txt_triangle_set_triangles;
  vector< HAPI::Collision::Triangle > triangle_set_triangles;

private:
  DECLARE_EVENT_TABLE()
  DECLARE_WIDGETS_PAGE( HapticShapeConstraintWidgetsPage )
};
#endif
