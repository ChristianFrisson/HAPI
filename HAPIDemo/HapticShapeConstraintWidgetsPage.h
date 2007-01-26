/////////////////////////////////////////////////////////////////////////////
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

#include "HAPIDemo.h"

#include "HapticShapeConstraint.h"

class HapticShapeConstraintWidgetsPage : public WidgetsPage
{
public:
    HapticShapeConstraintWidgetsPage(wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd);
    virtual ~HapticShapeConstraintWidgetsPage(){};

    virtual wxControl *GetWidget() const { return 0; }

    virtual void createForceEffect();
    virtual void removeForceEffect();

protected:
    // event handlers
    void OnCheckOrRadioBox(wxCommandEvent& event);

    // reset the parameters
    void Reset();

    wxRadioBox *m_radioInterpolate;
    bool interpolate;

    wxPanel *rightPanel;
    wxSizer *panel_sizer;

    wxRadioBox *m_radio_shapes;
    int choosen_shape;

    wxPanel *sphere_panel;
    wxTextCtrl *m_txt_spring_constant;
    HAPI::HAPIFloat spring_constant;
    wxTextCtrl *m_txt_sphere_radius;
    HAPI::HAPIFloat sphere_radius;

    wxPanel *box_panel;
    wxTextCtrl *m_txt_box_sizeX;
    wxTextCtrl *m_txt_box_sizeY;
    wxTextCtrl *m_txt_box_sizeZ;
    HAPI::Vec3 box_size;

    wxPanel *cone_panel;
    wxTextCtrl *m_txt_cone_bottomRadius;
    HAPI::HAPIFloat cone_bottom_radius;
    wxTextCtrl *m_txt_cone_height;
    HAPI::HAPIFloat cone_height;

    wxPanel *cylinder_panel;
    wxTextCtrl *m_txt_cylinder_radius;
    HAPI::HAPIFloat cylinder_radius;
    wxTextCtrl *m_txt_cylinder_height;
    HAPI::HAPIFloat cylinder_height;

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

    wxPanel *lineSet_panel;
    wxTextCtrl *m_txt_line_set_points;
    vector< HAPI::Bounds::LineSegment > line_set_lines;

    wxPanel *pointSet_panel;
    wxTextCtrl *m_txt_point_set_points;
    vector< HAPI::Bounds::Point > point_set_points;

    wxPanel *triangleSet_panel;
    wxTextCtrl *m_txt_triangle_set_triangles;
    vector< HAPI::Bounds::Triangle > triangle_set_triangles;

    H3DUtil::AutoRef<HAPI::HapticShapeConstraint> force_effect;

private:
    DECLARE_EVENT_TABLE()
    DECLARE_WIDGETS_PAGE(HapticShapeConstraintWidgetsPage)
};
#endif
