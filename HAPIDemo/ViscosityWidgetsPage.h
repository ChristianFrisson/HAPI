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

#ifndef __VISCOSITYWIDGETSPAGE_H__
#define __VISCOSITYWIDGETSPAGE_H__

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

#include "HapticViscosity.h"

class ViscosityWidgetsPage : public WidgetsPage
{
public:
  ViscosityWidgetsPage(wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd);
    virtual ~ViscosityWidgetsPage(){};

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

    wxTextCtrl *m_txt_viscosity;
    wxTextCtrl *m_txt_radius;
    wxTextCtrl *m_txt_damping_factor;
    double viscosity, radius, damping_factor;

    H3DUtil::AutoRef<HAPI::HapticViscosity> force_effect;

private:
    DECLARE_EVENT_TABLE()
    DECLARE_WIDGETS_PAGE(ViscosityWidgetsPage)
};
#endif
