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
/// \file HAPIDemo.h
/// \brief header file containing classes for creating pages used in a
/// wxwidgets book
///
//
//////////////////////////////////////////////////////////////////////////////
// FILE IS BASED ON THIS EXAMPLE. MAYBE MODIFIED BEYOND RECOGNITION
// Program:     wxWidgets Widgets Sample
// Name:        widgets.h
// Purpose:     Common stuff for all widgets project files
// Author:      Vadim Zeitlin
// Created:     27.03.01
// Id:          $Id: widgets.h,v 1.12 2005/08/28 08:54:56 MBN Exp $
// Copyright:   (c) 2001 Vadim Zeitlin
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIDEMO__H__
#define __HAPIDEMO__H__

class wxCheckBox;
class wxBookCtrlBase;
class wxSizer;
class wxTextCtrl;

// Forward Declarations
class WidgetsPageInfo;

// wx includes
#include "wx/panel.h"

// HAPI includes
#include <HAPI/AnyHapticsDevice.h>

namespace HAPIDemo {
  inline std::string toStr( const wxString &s ) {
# if(wxUSE_UNICODE)
    char *b = new char[s.size()+1];
    const wchar_t *wb = s.c_str();
    for( unsigned int i = 0; i < s.size(); i++ ) {
      b[i] = (char)(wb[i]);
    }

    b[s.size()] = '\0';
    std::string sb(b);
    delete[] b;
    return sb;
#else
    return std::string( s.c_str() );
#endif
  }
}

// Custom created Base class used as a new page in the "book UI" of the demo.
class WidgetsPage : public wxPanel
{
public:
  // Constructor
  WidgetsPage( wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd );

  // return the control shown by this page
  virtual wxControl *GetWidget() const = 0;

  // some pages show 2 controls, in this case override this one as well
  virtual wxControl *GetWidget2() const { return NULL; }

  // Set the pointer to the haptics device.
  void setHDPointer( HAPI::AnyHapticsDevice *_hd ) { hd = _hd; }

  // virtual function to create the force effect given input from the page.
  // Should also add the force effect to the haptic loop.
  virtual void createForceEffect() = 0;

  // virtual function to remove the force effect from the haptic loop.
  virtual void removeForceEffect() {
    if( force_effect.get() ) {
      hd->removeEffect( force_effect.get() );
      hd->transferObjects();
    }
  }

protected:
  // A pointer to the haptics device in use.
  HAPI::AnyHapticsDevice *hd;

  // The force effect in use. Contains 0 if there is no force effect in use.
  static H3DUtil::AutoRef<HAPI::HAPIForceEffect> force_effect;
  
  // several helper functions for page creation

  // create a horz sizer containing the given control and the text ctrl
  // (pointer to which will be saved in the provided variable if not NULL)
  // with the specified id
  wxSizer *CreateSizerWithText( wxControl *control,
                                wxWindowID id = wxID_ANY,
                                wxTextCtrl **ppText = NULL,
                                wxWindow *_parent = 0 );

  // create a sizer containing a label and a text ctrl
  wxSizer *CreateSizerWithTextAndLabel( const wxString& label,
                                        wxWindowID id = wxID_ANY,
                                        wxTextCtrl **ppText = NULL,
                                        wxWindow * _parent = 0 );

  // create a sizer containing a button and a text ctrl
  wxSizer *CreateSizerWithTextAndButton( wxWindowID idBtn,
                                         const wxString& labelBtn,
                                         wxWindowID id = wxID_ANY,
                                         wxTextCtrl **ppText = NULL );

  // create a checkbox and add it to the sizer
  wxCheckBox *CreateCheckBoxAndAddToSizer( wxSizer *sizer,
                                           const wxString& label,
                                           wxWindowID id = wxID_ANY );

  // creates three sizers each containing a label and a text control.
  // returns a sizer in which these three are added.
  wxSizer * createXYZInputControls( wxWindow * parent,
                                    const wxString& label,
                                    int vertex_x,
                                    wxTextCtrl **triangle_x,
                                    int vertex_y,
                                    wxTextCtrl **triangle_y,
                                    int vertex_z,
                                    wxTextCtrl **triangle_z,
                                    const wxString& x_label = wxT("x:"),
                                    const wxString& y_label = wxT("y:"),
                                    const wxString& z_label = wxT("z:") );


public:
  // the head of the linked list containing info about all pages
  static WidgetsPageInfo *ms_widgetPages;
};

// ----------------------------------------------------------------------------
// dynamic WidgetsPage creation helpers
// ----------------------------------------------------------------------------

class WidgetsPageInfo
{
public:
  typedef WidgetsPage *(*Constructor)
    ( wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd );

  // our ctor
  WidgetsPageInfo( Constructor ctor, const wxChar *label );

  // accessors
  const wxString& GetLabel() const { return m_label; }
  Constructor GetCtor() const { return m_ctor; }
  WidgetsPageInfo *GetNext() const { return m_next; }

  inline void SetNext( WidgetsPageInfo *next ) { m_next = next; }

private:
  // the label of the page
  wxString m_label;

  // the function to create this page
  Constructor m_ctor;

  // next node in the linked list or NULL
  WidgetsPageInfo *m_next;
};

// to declare a page, this macro must be used in the class declaration
#define DECLARE_WIDGETS_PAGE(classname)                                   \
  private:                                                                \
    static WidgetsPageInfo ms_info##classname;                            \
  public:                                                                 \
    const WidgetsPageInfo *GetPageInfo() const                            \
      { return &ms_info##classname; }

// and this one must be inserted somewhere in the source file
#define IMPLEMENT_WIDGETS_PAGE(classname, label)                          \
  WidgetsPage *wxCtorFor##classname(wxBookCtrlBase *book,                 \
                                    HAPI::AnyHapticsDevice *_hd )         \
    { return new classname(book, _hd); }                                  \
  WidgetsPageInfo classname::                                             \
    ms_info##classname(wxCtorFor##classname, label)

#endif // __HAPIDEMO__H__
