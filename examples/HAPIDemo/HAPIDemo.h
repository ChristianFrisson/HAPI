/////////////////////////////////////////////////////////////////////////////
// Program:     wxWidgets Widgets Sample
// Name:        widgets.h
// Purpose:     Common stuff for all widgets project files
// Author:      Vadim Zeitlin
// Created:     27.03.01
// Id:          $Id: widgets.h,v 1.12 2005/08/28 08:54:56 MBN Exp $
// Copyright:   (c) 2001 Vadim Zeitlin
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

#ifndef _WX_SAMPLE_WIDGETS_H_
#define _WX_SAMPLE_WIDGETS_H_

class WXDLLEXPORT wxCheckBox;
class WXDLLEXPORT wxBookCtrlBase;
class WXDLLEXPORT wxSizer;
class WXDLLEXPORT wxTextCtrl;

class WidgetsPageInfo;

#include "wx/panel.h"

#include <HAPI/AnyHapticsDevice.h>

namespace HAPIDemo {
  inline string toStr( const wxString &s ) {
# if(wxUSE_UNICODE)
    char *b = new char[s.size()+1];
    const wchar_t *wb = s.c_str();
    for( unsigned int i = 0; i < s.size(); i++ ) {
      b[i] = (char)(wb[i]);
    }
    
    b[s.size()] = '\0';
    string sb(b);
    delete[] b;
    return sb;
#else
    return string( s.c_str() );
#endif
}
}


class WidgetsPage : public wxPanel
{
public:
    WidgetsPage(wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd);

    // return the control shown by this page
    virtual wxControl *GetWidget() const = 0;

    // some pages show 2 controls, in this case override this one as well
    virtual wxControl *GetWidget2() const { return NULL; }

    void setHDPointer( HAPI::AnyHapticsDevice *_hd ) { hd = _hd; }
    virtual void createForceEffect() = 0;
    virtual void removeForceEffect() = 0;

protected:
  HAPI::AnyHapticsDevice *hd;
    // several helper functions for page creation

    // create a horz sizer containing the given control and the text ctrl
    // (pointer to which will be saved in the provided variable if not NULL)
    // with the specified id
    wxSizer *CreateSizerWithText(wxControl *control,
                                 wxWindowID id = wxID_ANY,
                                 wxTextCtrl **ppText = NULL,
                                 wxWindow *_parent = 0 );

    // create a sizer containing a label and a text ctrl
    wxSizer *CreateSizerWithTextAndLabel(const wxString& label,
                                         wxWindowID id = wxID_ANY,
                                         wxTextCtrl **ppText = NULL,
                                         wxWindow * _parent = 0 );

    // create a sizer containing a button and a text ctrl
    wxSizer *CreateSizerWithTextAndButton(wxWindowID idBtn,
                                          const wxString& labelBtn,
                                          wxWindowID id = wxID_ANY,
                                          wxTextCtrl **ppText = NULL);

    // create a checkbox and add it to the sizer
    wxCheckBox *CreateCheckBoxAndAddToSizer(wxSizer *sizer,
                                            const wxString& label,
                                            wxWindowID id = wxID_ANY);

public:
    // the head of the linked list containinginfo about all pages
    static WidgetsPageInfo *ms_widgetPages;
};

// ----------------------------------------------------------------------------
// dynamic WidgetsPage creation helpers
// ----------------------------------------------------------------------------

class WidgetsPageInfo
{
public:
    typedef WidgetsPage *(*Constructor)(wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd);

    // our ctor
    WidgetsPageInfo(Constructor ctor, const wxChar *label);

    // accessors
    const wxString& GetLabel() const { return m_label; }
    Constructor GetCtor() const { return m_ctor; }
    WidgetsPageInfo *GetNext() const { return m_next; }

    void SetNext(WidgetsPageInfo *next) { m_next = next; }

private:
    // the label of the page
    wxString m_label;

    // the function to create this page
    Constructor m_ctor;

    // next node in the linked list or NULL
    WidgetsPageInfo *m_next;
};

// to declare a page, this macro must be used in the class declaration
#define DECLARE_WIDGETS_PAGE(classname)                                     \
    private:                                                                \
        static WidgetsPageInfo ms_info##classname;                          \
    public:                                                                 \
        const WidgetsPageInfo *GetPageInfo() const                          \
            { return &ms_info##classname; }

// and this one must be inserted somewhere in the source file
#define IMPLEMENT_WIDGETS_PAGE(classname, label)                            \
    WidgetsPage *wxCtorFor##classname(wxBookCtrlBase *book,                 \
                                      HAPI::AnyHapticsDevice *_hd )         \
        { return new classname(book, _hd); }                                 \
    WidgetsPageInfo classname::                                             \
        ms_info##classname(wxCtorFor##classname, label)

#endif // _WX_SAMPLE_WIDGETS_H_
