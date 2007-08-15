// for compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

// for all others, include the necessary headers
#ifndef WX_PRECOMP
    #include "wx/app.h"
    #include "wx/frame.h"
    #include "wx/menu.h"

    #include "wx/button.h"
    #include "wx/checkbox.h"
    #include "wx/listbox.h"
    #include "wx/statbox.h"
    #include "wx/stattext.h"
    #include "wx/textctrl.h"
    #include "wx/msgdlg.h"
#endif

#include "wx/sysopt.h"
#include "wx/bookctrl.h"
#include "wx/sizer.h"
#include "wx/colordlg.h"
#include "wx/fontdlg.h"
#include "wx/textdlg.h"

#include "HAPIDemo.h"

#include <HAPIForceEffect.h>

// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------


enum {
  
} my_Buttons;

// control ids
enum
{
    Widgets_Quit,
#if wxUSE_TOOLTIPS
    Widgets_SetTooltip,
#endif // wxUSE_TOOLTIPS
    Widgets_SetFgColour,
    Widgets_SetBgColour,
    Widgets_SetFont,
    Widgets_Enable,
    StartEffect,
    StopEffect
};

// Define a new application type, each program should derive a class from wxApp
class WidgetsApp : public wxApp
{
public:
    // override base class virtuals
    // ----------------------------

    // this one is called on application startup and is a good place for the app
    // initialization (doing it here and not in the ctor allows to have an error
    // return: if OnInit() returns false, the application terminates)
    virtual bool OnInit();
};

// Define a new frame type: this is going to be our main frame
class WidgetsFrame : public wxFrame
{
public:
    // ctor(s) and dtor
    WidgetsFrame(const wxString& title);
    virtual ~WidgetsFrame() { hd.releaseDevice(); }

    HAPI::AnyHapticsDevice hd;

protected:
  H3DUtil::AutoRef<HAPI::HAPIForceEffect> force_to_run;
    void OnExit(wxCommandEvent& event);
    void OnStartEffect( wxCommandEvent& WXUNUSED(event) );
    void OnStopEffect( wxCommandEvent& WXUNUSED(event) );

    // initialize the book: add all pages to it
    void InitBook();

private:
    // the panel containing everything
    wxPanel *m_panel;

    // the book containing the test pages
    wxBookCtrlBase *m_book;

    // any class wishing to process wxWidgets events must use this macro
    DECLARE_EVENT_TABLE()
};

// array of pages
WX_DEFINE_ARRAY_PTR(WidgetsPage *, ArrayWidgetsPage);

// ----------------------------------------------------------------------------
// misc macros
// ----------------------------------------------------------------------------

IMPLEMENT_APP(WidgetsApp)

// ----------------------------------------------------------------------------
// event tables
// ----------------------------------------------------------------------------

BEGIN_EVENT_TABLE(WidgetsFrame, wxFrame)
    EVT_BUTTON(Widgets_Quit, WidgetsFrame::OnExit)
    EVT_BUTTON( StartEffect, WidgetsFrame::OnStartEffect )
    EVT_BUTTON( StopEffect, WidgetsFrame::OnStopEffect )
    EVT_MENU(wxID_EXIT, WidgetsFrame::OnExit)
END_EVENT_TABLE()

bool WidgetsApp::OnInit()
{
    if ( !wxApp::OnInit() )
        return false;

    // the reason for having these ifdef's is that I often run two copies of
    // this sample side by side and it is useful to see which one is which
    wxString title;
#if defined(__WXUNIVERSAL__)
    title = _T("wxUniv/");
#endif

#if defined(__WXMSW__)
    title += _T("");
#elif defined(__WXGTK__)
    title += _T("wxGTK");
#elif defined(__WXMAC__)
    title += _T("wxMAC");
#elif defined(__WXMOTIF__)
    title += _T("wxMOTIF");
#else
    title += _T("wxWidgets");
#endif

    WidgetsFrame *frame = new WidgetsFrame(title + _T("HAPI demo"));
    frame->Show();

    if( frame->hd.initDevice() != HAPI::HAPIHapticsDevice::SUCCESS ) {
      wxString msg;
      msg.Printf( _T(frame->hd.getLastErrorMsg().c_str()) );
      wxMessageBox( msg, _T("hd-problem"), wxOK | wxICON_INFORMATION, frame );
      return false;
    }
    frame->hd.enableDevice();

    return true;
}

// ----------------------------------------------------------------------------
// WidgetsFrame construction
// ----------------------------------------------------------------------------

WidgetsFrame::WidgetsFrame(const wxString& title)
            : wxFrame(NULL, wxID_ANY, title,
                      wxPoint(0, 50), wxDefaultSize,
                      wxDEFAULT_FRAME_STYLE |
                      wxNO_FULL_REPAINT_ON_RESIZE |
                      wxCLIP_CHILDREN |
                      wxTAB_TRAVERSAL)
{
    // init everything
    m_book = (wxBookCtrlBase *)NULL;
    force_to_run.reset( 0 );

#if wxUSE_MENUS
    // create the menubar
    wxMenuBar *mbar = new wxMenuBar;
    wxMenu *menuWidget = new wxMenu;
    menuWidget->Append(wxID_EXIT, _T("&Quit\tCtrl-Q"));
    mbar->Append(menuWidget, _T("&HAPI"));
    SetMenuBar(mbar);

    //mbar->Check(Widgets_Enable, true);
#endif // wxUSE_MENUS

    // create controls
    m_panel = new wxPanel(this, wxID_ANY,
        wxDefaultPosition, wxDefaultSize, wxCLIP_CHILDREN);

    wxSizer *sizerTop = new wxBoxSizer(wxVERTICAL);

    // we have 2 panes: book with pages demonstrating the controls in the
    // upper one and the start effect, stop effect and exit button in the lower

    int style = wxNO_FULL_REPAINT_ON_RESIZE|wxCLIP_CHILDREN|wxBC_DEFAULT;

    m_book = new wxBookCtrl(m_panel, wxID_ANY, wxDefaultPosition,
#ifdef __WXMOTIF__
        wxSize(500, -1), // under Motif, height is a function of the width...
#else
        wxDefaultSize,
#endif
        style);
    InitBook();

#ifndef __SMARTPHONE__

    wxBoxSizer *sizerDown = new wxBoxSizer( wxHORIZONTAL );
    sizerDown->Add(
      new wxButton( m_panel, StartEffect, _T("StartEffect") ),
      0,           // make horizontally unstretchable
      wxALL,       // make border all around (implicit top alignment)
      10 );        // set border width to 10
    sizerDown->Add(
      new wxButton( m_panel, StopEffect, _T("StopEffect") ),
      0,           // make horizontally unstretchable
      wxALL,       // make border all around (implicit top alignment)
      10 );        // set border width to 10

    wxButton *btn;
    btn = new wxButton(m_panel, Widgets_Quit, _T("E&xit"));
    sizerDown->Add( btn, 0, wxALL, 10 );

    // put everything together
    sizerTop->Add(m_book, 1, wxGROW | (wxALL & ~(wxTOP | wxBOTTOM)), 10);
    sizerTop->Add(0, 5, 0, wxGROW); // spacer in between
    sizerTop->Add(sizerDown, 0,  wxGROW | (wxALL & ~wxTOP), 10);

#else // !__SMARTPHONE__/__SMARTPHONE__

    sizerTop->Add(m_book, 1, wxGROW | wxALL );

#endif // __SMARTPHONE__

    m_panel->SetSizer(sizerTop);

    sizerTop->Fit(this);
    sizerTop->SetSizeHints(this);
}

void WidgetsFrame::InitBook()
{
    ArrayWidgetsPage pages;
    wxArrayString labels;

    // we need to first create all pages and only then add them to the book
    // as we need the image list first
    WidgetsPageInfo *info = WidgetsPage::ms_widgetPages;
    while ( info )
    {
        WidgetsPage *page = (*info->GetCtor())(m_book, &hd);
        pages.Add(page);

        labels.Add(info->GetLabel());

        info = info->GetNext();
    }

    // now do add them
    size_t count = pages.GetCount();
    for ( size_t n = 0; n < count; n++ )
    {
        m_book->AddPage(
                        pages[n],
                        labels[n],
                        false
                       );

    }
}

// ----------------------------------------------------------------------------
// WidgetsFrame event handlers
// ----------------------------------------------------------------------------

void WidgetsFrame::OnExit(wxCommandEvent& WXUNUSED(event))
{
    Close();
}

void WidgetsFrame::OnStartEffect( wxCommandEvent& WXUNUSED(event) ) {
  WidgetsPage *page = wxStaticCast(m_book->GetCurrentPage(), WidgetsPage);
  if( page )
    page->createForceEffect( /*hd*/ );
}

void WidgetsFrame::OnStopEffect( wxCommandEvent& WXUNUSED(event) ) {
  WidgetsPage *page = wxStaticCast(m_book->GetCurrentPage(), WidgetsPage);
  if( page )
    page->removeForceEffect( /*hd*/ );
}

// ----------------------------------------------------------------------------
// WidgetsPageInfo
// ----------------------------------------------------------------------------

WidgetsPageInfo *WidgetsPage::ms_widgetPages = NULL;

WidgetsPageInfo::WidgetsPageInfo(Constructor ctor, const wxChar *label)
               : m_label(label)
{
    m_ctor = ctor;

    m_next = NULL;

    // dummy sorting: add and immediately sort on list according to label

    if(WidgetsPage::ms_widgetPages)
    {
        WidgetsPageInfo *node_prev = WidgetsPage::ms_widgetPages;
        if(wxStrcmp(label,node_prev->GetLabel().c_str())<0)
        {
            // add as first
            m_next = node_prev;
            WidgetsPage::ms_widgetPages = this;
        }
        else
        {
            WidgetsPageInfo *node_next;
            do
            {
                node_next = node_prev->GetNext();
                if(node_next)
                {
                    // add if between two
                    if(wxStrcmp(label,node_next->GetLabel().c_str())<0)
                    {
                        node_prev->SetNext(this);
                        m_next = node_next;
                        // force to break loop
                        node_next = NULL;
                    }
                }
                else
                {
                    // add as last
                    node_prev->SetNext(this);
                    m_next = node_next;
                }
                node_prev = node_next;
            }while(node_next);
        }
    }
    else
    {
        // add when first

        WidgetsPage::ms_widgetPages = this;

    }

}

// ----------------------------------------------------------------------------
// WidgetsPage
// ----------------------------------------------------------------------------

WidgetsPage::WidgetsPage(wxBookCtrlBase *book, HAPI::AnyHapticsDevice *_hd)
           : wxPanel(book, wxID_ANY,
                     wxDefaultPosition, wxDefaultSize,
                     wxNO_FULL_REPAINT_ON_RESIZE |
                     wxCLIP_CHILDREN |
                     wxTAB_TRAVERSAL), hd(_hd)
{
}

wxSizer *WidgetsPage::CreateSizerWithText(wxControl *control,
                                          wxWindowID id,
                                          wxTextCtrl **ppText,
                                          wxWindow *_parent)
{
    wxSizer *sizerRow = new wxBoxSizer(wxHORIZONTAL);
    wxTextCtrl *text = new wxTextCtrl(_parent, id, wxEmptyString,
        wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);

    sizerRow->Add(control, 0, wxRIGHT | wxALIGN_CENTRE_VERTICAL, 5);
    sizerRow->Add(text, 1, wxLEFT | wxALIGN_CENTRE_VERTICAL, 5);

    if ( ppText )
        *ppText = text;

    return sizerRow;
}

// create a sizer containing a label and a text ctrl
wxSizer *WidgetsPage::CreateSizerWithTextAndLabel(const wxString& label,
                                                  wxWindowID id,
                                                  wxTextCtrl **ppText,
                                                  wxWindow *_parent )
{
    return CreateSizerWithText(new wxStaticText(_parent, wxID_ANY, label),
        id, ppText, _parent);
}

// create a sizer containing a button and a text ctrl
wxSizer *WidgetsPage::CreateSizerWithTextAndButton(wxWindowID idBtn,
                                                   const wxString& label,
                                                   wxWindowID id,
                                                   wxTextCtrl **ppText)
{
    return CreateSizerWithText(new wxButton(this, idBtn, label), id, ppText, this);
}

wxCheckBox *WidgetsPage::CreateCheckBoxAndAddToSizer(wxSizer *sizer,
                                                     const wxString& label,
                                                     wxWindowID id)
{
    wxCheckBox *checkbox = new wxCheckBox(this, id, label);
    sizer->Add(checkbox, 0, wxLEFT | wxRIGHT, 5);
    sizer->Add(0, 2, 0, wxGROW); // spacer

    return checkbox;
}