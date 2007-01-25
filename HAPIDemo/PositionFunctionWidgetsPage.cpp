#include "PositionFunctionWidgetsPage.h"
using namespace HAPI;

enum
{
  ButtonInterpolate_true,
  ButtonInterpolate_false,
  x_function_ValueText,
  y_function_ValueText,
  z_function_ValueText
};

enum {
  zero_time_zero,
  zero_time_now
};

BEGIN_EVENT_TABLE(PositionFunctionWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, PositionFunctionWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(PositionFunctionWidgetsPage, _T("PositionFunction"));

PositionFunctionWidgetsPage::PositionFunctionWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
                  : WidgetsPage(book, _hd)
{

    m_radioInterpolate = (wxRadioBox *)NULL;
    m_txt_x_function = NULL;
    m_txt_y_function = NULL;
    m_txt_z_function = NULL;
    x_function = 0;
    y_function = 0;
    z_function = 0;

    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Position Function") );

    sizerLeft->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    // should be in sync with enums ButtonInterpolate_true(false)!
    static const wxString interpolate[] =
    {
        _T("true"),
        _T("false")
    };

    m_radioInterpolate = new wxRadioBox(this, wxID_ANY, _T("&interpolate"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(interpolate), interpolate);

    sizerLeft->Add(m_radioInterpolate, 0, wxALL, 5);

    sizerLeft->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    wxSizer *sizerForce = new wxStaticBoxSizer(wxVERTICAL, this, _T("Position functions for each dimension") );

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x(x,y,z):"),
                                            x_function_ValueText,
                                            &m_txt_x_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y(x,y,z):"),
                                            y_function_ValueText,
                                            &m_txt_y_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z(x,y,z):"),
                                            z_function_ValueText,
                                            &m_txt_z_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerForce->SetMinSize( 200, 10 );

    sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5);

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void PositionFunctionWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_txt_x_function->SetValue(_T("x*0 + y + z*0" ));
    m_txt_y_function->SetValue(_T("0" ));
    m_txt_z_function->SetValue(_T("0" ));
}

void PositionFunctionWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
{
    switch ( m_radioInterpolate->GetSelection() )
    {
        case ButtonInterpolate_true:
          interpolate = true;
            break;

        default:
            wxFAIL_MSG(_T("unexpected radiobox selection"));
            // fall through

        case ButtonInterpolate_false:
          interpolate = false;
            break;
    }
}

void PositionFunctionWidgetsPage::createForceEffect( ) {

  x_function = new ParsedFunction();
  x_function->setFunctionString( (string)(m_txt_x_function->GetValue()), "x,y,z" );
  y_function = new ParsedFunction();
  y_function->setFunctionString( (string)(m_txt_y_function->GetValue()), "x,y,z" );
  z_function = new ParsedFunction();
  z_function->setFunctionString( (string)(m_txt_z_function->GetValue()), "x,y,z" );

  force_effect.reset( new HapticPositionFunctionEffect( Matrix4(), interpolate,
    x_function,
    y_function,
    z_function ) );
    hd->addEffect( force_effect.get() );
    hd->clearEffects();
    hd->addEffect( force_effect.get() );
}

void PositionFunctionWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
