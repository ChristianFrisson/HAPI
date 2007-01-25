#include "TimeFunctionWidgetsPage.h"
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

BEGIN_EVENT_TABLE(TimeFunctionWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, TimeFunctionWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(TimeFunctionWidgetsPage, _T("TimeFunction"));

TimeFunctionWidgetsPage::TimeFunctionWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
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

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Time Function") );

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

    wxSizer *sizerForce = new wxStaticBoxSizer(wxVERTICAL, this, _T("Time functions for each dimension") );

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x(t):"),
                                            x_function_ValueText,
                                            &m_txt_x_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y(t):"),
                                            y_function_ValueText,
                                            &m_txt_y_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z(t):"),
                                            z_function_ValueText,
                                            &m_txt_z_function,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    // should be in sync with enums ButtonInterpolate_true(false)!
    static const wxString time_choice[] =
    {
        _T("0"),
        _T("now")
    };

    x_radio_zero_time = new wxRadioBox(this, wxID_ANY, _T("&x_zero_time"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(time_choice), time_choice);

    sizerForce->Add( x_radio_zero_time, 0, wxALL, 5 );

    y_radio_zero_time = new wxRadioBox(this, wxID_ANY, _T("&y_zero_time"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(time_choice), time_choice);

    sizerForce->Add( y_radio_zero_time, 0, wxALL, 5 );

    z_radio_zero_time = new wxRadioBox(this, wxID_ANY, _T("&z_zero_time"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(time_choice), time_choice);

    sizerForce->Add( z_radio_zero_time, 0, wxALL, 5 );



    sizerForce->SetMinSize( 200, 10 );

    sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5);

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void TimeFunctionWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_txt_x_function->SetValue(_T("sin(t)" ));
    m_txt_y_function->SetValue(_T("0" ));
    m_txt_z_function->SetValue(_T("0" ));
    x_radio_zero_time->SetSelection(zero_time_now); 
    use_x_zero = false;
    y_radio_zero_time->SetSelection(zero_time_now); 
    use_y_zero = false;
    z_radio_zero_time->SetSelection(zero_time_now); 
    use_z_zero = false;
}

void TimeFunctionWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
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

    switch ( x_radio_zero_time->GetSelection() )
    {
        case zero_time_zero:
          use_x_zero = true;
            break;

        default:
            wxFAIL_MSG(_T("unexpected radiobox selection"));
            // fall through

        case zero_time_now:
          use_x_zero = false;
            break;
    }

    switch ( y_radio_zero_time->GetSelection() )
    {
        case zero_time_zero:
          use_y_zero = true;
            break;

        default:
            wxFAIL_MSG(_T("unexpected radiobox selection"));
            // fall through

        case zero_time_now:
          use_y_zero = false;
            break;
    }

    switch ( z_radio_zero_time->GetSelection() )
    {
        case zero_time_zero:
          use_z_zero = true;
            break;

        default:
            wxFAIL_MSG(_T("unexpected radiobox selection"));
            // fall through

        case zero_time_now:
          use_z_zero = false;
            break;
    }
}

void TimeFunctionWidgetsPage::createForceEffect( ) {
  HAPITime current_time = H3DUtil::TimeStamp();
  HAPITime x_zero_time = 0, y_zero_time = 0, z_zero_time = 0;
  if( !use_x_zero )
    x_zero_time = current_time;
  if( !use_y_zero )
    y_zero_time = current_time;
  if( !use_z_zero )
    z_zero_time = current_time;

  x_function = new ParsedFunction();
  x_function->setFunctionString( (string)(m_txt_x_function->GetValue()), "t" );
  y_function = new ParsedFunction();
  y_function->setFunctionString( (string)(m_txt_y_function->GetValue()), "t" );
  z_function = new ParsedFunction();
  z_function->setFunctionString( (string)(m_txt_z_function->GetValue()), "t" );

  force_effect.reset( new HapticTimeFunctionEffect( Matrix4(), interpolate,
    x_function,
    y_function,
    z_function,
    x_zero_time,
    y_zero_time,
    z_zero_time ) );
    hd->addEffect( force_effect.get() );
    hd->clearEffects();
    hd->addEffect( force_effect.get() );
}

void TimeFunctionWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
