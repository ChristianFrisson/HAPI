#include "SpringWidgetsPage.h"
using namespace HAPI;

enum
{
  ButtonInterpolate_true,
  ButtonInterpolate_false,
  X_ValueText,
  Y_ValueText,
  Z_ValueText,
  spring_constant_ValueText
};

BEGIN_EVENT_TABLE(SpringWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, SpringWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(SpringWidgetsPage, _T("Spring"));

SpringWidgetsPage::SpringWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
                  : WidgetsPage(book, _hd)
{

    m_radioInterpolate = (wxRadioBox *)NULL;
    m_txt_positionX = NULL;
    m_txt_positionY = NULL;
    m_txt_positionZ = NULL;
    m_txt_spring_constant = NULL;

    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Spring Force") );

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

    wxSizer *sizer_position = new wxStaticBoxSizer(wxVERTICAL, this, _T("Position in meters") );

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            X_ValueText,
                                            &m_txt_positionX,
                                            this );
    sizer_position->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            Y_ValueText,
                                            &m_txt_positionY,
                                            this );
    sizer_position->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            Z_ValueText,
                                            &m_txt_positionZ,
                                            this );
    sizer_position->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerLeft->Add( sizer_position, 0, wxALL | wxGROW, 5);

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("spring constant:"),
                                            spring_constant_ValueText,
                                            &m_txt_spring_constant,
                                            this );

    sizerLeft->Add( sizerRow, 0, wxALL, 5 );

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void SpringWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_txt_positionX->SetValue(_T("0.0" ));
    position.x = 0.0;
    m_txt_positionY->SetValue(_T("0.0" ));
    position.y = 0.0;
    m_txt_positionZ->SetValue(_T("0.0" ));
    position.z = 0.0;
    m_txt_spring_constant->SetValue( _T("100"));
    spring_constant = 100 / 1000;
}

void SpringWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
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

void SpringWidgetsPage::createForceEffect( ) {
  double val;
  if( m_txt_positionX->GetValue().ToDouble(&val) ) {
    position.x = val * 1000;
  }

  if( m_txt_positionY->GetValue().ToDouble(&val) ) {
    position.y = val * 1000;
  }

  if( m_txt_positionZ->GetValue().ToDouble(&val) ) {
    position.z = val * 1000;
  }

  if( m_txt_spring_constant->GetValue().ToDouble(&val) ) {
    spring_constant = val / 1000;
  }

  force_effect.reset( new HapticSpring( 
    position,
    spring_constant,
    interpolate ) );
    hd->addEffect( force_effect.get() );
    hd->clearEffects();
    hd->addEffect( force_effect.get() );
}

void SpringWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
