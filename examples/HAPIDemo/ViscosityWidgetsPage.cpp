#include "ViscosityWidgetsPage.h"
using namespace HAPI;

enum
{
  ButtonInterpolate_true,
  ButtonInterpolate_false,
  Viscosity_ValueText,
  Radius_ValueText,
  Damping_Factor_ValueText
};

BEGIN_EVENT_TABLE(ViscosityWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, ViscosityWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(ViscosityWidgetsPage, _T("Viscosity"));

ViscosityWidgetsPage::ViscosityWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
                  : WidgetsPage(book, _hd)
{

    m_radioInterpolate = (wxRadioBox *)NULL;
    m_txt_viscosity = NULL;
    m_txt_radius = NULL;
    m_txt_damping_factor = NULL;

    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Viscosity effect values") );

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

    wxSizer *sizerForce = new wxStaticBoxSizer(wxVERTICAL, this, _T("Values in meter") );

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("viscosity:"),
                                            Viscosity_ValueText,
                                            &m_txt_viscosity,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("radius:"),
                                            Radius_ValueText,
                                            &m_txt_radius,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("damping factor:"),
                                            Damping_Factor_ValueText,
                                            &m_txt_damping_factor,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5);

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void ViscosityWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_txt_viscosity->SetValue(_T("0.01" ));
    viscosity = 0.01 / 1000;
    m_txt_radius->SetValue(_T("0.0025" ));
    radius = 0.0025 * 1000;
    m_txt_damping_factor->SetValue(_T("0.5" ));
    damping_factor = 0.5;
}

void ViscosityWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
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

void ViscosityWidgetsPage::createForceEffect( ) {
  double val;
  if( m_txt_viscosity->GetValue().ToDouble(&val) ) {
    viscosity = val / 1000;
  }

  if( m_txt_radius->GetValue().ToDouble(&val) ) {
    radius = val * 1000;
  }

  if( m_txt_damping_factor->GetValue().ToDouble(&val) ) {
    damping_factor = val;
  }

  force_effect.reset( new HapticViscosity( 
    viscosity,
    radius,
    damping_factor ) );
    hd->addEffect( force_effect.get() );
    hd->clearEffects();
    hd->addEffect( force_effect.get() );
}

void ViscosityWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
