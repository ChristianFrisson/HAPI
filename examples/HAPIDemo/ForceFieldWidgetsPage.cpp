#include "ForceFieldWidgetsPage.h"
using namespace HAPI;

enum
{
  ButtonInterpolate_true,
  ButtonInterpolate_false,
  X_ValueText,
  Y_ValueText,
  Z_ValueText
};

BEGIN_EVENT_TABLE(ForceFieldWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, ForceFieldWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(ForceFieldWidgetsPage, _T("ForceField"));

ForceFieldWidgetsPage::ForceFieldWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
                  : WidgetsPage(book, _hd)
{

    m_radioInterpolate = (wxRadioBox *)NULL;
    m_txt_forceX = NULL;
    m_txt_forceY = NULL;
    m_txt_forceZ = NULL;

    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Force Field values") );

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

    wxSizer *sizerForce = new wxStaticBoxSizer(wxVERTICAL, this, _T("Force") );

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            X_ValueText,
                                            &m_txt_forceX,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            Y_ValueText,
                                            &m_txt_forceY,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            Z_ValueText,
                                            &m_txt_forceZ,
                                            this );
    sizerForce->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerLeft->Add( sizerForce, 0, wxALL | wxGROW, 5);

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void ForceFieldWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_txt_forceX->SetValue(_T("1.0" ));
    force.x = 1.0;
    m_txt_forceY->SetValue(_T("0.0" ));
    force.y = 0.0;
    m_txt_forceZ->SetValue(_T("0.0" ));
    force.z = 0.0;
}

void ForceFieldWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
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

void ForceFieldWidgetsPage::createForceEffect( ) {
  double val;
  if( m_txt_forceX->GetValue().ToDouble(&val) ) {
    force.x = val;
  }

  if( m_txt_forceY->GetValue().ToDouble(&val) ) {
    force.y = val;
  }

  if( m_txt_forceZ->GetValue().ToDouble(&val) ) {
    force.z = val;
  }

  force_effect.reset( new HapticForceField( Matrix4(),
    force,
    interpolate ) );
    hd->addEffect( force_effect.get() );
    hd->clearEffects();
    hd->addEffect( force_effect.get() );
}

void ForceFieldWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
