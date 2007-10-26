#include "HapticShapeConstraintWidgetsPage.h"
#include <wx/tokenzr.h>
#include <wx/msgdlg.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/HapticLineSet.h>
#include <HAPI/HapticPointSet.h>
#include <HAPI/HapticTriangleSet.h>
using namespace HAPI;

enum
{
  ButtonInterpolate_true,
  ButtonInterpolate_false,
  spring_constant_ValueText,
  sphere_radius_ValueText,
  box_size_ValueTextX,
  box_size_ValueTextY,
  box_size_ValueTextZ,
  /*cone_bottomRadius_ValueText,
  cone_height_ValueText,*/
  cylinder_radius_ValueText,
  cylinder_height_ValueText,
  triangle_vertex1X,
  triangle_vertex1Y,
  triangle_vertex1Z,
  triangle_vertex2X,
  triangle_vertex2Y,
  triangle_vertex2Z,
  triangle_vertex3X,
  triangle_vertex3Y,
  triangle_vertex3Z,
  line_set_points_ValueText,
  point_set_points_ValueText,
  triangle_set_triangles_ValueText,
  plane_pointX,
  plane_pointY,
  plane_pointZ,
  plane_normalX,
  plane_normalY,
  plane_normalZ,
};

enum {
  Button_sphere,
  Button_box,
  //Button_cone,
  Button_cylinder,
  Button_triangle,
  Button_LineSet,
  Button_PointSet,
  Button_TriangleSet,
  Button_Plane
};

BEGIN_EVENT_TABLE(HapticShapeConstraintWidgetsPage, WidgetsPage)
    EVT_RADIOBOX(wxID_ANY, HapticShapeConstraintWidgetsPage::OnCheckOrRadioBox)
END_EVENT_TABLE()

IMPLEMENT_WIDGETS_PAGE(HapticShapeConstraintWidgetsPage, _T("Shape Constraints"));

HapticShapeConstraintWidgetsPage::HapticShapeConstraintWidgetsPage(wxBookCtrlBase *book, AnyHapticsDevice *_hd)
                  : WidgetsPage(book, _hd)
{

    m_radioInterpolate = (wxRadioBox *)NULL;
    m_radio_shapes = (wxRadioBox * )NULL;
    panel_sizer = 0;

    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *sizerLeft = new wxStaticBoxSizer(wxVERTICAL, this, _T("Shapes") );

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

    // should be in sync with enums ButtonInterpolate_true(false)!
    static const wxString shapes[] =
    {
        _T("Sphere"),
        _T("Box"),
        //_T("Cone"),
        _T("Cylinder"),
        _T("Triangle"),
        _T("LineSet"),
        _T("PointSet"),
        _T("TriangleSet"),
        _T("Plane")
    };

    m_radio_shapes = new wxRadioBox(this, wxID_ANY, _T("&Shape"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(shapes), shapes, 3, wxRA_SPECIFY_COLS );

    sizerLeft->Add(m_radio_shapes, 0, wxGROW | wxALL, 5);

    sizerLeft->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    wxSizer *sizerRow = CreateSizerWithTextAndLabel(
                                            _T("spring constant:"),
                                            spring_constant_ValueText,
                                            &m_txt_spring_constant,
                                            this );
    
    sizerLeft->Add( sizerRow, 0, wxALL, 5 );

    rightPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 400, 100 ) );
    wxSizer *sizerMiddle = new wxBoxSizer(wxVERTICAL);
    sizerMiddle->SetMinSize( 100, 300 );
    sizerMiddle->Add( rightPanel, 1, wxALL | wxGROW, 0 );

    panel_sizer = new wxBoxSizer( wxVERTICAL );
    rightPanel->SetSizer( panel_sizer );

    // Sphere
    sphere_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    wxSizer *sphere_content = new wxBoxSizer( wxVERTICAL );

    sphere_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    sizerRow = CreateSizerWithTextAndLabel(
      _T("radius:"),
      sphere_radius_ValueText,
      &m_txt_sphere_radius,
      sphere_panel );
    sphere_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sphere_panel->SetSizer( sphere_content );
    // End Sphere

    //Box        
    box_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );

    wxSizer *box_size_sizer = new wxStaticBoxSizer(wxVERTICAL, box_panel, _T("Box size") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            box_size_ValueTextX,
                                            &m_txt_box_sizeX,
                                            box_panel );
    box_size_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            box_size_ValueTextY,
                                            &m_txt_box_sizeY,
                                            box_panel );
    box_size_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            box_size_ValueTextZ,
                                            &m_txt_box_sizeZ,
                                            box_panel );
    box_size_sizer->Add( sizerRow, 0, wxALL, 5 );

    box_panel->SetSizer( box_size_sizer );

    box_panel->Hide();
    // End box

    // Cone
    //cone_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    //wxSizer *cone_content = new wxBoxSizer( wxVERTICAL );

    //cone_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    //sizerRow = CreateSizerWithTextAndLabel(
    //  _T("bottom radius:"),
    //  cone_bottomRadius_ValueText,
    //  &m_txt_cone_bottomRadius,
    //  cone_panel );
    //cone_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    //sizerRow = CreateSizerWithTextAndLabel(
    //  _T("height:"),
    //  cone_height_ValueText,
    //  &m_txt_cone_height,
    //  cone_panel );
    //cone_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    //cone_panel->SetSizer( cone_content );

    //cone_panel->Hide();
    // End Cone

    // Cylinder
    cylinder_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    wxSizer *cylinder_content = new wxBoxSizer( wxVERTICAL );

    cylinder_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    sizerRow = CreateSizerWithTextAndLabel(
      _T("radius:"),
      cylinder_radius_ValueText,
      &m_txt_cylinder_radius,
      cylinder_panel );
    cylinder_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
      _T("height:"),
      cylinder_height_ValueText,
      &m_txt_cylinder_height,
      cylinder_panel );
    cylinder_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    cylinder_panel->SetSizer( cylinder_content );

    cylinder_panel->Hide();
    // End Cylinder

    // Triangle
    triangle_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );

    wxSizer *triangle_content = new wxBoxSizer( wxVERTICAL );

    wxSizer *triangle_vertex_sizer = new wxStaticBoxSizer(wxVERTICAL, triangle_panel, _T("Vertex 1") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            triangle_vertex1X,
                                            &m_txt_triangle_1X,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            triangle_vertex1Y,
                                            &m_txt_triangle_1Y,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            triangle_vertex1Z,
                                            &m_txt_triangle_1Z,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    triangle_content->Add( triangle_vertex_sizer, 0, wxALL, 5 );
    
    triangle_vertex_sizer = new wxStaticBoxSizer(wxVERTICAL, triangle_panel, _T("Vertex 2") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            triangle_vertex2X,
                                            &m_txt_triangle_2X,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            triangle_vertex2Y,
                                            &m_txt_triangle_2Y,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            triangle_vertex2Z,
                                            &m_txt_triangle_2Z,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    triangle_content->Add( triangle_vertex_sizer, 0, wxALL, 5 );

    triangle_vertex_sizer = new wxStaticBoxSizer(wxVERTICAL, triangle_panel, _T("Vertex 3") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            triangle_vertex3X,
                                            &m_txt_triangle_3X,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            triangle_vertex3Y,
                                            &m_txt_triangle_3Y,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            triangle_vertex3Z,
                                            &m_txt_triangle_3Z,
                                            triangle_panel );
    triangle_vertex_sizer->Add( sizerRow, 0, wxALL, 5 );

    triangle_content->Add( triangle_vertex_sizer, 0, wxALL, 5 );

    triangle_panel->SetSizer( triangle_content );

    triangle_panel->Hide();
    // End Triangle

    // LineSet
    lineSet_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    wxSizer *lineSet_content = new wxBoxSizer( wxVERTICAL );

    lineSet_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    sizerRow = CreateSizerWithTextAndLabel(
      _T("points:"),
      line_set_points_ValueText,
      &m_txt_line_set_points,
      lineSet_panel );
    lineSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    lineSet_panel->SetSizer( lineSet_content );
    lineSet_panel->Hide();
    // End LineSet

    // PointSet
    pointSet_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    wxSizer *pointSet_content = new wxBoxSizer( wxVERTICAL );

    pointSet_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    sizerRow = CreateSizerWithTextAndLabel(
      _T("points:"),
      point_set_points_ValueText,
      &m_txt_point_set_points,
      pointSet_panel );
    pointSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    pointSet_panel->SetSizer( pointSet_content );
    pointSet_panel->Hide();
    // End PointSet

    // TriangleSet
    triangleSet_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ));
    wxSizer *triangleSet_content = new wxBoxSizer( wxVERTICAL );

    triangleSet_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

    sizerRow = CreateSizerWithTextAndLabel(
      _T("triangles:"),
      triangle_set_triangles_ValueText,
      &m_txt_triangle_set_triangles,
      triangleSet_panel );
    triangleSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

    triangleSet_panel->SetSizer( triangleSet_content );
    triangleSet_panel->Hide();
    // End TriangleSet

    // Plane
    plane_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );

    wxSizer *plane_content = new wxBoxSizer( wxVERTICAL );

    wxSizer *plane_point_sizer = new wxStaticBoxSizer(wxVERTICAL, plane_panel, _T("Plane point") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            plane_pointX,
                                            &m_txt_plane_pointX,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            plane_pointY,
                                            &m_txt_plane_pointY,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            plane_pointZ,
                                            &m_txt_plane_pointZ,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    plane_content->Add( plane_point_sizer, 0, wxALL, 5 );

    plane_point_sizer = new wxStaticBoxSizer(wxVERTICAL, plane_panel, _T("Plane normal") );
    
    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("x:"),
                                            plane_normalX,
                                            &m_txt_plane_normalX,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("y:"),
                                            plane_normalY,
                                            &m_txt_plane_normalY,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    sizerRow = CreateSizerWithTextAndLabel(
                                            _T("z:"),
                                            plane_normalZ,
                                            &m_txt_plane_normalZ,
                                            plane_panel );
    plane_point_sizer->Add( sizerRow, 0, wxALL, 5 );

    plane_content->Add( plane_point_sizer, 0, wxALL, 5 );

    plane_panel->SetSizer( plane_content );

    plane_panel->Hide();
    // End Plane

    sizerTop->Add(sizerLeft, 0, wxALL | wxGROW, 10);
    sizerTop->Add(sizerMiddle, 1, wxALL | wxEXPAND, 10);

    // final initializations
    Reset();

    SetSizer(sizerTop);

    sizerTop->Fit(this);
}

void HapticShapeConstraintWidgetsPage::Reset()
{
    m_radioInterpolate->SetSelection(ButtonInterpolate_false);
    interpolate = false;
    m_radio_shapes->SetSelection( Button_sphere );
    choosen_shape = Button_sphere;
    panel_sizer->Add( sphere_panel, 0, wxALL | wxGROW, 0 );

    m_txt_spring_constant->SetValue( _T("300") );
    spring_constant = 300 / 1000;
    m_txt_sphere_radius->SetValue( _T("0.05") );
    sphere_radius = 0.05 * 1000;

    m_txt_box_sizeX->SetValue( _T("0.05") );
    box_size.x = 0.05 * 1000;
    m_txt_box_sizeY->SetValue( _T("0.05") );
    box_size.y = 0.05 * 1000;
    m_txt_box_sizeZ->SetValue( _T("0.05") );
    box_size.z = 0.05 * 1000;

    /*m_txt_cone_bottomRadius->SetValue( _T("0.05") );
    cone_bottom_radius = 0.05;
    m_txt_cone_height->SetValue( _T("0.05") );
    cone_height = 0.05;*/

    m_txt_cylinder_radius->SetValue( _T("0.02") );
    cylinder_radius = 0.05;
    m_txt_cylinder_height->SetValue( _T("0.08") );
    cylinder_height = 0.05;

    m_txt_triangle_1X->SetValue( _T("0.0") );
    triangle_vertex1.x = 0.0 * 1000;
    m_txt_triangle_1Y->SetValue( _T("0.0") );
    triangle_vertex1.y = 0.0 * 1000;
    m_txt_triangle_1Z->SetValue( _T("0.0") );
    triangle_vertex1.z = 0.0 * 1000;

    m_txt_triangle_2X->SetValue( _T("0.05") );
    triangle_vertex2.x = 0.05 * 1000;
    m_txt_triangle_2Y->SetValue( _T("0.0") );
    triangle_vertex2.y = 0.0 * 1000;
    m_txt_triangle_2Z->SetValue( _T("0.0") );
    triangle_vertex2.z = 0.0 * 1000;

    m_txt_triangle_3X->SetValue( _T("0.05") );
    triangle_vertex3.x = 0.05 * 1000;
    m_txt_triangle_3Y->SetValue( _T("0.05") );
    triangle_vertex3.y = 0.05 * 1000;
    m_txt_triangle_3Z->SetValue( _T("0.01") );
    triangle_vertex3.z = 0.01 * 1000;

    m_txt_line_set_points->SetValue("-0.1 0 0, 0.1 0 0");
    if( !line_set_lines.empty() )
      line_set_lines.clear();
    line_set_lines.push_back( Collision::LineSegment( Vec3( -0.1 * 1000, 0, 0 ), Vec3( 0.1 * 1000, 0, 0 ) ) );

    m_txt_point_set_points->SetValue("-0.05 0 0, 0.05 0 0");
    if( !point_set_points.empty() )
      point_set_points.clear();
    point_set_points.push_back( Collision::Point( Vec3( -0.05 * 1000, 0, 0 ) ) );
    point_set_points.push_back( Collision::Point( Vec3( 0.05 * 1000, 0, 0 ) ) );

    m_txt_triangle_set_triangles->SetValue("-0.05 0 0, 0.05 0 0, 0 0 -0.05, -0.05 0 0, 0.05 0 0, 0 0.05 0");
    if( !triangle_set_triangles.empty() )
      triangle_set_triangles.clear();
    triangle_set_triangles.push_back( Collision::Triangle( Vec3( -0.05 * 1000, 0, 0 ), Vec3( 0.05 * 1000, 0, 0 ), Vec3( 0, 0, -0.05 ) ) );
    triangle_set_triangles.push_back( Collision::Triangle( Vec3( -0.05 * 1000, 0, 0 ), Vec3( 0.05 * 1000, 0, 0 ), Vec3( 0, 0.05 * 1000, 0 ) ) );


    m_txt_plane_pointX->SetValue( _T("0.0") );
    plane_point.x = 0.0 * 1000;
    m_txt_plane_pointY->SetValue( _T("0.0") );
    plane_point.y = 0.0 * 1000;
    m_txt_plane_pointZ->SetValue( _T("0.0") );
    plane_point.z = 0.0 * 1000;

    m_txt_plane_normalX->SetValue( _T("0.0") );
    plane_normal.x = 0.0;
    m_txt_plane_normalY->SetValue( _T("1.0") );
    plane_normal.y = 1.0;
    m_txt_plane_normalZ->SetValue( _T("0.0") );
    plane_normal.z = 0.0;
}

void HapticShapeConstraintWidgetsPage::OnCheckOrRadioBox(wxCommandEvent& WXUNUSED(event))
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

  switch ( m_radio_shapes->GetSelection() )
  {
  case Button_sphere:
    choosen_shape = Button_sphere;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( sphere_panel, 0, wxALL | wxGROW, 0 );
    sphere_panel->Show();
    panel_sizer->Layout();
    break;

  default:
    wxFAIL_MSG(_T("unexpected radiobox selection"));
    // fall through

  case Button_box:
    choosen_shape = Button_box;
    sphere_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( box_panel, 0, wxALL | wxGROW, 0 );
    box_panel->Show();
    panel_sizer->Layout();  
    break;
  

 /* case Button_cone:
    choosen_shape = Button_cone;
    box_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( cone_panel, 0, wxALL | wxGROW, 0 );
    cone_panel->Show();
    panel_sizer->Layout();  
    break;*/

  case Button_cylinder:
    choosen_shape = Button_cylinder;
    box_panel->Hide();
    //cone_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( cylinder_panel, 0, wxALL | wxGROW, 0 );
    cylinder_panel->Show();
    panel_sizer->Layout();  
    break;

  case Button_triangle:
    choosen_shape = Button_triangle;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    sphere_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( triangle_panel, 0, wxALL | wxGROW, 0 );
    triangle_panel->Show();
    panel_sizer->Layout();  
    break;

  case Button_LineSet:
    choosen_shape = Button_LineSet;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    plane_panel->Hide();
    pointSet_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( lineSet_panel, 0, wxALL | wxGROW, 0 );
    lineSet_panel->Show();
    panel_sizer->Layout();  
    break;

  case Button_PointSet:
    choosen_shape = Button_PointSet;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    triangleSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( pointSet_panel, 0, wxALL | wxGROW, 0 );
    pointSet_panel->Show();
    panel_sizer->Layout();
    break;

  case Button_TriangleSet:
    choosen_shape = Button_TriangleSet;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    plane_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    pointSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( triangleSet_panel, 0, wxALL | wxGROW, 0 );
    triangleSet_panel->Show();
    panel_sizer->Layout();
    break;

  case Button_Plane:
    choosen_shape = Button_Plane;
    box_panel->Hide();
    //cone_panel->Hide();
    cylinder_panel->Hide();
    lineSet_panel->Hide();
    triangleSet_panel->Hide();
    sphere_panel->Hide();
    triangle_panel->Hide();
    pointSet_panel->Hide();
    panel_sizer->Clear();
    panel_sizer->Add( plane_panel, 0, wxALL | wxGROW, 0 );
    plane_panel->Show();
    panel_sizer->Layout();
    break;
  }
}

void HapticShapeConstraintWidgetsPage::createForceEffect( ) {
  double val;
  if( force_effect.get() ) {
    hd->clearEffects();
  }

  if( m_txt_spring_constant->GetValue().ToDouble(&val) ) {
    spring_constant = val / 1000;
  }
  
  switch( choosen_shape ) {
    case Button_sphere: {
      if( m_txt_sphere_radius->GetValue().ToDouble(&val) ) {
        sphere_radius = val * 1000;
      }
	  force_effect.reset( new HapticShapeConstraint( new HapticPrimitive( Matrix4(), new Collision::Sphere( Vec3( 0, 0, 0 ), sphere_radius ), 0 ), spring_constant ) );
      break;
    }
    /*case Button_box: {
      if( m_txt_box_sizeX->GetValue().ToDouble(&val) ) {
        box_size.x = val * 1000;
      }
      if( m_txt_box_sizeY->GetValue().ToDouble(&val) ) {
        box_size.y = val * 1000;
      }
      if( m_txt_box_sizeZ->GetValue().ToDouble(&val) ) {
        box_size.z = val * 1000;
      }
      force_effect.reset( new HapticShapeConstraint( Matrix4(), interpolate, new HapticBox( box_size, 0, 0, Matrix4() ), spring_constant ) );
      break;
    }
     case Button_cone: {
      if( m_txt_cone_bottomRadius->GetValue().ToDouble(&val) ) {
        cone_bottom_radius = val * 1000;
      }
      if( m_txt_cone_height->GetValue().ToDouble(&val) ) {
        cone_height = val * 1000;
      }
      force_effect.reset( new HapticShapeConstraint( Matrix4(), interpolate, new HapticCone( cone_bottom_radius, cone_height, false, 0, 0, Matrix4() ), spring_constant ) );
      break;
    }
    case Button_cylinder: {
      if( m_txt_cylinder_radius->GetValue().ToDouble(&val) ) {
        cylinder_radius = val * 1000;
      }
      if( m_txt_cylinder_height->GetValue().ToDouble(&val) ) {
        cylinder_height = val * 1000;
      }
      force_effect.reset( new HapticShapeConstraint( Matrix4(), interpolate, new HapticCylinder( cylinder_height, cylinder_radius, 0, 0, Matrix4() ), spring_constant ) );
      break;
    }

    case Button_triangle: {
      if( m_txt_triangle_1X->GetValue().ToDouble(&val) ) {
        triangle_vertex1.x = val * 1000;
      }
      if( m_txt_triangle_1Y->GetValue().ToDouble(&val) ) {
        triangle_vertex1.y = val * 1000;
      }
      if( m_txt_triangle_1Z->GetValue().ToDouble(&val) ) {
        triangle_vertex1.z = val * 1000;
      }
      if( m_txt_triangle_2X->GetValue().ToDouble(&val) ) {
        triangle_vertex2.x = val * 1000;
      }
      if( m_txt_triangle_2Y->GetValue().ToDouble(&val) ) {
        triangle_vertex2.y = val * 1000;
      }
      if( m_txt_triangle_2Z->GetValue().ToDouble(&val) ) {
        triangle_vertex2.z = val * 1000;
      }
      if( m_txt_triangle_3X->GetValue().ToDouble(&val) ) {
        triangle_vertex3.x = val * 1000;
      }
      if( m_txt_triangle_3Y->GetValue().ToDouble(&val) ) {
        triangle_vertex3.y = val * 1000;
      }
      if( m_txt_triangle_3Z->GetValue().ToDouble(&val) ) {
        triangle_vertex3.z = val * 1000;
      }
      force_effect.reset( new HapticShapeConstraint( Matrix4(), interpolate, new HapticTriangle( Collision::Triangle( triangle_vertex1, triangle_vertex2, triangle_vertex3 ), 0, 0, Matrix4() ), spring_constant ) );
      break;
    }
*/
    case Button_LineSet: {
      wxString temp_string = m_txt_line_set_points->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      vector< HAPIFloat > line_values;
      while ( tkz.HasMoreTokens() )
      {
        wxString token = tkz.GetNextToken();
        wxStringTokenizer vec3_tkz( token, wxT(" "), wxTOKEN_STRTOK );
        while( vec3_tkz.HasMoreTokens() ) {
          wxString value = vec3_tkz.GetNextToken();
          if( value.ToDouble(&val) ) {
            line_values.push_back( val );
          }
        }
      }

      if( !line_values.empty() && line_values.size() > 5 ) {
        line_set_lines.clear();
        for( unsigned int i = 0; i < line_values.size(); i += 3 ) {
          if( line_values.size() - i > 5 ) {
            line_set_lines.push_back( Collision::LineSegment( Vec3( line_values[i] * 1000, line_values[i+1] * 1000, line_values[i+2] * 1000 ), Vec3( line_values[i+3] * 1000, line_values[i+4] * 1000, line_values[i+5] * 1000 ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint( new HapticLineSet( line_set_lines, 0 ), spring_constant ) );
      break;
    }

    case Button_PointSet: {
      wxString temp_string = m_txt_point_set_points->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      vector< HAPIFloat > point_values;
      while ( tkz.HasMoreTokens() )
      {
        wxString token = tkz.GetNextToken();
        wxStringTokenizer vec3_tkz( token, wxT(" "), wxTOKEN_STRTOK );
        while( vec3_tkz.HasMoreTokens() ) {
          wxString value = vec3_tkz.GetNextToken();
          if( value.ToDouble(&val) ) {
            point_values.push_back( val );
          }
        }
      }

      if( !point_values.empty() && point_values.size() > 2 ) {
        point_set_points.clear();
        for( unsigned int i = 0; i < point_values.size(); i += 3 ) {
          if( point_values.size() - i > 2 ) {
            point_set_points.push_back( Collision::Point( Vec3( point_values[i] * 1000, point_values[i+1] * 1000, point_values[i+2] * 1000 ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint( new HapticPointSet( point_set_points, 0 ), spring_constant ) );
      break;
    }

    case Button_TriangleSet: {
      wxString temp_string = m_txt_triangle_set_triangles->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      vector< HAPIFloat > triangle_values;
      while ( tkz.HasMoreTokens() )
      {
        wxString token = tkz.GetNextToken();
        wxStringTokenizer vec3_tkz( token, wxT(" "), wxTOKEN_STRTOK );
        while( vec3_tkz.HasMoreTokens() ) {
          wxString value = vec3_tkz.GetNextToken();
          if( value.ToDouble(&val) ) {
            triangle_values.push_back( val );
          }
        }
      }

      if( !triangle_values.empty() && triangle_values.size() > 8 ) {
        triangle_set_triangles.clear();
        for( unsigned int i = 0; i < triangle_values.size(); i += 9 ) {
          if( triangle_values.size() - i > 8 ) {
            triangle_set_triangles.push_back( Collision::Triangle( Vec3( triangle_values[i] * 1000, triangle_values[i+1] * 1000, triangle_values[i+2] * 1000 ),
                                                                Vec3( triangle_values[i+3] * 1000, triangle_values[i+4] * 1000, triangle_values[i+5] * 1000 ),
                                                                Vec3( triangle_values[i+6] * 1000, triangle_values[i+7] * 1000, triangle_values[i+8] * 1000 ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint( new HapticTriangleSet( triangle_set_triangles, 0 ), spring_constant ) );
      break;
    }

    case Button_Plane: {
      if( m_txt_plane_pointX->GetValue().ToDouble(&val) ) {
        plane_point.x = val * 1000;
      }
      if( m_txt_plane_pointY->GetValue().ToDouble(&val) ) {
        plane_point.y = val * 1000;
      }
      if( m_txt_plane_pointZ->GetValue().ToDouble(&val) ) {
        plane_point.z = val * 1000;
      }
      if( m_txt_plane_normalX->GetValue().ToDouble(&val) ) {
        plane_normal.x = val;
      }
      if( m_txt_plane_normalY->GetValue().ToDouble(&val) ) {
        plane_normal.y = val;
      }
      if( m_txt_plane_normalZ->GetValue().ToDouble(&val) ) {
        plane_normal.z = val;
      }

      plane_normal.normalizeSafe();

      if( plane_normal.lengthSqr() > Constants::epsilon )
		  force_effect.reset( new HapticShapeConstraint( new HapticPrimitive( Matrix4(), new Collision::Plane( plane_point, plane_normal ), 0 ), spring_constant ) );
      break;
    }
  }

  if( force_effect.get() ) {
    hd->addEffect( force_effect.get() );
  }
}

void HapticShapeConstraintWidgetsPage::removeForceEffect() {
  if( force_effect.get() ) {
    hd->removeEffect( force_effect.get() );
  }
}
