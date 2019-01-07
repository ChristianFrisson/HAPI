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
/// \file HapticShapeConstraintWidgetsPage.cpp
/// \brief CPP file used to collect user input and create the force effect
/// HapticShapeConstraint found in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPIDemo includes
#include "HapticShapeConstraintWidgetsPage.h"

// wx includes
#include <wx/tokenzr.h>
#include <wx/msgdlg.h>

// HAPI includes
#include <HAPI/HapticPrimitive.h>
#include <HAPI/HapticLineSet.h>
#include <HAPI/HapticPointSet.h>
#include <HAPI/HapticTriangleSet.h>
#include <HAPI/HapticShapeConstraint.h>

using namespace HAPI;

// control ids
enum
{
  spring_constant_ValueText,
  sphere_radius_ValueText,
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

// control ids.
enum {
  Button_Sphere,
  Button_Triangle,
  Button_LineSet,
  Button_PointSet,
  Button_TriangleSet,
  Button_Plane
};

// Event table.
BEGIN_EVENT_TABLE( HapticShapeConstraintWidgetsPage, WidgetsPage )
  EVT_RADIOBOX( wxID_ANY, HapticShapeConstraintWidgetsPage::OnCheckOrRadioBox )
END_EVENT_TABLE()

// Macro to construct this page.
IMPLEMENT_WIDGETS_PAGE( HapticShapeConstraintWidgetsPage,
                        _T("Shape Constraints") );

HapticShapeConstraintWidgetsPage::HapticShapeConstraintWidgetsPage(
  wxBookCtrlBase *book,
  AnyHapticsDevice *_hd ) : WidgetsPage( book, _hd )
{
  m_radio_shapes = ( wxRadioBox * )NULL;
  panel_sizer = 0;

  sizerTop = new wxBoxSizer( wxHORIZONTAL );

  wxSizer *sizerLeft = new wxStaticBoxSizer( wxVERTICAL, this, _T("Shapes") );

  // should be in sync with enums Button_Sphere ( Triangle,
  //                                              LineSet, PointSet,
  //                                              TriangleSet, Plane )!
  static const wxString shapes[] =
  {
    _T("Sphere"),
    _T("Triangle"),
    _T("LineSet"),
    _T("PointSet"),
    _T("TriangleSet"),
    _T("Plane")
  };

  // RadioBox to choose which shape to constrain to.
  m_radio_shapes = new wxRadioBox( this, wxID_ANY, _T("&Shape"),
                                   wxDefaultPosition, wxDefaultSize,
                                   WXSIZEOF(shapes), shapes, 3,
                                   wxRA_SPECIFY_COLS );

  sizerLeft->Add( m_radio_shapes, 0, wxGROW | wxALL, 5 );

  sizerLeft->Add( 5, 5, 0, wxGROW | wxALL, 5 ); // spacer

  // Properties of the spring used to constrain to shape.
  wxSizer *sizerRow = CreateSizerWithTextAndLabel( _T("spring constant:"),
                                                   spring_constant_ValueText,
                                                   &m_txt_spring_constant,
                                                   this );

  sizerLeft->Add( sizerRow, 0, wxALL, 5 );

  rightPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition,
                            wxSize( 400, 100 ) );
  wxSizer *sizerMiddle = new wxBoxSizer( wxVERTICAL );
  sizerMiddle->SetMinSize( 100, 300 );
  sizerMiddle->Add( rightPanel, 1, wxALL | wxGROW, 0 );

  panel_sizer = new wxBoxSizer( wxVERTICAL );
  rightPanel->SetSizer( panel_sizer );

  // Properties for Sphere
  sphere_panel = new wxPanel( rightPanel, wxID_ANY,
                              wxDefaultPosition, wxSize( 100, 100 ) );
  wxSizer *sphere_content = new wxBoxSizer( wxVERTICAL );

  sphere_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

  sizerRow = CreateSizerWithTextAndLabel( _T("radius:"),
                                          sphere_radius_ValueText,
                                          &m_txt_sphere_radius,
                                          sphere_panel );
  sphere_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

  sphere_panel->SetSizer( sphere_content );
  // End of properties for Sphere

  // Properties for Triangle
  triangle_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition,
                                wxSize( 100, 100 ) );

  wxSizer *triangle_content = new wxBoxSizer( wxVERTICAL );

  triangle_content->Add( createXYZInputControls( triangle_panel,
                                                 _T("Vertex 1"),
                                                 triangle_vertex1X,
                                                 &m_txt_triangle_1X,
                                                 triangle_vertex1Y,
                                                 &m_txt_triangle_1Y,
                                                 triangle_vertex1Z,
                                                 &m_txt_triangle_1Z ),
                         0, wxALL, 5 );

  triangle_content->Add( createXYZInputControls( triangle_panel,
                                                 _T("Vertex 2"),
                                                 triangle_vertex2X,
                                                 &m_txt_triangle_2X,
                                                 triangle_vertex2Y,
                                                 &m_txt_triangle_2Y,
                                                 triangle_vertex2Z,
                                                 &m_txt_triangle_2Z ),
                         0, wxALL, 5 );

  triangle_content->Add( createXYZInputControls( triangle_panel,
                                                 _T("Vertex 3"),
                                                 triangle_vertex3X,
                                                 &m_txt_triangle_3X,
                                                 triangle_vertex3Y,
                                                 &m_txt_triangle_3Y,
                                                 triangle_vertex3Z,
                                                 &m_txt_triangle_3Z ),
                         0, wxALL, 5 );

  triangle_panel->SetSizer( triangle_content );

  triangle_panel->Hide();
  // End of properties for Triangle

  // Properties for LineSet
  lineSet_panel = new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition,
                               wxSize( 100, 100 ) );
  wxSizer *lineSet_content = new wxBoxSizer( wxVERTICAL );

  lineSet_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

  sizerRow = CreateSizerWithTextAndLabel( _T("points:"),
                                          line_set_points_ValueText,
                                          &m_txt_line_set_points,
                                          lineSet_panel );
  lineSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

  lineSet_panel->SetSizer( lineSet_content );
  lineSet_panel->Hide();
  // End of properties for LineSet

  // Properties for PointSet
  pointSet_panel =
    new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );
  wxSizer *pointSet_content = new wxBoxSizer( wxVERTICAL );

  pointSet_content->Add(5, 5, 0, wxGROW | wxALL, 5); // spacer

  sizerRow = CreateSizerWithTextAndLabel( _T("points:"),
                                          point_set_points_ValueText,
                                          &m_txt_point_set_points,
                                          pointSet_panel );
  pointSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

  pointSet_panel->SetSizer( pointSet_content );
  pointSet_panel->Hide();
  // End of properties for PointSet

  // Properties for TriangleSet
  triangleSet_panel =
    new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );
  wxSizer *triangleSet_content = new wxBoxSizer( wxVERTICAL );

  triangleSet_content->Add( 5, 5, 0, wxGROW | wxALL, 5 ); // spacer

  sizerRow = CreateSizerWithTextAndLabel( _T("triangles:"),
                                          triangle_set_triangles_ValueText,
                                          &m_txt_triangle_set_triangles,
                                          triangleSet_panel );
  triangleSet_content->Add( sizerRow, 0, wxALL | wxGROW, 5 );

  triangleSet_panel->SetSizer( triangleSet_content );
  triangleSet_panel->Hide();
  // End of properties for TriangleSet

  // Properties for Plane
  plane_panel =
    new wxPanel( rightPanel, wxID_ANY, wxDefaultPosition, wxSize( 100, 100 ) );

  wxSizer *plane_content = new wxBoxSizer( wxVERTICAL );

  plane_content->Add( createXYZInputControls( plane_panel,
                                                 _T("Plane point"),
                                                 plane_pointX,
                                                 &m_txt_plane_pointX,
                                                 plane_pointY,
                                                 &m_txt_plane_pointY,
                                                 plane_pointZ,
                                                 &m_txt_plane_pointZ ),
                         0, wxALL, 5 );

  plane_content->Add( createXYZInputControls( plane_panel,
                                              _T("Plane normal"),
                                              plane_normalX,
                                              &m_txt_plane_normalX,
                                              plane_normalY,
                                              &m_txt_plane_normalY,
                                              plane_normalZ,
                                              &m_txt_plane_normalZ ),
                         0, wxALL, 5 );

  plane_panel->SetSizer( plane_content );

  plane_panel->Hide();
  // End of properties for Plane

  sizerTop->Add( sizerLeft, 0, wxALL, 10 );
  sizerTop->Add( sizerMiddle, 1, wxALL, 10 );

  // final initializations
  Reset();

  SetSizer(sizerTop);

  sizerTop->Fit(this);
}

// reset all values in check boxes and such to default.
void HapticShapeConstraintWidgetsPage::Reset()
{
  // Spring constant
  m_txt_spring_constant->SetValue( _T("300") );
  spring_constant = 300;

  // Choose the sphere shape as default.
  m_radio_shapes->SetSelection( Button_Sphere );
  choosen_shape = Button_Sphere;
  panel_sizer->Add( sphere_panel, 0, wxALL | wxGROW, 0 );

  // Sphere defaults.
  m_txt_sphere_radius->SetValue( _T("0.05") );
  sphere_radius = 0.05;

  // Triangle defaults
  m_txt_triangle_1X->SetValue( _T("0.0") );
  triangle_vertex1.x = 0.0;
  m_txt_triangle_1Y->SetValue( _T("0.0") );
  triangle_vertex1.y = 0.0;
  m_txt_triangle_1Z->SetValue( _T("0.0") );
  triangle_vertex1.z = 0.0;

  m_txt_triangle_2X->SetValue( _T("0.05") );
  triangle_vertex2.x = 0.05;
  m_txt_triangle_2Y->SetValue( _T("0.0") );
  triangle_vertex2.y = 0.0;
  m_txt_triangle_2Z->SetValue( _T("0.0") );
  triangle_vertex2.z = 0.0;

  m_txt_triangle_3X->SetValue( _T("0.05") );
  triangle_vertex3.x = 0.05;
  m_txt_triangle_3Y->SetValue( _T("0.05") );
  triangle_vertex3.y = 0.05;
  m_txt_triangle_3Z->SetValue( _T("0.01") );
  triangle_vertex3.z = 0.01;

  // LineSet defaults
  m_txt_line_set_points->SetValue(_T("-0.1 0 0, 0.1 0 0"));
  if( !line_set_lines.empty() )
    line_set_lines.clear();
  line_set_lines.push_back( Collision::LineSegment(
                              Vec3( -0.1, 0, 0 ),
                              Vec3( 0.1, 0, 0 ) ) );

  // PointSet defaults
  m_txt_point_set_points->SetValue(_T("-0.05 0 0, 0.05 0 0"));
  if( !point_set_points.empty() )
    point_set_points.clear();
  point_set_points.push_back( Collision::Point( Vec3( -0.05, 0, 0 ) ) );
  point_set_points.push_back( Collision::Point( Vec3( 0.05, 0, 0 ) ) );

  // TriangleSet defaults
  m_txt_triangle_set_triangles->SetValue(
    _T("-0.05 0 0, 0.05 0 0, 0 0 -0.05, -0.05 0 0, 0.05 0 0, 0 0.05 0") );
  if( !triangle_set_triangles.empty() )
    triangle_set_triangles.clear();
  triangle_set_triangles.push_back(
    Collision::Triangle( Vec3( -0.05, 0, 0 ),
                         Vec3( 0.05, 0, 0 ),
                         Vec3( 0, 0, -0.05 ) ) );
  triangle_set_triangles.push_back(
    Collision::Triangle( Vec3( -0.05, 0, 0 ),
                         Vec3( 0.05, 0, 0 ),
                         Vec3( 0, 0.05, 0 ) ) );


  // Plane defaults
  m_txt_plane_pointX->SetValue( _T("0.0") );
  plane_point.x = 0.0;
  m_txt_plane_pointY->SetValue( _T("0.0") );
  plane_point.y = 0.0;
  m_txt_plane_pointZ->SetValue( _T("0.0") );
  plane_point.z = 0.0;

  m_txt_plane_normalX->SetValue( _T("0.0") );
  plane_normal.x = 0.0;
  m_txt_plane_normalY->SetValue( _T("1.0") );
  plane_normal.y = 1.0;
  m_txt_plane_normalZ->SetValue( _T("0.0") );
  plane_normal.z = 0.0;
}

void HapticShapeConstraintWidgetsPage::OnCheckOrRadioBox(
  wxCommandEvent& WXUNUSED(event) )
{

  switch ( m_radio_shapes->GetSelection() )
  {
    case Button_Sphere: 
    {
      choosen_shape = Button_Sphere;
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
    }

    case Button_Triangle:
    {
      choosen_shape = Button_Triangle;
      lineSet_panel->Hide();
      plane_panel->Hide();
      pointSet_panel->Hide();
      sphere_panel->Hide();
      triangleSet_panel->Hide();
      panel_sizer->Clear();
      panel_sizer->Add( triangle_panel, 0, wxALL | wxGROW, 0 );
      triangle_panel->Show();
      panel_sizer->Layout();
      sizerTop->Layout();
      break;
    }

    case Button_LineSet:
    {
      choosen_shape = Button_LineSet;
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
    }

    case Button_PointSet:
    {
      choosen_shape = Button_PointSet;
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
    }

    case Button_TriangleSet:
    {
      choosen_shape = Button_TriangleSet;
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
    }

    case Button_Plane:
    {
      choosen_shape = Button_Plane;
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

    default:
      wxFAIL_MSG( _T("unexpected radiobox selection") );
  }
}

// Creates the force effect choosen.
void HapticShapeConstraintWidgetsPage::createForceEffect( ) {
  double val;
  if( force_effect.get() ) {
    hd->clearEffects();
  }

  if( m_txt_spring_constant->GetValue().ToDouble(&val) ) {
    spring_constant = val;
  }
  
  // For the choosen shape a force effect is created and stored in
  // the force_effect variable.
  switch( choosen_shape ) {
    case Button_Sphere: {
      if( m_txt_sphere_radius->GetValue().ToDouble(&val) ) {
        sphere_radius = val;
      }

      force_effect.reset( new HapticShapeConstraint(
                            new Collision::Sphere( Vec3( 0, 0, 0 ),
                                                   sphere_radius  ),
                            spring_constant ) );
      break;
    }

    case Button_Triangle: {
      if( m_txt_triangle_1X->GetValue().ToDouble(&val) ) {
        triangle_vertex1.x = val;
      }
      if( m_txt_triangle_1Y->GetValue().ToDouble(&val) ) {
        triangle_vertex1.y = val;
      }
      if( m_txt_triangle_1Z->GetValue().ToDouble(&val) ) {
        triangle_vertex1.z = val;
      }
      if( m_txt_triangle_2X->GetValue().ToDouble(&val) ) {
        triangle_vertex2.x = val;
      }
      if( m_txt_triangle_2Y->GetValue().ToDouble(&val) ) {
        triangle_vertex2.y = val;
      }
      if( m_txt_triangle_2Z->GetValue().ToDouble(&val) ) {
        triangle_vertex2.z = val;
      }
      if( m_txt_triangle_3X->GetValue().ToDouble(&val) ) {
        triangle_vertex3.x = val;
      }
      if( m_txt_triangle_3Y->GetValue().ToDouble(&val) ) {
        triangle_vertex3.y = val;
      }
      if( m_txt_triangle_3Z->GetValue().ToDouble(&val) ) {
        triangle_vertex3.z = val;
      }
      force_effect.reset( new HapticShapeConstraint(
                            new Collision::Triangle( triangle_vertex1,
                                                     triangle_vertex2,
                                                     triangle_vertex3 ),
                            spring_constant ) );
      break;
    }

    case Button_LineSet: {
      wxString temp_string = m_txt_line_set_points->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      std::vector< HAPIFloat > line_values;
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
            line_set_lines.push_back( Collision::LineSegment(
              Vec3( line_values[i],
                    line_values[i+1],
                    line_values[i+2] ),
              Vec3( line_values[i+3],
                    line_values[i+4],
                    line_values[i+5] ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint(
                            new HapticLineSet( line_set_lines, 0 ),
                            spring_constant ) );
      break;
    }

    case Button_PointSet: {
      wxString temp_string = m_txt_point_set_points->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      std::vector< HAPIFloat > point_values;
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
            point_set_points.push_back(
              Collision::Point( Vec3( point_values[i],
                                      point_values[i+1],
                                      point_values[i+2] ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint(
                            new HapticPointSet( point_set_points, 0 ),
                            spring_constant ) );
      break;
    }

    case Button_TriangleSet: {
      wxString temp_string = m_txt_triangle_set_triangles->GetValue();
      wxStringTokenizer tkz( temp_string, wxT(","), wxTOKEN_STRTOK );
      std::vector< HAPIFloat > triangle_values;
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
            triangle_set_triangles.push_back(
              Collision::Triangle( Vec3( triangle_values[i],
                                         triangle_values[i+1],
                                         triangle_values[i+2] ),
                                   Vec3( triangle_values[i+3],
                                         triangle_values[i+4],
                                         triangle_values[i+5] ),
                                   Vec3( triangle_values[i+6],
                                         triangle_values[i+7],
                                         triangle_values[i+8] ) ) );
          }
        }
      }

      force_effect.reset( new HapticShapeConstraint(
                            new HapticTriangleSet( triangle_set_triangles, 0 ),
                            spring_constant ) );
      break;
    }

    case Button_Plane: {
      if( m_txt_plane_pointX->GetValue().ToDouble(&val) ) {
        plane_point.x = val;
      }
      if( m_txt_plane_pointY->GetValue().ToDouble(&val) ) {
        plane_point.y = val;
      }
      if( m_txt_plane_pointZ->GetValue().ToDouble(&val) ) {
        plane_point.z = val;
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
        force_effect.reset( new HapticShapeConstraint(
                            new Collision::Plane( plane_point,
                                                  plane_normal ),
                            spring_constant ) );
      break;
    }
  }

  if( force_effect.get() ) {
    hd->addEffect( force_effect.get() );
    hd->transferObjects();
  }
}

