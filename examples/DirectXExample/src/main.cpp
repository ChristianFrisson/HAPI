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
/// \file main.cpp
/// \brief CPP file for all the code of example that connects DirectX and HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////
// DirectX code is based on the tutorial found on this site.
// http://www.riemers.net/eng/Tutorials/dxcpp.php

// In this example we pretend that the world coordinates are given in m.
// Therefore no transformation will be done when sending vertex information to
// HAPI.
// Note that there is no co-location in this example.
#include<windows.h>

// directX includes
#include<d3d9.h> 
#include<d3dx9.h>

// HAPI includes
#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/GodObjectRenderer.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/HapticPrimitive.h>

// Used to keep main loop running.
bool is_app_running = true;

// Struct specifying vertex data for DirectX.
struct MyCustomVertex
{
  float x, y, z;
  DWORD color;
};

// Window callback procedure. Used to capture events.
LRESULT CALLBACK myWindowProcedure( HWND window_handle,
                                    UINT msg,
                                    WPARAM w_param,
                                    LPARAM l_param )
{
  switch( msg )
  {
    case WM_KEYDOWN:
    case WM_CLOSE:
    {
      // Set to false means that the main loop will exit.
      is_app_running = false;
      break;
    }
    break;
  }

  // Send message to default windows procedure.
  return DefWindowProc( window_handle, msg, w_param, l_param );
}

// Set up a window using Windows functions.
HWND newWindow( LPCTSTR title, int x_pos, int y_pos, int width, int height )
{
  WNDCLASSEX wnd_struct;

  wnd_struct.cbSize = sizeof(WNDCLASSEX);
  wnd_struct.style = CS_HREDRAW | CS_VREDRAW;
  wnd_struct.lpfnWndProc = myWindowProcedure;
  wnd_struct.cbClsExtra = 0;
  wnd_struct.cbWndExtra = 0;
  wnd_struct.hInstance = GetModuleHandle(NULL);
  wnd_struct.hIcon = NULL;
  wnd_struct.hCursor = NULL;
  wnd_struct.hbrBackground = GetSysColorBrush(COLOR_BTNFACE);
  wnd_struct.lpszMenuName = NULL;
  wnd_struct.lpszClassName = "WindowClassName";
  wnd_struct.hIconSm = LoadIcon(NULL,IDI_APPLICATION);

  RegisterClassEx( &wnd_struct );
 
  return CreateWindowEx( WS_EX_CONTROLPARENT, "WindowClassName", title,
                         WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU |
                         WS_MINIMIZEBOX | WS_VISIBLE, x_pos, y_pos,
                         width, height, NULL, NULL,
                         GetModuleHandle(NULL), NULL );
}

// Initialize directX device which is an object that gives access to
// the hardware on the system.
LPDIRECT3DDEVICE9 initializeDirectXDevice( HWND window_handle )
{
  // The top level directX object.
  LPDIRECT3D9 p_dx_object;
  p_dx_object = Direct3DCreate9( D3D_SDK_VERSION );

  // Check if DirectX runtime is installed.
  if ( p_dx_object == NULL )
  {
    MessageBox( window_handle, "DirectX Runtime library not installed!",
                "initializeDirectXDevice()", MB_OK );
  }

  // Desired properties of the directX device.
  D3DPRESENT_PARAMETERS dx_pres_params;
  ZeroMemory( &dx_pres_params, sizeof( dx_pres_params ) );
  dx_pres_params.Windowed = TRUE;
  dx_pres_params.SwapEffect = D3DSWAPEFFECT_DISCARD;
  dx_pres_params.BackBufferFormat = D3DFMT_UNKNOWN;

  // Try to create the device. First using hardware acceleration.
  // After that try getting software device.
  LPDIRECT3DDEVICE9 p_dx_device;
 
  if( FAILED( p_dx_object->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL,
                                         window_handle,
                                         D3DCREATE_HARDWARE_VERTEXPROCESSING,
                                         &dx_pres_params, &p_dx_device ) ) )
  {
    // No hardware acceleration, try software.
    if ( FAILED( p_dx_object->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_REF,
                                            window_handle,
                                           D3DCREATE_SOFTWARE_VERTEXPROCESSING,
                                            &dx_pres_params, &p_dx_device ) ) )
    {
      MessageBox( window_handle,
                  "Failed to create even the reference device!",
                  "initializeDirectXDevice()", MB_OK );
    }
  }

  // return aquired device.
  return p_dx_device;
}

// draw the stylus as a white point at position.
void drawStylus( HWND window_handle,
                 LPDIRECT3DDEVICE9 p_dx_device,
                 LPDIRECT3DVERTEXBUFFER9 stylus_vertex_buffer,
                 HAPI::Vec3 &position ) {

  // Temporary vertex array.
  MyCustomVertex stylus_vertex[1];
  stylus_vertex[0].x = (float)position.x;
  stylus_vertex[0].y = (float)position.y;
  stylus_vertex[0].z = (float)position.z;
  stylus_vertex[0].color = 0xffffffff;

  // Lock memory. If succeded copy position (and color) data to memory.
  VOID* p_vertices;
  if( FAILED( stylus_vertex_buffer->Lock( 0, sizeof(MyCustomVertex),
                                          (void**)&p_vertices, 0 ) ) )
  {
    MessageBox( window_handle, "Error trying to lock", "drawStylus()", MB_OK );
  } else {
    memcpy( p_vertices, stylus_vertex, sizeof(MyCustomVertex) );
    stylus_vertex_buffer->Unlock();
  }

  // Tell directX where to find data.
  p_dx_device->SetStreamSource( 0, stylus_vertex_buffer, 0,
                                sizeof( MyCustomVertex ) );
  // Indicate what kind of data directX should find in the source.
  p_dx_device->SetFVF( D3DFVF_XYZ|D3DFVF_DIFFUSE );
  // Draw a pointlist. Start at 0, finish after 1 point is drawn.
  p_dx_device->DrawPrimitive( D3DPT_POINTLIST, 0, 1 );
}

// draw the scene.
void drawScene( HWND window_handle,
                LPDIRECT3DDEVICE9 p_dx_device,
                LPDIRECT3DVERTEXBUFFER9 p_dx_vertex_buffer,
                LPDIRECT3DVERTEXBUFFER9 stylus_vertex_buffer,
                HAPI::AnyHapticsDevice &hd )
{
  // Clear the screen
  p_dx_device->Clear( 0, NULL, D3DCLEAR_TARGET,
                      D3DCOLOR_XRGB(0,0,0), 1.0f, 0 );
  // Begin scene
  p_dx_device->BeginScene();

  HAPI::Vec3 position;
  HAPI::HAPIProxyBasedRenderer *proxy_renderer = 
    dynamic_cast< HAPI::HAPIProxyBasedRenderer * >( hd.getHapticsRenderer() );
  if( proxy_renderer ) {
    position = proxy_renderer->getProxyPosition();
  } else {
    position = hd.getPosition();
  }

  if( position.z >= 0 ) {
    p_dx_device->SetStreamSource( 0, p_dx_vertex_buffer, 0,
                                sizeof( MyCustomVertex ) );
    p_dx_device->SetFVF( D3DFVF_XYZ|D3DFVF_DIFFUSE );
    p_dx_device->DrawPrimitive( D3DPT_TRIANGLELIST, 0, 1 );

    drawStylus( window_handle, p_dx_device, stylus_vertex_buffer, position );
  } else {
    drawStylus( window_handle, p_dx_device, stylus_vertex_buffer, position );

    p_dx_device->SetStreamSource( 0, p_dx_vertex_buffer, 0,
                                sizeof( MyCustomVertex ) );
    p_dx_device->SetFVF( D3DFVF_XYZ|D3DFVF_DIFFUSE );
    p_dx_device->DrawPrimitive( D3DPT_TRIANGLELIST, 0, 1 );
  }
 
  // End scene.
  p_dx_device->EndScene();
  // Presents the display with the contents of the next buffer in the sequence
  // of back buffers owned by the device. In this case it draws to the window
  // in the D3DPRESENT_PARAMETERS struct in initializeDirectXDevice.
  p_dx_device->Present( NULL, NULL, NULL, NULL );
}

// Create buffer used to render stylus. Need to be updated with position
// information each frame and is therefore not filled here.
LPDIRECT3DVERTEXBUFFER9 createStylusVexterBuffer( HWND window_handle,
                                               LPDIRECT3DDEVICE9 p_dx_device )
{
  // buffer to return.
  LPDIRECT3DVERTEXBUFFER9 p_dx_vertex_buffer;

  // Try to create buffer.
  if( FAILED( p_dx_device->CreateVertexBuffer( sizeof(MyCustomVertex), 0,
                                               D3DFVF_XYZ|D3DFVF_DIFFUSE,
                                               D3DPOOL_DEFAULT,
                                               &p_dx_vertex_buffer, NULL ) ) )
  {
    MessageBox( window_handle, "Error while creating VertexBuffer for stylus",
                "fillTriangleVertices()", MB_OK );
  }

  return p_dx_vertex_buffer;
}

// Create buffer used for triangle information. This info will not change
// through the lifetime of the scene. Therefore the vertex information is
// copied into memory right away.
// The parameter tri_vertices will be filled with vertex information to
// be able to send this information to HAPI later.
LPDIRECT3DVERTEXBUFFER9 fillTriangleVertices( HWND window_handle,
                                              LPDIRECT3DDEVICE9 p_dx_device,
                                              MyCustomVertex *tri_vertices )
{
  // Vertices of the triangle.
  tri_vertices[0].x = -0.03f;
  tri_vertices[0].y = -0.02f;
  tri_vertices[0].z = 0.0f;
  tri_vertices[0].color = 0xffff0000;

  tri_vertices[1].x = -0.03f;
  tri_vertices[1].y = 0.02f;
  tri_vertices[1].z = 0.0f;
  tri_vertices[1].color = 0xff00ff00;

  tri_vertices[2].x = 0.02f;
  tri_vertices[2].y = 0.0f;
  tri_vertices[2].z = 0.0f;
  tri_vertices[2].color = 0xff00ffff;

  // Try to create buffer.
  LPDIRECT3DVERTEXBUFFER9 p_dx_vertex_buffer;
  if( FAILED( p_dx_device->CreateVertexBuffer( 3*sizeof(MyCustomVertex), 0,
                                               D3DFVF_XYZ|D3DFVF_DIFFUSE,
                                               D3DPOOL_DEFAULT,
                                               &p_dx_vertex_buffer, NULL ) ) )
  {
    MessageBox( window_handle, "Error while creating VertexBuffer",
                "fillTriangleVertices()", MB_OK );
  }

  // Fill memory with vertex information.
  VOID* p_vertices;
  if( FAILED( p_dx_vertex_buffer->Lock( 0, 3*sizeof(MyCustomVertex),
                                       (void**)&p_vertices, 0 ) ) )
  {
    MessageBox( window_handle, "Error trying to lock",
                "fillTriangleVertices()", MB_OK );
  } else {
    memcpy( p_vertices, tri_vertices, 3*sizeof(MyCustomVertex) );
    p_dx_vertex_buffer->Unlock();
  }

  // return buffer.
  return p_dx_vertex_buffer;
}

// help function to convert from MyCustomVertex to HAPI::Vec3.
inline HAPI::Vec3 myCustomVertex2Vec3f( const MyCustomVertex & cv ) {
  return HAPI::Vec3( cv.x, cv.y, cv.z );
}

// Set up camera for the scene.
void setUpCamera( LPDIRECT3DDEVICE9 p_dx_device )
{
  D3DXVECTOR3 eye_pos(0, 0, -0.09f);
  D3DXVECTOR3 target_pos(0, 0, 0);
  D3DXVECTOR3 up_vector(0, 1, 0);
  D3DXMATRIXA16 view_matrix;
  D3DXMatrixLookAtLH( &view_matrix, &eye_pos, &target_pos, &up_vector );
  p_dx_device->SetTransform(D3DTS_VIEW, &view_matrix);

  D3DXMATRIX proj_matrix;
  // The parameters are.
  // matrix to create, field of view, window ratio, near_plane, far_plane
  D3DXMatrixPerspectiveFovLH( &proj_matrix, D3DX_PI/4, 500/500, 0, 500 );
  p_dx_device->SetTransform( D3DTS_PROJECTION, &proj_matrix );

  p_dx_device->SetRenderState( D3DRS_LIGHTING, false );
}

// main function
int WINAPI WinMain( HINSTANCE hInstance,
                    HINSTANCE hPreviousInstance,
                    LPSTR lpcmdline,
                    int nCmdShow )
{
  // Init window.
  HWND window_handle = newWindow( "DirectX and HAPI", 100, 100, 500, 500 );
  // Init DirectX device
  LPDIRECT3DDEVICE9 p_device = initializeDirectXDevice( window_handle );
  // set up camera
  setUpCamera( p_device );

  // Create two buffers.
  MyCustomVertex tri_vertices[3];
  LPDIRECT3DVERTEXBUFFER9 tri_dx_vb = fillTriangleVertices( window_handle,
                                                            p_device,
                                                            tri_vertices );
  LPDIRECT3DVERTEXBUFFER9 stylus_vb = createStylusVexterBuffer( window_handle,
                                                                p_device );
  // Get a connected device.
  HAPI::AnyHapticsDevice hd;
  
  // The haptics renderer to use.
  hd.setHapticsRenderer( new HAPI::GodObjectRenderer() );

  // Init the device.
  if( hd.initDevice() != HAPI::HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    system("PAUSE");
    return 0;
  }
  // Enable the device
  hd.enableDevice();

  // Creating a default surface.
  HAPI::HAPISurfaceObject * my_surface = new HAPI::FrictionSurface();
  // Creating a triangle with the information in tri_vertices.
  HAPI::HapticPrimitive *my_haptic_triangle =
    new HAPI::HapticPrimitive( new HAPI::Collision::Triangle(
                                 myCustomVertex2Vec3f( tri_vertices[0] ),
                                 myCustomVertex2Vec3f( tri_vertices[1] ),
                                 myCustomVertex2Vec3f( tri_vertices[2] ) ),
                               my_surface );

  // Add the shape to be rendered on the device.
  hd.addShape( my_haptic_triangle );

  // Transfer objects (shapes) to the haptics loop.
  hd.transferObjects();

  
  // Loop until is_app_running is set to false which happens when
  // the window closed or keyboard button pressed.
  MSG msg; 
  while(is_app_running)
  {
    if( PeekMessage( &msg, window_handle, 0, 0, PM_REMOVE ) )
    {
      if( !IsDialogMessage( window_handle, &msg ) )
      {
        DispatchMessage( &msg );
      }
    }

    // draw scene
    drawScene( window_handle, p_device, tri_dx_vb, stylus_vb, hd );
  }

  // release DirectX device.
  p_device->Release();

  // destroy window.
  DestroyWindow( window_handle );
  
  // Disable HAPI device.
  hd.disableDevice();
  // Release HAPI device.
  hd.releaseDevice();
  return 0;
}
