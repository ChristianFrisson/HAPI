cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

project( HAPIExamples )

set( CMAKE_MODULE_PATH "${HAPIExamples_SOURCE_DIR}/../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${HAPIExamples_SOURCE_DIR}/.. )

# Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
set( WXWINDOWS_USE_GL 1 )
find_package( wxWidgets COMPONENTS base core )

if( wxWidgets_FOUND )
  message( STATUS "Including HAPIDemo" )
  add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../HAPIDemo/build
                    ${CMAKE_CURRENT_BINARY_DIR}/HAPIDemo )
else()
  message( WARNING "wxWidgets was not found. HAPIDemo will not be generated." )
endif()

message( STATUS "Including HAPI SpringExample" )
add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../SpringExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SpringExample )

message( STATUS "Including HAPI SurfaceExample" )
add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../SurfaceExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SurfaceExample )

find_package( OpenGL )
find_package( GLUT )
if( OPENGL_FOUND AND GLUT_FOUND )
  message( STATUS "Including HAPI FeedbackBufferCollectionExample" )
  add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../FeedbackBufferCollectorExample/build
                    ${CMAKE_CURRENT_BINARY_DIR}/FeedbackBufferCollectorExample )
else()
  message( STATUS "OpenGL or GLUT was not found. FeedbackBufferCollectorExample will not be generated." )
endif()

message( STATUS "Including HAPI ThreadExamples" )
add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../ThreadExamples/build
                  ${CMAKE_CURRENT_BINARY_DIR}/ThreadExamples )

if( WIN32 )
  find_package( DirectX )
  if( DirectX_FOUND )
    message( STATUS "Including HAPI DirectXExample" )
    add_subdirectory( ${HAPIExamples_SOURCE_DIR}/../DirectXExample/build
                      ${BINARY_DIR_FOR_Example} )
  else()
    message( STATUS "DirectX headers and libraries were not found. DirectXExample will not be generated." )
  endif()
endif()
