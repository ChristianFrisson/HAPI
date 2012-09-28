IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
PROJECT (HAPIExamples)

# set the install directory on Windows
IF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
 SET( CMAKE_INSTALL_PREFIX ${HAPIExamples_SOURCE_DIR} )
 SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
MESSAGE( STATUS "Including HAPIDemo" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../HAPIDemo/build
                  ${CMAKE_CURRENT_BINARY_DIR}/HAPIDemo )

MESSAGE( STATUS "Including HAPI SpringExample" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../SpringExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SpringExample )

MESSAGE( STATUS "Including HAPI SurfaceExample" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../SurfaceExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SurfaceExample )

FIND_PACKAGE( OpenGL )
IF( OPENGL_FOUND )
  MESSAGE( STATUS "Including HAPI FeedbackBufferCollectionExample" )
  ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../FeedbackBufferCollectorExample/build
                    ${CMAKE_CURRENT_BINARY_DIR}/FeedbackBufferCollectorExample )
ELSE( OPENGL_FOUND )
  MESSAGE( STATUS "OpenGL was not found. FeedbackBufferCollectorExample will not be generated." )
ENDIF( OPENGL_FOUND )

MESSAGE( STATUS "Including HAPI ThreadExamples" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../ThreadExamples/build
                  ${CMAKE_CURRENT_BINARY_DIR}/ThreadExamples )

IF( WIN32 )
  SET(CMAKE_MODULE_PATH ${HAPIExamples_SOURCE_DIR}/../../build/modules )
  FIND_PACKAGE( DirectX )
  IF( DirectX_FOUND )
    MESSAGE( STATUS "Including HAPI DirectXExample" )
    ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../DirectXExample/build
                      ${BINARY_DIR_FOR_Example} )
  ELSE( DirectX_FOUND )
    MESSAGE( STATUS "DirectX headers and libraries were not found. DirectXExample will not be generated." )
  ENDIF( DirectX_FOUND )
ENDIF( WIN32 )