IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
PROJECT (HAPIExamples)

# set the install directory on Windows
IF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
 SET( CMAKE_INSTALL_PREFIX ${HAPIExamples_SOURCE_DIR}/.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
 SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
SET(CMAKE_MODULE_PATH ${HAPIExamples_SOURCE_DIR}/../../build/modules )

# Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
IF( H3D_USE_DEPENDENCIES_ONLY )
  IF(WIN32)
    FIND_PACKAGE(wxWidgetsWin)
    IF(wxWidgets_FOUND)
      INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIR} ) 
      SET(requiredLibs ${requiredLibs} ${wxWidgets_LIBRARIES} )
    ENDIF(wxWidgets_FOUND)
  ENDIF(WIN32)

  IF(wxWidgets_FOUND)
  ELSE(wxWidgets_FOUND)
    SET(wxWidgets_USE_LIBS base core)
    FIND_PACKAGE(wxWidgets)
    IF(wxWidgets_FOUND)
      INCLUDE(${wxWidgets_USE_FILE})
      SET(requiredLibs ${requiredLibs} ${wxWidgets_LIBRARIES} )
    ENDIF(wxWidgets_FOUND)
  ENDIF(wxWidgets_FOUND)
ENDIF( H3D_USE_DEPENDENCIES_ONLY )

IF( ( NOT H3D_USE_DEPENDENCIES_ONLY ) OR wxWidgets_FOUND )
  MESSAGE( STATUS "Including HAPIDemo" )
  ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../HAPIDemo/build
                    ${CMAKE_CURRENT_BINARY_DIR}/HAPIDemo )
ELSE( ( NOT H3D_USE_DEPENDENCIES_ONLY ) OR wxWidgets_FOUND )
  MESSAGE( WARNING "wxWidgets was not found. HAPIDemo will not be generated." )
ENDIF( ( NOT H3D_USE_DEPENDENCIES_ONLY ) OR wxWidgets_FOUND )

MESSAGE( STATUS "Including HAPI SpringExample" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../SpringExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SpringExample )

MESSAGE( STATUS "Including HAPI SurfaceExample" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../SurfaceExample/build
                  ${CMAKE_CURRENT_BINARY_DIR}/SurfaceExample )

FIND_PACKAGE( OpenGL )
IF(WIN32)
  FIND_PACKAGE(GLUTWin)
ELSE(WIN32)
  FIND_PACKAGE(GLUT)
ENDIF(WIN32)
IF( OPENGL_FOUND AND GLUT_FOUND )
  MESSAGE( STATUS "Including HAPI FeedbackBufferCollectionExample" )
  ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../FeedbackBufferCollectorExample/build
                    ${CMAKE_CURRENT_BINARY_DIR}/FeedbackBufferCollectorExample )
ELSE( OPENGL_FOUND AND GLUT_FOUND )
  MESSAGE( STATUS "OpenGL or GLUT was not found. FeedbackBufferCollectorExample will not be generated." )
ENDIF( OPENGL_FOUND AND GLUT_FOUND )

MESSAGE( STATUS "Including HAPI ThreadExamples" )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../ThreadExamples/build
                  ${CMAKE_CURRENT_BINARY_DIR}/ThreadExamples )

IF( WIN32 )
  FIND_PACKAGE( DirectX )
  IF( DirectX_FOUND )
    MESSAGE( STATUS "Including HAPI DirectXExample" )
    ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/../DirectXExample/build
                      ${BINARY_DIR_FOR_Example} )
  ELSE( DirectX_FOUND )
    MESSAGE( STATUS "DirectX headers and libraries were not found. DirectXExample will not be generated." )
  ENDIF( DirectX_FOUND )
ENDIF( WIN32 )