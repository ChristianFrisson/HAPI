IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)

# The name of our project is "FeedbackBufferCollector".  CMakeLists files in this project can
# refer to the root source directory of the project as ${FeedbackBufferCollector_SOURCE_DIR} and
# to the root binary directory of the project as ${FeedbackBufferCollector_BINARY_DIR}.
project (FeedbackBufferCollector)

# set the install directory on Windows
IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  SET( CMAKE_INSTALL_PREFIX ${FeedbackBufferCollector_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# Create a library called "FeedbackBufferCollector" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

SET( FeedbackBufferCollector_SRCS "${FeedbackBufferCollector_SOURCE_DIR}/../FeedbackBufferCollectorExample.cpp" )

INCLUDE_DIRECTORIES( ${FeedbackBufferCollector_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${FeedbackBufferCollector_SOURCE_DIR}/../../../build/modules )
IF( COMMAND cmake_policy )
  if( POLICY CMP0011 )
    cmake_policy( SET CMP0011 NEW )
  endif( POLICY CMP0011 )
ENDIF( COMMAND cmake_policy )
INCLUDE( StripAndAddLibraryDirectories )

IF( TARGET H3DUtil )
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  SET( requiredLibs ${requiredLibs} H3DUtil )
ELSE( TARGET H3DUtil )
  #H3DUtil
  FIND_PACKAGE(H3DUtil REQUIRED)

  IF(H3DUTIL_FOUND)
    INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
  ENDIF(H3DUTIL_FOUND)
ENDIF( TARGET H3DUtil )

IF( TARGET HAPI )
  INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
  SET( requiredLibs ${requiredLibs} HAPI )
ELSE( TARGET HAPI )
  #HAPI
  FIND_PACKAGE(HAPI REQUIRED)

  IF(HAPI_FOUND)
    INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${HAPI_LIBRARIES} )
  ENDIF(HAPI_FOUND)
ENDIF( TARGET HAPI )

IF( H3D_USE_DEPENDENCIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDENCIES_ONLY to true.
  IF(PTHREAD_FOUND)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  ENDIF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
  
  SET( requiredLibs ${requiredLibs} H3DUtil HAPI )
ELSE( H3D_USE_DEPENDENCIES_ONLY )
  FIND_PACKAGE(PTHREAD REQUIRED)
  IF(PTHREAD_FOUND AND WIN32)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
  ENDIF(PTHREAD_FOUND AND WIN32)
ENDIF( H3D_USE_DEPENDENCIES_ONLY )

FIND_PACKAGE(OpenGL REQUIRED)
IF(OPENGL_FOUND)
  SET(HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
  SET(requiredLibs ${requiredLibs} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

IF(WIN32)
  FIND_PACKAGE(GLUTWin REQUIRED)
  IF(GLUT_FOUND)
    INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} )
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${GLUT_LIBRARIES} )
  ENDIF(GLUT_FOUND)
ENDIF(WIN32)

IF(GLUT_FOUND)
ELSE(GLUT_FOUND)
  FIND_PACKAGE(GLUT REQUIRED)
  IF(GLUT_FOUND)
    INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${GLUT_LIBRARIES} )
  ENDIF(GLUT_FOUND)
ENDIF(GLUT_FOUND)
  
ADD_EXECUTABLE(FeedbackBufferCollector ${FeedbackBufferCollector_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( FeedbackBufferCollector PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( FeedbackBufferCollector ${requiredLibs} )

INSTALL( TARGETS FeedbackBufferCollector
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )
