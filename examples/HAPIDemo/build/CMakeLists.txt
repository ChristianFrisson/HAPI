cmake_minimum_required(VERSION 2.6.0)
if( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )
endif( COMMAND cmake_policy )
# The name of our project is "HAPIDemo".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HAPIDemo_SOURCE_DIR} and
# to the root binary directory of the project as ${HAPIDemo_BINARY_DIR}.
project (HAPIDemo)

# Create a library called "Hello" which include sourcefiles.
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(${HAPIDemo_SOURCE_DIR}/.. HAPIDemo_SRCS)

INCLUDE_DIRECTORIES( ${HAPIDemo_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${HAPIDemo_SOURCE_DIR}/../../../build/modules )

#H3DUtil
FIND_PACKAGE(H3DUtil REQUIRED)

IF(H3DUTIL_FOUND)
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
ENDIF(H3DUTIL_FOUND)

#HAPI
FIND_PACKAGE(HAPI REQUIRED)

IF(HAPI_FOUND)
  INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${HAPI_LIBRARIES} )
ENDIF(HAPI_FOUND)

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
  FIND_PACKAGE(wxWidgets REQUIRED)
  IF(wxWidgets_FOUND)
    INCLUDE(${wxWidgets_USE_FILE})
    SET(requiredLibs ${requiredLibs} ${wxWidgets_LIBRARIES} )
  ENDIF(wxWidgets_FOUND)
ENDIF(wxWidgets_FOUND)

FIND_PACKAGE(PTHREAD REQUIRED)
IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND)

ADD_EXECUTABLE(HAPIDemo WIN32 MACOSX_BUNDLE ${HAPIDemo_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES DEBUG_POSTFIX "_d" )

IF(MSVC70 OR MSVC71)
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ELSEIF(MSVC80)
  # remove some warnings about deprecated functions.
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ELSEIF(MSVC90)
  # remove some warnings about deprecated functions.
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES COMPILE_FLAGS "-D_CRT_SECURE_NO_DEPRECATE /Zc:wchar_t- /MP" )
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ENDIF(MSVC70 OR MSVC71)

TARGET_LINK_LIBRARIES( HAPIDemo ${requiredLibs} ${optionalLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${HAPIDemo_SOURCE_DIR}/../..)
ENDIF(WIN32)

INSTALL( TARGETS HAPIDemo
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin )
