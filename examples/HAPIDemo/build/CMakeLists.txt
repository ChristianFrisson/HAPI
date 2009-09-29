IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
if( COMMAND cmake_policy )
  if( POLICY CMP0003 )
    cmake_policy( SET CMP0003 NEW )
  endif( POLICY CMP0003 )
endif( COMMAND cmake_policy )
# The name of our project is "HAPIDemo".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HAPIDemo_SOURCE_DIR} and
# to the root binary directory of the project as ${HAPIDemo_BINARY_DIR}.
project (HAPIDemo)

# Create a library called "Hello" which include sourcefiles.
# The extension is already found.  Any number of sources could be listed here.

SET( HAPIDemo_SRCS "${HAPIDemo_SOURCE_DIR}/../ForceFieldWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../ForceFieldWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../HAPIDemo.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../HAPIDemo.h"
                   "${HAPIDemo_SOURCE_DIR}/../HapticShapeConstraintWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../HapticShapeConstraintWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../PositionFunctionWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../PositionFunctionWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../SpringWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../SpringWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../TimeFunctionWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../TimeFunctionWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../ViscosityWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../ViscosityWidgetsPage.h" )

INCLUDE_DIRECTORIES( ${HAPIDemo_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH "${HAPIDemo_SOURCE_DIR}/../../../build/modules" )

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
IF(PTHREAD_FOUND AND WIN32)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND AND WIN32)

ADD_EXECUTABLE(HAPIDemo WIN32 MACOSX_BUNDLE ${HAPIDemo_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES DEBUG_POSTFIX "_d" )

IF(MSVC70 OR MSVC71)
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ELSEIF(MSVC80)
  # Remove some warnings about deprecated functions. Do not treat wchar_t as built in type
  SET( HAPIDemo_COMPILE_FLAGS "-D_CRT_SECURE_NO_DEPRECATE /Zc:wchar_t-" )
    
  # This might be useful for visual studio 2005 users that often recompile the api.
  IF( NOT DEFINED USE_VC8_MP_FLAG )
    SET( USE_VC8_MP_FLAG "NO" CACHE BOOL "In visual studio 8 the MP flag exists but is not documented. Maybe it is unsafe to use. If you want to use it then set this flag to yes." )
  ENDIF( NOT DEFINED USE_VC8_MP_FLAG )
  
  IF( USE_VC8_MP_FLAG )
    SET( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /MP" )
  ENDIF( USE_VC8_MP_FLAG )
  
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS}" )
  # Remove link errors
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ELSEIF(MSVC90)
  # remove some warnings about deprecated functions, do not treat wchar_t as inbuilt type and use threaded compiling.
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES COMPILE_FLAGS "-D_CRT_SECURE_NO_DEPRECATE /Zc:wchar_t- /MP" )
  # Remove link errors
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ENDIF(MSVC70 OR MSVC71)

TARGET_LINK_LIBRARIES( HAPIDemo ${requiredLibs} ${optionalLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
  IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
    SET( CMAKE_INSTALL_PREFIX ${HAPIDemo_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  ENDIF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
ENDIF(WIN32)

INSTALL( TARGETS HAPIDemo
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin )
