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

# set the install directory on Windows
IF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  SET( CMAKE_INSTALL_PREFIX ${HAPIDemo_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

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

IF( H3D_USE_DEPENDENCIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDENCIES_ONLY to true.
  INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIR} ) 
ENDIF( H3D_USE_DEPENDENCIES_ONLY )

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

# TMP_wxWidgets_FOUND is needed in a metarepository build, otherwise if
# Wxwidgets have been found already then wxWidgets_USE_FILE will not be used
# properly. On non-Windows systems.
SET( TMP_wxWidgets_FOUND FALSE )
IF(WIN32)
  FIND_PACKAGE(wxWidgetsWin)
  IF(wxWidgets_FOUND)
    INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${wxWidgets_LIBRARIES} )
    SET( TMP_wxWidgets_FOUND TRUE )
  ENDIF(wxWidgets_FOUND)
ENDIF(WIN32)

IF(TMP_wxWidgets_FOUND)
ELSE(TMP_wxWidgets_FOUND)
  SET(wxWidgets_USE_LIBS base core)
  FIND_PACKAGE(wxWidgets REQUIRED)
  IF(wxWidgets_FOUND)
    INCLUDE(${wxWidgets_USE_FILE})
    SET(requiredLibs ${requiredLibs} ${wxWidgets_LIBRARIES} )
  ENDIF(wxWidgets_FOUND)
ENDIF(TMP_wxWidgets_FOUND)

ADD_EXECUTABLE(HAPIDemo WIN32 MACOSX_BUNDLE ${HAPIDemo_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES DEBUG_POSTFIX "_d" )

IF( MSVC )
  
  SET( HAPIDemo_COMPILE_FLAGS "-DWXUSINGDLL" )
  
  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 ( so far ) but we still set it for all.
  SET( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Zc:wchar_t")
  
  IF( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    SET( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  ENDIF( ${MSVC_VERSION} GREATER 1399 )
  
  IF( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    SET( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /MP" )
  ENDIF( ${MSVC_VERSION} GREATER 1499 )
  
  IF( ${MSVC_VERSION} LESS 1600 )
    # Turn off optimization for visual studio versions 8 and below.
    # Old setting which I (Markus) have no idea why it was added.
    SET( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Od" )
  ENDIF( ${MSVC_VERSION} LESS 1600 )

  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS}" )
  SET_TARGET_PROPERTIES( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
ENDIF( MSVC )

TARGET_LINK_LIBRARIES( HAPIDemo ${requiredLibs} ${optionalLibs} )

SET( DEFAULT_BIN_INSTALL "bin" )
SET( DEFAULT_LIB_INSTALL "lib" )

IF( WIN32 )
  SET( DEFAULT_BIN_INSTALL "bin32" )
  SET( DEFAULT_LIB_INSTALL "lib32" )
  IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    SET( DEFAULT_BIN_INSTALL "bin64" )
    SET( DEFAULT_LIB_INSTALL "lib64" )
  ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
ENDIF( WIN32 )

INSTALL( TARGETS HAPIDemo
         LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_examples_runtime )
