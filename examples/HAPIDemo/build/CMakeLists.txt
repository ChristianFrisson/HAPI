cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "HAPIDemo".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HAPIDemo_SOURCE_DIR} and
# to the root binary directory of the project as ${HAPIDemo_BINARY_DIR}.
project( HAPIDemo )

set( CMAKE_MODULE_PATH "${HAPIDemo_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${HAPIDemo_SOURCE_DIR}/../.. )
setupRPathForLib()

# Create a library called "Hello" which include sourcefiles.
# The extension is already found.  Any number of sources could be listed here.

set( HAPIDemo_SRCS "${HAPIDemo_SOURCE_DIR}/../ForceFieldWidgetsPage.cpp"
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

include_directories( ${HAPIDemo_SOURCE_DIR}/.. )

set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI )
include_directories( ${tmp_h3d_include_dirs} ) 

set( WXWINDOWS_USE_GL 1 )
find_package( wxWidgets REQUIRED COMPONENTS base core )
if( wxWidgets_FOUND )
  if( WIN32 )
    include_directories( ${wxWidgets_INCLUDE_DIR} ) 
  else()
    include( ${wxWidgets_USE_FILE} )
  endif()
  set( required_libs ${required_libs} ${wxWidgets_LIBRARIES} )
endif()

add_executable( HAPIDemo WIN32 MACOSX_BUNDLE ${HAPIDemo_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( HAPIDemo PROPERTIES DEBUG_POSTFIX "_d" )

if( MSVC )
  
  set( HAPIDemo_COMPILE_FLAGS "-DWXUSINGDLL" )
  
  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Zc:wchar_t" )
  
  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()
  
  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /MP" )
  endif()
  
  if( ${MSVC_VERSION} LESS 1600 )
    # Turn off optimization for visual studio versions 8 and below.
    # Old setting which I (Markus) have no idea why it was added.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Od" )
  endif()

  set_target_properties( HAPIDemo PROPERTIES COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS}" )
  set_target_properties( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
endif()

target_link_libraries( HAPIDemo ${required_libs} ${optional_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS HAPIDemo
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
