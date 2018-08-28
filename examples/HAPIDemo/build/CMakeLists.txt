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

set( HAPIDemo_COMPILE_FLAGS )
if( MSVC )
  
  set( HAPIDemo_COMPILE_FLAGS "-DWXUSINGDLL" )

  set_target_properties( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
endif()

addCommonH3DCompileFlags( HAPIDemo_COMPILE_FLAGS )

set_target_properties( HAPIDemo PROPERTIES COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS}" )

target_link_libraries( HAPIDemo ${required_libs} ${optional_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

set( bundle_installation_command )
if( APPLE )
  if( NOT DEFINED HAPIDemo_OSX_BUNDLE_INSTALL_DESTINATION )
    set( HAPIDemo_OSX_BUNDLE_INSTALL_DESTINATION /Applications CACHE PATH "Directory into which to install the HAPIDemo.app bundle." )
  endif()

  set( bundle_installation_command BUNDLE DESTINATION ${HAPIDemo_OSX_BUNDLE_INSTALL_DESTINATION} COMPONENT HAPI_cpack_examples_runtime )
endif()

install( TARGETS HAPIDemo
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime
         ${bundle_installation_command} )
