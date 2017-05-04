cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "DirectXExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${DirectXExample_SOURCE_DIR} and
# to the root binary directory of the project as ${DirectXExample_BINARY_DIR}.
project( DirectXExample )

set( CMAKE_MODULE_PATH "${DirectXExample_SOURCE_DIR}/../../../build/modules" )
include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${DirectXExample_SOURCE_DIR}/../.. )
setupRPathForLib()

# Create a library called "DirectXExample" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

set( DirectXExample_SRCS "${DirectXExample_SOURCE_DIR}/../src/main.cpp" )

include_directories( ${DirectXExample_SOURCE_DIR}/../src )

set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI )
include_directories( ${tmp_h3d_include_dirs} )

find_package( DirectX REQUIRED )
if( DirectX_FOUND )
  include_directories( ${DirectX_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${DirectX_LIBRARIES} )
endif()

add_executable( DirectXExample WIN32 ${DirectXExample_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( DirectXExample PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( DirectXExample ${required_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS DirectXExample
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
