cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "SurfaceExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SurfaceExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SurfaceExample_BINARY_DIR}.
project( SurfaceExample )

set( CMAKE_MODULE_PATH "${SurfaceExample_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${SurfaceExample_SOURCE_DIR}/../.. )
setupRPathForLib()
enableCpp11( FAIL_MESSAGE "Enabling C++11 support. This compiler lacks such support." )

# Create a library called "SurfaceExample" which includes the source file "SurfaceExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

set( SurfaceExample_SRCS "${SurfaceExample_SOURCE_DIR}/../SurfaceDemo.cpp" )

include_directories( ${SurfaceExample_SOURCE_DIR}/.. )

set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI )
include_directories( ${tmp_h3d_include_dirs} ) 

add_executable( SurfaceExample ${SurfaceExample_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( SurfaceExample PROPERTIES DEBUG_POSTFIX "_d" )

set( SurfaceExample_COMPILE_FLAGS )

addCommonH3DCompileFlags( SurfaceExample_COMPILE_FLAGS TRUE )

set_target_properties( SurfaceExample PROPERTIES COMPILE_FLAGS "${SurfaceExample_COMPILE_FLAGS}" )

target_link_libraries( SurfaceExample ${required_libs} ${optional_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS SurfaceExample
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
