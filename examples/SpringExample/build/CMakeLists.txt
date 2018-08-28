cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "SpringExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SpringExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SpringExample_BINARY_DIR}.
project( SpringExample )

set( CMAKE_MODULE_PATH "${SpringExample_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${SpringExample_SOURCE_DIR}/../.. )
setupRPathForLib()

# Create a library called "SpringExample" which includes the source file "SpringExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

set( SpringExample_SRCS "${SpringExample_SOURCE_DIR}/../SpringExample.cpp" )

include_directories( ${SpringExample_SOURCE_DIR}/.. )

set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI )
include_directories( ${tmp_h3d_include_dirs} ) 

add_executable( SpringExample ${SpringExample_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( SpringExample PROPERTIES DEBUG_POSTFIX "_d" )

set( SpringExample_COMPILE_FLAGS )

addCommonH3DCompileFlags( SpringExample_COMPILE_FLAGS )

set_target_properties( SpringExample PROPERTIES COMPILE_FLAGS "${SpringExample_COMPILE_FLAGS}" )

target_link_libraries( SpringExample ${required_libs} ${optional_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS SpringExample
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
