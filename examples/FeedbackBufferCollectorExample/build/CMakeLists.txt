cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "FeedbackBufferCollector".  CMakeLists files in this project can
# refer to the root source directory of the project as ${FeedbackBufferCollector_SOURCE_DIR} and
# to the root binary directory of the project as ${FeedbackBufferCollector_BINARY_DIR}.
project( FeedbackBufferCollector )

set( CMAKE_MODULE_PATH "${FeedbackBufferCollector_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${FeedbackBufferCollector_SOURCE_DIR}/../.. )

setupRPathForLib()

# Create a library called "FeedbackBufferCollector" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

set( FeedbackBufferCollector_SRCS "${FeedbackBufferCollector_SOURCE_DIR}/../FeedbackBufferCollectorExample.cpp" )

include_directories( ${FeedbackBufferCollector_SOURCE_DIR}/.. )

set( required_libs )

include( StripAndAddLibraryDirectories )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil HAPI
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil HAPI )
include_directories( ${tmp_h3d_include_dirs} )

find_package( OpenGL REQUIRED )
if( OPENGL_FOUND )
  set( HAVE_OPENGL 1 )
  include_directories( ${OPENGL_INCLUDE_DIR} )
  set( required_libs ${required_libs} ${OPENGL_LIBRARIES} )
endif()

# Glut.
find_package( GLUT REQUIRED )

if( GLUT_FOUND )
  include_directories( ${GLUT_INCLUDE_DIR} )
  if( WIN32 )
    stripAndAddLibraryDirectories( ${GLUT_LIBRARIES} )
  else()
    set( required_libs ${required_libs} ${GLUT_LIBRARIES} )
  endif()
endif()

add_executable( FeedbackBufferCollector ${FeedbackBufferCollector_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( FeedbackBufferCollector PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( FeedbackBufferCollector ${required_libs} )

set( FeedbackBufferCollector_COMPILE_FLAGS )

addCommonH3DCompileFlags( FeedbackBufferCollector_COMPILE_FLAGS )

set_target_properties( FeedbackBufferCollector PROPERTIES COMPILE_FLAGS "${FeedbackBufferCollector_COMPILE_FLAGS}" )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS FeedbackBufferCollector
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
