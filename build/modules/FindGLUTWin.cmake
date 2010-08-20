# - Find GLUT on windows
#
#  GLUT_INCLUDE_DIR -  where to find GLUT headers
#  GLUT_LIBRARIES    - List of libraries when using GLUT.
#  GLUT_FOUND        - True if GLUT found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

# Look for the header file.
FIND_PATH( GLUT_INCLUDE_DIR NAMES GL/glut.h
           PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                 $ENV{H3D_ROOT}/../External/include
                 ../../External/include
                 ${module_file_path}/../../../External/include
           DOC "Path in which the file GL/glut.h is located." )
MARK_AS_ADVANCED(GLUT_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY( GLUT_LIBRARY NAMES freeglut glut32
              PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                    $ENV{H3D_ROOT}/../External/${LIB}
                    ../../External/${LIB}
                    ${module_file_path}/../../../External/${LIB} 
              DOC "Path to glut32 library." )
MARK_AS_ADVANCED(GLUT_LIBRARY)

# Copy the results to the output variables.
IF(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
  SET(GLUT_FOUND 1)
  IF( WIN32 AND PREFER_FREEGLUT_STATIC_LIBRARIES )
    SET( FREEGLUT_STATIC 1 )
  ENDIF( WIN32 AND PREFER_FREEGLUT_STATIC_LIBRARIES )
  SET(GLUT_LIBRARIES ${GLUT_LIBRARY} )
  SET(GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
ELSE(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
  SET(GLUT_FOUND 0)
  SET(GLUT_LIBRARIES)
  SET(GLUT_INCLUDE_DIR)
ENDIF(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)

# Report the results.
IF(NOT GLUT_FOUND)
  SET(GLUT_DIR_MESSAGE
    "GLUT was not found. Make sure GLUT_LIBRARY and GLUT_INCLUDE_DIR are set to where you have your glut header and lib files.")
  IF(GLUTWin_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${GLUT_DIR_MESSAGE}")
  ELSEIF(NOT GLUTWin_FIND_QUIETLY)
    MESSAGE(STATUS "${GLUT_DIR_MESSAGE}")
  ENDIF(GLUTWin_FIND_REQUIRED)
ENDIF(NOT GLUT_FOUND)
