# - Find GLUT on windows
#
#  GLUT_INCLUDE_DIR -  where to find GLUT headers
#  GLUT_LIBRARIES    - List of libraries when using GLUT.
#  GLUT_FOUND        - True if GLUT found.


# Look for the header file.
FIND_PATH( GLUT_INCLUDE_DIR NAMES GL/glut.h
           PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                 ../../External/include )
MARK_AS_ADVANCED(GLUT_INCLUDE_DIR)

# Look for the library.
FIND_LIBRARY( GLUT_LIBRARY NAMES glut32   
              PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                    ../../External/lib )
MARK_AS_ADVANCED(GLUT_LIBRARY)

# Copy the results to the output variables.
IF(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
  SET(GLUT_FOUND 1)
  SET(GLUT_LIBRARIES ${GLUT_LIBRARY} )
  SET(GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
ELSE(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
  SET(GLUT_FOUND 0)
  SET(GLUT_LIBRARIES)
  SET(GLUT_INCLUDE_DIR)
ENDIF(GLUT_INCLUDE_DIR AND GLUT_LIBRARY)
