cmake_minimum_required(VERSION 2.4.8)
PROJECT (HAPIExamples)

# Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/HAPIDemo/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/HAPIDemo/build
                  ${BINARY_DIR_FOR_Example} )

STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/SpringExample/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/SpringExample/build
                  ${BINARY_DIR_FOR_Example} )

STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/SurfaceExample/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/SurfaceExample/build
                  ${BINARY_DIR_FOR_Example} )

STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/FeedbackBufferCollectorExample/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/FeedbackBufferCollectorExample/build
                  ${BINARY_DIR_FOR_Example} )

STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/ThreadExamples/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/ThreadExamples/build
                  ${BINARY_DIR_FOR_Example} )

IF( WIN32 )
  STRING( REPLACE ${HAPIExamples_SOURCE_DIR} ${HAPIExamples_SOURCE_DIR}/DirectXExample/build BINARY_DIR_FOR_Example ${CMAKE_CURRENT_BINARY_DIR} )
  ADD_SUBDIRECTORY( ${HAPIExamples_SOURCE_DIR}/DirectXExample/build
                    ${BINARY_DIR_FOR_Example} )
ENDIF( WIN32 )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${HAPIExamples_SOURCE_DIR} )
ENDIF(WIN32)
