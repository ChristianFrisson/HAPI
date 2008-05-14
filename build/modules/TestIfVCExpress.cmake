# Macro to check if MFC exists. Would like to check if ATL exists but this will have to do for now.
# Can be seen as a way of detecting whether an express version of visual studio is used or not.
MACRO(TestIfVCExpress)
  IF(MSVC)
    IF("CMake_HAVE_MFC" MATCHES "^CMake_HAVE_MFC$")
      SET(CHECK_INCLUDE_FILE_VAR "afxwin.h")
      CONFIGURE_FILE(${CMAKE_ROOT}/Modules/CheckIncludeFile.cxx.in
        ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/CheckIncludeFile.cxx)
      MESSAGE(STATUS "Looking for MFC")
      TRY_COMPILE(CMake_HAVE_MFC
        ${CMAKE_BINARY_DIR}
        ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/CheckIncludeFile.cxx
        CMAKE_FLAGS
        -DCMAKE_MFC_FLAG:STRING=2
        -DCOMPILE_DEFINITIONS:STRING=-D_AFXDLL
        OUTPUT_VARIABLE OUTPUT)
      IF(CMake_HAVE_MFC)
        MESSAGE(STATUS "Looking for MFC - found")
        SET(CMake_HAVE_MFC 1 CACHE INTERNAL "Have MFC?")
        FILE(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeOutput.log
          "Determining if MFC exists passed with the following output:\n"
          "${OUTPUT}\n\n")
      ELSE(CMake_HAVE_MFC)
        MESSAGE(STATUS "Looking for MFC - not found")
        SET(CMake_HAVE_MFC 0 CACHE INTERNAL "Have MFC?")
        FILE(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
          "Determining if MFC exists failed with the following output:\n"
          "${OUTPUT}\n\n")
      ENDIF(CMake_HAVE_MFC)
    ENDIF("CMake_HAVE_MFC" MATCHES "^CMake_HAVE_MFC$")
  ENDIF(MSVC)
ENDMACRO(TestIfVCExpress)
