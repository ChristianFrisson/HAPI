if( NOT TARGET HAPI )
  message( FATAL_ERROR "Include file HAPICPack.cmake require the target HAPI to exist. Please add HAPI/build/CMakeLists.txt as subdirectory first." )
endif()

# Add all sources, they are added to a variable called HAPI_SRCS defined
# in the included file. All header files are added to a variable called
# HAPI_HEADERS.
include( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )

# If cpack should be configured.
if( GENERATE_CPACK_PROJECT )
  if( WIN32 )
    # Add a cache variable which indicates where the Externals directory used for packaging
    # HAPI is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3DAPI_CPACK_EXTERNAL_ROOT )
      if( NOT DEFINED HAPI_CPACK_EXTERNAL_ROOT )
        set( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "" )
        if( H3D_USE_DEPENDENCIES_ONLY )
          foreach( EXTERNAL_INCLUDE_DIR_TMP ${EXTERNAL_INCLUDE_DIR} )
            if( EXISTS ${EXTERNAL_INCLUDE_DIR_TMP}/../include/pthread )
              set( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "${EXTERNAL_INCLUDE_DIR_TMP}/.." )
            endif()
          endforeach()
        else( H3D_USE_DEPENDENCIES_ONLY )
          set( HAPI_CPACK_EXTERNAL_ROOT_DEFAULT "$ENV{H3D_EXTERNAL_ROOT}" )
        endif()
        set( HAPI_CPACK_EXTERNAL_ROOT "${HAPI_CPACK_EXTERNAL_ROOT_DEFAULT}" CACHE PATH "Set to the External directory used with HAPI, needed to pack properly. If not set FIND_modules will be used instead." )
        mark_as_advanced( HAPI_CPACK_EXTERNAL_ROOT )
      endif()
    else( NOT DEFINED H3DAPI_CPACK_EXTERNAL_ROOT )
      set( HAPI_CPACK_EXTERNAL_ROOT ${H3DAPI_CPACK_EXTERNAL_ROOT} )
    endif()
  endif()
  include( ${HAPI_SOURCE_DIR}/../../H3DUtil/build/H3DUtilCPack.cmake )

  if( NOT TARGET H3DAPI )
    set( CPACK_ALL_INSTALL_TYPES Full Developer )
    set( CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules ${HAPI_SOURCE_DIR}/modules/sharedModules )
    set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform, device independent haptics library." )
    set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
    set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
    set( CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../ReadMe" )
    set( CPACK_RESOURCE_FILE_LICENSE "${HAPI_SOURCE_DIR}/../LICENSE" )
    set( CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};HAPI;ALL;/" )
    set( CPACK_PACKAGE_INSTALL_DIRECTORY "HAPI" )

    # File patterns to ignore, common for all operating systems.
    set( HAPI_CPACK_IGNORE_PATTERNS /\\\\.svn/
                                    \\\\.obj$
                                    \\\\.ncb$
                                    \\\\.log$
                                    \\\\.suo$
                                    \\\\.zip$
                                    \\\\.dir/
                                    \\\\.user$
                                    \\\\.cv$
                                    "/Debug(.)*/"
                                    "/debug(.)*/"
                                    /Release
                                    /release
                                    /linux
                                    /build/win32/
                                    "/(build|examples)/vc(7|(8|9))"
                                    /osx/ )
    set( CPACK_PACKAGE_VERSION_MAJOR ${HAPI_MAJOR_VERSION} )
    set( CPACK_PACKAGE_VERSION_MINOR ${HAPI_MINOR_VERSION} )
    set( CPACK_PACKAGE_VERSION_PATCH ${HAPI_BUILD_VERSION} )

    if( WIN32 AND NOT UNIX )
      set( CPACK_NSIS_INSTALL_ROOT "C:" )
      set( CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL "ON" )
      set( CPACK_PACKAGE_START_MENU_NAME "HAPI 1.1" )

      # Extra links to start menu if values are "ON"
      set( CPACK_ADD_HAPIDOC_LINKS "ON" )

      if( HAPI_EXAMPLE_PROJECTS )
        set( CPACK_ADD_HAPIEXAMPLES_LINKS "ON" )
      endif()

      # External binary directory to add to path.
      set( CPACK_EXTERNAL_BIN "bin32" )
      set( CPACK_H3D_64_BIT "FALSE" )
      if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
        set( CPACK_EXTERNAL_BIN "bin64" )
        set( CPACK_H3D_64_BIT "TRUE" )
      endif()

      # Extra install commands will be set to install vc8(9)_redists
      set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS "\\n" )

      set( redist_versions 8 9 10 )
      foreach( redist_version ${redist_versions} )
        # Add cache variable vc${redist_version}_redist which should be set to the install file
        # for microsoft visual studio redistributables, they can be found in the
        # installation folder for each visual studio installation.
        if( NOT DEFINED vc${redist_version}_redist )
          set( vc${redist_version}_redist CACHE FILEPATH "Set this to the exe installing microsoft visual studio redistributable for visual studio ${redist_version}" )
          mark_as_advanced( vc${redist_version} )
        endif()
        if( vc${redist_version}_redist )
          string( REPLACE "/" "\\\\" Temp_vc${redist_version}_redist ${vc${redist_version}_redist} )
          get_filename_component( VC${redist_version}_FILE_NAME ${vc${redist_version}_redist} NAME )
          set( MS_REDIST_INSTALL_COMMAND_1 " Set output Path\\n  SetOutPath \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n"
                                           " Code to install Visual studio redistributable\\n  File \\\"${Temp_vc${redist_version}_redist}\\\"\\n" )
          set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${MS_REDIST_INSTALL_COMMAND_1} )
          set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Check if uninstall vc redist \\n  MessageBox MB_YESNO \\\"Do you want to uninstall Visual studio ${redist_version} redistributable? It is recommended if no other applications use it.\\\" IDYES uninstall_vcredist_yes IDNO uninstall_vcredist_no\\n"
                                                   " A comment \\n  uninstall_vcredist_yes:\\n"
                                                   ${MS_REDIST_INSTALL_COMMAND_1} )
          if( ${redist_version} LESS 9 )
            set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /i vcredist.msi /qn\\\"'\\n" )
            set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /x vcredist.msi /qn\\\"'\\n" )
          else( )
            set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /norestart \\\"'" )
            set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /uninstall \\\"'" )
          endif()
          set( MS_REDIST_INSTALL_COMMAND_2 " Wait a bit for system to unlock file.\\n  Sleep 1000\\n"
                                           " Delete file\\n  Delete \\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\"\\n"
                                           " Reset output Path\\n  SetOutPath \\\"$INSTDIR\\\"\\n"
                                           " Remove folder\\n  RMDir /r \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n\\n" )
          set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${MS_REDIST_INSTALL_COMMAND_2} )
          set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   ${MS_REDIST_INSTALL_COMMAND_2}
                                                   " A comment \\n  uninstall_vcredist_no:\\n\\n" )
        endif()
      endforeach()

      # Modify path since in the NSIS template.
      set( CPACK_NSIS_MODIFY_PATH "ON" )
    endif()
  endif()

  if( WIN32 )
    # EXTERNAL_INCLUDES and EXTERNAL_INCLUDE_INSTALL_PATHS must be of equal lengths.
    # The reason for defining these variables here is in case we want to add functionality
    # to configure installation in some other way (using FIND-modules for example).
    set( EXTERNAL_INCLUDES "" )
    set( EXTERNAL_INCLUDE_INSTALL_PATHS "" )
    # The EXTERNAL_INCLUDES_FILES are installed directly in External/include
    set( EXTERNAL_INCLUDES_FILES "" )
    set( EXTERNAL_LIBRARIES "" )
    set( EXTERNAL_STATIC_LIBRARIES "" )
    set( EXTERNAL_BINARIES "" )

    set( EXTERNAL_BIN_PATH "bin32" )
    set( EXTERNAL_BIN_REPLACE_PATH "bin64" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( EXTERNAL_BIN_PATH "bin64" )
      set( EXTERNAL_BIN_REPLACE_PATH "bin32" )
    endif()

    if( EXISTS ${HAPI_CPACK_EXTERNAL_ROOT} )
      set( EXTERNAL_INCLUDES ${HAPI_CPACK_EXTERNAL_ROOT}/include/GL/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/DHD-API/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/chai3d/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/Simball/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/wx/
                             ${HAPI_CPACK_EXTERNAL_ROOT}/include/fparser/ )
      set( EXTERNAL_INCLUDE_INSTALL_PATHS External/include/GL
                                          External/include/DHD-API
                                          External/include/chai3d
                                          External/include/Simball
                                          External/include/wx
                                          External/include/fparser )
      set( EXTERNAL_INCLUDE_IGNORE_PATTERN "((/.svn)|(/CVS))|(/old)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)" )

      set( EXTERNAL_INCLUDES_FILES ${HAPI_CPACK_EXTERNAL_ROOT}/include/VirtuoseAPI.h
                                   ${HAPI_CPACK_EXTERNAL_ROOT}/include/EntactAPI.h )

      set( EXTERNAL_LIBRARIES ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/freeglut.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/virtuoseDLL.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/EntactAPI.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/SimballMedicalHID.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/dhdms64.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/dhdms.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/fparser.lib
                              ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/fparser_d.lib )

      set( wxlibs core adv aui html media propgrid ribbon stc webview xrc gl qa richtext )
      foreach( library_name ${wxlibs} )
        set( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxmsw30u_${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxmsw30ud_${library_name}.lib )
        #set( EXTERNAL_STATIC_LIBRARIES ${EXTERNAL_STATIC_LIBRARIES}
        #                               ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/wxmsw30u_${library_name}.lib
        #                               ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/wxmsw30ud_${library_name}.lib )
        set( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30u_${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30u_${library_name}_vc_x64_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30ud_${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxmsw30ud_${library_name}_vc_x64_custom.dll )
      endforeach()
      set( wxlibs "" _net _xml )
      # IN LISTS means that the empty argument is parsed
      foreach( library_name IN LISTS wxlibs )
        set( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxbase30u${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wxbase30ud${library_name}.lib )
        set( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30u${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30u${library_name}_vc_x64_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30ud${library_name}_vc_custom.dll
                               ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/wxbase30ud${library_name}_vc_x64_custom.dll )
      endforeach()

      set( wxlibs expat jpeg png regexu scintilla tiff zlib )
      foreach( library_name ${wxlibs} )
        set( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES}
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wx${library_name}.lib
                                ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/wx${library_name}d.lib )
        set( EXTERNAL_BINARIES ${EXTERNAL_BINARIES} )
      endforeach()

      set( EXTERNAL_STATIC_LIBRARIES ${EXTERNAL_STATIC_LIBRARIES}
                                     ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10.lib
                                     ${HAPI_CPACK_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10_d.lib )

      set( EXTERNAL_BINARIES ${EXTERNAL_BINARIES}
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/freeglut.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/virtuoseDLL.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/EntactAPI.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/SimballMedicalHID.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/HapticAPI.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/HapticMasterDriver.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/fparser.dll
                             ${HAPI_CPACK_EXTERNAL_ROOT}/${EXTERNAL_BIN_PATH}/fparser_d.dll )

    else( EXISTS ${HAPI_CPACK_EXTERNAL_ROOT} )
      message( WARNING "HAPI_CPACK_EXTERNAL_ROOT must be set to the External directory used by HAPI in order to package properly." )
    endif()

    if( EXTERNAL_INCLUDES )
      list( LENGTH EXTERNAL_INCLUDES EXTERNAL_INCLUDES_LENGTH )
      math( EXPR EXTERNAL_INCLUDES_LENGTH "${EXTERNAL_INCLUDES_LENGTH} - 1" )
      foreach( val RANGE ${EXTERNAL_INCLUDES_LENGTH} )
        list( GET EXTERNAL_INCLUDES ${val} val1 )
        list( GET EXTERNAL_INCLUDE_INSTALL_PATHS ${val} val2 )
        list( GET EXTERNAL_INCLUDE_IGNORE_PATTERN ${val} val3 )
        install( DIRECTORY ${val1}
                 DESTINATION ${val2}
                 COMPONENT HAPI_cpack_external_source
                 REGEX ${val3} EXCLUDE )
      endforeach()
    endif()

    foreach( include_file ${EXTERNAL_INCLUDES_FILES} )
      if( EXISTS ${include_file} )
        install( FILES ${include_file}
                 DESTINATION External/include
                 COMPONENT HAPI_cpack_external_source )
      endif()
    endforeach()

    foreach( library ${EXTERNAL_LIBRARIES} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32
                 COMPONENT HAPI_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64
                 COMPONENT HAPI_cpack_external_source )
      endif()
    endforeach()

    foreach( library ${EXTERNAL_STATIC_LIBRARIES} )
      if( EXISTS ${library} )
        install( FILES ${library}
                 DESTINATION External/lib32/static
                 COMPONENT HAPI_cpack_external_source )
      endif()
      # Add the other library path as well
      string( REGEX REPLACE "(/lib32/)" "/lib64/" other_library ${library} )
      if( EXISTS ${other_library} )
        install( FILES ${other_library}
                 DESTINATION External/lib64/static
                 COMPONENT HAPI_cpack_external_source )
      endif()
    endforeach()

    foreach( binary ${EXTERNAL_BINARIES} )
      if( EXISTS ${binary} )
        install( FILES ${binary}
                 DESTINATION External/${EXTERNAL_BIN_PATH}
                 COMPONENT HAPI_cpack_external_runtime )
      endif()

      string( REGEX REPLACE "(/${EXTERNAL_BIN_PATH}/)" "/${EXTERNAL_BIN_REPLACE_PATH}/" other_binary ${binary} )
      if( EXISTS ${other_binary} )
        install( FILES ${other_binary}
                 DESTINATION External/${EXTERNAL_BIN_REPLACE_PATH}
                 COMPONENT HAPI_cpack_external_runtime )
      endif()
    endforeach()

    # setting names and dependencies between components and also grouping them.
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DISPLAY_NAME "External runtime" )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DESCRIPTION "External runtime binaries needed by HAPI." )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_DEPENDS H3DUtil_cpack_external_runtime )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_GROUP "HAPI_cpack_group" )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_RUNTIME_INSTALL_TYPES Developer Full )

    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DISPLAY_NAME "External header/libraries" )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DESCRIPTION "External headers and libraries needed by HAPI." )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_DEPENDS H3DUtil_cpack_external_source HAPI_cpack_external_runtime )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_GROUP "HAPI_cpack_group" )
    set( CPACK_COMPONENT_HAPI_CPACK_EXTERNAL_SOURCE_INSTALL_TYPES Developer Full )
  endif()

  # Our project depends on these debian packages for Linux.
  set( DEBIAN_PACKAGE_DEPENDS "libgl-dev, h3dutil(>=1.0.0)" )

  # Install header files
  install( FILES ${HAPI_HEADERS}
           DESTINATION HAPI/include/HAPI
           COMPONENT HAPI_cpack_headers )

  # HAPI.cmake that goes to headers is not needed unless sources is required.
  install( FILES ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake
      DESTINATION HAPI/include/HAPI
      COMPONENT HAPI_cpack_sources )

  install( FILES ${OH_HEADERS}
           DESTINATION HAPI/OpenHapticsRenderer/include/HAPI
           COMPONENT HAPI_cpack_headers )

  install( FILES ${CHAI_HEADERS}
           DESTINATION HAPI/Chai3DRenderer/include/HAPI
           COMPONENT HAPI_cpack_headers )

  # Install src files.
  install( FILES ${HAPI_SRCS}
           DESTINATION HAPI/src
           COMPONENT HAPI_cpack_sources )

  install( FILES ${OH_SRCS}
           DESTINATION HAPI/OpenHapticsRenderer/src
           COMPONENT HAPI_cpack_sources )

  install( FILES ${CHAI_SRCS}
           DESTINATION HAPI/Chai3DRenderer/src
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../changelog
                 ${HAPI_SOURCE_DIR}/../LICENSE
                 ${HAPI_SOURCE_DIR}/../ReadMe
           DESTINATION HAPI
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc.cmake
                 ${HAPI_SOURCE_DIR}/CMakeLists.txt
                 ${HAPI_SOURCE_DIR}/HAPI.rc.cmake
                 ${HAPI_SOURCE_DIR}/HAPICPack.cmake
                 ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt
                 ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc.cmake
                 ${HAPI_SOURCE_DIR}/UpdateResourceFile.exe
           DESTINATION HAPI/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/modules/sharedModules/FindChai3D.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindDHD.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FINDDirectX.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindEntactAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindGLUTWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindH3Dfparser.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindH3DUtil.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindHAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindHaptik.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindMd5sum.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindMLHI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindNiFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindOpenHaptics.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindPTHREAD.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindSimballMedical.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindVirtuoseAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/FindWxWidgetsWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/InstallHAPIAndExternals.cmake
                 ${HAPI_SOURCE_DIR}/modules/NSIS.InstallOptions.ini.in
                 ${HAPI_SOURCE_DIR}/modules/NSIS.template.in
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/StripAndAddLibraryDirectories.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/TestIfVCExpress.cmake
                 ${HAPI_SOURCE_DIR}/modules/sharedModules/UseDebian.cmake
           DESTINATION HAPI/build/modules
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/build/CMakeLists.txt
           DESTINATION HAPI/examples/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/DirectXExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/DirectXExample/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/DirectXExample/src/main.cpp
           DESTINATION HAPI/examples/DirectXExample/src
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/FeedbackBufferCollectorExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/FeedbackBufferCollectorExample/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/FeedbackBufferCollectorExample/FeedbackBufferCollectorExample.cpp
           DESTINATION HAPI/examples/FeedbackBufferCollectorExample
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/build/CMakeLists.txt
           DESTINATION HAPI/examples/HAPIDemo/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ForceFieldWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ForceFieldWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HAPIDemo.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HAPIDemo.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HapticShapeConstraintWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/HapticShapeConstraintWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/PositionFunctionWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/PositionFunctionWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/SpringWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/SpringWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/TimeFunctionWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/TimeFunctionWidgetsPage.h
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ViscosityWidgetsPage.cpp
                 ${HAPI_SOURCE_DIR}/../examples/HAPIDemo/ViscosityWidgetsPage.h
           DESTINATION HAPI/examples/HAPIDemo
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/SpringExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/SpringExample/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/SpringExample/SpringExample.cpp
           DESTINATION HAPI/examples/SpringExample
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/SurfaceExample/build/CMakeLists.txt
           DESTINATION HAPI/examples/SurfaceExample/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/SurfaceExample/SurfaceDemo.cpp
           DESTINATION HAPI/examples/SurfaceExample
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/build/CMakeLists.txt
           DESTINATION HAPI/examples/ThreadExamples/build
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/PeriodicThreadCallbacks/PeriodicThreadCallbacks.cpp
           DESTINATION HAPI/examples/ThreadExamples/PeriodicThreadCallbacks
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/SimpleThreadPrint/SimpleThreadPrint.cpp
           DESTINATION HAPI/examples/ThreadExamples/SimpleThreadPrint
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/../examples/ThreadExamples/SimpleThreadPrintLock/SimpleThreadPrintLock.cpp
           DESTINATION HAPI/examples/ThreadExamples/SimpleThreadPrintLock
           COMPONENT HAPI_cpack_sources )

  # Add a cache variable HAPI_DOCS_DIRECTORY used to indicate where the HAPI docs are.
  if( NOT DEFINED HAPI_DOCS_DIRECTORY )
    set( HAPI_DOCS_DIRECTORY_DEFAULT "" )
    if( H3D_USE_DEPENDENCIES_ONLY )
      set( HAPI_DOCS_DIRECTORY_DEFAULT "${HAPI_SOURCE_DIR}/../../doc" )
    elseif( TARGET H3DAPI )
      set( HAPI_DOCS_DIRECTORY_DEFAULT "${H3DAPI_DOCS_DIRECTORY}" )
    endif()
    set( HAPI_DOCS_DIRECTORY "${HAPI_DOCS_DIRECTORY_DEFAULT}" CACHE PATH "Set this to the directory containing the documentation of HAPI." )
    mark_as_advanced( HAPI_DOCS_DIRECTORY )
  endif()

  if( EXISTS ${HAPI_DOCS_DIRECTORY} )
    install( DIRECTORY ${HAPI_DOCS_DIRECTORY}/HAPI
             DESTINATION doc
             COMPONENT HAPI_cpack_headers
             REGEX "(/.svn)|(/CVS)" EXCLUDE )
    install( FILES "${HAPI_DOCS_DIRECTORY}/HAPI Manual.pdf"
             DESTINATION doc
             COMPONENT HAPI_cpack_headers )
  endif()

  # setting names and dependencies between components and also grouping them.
  set( CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DISPLAY_NAME "Runtime" )
  set( CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DESCRIPTION "The runtime libraries ( dlls ) for HAPI." )
  set( CPACK_COMPONENT_HAPI_CPACK_RUNTIME_DEPENDS H3DUtil_cpack_runtime HAPI_cpack_external_runtime )
  set( CPACK_COMPONENT_HAPI_CPACK_RUNTIME_GROUP "HAPI_cpack_group" )
  set( CPACK_COMPONENT_HAPI_CPACK_RUNTIME_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DISPLAY_NAME "Libraries" )
  set( CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DESCRIPTION "HAPI libraries, needed for building against HAPI." )
  set( CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_DEPENDS H3DUtil_cpack_libraries HAPI_cpack_external_source HAPI_cpack_headers )
  set( CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_GROUP "HAPI_cpack_group" )
  set( CPACK_COMPONENT_HAPI_CPACK_LIBRARIES_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_HAPI_CPACK_HEADERS_DISPLAY_NAME "C++ Headers" )
  set( CPACK_COMPONENT_HAPI_CPACK_HEADERS_DESCRIPTION "HAPI C++ headers, needed for building against HAPI." )
  set( CPACK_COMPONENT_HAPI_CPACK_HEADERS_DEPENDS H3DUtil_cpack_headers HAPI_cpack_external_source HAPI_cpack_libraries )
  set( CPACK_COMPONENT_HAPI_CPACK_HEADERS_GROUP "HAPI_cpack_group" )
  set( CPACK_COMPONENT_HAPI_CPACK_HEADERS_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_HAPI_CPACK_SOURCES_DISPLAY_NAME "C++ Source" )
  set( CPACK_COMPONENT_HAPI_CPACK_SOURCES_DESCRIPTION "Everything needed to build HAPI." )
  set( CPACK_COMPONENT_HAPI_CPACK_SOURCES_DEPENDS HAPI_cpack_headers H3DUtil_cpack_sources )
  set( CPACK_COMPONENT_HAPI_CPACK_SOURCES_GROUP "HAPI_cpack_group" )
  set( CPACK_COMPONENT_HAPI_CPACK_SOURCES_INSTALL_TYPES Full )

  set( CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DISPLAY_NAME "Example applications" )
  set( CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DESCRIPTION "The example applications for HAPI." )
  set( CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_DEPENDS HAPI_cpack_runtime )
  set( CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_GROUP "HAPI_cpack_group" )
  set( CPACK_COMPONENT_HAPI_CPACK_EXAMPLES_RUNTIME_INSTALL_TYPES Developer Full )

  set( CPACK_COMPONENT_GROUP_HAPI_CPACK_GROUP_DISPLAY_NAME "HAPI" )
  set( CPACK_COMPONENT_GROUP_HAPI_CPACK_GROUP_DESCRIPTION "An open source cross platform haptics rendering engine with support for several haptics devices. C++ interface only." )
  set( CPACK_COMPONENT_GROUP_H3DUTIL_CPACK_GROUP_PARENT_GROUP "HAPI_cpack_group" )

  # Add a cache variable H3D_cmake_runtime_path to point to cmake binary.
  set( H3D_cmake_runtime_path_default "" )
  if( NOT DEFINED H3D_cmake_runtime_path )
    if( WIN32 AND NOT UNIX )
      set( VERSION_CMAKES "4.0" "3.9" "3.8" "3.7" "3.6" "3.5" "3.4" "3.3" "3.2" "3.1" "3.0" "2.9" "2.8" "2.7" "2.6" )
      foreach( version_cmake ${VERSION_CMAKES} )
        if( EXISTS "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          set( H3D_cmake_runtime_path_default "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()

        if( EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          set( H3D_cmake_runtime_path_default "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()

        if( EXISTS "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          set( H3D_cmake_runtime_path_default "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()
      endforeach()
    else( WIN32 AND NOT UNIX )
      set( H3D_cmake_runtime_path_default "cmake" )
    endif()
    set( H3D_cmake_runtime_path ${H3D_cmake_runtime_path_default} CACHE FILEPATH "The path to the cmake runtime." )
    mark_as_advanced( H3D_cmake_runtime_path )
  endif()

  if( UNIX )
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${HAPI_SOURCE_DIR}/..;/" )
    set( CPACK_SOURCE_GENERATOR TGZ ZIP )
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "hapi-${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION}" )


    set( HAPI_CPACK_IGNORE_PATTERNS ${HAPI_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${HAPI_CPACK_IGNORE_PATTERNS} )
  endif()

  if( H3D_cmake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_cmake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_runtime -P cmake_install.cmake
                                                       COMMAND ${H3D_cmake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_libraries -P cmake_install.cmake
                                                       COMMAND ${H3D_cmake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_examples_runtime -P cmake_install.cmake )

    if( NOT TARGET H3DAPI )

      add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DummyFile
                          COMMAND echo )
      add_custom_target( INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/DummyFile )

      add_custom_command( TARGET INSTALL_RUNTIME_AND_LIBRARIES_ONLY
                          POST_BUILD
                          ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD} )
      add_dependencies( INSTALL_RUNTIME_AND_LIBRARIES_ONLY HAPI ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} )
    endif()
  else( H3D_cmake_runtime_path )
    message( STATUS "H3D_cmake_runtime_path is not set, please set it to continue" )
  endif()

  if( NOT H3D_USE_DEPENDENCIES_ONLY )
    if( NOT TARGET H3DAPI )
      include( CPack )
      if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
        include( UseDebian )
        if( DEBIAN_FOUND )
          ADD_DEBIAN_TARGETS( HAPI )
        endif()
      endif()
    endif()
  endif()
endif()
