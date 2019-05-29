if( NOT TARGET HAPI )
  message( FATAL_ERROR "Include file HAPICPack.cmake require the target HAPI to exist. Please add HAPI/build/CMakeLists.txt as subdirectory first." )
endif()

# Add all sources, they are added to a variable called HAPI_SRCS defined
# in the included file. All header files are added to a variable called
# HAPI_HEADERS.
include( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )
list( APPEND HAPI_HEADERS ${HAPI_SOURCE_DIR}/../include/HAPI/StdAfx.h )
list( APPEND HAPI_SRCS ${HAPI_SOURCE_DIR}/../src/StdAfx.cpp )

# If cpack should be configured.
if( GENERATE_H3D_PACKAGE_PROJECT )
  if( WIN32 )
    handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES H3D_EXTERNAL_ROOT
                                                  OLD_VARIABLE_NAMES HAPI_CPACK_EXTERNAL_ROOT
                                                  DOC_STRINGS "Set to the External directory used with HAPI, needed to pack properly. If not set FIND_modules will be used instead." )

    # Add a cache variable which indicates where the Externals directory used for packaging
    # HAPI is located. If not set then FIND modules will be used instead.
    if( NOT DEFINED H3D_EXTERNAL_ROOT )
      if( NOT DEFINED H3D_EXTERNAL_ROOT )
        set( h3d_external_root_default "" )
        if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "HAPI" )
          foreach( external_include_dir_tmp ${HAPI_INCLUDE_DIRS} )
            if( EXISTS ${external_include_dir_tmp}/../include/pthread )
              set( h3d_external_root_default "${external_include_dir_tmp}/.." )
            endif()
          endforeach()
        else()
          set( h3d_external_root_default "$ENV{H3D_EXTERNAL_ROOT}" )
        endif()
        set( H3D_EXTERNAL_ROOT "${h3d_external_root_default}" CACHE PATH "Set to the External directory used with HAPI, needed to pack properly. If not set FIND_modules will be used instead." )
        mark_as_advanced( H3D_EXTERNAL_ROOT )
      endif()
    else()
      set( H3D_EXTERNAL_ROOT ${H3D_EXTERNAL_ROOT} )
    endif()
  endif()
  include( ${HAPI_SOURCE_DIR}/../../H3DUtil/build/H3DUtilCPack.cmake )

  if( NOT TARGET H3DAPI )
    set( CPACK_ALL_INSTALL_TYPES Full Developer )
    set( CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/localModules ${HAPI_SOURCE_DIR}/modules )
    set( CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform, device independent haptics library." )
    set( CPACK_PACKAGE_VENDOR "SenseGraphics AB" )
    set( CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
    set( CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../README.md" )
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

      if( GENERATE_HAPI_example_PROJECTS )
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
          set( ms_redist_install_command_1 " Set output Path\\n  SetOutPath \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n"
                                           " Code to install Visual studio redistributable\\n  File \\\"${Temp_vc${redist_version}_redist}\\\"\\n" )
          set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${ms_redist_install_command_1} )
          set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Check if uninstall vc redist \\n  MessageBox MB_YESNO \\\"Do you want to uninstall Visual studio ${redist_version} redistributable? It is recommended if no other applications use it.\\\" IDYES uninstall_vcredist_yes IDNO uninstall_vcredist_no\\n"
                                                   " A comment \\n  uninstall_vcredist_yes:\\n"
                                                   ${ms_redist_install_command_1} )
          if( ${redist_version} LESS 9 )
            set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /i vcredist.msi /qn\\\"'\\n" )
            set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q:a /norestart /c:\\\"msiexec /x vcredist.msi /qn\\\"'\\n" )
          else()
            set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /norestart \\\"'" )
            set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   " Execute silent and wait\\n  ExecWait '\\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\" /q /uninstall \\\"'" )
          endif()
          set( ms_redist_install_command_2 " Wait a bit for system to unlock file.\\n  Sleep 1000\\n"
                                           " Delete file\\n  Delete \\\"$INSTDIR\\\\vc${redist_version}\\\\${VC${redist_version}_FILE_NAME}\\\"\\n"
                                           " Reset output Path\\n  SetOutPath \\\"$INSTDIR\\\"\\n"
                                           " Remove folder\\n  RMDir /r \\\"$INSTDIR\\\\vc${redist_version}\\\"\\n\\n" )
          set( CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
                                                 ${ms_redist_install_command_2} )
          set( CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}
                                                   ${ms_redist_install_command_2}
                                                   " A comment \\n  uninstall_vcredist_no:\\n\\n" )
        endif()
      endforeach()

      # Modify path since in the NSIS template.
      set( CPACK_NSIS_MODIFY_PATH "ON" )
    endif()
  endif()

  if( WIN32 )
    # external_includes and external_include_install_paths must be of equal lengths.
    # The reason for defining these variables here is in case we want to add functionality
    # to configure installation in some other way (using FIND-modules for example).
    set( external_includes "" )
    set( external_include_install_paths "" )
    # The external_include_files are installed directly in External/include
    set( external_include_files "" )
    set( external_libraries "" )
    set( external_static_libraries "" )
    set( external_binaries "" )

    set( external_bin_path "bin32" )
    set( external_bin_replace_path "bin64" )
    if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      set( external_bin_path "bin64" )
      set( external_bin_replace_path "bin32" )
    endif()

    if( EXISTS ${H3D_EXTERNAL_ROOT} )
      set( external_includes ${H3D_EXTERNAL_ROOT}/include/GL/
                             ${H3D_EXTERNAL_ROOT}/include/DHD-API/
                             ${H3D_EXTERNAL_ROOT}/include/Simball/
                             ${H3D_EXTERNAL_ROOT}/include/wx/
                             ${H3D_EXTERNAL_ROOT}/include/fparser/ )
      set( external_include_install_paths External/include/GL
                                          External/include/DHD-API
                                          External/include/Simball
                                          External/include/wx
                                          External/include/fparser )
      if( EXISTS ${H3D_EXTERNAL_ROOT}/include/chai3d/ )
        set( external_includes ${external_includes} ${H3D_EXTERNAL_ROOT}/include/chai3d/ )
        set( external_include_install_paths ${external_include_install_paths} External/include/chai3d )
      endif()
      set( external_include_ignore_pattern "((/.svn)|(/CVS))|(/old)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)"
                                           "(/.svn)|(/CVS)" )

      set( external_include_files ${H3D_EXTERNAL_ROOT}/include/VirtuoseAPI.h
                                  ${H3D_EXTERNAL_ROOT}/include/EntactAPI.h )

      set( external_libraries ${H3D_EXTERNAL_ROOT}/lib32/freeglut.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/virtuoseDLL.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/EntactAPI.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/EntactAPI_d.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/SimballMedicalHID.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/dhdms64.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/dhdms.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/fparser.lib
                              ${H3D_EXTERNAL_ROOT}/lib32/fparser_d.lib )

      set( wxlibs core adv aui html media propgrid ribbon stc webview xrc gl qa richtext )
      foreach( library_name ${wxlibs} )
        set( external_libraries ${external_libraries}
                                ${H3D_EXTERNAL_ROOT}/lib32/wxmsw30u_${library_name}.lib
                                ${H3D_EXTERNAL_ROOT}/lib32/wxmsw30ud_${library_name}.lib )
        #set( external_static_libraries ${external_static_libraries}
        #                               ${H3D_EXTERNAL_ROOT}/lib32/static/wxmsw30u_${library_name}.lib
        #                               ${H3D_EXTERNAL_ROOT}/lib32/static/wxmsw30ud_${library_name}.lib )
        set( external_binaries ${external_binaries}
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_${library_name}_vc_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30u_${library_name}_vc_x64_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30ud_${library_name}_vc_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxmsw30ud_${library_name}_vc_x64_custom.dll )
      endforeach()
      set( wxlibs "" _net _xml )
      # IN LISTS means that the empty argument is parsed
      foreach( library_name IN LISTS wxlibs )
        set( external_libraries ${external_libraries}
                                ${H3D_EXTERNAL_ROOT}/lib32/wxbase30u${library_name}.lib
                                ${H3D_EXTERNAL_ROOT}/lib32/wxbase30ud${library_name}.lib )
        set( external_binaries ${external_binaries}
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u${library_name}_vc_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30u${library_name}_vc_x64_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30ud${library_name}_vc_custom.dll
                               ${H3D_EXTERNAL_ROOT}/${external_bin_path}/wxbase30ud${library_name}_vc_x64_custom.dll )
      endforeach()

      set( wxlibs expat jpeg png regexu scintilla tiff zlib )
      foreach( library_name ${wxlibs} )
        set( external_libraries ${external_libraries}
                                ${H3D_EXTERNAL_ROOT}/lib32/wx${library_name}.lib
                                ${H3D_EXTERNAL_ROOT}/lib32/wx${library_name}d.lib )
        set( external_binaries ${external_binaries} )
      endforeach()

      set( external_static_libraries ${external_static_libraries}
                                     ${H3D_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10.lib
                                     ${H3D_EXTERNAL_ROOT}/lib32/static/chai3d_complete_vc10_d.lib )

      set( external_binaries ${external_binaries}
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/freeglut.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/virtuoseDLL.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/EntactAPI.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/SimballMedicalHID.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/HapticAPI.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/HapticMasterDriver.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/fparser.dll
                             ${H3D_EXTERNAL_ROOT}/${external_bin_path}/fparser_d.dll )

    else()
      message( WARNING "H3D_EXTERNAL_ROOT must be set to the External directory used by HAPI in order to package properly." )
    endif()

    if( external_includes )
      list( LENGTH external_includes external_includes_length )
      math( EXPR external_includes_length "${external_includes_length} - 1" )
      foreach( val RANGE ${external_includes_length} )
        list( GET external_includes ${val} val1 )
        list( GET external_include_install_paths ${val} val2 )
        list( GET external_include_ignore_pattern ${val} val3 )
        install( DIRECTORY ${val1}
                 DESTINATION ${val2}
                 COMPONENT HAPI_cpack_external_source
                 REGEX ${val3} EXCLUDE )
      endforeach()
    endif()

    foreach( include_file ${external_include_files} )
      if( EXISTS ${include_file} )
        install( FILES ${include_file}
                 DESTINATION External/include
                 COMPONENT HAPI_cpack_external_source )
      endif()
    endforeach()

    foreach( library ${external_libraries} )
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

    foreach( library ${external_static_libraries} )
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

    foreach( binary ${external_binaries} )
      if( EXISTS ${binary} )
        install( FILES ${binary}
                 DESTINATION External/${external_bin_path}
                 COMPONENT HAPI_cpack_external_runtime )
      endif()

      string( REGEX REPLACE "(/${external_bin_path}/)" "/${external_bin_replace_path}/" other_binary ${binary} )
      if( EXISTS ${other_binary} )
        install( FILES ${other_binary}
                 DESTINATION External/${external_bin_replace_path}
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
                 ${HAPI_SOURCE_DIR}/../README.md
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

  install( FILES ${HAPI_SOURCE_DIR}/localModules/NSIS.InstallOptions.ini.in
                 ${HAPI_SOURCE_DIR}/localModules/NSIS.template.in
           DESTINATION HAPI/build/localModules
           COMPONENT HAPI_cpack_sources )

  install( FILES ${HAPI_SOURCE_DIR}/modules/FindChai3D.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindDHD.cmake
                 ${HAPI_SOURCE_DIR}/modules/FINDDirectX.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindEntactAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/Findfparser.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindGLUT.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindGLUTWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindH3Dfparser.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindH3DUtil.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindHAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindHaptik.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindMd5sum.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindMLHI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindNiFalconAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindOpenHaptics.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindPTHREAD.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindSimballMedical.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindVirtuoseAPI.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindWxWidgetsWin.cmake
                 ${HAPI_SOURCE_DIR}/modules/FindwxWidgets.cmake
                 ${HAPI_SOURCE_DIR}/modules/H3DCommonFindModuleFunctions.cmake
                 ${HAPI_SOURCE_DIR}/modules/H3DCommonFunctions.cmake
                 ${HAPI_SOURCE_DIR}/modules/H3DExternalSearchPath.cmake
                 ${HAPI_SOURCE_DIR}/modules/H3DUtilityFunctions.cmake
                 ${HAPI_SOURCE_DIR}/modules/InstallHAPIAndExternals.cmake
                 ${HAPI_SOURCE_DIR}/modules/StripAndAddLibraryDirectories.cmake
                 ${HAPI_SOURCE_DIR}/modules/TestIfVCExpress.cmake
                 ${HAPI_SOURCE_DIR}/modules/UseDebian.cmake
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

  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES HAPI_documentation_directory H3D_CMake_runtime_path
                                                OLD_VARIABLE_NAMES HAPI_DOCS_DIRECTORY H3D_cmake_runtime_path
                                                DOC_STRINGS "Set this to the directory containing the manual and generated doxygen documentation of HAPI."
                                                            "The path to the cmake runtime." )

  # Add a cache variable HAPI_DOCS_DIRECTORY used to indicate where the HAPI docs are.
  if( NOT DEFINED HAPI_documentation_directory )
    set( HAPI_DOCS_DIRECTORY_DEFAULT "" )
    if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "HAPI" )
      set( HAPI_DOCS_DIRECTORY_DEFAULT "${HAPI_SOURCE_DIR}/../../doc" )
    elseif( TARGET H3DAPI )
      set( HAPI_DOCS_DIRECTORY_DEFAULT "${H3DAPI_documentation_directory}" )
    endif()
    set( HAPI_documentation_directory "${HAPI_DOCS_DIRECTORY_DEFAULT}" CACHE PATH "Set this to the directory containing the manual and generated doxygen documentation of HAPI." )
    mark_as_advanced( HAPI_documentation_directory )
  endif()

  if( EXISTS ${HAPI_documentation_directory} )
    install( DIRECTORY ${HAPI_documentation_directory}/HAPI
             DESTINATION doc
             COMPONENT HAPI_cpack_headers
             REGEX "(/.svn)|(/CVS)" EXCLUDE )
    install( FILES "${HAPI_documentation_directory}/HAPI Manual.pdf"
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

  # Add a cache variable H3D_CMake_runtime_path to point to cmake binary.
  set( h3d_cmake_runtime_path_default "" )
  if( NOT DEFINED H3D_CMake_runtime_path )
    if( WIN32 AND NOT UNIX )
      set( VERSION_CMAKES "4.0" "3.9" "3.8" "3.7" "3.6" "3.5" "3.4" "3.3" "3.2" "3.1" "3.0" "2.9" "2.8" "2.7" "2.6" )
      foreach( version_cmake ${VERSION_CMAKES} )
        if( EXISTS "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program Files/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()

        if( EXISTS "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program Files (x86)/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()

        if( EXISTS "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          set( h3d_cmake_runtime_path_default "C:/Program/CMake ${version_cmake}/bin/cmake.exe" )
          break()
        endif()
      endforeach()
    else()
      set( h3d_cmake_runtime_path_default "cmake" )
    endif()
    set( H3D_CMake_runtime_path ${h3d_cmake_runtime_path_default} CACHE FILEPATH "The path to the cmake runtime." )
    mark_as_advanced( H3D_CMake_runtime_path )
  endif()

  if( UNIX )
    set( CPACK_SOURCE_INSTALLED_DIRECTORIES "${HAPI_SOURCE_DIR}/..;/" )
    set( CPACK_SOURCE_GENERATOR TGZ ZIP )
    set( CPACK_SOURCE_PACKAGE_FILE_NAME "hapi-${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION}" )


    set( HAPI_CPACK_IGNORE_PATTERNS ${HAPI_CPACK_IGNORE_PATTERNS}
            "/CVS/;/.svn/;/.bzr/;/.hg/;/.git.*/;.swp$;.#;/#;~$" )
    set( CPACK_SOURCE_IGNORE_FILES ${HAPI_CPACK_IGNORE_PATTERNS} )
  endif()

  if( H3D_CMake_runtime_path )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_POST_BUILD}
                                                       COMMAND ${H3D_CMake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_runtime -P cmake_install.cmake
                                                       COMMAND ${H3D_CMake_runtime_path}
                                                       ARGS -DBUILD_TYPE=$(Configuration) -DCOMPONENT=HAPI_cpack_libraries -P cmake_install.cmake
                                                       COMMAND ${H3D_CMake_runtime_path}
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
  else()
    message( STATUS "H3D_CMake_runtime_path is not set, please set it to continue" )
  endif()

  if( ${CMAKE_PROJECT_NAME} STREQUAL "HAPI" )
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
