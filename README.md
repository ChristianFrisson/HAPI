# HAPI
HAPI is a cross-platform, device independent easily extendable API that can be
used to add haptics to an application. For information on how you are allowed
to use HAPI see the LICENSE file in the same folder as this file.

This file contains complete installation instructions for Linux, Mac and Windows.


## External Libraries
HAPI depends on other libraries. Some of them are required for HAPI to
compile others are optional. The build system used by HAPI will detect which
libraries are installed on the system and only enable the features that the
system can support. For Windows most libraries are included. Those missing must
be downloaded, compiled and installed on the system used if that feature is
desired. The libraries used by HAPI are:
 
**Required libraries :** Without these HAPI will not function.
  - pthread. Included with Windows distribution, most probably installed on
    other systems.
 
**Optional libraries :** Without these some features of HAPI will be disabled
 and some of the distributed examples will not work.
  - wxWidgets. Needed for one of the examples distributed. http://www.wxwidgets.org/
  - OpenGL. Installed on all systems. For those interested see http://www.opengl.org/
  - GLUT or Freeglut. Freeglut is a better choice. http://freeglut.sourceforge.net/
    Must be compiled with -fexceptions as a compiler flag to allow c++ exceptions.
    Used by one of the examples.
  - OpenHaptics, for support of devices from SensAble. Obtained through SensAble.
    OpenHaptics is not open source. http://www.sensable.com
  - DHD-API, obtained through ForceDimension. http://www.forcedimension.com
  - CHAI3D. Extra haptic rendering feature. http://www.chai3d.org
  - FreeImage. Used to read images. http://freeimage.sourceforge.net
  - zlib. Required for parsing zipped files. http://www.zlib.net/
  - Teem. Required for reading the Nrrd file format. http://teem.sourceforge.net/
  - DICOM toolkit. Required for reading dicom files. http://dicom.offis.de/dcmtk
  - HDAL SDK. Required for connecting to the haptics device Falcon by Novint.
    http://home.novint.com/
  - libnifalcon. Required for Falcon support on linux.
    http://sourceforge.net/projects/libnifalcon/
  - HapticAPI. Needed to compile with support for Haptic Master, a device from Moog.
    Contact Moog to get HapticAPI. Note that HapticAPI is included in the Windows
    distribution of HAPI. http://www.moog.com/Industrial/
  - EntactAPI. Needed to compile with support for devices from Entact. Included with
    the windows distrubition of HAPI. See http://www.entactrobotics.com/ for more info.
  - Virtuose API. Needed to compile with support for devices from Haption. Included with
    the windows distribution of HAPI. See www.haption.com for more info.
  - SimballMedical API. Needed for the SimballHapticsDevice class. Included with
    the windows distribution of HAPI. See http://www.g-coder.com for more info.
    Note that this is not really a haptics device at the moment.
  - Quanser API. Needed to compile with support for Quanser devices.
    See www.quanser.com for more info.
  - MLHI API. Needed to compile with support for MLHI haptics device. That is
    devices from Butterfly Haptics.
  
  Note that all haptics libraries does not exist for all operating systems.


## Installation on Linux
Using debian packages:
1. Modify sources.list ( or whatever file is used to specify urls to debian
   packages ) by adding ftp://www.h3dapi.org/pub/releases/linux/debian/
   to it.  
   On Ubuntu 18.04 simply use:  
     ``` 
     sudo gedit /etc/apt/sources.list
     ```
   Then add
     ``` 
     deb ftp://www.h3dapi.org/pub/releases/linux/debian bionic sensegraphics
     deb-src ftp://www.h3dapi.org/pub/releases/linux/debian bionic sensegraphics
     ``` 
   If you use a different linux version simply switch "bionic" for the name
   of your distribution.

2. ` sudo apt-get update `
3. ` sudo apt-get install libhapi1 `
4. If examples are required use:
   ```
   apt-get source libhapi
   ```
   then navigate to libhapi-directory/examples/build
   ```
   mkdir linux
   cd linux
   cmake ../
   make
   ```

### Building from source:
Since you have this file you have already obtained and unpacked the source for
HAPI. To build HAPI on Linux follow these steps.
1. Install version 2.8.7 or later of CMake. To do this on Ubuntu open a
   terminal and write:
   ```
   sudo apt-get install cmake
   ```

2. Download and install H3DUtil.

3. HAPI depends on other libraries. The build system used by HAPI will 
   detect which libraries are installed on the system and only enable the
   features that the system can support. If more features are desired the
   corresponding library has to be installed. Note that some libraries are
   required for HAPI to compile. See the list of external libraries in the
   beginning of this README.
   
   If DHD-API is obtained then libusb needs to be installed .
   PCISCAN_LIBRARY should also be set if the library is not found in
   default location.

   If using Ubuntu, several of these libraries can be obtained
   by using the apt-get feature. Note that for previous or later versions of
   Ubuntu than 18.04 the required and/or optional libraries might be of newer versions
   that those given here. The newer versions should work with HAPI but it might not
   be guaranteed. In a terminal write:
   ```
   sudo apt-get update sudo apt-get install gcc g++ libglew-dev freeglut3 freeglut3-dev alien
   ```
   The first two are the C++ compilers needed to compile HAPI.

   HAPI support more devices but only the two mentioned are tested. See the wiki
   and/or manual for more information.

   To install libusb required by DHD-api from ForceDimension on Ubuntu do the
   following:
   ```
   sudo apt-get install libusb++-dev
   ```
   
   In version 8.04 of Ubuntu another library might need to be installed:
   ```
   sudo apt-get install libxi-dev libxmu-dev
   ```   

4. In the terminal change folder to the HAPI/build folder. Write:
   ```
   cmake .
   ```
   This will generate a make file. To use the makefile write:
   ```
   make
   ```

   HAPI will be built. When the make finished write:
   ```
   sudo make install
   ```
   HAPI libraries are now installed on your system. But there is no
   application installed that use HAPI libraries.

5. There are example applications in this distribution that use HAPI. To build
   these simply do the following:
    Change folder to HAPI/examples.
    ```
    cmake .
    ```
   To test one of the examples do the following.  
    Change folder to HAPI/examples/SurfaceExample/build
    ./SurfaceExample


## Installation on Windows
To install HAPI on Windows follow these steps:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. In the installer, on the page where you choose what to install,
   expand the H3DAPI line and deselect and then select HAPI runtime.
   Then proceed with installation.

### Building from source:
1. Go to www.h3d.org.
2. Download latest release of H3DAPI.
3. In the installer, on the page where you choose what to install,
   expand the H3DAPI line and deselect and then select HAPI (all features of HAPI).
   Then proceed with installation.
4. Use CMake to generate project files for your compiler. Tested with visual
   studio 2010, 2012, 2013, 2015 and 2017.  
5. Open the solution file and build the INSTALL target.
6. Build examples in HAPI/examples/build directory.


## Installation on MacOS X
Currently you need to build HAPI on MacOS X yourself. Since you have this
file you have already obtained and unpacked the source for HAPI. To build
HAPI on MacOS X follow the steps below.

### Building from source:
1. Install version 2.8.7 or later of CMake.
2. Install XCode. If gcc is an unrecognized command in the terminal after
   installing XCode there is a high probability that CMake will not be able to
   generate build files for gcc and/or XCode. Another version of XCode could be
   used or fix it in some other way.
3. HAPI depends on other libraries. The build system used by HAPI will 
   detect which libraries are installed on the system and only enable the
   features that the system can support. If more features are desired the
   corresponding library has to be installed. Note that some libraries are
   required for HAPI to compile. See the list of external libraries in the
   beginning of this README.
4. Generate build files for your build system using CMake. Either use the
   console version as in the guide for linux above or use the GUI version in
   which the first textbox should contain the location of the CMakeLists.txt
   file. This file is located in HAPI/build. The second box is the location
   of where the build files will be created. Use for example HAPI/build/OSX.
   Press configure, choose which system to generate build files for wait and
   press configure again until the generate button can be used. Press generate
   and then the build files will be in the chosen folder. If "Unix Makefile"
   are chosen then proceed from step 3 in the linux guide above.
5. There are example applications in this distribution that use HAPI. To build
   these use the CMakeLists.txt located in the folder HAPI/examples.

   Run any of the built examples to test your HAPI build.
