# Copyright (C) 2011 by Daniel Cabrini Hauagge
#
# Permission is hereby granted, free  of charge, to any person obtaining
# a  copy  of this  software  and  associated  documentation files  (the
# "Software"), to  deal in  the Software without  restriction, including
# without limitation  the rights to  use, copy, modify,  merge, publish,
# distribute,  sublicense, and/or sell  copies of  the Software,  and to
# permit persons to whom the Software  is furnished to do so, subject to
# the following conditions:
#
# The  above  copyright  notice  and  this permission  notice  shall  be
# included in all copies or substantial portions of the Software.
#
# THE  SOFTWARE IS  PROVIDED  "AS  IS", WITHOUT  WARRANTY  OF ANY  KIND,
# EXPRESS OR  IMPLIED, INCLUDING  BUT NOT LIMITED  TO THE  WARRANTIES OF
# MERCHANTABILITY,    FITNESS    FOR    A   PARTICULAR    PURPOSE    AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE,  ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(PkgConfig REQUIRED)

# Eigen3
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Boost
#SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} 
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

INCLUDE_DIRECTORIES(".")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Build a universal binary
  SET( CMAKE_OSX_ARCHITECTURES x86_64;i386 )

  # The framework
  ADD_LIBRARY( SfMFiles 
    SHARED 
    BundlerData.cpp  PMVSData.cpp
    BundlerData.hpp PMVSData.hpp)

  # MacOS Framework
  SET_TARGET_PROPERTIES( SfMFiles PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION none
    PUBLIC_HEADER "BundlerData.hpp;PMVSData.hpp" 
    DEBUG_POSTIFX -d
    )
  
  INSTALL( TARGETS SfMFiles
    FRAMEWORK DESTINATION Library/Frameworks
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )
ELSE()
  ADD_LIBRARY(SfMFiles SHARED
    BundlerData.cpp  PMVSData.cpp BundlerData.hpp PMVSData.hpp)
  
  INSTALL_FILES(/include/SfMFiles .hpp BundlerData.hpp PMVSData.hpp)
  INSTALL_TARGETS(/lib SfMFiles)
  #INSTALL_TARGETS(/lib RUNTIME_DIRECTORY /bin SharedLibraryTarget)


ENDIF()

# Some utilities
ADD_EXECUTABLE(bundler_file_converter bundler_file_converter.cpp)
TARGET_LINK_LIBRARIES(bundler_file_converter SfMFiles)

ADD_EXECUTABLE(testBundlerData testBundlerData.cpp)
TARGET_LINK_LIBRARIES(testBundlerData SfMFiles)
