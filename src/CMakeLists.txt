# Copyright (C) 2011 by Daniel Hauagge
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
# ----------------------------------------------------------------------

PROJECT(SfMFiles)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(".")

ADD_DEFINITIONS( -DSFMF_COMPILING )

SUBDIRS(utils test)

# ----------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------
FIND_PACKAGE(PkgConfig REQUIRED)

# Eigen3
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Boost (http://goo.gl/xEHzb)
FIND_PACKAGE(Boost 1.33 COMPONENTS system filesystem iostreams REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# Command line core
SET(IS_APPLE ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
IF(APPLE)
  # Command line core
  #SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /Users/danielhauagge/Projects/CommandLineCore/build )
  FIND_LIBRARY(CMDCORE_LIBRARIES CMDCore)

  #SET(EXTRA_LIBS ${_LIBRARY})
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -F /Users/danielhauagge/Projects/CommandLineCore/build  -framework CMDCore")
  # SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -framework CMDCore")
ELSE()
  PKG_CHECK_MODULES(CMDCORE REQUIRED cmdcore)
  INCLUDE_DIRECTORIES(${CMDCORE_INCLUDE_DIRS})
  LINK_DIRECTORIES(${CMDCORE_LIBRARY_DIRS})
  # SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS}")
ENDIF (APPLE)

ADD_LIBRARY( SfMFiles 
  SHARED 
  utils.hpp                 utils.cpp
  SfMFiles/BundlerData.hpp  BundlerData.cpp  
  SfMFiles/PMVSData.hpp     PMVSData.cpp            
  SfMFiles/sfmfiles )

TARGET_LINK_LIBRARIES(SfMFiles ${Boost_LIBRARIES} ${CMDCORE_LIBRARIES})

# ----------------------------------------------------------------------
# How to package and where to install stuff
# ----------------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  MESSAGE(STATUS "Making framework")

  # Build a universal binary
  SET( CMAKE_OSX_ARCHITECTURES x86_64) #;i386 ) # Have to leave out i386 because boost is compiled only in 64 bits

  # MacOS Framework
  SET_TARGET_PROPERTIES( SfMFiles PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION Current
    PUBLIC_HEADER "SfMFiles/sfmfiles;SfMFiles/BundlerData.hpp;SfMFiles/PMVSData.hpp"
    DEBUG_POSTIFX -d
    )
  
  INSTALL(TARGETS SfMFiles
          FRAMEWORK DESTINATION "/Library/Frameworks" COMPONENT bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)

ELSE()  
  INSTALL_FILES(/include/SfMFiles FILES SfMFiles/sfmfiles)
  INSTALL_FILES(/include/SfMFiles .hpp SfMFiles/BundlerData.hpp SfMFiles/PMVSData.hpp)
  INSTALL_TARGETS(/lib SfMFiles)
  #INSTALL_TARGETS(/lib RUNTIME_DIRECTORY /bin SharedLibraryTarget)

  # Generating the .pc file for pkg-config
  CONFIGURE_FILE("sfmfiles.pc.in" "${CMAKE_BINARY_DIR}/sfmfiles.pc" @ONLY IMMEDIATE)
  INSTALL(FILES ${CMAKE_BINARY_DIR}/sfmfiles.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
ENDIF()

# ----------------------------------------------------------------------
# Optimization flags
# ----------------------------------------------------------------------

SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -msse -msse2")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -msse -msse2")

