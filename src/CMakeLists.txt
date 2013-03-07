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

PROJECT(SfMFiles)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(".")

FIND_PACKAGE(PkgConfig REQUIRED)

# Eigen3
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Boost
# http://goo.gl/xEHzb
FIND_PACKAGE(Boost 1.33 COMPONENTS system filesystem REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

# OptParse
SET(IS_APPLE ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
IF(APPLE)
  # OptParser
  SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /Users/danielcabrinihauagge/Projects/OptParser/build )
  FIND_LIBRARY(OPTPARSER_LIBRARY OptParser)
  #SET(EXTRA_LIBS ${_LIBRARY})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -F /Users/danielcabrinihauagge/Projects/OptParser/build  -framework OptParser")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -framework OptParser")
ELSE()
  PKG_CHECK_MODULES(OPTPARSER REQUIRED optparser)
  INCLUDE_DIRECTORIES(${OPTPARSER_INCLUDE_DIRS})
  LINK_DIRECTORIES(${OPTPARSER_LIBRARY_DIRS})
  SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS}")
ENDIF (APPLE)

ADD_DEFINITIONS( -DSFMF_COMPILING )

ADD_LIBRARY( SfMFiles 
  SHARED 
  BundlerData.cpp  PMVSData.cpp
  SfMFiles/sfmfiles)

TARGET_LINK_LIBRARIES(SfMFiles ${Boost_LIBRARIES})

# Some utilities
ADD_EXECUTABLE(bundler_file_converter bundler_file_converter.cpp)
TARGET_LINK_LIBRARIES(bundler_file_converter SfMFiles ${Boost_LIBRARIES})

ADD_EXECUTABLE(testBundlerData testBundlerData.cpp)
TARGET_LINK_LIBRARIES(testBundlerData SfMFiles ${Boost_LIBRARIES})

ADD_EXECUTABLE(testPMVSData testPMVSData.cpp)
TARGET_LINK_LIBRARIES(testPMVSData SfMFiles ${Boost_LIBRARIES})

ADD_EXECUTABLE(pmvs_info pmvs_info.cpp)
TARGET_LINK_LIBRARIES(pmvs_info SfMFiles ${Boost_LIBRARIES})

ADD_EXECUTABLE(pmvs_merge pmvs_merge.cpp)
TARGET_LINK_LIBRARIES(pmvs_merge SfMFiles ${Boost_LIBRARIES})

ADD_EXECUTABLE(pmvs_filter pmvs_filter.cpp)
TARGET_LINK_LIBRARIES(pmvs_filter SfMFiles ${Boost_LIBRARIES} ${OPTPARSER_LIBRARY} ${OPTPARSER_LIBRARIES})

ADD_EXECUTABLE(pmvs2ply pmvs2ply.cpp)
TARGET_LINK_LIBRARIES(pmvs2ply SfMFiles ${Boost_LIBRARIES} ${OPTPARSER_LIBRARY} ${OPTPARSER_LIBRARIES})

ADD_EXECUTABLE(pmvs2bundler pmvs2bundler.cpp)
TARGET_LINK_LIBRARIES(pmvs2bundler SfMFiles ${Boost_LIBRARIES} ${OPTPARSER_LIBRARY} ${OPTPARSER_LIBRARIES})

ADD_EXECUTABLE(bundler_morecameras bundler_morecameras.cpp)
TARGET_LINK_LIBRARIES(bundler_morecameras SfMFiles ${Boost_LIBRARIES})

# How to package and where to install stuff
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  # Build a universal binary
  SET( CMAKE_OSX_ARCHITECTURES x86_64)#;i386 )

  # MacOS Framework
  SET_TARGET_PROPERTIES( SfMFiles PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION none
    PUBLIC_HEADER "sfmfiles" 
    DEBUG_POSTIFX -d
    )
  
  INSTALL( TARGETS SfMFiles
    FRAMEWORK DESTINATION Library/Frameworks
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )
ELSE()  
  INSTALL_FILES(/include/SfMFiles FILES SfMFiles/sfmfiles)
  INSTALL_FILES(/include/SfMFiles .hpp SfMFiles/BundlerData.hpp SfMFiles/PMVSData.hpp)
  INSTALL_TARGETS(/lib SfMFiles)
  INSTALL_PROGRAMS(/bin FILES pmvs_merge pmvs_info bundler_morecameras pmvs_filter pmvs2ply pmvs2bundler) 
  #INSTALL_TARGETS(/lib RUNTIME_DIRECTORY /bin SharedLibraryTarget)
ENDIF()
