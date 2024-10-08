cmake_minimum_required (VERSION 3.5)

project(RayTracing)

# C++ 11 is required
set(CMAKE_CXX_STANDARD 11)

OPTION(RayTracing_DOUBLE_AS_FLOAT "Use 64-bit floats" OFF)

IF (RayTracing_DOUBLE_AS_FLOAT)
  ADD_DEFINITIONS ( -D RayTracing_DOUBLE_AS_FLOAT )
ENDIF()

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

include_directories(src/core/)


###########################################################################
# header-only libraries

INCLUDE_DIRECTORIES (
  src/vendor/header_only
)

###########################################################################
# glog

SET(WITH_GFLAGS OFF CACHE BOOL "Use gflags")
SET(BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)
IF(WIN32)
  ADD_DEFINITIONS( -D GOOGLE_GLOG_DLL_DECL= )
ENDIF()
ADD_DEFINITIONS( -D HAVE_SNPRINTF)
ADD_SUBDIRECTORY(src/vendor/glog)
SET_PROPERTY(TARGET glog logging_unittest demangle_unittest utilities_unittest stl_logging_unittest signalhandler_unittest stacktrace_unittest symbolize_unittest PROPERTY FOLDER "vendor")
INCLUDE_DIRECTORIES (
  src/vendor/glog/src
  ${CMAKE_BINARY_DIR}/src/vendor/glog
)

###########################################################################
# assimp
ADD_SUBDIRECTORY(src/vendor/assimp)
SET_PROPERTY(TARGET assimp assimp_cmd uninstall unit UpdateAssimpLibsDebugSymbolsAndDLLs zlibstatic PROPERTY FOLDER "vendor")
INCLUDE_DIRECTORIES (
  src/vendor/assimp/include
  ${CMAKE_BINARY_DIR}/src/vendor/assimp
)

file(GLOB_RECURSE CORE ./src/core/*.h ./src/core/*.cpp)
source_group("core" FILES ${CORE})

file(GLOB SRCS ./src/*.cpp)

add_executable(${PROJECT_NAME} ${CORE} ${SRCS})

target_link_libraries( ${PROJECT_NAME} 
    PRIVATE 
	glog::glog
	assimp::assimp
)

# Installation
INSTALL ( TARGETS
  ${PROJECT_NAME}
  DESTINATION
  bin
  )