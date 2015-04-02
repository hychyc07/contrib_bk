#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "mini_lemon")
PROJECT(${PROJECT_NAME})
#set (PROJECT_BINARY_DIR ${BODYSCHEMALIB_BINARY_DIR}/mini_lemon CACHE STRING "Lemon project binary dir")
message (STATUS "lemon binary dir: ${PROJECT_BINARY_DIR}")

INCLUDE(FindPythonInterp)
INCLUDE(FindWget)

IF(EXISTS ${PROJECT_SOURCE_DIR}/cmake/version.cmake)
  INCLUDE(${PROJECT_SOURCE_DIR}/cmake/version.cmake)
ELSEIF(DEFINED ENV{LEMON_VERSION})
  SET(LEMON_VERSION $ENV{LEMON_VERSION} CACHE STRING "LEMON version string.")
ELSE()
  EXECUTE_PROCESS(
    COMMAND ${PYTHON_EXECUTABLE} ./scripts/chg-len.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE HG_REVISION_PATH
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  EXECUTE_PROCESS(
    COMMAND hg id -i
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE HG_REVISION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  IF(HG_REVISION STREQUAL "")
    SET(HG_REVISION_ID "hg-tip")
  ELSE()
    IF(HG_REVISION_PATH STREQUAL "")
      SET(HG_REVISION_ID ${HG_REVISION})
    ELSE()
      SET(HG_REVISION_ID ${HG_REVISION_PATH}.${HG_REVISION})
    ENDIF()
  ENDIF()
  SET(LEMON_VERSION ${HG_REVISION_ID} CACHE STRING "LEMON version string.")
ENDIF()

SET(PROJECT_VERSION ${LEMON_VERSION})

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(Doxygen)
FIND_PACKAGE(Ghostscript)
#FIND_PACKAGE(GLPK 4.33)
#FIND_PACKAGE(CPLEX)
#FIND_PACKAGE(COIN)

IF(DEFINED ENV{LEMON_CXX_WARNING})
  SET(CXX_WARNING $ENV{LEMON_CXX_WARNING})
ELSE()
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CXX_WARNING "-Wall -W -Wunused -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wno-char-subscripts -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -fno-strict-aliasing -Wold-style-cast -Wno-unknown-pragmas")
    SET(CMAKE_CXX_FLAGS_DEBUG CACHE STRING "-ggdb")
    SET(CMAKE_C_FLAGS_DEBUG CACHE STRING "-ggdb")
  ELSEIF(MSVC)
    # This part is unnecessary 'casue the same is set by the lemon/core.h.
    # Still keep it as an example.
    SET(CXX_WARNING "/wd4250 /wd4355 /wd4503 /wd4800 /wd4996")
    # Suppressed warnings:
    # C4250: 'class1' : inherits 'class2::member' via dominance
    # C4355: 'this' : used in base member initializer list
    # C4503: 'function' : decorated name length exceeded, name was truncated
    # C4800: 'type' : forcing value to bool 'true' or 'false'
    #        (performance warning)
    # C4996: 'function': was declared deprecated
  ELSE()
    SET(CXX_WARNING "-Wall -W")
  ENDIF()
ENDIF()
SET(LEMON_CXX_WARNING_FLAGS ${CXX_WARNING} CACHE STRING "LEMON warning flags.")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LEMON_CXX_WARNING_FLAGS}")

SET( CMAKE_CXX_FLAGS_MAINTAINER "-Werror -ggdb" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    FORCE )
SET( CMAKE_C_FLAGS_MAINTAINER "-Werror" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_MAINTAINER
    CMAKE_C_FLAGS_MAINTAINER
    CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    CMAKE_SHARED_LINKER_FLAGS_MAINTAINER )

IF(CMAKE_CONFIGURATION_TYPES)
  LIST(APPEND CMAKE_CONFIGURATION_TYPES Maintainer)
  LIST(REMOVE_DUPLICATES CMAKE_CONFIGURATION_TYPES)
  SET(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING
      "Add the configurations that we need"
      FORCE)
 endif()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

SET( CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE )


INCLUDE(CheckTypeSize)
CHECK_TYPE_SIZE("long long" LONG_LONG)
SET(LEMON_HAVE_LONG_LONG ${HAVE_LONG_LONG})

INCLUDE(FindPythonInterp)


# -------------------------------------------------------------------------------------------------
# modified version for mini_lemon library

ADD_SUBDIRECTORY(lemon)

# here starts the original version again
#IF(${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
#ENDIF()

CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/cmake/LEMONConfig.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
  @ONLY
)
IF(UNIX)
  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
    DESTINATION share/lemon/cmake
  )
ELSEIF(WIN32)
  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/cmake/LEMONConfig.cmake
    DESTINATION cmake
  )
ENDIF()
