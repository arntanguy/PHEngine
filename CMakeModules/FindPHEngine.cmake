# FindPHENGINE - attempts to locate the PHEngine matrix/vector library.
#
# This module defines the following variables (on success):
#   PHENGINE_INCLUDE_DIRS  - where to find PHEngine/PHEngine.hpp
#   PHENGINE_FOUND         - if the library was successfully located
#
# It is trying a few standard installation locations, but can be customized
# with the following variables:
#   PHENGINE_ROOT_DIR      - root directory of a PHEngine installation
#                       Headers are expected to be found in either:
#                       <PHENGINE_ROOT_DIR>/PHEngine/PHEngine.hpp           OR
#                       <PHENGINE_ROOT_DIR>/include/PHEngine/PHEngine.hpp
#                       This variable can either be a cmake or environment
#                       variable. Note however that changing the value
#                       of the environment varible will NOT result in
#                       re-running the header search and therefore NOT
#                       adjust the variables set by this module.

#=============================================================================
# Copyright 2013 Arnaud TANGUY
#
# Distributed under the GNU GPL v3 (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

# default search dirs
SET(_PHEngine_HEADER_SEARCH_DIRS
    "/usr/include"
    "/usr/local/include")
SET(_PHEngine_LIBS_SEARCH_DIRS
    "/usr/lib"
    "/usr/local/lib")
  set(_PHEngine_LIBRARY_NAMES PHEngine)

# check environment variable
SET(_PHEngine_ENV_ROOT_DIR "$ENV{PHENGINE_ROOT_DIR}")

IF(NOT PHENGINE_ROOT_DIR AND _PHEngine_ENV_ROOT_DIR)
    SET(PHENGINE_ROOT_DIR "${_PHEngine_ENV_ROOT_DIR}")
ENDIF(NOT PHENGINE_ROOT_DIR AND _PHEngine_ENV_ROOT_DIR)

# put user specified location at beginning of search
IF(PHENGINE_ROOT_DIR)
    SET(_PHEngine_HEADER_SEARCH_DIRS "${PHENGINE_ROOT_DIR}"
                                "${PHENGINE_ROOT_DIR}/include"
                                 ${_PHEngine_HEADER_SEARCH_DIRS})
ENDIF(PHENGINE_ROOT_DIR)

# locate header
FIND_PATH(PHENGINE_INCLUDE_DIR "PHEngine/PHEngine.h"
    PATHS ${_PHEngine_HEADER_SEARCH_DIRS})

find_library(PHEngine_LIBRARY NAMES ${_PHEngine_LIBRARY_NAMES} HINTS ${_PHEngine_LIBS_SEARCH_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
make_library_set(CGENINE_LIBRARY)


INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PHENGINE DEFAULT_MSG
    PHENGINE_INCLUDE_DIR)

IF(PHENGINE_FOUND)
    SET(PHENGINE_INCLUDE_DIRS "${PHENGINE_INCLUDE_DIR}")
	SET(PHENGINE_LIBRARIES "${PHEngine_LIBRARY}")

    MESSAGE(STATUS "PHENGINE_INCLUDE_DIR = ${PHENGINE_INCLUDE_DIR}")
    MESSAGE(STATUS "PHENGINE_LIBS = ${PHENGINE_LIBRARIES}")
ENDIF(PHENGINE_FOUND)
