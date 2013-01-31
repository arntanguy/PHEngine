# FindCGENGINE - attempts to locate the CGEngine matrix/vector library.
#
# This module defines the following variables (on success):
#   CGENGINE_INCLUDE_DIRS  - where to find CGEngine/CGEngine.hpp
#   CGENGINE_FOUND         - if the library was successfully located
#
# It is trying a few standard installation locations, but can be customized
# with the following variables:
#   CGENGINE_ROOT_DIR      - root directory of a CGEngine installation
#                       Headers are expected to be found in either:
#                       <CGENGINE_ROOT_DIR>/CGEngine/CGEngine.hpp           OR
#                       <CGENGINE_ROOT_DIR>/include/CGEngine/CGEngine.hpp
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
SET(_CGEngine_HEADER_SEARCH_DIRS
    "/usr/include"
    "/usr/local/include"
    "/home/arnaud/TCD/CGEngine/src")
SET(_CGEngine_LIBS_SEARCH_DIRS
    "/usr/lib"
    "/usr/local/lib"
    "/home/arnaud/TCD/CGEngine")
set(_CGEngine_LIBRARY_NAMES CGEngine)

# check environment variable
SET(_CGEngine_ENV_ROOT_DIR "$ENV{CGENGINE_ROOT_DIR}")

IF(NOT CGENGINE_ROOT_DIR AND _CGEngine_ENV_ROOT_DIR)
    SET(CGENGINE_ROOT_DIR "${_CGEngine_ENV_ROOT_DIR}")
ENDIF(NOT CGENGINE_ROOT_DIR AND _CGEngine_ENV_ROOT_DIR)

# put user specified location at beginning of search
IF(CGENGINE_ROOT_DIR)
    SET(_CGEngine_HEADER_SEARCH_DIRS "${CGENGINE_ROOT_DIR}"
                                "${CGENGINE_ROOT_DIR}/include"
                                 ${_CGEngine_HEADER_SEARCH_DIRS})
ENDIF(CGENGINE_ROOT_DIR)

# locate header
FIND_PATH(CGENGINE_INCLUDE_DIR "CGEngine.h"
    PATHS ${_CGEngine_HEADER_SEARCH_DIRS})

find_library(CGEngine_LIBRARY NAMES ${_CGEngine_LIBRARY_NAMES} HINTS ${_CGEngine_LIBS_SEARCH_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
make_library_set(CGENINE_LIBRARY)


INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CGENGINE DEFAULT_MSG
    CGENGINE_INCLUDE_DIR)

IF(CGENGINE_FOUND)
    SET(CGENGINE_INCLUDE_DIRS "${CGENGINE_INCLUDE_DIR}")
	SET(CGENGINE_LIBRARIES "${CGEngine_LIBRARY}")

    MESSAGE(STATUS "CGENGINE_INCLUDE_DIR = ${CGENGINE_INCLUDE_DIR}")
    MESSAGE(STATUS "CGENGINE_LIBS = ${CGENGINE_LIBRARIES}")
ENDIF(CGENGINE_FOUND)
