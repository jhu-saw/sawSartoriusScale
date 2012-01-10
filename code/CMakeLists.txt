#
# $Id$
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# Core library
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  add_library (sawSartoriusScale
               ../sawSartoriusScaleExport.h
               mtsSartoriusSerial.cpp
               ../mtsSartoriusSerial.h)
  set_property (TARGET sawSartoriusScale PROPERTY FOLDER "sawSartoriusScale")
  cisst_target_link_libraries (sawSartoriusScale ${REQUIRED_CISST_LIBRARIES})

  # create/configure file for find_package (sawSartoriusScale)
  set (sawSartoriusScale_INCLUDE_DIR "${sawComponents_SOURCE_DIR}")
  set (sawSartoriusScale_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawSartoriusScale_LIBRARIES sawSartoriusScale)

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawSartoriusScaleConfig.cmake.in"
                  "${sawComponents_BINARY_DIR}/sawSartoriusScaleConfig.cmake"
                   @ONLY@)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
