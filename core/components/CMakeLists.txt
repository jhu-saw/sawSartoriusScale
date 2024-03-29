#
# (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawSartoriusScale VERSION 1.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawSartoriusScale)
  set (sawSartoriusScale_INCLUDE_DIR
       "${sawSartoriusScale_SOURCE_DIR}/include"
       "${sawSartoriusScale_BINARY_DIR}/include")
  set (sawSartoriusScale_HEADER_DIR "${sawSartoriusScale_SOURCE_DIR}/include/sawSartoriusScale")
  set (sawSartoriusScale_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawSartoriusScale_LIBRARIES sawSartoriusScale)

  # Allow c++ code to find local header files
  include_directories (${sawSartoriusScale_INCLUDE_DIR} ${sawSartoriusScale_BINARY_DIR})

  # add all config files for this component
  cisst_add_config_files (sawSartoriusScale)

  add_library (sawSartoriusScale
               ${sawSartoriusScale_HEADER_DIR}/sawSartoriusScaleExport.h
               code/mtsSartoriusSerial.cpp
               ${sawSartoriusScale_HEADER_DIR}/mtsSartoriusSerial.h)
  set_target_properties (sawSartoriusScale PROPERTIES
                         VERSION ${sawSartoriusScale_VERSION}
                         FOLDER "sawSartoriusScale")
  cisst_target_link_libraries (sawSartoriusScale ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY
           "${sawSartoriusScale_SOURCE_DIR}/include/sawSartoriusScale"
           "${sawSartoriusScale_BINARY_DIR}/include/sawSartoriusScale"
           DESTINATION include
           COMPONENT sawSartoriusScale-dev)

  install (TARGETS sawSartoriusScale COMPONENT sawSartoriusScale
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
