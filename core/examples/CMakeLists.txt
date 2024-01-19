#
# (C) Copyright 2009-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawSartoriusScaleExamples VERSION 1.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes
                              cisstParameterTypesQt
                              cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.1 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # sawSartoriusScale has been compiled within cisst, we should find it automatically
  find_package (sawSartoriusScale REQUIRED)

  if (sawSartoriusScale_FOUND)

    # sawSartoriusScale configuration
    include_directories (${sawSartoriusScale_INCLUDE_DIR})
    link_directories (${sawSartoriusScale_LIBRARY_DIR})

    add_executable (sawSartoriusScaleQtExample mainQt.cpp)
    set_target_properties (sawSartoriusScaleQtExample PROPERTIES
                           COMPONENT sawSartoriusScale-Examples
                           FOLDER "sawSartoriusScale")

    # link against non cisst libraries and saw components
    target_link_libraries (sawSartoriusScaleQtExample
                           ${sawSartoriusScale_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawSartoriusScaleQtExample ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS sawSartoriusScaleQtExample
             COMPONENT sawSartoriusScale-Examples
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  endif (sawSartoriusScale_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
