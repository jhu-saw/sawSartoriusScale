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

cmake_minimum_required(VERSION 3.10)
project (sawSartoriusScaleCore VERSION 1.1.0)

find_package (cisst REQUIRED)
include (${CISST_USE_FILE})
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)

# Examples
set (sawSartoriusScale_DIR "${sawSartoriusScaleCore_BINARY_DIR}/components")
add_subdirectory (examples)

include (CPack)
cpack_add_component (sawSartoriusScale)
cpack_add_component (sawSartoriusScale-dev
                     DEPENDS sawSartoriusScale)
cpack_add_component (sawSartoriusScale-Examples
                     DEPENDS sawSartoriusScale)
