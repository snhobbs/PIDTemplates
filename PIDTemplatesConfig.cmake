# It defines the following variables
#  PIDTEMPLATES_INCLUDE_DIRS - include directories
#  PIDTEMPLATES_LIBRARIES    - libraries to link against
#  PIDTEMPLATES_EXECUTABLE   - the bar executable

# Compute paths
get_filename_component("PIDTEMPLATES_CMAKE_DIR" "${CMAKE_CURRENT_LIST_FILE}" PATH)
set("PIDTEMPLATES_INCLUDE_DIRS" "${PIDTEMPLATES_CMAKE_DIR}/include")
