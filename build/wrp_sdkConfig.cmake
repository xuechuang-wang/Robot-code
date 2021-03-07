include(CMakeFindDependencyMacro)

# Capturing values from configure (optional)
# set(my-config-var )

# Same syntax as find_package
find_dependency(Threads REQUIRED)

# Any extra setup

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/wrp_utilsTargets.cmake")
