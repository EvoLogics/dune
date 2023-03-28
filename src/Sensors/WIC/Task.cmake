# ------------------------------------------
# CHECK C++ COMPILER SUPPORTS C++11 STANDARD
# ------------------------------------------

include(CheckCXXCompilerFlag)
unset(COMPILER_SUPPORTS_SENSORS_WIC_CXX_VERSION CACHE)
check_cxx_compiler_flag(-std=c++11 COMPILER_SUPPORTS_SENSORS_WIC_CXX_VERSION)

# ---------------------
# FIND WIC & PLEORA SDK
# ---------------------

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(WIC REQUIRED)
find_package(Pleora REQUIRED)

# --------------
# FIND GSTREAMER
# --------------

find_package(PkgConfig REQUIRED)
pkg_check_modules(gstreamer REQUIRED gstreamer-1.0)
pkg_check_modules(gstreamer_app REQUIRED gstreamer-app-1.0)

# -----------------------------------------
# ACTIVATE TASK IF ALL CONDITIONS SATISFIED
# -----------------------------------------

if(NOT DUNE_OS_LINUX OR NOT COMPILER_SUPPORTS_SENSORS_WIC_CXX_VERSION OR NOT WIC_FOUND)
  set(TASK_ENABLED FALSE)
else(NOT DUNE_OS_LINUX OR NOT COMPILER_SUPPORTS_SENSORS_WIC_CXX_VERSION OR NOT WIC_FOUND)
  set(
    DUNE_SYS_LIBS
    ${DUNE_SYS_LIBS}
    ${WIC_LIBRARY}
    ${Pleora_LIBRARIES}
    ${gstreamer_LIBRARIES}
    ${gstreamer_app_LIBRARIES}
  )
  include_directories(
    ${WIC_INCLUDE_DIR}
    ${Pleora_INCLUDE_DIR}
    ${gstreamer_INCLUDE_DIRS}
    ${gstreamer_app_INCLUDE_DIRS}
  )
  add_definitions(-D_UNIX_ -D_LINUX_) # (see WIC SDK 1.1.0 documentation)
endif(NOT DUNE_OS_LINUX OR NOT COMPILER_SUPPORTS_SENSORS_WIC_CXX_VERSION OR NOT WIC_FOUND)
