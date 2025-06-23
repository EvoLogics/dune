# ---------------------
# FIND WIC & PLEORA SDK
# ---------------------

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(WIC)
find_package(Pleora)

# --------------
# FIND GSTREAMER
# --------------

find_package(PkgConfig)
# SET(ENV{PKG_CONFIG_LIBDIR} /work/build-tegra194-evo/tmp/work/armv8a_tegra194-evologics-linux/dune-sonobot-tegra-wic/git-evo-r0/recipe-sysroot/usr/lib/pkgconfig/)
pkg_check_modules(gstreamer gstreamer-1.0)
pkg_check_modules(gstreamer_app gstreamer-app-1.0)

# -----------------------------------------
# ACTIVATE TASK IF ALL CONDITIONS SATISFIED
# -----------------------------------------

if(NOT DUNE_OS_LINUX OR NOT WIC_FOUND OR NOT PLEORA_FOUND OR NOT gstreamer_FOUND OR NOT gstreamer_app_FOUND)
  set(TASK_ENABLED FALSE)
else(NOT DUNE_OS_LINUX OR NOT WIC_FOUND OR NOT PLEORA_FOUND OR NOT gstreamer_FOUND OR NOT gstreamer_app_FOUND)
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
  # We have to add the following definitions
  # (see WIC SDK 1.1.0 documentation: https://software.workswell.eu/wic_sdk/ARM/doc/)
  add_definitions(-D_UNIX_ -D_LINUX_)
endif(NOT DUNE_OS_LINUX OR NOT WIC_FOUND OR NOT PLEORA_FOUND OR NOT gstreamer_FOUND OR NOT gstreamer_app_FOUND)
