unset(WIC_FOUND CACHE)
unset(WIC_INCLUDE_DIR CACHE)
unset(WIC_LIBRARIES CACHE)

list(APPEND WIC_CHECK_INCLUDE_DIRS
    "/opt/workswell/wic_sdk/include"
    )

list(APPEND WIC_CHECK_LIBRARY_DIRS
    "/opt/workswell/wic_sdk/lib"
    )

find_path(
    WIC_INCLUDE_DIR Camera.h
    PATHS ${WIC_CHECK_INCLUDE_DIRS}
    NO_CACHE
    )

find_library(
    WIC_LIBRARY
    "WIC_SDK"
    PATHS ${WIC_CHECK_LIBRARY_DIRS}
    NO_CACHE
    )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    WIC
    REQUIRED_VARS
    WIC_INCLUDE_DIR
    WIC_LIBRARY
    )

include(CMakeFindDependencyMacro)
find_dependency(Pleora REQUIRED)
