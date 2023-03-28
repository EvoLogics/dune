# FindPleora.cmake
#
# Attempts to find Pleora EBus SDK library
#
# Following imported targets will be defined
#
# Pleora::Pleora
#
# Author: Filip Cacky
# Email: filip.cacky@workswell.cz

if(${CMAKE_CL_64})
        set(SUFFIX "64")
else()
        set(SUFFIX "")
endif()

unset(Pleora_FOUND)
unset(Pleora_LIBRARIES)

list(APPEND Pleora_CHECK_INCLUDE_DIRS
        "/opt/pleora/ebus_sdk/Ubuntu-x86_64/include"
        "/opt/pleora/ebus_sdk/Ubuntu-14.04-x86_64/include"
        "/opt/pleora/ebus_sdk/linux-aarch64-arm/include"
        "$ENV{ProgramFiles}/Pleora Technologies Inc/eBUS SDK/Includes"
        )

list(APPEND Pleora_CHECK_LIBRARY_DIRS
        "/opt/pleora/ebus_sdk/Ubuntu-x86_64/lib"
        "/opt/pleora/ebus_sdk/Ubuntu-14.04-x86_64/lib"
        "/opt/pleora/ebus_sdk/linux-aarch64-arm/lib"
        "$ENV{ProgramFiles}/Pleora Technologies Inc/eBUS SDK/Libraries"
        )

find_path(
        Pleora_INCLUDE_DIR PvBase.h
        PATHS ${Pleora_CHECK_INCLUDE_DIRS}
        )

# find_library(
#         EbTransportLayerLib_LIBRARY
#         NAMES "EbTransportLayerLib${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )
# find_library(
#         EbUtilsLib_LIBRARY
#         NAMES "EbUtilsLib${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

# find_library(
#         PtConvertersLib_LIBRARY
#         NAMES "PtConvertersLib${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

# find_library(
#         PtUtilsLib_LIBRARY
#         NAMES "PtUtilsLib${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

find_library(
        PvAppUtils_LIBRARY
        NAMES "PvAppUtils${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvBase_LIBRARY
        NAMES "PvBase${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvBuffer_LIBRARY
        NAMES "PvBuffer${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

# find_library(
#         PvCameraBridge_LIBRARY
#         NAMES "PvCameraBridge${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

find_library(
        PvDevice_LIBRARY
        NAMES "PvDevice${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvGenICam_LIBRARY
        NAMES "PvGenICam${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvPersistence_LIBRARY
        NAMES "PvPersistence${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvSerial_LIBRARY
        NAMES "PvSerial${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvStream_LIBRARY
        NAMES "PvStream${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

# find_library(
#         PvSystem_LIBRARY
#         NAMES "PvSystem${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

find_library(
        PvTransmitter_LIBRARY
        NAMES "PvTransmitter${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

find_library(
        PvVirtualDevice_LIBRARY
        NAMES "PvVirtualDevice${SUFFIX}"
        PATHS ${Pleora_CHECK_LIBRARY_DIRS}
        )

# find_library(
#         SimpleImagingLib_LIBRARY
#         NAMES "SimpleImagingLib${SUFFIX}"
#         PATHS ${Pleora_CHECK_LIBRARY_DIRS}
#         )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pleora REQUIRED_VARS
        Pleora_INCLUDE_DIR

        # EbTransportLayerLib_LIBRARY
        # EbUtilsLib_LIBRARY
        # PtConvertersLib_LIBRARY
        # PtUtilsLib_LIBRARY
        PvAppUtils_LIBRARY
        PvBase_LIBRARY
        PvBuffer_LIBRARY
        # PvCameraBridge_LIBRARY
        PvDevice_LIBRARY
        PvGenICam_LIBRARY
        PvPersistence_LIBRARY
        PvSerial_LIBRARY
        PvStream_LIBRARY
        # PvSystem_LIBRARY
        PvTransmitter_LIBRARY
        PvVirtualDevice_LIBRARY
        # SimpleImagingLib_LIBRARY
)

list(APPEND Pleora_LIBRARIES
	# ${EbTransportLayerLib_LIBRARY}
	# ${EbUtilsLib_LIBRARY}
	# ${PtConvertersLib_LIBRARY}
	# ${PtUtilsLib_LIBRARY}
	${PvAppUtils_LIBRARY}
	${PvBase_LIBRARY}
	${PvBuffer_LIBRARY}
	# ${PvCameraBridge_LIBRARY}
	${PvDevice_LIBRARY}
	${PvGenICam_LIBRARY}
	${PvPersistence_LIBRARY}
	${PvSerial_LIBRARY}
	${PvStream_LIBRARY}
	# ${PvSystem_LIBRARY}
	${PvTransmitter_LIBRARY}
	${PvVirtualDevice_LIBRARY}
	# ${SimpleImagingLib_LIBRARY}
	)

# unset(EbTransportLayerLib_LIBRARY CACHE)
# unset(EbUtilsLib_LIBRARY CACHE)
# unset(PtConvertersLib_LIBRARY CACHE)
# unset(PtUtilsLib_LIBRARY CACHE)
unset(PvAppUtils_LIBRARY CACHE)
unset(PvBase_LIBRARY CACHE)
unset(PvBuffer_LIBRARY CACHE)
# unset(PvCameraBridge_LIBRARY CACHE)
unset(PvDevice_LIBRARY CACHE)
unset(PvGenICam_LIBRARY CACHE)
unset(PvPersistence_LIBRARY CACHE)
unset(PvSerial_LIBRARY CACHE)
unset(PvStream_LIBRARY CACHE)
# unset(PvSystem_LIBRARY CACHE)
unset(PvTransmitter_LIBRARY CACHE)
unset(PvVirtualDevice_LIBRARY CACHE)
# unset(SimpleImagingLib_LIBRARY CACHE)
