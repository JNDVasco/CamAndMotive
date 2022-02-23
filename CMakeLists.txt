cmake_minimum_required(VERSION 3.20)
project(CamAndMotive)

set(CMAKE_CXX_STANDARD 14)

# =================== #
# Directories RS SDK  #
# =================== #
include_directories("C:/Program Files (x86)/Intel RealSense SDK 2.0/include" "C:/Program Files (x86)/Intel RealSense SDK 2.0/third-party")
link_directories("C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64" "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64")


# =================== #
# Directories NatNet  #
# =================== #
include_directories("NatNetSDK/include")
link_directories("./NatNetSDK/Samples/bin/x64" "./NatNetSDK/lib/x64/")


# =================== #
# Directories ZED SDK #
# =================== #
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)

IF (NOT WIN32)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF ()

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

if (LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else ()
    set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif ()

# =================== #
#     Executables     #
# =================== #
add_executable(Main main.cpp)
target_link_libraries(Main NatNetLib.lib realsense2.lib ${ZED_LIBS} ${OpenCV_LIBRARIES})

if (INSTALL_SAMPLES)
    list(APPEND SAMPLE_LIST Main)
    set(SAMPLE_LIST "${SAMPLE_LIST}" PARENT_SCOPE)
endif ()

add_executable(SampleClient NatNetSDK/Samples/SampleClient/SampleClient.cpp)
target_link_libraries(SampleClient NatNetLib.lib)


add_executable(motiveExport motiveExport.cpp)
target_link_libraries(motiveExport NatNetLib.lib)

