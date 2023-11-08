cmake_minimum_required(VERSION 3.10)
project(cam_driver LANGUAGES CXX CUDA)
enable_language(CUDA)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_GENERATOR "Makefile")
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Ofast -march=native -flto -funsafe-loop-optimizations")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb -Og")

###########################################
# enable debug mode by below definition   #
###########################################
# Enanle Below Flag For Using OpenCV Visual Image
# add_compile_definitions(DEBUG)

# Enanle Below Flag When The JetPack Version Support NvBufSurface DMA Transform Api 
# add_compile_definitions(JETPACK_SUPPORT)

# Enanle Below Flag When Your Want Using Ros1 Launch For Start Cameara
add_compile_definitions(ROS_NODE)

# Enanle Below Flag For Publish BGR image Using Iceoryx
add_compile_definitions(PUB_WITH_ICEORYX)

# Modify Below Path For Path
set(NV_COMMON_LIB_PATH "/usr/lib/aarch64-linux-gnu/tegra")
set(NPP_LIB_PATH "/usr/local/cuda-11.4/lib64")
set(NPP_INCLUDE_PATH "/usr/local/cuda-11.4/include")

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/externel/jetson_multimedia_api/lib)
link_directories(${NV_COMMON_LIB_PATH})
link_directories(${NPP_LIB_PATH})

find_package( OpenCV REQUIRED )
find_package( iceoryx_posh REQUIRED )
find_package( iceoryx_hoofs REQUIRED )
find_package( iceoryx_platform REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Boost REQUIRED COMPONENTS program_options REQUIRED)

find_package(CUDA REQUIRED)
message(STATUS "cuda_compiler: " ${CMAKE_CUDA_COMPILER})

find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    cv_bridge
    roscpp
    sensor_msgs
    std_msgs)

cuda_add_library(cuda_comp SHARED 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/yuv2rgb.cu
)

target_compile_options(cuda_comp PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    -Xcompiler 
    -fPIC
    -dc
    -dlink
    >
)

target_link_libraries(cuda_comp
    cudart
)


add_executable(cam_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/v4l2cam.cc
)

set_target_properties(cam_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(cam_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/externel
    ${CMAKE_CURRENT_SOURCE_DIR}/externel/jetson_multimedia_api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRECTORIES}
)


target_link_libraries(cam_node PRIVATE
    ${catkin_LIBRARIES}
    # ${Boost_LINK_LIBRARIES}
    ${OpenCV_LIBS}
    ${NPPC_LIBRARY}
    Boost::program_options
    nv_api_common
    nvbufsurface
    nvbufsurftransform
    cudart
    cuda_comp
    -lpthread
    -lrt
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
)


# Add Test Reciver Folder
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
