cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_DIR "$PATH_TO_PCLCONFIG")
# $PATH_TO_PCLCONFIG: path to PCLConfig.cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -DNDEBUG")

include_directories("./pcl-1.7;$PATH_TO_EIGEN" include)
# $PATH_TO_EIGEN: path to EIGEN

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -O3 -DNDEBUG -DPCL_NO_PRECOMPILE)
add_executable (pc_pipeline src/modules.cpp src/utils.cpp src/read_config.cpp src/pc_pipeline.cpp)
target_link_libraries (pc_pipeline ${PCL_LIBRARIES})

set_property(TARGET pc_pipeline PROPERTY CXX_STANDARD 11)

SET(EXECUTABLE_OUTPUT_PATH .././bin) 
