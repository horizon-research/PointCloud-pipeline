cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_DIR "/usr/local/share/pcl-1.8")
#find_package(PCL 1.9 REQUIRED HINTS "~/local/")
find_package(PCL 1.7 REQUIRED PATHS "./" NO_DEFAULT_PATH)
# find_package(PCL 1.7 REQUIRED PATHS "./")
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -DNDEBUG -fopenmp")

message("\nPCL include: \n${PCL_INCLUDE_DIRS}\n")
include_directories("./pcl-1.7;/usr/include/eigen3;~/local/flann/include;" include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -O3 -DNDEBUG -DPCL_NO_PRECOMPILE)
add_executable (pc_pipeline src/modules.cpp src/utils.cpp src/read_config.cpp src/pc_pipeline.cpp)
target_link_libraries (pc_pipeline ${PCL_LIBRARIES})

set_property(TARGET pc_pipeline PROPERTY CXX_STANDARD 11)

SET(EXECUTABLE_OUTPUT_PATH .././bin) 