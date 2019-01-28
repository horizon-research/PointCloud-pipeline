cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -O3 -DNDEBUG -DPCL_NO_PRECOMPILE)

add_executable (pc-registration src/utils.cpp src/pc-registration.cpp)
target_link_libraries (pc-registration ${PCL_LIBRARIES})

set_property(TARGET pc-registration PROPERTY CXX_STANDARD 11)
