cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.2 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registration src/utils.cpp src/registration.cpp)
target_link_libraries (registration ${PCL_LIBRARIES})

set_property(TARGET registration PROPERTY CXX_STANDARD 11)
