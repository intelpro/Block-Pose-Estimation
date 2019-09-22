cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(matching_algorithm)

find_package(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    OpenCV
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files( FILES Data1d.msg Data2d.msg )
generate_messages( DEPENDENCIES std_msgs )
catkin_package(CATKIN_DEPENDS messge_runtime)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (matching_algorithm matching_algorithm.h matching_algorithm.cpp)
add_dependencies(matching_algorithm ${matching_algorithm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries (matching_algorithm ${PCL_LIBRARIES})
target_link_libraries (matching_algorithm ${OpenCV_LIBS})
target_link_libraries(matching_algorithm ${catkin_LIBRARIES})
