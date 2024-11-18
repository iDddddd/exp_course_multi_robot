cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        cv_bridge
        image_transport
        roscpp
        std_msgs tf
        camera_calibration_parsers
        message_generation
        geometry_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
include_directories(/usr/local/include)

add_message_files(FILES
        aruco_info.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs tf sensor_msgs geometry_msgs message_runtime
)

add_executable(aruco_detection src/aruco_detection.cpp src/main.cpp include/aruco_detection.h)
target_link_libraries(aruco_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(create_marker src/create_marker.cpp)
#target_link_libraries(create_marker ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(decision src/decision.cpp include/cvui.h)
#target_link_libraries(decision ${catkin_LIBRARIES} ${OpenCV_LIBS})

