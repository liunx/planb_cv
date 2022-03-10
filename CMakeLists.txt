cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

PROJECT(planb_cv)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(third_parts/BehaviorTree.CPP)
include_directories(third_parts/BehaviorTree.CPP/include)
set(DEPS behaviortree_cpp_v3)
set(SRCS src/main.cpp src/robot_control.cpp src/behavior_tree_nodes.cpp
    src/action_nodes.cpp)

ADD_EXECUTABLE(main ${SRCS})
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS} ${DEPS})
