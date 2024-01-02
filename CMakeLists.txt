cmake_minimum_required(VERSION 3.8 )
project(particle_filter_mesh)
#enable_language(CUDA)
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(particle_filter_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(detection_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(shr_utils REQUIRED)


set(dependencies1
        rclcpp
        visualization_msgs
        sensor_msgs
        ament_index_cpp
        geometry_msgs
        cv_bridge
        detection_msgs
        tf2_ros
        particle_filter_msgs
        shr_utils
        zed_interfaces
        std_msgs
        )

# Add executable
add_library(particle_filter src/particle_filter.cpp)
ament_target_dependencies(particle_filter ${dependencies1})

target_link_libraries(particle_filter ${OpenCV_LIBRARIES})
target_include_directories(particle_filter PUBLIC ${EIGEN3_INCLUDE_DIRS})


add_executable(particle_filter_node src/particle_filter_node.cpp)
ament_target_dependencies(particle_filter_node ${dependencies1})
target_include_directories(particle_filter_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(particle_filter_node PUBLIC ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(particle_filter_node yaml-cpp)
target_link_libraries(particle_filter_node ${OpenCV_LIBRARIES})

#add_executable(yaml_node src/example_yaml.cpp)
#target_include_directories(yaml_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
#target_include_directories(yaml_node PUBLIC ${YAML_CPP_INCLUDE_DIRS})

## Link the libraries
#target_link_libraries(yaml_node
#        ${YAML_CPP_LIBRARIES}
#        ${EIGEN3_LIBRARIES}
#        )

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

#target_include_directories(particle_filter_node PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
target_include_directories(particle_filter_node PUBLIC
        ${YAML_CPP_LIBRARIES}
        ${OpenCV_INCLUDE_DIRS}
        )

target_include_directories(particle_filter PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        )

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
        particle_filter
        particle_filter_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()

