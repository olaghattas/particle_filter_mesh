cmake_minimum_required(VERSION 3.5)
project(detection_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/PoseMsg.msg"
        DEPENDENCIES
        std_msgs
        builtin_interfaces
        )
ament_export_dependencies(rosidl_default_runtime)
ament_package()
