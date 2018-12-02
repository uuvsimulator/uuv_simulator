cmake_minimum_required(VERSION 2.8.3)
project(uuv_auv_control_allocator)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation)

catkin_python_setup()

add_message_files(
    FILES
    AUVCommand.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        std_msgs
)

catkin_install_python(PROGRAMS scripts/control_allocator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
